#![no_std]
#![no_main]

use crate::bl_info::{BootloaderInfo, MemoryRegion};

use atsamd_hal::{
    can::Dependencies,
    clock::v2::{
        clock_system_at_reset,
        dfll::FromUsb,
        dpll::Dpll,
        gclk::{Gclk, GclkDiv16},
        pclk::Pclk,
    },
    clock::v2::{osculp32k::OscUlp32k, pclk, rtcosc::RtcOsc},
    ehal::digital::{InputPin, OutputPin},
    fugit::ExtU64,
    fugit::HertzU32,
    gpio::{Alternate, F, PD08},
    nvm::Nvm,
    pac::Peripherals,
    prelude::_embedded_hal_Pwm,
    prelude::_embedded_hal_watchdog_WatchdogEnable,
    pwm::{Channel, TCC0Pinout, Tcc0Pwm},
    rtc::rtic::rtc_clock,
    rtic_time::Monotonic,
    serial_number,
    trng::Trng,
    usb::{
        usb_device::{
            bus::UsbBusAllocator,
            device::{StringDescriptors, UsbDeviceBuilder, UsbRev, UsbVidPid},
        },
        UsbBus,
    },
    watchdog::*,
};

use core::sync::atomic::Ordering;

use bsp::can_deps::{Capacities, RxFifo0};
use futures::FutureExt;
use mcan::{
    embedded_can::Id,
    interrupt::{state::EnabledLine0, Interrupt, OwnedInterruptSet},
    message::Raw,
    tx_buffers::{DynTx, TxBufferSet},
};
use rtic_sync::{arbiter::Arbiter, signal::Signal};

use core::{panic::PanicInfo, sync::atomic::AtomicU8};
use defmt::info;
use heapless::format;
use mcan::{embedded_can::StandardId, filter::Filter, messageram::SharedMemory};

use defmt_rtt as _;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use crate::kwp::KwpServer;

use diag_common::ram_info::*;
mod bl_info;
pub mod kwp;
pub mod usb;

pub static ST_MIN_EGS: AtomicU8 = AtomicU8::new(0x02);
pub static BS_EGS: AtomicU8 = AtomicU8::new(0x08);

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    defmt::error!("{}", defmt::Display2Format(info));

    let bsp_peripherals = unsafe { Peripherals::steal() };
    let pins = bsp::Pins::new(bsp_peripherals.port);
    let _ = pins.led_eeprom.into_push_pull_output().set_high();
    let _ = pins.led_ext_flash.into_push_pull_output().set_high();
    let _ = pins.led_tle_act.into_push_pull_output().set_high();
    let _ = pins.led_usb.into_push_pull_output().set_high();
    loop {}
}

pub const CAN_ID_TX: StandardId = unsafe { StandardId::new_unchecked(0x7E9) };
pub const CAN_ID_RX: StandardId = unsafe { StandardId::new_unchecked(0x7E1) };

// memory.x
const RAM_START_ADDR: u32 = 0x20040000;

fn can_app_start(bl_info: &BootloaderInfo) -> bool {
    // Check app actually exists (Check for vector table)
    let stack_ptr = unsafe { (MemoryRegion::Application.start_addr() as *const u32).read() };
    let stack_addr_ok = stack_ptr == RAM_START_ADDR;
    #[cfg(not(feature = "skip-app-check"))]
    return bl_info.app_flashing_not_done == 0 && stack_addr_ok;
    #[cfg(feature = "skip-app-check")]
    return true && stack_addr_ok;
}

const ISOTP_BUF_SIZE: usize = 4096;

atsamd_hal::rtc_monotonic!(Mono, rtc_clock::Clock32k);

#[rtic::app(device = atsamd_hal::pac, dispatchers = [DAC_EMPTY_0])]
mod app {
    use atsamd_hal::clock::v2::types::Can0;
    use automotive_diag::kwp2000::KwpSessionType;
    use diag_common::{
        isotp_endpoints::{
            can_isotp::{make_isotp_endpoint, IsoTpInterruptHandler, IsotpConsumer, IsotpCtsMsg},
            usb_isotp::{new_usb_isotp, UsbIsoTpConsumer},
            SharedIsoTpBuf,
        },
        BootloaderStayReason,
    };
    use usbd_serial::DefaultBufferStore;

    use crate::usb::UsbData;

    use super::*;

    #[local]
    struct Resources {
        isotp_isr: IsoTpInterruptHandler<'static, Can0, Capacities, ISOTP_BUF_SIZE>,
        isotp_thread: IsotpConsumer<'static, Can0, Capacities, ISOTP_BUF_SIZE>,
        usb_isotp_thread: UsbIsoTpConsumer<
            'static,
            UsbBus,
            DefaultBufferStore,
            DefaultBufferStore,
            ISOTP_BUF_SIZE,
        >,

        diag_server: KwpServer,
        tcc_led: Tcc0Pwm<PD08, Alternate<F>>,
        can0_interrupts: OwnedInterruptSet<pclk::ids::Can0, EnabledLine0>,
        can0_fifo0: RxFifo0,
    }

    #[shared]
    struct Shared {
        #[lock_free]
        usb: UsbData<'static>,
        #[lock_free]
        old_bootloader_info: BootloaderRamInfo,
    }

    #[init(local = [
        #[link_section = ".can"]
        message_ram: SharedMemory<Capacities> = SharedMemory::new(),
        usb_ctrl_buf: [u8; 128] = [0; 128],
        usb_alloc: Option<UsbBusAllocator<UsbBus>> = None,
        usb_sn: heapless::String<32> = heapless::String::new(),
        isotp_can_fc_signal: Signal<IsotpCtsMsg> = Signal::new(),
        isotp_msg_signal_can: Signal<SharedIsoTpBuf<ISOTP_BUF_SIZE>> = Signal::new(),
        isotp_msg_signal_usb: Signal<SharedIsoTpBuf<ISOTP_BUF_SIZE>> = Signal::new(),
        arbiter_cantx: Option<Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>> = None
        arbiter_serial: Option<Arbiter<SerialPort<'static, UsbBus, DefaultBufferStore, DefaultBufferStore>>> = None
    ])]
    fn init(cx: init::Context) -> (Shared, Resources) {
        // Check for good app and try to bootload
        let mut device = cx.device;
        let mut core: rtic::export::Peripherals = cx.core;
        let pins = bsp::Pins::new(device.port);
        let mut start_diag_in_reprog_mode = false;
        let mut old_bootloader_info = BootloaderRamInfo::default();
        let mut reset_reason = BootloaderStayReason::None;
        #[cfg(not(feature = "stay-in-bootloader"))]
        {
            let rst_rsn = device.rstc.rcause().read();
            if pins.eeprom_sda.into_floating_input().is_low().unwrap() {
                defmt::warn!("Magic pin connected, staying in bootloader");
                reset_reason = BootloaderStayReason::MagicPin;
            } else if rst_rsn.wdt().bit_is_set() {
                defmt::warn!("Watchdog triggered, staying in bootloader!");
                reset_reason = BootloaderStayReason::Watchdog;
            } else {
                let mut continue_boot = true;
                if let Some(ram_info) = get_bootloader_comm_info() {
                    continue_boot = false;
                    // Sanity check first to ensure both app and bootloader
                    // have the same version of the data structure
                    if ram_info.diag_request_bootloader.0 {
                        // Asked to enter reprogramming mode
                        start_diag_in_reprog_mode = true;
                        if let Some(override_timing) = ram_info.diag_request_bootloader.1 {
                            ST_MIN_EGS.store(override_timing.0, Ordering::SeqCst);
                            BS_EGS.store(override_timing.1, Ordering::SeqCst);
                        }
                        defmt::warn!("Bootloader Diag reqest received");
                    } else if ram_info.reset_counter >= MAX_RESET_COUNT {
                        // Emergency mode (User reset 5 times quickly)
                        defmt::warn!("Bootloader Reset count exceeded!");
                        reset_reason = BootloaderStayReason::ResetCount;
                    } else if ram_info.app_panic.is_some() {
                        // Panic detected in application
                        defmt::warn!("Bootloader detected App crashed!");
                        reset_reason = BootloaderStayReason::Panic;
                    } else {
                        // No requirements by the app to stay in bootloader
                        continue_boot = true;
                    }
                    // Copy the current info
                    old_bootloader_info = ram_info;
                } else {
                    // Create default bootloader info if corrupt
                    create_default_comm_info();
                }

                let bl_info = bl_info::get_bootloader_info();
                if BootloaderStayReason::None == reset_reason {
                    if !can_app_start(bl_info) {
                        reset_reason = BootloaderStayReason::AppInvalid;
                    }
                }

                if can_app_start(bl_info) && continue_boot {
                    let app_addr = MemoryRegion::Application.range_exclusive().start;
                    #[cfg(feature = "skip-app-check")]
                    defmt::warn!("Skip app check enabled - Launching app without verifying");
                    // Start watchdog in bootloader, this way if the CPU freezes,
                    // then the watchdog shall reset, and the bootloader will know!
                    let mut wdt = Watchdog::new(device.wdt);
                    // (1 second / 1024) * period = timeout
                    // 2048 = 2 seconds
                    wdt.start(WatchdogTimeout::Cycles2K as u8);

                    modify_bootloader_info(|state| {
                        // Application will reset this
                        state.reset_counter += 1;
                    });

                    unsafe {
                        core.SCB.invalidate_icache();
                        core.SCB.vtor.write(app_addr);
                        cortex_m::asm::bootload(app_addr as *const u32);
                    }
                }
            }
        }

        // Bootloader init - Reset the bootloader state
        create_default_comm_info();

        // Did not jump to app, start the bootloader diagnostic system
        info!("Bootloader diag start");
        // No app, or we have to stay in bootloader, so now we setup clocks and start the TCU
        // in bootloader mode
        let (mut _buses, clocks, tokens) = clock_system_at_reset(
            device.oscctrl,
            device.osc32kctrl,
            device.gclk,
            device.mclk,
            &mut device.nvmctrl,
        );

        // Clock GCLK0 to 100Mhz
        let (gclk1, dfll) = Gclk::from_source(tokens.gclks.gclk1, clocks.dfll);
        let gclk1 = gclk1.div(GclkDiv16::Div(24)).enable(); // Gclk1 is now at 2Mhz
        let (clk_dpll0, _gclk1) = Pclk::enable(tokens.pclks.dpll0, gclk1);
        // DPLL0 at 100Mhz (2*50)
        let dpll0 = Dpll::from_pclk(tokens.dpll0, clk_dpll0)
            .loop_div(50, 0)
            .enable();
        let (_gclk0_100, dfll, _dpll0) = clocks.gclk0.swap_sources(dfll, dpll0);
        let (dfll_usb, _old_mode) = dfll.into_mode(FromUsb, |_dfll| {});
        let (gclk2, _dpll0) = Gclk::from_source(tokens.gclks.gclk2, dfll_usb);
        let gclk2_48 = gclk2.enable();
        let mut mclk = unsafe { clocks.pac.steal().3 };

        // Enable the 32Khz clock  and start the RTIC Monotonic driver
        let (osculp32k, _) = OscUlp32k::enable(tokens.osculp32k.osculp32k, clocks.osculp32k_base);
        let _ = RtcOsc::enable(tokens.rtcosc, osculp32k);
        // Start OS time queue
        Mono::start(device.rtc);

        // -- CAN init --
        let (clk_can, gclk2_48) = Pclk::enable(tokens.pclks.can0, gclk2_48);
        let (can0_deps, gclk2_48) = Dependencies::new(
            gclk2_48,
            clk_can,
            clocks.ahbs.can0,
            pins.can_rx.into_mode(),
            pins.can_tx.into_mode(),
            device.can0,
        );

        let mut can0_cfg =
            mcan::bus::CanConfigurable::new(HertzU32::Hz(500_000), can0_deps, cx.local.message_ram)
                .unwrap();
        // Only 1 filter for CAN Diag Rx ID
        can0_cfg
            .filters_standard()
            .push(Filter::Classic {
                action: mcan::filter::Action::StoreFifo0,
                filter: CAN_ID_RX,
                mask: StandardId::MAX,
            })
            .ok();

        // Enable new MSG interrupt for FIFO0
        let interrupts = can0_cfg
            .interrupts()
            .split([Interrupt::RxFifo0NewMessage].iter().copied().collect())
            .unwrap();
        let line0_interrupts = can0_cfg.interrupt_configuration().enable_line_0(interrupts);
        let mut can = can0_cfg.finalize().unwrap();
        let _ = can.tx.cancel_multi(TxBufferSet::all());
        let arbiter_cantx: &'static _ = cx.local.arbiter_cantx.insert(Arbiter::new(can.tx));
        let (isotp_isr, isotp_thread) = make_isotp_endpoint(
            Id::Standard(CAN_ID_TX),
            Id::Standard(CAN_ID_RX),
            arbiter_cantx,
            cx.local.isotp_can_fc_signal,
            cx.local.isotp_msg_signal_can,
        );

        // -- USB Init --
        let (usb_clock, gclk2_48) = Pclk::enable(tokens.pclks.usb, gclk2_48);
        let usb_bus = UsbBus::new(
            &(usb_clock.into()),
            &mut mclk,
            pins.usb_dm,
            pins.usb_dp,
            device.usb,
        );
        // Bus allocator
        let usb_alloc: &'static _ = cx.local.usb_alloc.insert(UsbBusAllocator::new(usb_bus));
        // SerialPort CDC
        let uart: &'static _ = cx
            .local
            .arbiter_serial
            .insert(Arbiter::new(SerialPort::new(usb_alloc)));
        // Write down the device serial number in ASCII form
        let sn = serial_number();
        for b in sn {
            let _ = cx.local.usb_sn.push_str(&format!(2; "{b:02X}").unwrap());
        }
        // Build up the USB device
        let usb =
            UsbDeviceBuilder::new(usb_alloc, UsbVidPid(0x16c0, 0x27de), cx.local.usb_ctrl_buf)
                .strings(&[StringDescriptors::default()
                    .manufacturer("rnd-ash")
                    .product("Ultimate NAG52 V2 BOOTLOADER")
                    .serial_number(cx.local.usb_sn)])
                .expect("Failed to set strings")
                .device_class(USB_CLASS_CDC)
                .build()
                .unwrap();
        // Configure the USB ISOTP endpoint (Over serial)
        let (isotp_usb_tx, isotp_usb_thread) = new_usb_isotp(uart, cx.local.isotp_msg_signal_usb);
        let usb_data = UsbData {
            led: pins.led_usb.into(),
            dev: usb,
            isotp: isotp_usb_tx,
        };

        // LED status init (Pulsing)
        let (clock_tcc0, _gclk2_48) = Pclk::enable(tokens.pclks.tcc0_tcc1, gclk2_48);
        let pinout = TCC0Pinout::Pd8(pins.led_status);
        let tcc_led = Tcc0Pwm::new(
            &clock_tcc0.into(),
            HertzU32::kHz(1),
            device.tcc0,
            pinout,
            &mut mclk,
        );

        // Diagnostic init
        let nvm = Nvm::new(device.nvmctrl);
        let trng = Trng::new(&mut mclk, device.trng);
        let mut server = KwpServer::new(nvm, trng, old_bootloader_info, reset_reason);
        if start_diag_in_reprog_mode {
            server.mode = KwpSessionType::Reprogramming;
        }

        // Task init
        diag_task::spawn().unwrap();
        led_task::spawn().unwrap();
        (
            Shared {
                usb: usb_data,
                old_bootloader_info,
            },
            Resources {
                tcc_led,
                usb_isotp_thread: isotp_usb_thread,
                isotp_thread,
                isotp_isr,
                diag_server: server,
                can0_interrupts: line0_interrupts,
                can0_fifo0: can.rx_fifo_0,
            },
        )
    }

    #[task(priority = 2, local = [usb_isotp_thread, isotp_thread, diag_server])]
    async fn diag_task(cx: diag_task::Context) {
        let diag_task::LocalResources {
            usb_isotp_thread,
            isotp_thread,
            diag_server,
            ..
        } = cx.local;
        let mut is_usb: bool = false;
        loop {
            futures::select_biased! {
                buf = isotp_thread.read_payload().fuse() => {
                    is_usb = false;
                    let response = diag_server.process_cmd(
                        buf.payload(),
                        Mono::now().duration_since_epoch().to_millis(),
                    );
                    let _ = isotp_thread.write_payload(&mut Mono, response).await;
                },
                buf = usb_isotp_thread.read().fuse() => {
                    is_usb = true;
                    let response = diag_server.process_cmd(
                        buf.payload(),
                        Mono::now().duration_since_epoch().to_millis(),
                    );
                    let _ = usb_isotp_thread.write(response).await;
                },
                _ = Mono::delay(10.millis()).fuse() => {
                    // Fallthrough so we update KWP server
                }
            }
            if let Some(tx) = diag_server
                .update(Mono::now().duration_since_epoch().to_millis())
                .await
            {
                match is_usb {
                    true => {
                        let _ = usb_isotp_thread.write(tx).await;
                    }
                    false => {
                        let _ = isotp_thread.write_payload(&mut Mono, tx).await;
                    }
                }
            }
        }
    }

    #[task(priority = 2, local=[tcc_led])]
    async fn led_task(cx: led_task::Context) {
        const DELAY_MS: u64 = 20;

        let mut i: u32 = 0;
        let max = cx.local.tcc_led.get_max_duty() / 4;
        let step = max as u64 / (2000 / DELAY_MS);
        loop {
            cx.local.tcc_led.set_duty(Channel::_1, i % max);
            i = i.wrapping_add(step as u32);
            Mono::delay(DELAY_MS.millis()).await;
        }
    }

    #[task(priority = 1, binds=USB_TRCPT0, shared=[usb])]
    #[link_section = ".data.usb_trcpt0"]
    fn usb_trcpt0(cx: usb_trcpt0::Context) {
        cx.shared.usb.poll();
    }

    #[task(priority = 1, binds=USB_TRCPT1, shared=[usb])]
    #[link_section = ".data.usb_trcpt1"]
    fn usb_trcpt1(cx: usb_trcpt1::Context) {
        cx.shared.usb.poll();
    }

    #[task(priority = 1, binds=USB_OTHER, shared=[usb])]
    #[link_section = ".data.usb_other"]
    fn usb_other(cx: usb_other::Context) {
        cx.shared.usb.poll();
    }

    #[task(priority = 1, binds=CAN0, local=[can0_interrupts, can0_fifo0, isotp_isr])]
    #[link_section = ".data.can0"]
    fn can0(cx: can0::Context) {
        for interrupt in cx.local.can0_interrupts.iter_flagged() {
            match interrupt {
                Interrupt::RxFifo0NewMessage => {
                    for msg in cx.local.can0_fifo0.into_iter() {
                        if msg.id() == cx.local.isotp_isr.rx_id {
                            let stmin = ST_MIN_EGS.load(Ordering::Relaxed);
                            let bs = BS_EGS.load(Ordering::Relaxed);
                            cx.local.isotp_isr.on_frame_rx(msg.data(), stmin, bs);
                        }
                    }
                }
                _ => {}
            }
        }
    }
}
