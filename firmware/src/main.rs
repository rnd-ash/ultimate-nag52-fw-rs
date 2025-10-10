#![no_std]
#![no_main]

use atsamd_hal::adc;
use atsamd_hal::adc::Adc0;
use atsamd_hal::adc::Adc1;
use atsamd_hal::bind_multiple_interrupts;
use atsamd_hal::prelude::ExtU64;
use atsamd_hal::prelude::_embedded_hal_watchdog_Watchdog;
use atsamd_hal::rtc::rtic::rtc_clock;
use atsamd_hal::rtic_time::Monotonic;
use atsamd_hal::sercom::Sercom6;
use atsamd_hal::{
    clock::v2::{
        clock_system_at_reset,
        dfll::FromUsb,
        dpll::Dpll,
        gclk::{Gclk, GclkDiv16, GclkDiv8},
        osculp32k::OscUlp32k,
        pclk::Pclk,
        rtcosc::RtcOsc,
    },
    pac::SCB,
};
use core::panic::PanicInfo;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;
use cortex_m_rt::exception;
use defmt_rtt as _;
use diag_common::{dyn_panic::AppPanicInfo, ram_info::modify_bootloader_info};
use mcan::embedded_can::StandardId;
use rtic_sync::portable_atomic::AtomicU16;

use crate::diag::dev_mode::EgsDeviceMode;

pub mod can;
pub mod diag;
pub mod maths;
pub mod sensors;
pub mod solenoids;
pub mod usb;

// -- Interrupt handlers for async APIs --  //
bind_multiple_interrupts!(struct Sercom6Irqs {
    SERCOM6: [SERCOM6_0, SERCOM6_1, SERCOM6_2, SERCOM6_3, SERCOM6_OTHER] => atsamd_hal::sercom::spi::InterruptHandler<Sercom6>;
});

atsamd_hal::bind_multiple_interrupts!(struct DmacIrqs {
    DMAC: [DMAC_0, DMAC_1, DMAC_2, DMAC_OTHER] => atsamd_hal::dmac::InterruptHandler;
});

atsamd_hal::bind_multiple_interrupts!(pub struct Adc0Irqs {
    ADC0: [ADC0_RESRDY, ADC0_OTHER] => adc::InterruptHandler<Adc0>;
});

atsamd_hal::bind_multiple_interrupts!(pub struct Adc1Irqs {
    ADC1: [ADC1_RESRDY, ADC1_OTHER] => adc::InterruptHandler<Adc1>;
});

// -- Panic handler -- //

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    defmt::error!("{}", info);
    modify_bootloader_info(|inf| {
        let panic = AppPanicInfo::new(info);
        inf.app_panic = Some(panic);
    });
    SCB::sys_reset();
}

// RTIC Monotonic declaration using RTC and Clock32K
atsamd_hal::rtc_monotonic!(Mono, rtc_clock::Clock32k);

// ISOTP Spec
const ISOTP_BUF_SIZE: usize = 4096;
pub const CAN_ID_TX: StandardId = unsafe { StandardId::new_unchecked(0x7E9) };
pub const CAN_ID_RX: StandardId = unsafe { StandardId::new_unchecked(0x7E1) };

#[rtic::app(device = atsamd_hal::pac, dispatchers = [DAC_EMPTY_0, DAC_EMPTY_1, DAC_RESRDY_0, DAC_RESRDY_1])]
mod app {
    use core::sync::atomic::Ordering;

    use crate::{
        can::{
            data::slave_mode::SolenoidControl,
            egs52::Egs52Can,
            slave::{SlaveCan, SolenoidReport},
            CanLayer, CanLayerTy,
        },
        diag::KwpServer,
        sensors::{AdcData, AdcPins},
        solenoids::{
            tcc_sol::TccSol,
            tle8242::{Tle8242, Tle8242Pins, TLE_SPI_BAUD},
            SolenoidControler,
        },
        usb::UsbData,
    };
    use atsamd_hal::{
        can::Dependencies,
        clock::v2::{pclk, types::Can0},
        dmac::{self, DmaController, PriorityLevel},
        fugit::HertzU32,
        serial_number,
        usb::{
            usb_device::{
                bus::UsbBusAllocator,
                device::{StringDescriptors, UsbDeviceBuilder, UsbRev, UsbVidPid},
            },
            UsbBus,
        },
        watchdog::Watchdog,
    };
    use bsp::can_deps::{Capacities, RxFifo0};
    use cortex_m::asm::wfi;
    use defmt::{info, println};
    use diag_common::isotp_endpoints::{
        can_isotp::{make_isotp_endpoint, IsoTpInterruptHandler, IsotpConsumer, IsotpCtsMsg},
        usb_isotp::{new_usb_isotp, UsbIsoTpConsumer},
        SharedIsoTpBuf,
    };
    use futures::FutureExt;
    use heapless::format;
    use mcan::{
        embedded_can::{Id, StandardId},
        filter::Filter,
        interrupt::{state::EnabledLine0, Interrupt, OwnedInterruptSet},
        message::Raw,
        messageram::SharedMemory,
    };
    use rtic_sync::{arbiter::Arbiter, signal::Signal};
    use usbd_serial::{DefaultBufferStore, SerialPort, USB_CLASS_CDC};

    use super::*;

    #[local]
    struct Resources {
        adc_data: AdcData,

        isotp_isr: IsoTpInterruptHandler<'static, Can0, Capacities, ISOTP_BUF_SIZE>,
        isotp_thread: IsotpConsumer<'static, Can0, Capacities, ISOTP_BUF_SIZE>,
        usb_isotp_thread: UsbIsoTpConsumer<
            'static,
            UsbBus,
            DefaultBufferStore,
            DefaultBufferStore,
            ISOTP_BUF_SIZE,
        >,

        can0_interrupts: OwnedInterruptSet<pclk::ids::Can0, EnabledLine0>,
        can0_fifo0: RxFifo0,

        diag_server: KwpServer,
    }

    #[shared]
    struct Shared {
        #[lock_free]
        usb_data: UsbData<'static>,
        wdt: Watchdog,
        can_layer: CanLayerTy,

        slave_can: SlaveCan,
        soltcc: TccSol,
    }

    #[init(local = [
        #[link_section = ".can"]
        message_ram: SharedMemory<Capacities> = SharedMemory::new(),
        usb_ctrl_buf: [u8; 256] = [0; 256],
        usb_alloc: Option<UsbBusAllocator<UsbBus>> = None,
        usb_sn: heapless::String<32> = heapless::String::new(),
        isotp_can_fc_signal: Signal<IsotpCtsMsg> = Signal::new(),
        isotp_msg_signal_can: Signal<SharedIsoTpBuf<ISOTP_BUF_SIZE>> = Signal::new(),
        isotp_msg_signal_usb: Signal<SharedIsoTpBuf<ISOTP_BUF_SIZE>> = Signal::new(),
        arbiter_cantx: Option<Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>> = None
        arbiter_serial: Option<Arbiter<SerialPort<'static, UsbBus, DefaultBufferStore, DefaultBufferStore>>> = None,
    ])]
    fn init(cx: init::Context) -> (Shared, Resources) {
        let mut device = cx.device;
        let mut core: rtic::export::Peripherals = cx.core;
        let pins = bsp::Pins::new(device.port);
        let (mut buses, clocks, tokens) = clock_system_at_reset(
            device.oscctrl,
            device.osc32kctrl,
            device.gclk,
            device.mclk,
            &mut device.nvmctrl,
        );
        // Steal PAC controller (We need mclk later)
        let (_, _, _, mut mclk) = unsafe { clocks.pac.steal() };

        // Enable watchdog alarm
        let mut wdt = Watchdog::new(device.wdt);
        wdt.feed();

        // Obeying the max clock speeds for 125C operation (For AEC-Q100),
        // the processor clock setup shall be configured as follows
        //
        //     OSCULP(32Khz)
        //     ├── RTIC OS Monotonic
        //     └── Watchdog (2 sec timeout)
        //
        //     DFLL(48Mhz)
        //     ├── GCLK1 (2Mhz)
        //     │   ├── DPLL0(100Mhz)
        //     C   │   └── GCLK0(100Mhz)
        //     L   │       ├── TCC2 (TCC Solenoid)
        //     K   │       └── F_CPU
        //     │   │           └── QSPI
        //     R   └── DPLL1(160Mhz)
        //     E       ├── GCLK2(40Mhz)
        //     C       │   └── CAN0
        //     O       ├── GCLK3(80Mhz)
        //     V       │   ├── ADC0
        //     E       │   ├── ADC1
        //     R       │   ├── SERCOM(s)
        //     Y       │   └── EIC
        //     │       └── GCLK4(160Mhz)
        //     |           └── TCC0
        //     └── GCLK6 (48Mhz)
        //         └── USB

        // Take GCLK1 from DFLL48 and divide by 12 to get 2Mhz
        let (gclk1, dfll) = Gclk::from_source(tokens.gclks.gclk1, clocks.dfll);
        let gclk1 = gclk1.div(GclkDiv16::Div(24)).enable(); // Gclk1 is now at 2Mhz
                                                            // Now configure both DPLLs. (loop div values from ATMEL SMART)
                                                            // DPLL0 loop div 50 = 100Mhz
                                                            // DPLL1 loop div 80 = 160Mhz
        let (clk_dpll0, gclk1) = Pclk::enable(tokens.pclks.dpll0, gclk1);
        let (clk_dpll1, _gclk1) = Pclk::enable(tokens.pclks.dpll1, gclk1);
        // DPLL0 at 100Mhz (2*50)
        let dpll0 = Dpll::from_pclk(tokens.dpll0, clk_dpll0)
            .loop_div(50, 0)
            .enable();
        // DPLL1 at 160Mhz (2x80)
        let dpll1 = Dpll::from_pclk(tokens.dpll1, clk_dpll1)
            .loop_div(80, 0)
            .enable();
        // Now swap GCLK0 so it is using DPLL0 as a reference rather than DFLL
        let (gclk0_100, dfll, _dpll0) = clocks.gclk0.swap_sources(dfll, dpll0);
        // Switch DFLL to running with USB Clock recovery
        let (dfll_usb, _old_mode) = dfll.into_mode(FromUsb, |_dfll| {});
        let (gclk6, _dfll) = Gclk::from_source(tokens.gclks.gclk6, dfll_usb);
        let gclk6_48 = gclk6.enable();

        // Enable GCLK2 at 40Mhz (160/(4))
        let (gclk2, dpll1) = Gclk::from_source(tokens.gclks.gclk2, dpll1);
        let gclk2_40 = gclk2.div(GclkDiv8::Div(4)).enable();
        // Enable GCLK3 at 80Mhz (160/(2))
        let (gclk3, dpll1) = Gclk::from_source(tokens.gclks.gclk3, dpll1);
        let gclk3_80 = gclk3.div(GclkDiv8::Div(2)).enable();
        // Enable GCLK4 at 160Mhz (Match DPLL1 freq)
        let (gclk4, _dpll1) = Gclk::from_source(tokens.gclks.gclk4, dpll1);
        let gclk4_160 = gclk4.enable();

        // Enable the 32Khz clock and start the RTIC Monotonic driver
        let (osculp32k, _) = OscUlp32k::enable(tokens.osculp32k.osculp32k, clocks.osculp32k_base);
        let _ = RtcOsc::enable(tokens.rtcosc, osculp32k);
        Mono::start(device.rtc);

        // Setup the systick timer to generate an interrupt at 10Hz, this will be
        // used to monitor CPU usage
        core.SYST
            .set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
        // Set reload value to maximum as CPU monitor resets it
        core.SYST.set_reload(0xFF_FFFF);
        core.SYST.clear_current();
        core.SYST.enable_counter();
        core.SYST.enable_interrupt();

        // Init ADCs
        let (pclk_adc0, gclk3_80) = Pclk::enable(tokens.pclks.adc0, gclk3_80);
        let (pclk_adc1, gclk3_80) = Pclk::enable(tokens.pclks.adc1, gclk3_80);
        let apb_adc0 = buses.apb.enable(tokens.apbs.adc0);
        let apb_adc1 = buses.apb.enable(tokens.apbs.adc1);
        let adc_pins = AdcPins {
            vbatt_sense: pins.vbatt_sense.into(),
            vsensor_sense: pins.vsensor_sense.into(),
            accel_plus: pins.accel_p_sense.into(),
            accel_minus: pins.accel_m_sense.into(),
            tft: pins.tft.into(),
            sol_pwr_sense: pins.sol_pwr_sense.into(),
            vsol_sense: pins.vsol_sense.into(),
        };
        let adc_data = AdcData::new(
            device.adc0,
            device.adc1,
            device.supc,
            adc_pins,
            apb_adc0,
            apb_adc1,
            pclk_adc0,
            pclk_adc1,
        );

        // DMA Init (SPI + I2C Requires this)
        let dmac = DmaController::init(device.dmac, &mut device.pm);
        let mut fut_dmac = dmac.into_future(DmacIrqs);
        let dma_channels = fut_dmac.split();
        // TLE8242 SPI Channels
        let dma_ch0 = dma_channels.0.init(PriorityLevel::Lvl0);
        let dma_ch1 = dma_channels.1.init(PriorityLevel::Lvl0);

        // eeprom I2C Channels
        let dma_ch2 = dma_channels.2.init(PriorityLevel::Lvl0);
        let dma_ch3 = dma_channels.3.init(PriorityLevel::Lvl0);

        let (tcc01_clock, gclk4_160) = Pclk::enable(tokens.pclks.tcc0_tcc1, gclk4_160);
        let tcc01_clock_compat = tcc01_clock.into();
        // Much better resolution to run this at 100Mhz vs 160Mhz
        let (tcc23_clock, gclk0_100) = Pclk::enable(tokens.pclks.tcc2_tcc3, gclk0_100);

        // -- TCC Solenoid init  --
        let sol_tcc = TccSol::new(
            device.tcc1,
            device.tcc2,
            &tcc01_clock_compat,
            &tcc23_clock.into(),
            pins.tcc_pwm.into(),
            pins.tcc_cutoff.into(),
            &mut mclk,
        );

        // -- TLE8242 init --

        // Init SPI
        let (pclk_sercom6, gclk3_80) = Pclk::enable(tokens.pclks.sercom6, gclk3_80);
        let spi = bsp::tle_spi(
            pclk_sercom6,
            TLE_SPI_BAUD,
            device.sercom6,
            &mut mclk,
            pins.tle_sck,
            pins.tle_si,
            pins.tle_so,
        );
        let spi_fut = spi.into_future(Sercom6Irqs);
        let spi_fut_dma = spi_fut.with_dma_channels(dma_ch0, dma_ch1);

        let tle8242_pins = Tle8242Pins {
            phase_sync: pins.tle_phase_sync.into(),
            fault: pins.tle_fault.into(),
            reset: pins.tle_reset.into(),
            enable: pins.tle_enable.into(),
            clk: pins.tle_clk.into(),
            cs: pins.tle_cs.into(),
            led: pins.led_tle_act.into(),
        };

        let tle8242: Tle8242<dmac::Ch0, dmac::Ch1> = Tle8242::new(
            tle8242_pins,
            spi_fut_dma,
            &mut mclk,
            &tcc01_clock_compat,
            device.tcc0,
        );

        let solenoid_io = SolenoidControler::new(tle8242, pins.sol_pwr_en.into());

        // -- CAN init --
        let (clk_can, gclk2_40) = Pclk::enable(tokens.pclks.can0, gclk2_40);
        let (can0_deps, gclk2_40) = Dependencies::new(
            gclk2_40,
            clk_can,
            clocks.ahbs.can0,
            pins.can_rx.into_mode(),
            pins.can_tx.into_mode(),
            device.can0,
        );

        let mut can0_cfg =
            mcan::bus::CanConfigurable::new(HertzU32::Hz(500_000), can0_deps, cx.local.message_ram)
                .unwrap();
        can0_cfg
            .filters_standard()
            .push(Filter::Classic {
                action: mcan::filter::Action::StoreFifo0,
                filter: StandardId::ZERO,
                mask: StandardId::ZERO,
            })
            .ok();

        // Enable new MSG interrupt for FIFO0
        let interrupts = can0_cfg
            .interrupts()
            .split([Interrupt::RxFifo0NewMessage].iter().copied().collect())
            .unwrap();
        let line0_interrupts = can0_cfg.interrupt_configuration().enable_line_0(interrupts);
        let mut can = can0_cfg.finalize().unwrap();
        let arbiter_cantx: &'static _ = cx.local.arbiter_cantx.insert(Arbiter::new(can.tx));
        let (isotp_isr, isotp_thread) = make_isotp_endpoint(
            Id::Standard(CAN_ID_TX),
            Id::Standard(CAN_ID_RX),
            arbiter_cantx,
            cx.local.isotp_can_fc_signal,
            cx.local.isotp_msg_signal_can,
        );

        // Init USB
        let (usb_clock, gclk2_48) = Pclk::enable(tokens.pclks.usb, gclk6_48);
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
                    .product("Ultimate NAG52 V2")
                    .serial_number(cx.local.usb_sn)])
                .expect("Failed to set strings")
                .device_class(USB_CLASS_CDC)
                .usb_rev(UsbRev::Usb200)
                .build()
                .unwrap();
        // Configure the USB ISOTP endpoint (Over serial)
        let (isotp_usb_tx, isotp_usb_thread) = new_usb_isotp(uart, cx.local.isotp_msg_signal_usb);
        let usb_data = UsbData {
            led: pins.led_usb.into(),
            dev: usb,
            isotp: isotp_usb_tx,
        };

        let can_layer = CanLayerTy::Egs52(Egs52Can::new());

        async_init::spawn(arbiter_cantx, solenoid_io)
            .unwrap_or_else(|_| panic!("Could not start async init"));
        cpu_monitor::spawn().unwrap();

        wdt.feed();
        (
            Shared {
                usb_data,
                wdt,
                can_layer,
                slave_can: SlaveCan::new(),
                soltcc: sol_tcc,
            },
            Resources {
                adc_data,
                can0_fifo0: can.rx_fifo_0,
                can0_interrupts: line0_interrupts,
                isotp_isr,
                isotp_thread,
                usb_isotp_thread: isotp_usb_thread,

                diag_server: KwpServer::new(),
            },
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        let mut syst = unsafe { cortex_m::Peripherals::steal().SYST };
        loop {
            // Stop interrupts from context switch
            cortex_m::interrupt::disable();
            syst.clear_current();
            syst.enable_counter();
            // Wait for something to wake up the CPU
            wfi();
            // Stop counter (We read it later)
            syst.disable_counter();
            // Enable context switching, CPU will now perform
            // the interrupt
            unsafe {
                cortex_m::interrupt::enable();
            }

            // We can now write down CPU usage since counters were disabled
            let overflow_count = SYST_OVERFLOW_COUNT.swap(0, Ordering::Relaxed);
            // Write down CPU usage after interrupt performed (Lowers latency to interrupt)
            CPU_SLEEP_TICKS.fetch_add(
                (0xFF_FFFF - syst.cvr.read()) + (0xFF_FFFF * overflow_count),
                Ordering::Relaxed,
            );
        }
    }

    #[task(priority = 2, shared=[wdt])]
    async fn cpu_monitor(mut ctx: cpu_monitor::Context) {
        const TPS: u32 = 100_000_000;
        const SECOND_MILLIS: u32 = 1000;
        const UPDATES_PER_SEC: u32 = 2;
        const MAX_TICKS_PER_UPDATE: u32 = TPS / UPDATES_PER_SEC;
        loop {
            let now = Mono::now();
            // Reset
            let asleep_ticks = CPU_SLEEP_TICKS.swap(0, Ordering::SeqCst);
            let percentage: f32 =
                ((MAX_TICKS_PER_UPDATE - asleep_ticks) * 100) as f32 / MAX_TICKS_PER_UPDATE as f32;

            info!("CPU: {}%", percentage);
            ctx.shared.wdt.lock(|wdt| wdt.feed());
            Mono::delay_until(now + ((SECOND_MILLIS / UPDATES_PER_SEC) as u64).millis()).await;
        }
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

    #[task(priority = 1)]
    async fn async_init(
        _: async_init::Context,
        can_tx: &'static Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>,
        mut solenoid_io: SolenoidControler<dmac::Ch0, dmac::Ch1>,
    ) {
        adc_task::spawn().unwrap();
        solenoid_io.init().await;
        gearbox_task::spawn(can_tx, solenoid_io)
            .unwrap_or_else(|_| panic!("Could not start async init"));
        diag_task::spawn().unwrap();
        // Wait one second - Most likely a crash will happen whilst all the async tasks
        // are initializing
        Mono::delay(1000u64.millis()).await;
        // Now reset the reset counter
        diag_common::ram_info::modify_bootloader_info(|info| {
            info.reset_counter = 0;
        });
    }

    #[task(priority = 1, local=[adc_data], shared=[wdt])]
    async fn adc_task(mut cx: adc_task::Context) {
        loop {
            let now = Mono::now();
            cx.local.adc_data.update().await;
            cx.shared.wdt.lock(|wdt| wdt.feed());
            Mono::delay_until(now + 20u64.millis()).await;
        }
    }

    #[task(priority = 2, shared=[can_layer, slave_can, soltcc])]
    async fn gearbox_task(
        mut cx: gearbox_task::Context,
        can_tx: &'static Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>,
        mut solenoid_controller: SolenoidControler<dmac::Ch0, dmac::Ch1>,
    ) {
        // Gearbox has started, set device mode
        DEVICE_MODE.store(EgsDeviceMode::SLAVE.bits(), Ordering::Relaxed);
        loop {
            let now = Mono::now();
            let mode = EgsDeviceMode::from_bits_retain(DEVICE_MODE.load(Ordering::Relaxed));

            // Process inputs
            //cx.shared.can_layer.lock(|lck| lck.read_signals());

            // TODO GEARBOX STUFF

            // Set outputs
            //let mut can = can_tx.access().await;
            //let _ = cx.shared.can_layer.lock(|lck| {
            //    lck.write_signals();
            //    lck.transmit(&mut can)
            //});

            if mode.contains(EgsDeviceMode::SLAVE) {
                // Slave mode has special logic
                use can::CanLayer;
                let mut control_frame = SolenoidControl::ZERO;
                cx.shared
                    .slave_can
                    .lock(|lck| lck.read_signals(&mut control_frame));

                let spc_req = control_frame.spc_req().swap_bytes();
                let mpc_req = control_frame.mpc_req().swap_bytes();
                let tcc_req = control_frame.tcc_req();

                solenoid_controller.set_mpc_current(mpc_req).await;
                solenoid_controller.set_spc_current(spc_req).await;

                cx.shared.soltcc.lock(|tcc| {
                    solenoid_controller.set_tcc_pwm((tcc_req as u16) << 8, tcc);
                });

                // Actuate solenoids
                // Todo - Only do delay and second query if first failed
                let mut rpt = SolenoidReport::ZERO;
                let spc_current = solenoid_controller.read_spc_current();
                let mpc_current = solenoid_controller.read_mpc_current();
                solenoid_controller.update_current_readings().await;

                rpt.set_mpc_curr(mpc_current.swap_bytes());
                rpt.set_spc_curr(spc_current.swap_bytes());

                let mut can = can_tx.access().await;
                let _ = cx.shared.slave_can.lock(|lck| {
                    lck.write_signals(&rpt);
                    lck.transmit(&mut can)
                });
            }

            Mono::delay_until(now + 20u64.millis()).await;
        }
    }

    // -- HARDWARE TASKS BELOW --

    #[task(priority = 1, binds=USB_TRCPT0, shared=[usb_data])]
    fn usb_trcpt0(cx: usb_trcpt0::Context) {
        cx.shared.usb_data.poll();
    }

    #[task(priority = 1, binds=USB_TRCPT1, shared=[usb_data])]
    fn usb_trcpt1(cx: usb_trcpt1::Context) {
        cx.shared.usb_data.poll();
    }

    #[task(priority = 1, binds=USB_OTHER, shared=[usb_data])]
    fn usb_other(cx: usb_other::Context) {
        cx.shared.usb_data.poll();
    }

    #[task(priority = 1, binds=CAN0, local=[can0_interrupts, can0_fifo0, isotp_isr], shared=[can_layer, slave_can])]
    fn can0(mut cx: can0::Context) {
        let mut buf = [0; 8];
        for interrupt in cx.local.can0_interrupts.iter_flagged() {
            match interrupt {
                Interrupt::RxFifo0NewMessage => {
                    for msg in cx.local.can0_fifo0.into_iter() {
                        if msg.id() == cx.local.isotp_isr.rx_id {
                            cx.local.isotp_isr.on_frame_rx(msg.data(), 2, 8);
                        } else {
                            buf[0..msg.dlc() as usize].copy_from_slice(msg.data());
                            cx.shared.can_layer.lock(|lck| {
                                lck.on_frame(msg.id(), &buf);
                            });
                            cx.shared.slave_can.lock(|lck| {
                                lck.on_frame(msg.id(), &buf);
                            })
                        }
                    }
                }
                _ => {}
            }
        }
    }

    // -- Interrupts for TCC2 for the TCC Solenoid -- //
    //    These have the HIGHEST priority to keep     //
    //    the TCC solenoid waveform accurate!         //

    #[task(priority = 3, binds=TCC2_OTHER, shared=[soltcc])]
    fn tcc2_ovf(mut cx: tcc2_ovf::Context) {
        cx.shared.soltcc.lock(|lck| lck.on_tcc_ovf());
    }

    #[task(priority = 3, binds=TCC2_MC0, shared=[soltcc])]
    fn tcc2_mc0(mut cx: tcc2_mc0::Context) {
        cx.shared.soltcc.lock(|lck| lck.on_tcc_mc0());
    }

    #[task(priority = 3, binds=TCC2_MC1, shared=[soltcc])]
    fn tcc2_mc1(mut cx: tcc2_mc1::Context) {
        cx.shared.soltcc.lock(|lck| lck.on_tcc_mc1());
    }
}

// For CPU performance counting
static CPU_SLEEP_TICKS: AtomicU32 = AtomicU32::new(0);
static SYST_OVERFLOW_COUNT: AtomicU32 = AtomicU32::new(0);
#[exception]
fn SysTick() {
    SYST_OVERFLOW_COUNT.fetch_add(1, Ordering::Relaxed);
}

// For Device mode operation
static DEVICE_MODE: AtomicU16 = AtomicU16::new(EgsDeviceMode::INITIALIZATION.bits());
