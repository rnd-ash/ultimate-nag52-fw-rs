#![no_std]
#![no_main]

use atsamd_hal::adc;
use atsamd_hal::adc::Adc0;
use atsamd_hal::adc::Adc1;
use atsamd_hal::bind_multiple_interrupts;
use atsamd_hal::pac::Peripherals;
use atsamd_hal::pac::SCB;
use atsamd_hal::rtc::rtic::rtc_clock;
use atsamd_hal::rtic_time::Monotonic;
use atsamd_hal::sercom::Sercom2;
use atsamd_hal::sercom::Sercom6;
use bbqueue::nicknames::Churrasco;
use core::panic::PanicInfo;
use cortex_m_rt::{ExceptionFrame, exception};
//use defmt_rtt as _;
use diag_common::hal_extensions::dsu::Dsu;
use diag_common::parse_git_sha;
use diag_common::parse_u8;
use diag_common::smarteeprom::CodeSectionInfo;
use diag_common::{dyn_panic::AppPanicInfo, ram_info::modify_bootloader_info};
use mcan::embedded_can::StandardId;

pub mod diag;
pub mod egs_logic_impl;
pub mod hal_extension;
pub mod ram_test;
pub mod sensors;
pub mod solenoids;
pub mod storage;
pub mod tasks;
pub mod usb;

#[unsafe(link_section = ".log_ram.log_buffer")]
static LOG_BUFFER: bbqueue::nicknames::Churrasco<4096> = Churrasco::new();

// -- Interrupt handlers for async APIs --  //
bind_multiple_interrupts!(struct Sercom6Irqs {
    SERCOM6: [SERCOM6_0, SERCOM6_1, SERCOM6_2, SERCOM6_3, SERCOM6_OTHER] => atsamd_hal::sercom::spi::InterruptHandler<Sercom6>;
});

bind_multiple_interrupts!(struct Sercom2Irqs {
    SERCOM2: [SERCOM2_0, SERCOM2_1, SERCOM2_2, SERCOM2_3, SERCOM2_OTHER] => atsamd_hal::sercom::i2c::InterruptHandler<Sercom2>;
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

// -- Timestamp for DEFMT -- //

defmt::timestamp!("{=u64:us}", {
    Mono::now().duration_since_epoch().to_micros()
});

// -- Panic handler -- //

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let p = unsafe { Peripherals::steal() };
    let mut dsu = Dsu::new(p.dsu, &p.pac).unwrap();

    modify_bootloader_info(&mut dsu, |inf| {
        let panic = AppPanicInfo::new(info);
        inf.app_panic = Some(panic);
    });
    SCB::sys_reset();
}

pub const fn create_code_info(name: [u8; 20]) -> CodeSectionInfo {
    CodeSectionInfo {
        name,
        git_sha: parse_git_sha(env!("VERGEN_GIT_SHA")),
        version_major: parse_u8(env!("CARGO_PKG_VERSION_MAJOR")),
        version_minor: parse_u8(env!("CARGO_PKG_VERSION_MINOR")),
        version_patch: parse_u8(env!("CARGO_PKG_VERSION_PATCH")),
        compile_year: parse_u8(env!("BUILD_YEAR")),
        compile_month: parse_u8(env!("BUILD_MONTH")),
        compile_week: parse_u8(env!("BUILD_WEEK")),
        compile_day: parse_u8(env!("BUILD_DAY")),
        rustc_version_major: parse_u8(env!("RUSTC_VER_MAJOR")),
        rustc_version_minor: parse_u8(env!("RUSTC_VER_MINOR")),
        rustc_version_patch: parse_u8(env!("RUSTC_VER_PATCH")),
        #[cfg(debug_assertions)]
        is_debug: 1,
        #[cfg(not(debug_assertions))]
        is_debug: 0,
    }
}

// RTIC Monotonic declaration using RTC and Clock32K
atsamd_hal::rtc_monotonic!(Mono, rtc_clock::Clock32k);

// ISOTP Spec
const ISOTP_BUF_SIZE: usize = 4096;
pub const CAN_ID_DIAG_TX: StandardId = unsafe { StandardId::new_unchecked(0x7E9) };
pub const CAN_ID_DIAG_RX: StandardId = unsafe { StandardId::new_unchecked(0x7E1) };

// Logging ram definition

#[rtic::app(device = atsamd_hal::pac, dispatchers = [DAC_EMPTY_0, DAC_EMPTY_1, EVSYS_0, EVSYS_1])]
mod app {

    use core::{ops::Sub, sync::atomic::AtomicU8};

    use crate::{
        diag::{KwpServer, PerfStatsTracker},
        sensors::{AdcData, SensorData, speed_sensors::AllSpeedSensors},
        solenoids::{SolenoidController, tcc_sol::TccSol},
        storage::eeprom::Eeprom,
        usb::UsbData,
    };
    use atsamd_hal::{
        clock::v2::{pclk, types::Can0},
        dmac::{self},
        fugit::ExtU64,
        usb::{UsbBus, usb_device::bus::UsbBusAllocator},
        watchdog::Watchdog,
    };
    use bsp::can_deps::{Capacities, RxDedicated, RxFifo0};
    use defmt::println;
    use diag_common::{
        hal_extensions::{
            dsu::Dsu,
            qspi_async::{OneShot, Qspi},
        },
        isotp_endpoints::{
            SharedIsoTpBuf,
            can_isotp::{IsoTpInterruptHandler, IsotpConsumer, IsotpCtsMsg},
            usb_isotp::UsbIsoTpConsumer,
        },
        qspi_driver::QspiStorage,
    };

    use super::*;
    use crate::diag::SolenoidReport;
    use crate::egs_logic_impl::V2Gearbox;
    use crate::tasks::PerformanceInfo;
    use egs_logic::{
        egs_can::{self, CanLayerTy},
        vars::{GearboxInputs, GearboxOutputs, GearboxVars},
    };

    use mcan::{
        interrupt::{Interrupt, OwnedInterruptSet, state::EnabledLine0},
        message::Raw,
        messageram::SharedMemory,
        rx_dedicated_buffers::DynRxDedicatedBuffer,
    };
    use rtic_sync::{arbiter::Arbiter, signal::Signal};
    use usbd_serial::{DefaultBufferStore, SerialPort};

    #[local]
    pub struct Resources {
        pub adc_data: AdcData,
        pub speed_sensors: AllSpeedSensors,

        pub isotp_isr: IsoTpInterruptHandler<'static, Can0, Capacities, ISOTP_BUF_SIZE>,
        pub isotp_thread: IsotpConsumer<'static, Can0, Capacities, ISOTP_BUF_SIZE>,
        pub usb_isotp_thread: UsbIsoTpConsumer<
            'static,
            UsbBus,
            DefaultBufferStore,
            DefaultBufferStore,
            ISOTP_BUF_SIZE,
        >,

        pub can0_interrupts: OwnedInterruptSet<pclk::ids::Can0, EnabledLine0>,
        pub can0_fifo0: RxFifo0,
        pub can0_dedicated: RxDedicated,
        pub diag_server: KwpServer,
        pub wdt: Watchdog,
    }

    #[shared]
    pub struct Shared {
        #[lock_free]
        pub usb_data: UsbData<'static>,
        pub log_mode: AtomicU8,

        pub can_layer: CanLayerTy,

        pub slave_can: egs_can::slave::SlaveCan,
        pub soltcc: TccSol,
        pub sensor_data: &'static mut SensorData,

        pub perf_info: PerformanceInfo,
        pub dsu: &'static Arbiter<diag_common::hal_extensions::dsu::Dsu>,
        pub qspi: &'static Arbiter<diag_common::qspi_driver::QspiStorage>,
        pub perf_stats: PerfStatsTracker,
        pub gearbox: V2Gearbox<'static>,
        pub solenoid_statuses: SolenoidReport,
    }

    #[init(local = [
        #[unsafe(link_section = ".can")]
        message_ram: SharedMemory<Capacities> = SharedMemory::new(),
        usb_ctrl_buf: [u8; 256] = [0; 256],
        usb_alloc: Option<UsbBusAllocator<UsbBus>> = None,
        usb_sn: heapless::String<32> = heapless::String::new(),
        isotp_can_fc_signal: Signal<IsotpCtsMsg> = Signal::new(),
        isotp_msg_signal_can: Signal<SharedIsoTpBuf<ISOTP_BUF_SIZE>> = Signal::new(),
        isotp_msg_signal_usb: Signal<SharedIsoTpBuf<ISOTP_BUF_SIZE>> = Signal::new(),
        arbiter_cantx: Option<Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>> = None
        arbiter_serial: Option<Arbiter<SerialPort<'static, UsbBus, DefaultBufferStore, DefaultBufferStore>>> = None,
        dsu_init: Option<Arbiter<diag_common::hal_extensions::dsu::Dsu>> = None,
        qspi_init: Option<Arbiter<diag_common::qspi_driver::QspiStorage>> = None,
        // Vars below are used by diagnostic and common functions
        inputs: GearboxInputs = GearboxInputs::new(),
        vars: GearboxVars = GearboxVars::new(),
        outputs: GearboxOutputs = GearboxOutputs::new(),
        sensor_data: SensorData = SensorData::new()
    ])]
    fn init(cx: init::Context) -> (Shared, Resources) {
        tasks::init(cx)
    }

    #[task(priority = 1)]
    async fn async_init(
        _ctx: async_init::Context,
        dsu: &'static Arbiter<Dsu>,
        qspi: &'static Arbiter<QspiStorage>,
    ) {
        qspi.access().await.init_chip(&mut Mono).await;
        // Wait 5 seconds - Most likely a crash will happen whilst all the async tasks
        // are initializing
        Mono::delay(5000u64.millis()).await;
        // Now reset the reset counter
        let mut dsu_lock = dsu.access().await;
        diag_common::ram_info::modify_bootloader_info(&mut dsu_lock, |info| {
            info.reset_counter = 0;
        });
    }

    #[idle(shared=[perf_info, &dsu])]
    fn idle(mut ctx: idle::Context) -> ! {
        tasks::idle(&mut ctx)
    }

    #[task(priority = 1, shared=[perf_info, perf_stats], local=[wdt])]
    async fn perf_monitor(ctx: perf_monitor::Context, tps: u32) {
        tasks::performance_monitor(ctx, tps).await;
    }

    #[task(priority = 1, local = [usb_isotp_thread, isotp_thread, diag_server], shared=[perf_stats, gearbox, sensor_data, solenoid_statuses])]
    async fn diag_task(cx: diag_task::Context) {
        tasks::diag_task(cx).await;
    }

    #[task(priority = 2, local=[adc_data], shared=[sensor_data])]
    async fn sensor_query(cx: sensor_query::Context) {
        tasks::sensor_query(cx).await;
    }

    #[task(priority = 3,
        shared=[
            can_layer, slave_can, soltcc, sensor_data, perf_stats,
            gearbox, solenoid_statuses
        ],
        local=[speed_sensors]
    )]
    async fn gearbox_task(
        cx: gearbox_task::Context,
        can_tx: &'static Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>,
        solenoid_controller: SolenoidController<dmac::Ch0, dmac::Ch1>,
        eeprom: Eeprom<dmac::Ch2>,
    ) {
        tasks::gearbox_task(cx, can_tx, eeprom, solenoid_controller).await;
    }

    // -- HARDWARE TASKS BELOW --

    #[task(priority = 1, binds=USB_TRCPT0, shared=[usb_data])]
    #[unsafe(link_section = ".data.usbtrcpt0")]
    fn usb_trcpt0(cx: usb_trcpt0::Context) {
        cx.shared.usb_data.poll();
    }

    #[task(priority = 1, binds=USB_TRCPT1, shared=[usb_data])]
    #[unsafe(link_section = ".data.usbtrcpt1")]
    fn usb_trcpt1(cx: usb_trcpt1::Context) {
        cx.shared.usb_data.poll();
    }

    #[task(priority = 1, binds=USB_OTHER, shared=[usb_data])]
    #[unsafe(link_section = ".data.usbother")]
    fn usb_other(cx: usb_other::Context) {
        cx.shared.usb_data.poll();
    }

    #[task(priority = 1, binds=WDT)]
    #[unsafe(link_section = ".data.wdt")]
    fn wdt_ew(_cx: wdt_ew::Context) {
        defmt::error!("Watchdog early warning!")
    }

    #[task(priority = 2, binds=CAN0, local=[can0_interrupts, can0_fifo0, can0_dedicated, isotp_isr, buf: [u8; 8] = [0; 8]], shared=[can_layer, slave_can])]
    #[unsafe(link_section = ".data.can0")]
    fn can0(mut cx: can0::Context) {
        let buf = cx.local.buf;
        let now_ms = Mono::now().duration_since_epoch().to_millis() as u32;
        use egs_logic::egs_can::CanLayer;

        for interrupt in cx.local.can0_interrupts.iter_flagged() {
            match interrupt {
                Interrupt::MessageStoredToDedicatedRxBuffer => {
                    while let Ok(msg) = cx.local.can0_dedicated.receive_any() {
                        if msg.id() == cx.local.isotp_isr.rx_id {
                            cx.local.isotp_isr.on_frame_rx(msg.data(), 2, 8);
                        } else {
                            buf[0..msg.dlc() as usize].copy_from_slice(msg.data());
                            cx.shared.can_layer.lock(|lck| {
                                lck.on_frame(now_ms, msg.id(), msg.dlc(), &buf);
                            });
                            cx.shared.slave_can.lock(|lck| {
                                lck.on_frame(now_ms, msg.id(), msg.dlc(), &buf);
                            })
                        }
                    }
                }
                Interrupt::RxFifo0NewMessage => {
                    for msg in &mut cx.local.can0_fifo0 {
                        if msg.id() == cx.local.isotp_isr.rx_id {
                            cx.local.isotp_isr.on_frame_rx(msg.data(), 2, 8);
                        } else {
                            buf[0..msg.dlc() as usize].copy_from_slice(msg.data());
                            cx.shared.can_layer.lock(|lck| {
                                lck.on_frame(now_ms, msg.id(), msg.dlc(), &buf);
                            });
                            cx.shared.slave_can.lock(|lck| {
                                lck.on_frame(now_ms, msg.id(), msg.dlc(), &buf);
                            })
                        }
                    }
                }
                _ => {}
            }
        }
    }

    // -- Interrupts for the TCC Solenoid          -- //
    //    These have the HIGHEST priority to keep     //
    //    the TCC solenoid waveform accurate!         //

    #[task(priority = 4, binds=TCC2_OTHER, shared=[soltcc])]
    fn tcc2_ovf(mut cx: tcc2_ovf::Context) {
        cx.shared.soltcc.lock(|lck| lck.on_tcc_ovf());
    }

    #[task(priority = 4, binds=TCC2_MC1, shared=[soltcc])]
    fn tcc2_mc0(mut cx: tcc2_mc0::Context) {
        cx.shared.soltcc.lock(|lck| lck.on_tcc_mc1());
    }

    #[task(priority = 4, binds=TCC2_MC2, shared=[soltcc])]
    fn tcc2_mc1(mut cx: tcc2_mc1::Context) {
        cx.shared.soltcc.lock(|lck| lck.on_tcc_mc2());
    }
}

#[exception(trampoline = true)]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    // 1. Get initial program counter & link register from the exception frame
    let pc = ef.pc();
    let lr = ef.lr();

    // 2. Read the current stack pointer (R13)
    let sp: usize;
    unsafe {
        core::arch::asm!("mov {}, sp", out(reg) sp);
    }

    panic!(
        "Hard fault detected!\n\
         PC : 0x{:08X}\n\
         LR : 0x{:08X}\n\
         R0 : 0x{:08X}  R1: 0x{:08X}\n\
         R2 : 0x{:08X}  R3: 0x{:08X}\n\
         SP : 0x{:08X}\n\
         Backtrace (Stack Scan):\n\
         {}",
        pc,
        lr,
        ef.r0(),
        ef.r1(),
        ef.r2(),
        ef.r3(),
        sp,
        FormatBacktrace(sp)
    );

    //panic!("Hard fault detected R0-4: [0x{:08X} 0x{:08X} 0x{:08X} 0x{:08X}]. PC: 0x{:08X}", ef.r0(), ef.r1(), ef.r2(), ef.r3(), ef.pc())
}

struct FormatBacktrace(usize);
impl core::fmt::Display for FormatBacktrace {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let sp_ptr = self.0 as *const usize;

        // Scan up to 32 words on the stack looking for code addresses in Flash region
        // (Assuming Flash starts around 0x0800_0000 or 0x0000_0000 depending on standard Cortex-M chip)
        let mut depth = 0;
        for i in 0..32 {
            let val = unsafe { sp_ptr.add(i).read_volatile() };

            // Heuristic check: Is this address in Flash memory space? (Thumb code addresses are odd / bit 0 set)
            if is_possible_code_address(val) {
                writeln!(f, "  {:2}: 0x{:08X}", depth, val)?;
                depth += 1;
            }
        }
        Ok(())
    }
}

#[inline(always)]
fn is_possible_code_address(addr: usize) -> bool {
    // Adjust memory bounds based on your target microcontroller's Flash mapping
    // Common ARM Flash address space: 0x0000_0000..0x2000_0000
    (0x0000_0000..1024 * 1024).contains(&addr) && (addr & 1 == 1)
}

#[exception]
unsafe fn BusFault() {
    unsafe {
        cortex_m::Peripherals::steal().SCB.bfar.read();
    }
}

#[exception]
unsafe fn MemoryManagement() {}
