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
use core::panic::PanicInfo;
use core::sync::atomic::AtomicU32;
use cortex_m_rt::exception;
//use defmt_rtt as _;
use diag_common::hal_extensions::dsu::Dsu;
use diag_common::parse_git_sha;
use diag_common::parse_u8;
use diag_common::smarteeprom::CodeSectionInfo;
use diag_common::{dyn_panic::AppPanicInfo, ram_info::modify_bootloader_info};
use mcan::embedded_can::StandardId;
use rtic_sync::portable_atomic::AtomicU16;

use crate::diag::dev_mode::EgsDeviceMode;

pub mod can;
pub mod diag;
pub mod hal_extension;
pub mod ram_test;
pub mod sensors;
pub mod solenoids;
pub mod storage;
pub mod tasks;
pub mod usb;
pub mod gearbox_control;
pub mod calbrations;

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

#[rtic::app(device = atsamd_hal::pac, dispatchers = [DAC_EMPTY_0, DAC_EMPTY_1, EVSYS_0, EVSYS_1])]
mod app {

    use crate::{
        can::{CanLayer, CanLayerTy, slave::SlaveCan},
        diag::KwpServer,
        sensors::{AdcData, SensorData, speed_sensors::AllSpeedSensors},
        solenoids::{SolenoidControler, tcc_sol::TccSol},
        storage::eeprom::Eeprom,
        usb::UsbData,
    };
    use atsamd_hal::{
        clock::v2::{pclk, types::Can0},
        dmac::{self},
        usb::{UsbBus, usb_device::bus::UsbBusAllocator},
        watchdog::Watchdog,
    };
    use bsp::can_deps::{Capacities, RxDedicated, RxFifo0};
    use diag_common::{
        hal_extensions::dsu::Dsu,
        isotp_endpoints::{
            SharedIsoTpBuf,
            can_isotp::{IsoTpInterruptHandler, IsotpConsumer, IsotpCtsMsg},
            usb_isotp::UsbIsoTpConsumer,
        },
    };

    use mcan::{
        interrupt::{Interrupt, OwnedInterruptSet, state::EnabledLine0},
        message::Raw,
        messageram::SharedMemory,
        rx_dedicated_buffers::DynRxDedicatedBuffer,
    };

    use rtic_sync::{arbiter::Arbiter, signal::Signal};
    use usbd_serial::{DefaultBufferStore, SerialPort};

    use super::*;

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
    }

    #[shared]
    pub struct Shared {
        #[lock_free]
        pub usb_data: UsbData<'static>,
        pub wdt: Watchdog,
        pub can_layer: CanLayerTy,

        pub slave_can: SlaveCan,
        pub soltcc: TccSol,
        pub sensor_data: SensorData,

        pub cpu_idle_ticks: AtomicU32,
        pub hw_interrupts: AtomicU32,
        pub wakeups: AtomicU32,
        pub dsu: &'static Arbiter<diag_common::hal_extensions::dsu::Dsu>,
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
        dsu_init: Option<Arbiter<diag_common::hal_extensions::dsu::Dsu>> = None
    ])]
    fn init(cx: init::Context) -> (Shared, Resources) {
        tasks::init(cx)
    }

    #[task(priority = 1)]
    async fn async_init(
        ctx: async_init::Context,
        dsu: &'static Arbiter<Dsu>,
        can_tx: &'static Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>,
        eeprom: Eeprom<dmac::Ch2>,
        solenoid_io: SolenoidControler<dmac::Ch0, dmac::Ch1>,
    ) {
        tasks::async_init(ctx, dsu, can_tx, eeprom, solenoid_io).await;
    }

    #[idle(shared=[&cpu_idle_ticks, &hw_interrupts, &wakeups, &dsu])]
    fn idle(ctx: idle::Context) -> ! {
        tasks::idle(&ctx)
    }

    #[task(priority = 2, shared=[wdt, &cpu_idle_ticks, &hw_interrupts, &wakeups])]
    async fn perf_monitor(ctx: perf_monitor::Context, tps: u32) {
        tasks::performance_monitor(ctx, tps).await;
    }

    #[task(priority = 2, local = [usb_isotp_thread, isotp_thread, diag_server])]
    async fn diag_task(cx: diag_task::Context) {
        tasks::diag_task(cx).await;
    }

    #[task(priority = 1, local=[adc_data, speed_sensors], shared=[sensor_data])]
    async fn sensor_query(cx: sensor_query::Context) {
        tasks::sensor_query(cx).await;
    }

    #[task(priority = 2, shared=[can_layer, slave_can, soltcc, sensor_data])]
    async fn gearbox_task(
        cx: gearbox_task::Context,
        can_tx: &'static Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>,
        solenoid_controller: SolenoidControler<dmac::Ch0, dmac::Ch1>,
    ) {
        tasks::gearbox_task(cx, can_tx, solenoid_controller).await;
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

    #[task(priority = 1, binds=CAN0, local=[can0_interrupts, can0_fifo0, can0_dedicated, isotp_isr, buf: [u8; 8] = [0; 8]], shared=[can_layer, slave_can])]
    #[unsafe(link_section = ".data.can0")]
    fn can0(mut cx: can0::Context) {
        let buf = cx.local.buf;
        for interrupt in cx.local.can0_interrupts.iter_flagged() {
            match interrupt {
                Interrupt::MessageStoredToDedicatedRxBuffer => {
                    while let Ok(msg) = cx.local.can0_dedicated.receive_any() {
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
                Interrupt::RxFifo0NewMessage => {
                    for msg in &mut cx.local.can0_fifo0 {
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

    // -- Interrupts for the TCC Solenoid          -- //
    //    These have the HIGHEST priority to keep     //
    //    the TCC solenoid waveform accurate!         //

    #[task(priority = 3, binds=TCC2_OTHER, shared=[soltcc])]
    fn tcc2_ovf(mut cx: tcc2_ovf::Context) {
        cx.shared.soltcc.lock(|lck| lck.on_tcc_ovf());
    }

    #[task(priority = 3, binds=TCC2_MC1, shared=[soltcc])]
    fn tcc2_mc0(mut cx: tcc2_mc0::Context) {
        cx.shared.soltcc.lock(|lck| lck.on_tcc_mc1());
    }

    #[task(priority = 3, binds=TCC2_MC2, shared=[soltcc])]
    fn tcc2_mc1(mut cx: tcc2_mc1::Context) {
        cx.shared.soltcc.lock(|lck| lck.on_tcc_mc2());
    }
}

// For Device mode operation
static DEVICE_MODE: AtomicU16 = AtomicU16::new(EgsDeviceMode::INITIALIZATION.bits());

#[exception(trampoline = false)]
unsafe fn HardFault() -> ! {
    panic!("Hard fault detected")
}

#[exception]
unsafe fn BusFault() {
    unsafe {
        cortex_m::Peripherals::steal().SCB.bfar.read();
    }
}

#[exception]
unsafe fn MemoryManagement() {}
