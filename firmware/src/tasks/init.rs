use core::sync::atomic::AtomicU32;

use crate::can::CanLayerTy;
use crate::can::data::SignalFrame;
use crate::can::egs52::Egs52Can;
use crate::can::slave::{self, SlaveCan};
use crate::diag::KwpServer;
use crate::hal_extension::evsys;
use crate::sensors::{AdcData, SensorData};
use crate::sensors::adc::{Adc0Pins, Adc1Pins, Adc1VariableInputs};
use crate::sensors::speed_sensors::{AllSpeedSensors, IntN2RpmPc, IntN3RpmPc, init_speed_sensor};
use crate::sensors::variable_adc_input::VariableAdcInput;
use crate::solenoids::SolenoidControler;
use crate::solenoids::tcc_sol::TccSol;
use crate::solenoids::tle8242::{TLE_SPI_BAUD, Tle8242, Tle8242Pins};
use crate::storage::eeprom::Eeprom;
use crate::usb::UsbData;
use crate::{CAN_ID_DIAG_RX, CAN_ID_DIAG_TX, DmacIrqs, Mono, Sercom2Irqs, Sercom6Irqs, app, create_code_info};

use app::init::Context as InitContext;
use app::async_init::Context as AsyncInitContext;
use app::{Resources, Shared};
use atsamd_hal::can::Dependencies;
use atsamd_hal::clock::v2::{Source, clock_system_at_reset, pclk};
use atsamd_hal::clock::v2::dfll::FromUsb;
use atsamd_hal::clock::v2::dpll::Dpll;
use atsamd_hal::clock::v2::gclk::{Gclk, GclkDiv8, GclkDiv16};
use atsamd_hal::clock::v2::osculp32k::OscUlp32k;
use atsamd_hal::clock::v2::pclk::Pclk;
use atsamd_hal::clock::v2::rtcosc::RtcOsc;
use atsamd_hal::dmac::{self, DmaController, PriorityLevel};
use atsamd_hal::eic::Eic;
use atsamd_hal::fugit::{ExtU64, HertzU32, RateExtU32};
use atsamd_hal::nvm::Nvm;
use atsamd_hal::nvm::smart_eeprom::SmartEepromMode;
use atsamd_hal::prelude::_atsamd_hal_embedded_hal_digital_v2_OutputPin;
use atsamd_hal::rtic_time::Monotonic;
use atsamd_hal::serial_number;
use atsamd_hal::usb::UsbBus;
use atsamd_hal::usb::usb_device::bus::UsbBusAllocator;
use atsamd_hal::usb::usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbRev, UsbVidPid};
use atsamd_hal::watchdog::Watchdog;
use bsp::can_deps::{self, Capacities};
use cortex_m::prelude::_embedded_hal_watchdog_Watchdog;
use defmt::println;
use diag_common::isotp_endpoints::can_isotp::make_isotp_endpoint;
use diag_common::isotp_endpoints::usb_isotp::new_usb_isotp;
use diag_common::smarteeprom::{CodeSectionInfo, mutate_smarteeprom_info};
use heapless::format;
use mcan::embedded_can::{Id, StandardId};
use rtic_sync::arbiter::Arbiter;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use mcan::interrupt::Interrupt as McanInterrupt;
use mcan::filter::Filter as McanFilter;


pub fn init(cx: InitContext) -> (Shared, Resources) {
    let mut device = cx.device;
    let _core: rtic::export::Peripherals = cx.core;
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
    //     ├── GCLK 5 (32Khz)
    //     │   └── EIC
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
    //     Y       │   └── TC0..4
    //     │       └── GCLK4(160Mhz)
    //     │           └── TCC0
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
    let (_rtc, osc) = RtcOsc::enable(tokens.rtcosc, osculp32k);

    // Low speed GCLK
    let (gclk5, _osc) = Gclk::from_source(tokens.gclks.gclk5, osc);
    let gclk5_32k = gclk5.enable();

    // Start RTIC system
    Mono::start(device.rtc);

    // Grab our version info
    let mut nvm = Nvm::new(device.nvmctrl);
    if let Ok(smart_eeprom) = nvm.smart_eeprom() {
        let mut smart_eeprom = match smart_eeprom {
            SmartEepromMode::Locked(smart_eeprom) => smart_eeprom.unlock(),
            SmartEepromMode::Unlocked(smart_eeprom) => smart_eeprom,
        };
        mutate_smarteeprom_info(&mut smart_eeprom, |info| {
            const SECTION_INFO: CodeSectionInfo = create_code_info(*b"UN52PICEGS");
            if info.firmware_info != SECTION_INFO {
                info.firmware_info = SECTION_INFO
            }
        });
    }

    // Init ADCs
    let (pclk_adc0, gclk3_80) = Pclk::enable(tokens.pclks.adc0, gclk3_80);
    let (pclk_adc1, gclk3_80) = Pclk::enable(tokens.pclks.adc1, gclk3_80);
    let apb_adc0 = buses.apb.enable(tokens.apbs.adc0);
    let apb_adc1 = buses.apb.enable(tokens.apbs.adc1);
    let adc0_pins = Adc0Pins {
        tsen_tl8242: pins.tsen_tle8242.into(),
        tsen_pcb: pins.tsen_pcb.into(),
    };
    let adc1_pins = Adc1Pins {
        ac_p: pins.accel_p.into(),
        tft: pins.tft.into(),
        pmon_kl15: pins.pmon_kl15.into(),
        pmon_kl87: pins.pmon_kl87.into(),
        pmon_sens: pins.pmon_sensors.into(),
        pmon_kl87_diag: pins.pmon_kl87_diag.into(),
    };

    let adc_variable_inputs = Adc1VariableInputs {
        gpio_1: None,
        gpio_2: None,
        gpio_3: None,
        ac_m_brk: VariableAdcInput::new(pins.accel_m_or_brake, pins.scale_drp_acm),
    };

    let adc_data = AdcData::new(
        device.adc0,
        device.adc1,
        device.supc,
        adc0_pins,
        adc1_pins,
        adc_variable_inputs,
        apb_adc0,
        apb_adc1,
        pclk_adc0,
        pclk_adc1,
    );

    // Event system and RPM sensor setup
    let evsys_channels = evsys::EvSysController::new(&mut mclk, device.evsys).split();
    let (pclk_evsys, _gclk5_32k) = Pclk::enable(tokens.pclks.eic, gclk5_32k);
    let eic = Eic::new(&mut mclk, &pclk_evsys.into(), device.eic);
    //eic.switch_to_osc32k(&rtc);
    let eic_channels = eic.split();

    // DMA Init (SPI + I2C Requires this)
    let dmac = DmaController::init(device.dmac, &mut device.pm);
    let mut fut_dmac = dmac.into_future(DmacIrqs);
    let dma_channels = fut_dmac.split();
    // Init channels
    let dma_ch0 = dma_channels.0.init(PriorityLevel::Lvl0); // TLE8242 SPI
    let dma_ch1 = dma_channels.1.init(PriorityLevel::Lvl0); // TLE8242 SPI
    let dma_ch2 = dma_channels.2.init(PriorityLevel::Lvl0); // EEPROM I2C

    let (tcc01_clock, _gclk4_160) = Pclk::enable(tokens.pclks.tcc0_tcc1, gclk4_160);
    let tcc01_clock_compat = tcc01_clock.into();
    // Much better resolution to run this at 100Mhz vs 160Mhz
    let (tcc23_clock, gclk0_100) = Pclk::enable(tokens.pclks.tcc2_tcc3, gclk0_100);

    // -- TCC Solenoid init  --
    let sol_tcc = TccSol::new(
        device.tcc2,
        device.tcc3,
        &tcc23_clock.into(),
        pins.tcc_pwm.into(),
        pins.tcc_cutoff.into(),
        pins.tcc_fdbk.into(),
        eic_channels.11,
        &mut mclk,
        evsys_channels.5,
        evsys_channels.6,
        evsys_channels.7,
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
        enable: pins.tle_en.into(),
        clk: pins.tle_clk.into(),
        cs: pins.tle_cs.into(),
        led: pins.led_tle.into(),
    };

    let tle8242: Tle8242<dmac::Ch0, dmac::Ch1> = Tle8242::new(
        tle8242_pins,
        spi_fut_dma,
        &mut mclk,
        &tcc01_clock_compat,
        device.tcc0,
    );

    // -- EEPROM init -- //
    let (pclk_sercom2, gclk3_80) = Pclk::enable(tokens.pclks.sercom2, gclk3_80);
    let i2c = bsp::eeprom(
        pclk_sercom2,
        device.sercom2,
        400u32.kHz(),
        &mut mclk,
        pins.eeprom_sda,
        pins.eeprom_scl,
    )
    .into_future(Sercom2Irqs)
    .with_dma_channel(dma_ch2);

    let dsu_pac = diag_common::hal_extensions::dsu::Dsu::new(device.dsu, &device.pac).unwrap();
    let dsu = cx.local.dsu_init.insert(Arbiter::new(dsu_pac));

    let eeprom = crate::storage::eeprom::Eeprom::new(i2c, dsu);

    let solenoid_io = SolenoidControler::new(tle8242, pins.power_en_sol.into());

    // Enable sensor power supply (Testing)
    pins.power_en_sensors
        .into_push_pull_output()
        .set_high()
        .unwrap();

    // Speed sensors init
    let (pclk_tc01, gclk3_80) = Pclk::enable(tokens.pclks.tc0_tc1, gclk3_80);
    let (_pclk_tc23, _gclk3_80) = Pclk::enable(tokens.pclks.tc2_tc3, gclk3_80);

    let n2_pcnt: IntN2RpmPc = init_speed_sensor(
        pins.rpm_n2,
        eic_channels.2,
        evsys_channels.0,
        &mut mclk,
        device.tc0,
        &pclk_tc01,
    );

    let n3_pcnt: IntN3RpmPc = init_speed_sensor(
        pins.rpm_n3,
        eic_channels.3,
        evsys_channels.1,
        &mut mclk,
        device.tc1,
        &pclk_tc01,
    );

    let all_spd_sensors = AllSpeedSensors::new(n2_pcnt, n3_pcnt, None, None, None);

    // -- CAN init --
    let (clk_can, _gclk2_40) = Pclk::enable(tokens.pclks.can0, gclk2_40);
    let (can0_deps, gclk0_100) = Dependencies::new(
        gclk0_100,
        clk_can,
        clocks.ahbs.can0,
        pins.can_rx.into_mode(),
        pins.can_tx.into_mode(),
        device.can0,
    );

    let can_layer = CanLayerTy::Egs52(Egs52Can::new());

    let mut can0_cfg =
        mcan::bus::CanConfigurable::new(HertzU32::Hz(500_000), can0_deps, cx.local.message_ram)
            .unwrap();
    // Push diagnostic Rx filter ID
    can0_cfg
        .filters_standard()
        .push(McanFilter::Classic {
            action: mcan::filter::Action::StoreFifo0,
            filter: CAN_ID_DIAG_RX,
            mask: StandardId::MAX,
        })
        .ok();
    // Push slave mode Rx Frame
    can0_cfg
        .filters_standard()
        .push(McanFilter::Classic {
            action: mcan::filter::Action::StoreFifo0,
            filter: slave::SolenoidControl::CAN_ID,
            mask: StandardId::MAX,
        })
        .ok();

    for filter in can_layer.filters() {
        if can0_cfg
        .filters_standard()
        .push(McanFilter::Classic {
            action: mcan::filter::Action::StoreFifo0,
            filter: filter.id,
            mask: filter.mask,
        }).is_err() {
            panic!("Could not allocate CAN Filter for (ID: 0x{:04X}, MSK: 0x{:04X})", filter.id.as_raw(), filter.mask.as_raw());
        }
    }

    // Enable new MSG interrupt for FIFO0
    let interrupts = can0_cfg
        .interrupts()
        .split([McanInterrupt::RxFifo0NewMessage, McanInterrupt::MessageStoredToDedicatedRxBuffer].iter().copied().collect())
        .unwrap();
    let line0_interrupts = can0_cfg.interrupt_configuration().enable_line_0(interrupts);
    let can = can0_cfg.finalize().unwrap();

    let arbiter_cantx: &'static _ = cx.local.arbiter_cantx.insert(Arbiter::new(can.tx));
    let (isotp_isr, isotp_thread) = make_isotp_endpoint(
        Id::Standard(CAN_ID_DIAG_TX),
        Id::Standard(CAN_ID_DIAG_RX),
        Some(can_deps::CAN_TX_MAILBOX_DIAG),
        arbiter_cantx,
        cx.local.isotp_can_fc_signal,
        cx.local.isotp_msg_signal_can,
    );

    // Init USB
    let (usb_clock, _gclk2_48) = Pclk::enable(tokens.pclks.usb, gclk6_48);
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

    app::async_init::spawn(arbiter_cantx, eeprom, solenoid_io)
        .unwrap_or_else(|_| panic!("Could not start async init"));
    app::perf_monitor::spawn(gclk0_100.freq().raw()).unwrap();

    wdt.feed();
    (
        Shared {
            usb_data,
            wdt,
            can_layer,
            slave_can: SlaveCan::new(),
            soltcc: sol_tcc,
            sensor_data: SensorData::default(),
            cpu_idle_ticks: AtomicU32::new(0),
            hw_interrupts: AtomicU32::new(0),
            dsu
        },
        Resources {
            adc_data,
            speed_sensors: all_spd_sensors,
            can0_fifo0: can.rx_fifo_0,
            can0_dedicated: can.rx_dedicated_buffers,
            can0_interrupts: line0_interrupts,
            isotp_isr,
            isotp_thread,
            usb_isotp_thread: isotp_usb_thread,

            diag_server: KwpServer::new(),
        },
    )
}


pub async fn async_init(
    _ctx: AsyncInitContext<'_>,
    can_tx: &'static Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>,
    mut eeprom: Eeprom<dmac::Ch2>,
    mut solenoid_io: SolenoidControler<dmac::Ch0, dmac::Ch1>,
) {
    if let Some(info) = diag_common::ram_info::get_bootloader_comm_info() {
        println!("BL INFO COUNTER: {}", info.reset_counter);
    } else {
        defmt::error!("BL INFO CORRUPT");
        let addr = 0x20010000 as *const u8;
        let arr = unsafe { core::ptr::slice_from_raw_parts(addr, 512).as_ref().unwrap() };
        println!("{:02X}", arr);
    }
    app::sensor_query::spawn().unwrap();
    eeprom.init().await;
    solenoid_io.init().await;
    app::gearbox_task::spawn(can_tx, solenoid_io)
        .unwrap_or_else(|_| panic!("Could not start async init"));
    app::diag_task::spawn().unwrap();
    // Wait 5 seconds - Most likely a crash will happen whilst all the async tasks
    // are initializing
    Mono::delay(5000u64.millis()).await;
    // Now reset the reset counter
    diag_common::ram_info::modify_bootloader_info(|info| {
        info.reset_counter = 0;
    });
}