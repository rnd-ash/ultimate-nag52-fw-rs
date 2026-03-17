#![no_std]

use atsamd_hal::{
    clock::v2::{gclk::GclkId, pclk::Pclk, types::Usb},
    pac::{self, Mclk},
    qspi::{self, Qspi},
    sercom::{
        i2c::{self},
        spi, Sercom2, Sercom6,
    },
    time::Hertz,
    usb::{self, UsbBus},
};

pub mod can_deps;

atsamd_hal::bsp_peripherals!(
    Sercom0 { KlineSercom }
    Sercom2 { EepromSercom }
    Sercom6 { TleSercom }
);

atsamd_hal::bsp_pins!(
    // -- PORT A -- //
    PA00 {
        name: start_en
        aliases: {
            PushPullOutput: StartEn
        }
    },
    PA01 {
        name: start_en_diag
        aliases: {
            PushPullOutput: StartEnDiag
        }
    },
    // PA02 - NC
    // PA03 - NC
    PA04 {
        name: kline_rx
        aliases: {
            AlternateD: KlineRx
        }
    },
    PA05 {
        name: kline_tx
        aliases: {
            AlternateD: KlineTx
        }
    }
    PA06 {
        name: tsen_tle8242
        aliases: {
            AlternateB: TsenTle8242
        }
    }
    PA07 {
        name: tsen_pcb
        aliases: {
            AlternateB: TsenPcb
        }
    }
    PA08 {
        name: extflash_data0
        aliases: {
            AlternateH: ExtFlashD0
        }
    },
    PA09 {
        name: extflash_data1
        aliases: {
            AlternateH: ExtFlashD1
        }
    },
    PA10 {
        name: extflash_data2
        aliases: {
            AlternateH: ExtFlashD2
        }
    },
    PA11 {
        name: extflash_data3
        aliases: {
            AlternateH: ExtFlashD3
        }
    },
    PA12 {
        name: eeprom_sda
        aliases: {
            AlternateC: EepromSDA
        }
    },
    PA13 {
        name: eeprom_scl
        aliases: {
            AlternateC: EepromSCL
        }
    },
    // PA14 - NC
    // PA15 - NC
    // PA16 - NC
    // PA17 - NC
    // PA18 - NC
    // PA19 - NC
    PA20 {
        name: power_en_sol
        aliases: {
            PushPullOutput: PowerEnSol,
        }
    },
    PA21 {
        name: power_en_sensors
        aliases: {
            PushPullOutput: PowerEnSen
        }
    },
    PA22 {
        name: can_tx
        aliases: {
            AlternateI: CanTx
        }
    },
    PA23 {
        name: can_rx
        aliases: {
            AlternateI: CanRx
        }
    },
    PA24 {
        name: usb_dm
        aliases: {
            AlternateH: UsbDm
        }
    },
    PA25 {
        name: usb_dp
        aliases: {
            AlternateH: UsbDp
        }
    },
    // PA26 - NC
    // PA27 - NC
    // PA30 - DBG
    // PA31 - DBG

    // -- PORT B -- //
    PB00 {
        name: trss_3
        aliases: {
            PushPullOutput: Trss3,
        }
    },
    PB01 {
        name: trss_2
        aliases: {
            PushPullOutput: Trss2,
        }
    },
    PB02 {
        name: trss_1
        aliases: {
            PushPullOutput: Trss1,
        }
    },
    PB03 {
        name: trss_0
        aliases: {
            PushPullOutput: Trss0,
        }
    },
    PB04 {
        name: tft
        aliases: {
            AlternateB: Tft
        }
    },
    PB05 {
        name: gpio_sig_1
        aliases: {
            PushPullOutput: GpioRpm1
            AlternateB: GpioAnalog1
        }
    },
    PB06 {
        name: accel_m_or_brake
        aliases: {
            AlternateB: AccelMOrBrake
        }
    },
    PB07 {
        name: accel_p
        aliases: {
            AlternateB: AccelP
        }
    },
    PB08 {
        name: pmon_kl15
        aliases: {
            AlternateB: PmonKl15
        }
    },
    PB09 {
        name: pmon_kl87
        aliases: {
            AlternateB: PmonKl87
        }
    },
    PB10 {
        name: extflash_sck
        aliases: {
            AlternateH: ExtFlashSck
        }
    },
    PB11 {
        name: extflash_cs
        aliases: {
            AlternateH: ExtFlashCs
        }
    },
    PB12 {
        name: spkr
        aliases: {
            AlternateG: Spkr
        }
    },
    // PB13 - NC
    // PB14 - NC
    // PB15 - NC
    PB16 {
        name: tcc_pwm
        aliases: {
            AlternateF: TccPwm
        }
    },
    PB17 {
        name: tcc_cutoff
        aliases: {
            PushPullOutput: TccCutoff
        }
    },
    PB18 {
        name: rpm_n2
        aliases: {
            PushPullOutput: RpmN2
        }
    },
    PB19 {
        name: rpm_n3
        aliases: {
            PushPullOutput: RpmN3
        }
    },
    // PB20 - NC
    // PB21 - NC
    // PB22 - NC
    // PB23 - NC
    // PB24 - NC
    // PB25 - NC
    // PB26 - NC
    // PB27 - NC
    // PB28 - NC
    // PB29 - NC
    // PB30 - DBG
    // PB31 - DBG

    // -- PORT C -- //
    PC00 {
        name: trrs_prg
        aliases: {
            PushPullOutput: TrrsPrg
        }
    },
    PC01 {
        name: kickdown
        aliases: {
            PushPullOutput: Kickdown
        }
    },
    PC02 {
        name: pmon_sensors
        aliases: {
            AlternateB: PmonSensors
        }
    },
    PC03 {
        name: pmon_kl87_diag
        aliases: {
            AlternateB: PmonKl87Diag
        }
    },
    PC04 {
        name: scale_drp_gpio1
        aliases: {
            PushPullOutput: ScaleDrpGpio1
        }
    },
    PC05 {
        name: scale_drp_gpio2
        aliases: {
            PushPullOutput: ScaleDrpGpio2
        }
    },
    PC06 {
        name: scale_drp_gpio3
        aliases: {
            PushPullOutput: ScaleDrpGpio3
        }
    },
    PC07 {
        name: scale_drp_acm
        aliases: {
            PushPullOutput: ScaleDrpAcm
        }
    },

    PC10 {
        name: led_stat_ok
        aliases: {
            PushPullOutput: LedStatOk,
        }
    },
    // PC11 - NC
    // PC12 - NC
    // PC13 - NC
    // PC14 - NC
    // PC15 - NC
    PC16 {
        name: tle_cs
        aliases: {
            PushPullOutput: TleCs
        }
    },
    PC17 {
        name: tle_sck
        aliases: {
            AlternateC: TleSck
        }
    },
    PC18 {
        name: tle_so
        aliases: {
            AlternateC: TleSo
        }
    },
    PC19 {
        name: tle_si
        aliases: {
            AlternateC: TleSi
        }
    },
    PC20 {
        name: tle_fault
        aliases: {
            PushPullOutput: TleFault
        }
    },
    PC21 {
        name: tle_clk
        aliases: {
            AlternateF: TleClk
        }
    },
    // PC21 - NC
    PC22 {
        name: tle_reset,
        aliases: {
            PushPullOutput: TleReset
        }
    },
    PC23 {
        name: tle_phase_sync
        aliases: {
            PushPullOutput: TlePhaseSync
        }
    },
    // PC24 - NC
    // PC25 - NC
    // PC26 - NC
    // PC27 - NC
    // PC28 - NC
    // PC29 - NC
    // PC30 - NC
    // PC31 - NC

    // -- PORT D -- //
    PD00 {
        name: gpio_sig_2
        aliases: {
            PushPullOutput: GpioRpm2
            AlternateB: GpioAnalog2
        }
    },
    PD01 {
        name: gpio_sig_3
        aliases: {
            PushPullOutput: GpioRpm3
            AlternateB: GpioAnalog3
        }
    },

    PD08 {
        name: led_eeprom
        aliases: {
            PushPullOutput: LedEeprom
        }
    },
    PD09 {
        name: led_usb
        aliases: {
            PushPullOutput: LedUsb
        }
    },
    PD10 {
        name: led_qspi
        aliases: {
            PushPullOutput: LedQspi
        }
    },
    PD11 {
        name: led_tle
        aliases: {
            PushPullOutput: LedTle
        }
    },
    PD12 {
        name: led_stat_err
        aliases: {
            PushPullOutput: LedStatErr,
            AlternateF: LedStatErrPwm
        }
    },

    PD20 {
        name: tle_en
        aliases: {
            PushPullOutput: TleEn
        }
    }
    PD21 {
        name: tcc_fdbk
        aliases: {
            PushPullOutput: TccFdbk
        }
    }
);

pub fn ext_flash(
    mclk: &mut Mclk,
    qspi: pac::Qspi,
    sck: impl Into<ExtFlashSck>,
    cs: impl Into<ExtFlashCs>,
    io0: impl Into<ExtFlashD0>,
    io1: impl Into<ExtFlashD1>,
    io2: impl Into<ExtFlashD2>,
    io3: impl Into<ExtFlashD3>,
) -> Qspi<qspi::OneShot> {
    let qspi = qspi::Qspi::new(
        mclk,
        qspi,
        sck.into(),
        cs.into(),
        io0.into(),
        io1.into(),
        io2.into(),
        io3.into(),
    );
    qspi
}

pub type EepromPads = i2c::Pads<Sercom2, EepromSDA, EepromSCL>;
pub type EepromI2c = i2c::I2c<i2c::Config<EepromPads>>;

pub fn eeprom<Gclk: GclkId>(
    pclk_sercom2: Pclk<Sercom2, Gclk>,
    sercom: EepromSercom,
    baud: Hertz,
    mclk: &mut Mclk,
    sda: impl Into<EepromSDA>,
    scl: impl Into<EepromSCL>,
) -> EepromI2c {
    let pads = i2c::Pads::new(sda.into(), scl.into());
    i2c::Config::new(mclk, sercom, pads, pclk_sercom2.freq())
        .baud(baud)
        .enable()
}

pub type TleSpiPads = spi::Pads<TleSercom, TleSo, TleSi, TleSck>;
pub type TleSpi = spi::Spi<spi::Config<TleSpiPads>, spi::Duplex>;

pub fn tle_spi<Gclk: GclkId>(
    pclk_sercom6: Pclk<Sercom6, Gclk>,
    baud: Hertz,
    sercom: TleSercom,
    mclk: &mut pac::Mclk,
    sck: impl Into<TleSck>,
    si: impl Into<TleSi>,
    so: impl Into<TleSo>,
) -> TleSpi {
    let mut so: TleSo = so.into();
    so.set_drive_strength(false);
    let mut si: TleSi = si.into();
    si.set_drive_strength(false);
    let mut sck: TleSck = sck.into();
    sck.set_drive_strength(false);

    let pads = spi::Pads::default().data_in(so).data_out(si).sclk(sck);
    spi::Config::new(mclk, sercom, pads, pclk_sercom6.freq())
        .baud(baud)
        .spi_mode(spi::MODE_0)
        .bit_order(spi::BitOrder::MsbFirst)
        .enable()
}

pub fn native_usb<Gclk: GclkId>(
    clock: Pclk<Usb, Gclk>,
    usb: pac::Usb,
    mclk: &mut Mclk,
    dp: impl Into<UsbDp>,
    dm: impl Into<UsbDm>,
) -> UsbBus {
    usb::UsbBus::new(&clock.into(), mclk, dm.into(), dp.into(), usb)
}
