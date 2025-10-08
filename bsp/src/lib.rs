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
    // PA00 - NC
    // PA01 - NC
    PA02 {
        name: tft
        aliases: {
            AlternateB: Tft
        }
    },
    PA03 {
        name: sol_pwr_sense,
        aliases: {
            AlternateB: SolPwrSense
        }
    },
    PA04 {
        name: lin_rx
        aliases: {
            AlternateD: LinRx
        }
    },
    PA05 {
        name: lin_tx
        aliases: {
            AlternateD: LinTx
        }
    }
    PA06 {
        name: rpm_2
        aliases: {
            AlternateE: Rpm2
        }
    }
    // PA07 - NC
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
    PA14 {
        name: rpm_n3
        aliases: {
            PushPullOutput: RpmN3
        }
    },
    // PA15 - NC
    PA16 {
        name: rpm_n2
        aliases: {
            PushPullOutput: RpmN2
        }
    },
    // PA17 - NC
    // PA18 - NC
    PA19 {
        name: tle_enable
        aliases: {
            PushPullOutput: TleEnable
        }
    },
    PA20 {
        name: tle_clk
        aliases: {
            PushPullOutput: TleClk,
            AlternateG: TleClkG
        }
    },
    PA21 {
        name: sol_pwr_en
        aliases: {
            PushPullOutput: SolPwrEn
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
    PA27 {
        name: prg_btn_sense
        aliases: {
            PushPullOutput: PrgBtnSense
        }
    },
    // PA30 - DBG
    PA31 {
        // Special pin (Usually debug)
        // but the bootloader uses it to check
        // if it should stay in loader mode
        name: bootloader_check
    }
    // -- PORT B -- //
    // PB00 - NC
    // PB01 - NC
    // PB02 - NC
    // PB03 - NC
    PB04 {
        name: vsol_sense
        aliases: {
            AlternateB: VsolSense
        }
    },
    // PB05 - NC
    // PB06 - NC
    // PB07 - NC
    PB08 {
        name: rpm_1
        aliases: {
            PushPullOutput: Rpm1
        }
    },
    // PB09 - NC
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
    // PB12 - NC
    PB13 {
        name: spkr
        aliases: {
            AlternateG: Spkr
        }
    },
    PB14 {
        name: rpm_3
        aliases: {
            AlternateG: Rpm3
        }
    },
    // PB15 - NC
    PB16 {
        name: tcc_cutoff
        aliases: {
            PushPullOutput: TccCutoff
        }
    },
    PB17 {
        name: tcc_fdbk
        aliases: {
            PushPullOutput: TccFdbk
        }
    },
    PB18 {
        name: tcc_pwm
        aliases: {
            AlternateF: TccPwm
        }
    },
    // PB19 - NC
    // PB20 - NC
    // PB21 - NC
    // PB22 - NC
    // PB23 - NC
    // PB24 - NC
    // PB25 - NC
    // PB26 - NC
    // PB27 - NC
    PB28 {
        name: sensor_pwr_en
        aliases: {
            PushPullOutput: SensorPwrEn
        }
    },
    PB29 {
        name: start_ctrl
        aliases: {
            PushPullOutput: StartCtrl
        }
    },
    // PB30 - DBG
    // PB31 - DBG
    // -- PORT C -- //
    // PC00 - NC
    // PC01 - NC
    PC02 {
        name: accel_p_sense
        aliases: {
            AlternateB: AccelPSense
        }
    },
    PC03 {
        name: accel_m_sense
        aliases: {
            AlternateB: AccelMSense
        }
    },
    PC04 {
        name: lin_slp
        aliases: {
            PushPullOutput: LinSlp
        }
    },
    // PC05 - NC
    // PC06 - NC
    // PC07 - NC
    // PC10 - NC
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
    PC24 {
        name: brake_sense
        aliases: {
            PushPullOutput: BrakeSense
        }
    },
    PC25 {
        name: trss_a_sense
        aliases: {
            PushPullOutput: TrrsASense
        }
    },
    PC26 {
        name: trss_b_sense
        aliases: {
            PushPullOutput: TrrsBSense
        }
    },
    PC27 {
        name: trss_c_sense
        aliases: {
            PushPullOutput: TrrsCSense
        }
    },
    PC28 {
        name: trss_d_sense
        aliases: {
            PushPullOutput: TrrsDSense
        }
    },
    PC30 {
        name: vbatt_sense
        aliases: {
            AlternateB: VBattSense
        }
    },
    PC31 {
        name: vsensor_sense
        aliases: {
            AlternateB: VSensorSense
        }
    }
    // -- PORT D -- //
    // PD00 - NC
    // PD01 - NC
    PD08 {
        name: led_status
        aliases: {
            PushPullOutput: LedStatus
            AlternateF: PulsingStatus
        }
    },
    PD09 {
        name: led_usb
        aliases: {
            PushPullOutput: LedUsb
        }
    },
    PD10 {
        name: led_eeprom
        aliases: {
            PushPullOutput: LedEeprom
        }
    },
    PD11 {
        name: led_ext_flash
        aliases: {
            PushPullOutput: LedExtFlash
        }
    },
    PD12 {
        name: led_tle_act
        aliases: {
            PushPullOutput: LedTleAct
        }
    },
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

pub fn eeprom(
    sercom: EepromSercom,
    baud: Hertz,
    mclk: &mut Mclk,
    sda: impl Into<EepromSDA>,
    scl: impl Into<EepromSCL>,
) -> EepromI2c {
    let pads = i2c::Pads::new(sda.into(), scl.into());
    i2c::Config::new(mclk, sercom, pads, baud).enable()
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
