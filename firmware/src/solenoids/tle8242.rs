use core::f32::consts::PI;

use arbitrary_int::{traits::Integer, u11, u12, u14, u19, u5};
use atsamd_hal::{
    bind_multiple_interrupts,
    clock::Tcc0Tcc1Clock,
    dmac,
    ehal::digital::OutputPin,
    ehal_async::spi::SpiBus,
    fugit::ExtU64,
    gpio::{AlternateG, PA20},
    pac::{Mclk, Peripherals, Tcc0},
    prelude::_embedded_hal_Pwm,
    pwm::{Channel, TCC0Pinout, Tcc0Pwm},
    rtic_time::Monotonic,
    sercom::{
        dma,
        spi::{self, SpiFutureDuplex, SpiFutureDuplexDma},
        Sercom6,
    },
    time::Hertz,
};
use bsp::{LedTleAct, TleClk, TleCs, TleEnable, TleFault, TlePhaseSync, TleReset, TleSpiPads};
use cortex_m::asm::{self, nop};
use defmt::println;
use num_traits::Pow;

use crate::{
    solenoids::{
        commands::{
            AutoZeroTriggerRead, AverageCurrentRead, ControlMode, ControlVarsSet,
            CtrlMethodFaultMaskCfg, CurrentDitherAmpSet, DiagnosticTimer, DitherPeriodSet,
            DividerM, IcVersion, MainPeriodSet, PwmDutyCycle, ShortToBatThreshold, TleChannel,
            TleMsg,
        },
        solenoid_ctrl::Mode,
    },
    Mono,
};

pub struct Tle8242Pins {
    pub phase_sync: TlePhaseSync,
    pub fault: TleFault,
    pub reset: TleReset,
    pub enable: TleEnable,
    pub clk: TleClk,
    pub cs: TleCs,
    pub led: LedTleAct,
}

pub const TLE8242_CLK_FREQ: Hertz = Hertz::MHz(40);
const TCC0_CHANNEL_TLE: Channel = Channel::_0; // TCC0 WO[0] as per the datasheet

// T4 (Sck rise to rise time) is 100ns (10Mhz)
pub const TLE_SPI_BAUD: Hertz = Hertz::MHz(10);

// Values for Rsense
pub const R_SENSE_VAL: f32 = 0.05; // Ohms
const R_SENSE_EQ_VAL: f32 = 0.320 / 0.05;

#[derive(Copy, Clone)]
pub struct ChannelProps {
    pub mode: Mode,
    pub fault_enabled: bool,
    pub short_to_batt_threshold: ShortToBatThreshold,
    pub short_to_batt_retry_time: u8,
    pub pwm_offset: u8,
    // SPI msg 8 values
    pub sample_method: bool,
}

#[derive(Copy, Clone, Default)]
pub struct TleConfiguration {
    channel_settings: [Option<(TleChannel, ChannelProps)>; 8],
    fault_mask_reset: bool,
    fault_mask_enable: bool,
    diag_tmr: DiagnosticTimer,
}

impl TleConfiguration {
    pub fn with_props(mut self, channel: TleChannel, props: ChannelProps) -> Self {
        self.channel_settings[channel as usize] = Some((channel, props));
        self
    }
}

pub struct Tle8242<T: dmac::ChId, R: dmac::ChId> {
    spi: SpiFutureDuplexDma<spi::Config<TleSpiPads>, T, R>,
    pwm: Tcc0Pwm<PA20, AlternateG>,
    pin_phase_sync: TlePhaseSync,
    pin_fault: TleFault,
    pin_reset: TleReset,
    pin_enable: TleEnable,
    pin_cs: TleCs,
    pin_led: LedTleAct,
    config: TleConfiguration,
}

impl<T: dmac::ChId, R: dmac::ChId> Tle8242<T, R> {
    pub fn new(
        pins: Tle8242Pins,
        spi: SpiFutureDuplexDma<spi::Config<TleSpiPads>, T, R>,
        mclk: &mut Mclk,
        clock: &Tcc0Tcc1Clock,
        tcc0: Tcc0,
    ) -> Self {
        let pinout = TCC0Pinout::Pa20(pins.clk);
        let mut tcc = Tcc0Pwm::new(clock, TLE8242_CLK_FREQ, tcc0, pinout, mclk);
        let midpoint = tcc.get_max_duty() / 2;
        tcc.set_duty(TCC0_CHANNEL_TLE, midpoint);
        tcc.enable(TCC0_CHANNEL_TLE);
        Self {
            spi,
            pwm: tcc,
            pin_phase_sync: pins.phase_sync,
            pin_fault: pins.fault,
            pin_reset: pins.reset,
            pin_enable: pins.enable,
            pin_cs: pins.cs,
            pin_led: pins.led,
            config: Default::default(),
        }
    }

    pub async fn init(&mut self, settings: TleConfiguration) {
        self.pin_cs.set_high().unwrap();
        self.pin_reset.set_low().unwrap();
        self.pin_enable.set_low().unwrap();
        Mono::delay(1u64.millis()).await;
        self.pin_reset.set_high().unwrap();
        // Enable IC
        self.pin_enable.set_high().unwrap();
        // Issue a phase sync pulse
        self.pin_phase_sync.set_high().unwrap();
        Mono::delay(1u64.millis()).await;
        self.pin_phase_sync.set_low().unwrap();

        // Sanity check - Get version  info
        let tle_ver = IcVersion::new_with_id();
        if self.xfer(tle_ver).await.is_none() {
            // TODO - Panic or stop init if this happens (TLE is not responding)
        }

        // Now prepare to configure the channels
        let mut cfg_fault_msg = CtrlMethodFaultMaskCfg::new_with_id()
            .with_write(true)
            .with_diag_timer(settings.diag_tmr)
            .with_fme(settings.fault_mask_enable)
            .with_fmr(settings.fault_mask_reset);
        for (tle_chan, channel) in settings.channel_settings.iter().filter_map(|x| *x) {
            let idx = tle_chan as usize;
            // Configure the A/D sampler
            let mps_msg = MainPeriodSet::new_with_id()
                .with_write(true)
                .with_channel_id(tle_chan)
                .with_divider_m(channel.mode.div_m())
                .with_divider_n(u14::from_u16(channel.mode.div_n()));
            self.xfer(mps_msg).await;
            // Kp, Ki and dither config for constant current mode channels
            if let Mode::ConstantCurrent {
                kp,
                ki,
                dither_opts,
                ..
            } = channel.mode
            {
                let kpki_msg = ControlVarsSet::new_with_id()
                    .with_write(true)
                    .with_channel_id(tle_chan)
                    .with_ki(u12::from_u16(ki))
                    .with_kp(u12::from_u16(kp));
                self.xfer(kpki_msg).await;
                if let Some(dither_opts) = dither_opts {
                    let dither_period_msg = DitherPeriodSet::new_with_id()
                        .with_channel_id(tle_chan)
                        .with_write(true)
                        .with_number_of_steps(u5::from_u8(dither_opts.steps));
                    self.xfer(dither_period_msg).await;
                }
            }

            // Trigger autoZero
            let msg = AutoZeroTriggerRead::new_with_id()
                .with_write(true) // Write to perform autoZero
                .with_channel_id(tle_chan);
            self.xfer(msg).await;

            // Set the fault configuration. This is the last msg sent
            // Note its 7-x since the channels are backwards (Higher bit = lower channel)
            cfg_fault_msg.set_fmx(7 - idx, channel.fault_enabled);
            cfg_fault_msg.set_cmx(7 - idx, channel.mode.to_ctrl_mode());
        }
        // Send our fault configuration message
        self.xfer(cfg_fault_msg).await;
        // Wait needed for autoZero to work
        Mono::delay(1u64.micros()).await;

        self.config = settings;
    }

    // TODO - Error out if invalid option
    pub async fn set_channel_current(&mut self, channel: TleChannel, setpoint_val: u16) {
        if let Some((_, props)) = self.config.channel_settings[channel as usize] {
            if let Mode::ConstantCurrent { dither_opts, .. } = props.mode {
                let mut req_msg = CurrentDitherAmpSet::new_with_id()
                    .with_write(true)
                    .with_channel_id(channel)
                    .with_current_setpoint(u11::from_u16(setpoint_val));
                if let Some(dither_opts) = dither_opts {
                    req_msg = req_msg.with_dither_step_size(u11::from_u16(dither_opts.step_size))
                }
                self.xfer(req_msg).await;
            }
        }
    }

    // TODO - Error out if invalid option
    /// * percentage - Value from 0.0 to 1.0 representing PWM duty
    pub async fn set_channel_pwm(&mut self, channel: TleChannel, percentage: f32) {
        if let Some((_, props)) = self.config.channel_settings[channel as usize] {
            if let Mode::Pwm { divm: _, divn } = props.mode {
                let duty = percentage * (32.0 * divn as f32);
                let req_msg = PwmDutyCycle::new_with_id()
                    .with_write(true)
                    .with_channel_id(channel)
                    .with_pwm(u19::from_u32(duty as u32));
                self.xfer(req_msg).await;
            }
        }
    }

    /// Returns current usage in Milliamps
    pub async fn get_avg_current(&mut self, channel: TleChannel) -> Option<u32> {
        let msg = AverageCurrentRead::new_with_id().with_channel_id(channel);
        let response = self.xfer(msg).await?;
        if response.valid() {
            Some(response.avg().as_u32())
        } else {
            None
        }
    }

    pub async fn xfer<M: TleMsg>(&mut self, msg: M) -> Option<M> {
        // Note. Timing params (T1, T2, T3) delays are ignored
        //       the CPU is not fast enough to exceed these limits
        self.pin_led.set_high().unwrap();
        self.pin_cs.set_low().unwrap();
        let buf = msg.into().to_be_bytes();
        let mut read_buf = [0xFF; 4];
        // Write command
        self.spi.transfer(&mut read_buf, &buf).await.ok()?;
        self.pin_cs.set_high().unwrap();
        if msg.is_read() {
            self.pin_cs.set_low().unwrap();
            self.spi
                .transfer(&mut read_buf, &0u32.to_be_bytes())
                .await
                .ok()?;
            self.pin_cs.set_high().unwrap();
            self.pin_led.set_low().unwrap();
            let resp_u32 = u32::from_be_bytes(read_buf);
            if msg.id_match(resp_u32) {
                Some(u32::from_be_bytes(read_buf).into())
            } else {
                defmt::error!("CMD ID Mismatch");
                None
            }
        } else {
            self.pin_led.set_low().unwrap();
            None
        }
    }
}
