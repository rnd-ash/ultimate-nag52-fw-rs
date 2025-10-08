use atsamd_hal::{
    dmac, ehal::digital::OutputPin, fugit::ExtU64, rtic_time::Monotonic, time::Hertz,
};
use bsp::SolPwrEn;
use defmt::println;
use rtic_sync::signal::SignalWriter;

use crate::{
    solenoids::{
        commands::{CtrlMethodFaultMaskCfg, ShortToBatThreshold, TleChannel},
        solenoid_ctrl::{DitherSettings, Mode},
        tle8242::{ChannelProps, Tle8242, TleConfiguration, R_SENSE_VAL, TLE8242_CLK_FREQ},
    },
    Mono,
};

pub mod commands;
pub mod solenoid_ctrl;
pub mod tcc_sol;
pub mod tle8242;

const TLE_CHAN_Y3: TleChannel = TleChannel::_5;
const TLE_CHAN_Y4: TleChannel = TleChannel::_4;
const TLE_CHAN_Y5: TleChannel = TleChannel::_1;

const TLE_CHAN_MPC: TleChannel = TleChannel::_3;
const TLE_CHAN_SPC: TleChannel = TleChannel::_0;

const TLE_CHAN_GPIO: TleChannel = TleChannel::_6;
const TLE_CHAN_TRRS: TleChannel = TleChannel::_2;

#[bitbybit::bitfield(u8)]
pub struct SolenoidOnOffReq {
    #[bit(0)]
    pub y3: bool,
    #[bit(1)]
    pub y4: bool,
    #[bit(2)]
    pub y5: bool,
    #[bit(3)]
    pub trrs: bool,
}

pub struct SolenoidOutputReq {
    pub mpc_current: u16,
    pub spc_current: u16,
    pub gpio_current: u16,
    pub on_off_valves: SolenoidOnOffReq,
}

#[derive(Default)]
pub struct MonitoredOutputs {
    pub spc_current: u16,
    pub mpc_current: u16,
    pub y3_current: u16,
    pub y4_current: u16,
    pub y5_current: u16,
    pub gpio_current: u16,
    pub trrs_current: u16,
}

pub struct SolenoidControler<T: dmac::ChId, R: dmac::ChId> {
    tle8242: Tle8242<T, R>,
    pin_sol_pwr_en: SolPwrEn,
    last_mpc: u16,
    last_spc: u16,
    all_channels: [TleChannel; 7],
    monitored_currents: MonitoredOutputs,
    last_read_current_channel: usize,
}

impl<T: dmac::ChId, R: dmac::ChId> SolenoidControler<T, R> {
    pub fn new(tle8242: Tle8242<T, R>, pin_sol_pwr_en: SolPwrEn) -> Self {
        Self {
            tle8242,
            pin_sol_pwr_en,
            last_mpc: 0,
            last_spc: 0,
            all_channels: [
                TLE_CHAN_Y3,
                TLE_CHAN_Y4,
                TLE_CHAN_Y5,
                TLE_CHAN_MPC,
                TLE_CHAN_SPC,
                TLE_CHAN_TRRS,
                TLE_CHAN_GPIO,
            ],
            monitored_currents: MonitoredOutputs::default(),
            last_read_current_channel: 0,
        }
    }

    pub async fn init(&mut self) {
        self.pin_sol_pwr_en.set_high().unwrap();

        // Configuration for linear pressure solenoids
        let mode_linear = Mode::calculated_cc_mode(
            5.5,
            R_SENSE_VAL,
            0.029,
            14.40,
            Hertz::Hz(1000u32),
            TLE8242_CLK_FREQ,
            0.707,
            8.0,
            //None
            Some(DitherSettings::new(Hertz::Hz(250), 100)),
        )
        .unwrap_or_else(|_| panic!(""));

        let linear_props = ChannelProps {
            mode: mode_linear,
            fault_enabled: false,
            short_to_batt_threshold: ShortToBatThreshold::_1_3V,
            short_to_batt_retry_time: 0,
            pwm_offset: 0,
            sample_method: false,
        };

        // Configuration for shift solenoids (Direct PWM driven)
        let mode_shift = Mode::pwm(
            Hertz::Hz(1000u32),
            TLE8242_CLK_FREQ,
            commands::DividerM::_512Or128,
        );

        let shift_props = ChannelProps {
            mode: mode_shift,
            fault_enabled: false,
            short_to_batt_threshold: ShortToBatThreshold::_1_3V,
            short_to_batt_retry_time: 0,
            pwm_offset: 0,
            sample_method: false,
        };

        let cfg = TleConfiguration::default()
            .with_props(TLE_CHAN_MPC, linear_props)
            .with_props(TLE_CHAN_SPC, linear_props)
            .with_props(TLE_CHAN_Y3, shift_props)
            .with_props(TLE_CHAN_Y4, shift_props)
            .with_props(TLE_CHAN_Y5, shift_props);
        self.tle8242.init(cfg).await;
    }

    pub async fn update() {}

    pub async fn set_spc_current(&mut self, setpoint_ma: u16) {
        if self.last_spc != setpoint_ma {
            let setpoint_val = setpoint_ma as f32 / (320.0 / R_SENSE_VAL) * 2048.0;
            self.tle8242
                .set_channel_current(TLE_CHAN_SPC, setpoint_val as u16)
                .await;
            self.last_spc = setpoint_ma;
        }
    }

    pub async fn set_mpc_current(&mut self, setpoint_ma: u16) {
        if self.last_mpc != setpoint_ma {
            let setpoint_val = setpoint_ma as f32 / (320.0 / R_SENSE_VAL) * 2048.0;
            self.tle8242
                .set_channel_current(TLE_CHAN_MPC, setpoint_val as u16)
                .await;
            self.last_mpc = setpoint_ma;
        }
    }

    pub async fn set_y3_pwm(&mut self, duty: f32) {
        self.tle8242.set_channel_pwm(TLE_CHAN_Y3, duty).await;
    }

    pub async fn set_y4_pwm(&mut self, duty: f32) {
        self.tle8242.set_channel_pwm(TLE_CHAN_Y4, duty).await;
    }

    pub async fn set_y5_pwm(&mut self, duty: f32) {
        self.tle8242.set_channel_pwm(TLE_CHAN_Y5, duty).await;
    }

    /// Task is responsible for updating all solenoids on demand,
    /// and monitoring current consumption
    pub async fn update_task(&mut self) {}

    pub async fn update_current_readings(&mut self) {
        // Try to read the current channel
        while let Some(avg) = self
            .tle8242
            .get_avg_current(self.all_channels[self.last_read_current_channel])
            .await
        {
            // Valid response
            // Parse to mA
            let milliamps = match self.all_channels[self.last_read_current_channel] {
                TLE_CHAN_MPC | TLE_CHAN_SPC => {
                    // With dither
                    ((avg as f32 / 32768.0) * (320.0 / R_SENSE_VAL)) as u16
                }
                TLE_CHAN_Y3 | TLE_CHAN_Y4 | TLE_CHAN_Y5 => {
                    // Without dither
                    ((avg as f32 / 8192.0) * (320.0 / R_SENSE_VAL)) as u16
                }
                _ => 0,
            };
            //self.channel_currents[self.last_read_current_channel] = milliamps;
            // Cycle to the next channel
            let next_id = (self.last_read_current_channel + 1) % 7;
            self.last_read_current_channel = next_id;
        }
        // No response since valid bit is not set - Exit loop
    }

    pub fn read_spc_current(&mut self) -> u16 {
        self.monitored_currents.spc_current
    }

    pub fn read_mpc_current(&mut self) -> u16 {
        self.monitored_currents.mpc_current
    }
}
