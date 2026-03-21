use atsamd_hal::{
    dmac::{self, ChId},
    ehal::digital::OutputPin,
    rtic_time::Monotonic,
    time::Hertz,
};
use bsp::PowerEnSol;

use crate::{
    solenoids::{
        commands::{ShortToBatThreshold, TleChannel},
        solenoid_ctrl::{DitherSettings, Mode},
        tcc_sol::TccSol,
        tle8242::{ChannelProps, Tle8242, TleConfiguration, R_SENSE_VAL, TLE8242_CLK_FREQ},
    },
    Mono,
};

pub mod commands;
pub mod solenoid_ctrl;
pub mod tcc_sol;
pub mod tle8242;

const TLE_CHAN_Y3: TleChannel = TleChannel::_6;
const TLE_CHAN_Y4: TleChannel = TleChannel::_1;
const TLE_CHAN_Y5: TleChannel = TleChannel::_4;

const TLE_CHAN_MPC: TleChannel = TleChannel::_5;
const TLE_CHAN_SPC: TleChannel = TleChannel::_0;

const TLE_CHAN_GPIO: TleChannel = TleChannel::_3;
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

#[derive(Default, Copy, Clone, PartialEq, PartialOrd)]
pub enum ShiftValveState {
    #[default]
    Off,
    FullOn(u64),
    HoldOn,
}

impl ShiftValveState {
    pub fn is_on(&self) -> bool {
        !matches!(self, Self::Off)
    }
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

impl MonitoredOutputs {
    pub fn set_current(&mut self, chan: TleChannel, val: u16) {
        match chan {
            x if x == TLE_CHAN_MPC => self.mpc_current = val,
            x if x == TLE_CHAN_SPC => self.spc_current = val,
            x if x == TLE_CHAN_Y3 => self.y3_current = val,
            x if x == TLE_CHAN_Y4 => self.y4_current = val,
            x if x == TLE_CHAN_Y5 => self.y5_current = val,
            x if x == TLE_CHAN_GPIO => self.gpio_current = val,
            x if x == TLE_CHAN_TRRS => self.trrs_current = val,
            _ => {}
        }
    }
}

pub struct SolenoidControler<T: dmac::ChId, R: dmac::ChId> {
    tle8242: Tle8242<T, R>,
    pin_sol_pwr_en: PowerEnSol,
    last_mpc: u16,
    last_spc: u16,
    all_channels: [TleChannel; 7],
    monitored_currents: MonitoredOutputs,
    last_read_current_channel: usize,
    // Control of the shift valves
    y3_state: ShiftValveState,
    y4_state: ShiftValveState,
    y5_state: ShiftValveState,
}

async fn set_binary_valve<I: ChId, O: ChId>(
    tle8242: &mut Tle8242<I, O>,
    en: bool,
    chan: TleChannel,
    ss_state: &mut ShiftValveState,
) {
    if en && !ss_state.is_on() {
        tle8242.set_channel_pwm(chan, 1.0).await;
        *ss_state = ShiftValveState::FullOn(Mono::now().duration_since_epoch().to_millis());
    } else if !en && ss_state.is_on() {
        tle8242.set_channel_pwm(chan, 0.0).await;
        *ss_state = ShiftValveState::Off;
    }
}

macro_rules! make_binary_valve {
    ($name:ident, $chan:expr, $field:ident) => {
        pub async fn $name(&mut self, en: bool) {
            let tle8242 = &mut self.tle8242;
            let ss_state = &mut self.$field;
            set_binary_valve(tle8242, en, $chan, ss_state).await;
        }
    };
}

impl<T: dmac::ChId, R: dmac::ChId> SolenoidControler<T, R> {
    pub fn new(tle8242: Tle8242<T, R>, pin_sol_pwr_en: PowerEnSol) -> Self {
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
            y3_state: Default::default(),
            y4_state: Default::default(),
            y5_state: Default::default(),
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

    pub fn set_tcc_pwm(&mut self, duty: u16, sol_tcc: &mut TccSol) {
        sol_tcc.write_tcc_sol(duty);
    }

    pub fn get_observed_tcc_pwm(&self, sol_tcc: &mut TccSol) -> u16 {
        sol_tcc.get_observed_pwm()
    }

    /// Task is responsible for updating all solenoids on demand,
    /// and monitoring current consumption
    pub async fn update_task(&mut self) {
        let millis = Mono::now().duration_since_epoch().to_millis();
        // macro to easily update binary valve states
        macro_rules! update_binary_valve {
            ($chan:ident, $field:ident, $max_on_time: literal) => {
                if let ShiftValveState::FullOn(on_ts) = self.$field {
                    if millis - on_ts > $max_on_time {
                        // Reduce PWM
                        self.tle8242.set_channel_pwm($chan, 0.25).await;
                        self.$field = ShiftValveState::HoldOn;
                    }
                }
            };
        }

        // Update the valves if the PWM should be lowered
        update_binary_valve!(TLE_CHAN_Y3, y3_state, 250);
        update_binary_valve!(TLE_CHAN_Y4, y4_state, 250);
        update_binary_valve!(TLE_CHAN_Y5, y5_state, 250);

        self.update_current_readings().await;
    }

    pub fn is_channel_on(&self, channel: TleChannel) -> bool {
        match channel {
            TLE_CHAN_MPC => self.last_mpc != 0,
            TLE_CHAN_SPC => self.last_spc != 0,
            TLE_CHAN_Y3 => self.y3_state != ShiftValveState::Off,
            TLE_CHAN_Y4 => self.y4_state != ShiftValveState::Off,
            TLE_CHAN_Y5 => self.y5_state != ShiftValveState::Off,
            TLE_CHAN_TRRS => false, // TODO
            TLE_CHAN_GPIO => false, // TODO
            _ => false,
        }
    }

    pub async fn update_current_readings(&mut self) {
        let avg_maybe = if !self.is_channel_on(self.all_channels[self.last_read_current_channel]) {
            Some(0)
        } else {
            self.tle8242
                .get_avg_current(self.all_channels[self.last_read_current_channel])
                .await
        };

        // Try to read the current channel
        if let Some(avg) = avg_maybe {
            // Valid response
            // Parse to mA
            let milliamps = match self.all_channels[self.last_read_current_channel] {
                // TODO GPIO and TRRS channels
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
            self.monitored_currents
                .set_current(self.all_channels[self.last_read_current_channel], milliamps);
            // Cycle to the next channel
            let next_id = (self.last_read_current_channel + 1) % 7;
            self.last_read_current_channel = next_id;
        }
        // No response since valid bit is not set - Exit loop
    }

    pub fn read_spc_current(&self) -> u16 {
        self.monitored_currents.spc_current
    }

    pub fn read_mpc_current(&self) -> u16 {
        self.monitored_currents.mpc_current
    }

    pub fn read_y3_current(&self) -> u16 {
        self.monitored_currents.y3_current
    }

    pub fn read_y4_current(&self) -> u16 {
        self.monitored_currents.y4_current
    }

    pub fn read_y5_current(&self) -> u16 {
        self.monitored_currents.y5_current
    }

    // Binary valve manipulations from macro

    make_binary_valve!(set_y3, TLE_CHAN_Y3, y3_state);
    make_binary_valve!(set_y4, TLE_CHAN_Y4, y4_state);
    make_binary_valve!(set_y5, TLE_CHAN_Y5, y5_state);
}
