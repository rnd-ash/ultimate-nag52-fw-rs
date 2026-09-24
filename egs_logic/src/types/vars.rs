use egs_can::{CanRxData, CanTxData};

use crate::{
    StatTimes, create_timers_storage,
    errors::DeviceMode,
    hydraulics::SLIDING_COEF_COLD,
    mechanics::GearboxSpeeds,
    shifting_algos::{ShiftType, ShiftValveFlags},
    types::{Clutch, Gear, Profile, SelectorPos, ShiftCircuit, TftReading},
};

#[derive(Copy, Clone)]
#[repr(C)]
pub struct GearboxInputs {
    pub clock_time_us: u64,
    // Board vars (Gearbox side)
    pub n2_pulses_raw: u16,
    pub n3_pulses_raw: u16,
    pub n_out_pulses_raw: u16,
    pub atf: Option<TftReading>,
    // Board vars (Vehicle side)
    pub can_vars: CanRxData,
    pub shifter_position: Option<SelectorPos>,
}

unsafe impl Send for GearboxInputs {}

impl GearboxInputs {
    pub const fn new() -> Self {
        Self {
            clock_time_us: 0,
            n2_pulses_raw: 0,
            n3_pulses_raw: 0,
            n_out_pulses_raw: 0,
            atf: None,
            can_vars: CanRxData::new(),
            shifter_position: None,
        }
    }
}

#[derive(Default, Copy, Clone, Debug)]
#[repr(C)]
pub struct GearboxOutputs {
    // Hardware outputs
    pub solenoid_pwr_en: bool,
    pub tcc_pwm: u16,
    pub mpc_current: u16,
    pub spc_current: u16,
    pub y3_en: bool,
    pub y4_en: bool,
    pub y5_en: bool,
    pub rp_en: bool,
    pub gpio_en: bool,
    // Logical outputs
    pub can_outputs: CanTxData,
}

impl GearboxOutputs {
    pub const fn new() -> Self {
        Self {
            solenoid_pwr_en: false,
            tcc_pwm: 0,
            mpc_current: 0,
            spc_current: 0,
            y3_en: false,
            y4_en: false,
            y5_en: false,
            rp_en: false,
            gpio_en: false,
            can_outputs: CanTxData::new(),
        }
    }
}

unsafe impl Send for GearboxOutputs {}

create_timers_storage!({
    shift_phase_timer,
    mod_phase_timer,
    shift_emergency_timer
});

#[repr(C)]
#[derive(Copy, Clone)]
pub struct GearboxVars {
    pub mode: DeviceMode,
    pub profile: Profile,
    pub engine_running: bool,
    pub temperature_atf: i16,
    pub mpc_pressure: u16,
    pub spc_pressure: u16,
    pub shift_valve_flags: ShiftValveFlags,
    pub last_fwd_gear: Gear,
    pub current_gear: Gear,
    pub target_gear: Gear,
    pub restarted: bool,
    pub shift_active: Option<(ShiftType, ShiftCircuit)>,

    pub timers: AllEgsTimers,

    pub p_apply_clutch: u16,
    pub max_p_apply_clutch: u16,
    pub centrifugal_p_on_clutch: u16,
    pub centrifugal_p_off_clutch: u16,
    pub clutch_on_or_apply: Option<Clutch>,
    pub clutch_release: Option<Clutch>,

    pub filling_torque: u32,
    pub abs_input_torque: u32,
    pub sliding_coefficient: u32,
    pub speeds: GearboxSpeeds,
    pub stat_times: StatTimes,

    pub master_shift_phase: u8,
    pub shift_phase_s: u8,
    pub shift_phase_m: u8,
    pub engine_rpm: u16,
    pub calc_input_rpm: Option<u16>,

    pub n2_rpm: u16,
    pub n3_rpm: u16,
    pub n_out_rpm: u16,

    pub us_clock_time: u64,
    pub us_delta_this_cycle: u32,
}

impl GearboxVars {
    pub const fn new() -> Self {
        Self {
            mode: DeviceMode::INIT,
            profile: Profile::S,
            engine_running: false,
            temperature_atf: 20,
            mpc_pressure: 0,
            spc_pressure: 0,
            shift_valve_flags: ShiftValveFlags::NONE,
            last_fwd_gear: Gear::N,
            current_gear: Gear::N,
            target_gear: Gear::N,
            restarted: true,
            shift_active: None,
            timers: AllEgsTimers::new(),
            p_apply_clutch: 0,
            max_p_apply_clutch: 0,
            centrifugal_p_on_clutch: 0,
            centrifugal_p_off_clutch: 0,
            filling_torque: 0,
            clutch_on_or_apply: None,
            clutch_release: None,
            speeds: GearboxSpeeds::new(),
            abs_input_torque: 0,
            sliding_coefficient: SLIDING_COEF_COLD as u32,
            stat_times: StatTimes::new(),

            master_shift_phase: 0,
            shift_phase_m: 0,
            shift_phase_s: 0,
            engine_rpm: 0,
            calc_input_rpm: None,

            n2_rpm: 0,
            n3_rpm: 0,
            n_out_rpm: 0,

            us_clock_time: 0,
            us_delta_this_cycle: 0,
        }
    }

    pub fn millis(&self) -> u32 {
        (self.us_clock_time / 1000) as _
    }
}

unsafe impl Send for GearboxVars {}
