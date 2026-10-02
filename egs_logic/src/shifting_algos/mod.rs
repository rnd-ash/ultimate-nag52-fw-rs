use crate::{
    Gearbox, GearboxAdaptStorage, GearboxCalibStorage, GearboxDtcStorage, GearboxMapStorage,
    hydraulics::{calc_sliding_friction_val, calculate_mpc_in_gear, clutch_centrifugal_pressure},
};
use bitflags::bitflags;
use egs_maths::egs_timers::EgsCountDownTimer;

bitflags! {
    #[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
    pub struct ShiftValveFlags: u8 {
        const NONE  = 0b0000;
        const SS_1245 = 0b0001;
        const SS_23 = 0b0010;
        const SS_34 = 0b0100;
    }
}

impl<
    'a,
    DS: GearboxDtcStorage,
    CS: GearboxCalibStorage,
    MS: GearboxMapStorage,
    AS: GearboxAdaptStorage,
> Gearbox<'a, DS, CS, MS, AS>
{
    pub fn fill_clutch_vals(&mut self) {
        let speeds = self.speeds;
        let tft = self.temperature_atf;
        self.centrifugal_p_on_clutch = self
            .clutch_on_or_apply
            .map(|clutch| {
                clutch_centrifugal_pressure(tft, clutch, &speeds, self.cal_storage.mech_cal())
            })
            .unwrap_or_default();
        self.centrifugal_p_off_clutch = self
            .clutch_release
            .map(|clutch| {
                clutch_centrifugal_pressure(tft, clutch, &speeds, self.cal_storage.mech_cal())
            })
            .unwrap_or_default();
        self.filling_torque = core::cmp::max(self.abs_input_torque, 30);
        self.sliding_coefficient =
            calc_sliding_friction_val(self.temperature_atf, self.shift_active);
        self.max_p_apply_clutch = if let Some((_, circuit)) = self.shift_active {
            self.cal_storage.hydr_cal().max_p_apply_clutch(circuit)
        } else {
            0
        };
    }

    pub fn reset_shift_phases(&mut self) {
        self.master_shift_phase = 0;
        self.shift_phase_s = 0;
        self.shift_phase_m = 0;
    }

    pub fn calculate_solenoid_status(&mut self) {
        if self.dev_mode().has_error() {
            self.mpc_pressure = 0;
            self.spc_pressure = 0;
        } else if !self.engine_running {
            self.mpc_pressure = 750;
            self.spc_pressure = 0;
        } else {
            self.fill_clutch_vals();
            match self.shift_active {
                None => {
                    // Stationary in gear
                    self.shift_valve_flags = ShiftValveFlags::NONE;
                    self.reset_shift_phases();

                    let working_mpc = calculate_mpc_in_gear(
                        self.abs_input_torque as f32,
                        self.current_gear,
                        self.mpc_pressure,
                        &*self.cal_storage.hydr_cal(),
                        &self.cal_storage.mech_cal(),
                    );
                    self.mpc_pressure = self
                        .timers
                        .shift_phase_timer
                        .interp_value(self.mpc_pressure, working_mpc)
                        as u16;
                    self.p_apply_clutch = self
                        .timers
                        .shift_phase_timer
                        .interp_value(self.p_apply_clutch, self.max_p_apply_clutch)
                        as _;
                    self.spc_pressure = self.timers.shift_phase_timer.interp_value(
                        self.spc_pressure,
                        self.cal_storage.hydr_cal().max_pcs_pressure(),
                    ) as u16;
                }
                Some(_) => todo!(),
            }
        }
        // Memory saving for old variables (Shadowed for delta tracking)
        // -> TODO
        // Finally, output the shift valves intended current
        if self.dev_mode().has_error() {
            self.outputs.spc_current = 0;
            self.outputs.mpc_current = 0;
            self.outputs.y3_en = false;
            self.outputs.y4_en = false;
            self.outputs.y5_en = false;
        } else {
            self.outputs.spc_current =
                self.calculate_solenoid_current(self.spc_pressure as u32) as u16;
            self.outputs.mpc_current =
                self.calculate_solenoid_current(self.mpc_pressure as u32) as u16;
            self.outputs.y3_en = self.shift_valve_flags.contains(ShiftValveFlags::SS_1245);
            self.outputs.y4_en = self.shift_valve_flags.contains(ShiftValveFlags::SS_34);
            self.outputs.y5_en = self.shift_valve_flags.contains(ShiftValveFlags::SS_23);
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct ShiftSolenoidStatus {
    pub p_spc: u16,
    pub p_mpc: u16,
    pub shift_valves: ShiftValveFlags,
}

impl ShiftSolenoidStatus {
    pub const fn new(p_spc: u16, p_mpc: u16, shift_valves: ShiftValveFlags) -> Self {
        Self {
            p_mpc,
            p_spc,
            shift_valves,
        }
    }
}

#[derive(Copy, Clone)]
pub enum ShiftType {
    CrossoverUp,
    ReleaseUp,
    CrossoverDn,
    ReleaseDn,
    ToN,
    ToRorD,
}

pub struct ShiftAlgorithmData {
    pub timer_shift: EgsCountDownTimer,
    pub timer_mod: EgsCountDownTimer,
    pub timer_emergency: EgsCountDownTimer,

    pub subphase_mod: u8,
    pub subphase_shift: u8,

    pub momentum: i32,
    pub momentum_filtered: i32,

    pub target_turbine_speed: u32,

    pub overlap_shift_p: u32,
    pub max_p_apply_clutch: u32,
    pub p_apply_clutch: u32,
}

/// Clutch information, used by all shift algorithms
pub struct ClutchReplacementData {
    /// Speed in RPM that the clutch is rotating at
    speed: u16,
    /// Friction coefficient of the clutch
    friction_coefficient: u16,
    /// Current centrifugal force of the clutch in mBar
    centrifugal_force: u16,
    /// Return spring pressure
    spring_p: u16,
}
