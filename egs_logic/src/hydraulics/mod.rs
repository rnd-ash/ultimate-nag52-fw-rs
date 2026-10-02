use egs_maths::{first_order_filter, interp_linear};

use crate::{
    Clutch, Gear, Gearbox, GearboxAdaptStorage, GearboxCalibStorage, GearboxDtcStorage,
    GearboxMapStorage, ShiftCircuit,
    calbrations::{hydr::HydrCal, mech::MechCal},
    mechanics::GearboxSpeeds,
    shifting_algos::ShiftType,
};

const COEF_STATIC: f32 = 100.0;

impl<
    'a,
    DS: GearboxDtcStorage,
    CS: GearboxCalibStorage,
    MS: GearboxMapStorage,
    AS: GearboxAdaptStorage,
> Gearbox<'a, DS, CS, MS, AS>
{
    pub fn calculate_solenoid_current(&self, p_target: u32) -> u32 {
        let factor: u32;
        let extra_p: u32;
        let hydr_cal = self.cal_storage.hydr_cal();
        if let Some((_, active_circuit)) = self.vars.shift_active {
            let interp_max = if matches!(active_circuit, ShiftCircuit::_12 | ShiftCircuit::_21) {
                factor = hydr_cal.p_multi_1 as u32;
                hydr_cal.extra_pressure_adder_r1_1
            } else {
                factor = hydr_cal.p_multi_other as u32;
                hydr_cal.extra_pressure_adder_other_gears
            };
            extra_p = interp_linear(
                self.vars.engine_rpm,
                hydr_cal.extra_pressure_pump_speed_min,
                hydr_cal.extra_pressure_pump_speed_max,
                0,
                interp_max,
            ) as u32;
        } else {
            // No shift is active
            factor = if matches!(self.vars.current_gear, Gear::_1 | Gear::_R1) {
                hydr_cal.p_multi_1 as u32
            } else {
                hydr_cal.p_multi_other as u32
            };
            extra_p = 0;
        }
        // Now calculate
        let line_p =
            (hydr_cal.lp_reg_spring_pressure as u32 + self.vars.mpc_pressure as u32) * 1000;

        let working_p = (extra_p + (line_p / factor)) as u16;
        let inlet_p = interp_linear(
            working_p,
            hydr_cal.inlet_pressure_input_min,
            hydr_cal.inlet_pressure_input_max,
            hydr_cal.inlet_pressure_output_min,
            hydr_cal.inlet_pressure_output_max,
        ) as u32;

        let output_p = if p_target < inlet_p {
            // Target pressure falls within the max theoretical inlet pressure,
            // Compensate the output pressure based on inlet
            let p_inc_inlet = p_target as f32 + hydr_cal.inlet_pressure_offset as f32;
            let mut inlet_percent_adder = (hydr_cal.inlet_pressure_offset as u32
                + (hydr_cal.inlet_pressure_output_max as u32 - inlet_p))
                as f32
                / 1000.0;
            inlet_percent_adder *= p_inc_inlet;
            inlet_percent_adder /= 1000.0;
            (p_target as u16 + inlet_percent_adder as u16).min(hydr_cal.max_pcs_pressure())
        } else {
            // Target pressure exceeds the solenoid inlet pressure,
            // no choice but to just force the max solenoid pressure
            hydr_cal.max_pcs_pressure()
        };
        // Now calculate solenoid output
        hydr_cal
            .pcs_map()
            .interp(output_p, self.vars.temperature_atf + 50) as u32
    }
}

/// Returns the working pressure required for the current gear
pub fn calculate_mpc_in_gear(
    torque: f32,
    gear: Gear,
    prev_mpc: u16,
    h_cal: &HydrCal,
    m_cal: &MechCal,
) -> u16 {
    let mut output_p = if matches!(gear, Gear::N) {
        // P and N always require 0 working pressure
        0
    } else {
        // For working gears
        let weakest_clutch = m_cal.weakest_clutch()[gear];
        // Todo move this to some other code in case of repeating?
        let clutch = match weakest_clutch {
            0 => Clutch::K1,
            1 => Clutch::K2,
            2 => Clutch::K3,
            3 => Clutch::B1,
            4 => Clutch::B2,
            5 => Clutch::B3,
            _ => {
                defmt::error!(
                    "Weakest clutch array returned invalid clutch IDX {}",
                    &weakest_clutch
                );
                return h_cal.max_pcs_pressure();
            }
        };
        let clutch_coef = m_cal.friction_map().get_at(clutch, gear) as f32;
        let mut clutch_p = (clutch_coef * torque).abs() / COEF_STATIC;
        // Add force required to hold the release spring
        clutch_p += m_cal.release_spring()[clutch] as f32;
        // Add force required by hydraulics calibration
        clutch_p += h_cal.extra_p_not_shifting as f32;
        // Multiplier of total required pressure depending on if feedback of B1 is active or not
        clutch_p *= if matches!(gear, Gear::_1 | Gear::_R1) {
            h_cal.p_multi_1 as f32 / 1000.0
        } else {
            h_cal.p_multi_other as f32 / 1000.0
        };
        // Subtract the pressure provided already by the line pressure spring,
        // and then clamp it to the maximum pressure the solenoids can actually output
        let clutch_p_int = (clutch_p as u16)
            .saturating_sub(h_cal.lp_reg_spring_pressure)
            .min(h_cal.max_pcs_pressure());
        clutch_p_int
    };
    output_p = output_p.max(h_cal.min_mpc_pressure);
    if output_p < prev_mpc {
        // Filter when dropping
        output_p = first_order_filter(output_p, prev_mpc, h_cal.filter_factor) as _;
    }
    output_p
}

pub fn clutch_centrifugal_pressure(
    atf_temp: i16,
    clutch: Clutch,
    speeds: &GearboxSpeeds,
    m_cal: &MechCal,
) -> u16 {
    let (speed, factor) = match clutch {
        Clutch::K2 => (
            speeds.calc_turbine,
            m_cal.atf_density_centrifugal_force_factor[1] as f32,
        ),
        Clutch::K3 => (
            speeds.calc_rear_sun,
            m_cal.atf_density_centrifugal_force_factor[2] as f32,
        ),
        _ => return 0,
    };
    let delta_t = atf_temp + 50; // Base temperature is -50C
    let drop = (m_cal.atf_density_drop_per_c as f32 * delta_t as f32) / 100.0;
    let density = m_cal.atf_density_minus_50c as f32 - drop;
    let r = (((speed * speed) as f32 / 1000.0) * density) / factor;
    (r / 10.0) as _
}

/// Helper function - Calculate the maximum torque for a clutch
/// based on its friction value, and friction coefficient
#[inline(always)]
pub fn max_trq_for_clutch(clutch_val: u32, friction_coef: u32, pressure: u32) -> f32 {
    (pressure * friction_coef) as f32 / clutch_val as f32
}

/// Helper function - Calculate the required pressure for a clutch
/// based on its friction value, and friction coefficient
#[inline(always)]
pub fn pressure_for_clutch(clutch_val: u32, friction_coef: u32, torque_nm: f32) -> f32 {
    (clutch_val as f32 * torque_nm).abs() / friction_coef as f32
}

pub const SLIDING_COEF_COLD: u16 = 185;
const SLIDING_COEF_HOT: u16 = 140;
const SLIDING_COEF_COLD_T: u16 = 29;
const SLIDING_COEF_HOT_T: u16 = 65;

pub fn calc_sliding_friction_val(tft_c: i16, shift: Option<(ShiftType, ShiftCircuit)>) -> u32 {
    if let Some((ShiftType::CrossoverUp, ShiftCircuit::_12)) = shift {
        SLIDING_COEF_HOT as u32
    } else {
        interp_linear(
            tft_c,
            SLIDING_COEF_COLD_T,
            SLIDING_COEF_HOT_T,
            SLIDING_COEF_COLD,
            SLIDING_COEF_HOT,
        ) as u32
    }
}
