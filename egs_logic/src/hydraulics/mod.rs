use defmt::println;
use egs_maths::{first_order_filter, interp_linear};

use crate::{
    Clutch, Gear, ShiftCircuit,
    calbrations::{hydr::HydrCal, mech::MechCal},
};

const COEF_STATIC: f32 = 50.0;

pub struct HydraulicMgr<'a> {
    pub hydr_cal: &'a HydrCal,
    pub temperature_c_offset_50: u16,
    pub current_gear: Gear,
    pub engine_rpm: u16,
    pub shift_state: Option<ShiftCircuit>,
}

impl<'a> HydraulicMgr<'a> {
    pub fn calculate_solenoid_current(&self, p_targ: u32, p_mod: u32) -> u32 {
        let factor: u32;
        let extra_p: u32;

        if let Some(shift) = self.shift_state {
            let interp_max = if matches!(shift, ShiftCircuit::_12 | ShiftCircuit::_21) {
                factor = self.hydr_cal.p_multi_1 as u32;
                self.hydr_cal.extra_pressure_adder_r1_1
            } else {
                factor = self.hydr_cal.p_multi_other as u32;
                self.hydr_cal.extra_pressure_adder_other_gears
            };
            extra_p = interp_linear(
                self.engine_rpm,
                self.hydr_cal.extra_pressure_pump_speed_min,
                self.hydr_cal.extra_pressure_pump_speed_max,
                0,
                interp_max,
            ) as u32;
        } else {
            // No shift is active
            factor = if matches!(self.current_gear, Gear::_1 | Gear::_R1) {
                self.hydr_cal.p_multi_1 as u32
            } else {
                self.hydr_cal.p_multi_other as u32
            };
            extra_p = 0;
        }
        // Now calculate
        let line_p = (self.hydr_cal.lp_reg_spring_pressure as u32 + p_mod) * 1000;
        let working_p = (extra_p + (line_p / factor)) as u16;
        let inlet_p = interp_linear(
            working_p,
            self.hydr_cal.inlet_pressure_input_min,
            self.hydr_cal.inlet_pressure_input_max,
            self.hydr_cal.inlet_pressure_output_min,
            self.hydr_cal.inlet_pressure_output_max,
        ) as u32;

        let output_p = if p_targ < inlet_p {
            // Target pressure falls within the max theoretical inlet pressure,
            // Compensate the output pressure based on inlet
            let p_inc_inlet = p_targ as f32 + self.hydr_cal.inlet_pressure_offset as f32;
            let mut inlet_percent_adder = (self.hydr_cal.inlet_pressure_offset as u32
                + (self.hydr_cal.inlet_pressure_output_max as u32 - inlet_p))
                as f32
                / 1000.0;
            inlet_percent_adder *= p_inc_inlet;
            inlet_percent_adder /= 1000.0;
            (p_targ as u16 + inlet_percent_adder as u16).min(self.hydr_cal.max_pcs_pressure())
        } else {
            // Target pressure exceeds the solenoid inlet pressure,
            // no choise but to just force the max solenoid pressure
            self.hydr_cal.max_pcs_pressure()
        };
        // Now calculate solenoid output
        self.hydr_cal
            .pcs_map()
            .interp(output_p, self.temperature_c_offset_50) as u32
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
    let mut output_p = if matches!(gear, Gear::N | Gear::P) {
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
                return 0;
            }
        };
        let clutch_coef = m_cal.friction_map().get_at(clutch, gear) as f32;
        let mut clutch_p = (clutch_coef * torque).abs() / COEF_STATIC;
        // Add force required to hold the release spring
        clutch_p += m_cal.release_spring()[clutch] as f32;
        // Add force required by hydralic calibration
        clutch_p += h_cal.extra_p_not_shifting as f32;
        // Multiplier of total reqired pressure depending on if feedback of B1 is active or not
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
