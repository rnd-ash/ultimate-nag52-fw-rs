//! Hydraulic valve body processor

use egs_maths::interp_linear;

use crate::{calbrations::HydrCal, gearbox_control::{ControlState, Gear, ShiftCircuit}};


#[derive(Copy, Clone)]
pub enum PressureCtrlState {
    InGear(Gear),
    CircuitActive(ShiftCircuit)
}

pub fn calculate_pcs_solenoid_current(cal: &HydrCal, state: PressureCtrlState, p_targ: u16, p_mod: u16, engine_rpm: u16) -> u16 {
    let (factor, extra_p) = match state {
        PressureCtrlState::InGear(gear) => {
            if gear == Gear::_1 || gear == Gear::_R1 {
                (cal.p_multi_1, 0u16)
            } else {
                (cal.p_multi_other, 0u16)
            }
        },
        PressureCtrlState::CircuitActive(shift_circuit) => {
            let (f, extra_p_max) = if shift_circuit.is_1_2_circuit() {
                (cal.p_multi_1, cal.extra_pressure_adder_r1_1)
            } else {
                (cal.p_multi_other, cal.extra_pressure_adder_other_gears)
            };
            (f, interp_linear(engine_rpm, cal.extra_pressure_pump_speed_min, cal.extra_pressure_pump_speed_max, 0, extra_p_max) as u16)
        },
    };
    let line_pressure = (cal.lp_reg_spring_pressure + p_mod) as i32 * 1000;
    let working_pressure = extra_p + (line_pressure / factor as i32) as u16;
    let inlet_pressure = interp_linear(working_pressure, cal.inlet_pressure_input_min, cal.inlet_pressure_input_max, cal.inlet_pressure_output_min, cal.inlet_pressure_output_max) as u16;

    let mut inlet_factor = cal.shift_pressure_addr_percent as i32 * (cal.inlet_pressure_output_max.saturating_sub(inlet_pressure)) as i32;
    inlet_factor /= 1000;
    
    if p_targ < inlet_pressure {
        let compensated = p_targ as i32 + cal.inlet_pressure_offset as i32;
        inlet_factor *= compensated;
        inlet_factor /= 1000;
        (p_targ + inlet_factor as u16).min(cal.max_pcs_pressure())
    } else {
        cal.max_pcs_pressure()
    }
}