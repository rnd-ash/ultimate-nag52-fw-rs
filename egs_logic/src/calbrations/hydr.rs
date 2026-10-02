use egs_maths::{declare_2d_map, maps::Map2d};

use crate::{ShiftCircuit, calbrations::ShiftIndexedArray};

#[derive(Copy, Clone, Default)]
#[repr(align(4))]
pub struct HydrCal {
    pub p_multi_1: u16,
    pub p_multi_other: u16,
    pub lp_reg_spring_pressure: u16,
    pub overlap_circuit_factor_spc: ShiftIndexedArray<u16>,
    pub overlap_circuit_factor_mpc: ShiftIndexedArray<u16>,
    pub overlap_circuit_spring_pressure: ShiftIndexedArray<i16>,
    pub shift_reg_spring_pressure: u16,
    pub shift_spc_gain: ShiftIndexedArray<u16>,
    pub min_mpc_pressure: u16,
    pub filter_factor: u8,
    pub mpc_flush_temp_threshold: u8,
    pub mpc_no_flush_time: u16,
    pub mpc_flush_time: u16,
    pub extra_p_not_shifting: u16,
    pub shift_pressure_addr_percent: u16,
    pub inlet_pressure_offset: u16,
    pub inlet_pressure_input_min: u16,
    pub inlet_pressure_input_max: u16,
    pub inlet_pressure_output_min: u16,
    pub inlet_pressure_output_max: u16,
    pub extra_pressure_pump_speed_min: u16,
    pub extra_pressure_pump_speed_max: u16,
    pub extra_pressure_adder_r1_1: u16,
    pub extra_pressure_adder_other_gears: u16,
    pub shift_pressure_factor_percent: u16,
    pub pcs_map_x: [u16; 7],
    pub pcs_map_y: [u16; 4],
    pub pcs_map_z: [u16; 28],
}

const _: () = assert!(size_of::<HydrCal>() < 256);

declare_2d_map!(PcsMap, 7, 4, u16, u16, u16);

impl HydrCal {
    pub fn is_valid(&self) -> bool {
        self.p_multi_1 != 0
            && self.p_multi_other != 0
            && self.lp_reg_spring_pressure != 0
            && self.shift_reg_spring_pressure != 0
            && self.min_mpc_pressure != 0
            && self.pcs_map_x.iter().find(|x| **x > 10_000).is_none()
            && self.pcs_map_y.iter().find(|x| **x > 200).is_none()
            && self.pcs_map_z.iter().find(|x| **x > 3_000).is_none()
    }

    pub const fn pcs_map<'a>(&'a self) -> PcsMap<'a> {
        Map2d::new(&self.pcs_map_x, &self.pcs_map_y, &self.pcs_map_z)
    }

    pub const fn max_pcs_pressure(&self) -> u16 {
        self.pcs_map_x[6]
    }

    /// Returns the maximum hydraulic pressure that is possible to reach the clutch
    /// being manipulated by the SPC Solenoid, taking into account the resistance of the
    /// shift pressure regulator spring, and hydraulic gain for each shift circuit.
    pub fn max_p_apply_clutch(&self, ss: ShiftCircuit) -> u16 {
        let mut p_max_shift = self
            .max_pcs_pressure()
            .saturating_sub(self.shift_reg_spring_pressure) as u32;
        p_max_shift *= self.shift_spc_gain[ss] as u32;
        (p_max_shift / 1000) as _
    }
}
