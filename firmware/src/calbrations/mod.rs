pub const SHIFT_ARRAY_LEN: usize = 8;

#[derive(Copy, Clone)]
pub struct Calibrations {

}


#[derive(Copy, Clone)]
#[repr(packed)]
pub struct HydrCal {
    pub p_multi_1: u16,
    pub p_multi_other: u16,
    pub lp_reg_spring_pressure: u16,
    pub overlap_circuit_factor_spc: [u16; SHIFT_ARRAY_LEN],
    pub overlap_circuit_factor_mpc: [u16; SHIFT_ARRAY_LEN],
    pub overlap_circuit_spring_pressure: [i16; SHIFT_ARRAY_LEN],
    pub shift_reg_spring_pressure: u16,
    pub shift_spc_gain: [u16; SHIFT_ARRAY_LEN],
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