use core::ops::Index;

use egs_maths::{declare_2d_map, maps::Map2d};

use crate::gearbox_control::{Clutch, Gear, ShiftCircuit};

pub mod mech;
pub mod hydr;

pub const SHIFT_ARRAY_LEN: usize = 8;
pub const GEAR_ARRAY_LEN: usize = 8;
pub const CLUTCH_ARRAY_LEN: usize = 6;

pub type GearIndexedArray<T> = [T; GEAR_ARRAY_LEN];
pub type ClutchIndexedArray<T> = [T; CLUTCH_ARRAY_LEN];
pub type ShiftIndexedArray<T> = [T; SHIFT_ARRAY_LEN];
pub type HalfShiftIndexedArray<T> = [T; SHIFT_ARRAY_LEN/2];

impl<T> Index<ShiftCircuit> for ShiftIndexedArray<T> {
    type Output = T;

    fn index(&self, index: ShiftCircuit) -> &Self::Output {
        match index {
            ShiftCircuit::_12 => &self[0],
            ShiftCircuit::_23 => &self[1],
            ShiftCircuit::_34 => &self[2],
            ShiftCircuit::_45 => &self[3],
            ShiftCircuit::_21 => &self[4],
            ShiftCircuit::_32 => &self[5],
            ShiftCircuit::_43 => &self[6],
            ShiftCircuit::_54 => &self[7],
        }
    }
}

impl<T> Index<ShiftCircuit> for HalfShiftIndexedArray<T> {
    type Output = T;

    fn index(&self, index: ShiftCircuit) -> &Self::Output {
        match index {
            ShiftCircuit::_12 | ShiftCircuit::_21 => &self[0],
            ShiftCircuit::_23 | ShiftCircuit::_32 => &self[1],
            ShiftCircuit::_34 | ShiftCircuit::_43 => &self[2],
            ShiftCircuit::_45 | ShiftCircuit::_54 => &self[3],
        }
    }
}

impl<T> Index<Clutch> for ClutchIndexedArray<T> {
    type Output = T;

    fn index(&self, index: Clutch) -> &Self::Output {
        match index {
            Clutch::K1 => &self[0],
            Clutch::K2 => &self[1],
            Clutch::K3 => &self[2],
            Clutch::B1 => &self[3],
            Clutch::B2 => &self[4],
            Clutch::B3 => &self[5],
        }
    }
}

impl<T> Index<Gear> for GearIndexedArray<T> {
    type Output = T;

    fn index(&self, index: Gear) -> &Self::Output {
        match index {
            Gear::N | Gear::P => &self[0],
            Gear::_1 => &self[1],
            Gear::_2 => &self[2],
            Gear::_3 => &self[3],
            Gear::_4 => &self[4],
            Gear::_5 => &self[5],
            Gear::_R1 => &self[6],
            Gear::_R2 => &self[7],
        }
    }
}

#[derive(Copy, Clone)]
pub struct Calibrations {

}

#[derive(Copy, Clone)]
#[repr(packed)]
pub struct CodingHeader {
    // Version of the coding string for the car
    coding_name: [char; 6],
    // Version changes when calibration information index's change
    coding_version: u16,
}

#[derive(Copy, Clone)]
#[repr(packed)]
pub struct CodingString<const N: usize> {
    header: CodingHeader,
    calibration_indexes: [u16; N],

    checksum: u16
}

#[derive(Copy, Clone)]
#[repr(C)]
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

declare_2d_map!(PcsMap, 7, 4, u16, u16, u16);

impl HydrCal {
    pub const fn pcs_map<'a>(&'a self) -> PcsMap<'a> {
        Map2d::new(&self.pcs_map_x, &self.pcs_map_y, &self.pcs_map_z)
    }

    pub const fn max_pcs_pressure(&self) -> u16 {
        self.pcs_map_x[6]
    }
}

#[derive(Copy, Clone)]
#[repr(packed)]  
pub struct CalibrationPool<T, const N: usize> {
    pub scn_coding_idx: usize,
    pub pool_addr: core::ptr::NonNull<[T; N]>,
    pub current_val: core::ptr::NonNull<T>
}


pub enum CalibrationTypes {
    _Default = 0,
    Hydraulic = 1,
    Mechanical = 2,
    ShiftMaps = 3,
}