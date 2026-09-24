use core::ops::Index;

use crate::{Clutch, Gear, ShiftCircuit};

pub mod hydr;
pub mod mech;

pub const SHIFT_ARRAY_LEN: usize = 8;
pub const GEAR_ARRAY_LEN: usize = 8;
pub const CLUTCH_ARRAY_LEN: usize = 6;

pub type GearIndexedArray<T> = [T; GEAR_ARRAY_LEN];
pub type ClutchIndexedArray<T> = [T; CLUTCH_ARRAY_LEN];
pub type ShiftIndexedArray<T> = [T; SHIFT_ARRAY_LEN];
pub type HalfShiftIndexedArray<T> = [T; SHIFT_ARRAY_LEN / 2];

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
            Gear::N => &self[0],
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
pub struct Calibrations {}

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

    checksum: u16,
}

#[derive(Copy, Clone)]
#[repr(packed)]
pub struct CalibrationPool<T, const N: usize> {
    pub scn_coding_idx: usize,
    pub pool_addr: core::ptr::NonNull<[T; N]>,
    pub current_val: core::ptr::NonNull<T>,
}

pub enum CalibrationTypes {
    _Default = 0,
    Hydraulic = 1,
    Mechanical = 2,
    ShiftMaps = 3,
}
