use bitflags::bitflags;

use crate::calbrations::{hydr::HydrCal, mech::MechCal};

pub enum StoragePoll {
    Waiting,
    Ready,
    Error,
}

pub trait StorageBacking {
    fn init_poll(&mut self) -> StoragePoll;
}

pub trait GearboxDtcStorage: StorageBacking {}

pub trait GearboxMapStorage: StorageBacking {}

pub trait GearboxCalibStorage: StorageBacking {
    fn hydr_cal(&self) -> &HydrCal;
    fn mech_cal(&self) -> &MechCal;
}

pub trait GearboxAdaptStorage: StorageBacking {}

#[repr(C)]
pub enum EgsCanLayerTy {
    Egs51 = 1,
    EGs52 = 2,
    EGs53 = 3,
    Custom = 0xF0,
    NotDefined = 0xFF
}

bitflags! {
    pub struct VehicleConfigFlags: u32 {
        const FOURMATIC = 1;
    }

    pub struct VehicleSignalFlags: u32 {
        const SHIFT_PADDLES = 1 << 0;
        const TRANSFER_CASE = 1 << 1;
        const OUTPUT_SHAFT_SENSOR = 1 << 2;
        const PHYSICAL_START_SIGNAL = 1 << 3;
    }
}

pub struct VehicleCoding {
    pub name: [u8; 4],
    pub motor_inertia: u8,
    pub tyre_size_mm: u16,
    pub diff_ratio: u16,

    pub transfer_ratio_high: u16,
    pub transfer_ratio_low: u16,

    pub crc: u32,
}
