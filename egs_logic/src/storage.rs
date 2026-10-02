use bitflags::bitflags;

use crate::calbrations::{hydr::HydrCal, mech::MechCal, shift::ShiftMapCal, tcc_pump::TccPumpCal};

#[derive(Copy, Clone)]
/// Storage database poll result
pub enum StoragePoll {
    /// Storage is waiting for something (Not ready)
    Waiting,
    /// Ready - Guarantees that the data loaded
    /// is valid, and will NOT change for the duration
    /// of the current drive cycle
    Ready,
    /// Error - Data loaded is invalid or couldn't be
    /// loaded. This puts EGS into emergency mode
    /// and disables running completely.
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
    fn tcc_pump_cal(&self) -> &TccPumpCal;
    fn shift_map_cal(&self) -> &ShiftMapCal;
}

pub trait GearboxAdaptStorage: StorageBacking {}

#[repr(C)]
pub enum EgsCanLayerTy {
    Egs51 = 1,
    Egs52 = 2,
    Egs53 = 3,
    Custom = 0xF0,
    NotDefined = 0xFF,
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
