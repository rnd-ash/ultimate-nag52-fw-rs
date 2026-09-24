use core::mem::MaybeUninit;

use egs_maths::maps::TcuIdx;

pub mod vars;

#[derive(Default, Copy, Clone, defmt::Format, PartialEq, Eq, PartialOrd, Ord)]
#[repr(C)]
pub enum TftReading {
    #[default]
    ParkOrNeutral,
    Temperature(i16),
}

#[derive(Copy, Clone, defmt::Format, PartialEq, Eq, PartialOrd, Ord)]
#[repr(C)]
pub enum SelectorPos {
    P,
    RP,
    R,
    RN,
    N,
    DN,
    D,
    TipTronic(TipTronicPos),
    Trrs(TrrsPos),
}

#[derive(Copy, Clone, defmt::Format, PartialEq, Eq, PartialOrd, Ord)]
#[repr(C)]
pub enum TipTronicPos {
    Plus,
    Minus,
}

#[derive(Copy, Clone, defmt::Format, PartialEq, Eq, PartialOrd, Ord)]
#[repr(C)]
pub enum TrrsPos {
    _4,
    _3,
    _2,
    _1,
}

#[derive(Copy, Clone, defmt::Format, PartialEq, Eq, PartialOrd, Ord)]
#[repr(C)]
pub enum Profile {
    S = 0,
    C = 1,
    W = 2,
    A = 3,
    M = 4,
    R = 5,
}

impl Profile {
    pub fn manual_shifting(&self) -> bool {
        matches!(self, Self::M | Self::R)
    }
}

#[derive(Copy, Clone, defmt::Format, PartialEq, Eq, PartialOrd, Ord)]
#[repr(C)]
pub enum ShiftCircuit {
    _12 = 0,
    _23 = 1,
    _34 = 2,
    _45 = 3,
    _21 = 4,
    _32 = 5,
    _43 = 6,
    _54 = 7,
}

impl ShiftCircuit {
    pub fn is_up(&self) -> bool {
        (*self as u8) < 4
    }
}

#[derive(Copy, Clone, defmt::Format, PartialEq, Eq, PartialOrd, Ord)]
#[repr(C)]
pub enum Gear {
    N = 0,
    _1 = 1,
    _2 = 2,
    _3 = 3,
    _4 = 4,
    _5 = 5,
    _R1 = 6,
    _R2 = 7,
}

#[derive(Copy, Clone, defmt::Format, PartialEq, Eq, PartialOrd, Ord)]
#[repr(C)]
pub enum DisplayGear {
    N = 0,
    _1 = 1,
    _2 = 2,
    _3 = 3,
    _4 = 4,
    _5 = 5,
    _R1 = 6,
    _R2 = 7,
    P = 8,
}
impl Gear {
    pub fn is_power_free(&self) -> bool {
        matches!(self, Self::N)
    }

    pub fn is_reverse(&self) -> bool {
        matches!(self, Self::_R1 | Self::_R2)
    }
}

#[derive(Copy, Clone, defmt::Format, PartialEq, Eq, PartialOrd, Ord)]
#[repr(C)]
pub enum Clutch {
    K1,
    K2,
    K3,
    B1,
    B2,
    B3,
}

#[repr(C)]
pub enum TorqueRequest {
    None,
    LessThan { val: u16, is_end: bool },
    MoreThan { val: u16, is_end: bool },
}

impl TcuIdx<8> for Gear {}

impl Into<usize> for Gear {
    fn into(self) -> usize {
        match self {
            Gear::N => 0,
            Gear::_1 => 1,
            Gear::_2 => 2,
            Gear::_3 => 3,
            Gear::_4 => 4,
            Gear::_5 => 5,
            Gear::_R1 => 6,
            Gear::_R2 => 7,
        }
    }
}

impl TcuIdx<6> for Clutch {}

impl Into<usize> for Clutch {
    fn into(self) -> usize {
        match self {
            Clutch::K1 => 0,
            Clutch::K2 => 1,
            Clutch::K3 => 2,
            Clutch::B1 => 3,
            Clutch::B2 => 4,
            Clutch::B3 => 5,
        }
    }
}

/// EGS Statistics
/// These times are stored as 'ticks',
/// displaying how long processing took at various
/// code paths
#[derive(Copy, Clone)]
#[repr(C)]
pub struct StatTimes {
    /// Time for input processing and filtering functions to run
    pub input_processing: u16,
    /// Time for Shift actuation functions to run
    pub shift_actuation_logic: u16,
    /// Time for shift torque request functions to run
    pub torque_requests: u16,
    /// Time for adaptation logic,
    pub adaptation: u16,
    /// Time for error/safety code to run
    pub safety: u16,
    /// Time for Torque converter code to run
    pub tcc_update: u16,
    /// Time for shift point calculations to run
    pub shift_point_calc_logic: u16,
    /// Time for setting outputs for the HAL
    pub output_processing: u16,
}

impl StatTimes {
    pub const fn new() -> Self {
        Self {
            input_processing: 0,
            shift_actuation_logic: 0,
            torque_requests: 0,
            safety: 0,
            tcc_update: 0,
            shift_point_calc_logic: 0,
            output_processing: 0,
            adaptation: 0,
        }
    }
}
