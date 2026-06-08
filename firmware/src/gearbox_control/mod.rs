use egs_maths::maps::TcuIdx;

pub mod pressure_control;

pub enum ControlState {
    InGear,
    ShiftingRD,
    ShiftingPN,
    Shifting
}

#[derive(Copy, Clone, defmt::Format, PartialEq, Eq, PartialOrd, Ord)]
pub enum ShiftCircuit {
    _12,
    _23,
    _34,
    _45,
    _21,
    _32,
    _43,
    _54
}

impl ShiftCircuit {
    pub fn is_1_2_circuit(&self) -> bool {
        matches!(self, Self::_12 | Self::_21)
    }
}

#[derive(Copy, Clone, defmt::Format, PartialEq, Eq, PartialOrd, Ord)]
pub enum Gear {
    N,
    _1,
    _2,
    _3,
    _4,
    _5,
    _R1,
    _R2,
    P,
}

#[derive(Copy, Clone, defmt::Format, PartialEq, Eq, PartialOrd, Ord)]
pub enum Clutch {
    K1,
    K2,
    K3,
    B1,
    B2,
    B3
}

#[derive(Copy, Clone, defmt::Format, PartialEq, Eq, PartialOrd, Ord)]
pub enum GearStatus {
    Gear(Gear),
    PowerFreeInD,
    Abort
}