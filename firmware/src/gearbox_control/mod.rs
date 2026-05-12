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
pub enum GearStatus {
    Gear(Gear),
    PowerFreeInD,
    Abort
}