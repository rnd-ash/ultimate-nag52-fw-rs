use egs_maths::maps::TcuIdx;

use crate::{Clutch, Gear};

impl TcuIdx<8> for Gear {}

impl Into<usize> for Gear {
    fn into(self) -> usize {
        match self {
            Gear::N | Gear::P => 0,
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
