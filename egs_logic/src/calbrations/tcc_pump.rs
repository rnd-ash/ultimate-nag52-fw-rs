use egs_maths::{
    declare_2d_map,
    maps::{Map1d, Map2d},
};

use crate::ShiftCircuit;

#[derive(Copy, Clone, Default)]
#[repr(C)]
pub struct TccPumpCal {
    pub multiplier_x: [u16; 2],
    pub multiplier_z: [u16; 2],
    pub pump_lambda_x: [u16; 11],
    pub pump_lambda_z: [u16; 11],
}

const _: () = assert!(size_of::<TccPumpCal>() < 256);

impl TccPumpCal {
    pub fn is_valid(&self) -> bool {
        (self.multiplier_x[0] == 0 && (500..=1500).contains(&self.multiplier_x[1]))
            && (self.multiplier_z[0] != 0 && self.multiplier_z[1] == 100)
            && self.pump_lambda_x.iter().filter(|x| **x == 0).count() == 1
            && self.pump_lambda_z.iter().filter(|x| **x == 0).count() == 1
    }

    pub const fn torque_multiplier_map<'a>(&'a self) -> Map1d<'a, u16, u16, 2> {
        egs_maths::maps::Map1d::new(&self.multiplier_x, &self.multiplier_z)
    }

    pub const fn pump_lambda_map<'a>(&'a self) -> Map1d<'a, u16, u16, 11> {
        egs_maths::maps::Map1d::new(&self.pump_lambda_x, &self.pump_lambda_z)
    }
}
