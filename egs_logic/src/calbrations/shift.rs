use egs_maths::{
    declare_2d_map,
    maps::{Map1d, Map2d},
};

use crate::{ShiftCircuit, calbrations::HalfShiftIndexedArray};

#[derive(Copy, Clone)]
#[repr(C)]
pub struct ShiftMapCal {
    pub freeing_trq_up_x: HalfShiftIndexedArray<[u8; 3]>,
    pub freeing_trq_up_y: HalfShiftIndexedArray<[u8; 2]>,
    pub freeing_trq_up_z: HalfShiftIndexedArray<[u8; 6]>,

    pub freeing_trq_dn_x: HalfShiftIndexedArray<[u8; 6]>,
    pub freeing_trq_dn_y: HalfShiftIndexedArray<[u8; 10]>,
    pub freeing_trq_dn_z: HalfShiftIndexedArray<[u8; 60]>,

    pub trq_adder_up_x: HalfShiftIndexedArray<[u8; 6]>,
    pub trq_adder_up_y: HalfShiftIndexedArray<[u8; 8]>,
    pub trq_adder_up_z: HalfShiftIndexedArray<[u8; 48]>,

    pub trq_adder_dn_x: HalfShiftIndexedArray<[u8; 3]>,
    pub trq_adder_dn_y: HalfShiftIndexedArray<[u8; 4]>,
    pub trq_adder_dn_z: HalfShiftIndexedArray<[u8; 12]>,
}

impl Default for ShiftMapCal {
    fn default() -> Self {
        Self {
            freeing_trq_up_x: Default::default(),
            freeing_trq_up_y: Default::default(),
            freeing_trq_up_z: Default::default(),
            freeing_trq_dn_x: Default::default(),
            freeing_trq_dn_y: Default::default(),
            freeing_trq_dn_z: [[0u8; 60]; 4],
            trq_adder_up_x: Default::default(),
            trq_adder_up_y: Default::default(),
            trq_adder_up_z: [[0u8; 48]; 4],
            trq_adder_dn_x: Default::default(),
            trq_adder_dn_y: Default::default(),
            trq_adder_dn_z: Default::default(),
        }
    }
}

const _: () = assert!(size_of::<ShiftMapCal>() < 1024);

impl ShiftMapCal {
    pub fn is_valid(&self) -> bool {
        true
    }

    pub fn freeing_torque(
        &self,
        shift: ShiftCircuit,
        output_speed: u16,
        engine_torque: f32,
    ) -> f32 {
        if shift.is_up() {
            let map = Map2d::new(
                &self.freeing_trq_up_x[shift],
                &self.freeing_trq_up_y[shift],
                &self.freeing_trq_up_z[shift],
            );
            map.interp(output_speed as f32 / 30.0, engine_torque / 5.0) * 5.0
        } else {
            let map = Map2d::new(
                &self.freeing_trq_dn_x[shift],
                &self.freeing_trq_dn_y[shift],
                &self.freeing_trq_dn_z[shift],
            );
            map.interp(output_speed as f32 / 30.0, engine_torque / 5.0) * 5.0
        }
    }

    pub fn torque_adder(&self, shift: ShiftCircuit, output_speed: u16, engine_torque: f32) -> f32 {
        if shift.is_up() {
            let map = Map2d::new(
                &self.trq_adder_up_x[shift],
                &self.trq_adder_up_y[shift],
                &self.trq_adder_up_z[shift],
            );
            map.interp(output_speed as f32 / 30.0, engine_torque / 5.0) * 5.0
        } else {
            let map = Map2d::new(
                &self.trq_adder_dn_x[shift],
                &self.trq_adder_dn_y[shift],
                &self.trq_adder_dn_z[shift],
            );
            map.interp(output_speed as f32 / 30.0, engine_torque / 5.0) * 5.0
        }
    }
}
