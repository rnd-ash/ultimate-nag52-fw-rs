use egs_maths::{
    declare_2d_map,
    maps::{Indexed2dMap, Map1d},
};

use crate::calbrations::{
    ClutchIndexedArray, GearIndexedArray, HalfShiftIndexedArray, ShiftIndexedArray,
};

pub struct MechCal {
    pub gb_ty: u8,
    pub ratio_table: GearIndexedArray<u16>,
    pub intertia_factor: ShiftIndexedArray<u16>,
    pub friction_map: [u16; 48],
    pub max_torque_on_clutch: HalfShiftIndexedArray<u16>,
    pub max_torque_off_clutch: HalfShiftIndexedArray<u16>,
    pub release_spring_pressure: ClutchIndexedArray<u16>,
    pub intertia_torque: ShiftIndexedArray<u16>,
    pub strongest_loaded_clutch_idx: GearIndexedArray<u8>,
    pub turbine_drag: [u16; 8],
    pub atf_density_minus_50c: u16,
    pub atf_density_drop_per_c: u16,
    pub atf_density_centrifugal_force_factor: [u16; 3],
}

declare_2d_map!(FrictionMap, 6, 8, u16);

impl MechCal {
    pub const fn friction_map<'a>(&'a self) -> FrictionMap<'a> {
        Indexed2dMap::new(&self.friction_map)
    }

    pub const fn weakest_clutch<'a>(&'a self) -> &GearIndexedArray<u8> {
        &self.strongest_loaded_clutch_idx
    }

    pub const fn release_spring<'a>(&'a self) -> &ClutchIndexedArray<u16> {
        &self.release_spring_pressure
    }
}
