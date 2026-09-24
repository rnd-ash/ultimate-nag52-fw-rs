use egs_maths::{declare_2d_map, maps::Indexed2dMap};

use crate::{
    Gear,
    calbrations::{ClutchIndexedArray, GearIndexedArray, HalfShiftIndexedArray, ShiftIndexedArray},
};

pub struct MechCal {
    pub gb_ty: u8,
    pub ratio_table: GearIndexedArray<u16>,
    pub inertia_factor: ShiftIndexedArray<u16>,
    pub friction_map: [u16; 48],
    pub max_torque_on_clutch: HalfShiftIndexedArray<u16>,
    pub max_torque_off_clutch: HalfShiftIndexedArray<u16>,
    pub release_spring_pressure: ClutchIndexedArray<u16>,
    pub inertia_torque: ShiftIndexedArray<u16>,
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

    pub const fn weakest_clutch(&self) -> &GearIndexedArray<u8> {
        &self.strongest_loaded_clutch_idx
    }

    pub const fn release_spring(&self) -> &ClutchIndexedArray<u16> {
        &self.release_spring_pressure
    }

    /// Returns the gear ratio as u32 multiplied by 1000
    /// (EG: 3140 for 3.14 ratio)
    pub fn ratio_u32(&self, gear: Gear) -> u32 {
        self.ratio_table[gear] as u32
    }

    /// Returns the gear ratio as f32 (EG: 3.14)
    pub fn ratio_f32(&self, gear: Gear) -> f32 {
        self.ratio_table[gear] as f32 / 1000.0
    }
}
