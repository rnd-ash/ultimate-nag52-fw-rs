use egs_maths::{declare_2d_map, maps::Indexed2dMap};

use crate::calbrations::{ClutchIndexedArray, GearIndexedArray, HalfShiftIndexedArray, ShiftIndexedArray};


pub struct MechCal {
    gb_ty: u8,
    ratio_table: GearIndexedArray<u16>,
    intertia_factor: ShiftIndexedArray<u16>,
    friction_map: [u16; 48],
    max_torque_on_clutch: HalfShiftIndexedArray<u16>,
    max_torque_off_clutch: HalfShiftIndexedArray<u16>,
    release_spring_pressure: ClutchIndexedArray<u16>,
    intertia_torque: ShiftIndexedArray<u16>,
    strongest_loaded_clutch_idx: GearIndexedArray<u8>,
    turbine_drag: [u16; 8],
    atf_density_minus_50c: u16,
    atf_density_drop_per_c: u16,
    atf_density_centrifugal_force_factor: [u16; 3]
}

declare_2d_map!(FrictionMap, 8, 6, u16);

impl MechCal {
    pub const fn friction_map<'a>(&'a self) ->  FrictionMap<'a> {
        Indexed2dMap::new(&self.friction_map)
    }
}