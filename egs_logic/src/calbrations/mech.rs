use egs_maths::{declare_2d_map, maps::Indexed2dMap};

use crate::{
    Clutch, Gear,
    calbrations::{ClutchIndexedArray, GearIndexedArray, HalfShiftIndexedArray, ShiftIndexedArray},
};

#[derive(Copy, Clone)]
#[repr(C)]
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

impl Default for MechCal {
    fn default() -> Self {
        Self {
            gb_ty: Default::default(),
            ratio_table: Default::default(),
            inertia_factor: Default::default(),
            friction_map: [0; 48],
            max_torque_on_clutch: Default::default(),
            max_torque_off_clutch: Default::default(),
            release_spring_pressure: Default::default(),
            inertia_torque: Default::default(),
            strongest_loaded_clutch_idx: Default::default(),
            turbine_drag: Default::default(),
            atf_density_minus_50c: Default::default(),
            atf_density_drop_per_c: Default::default(),
            atf_density_centrifugal_force_factor: Default::default(),
        }
    }
}

const CLUTCH_COMBO: [[Clutch; 3]; 7] = [
    [Clutch::K3, Clutch::B1, Clutch::B2], // 1st
    [Clutch::K1, Clutch::K3, Clutch::B2], // 2nd
    [Clutch::K2, Clutch::K2, Clutch::B2], // 3rd <-- Quirk (K1 is missing for ALL Calibrations)
    [Clutch::K1, Clutch::K2, Clutch::K3], // 4th
    [Clutch::K2, Clutch::K3, Clutch::B1], // 5th
    [Clutch::K3, Clutch::B1, Clutch::B3], // R1
    [Clutch::K1, Clutch::K3, Clutch::B3], // R2
];

fn check_coef_map(x: FrictionMap<'_>) -> bool {
    let mut ok = true;
    for gear in [
        Gear::_1,
        Gear::_2,
        Gear::_3,
        Gear::_4,
        Gear::_5,
        Gear::_R1,
        Gear::_R2,
    ] {
        let c_list = CLUTCH_COMBO[(gear as u8 - 1) as usize];
        for clutch in c_list {
            if x.get_at(clutch, gear) == 0 {
                ok = false;
                defmt::error!("Null friction value for Gear {} Clutch {}", gear, clutch);
                break;
            }
        }
    }
    ok
}

const _: () = assert!(size_of::<MechCal>() < 256);

declare_2d_map!(FrictionMap, 6, 8, u16);

impl MechCal {
    pub fn is_valid(&self) -> bool {
        self.gb_ty < 2 // Only 0 or 1 allowed
            && !self.ratio_table[1..].contains(&0) // Ratio table cannot be 0 past gear N
            && self.strongest_loaded_clutch_idx[1..] // Strongest loaded clutch cannot be out of bounds
                .iter()
                .find(|x| **x > 6)
                .is_none()
            && check_coef_map(self.friction_map())
    }

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
