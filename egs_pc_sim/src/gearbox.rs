use egs_logic::{
    StoragePoll,
    calbrations::{hydr::HydrCal, mech::MechCal},
};

#[derive(Default)]
pub struct SimDtcStorage;

impl StorageBacking for SimDtcStorage {
    fn init_poll(&mut self) -> StoragePoll {
        StoragePoll::Ready
    }
}

impl GearboxDtcStorage for SimDtcStorage {}

#[derive(Default)]
pub struct SimAdpStorage;

impl egs_logic::StorageBacking for SimAdpStorage {
    fn init_poll(&mut self) -> StoragePoll {
        StoragePoll::Ready
    }
}

impl egs_logic::GearboxAdaptStorage for SimAdpStorage {}

#[derive(Default)]
pub struct SimCalStorage;

impl egs_logic::StorageBacking for SimCalStorage {
    fn init_poll(&mut self) -> StoragePoll {
        StoragePoll::Ready
    }
}

const HYDR_CAL: HydrCal = HydrCal {
    p_multi_1: 513,
    p_multi_other: 638,
    lp_reg_spring_pressure: 1926,
    overlap_circuit_factor_spc: [878, 1059, 1109, 1407, 1407, 534, 803, 878],
    overlap_circuit_factor_mpc: [1407, 534, 803, 878, 878, 1059, 1109, 1407],
    overlap_circuit_spring_pressure: [-826, -168, -432, -826, -826, -168, -432, -826],
    shift_reg_spring_pressure: 601,
    shift_spc_gain: [1993, 1000, 1000, 1000, 1993, 1000, 1000, 1000],
    min_mpc_pressure: 1500,
    filter_factor: 15,
    mpc_flush_temp_threshold: 75,
    mpc_no_flush_time: 30_000,
    mpc_flush_time: 50,
    extra_p_not_shifting: 0,
    shift_pressure_addr_percent: 20,
    inlet_pressure_offset: 1000,
    inlet_pressure_input_min: 4000,
    inlet_pressure_input_max: 10_000,
    inlet_pressure_output_min: 4000,
    inlet_pressure_output_max: 10_000,
    extra_pressure_pump_speed_min: 1000,
    extra_pressure_pump_speed_max: 4000,
    extra_pressure_adder_r1_1: 1500,
    extra_pressure_adder_other_gears: 1000,
    shift_pressure_factor_percent: 37,
    pcs_map_x: [100, 1300, 1800, 3250, 7100, 8200, 9700],
    pcs_map_y: [25, 75, 110, 200],
    pcs_map_z: [
        1155, 945, 880, 710, 410, 320, 150, 1055, 860, 810, 685, 395, 300, 0, 990, 805, 765, 660,
        385, 275, 0, 965, 770, 730, 620, 345, 235, 0,
    ],
};

pub(crate) const MECH_CAL: MechCal = MechCal {
    gb_ty: 0,
    ratio_table: [0, 3595, 2186, 1405, 1000, 831, 3167, 1926],
    inertia_factor: [1645, 1556, 1405, 1203, 1644, 1556, 1405, 1203],
    friction_map: [
        4709, 0, 0, 3574, 0, 0, 0, 0, 3076, 2303, 2685, 0, 1845, 0, 1871, 0, 1633, 0, 0, 1101, 0,
        0, 1109, 0, 958, 1673, 971, 0, 0, 0, 0, 1390, 807, 604, 0, 0, 0, 0, 3076, 2303, 0, 3387,
        1845, 0, 1871, 0, 0, 2060,
    ],
    max_torque_on_clutch: [1000, 1000, 640, 820],
    max_torque_off_clutch: [1000, 1000, 1440, 750],
    release_spring_pressure: [1270, 846, 1205, 1139, 1289, 488],
    inertia_torque: [16, 18, 25, 125, 16, 18, 25, 125],
    strongest_loaded_clutch_idx: [255, 2, 2, 1, 1, 1, 2, 2],
    turbine_drag: [16, 10, 35, 59, 16, 18, 25, 30],
    atf_density_minus_50c: 889,
    atf_density_drop_per_c: 60,
    atf_density_centrifugal_force_factor: [0, 40_000, 3_000],
};

impl egs_logic::GearboxCalibStorage for SimCalStorage {
    fn hydr_cal(&self) -> &egs_logic::calbrations::hydr::HydrCal {
        &HYDR_CAL
    }

    fn mech_cal(&self) -> &MechCal {
        &MECH_CAL
    }
}

#[derive(Default)]
pub struct SimMapStorage;

impl egs_logic::StorageBacking for SimMapStorage {
    fn init_poll(&mut self) -> StoragePoll {
        StoragePoll::Ready
    }
}

impl egs_logic::GearboxMapStorage for SimMapStorage {}

pub type SimGearbox<'a> =
    egs_logic::Gearbox<'a, SimDtcStorage, SimCalStorage, SimMapStorage, SimAdpStorage>;
