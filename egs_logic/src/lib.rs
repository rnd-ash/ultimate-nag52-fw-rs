#![no_std]

use core::u16;

use bitflags::Flags;
pub use egs_can;

use egs_can::{CanRxData, CanTxData};

use crate::{
    calbrations::{hydr::HydrCal, mech::MechCal},
    errors::DeviceMode,
    hydraulics::HydraulicMgr,
};

pub mod calbrations;
pub mod config;
pub mod errors;
pub mod hydraulics;
pub mod shifting_algos;
pub mod status;
pub mod types;

#[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum TftReading {
    #[default]
    ParkOrNeutral,
    Temperature(i16),
}

pub enum SelectorPos {
    P,
    RP,
    R,
    RN,
    N,
    DN,
    D,
    TipTronic(TipTronicPos),
    Trrs(TrrsPos),
}

pub enum TipTronicPos {
    Plus,
    Minus,
}

pub enum TrrsPos {
    _4,
    _3,
    _2,
    _1,
}

pub enum Profile {
    S = 0,
    C = 1,
    A = 2,
    M = 3,
    R = 4,
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
    _54,
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
    B3,
}

pub enum TorqueRequest {
    None,
    LessThan { val: u16, is_end: bool },
    MoreThan { val: u16, is_end: bool },
}

#[derive(Default, Copy, Clone)]
pub struct GearboxInputs {
    pub clock_time_ms: u32,
    // Board vars (Gearbox side)
    pub n2_rpm: Option<u16>,
    pub n3_rpm: Option<u16>,
    pub nout_rpm: Option<u16>,
    pub atf: Option<TftReading>,
    // Board vars (Vehicle side)
    pub can_vars: CanRxData,
}

#[derive(Default, Copy, Clone)]
pub struct GearboxOutputs {
    // Hardware outputs
    pub tcc_pwm: u16,
    pub mpc_current: u16,
    pub spc_current: u16,
    pub y3_en: bool,
    pub y4_en: bool,
    pub y5_en: bool,
    pub rp_en: bool,
    pub gpio_en: bool,
    // Logical outputs
    pub can_outputs: CanTxData,

    pub diag_mpc_pressure: u16,
    pub diag_spc_pressure: u16,
}

#[allow(async_fn_in_trait)]
pub trait StorageBacking {
    async fn init(&mut self, mode: &mut DeviceMode);
}

pub trait GearboxDtcStorage: Default + StorageBacking {}

pub trait GearboxMapStorage: Default + StorageBacking {}

pub trait GearboxCalibStorage: Default + StorageBacking {
    fn hydr_cal(&self) -> &HydrCal;
    fn mech_cal(&self) -> &MechCal;
}

pub trait GearboxAdaptStorage: Default + StorageBacking {}

/// High precision microsecond counter for profiling
pub trait TcuTickCounter {
    fn start(&mut self);
    // Maximum 500ns ticks u16::MAX (~32ms)
    fn ticks(&mut self) -> u16;
}

pub struct Gearbox<
    DS: GearboxDtcStorage,
    CS: GearboxCalibStorage,
    MS: GearboxMapStorage,
    AS: GearboxAdaptStorage,
> {
    dtc_storage: DS,
    cal_storage: CS,
    map_storage: MS,
    adp_storage: AS,

    mode: DeviceMode,
    profile: Profile,
    engine_running: bool,
    temperature_atf_offset_50: u16,
    mpc_pressure: u16,
    spc_pressure: Option<u16>,
}

impl<DS: GearboxDtcStorage, CS: GearboxCalibStorage, MS: GearboxMapStorage, AS: GearboxAdaptStorage>
    Default for Gearbox<DS, CS, MS, AS>
{
    fn default() -> Self {
        Self {
            mode: DeviceMode::INIT,
            profile: Profile::S,
            dtc_storage: DS::default(),
            cal_storage: CS::default(),
            map_storage: MS::default(),
            adp_storage: AS::default(),
            engine_running: false,
            temperature_atf_offset_50: 90, // 40C default
            mpc_pressure: 500,
            spc_pressure: None,
        }
    }
}

impl<DS: GearboxDtcStorage, CS: GearboxCalibStorage, MS: GearboxMapStorage, AS: GearboxAdaptStorage>
    Gearbox<DS, CS, MS, AS>
{
    /// This function MUST be called every 20ms to update the gearbox control loop
    pub async fn update(&mut self, inputs: &GearboxInputs, outputs: &mut GearboxOutputs) {
        if self.mode.contains(DeviceMode::INIT) {
            defmt::info!("Gearbox init!");
            self.cal_storage.init(&mut self.mode).await;
            self.adp_storage.init(&mut self.mode).await;
            self.map_storage.init(&mut self.mode).await;
            self.dtc_storage.init(&mut self.mode).await;
            self.mode.remove(DeviceMode::INIT);
        } else {
            // Looping
            if self.mode.contains(DeviceMode::SLAVE) {
                // Special mode (Handled per-device)
            } else {
                // Normal EGS logic
                if !self.engine_running {
                    if let Ok(rpm) = inputs.can_vars.engine_rpm
                        && rpm > 400
                    {
                        self.engine_running = true;
                    }
                    // Defaults from EGS for when the engine is off
                }
                self.calculate_solenoid_status(inputs, outputs);
                let motor_temperature =
                    inputs.can_vars.coolant_temperature_c.unwrap_or_else(|_| 80);
                match inputs.atf {
                    Some(TftReading::ParkOrNeutral) => {
                        outputs.can_outputs.can_start = true;
                        outputs.can_outputs.gearbox_temperature_c = motor_temperature;
                    }
                    Some(TftReading::Temperature(grad_c)) => {
                        outputs.can_outputs.can_start = false;
                        outputs.can_outputs.gearbox_temperature_c = grad_c;
                    }
                    None => {
                        outputs.can_outputs.can_start = false;
                        outputs.can_outputs.gearbox_temperature_c = motor_temperature;
                    }
                }
                outputs.can_outputs.gearbox_temperature_c =
                    self.temperature_atf_offset_50 as i16 - 50;
            }
        }
    }

    // Sets via OR
    pub fn set_dev_mode(&mut self, mode: DeviceMode) {
        self.mode |= mode;
    }

    pub fn dev_mode(&self) -> DeviceMode {
        self.mode
    }

    fn calculate_solenoid_status(&mut self, inputs: &GearboxInputs, outputs: &mut GearboxOutputs) {
        if !self.engine_running {
            self.spc_pressure = None;
            self.mpc_pressure = 500;
        } else {
            // Engine running
            // Test - assume always in gear D
            let torque = inputs.can_vars.engine_static_torque_nm.unwrap_or(0.0);
            self.mpc_pressure = hydraulics::calculate_mpc_in_gear(
                torque,
                Gear::_2,
                self.mpc_pressure,
                self.cal_storage.hydr_cal(),
                self.cal_storage.mech_cal(),
            ) as u16;
        }

        // Calculate as solenoid output current
        let hydr_mgr = HydraulicMgr {
            hydr_cal: self.cal_storage.hydr_cal(),
            temperature_c_offset_50: self.temperature_atf_offset_50,
            current_gear: Gear::_1,
            engine_rpm: inputs.can_vars.engine_rpm.unwrap_or(2000),
            shift_state: None,
        };
        // SPC is only actuated if pressure is required
        if let Some(spc_targ) = self.spc_pressure {
            outputs.spc_current = hydr_mgr
                .calculate_solenoid_current(spc_targ as u32, self.mpc_pressure as u32)
                as u16;
            outputs.diag_spc_pressure = spc_targ;
        } else {
            outputs.spc_current = 0;
            outputs.diag_spc_pressure = self.cal_storage.hydr_cal().max_pcs_pressure();
        }
        outputs.mpc_current = hydr_mgr
            .calculate_solenoid_current(self.mpc_pressure as u32, self.mpc_pressure as u32)
            as u16;
        outputs.diag_mpc_pressure = self.mpc_pressure;
    }
}
