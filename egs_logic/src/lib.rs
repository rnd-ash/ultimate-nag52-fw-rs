#![no_std]

use bitflags::Flags;
pub use egs_can;

use egs_can::{CanRxData, CanTxData};

use crate::errors::DeviceMode;


pub mod shifting_algos;
pub mod errors;
pub mod status;
pub mod config;


pub enum TftReading {
    Pll(bool),
    Temperature(i16)
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
    Trrs(TrrsPos)
}

pub enum TipTronicPos {
    Plus,
    Minus
}

pub enum TrrsPos {
    _4,
    _3,
    _2,
    _1
}

pub enum Profile {
    S = 0,
    C = 1,
    A = 2,
    M = 3,
    R = 4
}

pub enum TorqueRequest {
    None,
    LessThan { val: u16, is_end: bool },
    MoreThan { val: u16, is_end: bool }
}

#[derive(Default)]
pub struct GearboxPhysicalInputs {
    pub clock_time_ms: u32,
    // Board vars (Gearbox side)
    pub n2_rpm: Option<u16>,
    pub n3_rpm: Option<u16>,
    pub nout_rpm: Option<u16>,
    pub atf: Option<TftReading>,
    // Board vars (Vehicle side)

    pub can_vars: CanRxData
}

#[derive(Default)]
pub struct GearboxOutputs {
    // Hardware outputs
    pub tcc_pwm: u8,
    pub mpc_current: u16,
    pub spc_current: u16,
    pub y3_en: bool,
    pub y4_en: bool,
    pub y5_en: bool,
    pub rp_en: bool,
    pub gpio_en: bool,
    // Logical outputs
    pub can_outputs: CanTxData
}

#[allow(async_fn_in_trait)]
pub trait StorageBacking {
    async fn init(&mut self, mode: &mut DeviceMode);
}

pub trait GearboxDtcStorage: Default + StorageBacking {

}

pub trait GearboxMapStorage: Default + StorageBacking {
}

pub trait GearboxCalibStorage: Default + StorageBacking {

}

pub trait GearboxAdaptStorage: Default + StorageBacking {
    
}

pub struct Gearbox<
    DS: GearboxDtcStorage,
    CS: GearboxCalibStorage,
    MS: GearboxMapStorage,
    AS: GearboxAdaptStorage
> {
    dtc_storage: DS,
    cal_storage: CS,
    map_storage: MS,
    adp_storage: AS,

    mode: DeviceMode,
    profile: Profile,
}

impl<
    DS: GearboxDtcStorage,
    CS: GearboxCalibStorage,
    MS: GearboxMapStorage,
    AS: GearboxAdaptStorage
> Default for Gearbox<DS, CS, MS, AS> {
    fn default() -> Self {
        Self { 
            mode: DeviceMode::INIT,
            profile: Profile::S,
            dtc_storage: DS::default(),
            cal_storage: CS::default(),
            map_storage: MS::default(),
            adp_storage: AS::default()
        }
    }
}

impl<
    DS: GearboxDtcStorage,
    CS: GearboxCalibStorage,
    MS: GearboxMapStorage,
    AS: GearboxAdaptStorage
> Gearbox<DS, CS, MS, AS> {

    /// This function MUST be called every 20ms to update the gearbox control loop
    pub async fn update(&mut self, inputs: &GearboxPhysicalInputs, outputs: &mut GearboxOutputs) {
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
            }
        }
    }

    // Sets via OR
    pub fn set_dev_mode(&mut self, mode: DeviceMode){
        self.mode |= mode;
    }

    pub fn dev_mode(&self) -> DeviceMode {
        self.mode
    }
}