use egs_logic::calbrations::{
    hydr::HydrCal, mech::MechCal, shift::ShiftMapCal, tcc_pump::TccPumpCal,
};

pub mod eeprom;
pub mod qspi;

/// EGS Will ask the QSPI Commander for data
#[derive(Copy, Clone)]
pub enum QspiStorageCmd {
    ReadMechCal,
    ReadHydrCal,
    ReadShiftMapCal,
    ReadTccPumpCal,
}

/// QSPI will respond to EGS with these replies
#[derive(Copy, Clone)]
pub enum QspiStorageResp {
    MechCal(Option<MechCal>),
    HydrCal(Option<HydrCal>),
    ShiftMapCal(Option<ShiftMapCal>),
    TccPumpCal(Option<TccPumpCal>),
}

pub const EKV_KEY_HYDR_CAL: &str = "HYDR_CAL";
pub const EKV_KEY_MECH_CAL: &str = "MECH_CAL";
pub const EKV_KEY_SHIFT_MAP_CAL: &str = "SHIFT_MAP_CAL";
pub const EKV_KEY_TCC_PUMP_CAL: &str = "TCC_P_CAL";
