#[derive(Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum CanLayerTy {
    #[default]
    None = 0,
    EGS51 = 1,
    EGS52 = 2,
    EGS53 = 3,
    Hfm = 4,
    Custom = 5,
}

#[derive(Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum ShifterTy {
    #[default]
    None = 0,
    Ewm = 1,
    Slr = 2,
    Trrs = 3,
    Ism = 4,
}

#[derive(Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LogicThresh {
    _5V,
    #[default]
    _12V,
}

#[derive(Default, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum InPinPurpose {
    #[default]
    None,
    InputPulse {
        thresh: LogicThresh,
        pulses_rev: u8,
    },
}

bitflags::bitflags! {
    pub struct EgsConfigFlags: u32 {
        const FOURMATIC = 1;
        const CHRYSLER = 1 << 1;
        const HARDWARE_START = 1 << 2;

    }
}

#[derive(Default, Clone, Copy)]
#[repr(packed)]
pub struct EgsConfig {
    shifter_ty: ShifterTy,
    can_ty: CanLayerTy,

    engine_inertia_nm: u8,
    tyre_size_mm: u16,
    diff_ratio: u16,
    redline: u16,
    transfer_case_hi: u16,
    transfer_case_low: u16,

    pin_23_purpose: InPinPurpose,
}
