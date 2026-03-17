#![cfg_attr(feature = "mcu", no_std)]

//pub mod ioctl;
//pub use automotive_diag::*;

#[cfg(feature = "mcu")]
pub use embedded_crc32c;

#[cfg(feature = "mcu")]
pub mod isotp_endpoints;

#[cfg(feature = "mcu")]
pub mod userpage;

#[cfg(feature = "mcu")]
pub mod dyn_panic;

#[cfg(feature = "mcu")]
pub mod ram_info;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum BootloaderStayReason {
    None = 0,
    ResetCount = 1,
    Watchdog = 2,
    Panic = 3,
    MagicPin = 4,
    AppInvalid = 5,
    ProductionInfoNotSet = 6,
    RamFailure = 7,
    Unkown,
}

impl From<u8> for BootloaderStayReason {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::None,
            1 => Self::ResetCount,
            2 => Self::Watchdog,
            3 => Self::Panic,
            4 => Self::MagicPin,
            5 => Self::AppInvalid,
            6 => Self::ProductionInfoNotSet,
            _ => Self::Unkown,
        }
    }
}

pub struct KwpPanicInfo {}
