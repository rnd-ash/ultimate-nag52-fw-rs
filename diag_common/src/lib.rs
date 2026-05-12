#![cfg_attr(feature = "mcu", no_std)]

use core::ops::Range;

#[cfg(feature = "mcu")]
pub mod isotp_endpoints;

#[cfg(feature = "mcu")]
pub mod hal_extensions;

pub mod smarteeprom;

#[cfg(feature = "mcu")]
pub mod dyn_panic;

#[cfg(feature = "mcu")]
pub mod ram_info;

#[cfg(feature = "mcu")]
pub mod defmt_multi_output;

#[derive(Debug, Clone, Copy)]
pub enum DefmtTarget {
    Rtt = 0, // Default (Always available)
    Can = 1,
    Serial = 2,
}

pub const CAN_ID_DEFMT_LOG: u16 = 0x500; // Reserved on ALL CAN Layers
pub const USB_PACKET_TY_ISOTP: u8 = 0xFF;
pub const USB_PACKET_TY_DEFMT: u8 = 0xFE;

pub const fn parse_u8(s: &str) -> u8 {
    let mut p = konst::Parser::new(s);
    konst::result::unwrap!(p.parse_u8())
}

const GIT_SHA_LEN: usize = 12;
pub const fn parse_git_sha(s: &str) -> [u8; GIT_SHA_LEN] {
    let mut out = [0; GIT_SHA_LEN];

    let mut chars = konst::string::chars(s);
    konst::for_range! { i in 0..GIT_SHA_LEN =>
        out[i] = chars.next().unwrap() as u8
    }
    out
}

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
    Diagnostics = 8,
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
            7 => Self::RamFailure,
            8 => Self::Diagnostics,
            _ => Self::Unkown,
        }
    }
}

pub struct KwpPanicInfo {}

const KB: u32 = 1024;
const SECTOR_SIZE: u32 = 8 * KB;
const PRELOADER_ADDR_RANGE: Range<u32> = 0..(8 * KB);
const BOOTLOADER_ADDR_RANGE: Range<u32> = (8 * KB)..(120 * KB);
const BOOTLOADER_SCRATCH_ADDR_RANGE: Range<u32> = (120 * KB)..(240 * KB);
const APP_ADDR_RANGE: Range<u32> = (120 * KB)..(1024 * KB);

pub enum MemoryRegion {
    Preloader,
    Bootloader,
    BootloaderScratch,
    Application,
}

impl MemoryRegion {
    pub const fn range_exclusive(&self) -> Range<u32> {
        match self {
            MemoryRegion::Preloader => PRELOADER_ADDR_RANGE,
            MemoryRegion::Bootloader => BOOTLOADER_ADDR_RANGE,
            MemoryRegion::BootloaderScratch => BOOTLOADER_SCRATCH_ADDR_RANGE,
            MemoryRegion::Application => APP_ADDR_RANGE,
        }
    }

    pub const fn blocks_8k(&self) -> u32 {
        let range = self.range_exclusive();
        (range.end - range.start) / SECTOR_SIZE
    }

    pub const fn start_addr(&self) -> u32 {
        self.range_exclusive().start
    }

    pub const fn size_bytes(&self) -> u32 {
        self.blocks_8k() * SECTOR_SIZE
    }
}

// Ensure everything is aligned to per-page
static_assertions::const_assert!(PRELOADER_ADDR_RANGE.start.is_multiple_of(SECTOR_SIZE));
static_assertions::const_assert!(BOOTLOADER_ADDR_RANGE.start.is_multiple_of(SECTOR_SIZE));
static_assertions::const_assert!(APP_ADDR_RANGE.start.is_multiple_of(SECTOR_SIZE));
