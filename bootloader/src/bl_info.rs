#![allow(dead_code)]
use core::ops::Range;

use atsamd_hal::{
    aes::typenum::P196,
    dsu::Dsu,
    nvm::{
        self, Nvm,
        smart_eeprom::{SmartEeprom, SmartEepromState, Unlocked},
    },
};
use konst::{for_range, parsing::Parser, result};

const KB: u32 = 1024;
const SECTOR_SIZE: u32 = 8192;

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
}

// Ensure everything is aligned to per-page
static_assertions::const_assert!(PRELOADER_ADDR_RANGE.start % SECTOR_SIZE == 0);
static_assertions::const_assert!(BOOTLOADER_ADDR_RANGE.start % SECTOR_SIZE == 0);
static_assertions::const_assert!(APP_ADDR_RANGE.start % SECTOR_SIZE == 0);

#[derive(defmt::Format, Clone, Copy)]
pub struct SmartEepromInfo {
    /// Board production day - Burned to the device via bootloader
    pub board_prod_day: u8,
    /// Board production month - Burned to the device via bootloader
    pub board_prod_month: u8,
    /// Board production week - Burned to the device via bootloader
    pub board_prod_week: u8,
    /// Board production yyear - Burned to the device via bootloader
    pub board_prod_year: u8,
    pub preloader_info: CodeSectionInfo,
    pub bootloader_info: CodeSectionInfo,
    pub firmware_info: CodeSectionInfo,
    /// 0 means flashing complete, any other value - Flashing not compelted
    pub app_flashing_not_done: u8,
    /// 0 means flashing pending
    pub bl_flashing_pending: u8,
    /// CRC32 of app region
    pub crc32_app: u32,
    /// CRC32 of Bootloader region
    pub crc32_bl: u32,
}

impl SmartEepromInfo {
    pub fn is_production_date_set(&self) -> bool {
        self.board_prod_day != 0xFF
            && self.board_prod_month != 0xFF
            && self.board_prod_week != 0xFF
            && self.board_prod_year != 0xFF
    }
}

const SMART_EEPROM_INFO_SIZE: usize = core::mem::size_of::<SmartEepromInfo>();

#[derive(defmt::Format, Clone, Copy)]
#[repr(C, packed)]
pub struct CodeSectionInfo {
    pub name: [u8; 10],
    pub git_sha: [u8; 12],
    pub version_major: u8,
    pub version_minor: u8,
    pub version_patch: u8,

    pub rustc_version_major: u8,
    pub rustc_version_minor: u8,
    pub rustc_version_patch: u8,

    pub compile_year: u8,
    pub compile_month: u8,
    pub compile_week: u8,
    pub compile_day: u8,
    pub is_debug: u8,
}

impl CodeSectionInfo {
    pub fn clear(&mut self) {
        self.version_major = 0xFF;
        self.version_minor = 0xFF;
        self.version_patch = 0xFF;
        self.rustc_version_major = 0xFF;
        self.rustc_version_minor = 0xFF;
        self.rustc_version_patch = 0xFF;
        self.compile_day = 0xFF;
        self.compile_month = 0xFF;
        self.compile_week = 0xFF;
        self.compile_year = 0xFF;
    }
}

impl PartialEq for CodeSectionInfo {
    fn eq(&self, other: &Self) -> bool {
        self.name == other.name
            && self.version_major == other.version_major
            && self.version_minor == other.version_minor
            && self.version_patch == other.version_patch
            && self.rustc_version_major == other.rustc_version_major
            && self.rustc_version_minor == other.rustc_version_minor
            && self.rustc_version_patch == other.rustc_version_patch
            && self.compile_year == other.compile_year
            && self.compile_month == other.compile_month
            && self.compile_week == other.compile_week
            && self.compile_day == other.compile_day
    }
}

pub const fn parse_u8(s: &str) -> u8 {
    let mut p = Parser::new(s);
    result::unwrap!(p.parse_u8())
}

const GIT_SHA_LEN: usize = 12;
pub const fn parse_git_sha(s: &str) -> [u8; GIT_SHA_LEN] {
    let mut out = [0; GIT_SHA_LEN];

    let mut chars = konst::string::chars(s);
    for_range! { i in 0..GIT_SHA_LEN =>
        out[i] = chars.next().unwrap() as u8
    }
    out
}

#[cfg(not(feature = "lib"))]
pub const fn create_code_info(name: [u8; 10]) -> CodeSectionInfo {
    CodeSectionInfo {
        name,
        git_sha: parse_git_sha(env!("VERGEN_GIT_SHA")),
        version_major: parse_u8(env!("CARGO_PKG_VERSION_MAJOR")),
        version_minor: parse_u8(env!("CARGO_PKG_VERSION_MINOR")),
        version_patch: parse_u8(env!("CARGO_PKG_VERSION_PATCH")),
        compile_year: parse_u8(env!("BUILD_YEAR")),
        compile_month: parse_u8(env!("BUILD_MONTH")),
        compile_week: parse_u8(env!("BUILD_WEEK")),
        compile_day: parse_u8(env!("BUILD_DAY")),
        rustc_version_major: parse_u8(env!("RUSTC_VER_MAJOR")),
        rustc_version_minor: parse_u8(env!("RUSTC_VER_MINOR")),
        rustc_version_patch: parse_u8(env!("RUSTC_VER_PATCH")),
        #[cfg(debug_assertions)]
        is_debug: 1,
        #[cfg(not(debug_assertions))]
        is_debug: 0,
    }
}

#[inline(always)]
pub fn region_crc(addr_range: Range<u32>, dsu: &mut Dsu) -> u32 {
    dsu.crc32(addr_range.start, addr_range.len() as u32)
        .unwrap_or(0)
}

/// Modify the bootloader info page in NVM memory
///
/// # Safety
/// This function erases the info page and then writes it back, therefore
/// if power loss occurs during erase, then the bootloader info section
/// will be corrupt
pub fn mutate_smarteeprom_info<'a, F: FnOnce(&mut SmartEepromInfo)>(
    seeprom: &mut SmartEeprom<'a, Unlocked>,
    f: F,
) -> SmartEepromInfo {
    let mut s = [0u8; SMART_EEPROM_INFO_SIZE];
    seeprom.get(0, &mut s);
    let ptr_s = s.as_mut_ptr() as *mut SmartEepromInfo;
    let mut_s = unsafe { ptr_s.as_mut().unwrap() };
    f(mut_s);
    seeprom.set(0, &s);
    *mut_s
}

pub fn get_smarteeprom_info<'a, T: SmartEepromState>(
    seeprom: &SmartEeprom<'a, T>,
) -> SmartEepromInfo {
    let mut s = [0u8; SMART_EEPROM_INFO_SIZE];
    seeprom.get(0, &mut s);
    let ptr_s = s.as_mut_ptr() as *mut SmartEepromInfo;
    let ref_s = unsafe { ptr_s.as_ref().unwrap() };
    *ref_s
}
