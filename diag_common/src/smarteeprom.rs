//! Userpage information

#[cfg(feature="mcu")]
use atsamd_hal::nvm::smart_eeprom::*;

#[repr(C, packed(4))]
#[derive(Clone, Copy)]
#[cfg_attr(feature="mcu", derive(defmt::Format))]
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

static_assertions::const_assert!(size_of::<SmartEepromInfo>() < 512);

/// Modify the bootloader info page in NVM memory
///
/// # Safety
/// This function erases the info page and then writes it back, therefore
/// if power loss occurs during erase, then the bootloader info section
/// will be corrupt
#[cfg(feature="mcu")]
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

#[cfg(feature="mcu")]
pub fn get_smarteeprom_info<'a, T: SmartEepromState>(
    seeprom: &SmartEeprom<'a, T>,
) -> SmartEepromInfo {
    let mut s = [0u8; SMART_EEPROM_INFO_SIZE];
    seeprom.get(0, &mut s);
    let ptr_s = s.as_mut_ptr() as *mut SmartEepromInfo;
    let ref_s = unsafe { ptr_s.as_ref().unwrap() };
    *ref_s
}

#[derive(Clone, Copy)]
#[repr(C, packed(4))]
#[cfg_attr(feature="mcu", derive(defmt::Format))]
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