use automotive_diag::kwp2000::KwpError;

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SecurityLevel {
    Default = 1,
    Write = 3,
    Read = 5,
    FullUnlocked = 0xFD,
}

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct MemCfg {
    pub flash_size: u32,
    pub qspi_size: u32,
}

pub fn check_mem_addr(
    mem_cfg: MemCfg,
    sec_level: SecurityLevel,
    v: u32,
    reading: bool,
) -> Result<(), KwpError> {
    let (min_sec_level, can_read) = match v {
        // Code space (Bootloader)
        0x00000000..0x00010000 => (SecurityLevel::FullUnlocked, true),
        // Code space (Application)
        //0x00010000..0x00100000 => (SecurityLevel::AppRead, true),
        x if (0x00010000..mem_cfg.flash_size).contains(&x) => {
            let min = if reading {
                SecurityLevel::Read
            } else {
                SecurityLevel::Write
            };
            (min, true)
        }
        // CMCC
        0x03000000..0x04000000 => (SecurityLevel::FullUnlocked, true),
        0x04000000..0x05000000 => {
            if v - 0x04000000 < mem_cfg.qspi_size {
                (SecurityLevel::Default, true)
            } else {
                (SecurityLevel::Default, false)
            }
        }
        // RAM
        0x20000000..0x20040000 => (SecurityLevel::Read, true),
        // AHB-APB Bridge A
        0x40000000..0x40004000 => (SecurityLevel::FullUnlocked, true),
        // AHB-APB Bridge B
        0x41000000..0x4100C000 => (SecurityLevel::FullUnlocked, true),
        0x4100E000..0x41010000 => (SecurityLevel::FullUnlocked, true),
        0x41012000..0x4101E000 => (SecurityLevel::FullUnlocked, true),
        0x41020000..0x41022000 => (SecurityLevel::FullUnlocked, true),
        // AHB-APB Bridge C
        0x42000000..0x42003C00 => (SecurityLevel::FullUnlocked, true),
        // AHB-APB Bridge D
        0x43000000..0x43003000 => (SecurityLevel::FullUnlocked, true),
        // Other AHB-APB systems
        0x44000000..0x48000000 => (SecurityLevel::FullUnlocked, true),
        // System
        0xE0000000..0xE000F000 => (SecurityLevel::FullUnlocked, true),
        0xE00FF000..0xE0100000 => (SecurityLevel::FullUnlocked, true),

        _ => (SecurityLevel::Default, false),
    };
    if !can_read {
        Err(KwpError::RequestOutOfRange)
    } else if sec_level < min_sec_level {
        Err(KwpError::SecurityAccessDenied)
    } else {
        Ok(())
    }
}
