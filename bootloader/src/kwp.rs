use core::{ptr::NonNull, sync::atomic::Ordering};

use atsamd_hal::{
    self,
    dsu::Dsu,
    fugit::ExtU64,
    nvm::{
        self, Nvm,
        smart_eeprom::{SmartEeprom, SmartEepromState, Unlocked},
    },
    pac::{Peripherals, dsu::did::Seriesselect},
    rtic_time::Monotonic,
    serial_number,
    trng::Trng,
    usb::UsbBus,
};
pub use automotive_diag::kwp2000::*;
use bootloader::bl_info::{CodeSectionInfo, MemoryRegion};
use cortex_m::peripheral::SCB;
use defmt::println;
use diag_common::{
    BootloaderStayReason,
    isotp_endpoints::usb_isotp::UsbIsoTpConsumer,
    ram_info::{BootloaderRamInfo, MAX_RESET_COUNT},
};
use usbd_serial::DefaultBufferStore;

use crate::{
    BS_EGS, ISOTP_BUF_SIZE, Mono, ST_MIN_EGS,
    bl_info::{self, region_crc},
};

#[derive(Copy, Clone)]
pub enum PendingOperation {
    None,
    Reset,
    FlashErase {
        start: u32,
        total_sectors: u32,
        current: u32,
    },
    Flashing {
        blk_id: u8,
        current_addr: u32,
    },
}

#[derive(Copy, Clone)]
pub enum CompletedOperation {
    FlashErase(Result<(), nvm::Error>),
}

pub const P2_MAX_MS: u64 = 2500;

const DEFAULT_SEC_MODE: SecurityLevel = SecurityLevel::FullUnlocked;

#[repr(u8)]
#[derive(defmt::Format, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SecurityLevel {
    Default = 1,
    Write = 3,
    Read = 5,
    FullUnlocked = 0xFE,
}

impl SecurityLevel {
    pub fn get_seed_key(&self, _trng: &Trng) -> SecuritySeedKey {
        todo!()
    }
}

#[repr(u8)]
#[derive(defmt::Format, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SecuritySeedKey {
    Default(u16, u16),
    AppWrite(u16, u16),
    AppRead(u32, u32),
    FullUnlocked(u64, u64),
}

pub struct KwpServer {
    buf: [u8; 4096],
    flash_buf: [u8; 4096],
    pub mode: KwpSessionType,
    pending_operation: PendingOperation,
    completed_operation: Option<CompletedOperation>,
    nvm: Nvm,
    dsu: Dsu,
    last_cmd_time: u64,
    sec_level: SecurityLevel,
    flash_size: u32,
    old_bl_info: BootloaderRamInfo,
    bl_reason: BootloaderStayReason,
    _rnd: Trng,
}

type ServerResult = core::result::Result<usize, KwpError>;

pub fn smart_eeprom(nvm: &mut Nvm) -> SmartEeprom<'_, Unlocked> {
    match nvm.smart_eeprom().unwrap() {
        nvm::smart_eeprom::SmartEepromMode::Locked(smart_eeprom) => smart_eeprom.unlock(),
        nvm::smart_eeprom::SmartEepromMode::Unlocked(smart_eeprom) => smart_eeprom,
    }
}

impl KwpServer {
    pub fn new(
        mut nvm: Nvm,
        rnd: Trng,
        dsu: Dsu,
        bootloader_ram_info: BootloaderRamInfo,
        bl_reason: BootloaderStayReason,
    ) -> Self {
        let flash_size = atsamd_hal::nvm::retrieve_flash_size();
        Self {
            mode: KwpSessionType::Normal,
            pending_operation: PendingOperation::None,
            completed_operation: None,
            buf: [0; 4096],
            flash_buf: [0; 4096],
            nvm,
            dsu,
            last_cmd_time: 0,
            sec_level: DEFAULT_SEC_MODE,
            flash_size,
            old_bl_info: bootloader_ram_info,
            bl_reason,
            _rnd: rnd,
        }
    }
    pub fn check_addr(&mut self, v: u32, reading: bool) -> Result<(), KwpError> {
        let (min_sec_level, can_read) = match v {
            // Code space (Bootloader)
            0x00000000..0x00010000 => (SecurityLevel::FullUnlocked, true),
            // Code space (Application)
            //0x00010000..0x00100000 => (SecurityLevel::AppRead, true),
            x if (0x00010000..self.flash_size).contains(&x) => {
                let min = if reading {
                    SecurityLevel::Read
                } else {
                    SecurityLevel::Write
                };
                (min, true)
            }
            // CMCC
            0x03000000..0x04000000 => (SecurityLevel::FullUnlocked, true),
            // QSPI disabled (So not allowed to read)
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
        } else if self.sec_level < min_sec_level {
            Err(KwpError::SecurityAccessDenied)
        } else {
            Ok(())
        }
    }

    pub async fn update(&mut self, now_ms: u64) -> Option<&[u8]> {
        if now_ms - self.last_cmd_time > P2_MAX_MS && self.mode != KwpSessionType::Normal {
            defmt::debug!("Tester timeout. Going back to default mode");
            self.mode = KwpSessionType::Normal;
            self.pending_operation = PendingOperation::None;
            self.sec_level = DEFAULT_SEC_MODE;
        }
        match &mut self.pending_operation {
            PendingOperation::Reset => {
                Mono::delay(10u64.millis()).await;
                SCB::sys_reset();
            }
            PendingOperation::FlashErase {
                start,
                total_sectors,
                current,
            } => {
                let addr = (*start + (8192 * *current)) as *mut u32;
                match unsafe { self.nvm.erase_flash(addr, 1) } {
                    Ok(_) => {
                        *current += 1;
                        if *total_sectors == *current {
                            self.pending_operation = PendingOperation::None;
                            self.completed_operation = Some(CompletedOperation::FlashErase(Ok(())))
                        }
                    }
                    Err(e) => {
                        self.pending_operation = PendingOperation::None;
                        self.completed_operation = Some(CompletedOperation::FlashErase(Err(e)));
                    }
                }
                None
            }
            _ => None,
        }
    }

    pub fn make_nrc(&mut self, sid: u8, nrc: impl Into<u8>) -> usize {
        self.buf[0..3].copy_from_slice(&[0x7F, sid, nrc.into()]);
        3
    }

    pub fn make_positive_reply(&mut self, sid: u8, data: &[u8]) -> usize {
        self.buf[0] = sid + 0x40;
        self.buf[1..1 + data.len()].copy_from_slice(data);
        1 + data.len()
    }

    pub fn process_cmd<'a>(&'a mut self, cmd: &[u8], now_ms: u64) -> &'a [u8] {
        self.last_cmd_time = now_ms;
        let r = if let PendingOperation::FlashErase { .. } = self.pending_operation {
            Err(KwpError::BusyRepeatRequest)
        } else {
            match KwpCommand::try_from(cmd[0]).ok() {
                Some(KwpCommand::ECUReset) => self.ecu_reset(cmd),
                Some(KwpCommand::StartDiagnosticSession) => self.start_diag_session(cmd),
                Some(KwpCommand::ReadMemoryByAddress) => self.read_mem_by_address(cmd),
                Some(KwpCommand::RequestDownload) => self.start_download(cmd),
                Some(KwpCommand::TesterPresent) => self.tester_present(cmd),
                Some(KwpCommand::RequestTransferExit) => self.transfer_exit(cmd),
                Some(KwpCommand::StartRoutineByLocalIdentifier) => self.routine_start(cmd),
                Some(KwpCommand::ReadECUIdentification) => self.ecu_ident(cmd),
                Some(KwpCommand::ReadDataByLocalIdentifier) => self.read_data_local_ident(cmd),
                Some(KwpCommand::RequestRoutineResultsByLocalIdentifier) => {
                    self.routine_results(cmd)
                }
                Some(KwpCommand::TransferData) => self.transfer_data(cmd),
                _ => Err(KwpError::ServiceNotSupported),
            }
        };
        let reply_len = r.unwrap_or_else(|nrc| self.make_nrc(cmd[0], nrc));
        defmt::debug!(
            "KWP Reponse length {} {:02X}",
            reply_len,
            self.buf[..reply_len]
        );
        &self.buf[..reply_len]
    }

    fn start_diag_session(&mut self, cmd: &[u8]) -> ServerResult {
        if cmd.len() != 2 && cmd.len() != 4 {
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else {
            fn set_com_param(cmd: &[u8]) {
                if cmd.len() == 4 {
                    let bs = cmd[2];
                    let stmin = cmd[3];
                    ST_MIN_EGS.store(stmin, Ordering::Relaxed);
                    BS_EGS.store(bs, Ordering::Relaxed);
                }
            }

            match KwpSessionType::try_from(cmd[1]).ok() {
                Some(KwpSessionType::Reprogramming) => {
                    self.mode = KwpSessionType::Reprogramming;
                }
                Some(KwpSessionType::ExtendedDiagnostics) => {
                    self.mode = KwpSessionType::ExtendedDiagnostics;
                }
                Some(KwpSessionType::Normal) => {
                    self.mode = KwpSessionType::Normal;
                }
                _ => return Err(KwpError::SubFunctionNotSupportedInvalidFormat),
            }
            set_com_param(cmd);
            Ok(self.make_positive_reply(cmd[0], &[cmd[1]]))
        }
    }

    fn ecu_reset(&mut self, cmd: &[u8]) -> ServerResult {
        if self.mode != KwpSessionType::Reprogramming {
            return Err(KwpError::ServiceNotSupportedInActiveSession);
        }
        if cmd.len() != 2 {
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else if cmd[1] == 0x01 {
            self.pending_operation = PendingOperation::Reset;
            Ok(self.make_positive_reply(cmd[0], &[cmd[1]]))
        } else {
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        }
    }

    fn tester_present(&mut self, cmd: &[u8]) -> ServerResult {
        Ok(self.make_positive_reply(cmd[0], &[]))
    }

    fn read_mem_by_address(&mut self, cmd: &[u8]) -> ServerResult {
        if self.mode != KwpSessionType::Reprogramming
            && self.mode != KwpSessionType::ExtendedDiagnostics
        {
            return Err(KwpError::ServiceNotSupportedInActiveSession);
        }
        if cmd.len() != 6 {
            // 1 byte for len and 4 bytes for addr
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else {
            let len: usize = cmd[1] as usize;
            let addr = u32::from_le_bytes(cmd[2..6].try_into().unwrap());
            self.check_addr(addr, true)?;
            self.check_addr(addr + len as u32 - 1, true)?;

            unsafe {
                let mut buf = [0u8; 0xFF];
                let dest_ptr = buf.as_mut_ptr();

                let ptr = core::ptr::NonNull::new_unchecked(addr as *mut u8);
                ptr.copy_to_nonoverlapping(NonNull::new_unchecked(dest_ptr), len);
                Ok(self.make_positive_reply(cmd[0], &buf[..len]))
            }
        }
    }

    fn read_data_local_ident(&mut self, cmd: &[u8]) -> ServerResult {
        if cmd.len() != 2 {
            // 1 byte for ID type
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else {
            if cmd[1] == 0xE1 {
                let sn = serial_number();
                let mut res = [0; 17];
                res[0] = 0xE1;
                res[1..].copy_from_slice(&sn);
                Ok(self.make_positive_reply(cmd[0], &res))
            } else if cmd[1] == 0xE2 {
                // Enter bootloader reason
                let mut res = [0; 2];
                res[0] = 0xE2;
                res[1] = self.bl_reason as u8;
                Ok(self.make_positive_reply(cmd[0], &res))
            } else if cmd[1] == 0xE3 {
                // Panic message
                let mut res = [0u8; 512];
                res[0] = 0xE3;
                if let Some(panic_info) = self.old_bl_info.app_panic {
                    let len = core::cmp::min(panic_info.msg().len(), 511);
                    res[1..1 + len].copy_from_slice(&panic_info.msg().as_bytes()[..len]);
                    Ok(self.make_positive_reply(cmd[0], &res[..1 + len]))
                } else {
                    Ok(self.make_positive_reply(cmd[0], &res[..2]))
                }
            } else if cmd[1] == 0xE4 {
                // Panic location
                let mut res = [0u8; 512];
                res[0] = 0xE4;
                if let Some(panic_info) = self.old_bl_info.app_panic {
                    if let Some(loc) = panic_info.file() {
                        let len = core::cmp::min(loc.file_name.len(), 511 - 8);
                        res[1..5].copy_from_slice(&loc.col.to_le_bytes());
                        res[5..9].copy_from_slice(&loc.line.to_le_bytes());
                        res[9..9 + len].copy_from_slice(&loc.file_name.as_bytes()[..len]);
                        Ok(self.make_positive_reply(cmd[0], &res[..1 + len + 8]))
                    } else {
                        Ok(self.make_positive_reply(cmd[0], &res[..2]))
                    }
                } else {
                    Ok(self.make_positive_reply(cmd[0], &res[..2]))
                }
            } else {
                Err(KwpError::SubFunctionNotSupportedInvalidFormat)
            }
        }
    }

    fn ecu_ident(&mut self, cmd: &[u8]) -> ServerResult {
        if cmd.len() != 2 {
            // 1 byte for ID type
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else {
            fn bcd_encode(v: u8) -> u8 {
                let tens = v / 10;
                let remain = v % 10;
                tens << 4 | remain
            }
            if cmd[1] == 0x8A {
                // Return all the block information on the ECU
                let eeprom = smart_eeprom(&mut self.nvm);
                let info = bl_info::get_smarteeprom_info(&eeprom);

                // 9 = SID ID + CRC BL(4) + CRC APP(4)
                let mut response = [0; 9 + core::mem::size_of::<CodeSectionInfo>() * 3];
                response[0] = 0x8A;

                let mut offset = 1;
                for blk in &[
                    info.preloader_info,
                    info.bootloader_info,
                    info.firmware_info,
                ] {
                    let slice = unsafe {
                        let ptr = blk as *const _ as *const u8;
                        core::slice::from_raw_parts(ptr, core::mem::size_of::<CodeSectionInfo>())
                    };
                    response[offset..offset + slice.len()].copy_from_slice(slice);
                    offset += slice.len();
                }
                // Copy the 2 CRCs
                response[offset..offset + 4].copy_from_slice(&info.crc32_bl.to_le_bytes());
                response[offset + 4..offset + 8].copy_from_slice(&info.crc32_app.to_le_bytes());

                Ok(self.make_positive_reply(cmd[0], &response))
            } else if cmd[1] == 0x86 {
                let eeprom = smart_eeprom(&mut self.nvm);
                let info = bl_info::get_smarteeprom_info(&eeprom);

                if info.is_production_date_set() {
                    let mut response = [0; 17];
                    response[0] = 0x86;
                    response[6] = bcd_encode(01);
                    response[7] = bcd_encode(26);
                    response[8] = bcd_encode(info.bootloader_info.compile_week);
                    response[9] = bcd_encode(info.bootloader_info.compile_year);
                    response[10] = 0x08; // ECU Origin (siemens)
                    response[11] = 0x02; // EGS52
                    if info.bootloader_info.is_debug != 0 {
                        response[11] |= 0b1000_0000;
                    }
                    response[12] = 0xE1; // Diag version low byte
                    response[14] = bcd_encode(info.board_prod_year);
                    response[15] = bcd_encode(info.board_prod_month);
                    response[16] = bcd_encode(info.board_prod_day);
                    Ok(self.make_positive_reply(cmd[0], &response))
                } else {
                    // No production data
                    Err(KwpError::ConditionsNotCorrectRequestSequenceError)
                }
            } else if cmd[1] == 0x87 {
                let eeprom = smart_eeprom(&mut self.nvm);
                let info = bl_info::get_smarteeprom_info(&eeprom);
                let mut response = [0; 21];
                response[0] = 0x87;
                response[1] = 0x08; // ECU Origin (Siemens)
                response[2] = 0x00; // Supplier
                response[3] = 0x02; // EGS52
                #[cfg(debug_assertions)]
                {
                    // Set development bit if this is a debug build
                    response[3] |= 0b1000_0000;
                }
                response[4] = 0xE1; // Diag version low byte

                // HW Version
                response[6] = 2;
                response[7] = 0;
                // SW Version
                response[8] = info.bootloader_info.version_major;
                response[9] = info.bootloader_info.version_minor;
                response[10] = info.bootloader_info.version_patch;
                response[11..21].copy_from_slice("1234567890".as_bytes());
                Ok(self.make_positive_reply(cmd[0], &response))
            } else {
                Err(KwpError::SubFunctionNotSupportedInvalidFormat)
            }
        }
    }

    fn routine_start(&mut self, cmd: &[u8]) -> ServerResult {
        if self.mode != KwpSessionType::Reprogramming {
            return Err(KwpError::ServiceNotSupportedInActiveSession);
        }
        // We want 2 bytes for number of 8192 blocks (LE)
        // 4 bytes for start address (LE)
        if cmd.len() < 2 {
            // At least 1 arg for LID
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else if cmd[1] == 0x24 {
            if cmd.len() != 6 {
                return Err(KwpError::SubFunctionNotSupportedInvalidFormat);
            }
            if self.sec_level != SecurityLevel::FullUnlocked {
                return Err(KwpError::SecurityAccessDenied);
            }
            let mut eeprom = smart_eeprom(&mut self.nvm);
            let curr_info = bl_info::get_smarteeprom_info(&eeprom);
            // Day, Week, Month, year
            if cmd[2] > 31 || cmd[3] > 52 || cmd[4] > 12 || cmd[5] < 24 {
                Err(KwpError::SubFunctionNotSupportedInvalidFormat)
            } else if curr_info.is_production_date_set() {
                Err(KwpError::ConditionsNotCorrectRequestSequenceError)
            } else {
                bl_info::mutate_smarteeprom_info(&mut eeprom, |info| {
                    info.board_prod_day = cmd[2];
                    info.board_prod_week = cmd[3];
                    info.board_prod_month = cmd[4];
                    info.board_prod_year = cmd[5];
                });
                Ok(self.make_positive_reply(cmd[0], &[cmd[1]]))
            }
        } else if cmd[1] == 0xE0 {
            if cmd.len() != 8 {
                return Err(KwpError::SubFunctionNotSupportedInvalidFormat);
            }
            let mut start_addr = u32::from_le_bytes(cmd[2..6].try_into().unwrap());
            let num_blocks = u16::from_le_bytes(cmd[6..8].try_into().unwrap());

            if num_blocks == 0 {
                return Err(KwpError::SubFunctionNotSupportedInvalidFormat);
            }

            if start_addr == MemoryRegion::Bootloader.range_exclusive().start {
                start_addr = MemoryRegion::Application.range_exclusive().start;
            } else if start_addr >= MemoryRegion::Application.range_exclusive().start {
                // Mark app as erased now
                let mut eeprom = smart_eeprom(&mut self.nvm);
                bl_info::mutate_smarteeprom_info(&mut eeprom, |info| {
                    info.app_flashing_not_done = 0xFF;
                    info.crc32_app = 0xFFFF_FFFF
                });
            } else {
                return Err(KwpError::RequestOutOfRange);
            }

            // Do routine
            self.pending_operation = PendingOperation::FlashErase {
                start: start_addr,
                total_sectors: num_blocks as u32,
                current: 0,
            };
            Ok(self.make_positive_reply(cmd[0], &[cmd[1]]))
        } else if cmd[1] == 0xE1 {
            // Flash check routine [CRC32, Start Addr (4), End Addr (4)]
            if cmd.len() != 14 {
                return Err(KwpError::SubFunctionNotSupportedInvalidFormat);
            }
            let targ_crc = u32::from_le_bytes(cmd[2..6].try_into().unwrap());
            let mut start_addr = u32::from_le_bytes(cmd[6..10].try_into().unwrap());
            let len = u32::from_le_bytes(cmd[10..14].try_into().unwrap());
            let mut is_bootloader = false;

            if start_addr == MemoryRegion::Bootloader.start_addr() {
                start_addr = MemoryRegion::BootloaderScratch.start_addr();
                is_bootloader = true;
            }

            // Just check that addrs are valid
            self.check_addr(start_addr, false)?;
            self.check_addr(start_addr + len, false)?;
            let result = self.dsu.crc32(start_addr, len).unwrap_or(0);
            if result == targ_crc {
                let mut eeprom = smart_eeprom(&mut self.nvm);
                // CRC32 OK, now make a CRC of the entire app flash region, and write it to the app
                if is_bootloader {
                    bl_info::mutate_smarteeprom_info(&mut eeprom, |info| {
                        info.bl_flashing_pending = 0;
                        info.bootloader_info.clear();
                        info.crc32_bl = region_crc(
                            bl_info::MemoryRegion::BootloaderScratch.range_exclusive(),
                            &mut self.dsu,
                        )
                    });
                } else {
                    bl_info::mutate_smarteeprom_info(&mut eeprom, |info| {
                        info.app_flashing_not_done = 0;
                        info.firmware_info.clear();
                        info.crc32_app = region_crc(
                            bl_info::MemoryRegion::Application.range_exclusive(),
                            &mut self.dsu,
                        )
                    });
                }
                Ok(self.make_positive_reply(cmd[0], &[0xE1, 0x01]))
            } else {
                defmt::error!(
                    "CRC Failed: Target: 0x{:08X}, actual: 0x{:08X}",
                    targ_crc,
                    result
                );
                Ok(self.make_positive_reply(cmd[0], &[0xE1, 0x00]))
            }
        } else {
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        }
    }

    fn routine_results(&mut self, cmd: &[u8]) -> ServerResult {
        if self.mode != KwpSessionType::Reprogramming {
            return Err(KwpError::ServiceNotSupportedInActiveSession);
        }
        // We want 2 bytes for number of 8192 blocks (LE)
        // 4 bytes for start address (LE)
        if cmd.len() != 2 {
            // 1 arg (Always E0 = Flash Erase routine)
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else if cmd[1] != 0xE0 {
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else if let Some(completed) = &self.completed_operation {
            match (cmd[1], completed) {
                (0xE0, CompletedOperation::FlashErase(res)) => {
                    if let Err(e) = res {
                        defmt::error!("Flash erase error: {}", e);
                        self.completed_operation = None;
                        Ok(self.make_positive_reply(cmd[0], &[0xE0, 0x01]))
                    } else {
                        self.completed_operation = None;
                        Ok(self.make_positive_reply(cmd[0], &[0xE0, 0x00]))
                    }
                }
                _ => Err(KwpError::ConditionsNotCorrectRequestSequenceError),
            }
        } else {
            Err(KwpError::RoutineNotComplete)
        }
    }

    fn start_download(&mut self, cmd: &[u8]) -> ServerResult {
        if self.mode != KwpSessionType::Reprogramming {
            return Err(KwpError::ServiceNotSupportedInActiveSession);
        }
        if cmd.len() != 10 {
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else {
            // 0..2 -> Address
            // 3 -> Format (00 is only supported)
            // 4..8 -> Size
            let mut addr = u32::from_le_bytes(cmd[1..5].try_into().unwrap());
            let fmt = cmd[5];
            let size = u32::from_le_bytes(cmd[6..10].try_into().unwrap());
            let app_region = MemoryRegion::Application.range_exclusive();

            if addr == MemoryRegion::Bootloader.start_addr() {
                addr = MemoryRegion::BootloaderScratch.start_addr();
            } else if fmt != 0
                || !app_region.contains(&addr)
                || !app_region.contains(&(addr + size))
            {
                return Err(KwpError::SubFunctionNotSupportedInvalidFormat);
            }
            // Valid params, lets start flashing
            const BLOCK_SIZE: u16 = 1024;
            let bs = [(BLOCK_SIZE >> 8) as u8, (BLOCK_SIZE & 0xFF) as u8];
            self.pending_operation = PendingOperation::Flashing {
                blk_id: 0,
                current_addr: addr,
            };
            Ok(self.make_positive_reply(cmd[0], &bs))
        }
    }

    fn transfer_data(&mut self, cmd: &[u8]) -> ServerResult {
        if let PendingOperation::Flashing {
            blk_id,
            current_addr,
        } = &mut self.pending_operation
        {
            if cmd.len() > 2 {
                let req_blk_id = cmd[1];
                let data_size = cmd.len() - 2;
                if req_blk_id == *blk_id && data_size % 4 == 0 {
                    let addr = *current_addr as *mut u32;
                    // Copy to 4 byte aligned array
                    self.flash_buf[..cmd.len() - 2].copy_from_slice(&cmd[2..]);
                    // Write to aligned
                    unsafe {
                        let source: &[u32] = core::slice::from_raw_parts(
                            self.flash_buf.as_ptr() as *const u32,
                            data_size / 4,
                        );
                        if self
                            .nvm
                            .write_flash_from_slice(addr, source, nvm::WriteGranularity::QuadWord)
                            .is_err()
                        {
                            Err(KwpError::TransferSuspended)?;
                        }
                        *current_addr += data_size as u32;
                        *blk_id += 1;
                        Ok(self.make_positive_reply(cmd[0], &[0x00]))
                    }
                } else {
                    // Mismatch
                    Err(KwpError::TransferSuspended)
                }
            } else {
                Err(KwpError::SubFunctionNotSupportedInvalidFormat)
            }
        } else {
            Err(KwpError::ConditionsNotCorrectRequestSequenceError)
        }
    }

    fn transfer_exit(&mut self, cmd: &[u8]) -> ServerResult {
        if let PendingOperation::Flashing { .. } = &mut self.pending_operation {
            self.pending_operation = PendingOperation::None;
            Ok(self.make_positive_reply(cmd[0], &[0]))
        } else {
            Err(KwpError::ConditionsNotCorrectRequestSequenceError)
        }
    }
}
