use core::{ptr::NonNull, sync::atomic::Ordering};

use crate::{BS_EGS, Mono, ST_MIN_EGS};
use atsamd_hal::{
    self,
    fugit::ExtU64,
    nvm::{
        self, Nvm,
        smart_eeprom::{SmartEeprom, Unlocked},
    },
    rtic_time::Monotonic,
    serial_number,
    trng::Trng,
};
pub use automotive_diag::kwp2000::*;
use cortex_m::peripheral::SCB;
use defmt::println;
use diag_common::{
    BootloaderStayReason, MemoryRegion,
    hal_extensions::dsu::Dsu,
    qspi_driver::QspiStorage,
    ram_info::BootloaderRamInfo,
    smarteeprom::{CodeSectionInfo, get_smarteeprom_info, mutate_smarteeprom_info},
};
use diag_common::{
    QSPI_AHB,
    diag_core::{MemCfg, SecurityLevel, check_mem_addr},
};

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum PendingOperation {
    None,
    Reset,
    FlashErase {
        start: u32,
        total_sectors: u32,
        current: u32,
    },
    QspiErase {
        start: u32,
        total_sectors: u32,
        current: u32,
    },
    Flashing {
        blk_id: u8,
        current_addr: u32,
        use_compression: bool,
        qspi: bool,
    },
}

#[derive(Copy, Clone)]
pub enum CompletedOperation {
    FlashErase(Result<(), nvm::Error>),
    QspiErase(bool),
}

pub const P2_MAX_MS: u64 = 2500;

const DEFAULT_SEC_MODE: SecurityLevel = SecurityLevel::FullUnlocked;

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
    qspi: QspiStorage,
    last_cmd_time: u64,
    long_op: bool,
    sec_level: SecurityLevel,
    flash_config: MemCfg,
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
        nvm: Nvm,
        rnd: Trng,
        dsu: Dsu,
        qspi: QspiStorage,
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
            qspi,
            last_cmd_time: 0,
            long_op: false,
            sec_level: DEFAULT_SEC_MODE,
            flash_config: MemCfg {
                flash_size,
                qspi_size: 0,
            },
            old_bl_info: bootloader_ram_info,
            bl_reason,
            _rnd: rnd,
        }
    }

    pub async fn update(&mut self, now_ms: u64) -> Option<&[u8]> {
        if self.qspi.size_bytes().is_none() {
            self.qspi.init_chip(&mut Mono).await;
            if let Some(size) = self.qspi.size_bytes() {
                self.flash_config.qspi_size = size;
            }
        }
        if now_ms - self.last_cmd_time > P2_MAX_MS
            && self.mode != KwpSessionType::Normal
            && !self.long_op
        {
            defmt::debug!("Tester timeout. Going back to default mode");
            self.mode = KwpSessionType::Normal;
            self.pending_operation = PendingOperation::None;
            self.completed_operation = None;
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
            PendingOperation::QspiErase {
                start,
                total_sectors,
                current,
            } => {
                const BLOCK_32_KB: u32 = 32 * 1024;
                let addr = *start + (4096 * *current);
                // We can speed this up by doing 32K erase if possible
                let (res, inc) =
                    if addr % BLOCK_32_KB == 0 && (total_sectors.saturating_sub(*current)) >= 8 {
                        (self.qspi.erase_32k_block(addr, &mut Mono).await, 8)
                    } else {
                        (self.qspi.erase_4k_sector(addr, &mut Mono).await, 1)
                    };
                match res {
                    true => {
                        *current += inc;
                        if *current >= *total_sectors {
                            self.pending_operation = PendingOperation::None;
                            self.completed_operation = Some(CompletedOperation::QspiErase(true))
                        }
                    }
                    false => {
                        self.pending_operation = PendingOperation::None;
                        self.completed_operation = Some(CompletedOperation::QspiErase(false));
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

    pub async fn process_cmd<'a>(&'a mut self, cmd: &[u8], now_ms: u64) -> &'a [u8] {
        self.last_cmd_time = now_ms;
        self.long_op = false;
        defmt::debug!("Kwp req: {:02X}..", cmd[..core::cmp::min(cmd.len(), 5)]);
        let r = match KwpCommand::try_from(cmd[0]).ok() {
            Some(KwpCommand::ECUReset) => self.ecu_reset(cmd),
            Some(KwpCommand::StartDiagnosticSession) => self.start_diag_session(cmd),
            Some(KwpCommand::ReadMemoryByAddress) => self.read_mem_by_address(cmd),
            Some(KwpCommand::RequestDownload) => self.start_download(cmd),
            Some(KwpCommand::TesterPresent) => self.tester_present(cmd),
            Some(KwpCommand::RequestTransferExit) => self.transfer_exit(cmd),
            Some(KwpCommand::StartRoutineByLocalIdentifier) => self.routine_start(cmd),
            Some(KwpCommand::ReadECUIdentification) => self.ecu_ident(cmd),
            Some(KwpCommand::ReadDataByLocalIdentifier) => self.read_data_local_ident(cmd),
            Some(KwpCommand::RequestRoutineResultsByLocalIdentifier) => self.routine_results(cmd),
            Some(KwpCommand::TransferData) => self.transfer_data(cmd).await,
            _ => Err(KwpError::ServiceNotSupported),
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
            check_mem_addr(self.flash_config, self.sec_level, addr, true)?;
            check_mem_addr(
                self.flash_config,
                self.sec_level,
                addr + len as u32 - 1,
                true,
            )?;
            // QSPI handling
            if (0x04000000u32..0x05000000).contains(&addr) {
                let mut buf = [0; 0xFF];
                let b_addr = addr - 0x04000000u32;
                self.qspi.read(b_addr, &mut buf[..len]);
                Ok(self.make_positive_reply(cmd[0], &buf[..len]))
            } else {
                unsafe {
                    let mut buf = [0u8; 0xFF];
                    let dest_ptr = buf.as_mut_ptr();

                    let ptr = core::ptr::NonNull::new_unchecked(addr as *mut u8);
                    ptr.copy_to_nonoverlapping(NonNull::new_unchecked(dest_ptr), len);
                    Ok(self.make_positive_reply(cmd[0], &buf[..len]))
                }
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
                let info = get_smarteeprom_info(&eeprom);

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
                let info = get_smarteeprom_info(&eeprom);

                if info.is_production_date_set() {
                    let mut response = [0; 17];
                    response[0] = 0x86;
                    response[6] = bcd_encode(1);
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
                let info = get_smarteeprom_info(&eeprom);
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
        if self.pending_operation != PendingOperation::None {
            return Err(KwpError::ConditionsNotCorrectRequestSequenceError);
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
            let curr_info = get_smarteeprom_info(&eeprom);
            // Day, Week, Month, year
            if cmd[2] > 31 || cmd[3] > 52 || cmd[4] > 12 || cmd[5] < 24 {
                Err(KwpError::SubFunctionNotSupportedInvalidFormat)
            } else if curr_info.is_production_date_set() {
                Err(KwpError::ConditionsNotCorrectRequestSequenceError)
            } else {
                mutate_smarteeprom_info(&mut eeprom, |info| {
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
            } else if start_addr >= MemoryRegion::Application.range_exclusive().start
                && start_addr < MemoryRegion::QspiFlash.range_exclusive().start
            {
                // Mark app as erased now
                let mut eeprom = smart_eeprom(&mut self.nvm);
                mutate_smarteeprom_info(&mut eeprom, |info| {
                    info.app_flashing_not_done = 0xFF;
                    info.crc32_app = 0xFFFF_FFFF
                });
            } else if MemoryRegion::QspiFlash
                .range_exclusive()
                .contains(&start_addr)
            {
                // Check that QSPI was initialized
                if self.flash_config.qspi_size == 0 {
                    return Err(KwpError::ConditionsNotCorrectRequestSequenceError);
                }
                // QSPI operation
                if start_addr % 4096 != 0 {
                    return Err(KwpError::SubFunctionNotSupportedInvalidFormat);
                }
            } else {
                return Err(KwpError::RequestOutOfRange);
            }

            if MemoryRegion::QspiFlash
                .range_exclusive()
                .contains(&start_addr)
            {
                // Do routine
                self.pending_operation = PendingOperation::QspiErase {
                    start: start_addr - MemoryRegion::QspiFlash.start_addr(),
                    total_sectors: (num_blocks * 2) as u32, // Convert to 4K Sector sizes
                    current: 0,
                };
            } else {
                // Do routine
                self.pending_operation = PendingOperation::FlashErase {
                    start: start_addr,
                    total_sectors: num_blocks as u32,
                    current: 0,
                };
            }
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
            check_mem_addr(self.flash_config, self.sec_level, start_addr, false)?;
            check_mem_addr(self.flash_config, self.sec_level, start_addr + len, false)?;
            let result = self.dsu.crc32(start_addr, len).unwrap_or(0);
            if result == targ_crc {
                let mut eeprom = smart_eeprom(&mut self.nvm);
                // CRC32 OK, now make a CRC of the entire app flash region, and write it to the app
                if is_bootloader {
                    mutate_smarteeprom_info(&mut eeprom, |info| {
                        info.bl_flashing_pending = 0;
                        info.bootloader_info.clear();
                        info.crc32_bl = self
                            .dsu
                            .crc32(
                                MemoryRegion::BootloaderScratch.start_addr(),
                                MemoryRegion::BootloaderScratch.size_bytes(),
                            )
                            .unwrap_or(0xFFFF_FFFF)
                    });
                } else {
                    mutate_smarteeprom_info(&mut eeprom, |info| {
                        info.app_flashing_not_done = 0;
                        info.firmware_info.clear();
                        info.crc32_app = self
                            .dsu
                            .crc32(
                                MemoryRegion::Application.start_addr(),
                                MemoryRegion::Application.size_bytes(),
                            )
                            .unwrap_or(0xFFFF_FFFF)
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
                (0xE0, CompletedOperation::QspiErase(ok)) => {
                    let res_byte = !*ok;
                    self.completed_operation = None;
                    Ok(self.make_positive_reply(cmd[0], &[0xE0, res_byte as u8]))
                }
                _ => Err(KwpError::ConditionsNotCorrectRequestSequenceError),
            }
        } else if self.pending_operation == PendingOperation::None {
            Err(KwpError::ConditionsNotCorrectRequestSequenceError)
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
            // 3 -> Format (00 and 01 is supported (Uncompressed, compressed with lz4))
            // 4..8 -> Size
            let mut addr = u32::from_le_bytes(cmd[1..5].try_into().unwrap());
            let fmt = cmd[5];
            let size = u32::from_le_bytes(cmd[6..10].try_into().unwrap());

            let is_qspi = MemoryRegion::QspiFlash.range_exclusive().contains(&addr)
                && MemoryRegion::QspiFlash
                    .range_exclusive()
                    .contains(&(addr + size));
            let is_app = MemoryRegion::Application.range_exclusive().contains(&addr)
                && MemoryRegion::Application
                    .range_exclusive()
                    .contains(&(addr + size));

            if addr == MemoryRegion::Bootloader.start_addr() {
                addr = MemoryRegion::BootloaderScratch.start_addr();
            } else if fmt > 1 || (!is_qspi && !is_app) {
                return Err(KwpError::SubFunctionNotSupportedInvalidFormat);
            }
            // Valid params, lets start flashing
            const BLOCK_SIZE: u16 = 1024;
            let bs = [(BLOCK_SIZE >> 8) as u8, (BLOCK_SIZE & 0xFF) as u8];
            self.pending_operation = PendingOperation::Flashing {
                blk_id: 0,
                current_addr: addr,
                use_compression: fmt != 0,
                qspi: is_qspi,
            };
            Ok(self.make_positive_reply(cmd[0], &bs))
        }
    }

    async fn transfer_data(&mut self, cmd: &[u8]) -> ServerResult {
        if let PendingOperation::Flashing {
            blk_id,
            current_addr,
            use_compression,
            qspi,
        } = &mut self.pending_operation
        {
            if cmd.len() > 2 {
                let req_blk_id = cmd[1];
                let mut data_size = cmd.len() - 2;
                if req_blk_id == *blk_id {
                    let addr = *current_addr as *mut u32;
                    if *use_compression {
                        if let Ok(decoded) =
                            heatshrink::decoder::decode(&cmd[2..], &mut self.flash_buf)
                        {
                            data_size = decoded.len();
                            if !data_size.is_multiple_of(4) {
                                return Err(KwpError::TransferSuspended)?;
                            }
                        } else {
                            defmt::error!("Failed to decode slice");
                            return Err(KwpError::TransferSuspended)?;
                        }
                    } else {
                        if !data_size.is_multiple_of(4) {
                            return Err(KwpError::TransferSuspended)?;
                        }
                        // Copy to 4 byte aligned array
                        self.flash_buf[..cmd.len() - 2].copy_from_slice(&cmd[2..]);
                    }
                    if *qspi {
                        self.long_op = true;
                        let qspi_addr = *current_addr - QSPI_AHB;
                        self.qspi
                            .write(qspi_addr, &self.flash_buf[..data_size], &mut Mono)
                            .await;

                        *current_addr += data_size as u32;
                        *blk_id += 1;
                        Ok(self.make_positive_reply(cmd[0], &[0x00]))
                    } else {
                        // Write to aligned
                        unsafe {
                            let source: &[u32] = core::slice::from_raw_parts(
                                self.flash_buf.as_ptr() as *const u32,
                                data_size / 4,
                            );
                            if self
                                .nvm
                                .write_flash_from_slice(
                                    addr,
                                    source,
                                    nvm::WriteGranularity::QuadWord,
                                )
                                .is_err()
                            {
                                Err(KwpError::TransferSuspended)?;
                            }
                            *current_addr += data_size as u32;
                            *blk_id += 1;
                            Ok(self.make_positive_reply(cmd[0], &[0x00]))
                        }
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
