use atsamd_hal::{fugit::ExtU64, pac::SCB, rtic_time::Monotonic};
use automotive_diag::kwp2000::{KwpCommand, KwpError, KwpSessionType};
use core::ptr::NonNull;
use defmt::println;
use diag_common::defmt_multi_output;
use diag_common::diag_buffer::DiagBuffer;
use diag_common::diag_core::{MemCfg, SecurityLevel, check_mem_addr};
use diag_common::qspi_driver::QspiStorage;
use diag_common::{DefmtTarget, hal_extensions::dsu::Dsu, ram_info};
use egs_logic::TftReading;
use rtic::Mutex;
use rtic_sync::arbiter::Arbiter;

pub mod dev_mode;

use crate::{Mono, app::diag_task};

#[derive(Copy, Clone, Default)]
#[repr(C)]
pub struct SolenoidReport {
    pub tcc_pwm_target: u16,
    pub tcc_pwm_recorded: u16,
    pub pwm_target_y3: u16,
    pub pwm_target_y4: u16,
    pub pwm_target_y5: u16,
    pub current_target_mpc: u16,
    pub current_target_spc: u16,

    pub current_measured_y3: u16,
    pub current_measured_y4: u16,
    pub current_measured_y5: u16,
    pub current_measured_spc: u16,
    pub current_measured_mpc: u16,
    pub current_measured_trrs: u16,
    pub current_measured_gpio: u16,
}

#[derive(Copy, Clone, Default)]
pub struct PerfStatsTracker {
    pub cpu_percentage: u16,
    pub hw_interrupts: u16,
    pub wakeups: u16,
    pub us_input_funcs: u16,
    pub us_output_funcs: u16,
}

#[derive(Copy, Clone)]
pub enum PendingOp {
    None,
    Reboot,
}

pub struct KwpServer {
    last_cmd_time: u64,
    buf: DiagBuffer,
    pending_op: PendingOp,
    dsu: &'static Arbiter<Dsu>,
    qspi: &'static Arbiter<QspiStorage>,
    mode: KwpSessionType,
    flash_cfg: MemCfg,
    security_level: SecurityLevel,
}

#[repr(C)]
pub struct DiagEntry {
    pub pid: u8,
    pub addr_to_read: usize,
    pub bytes_to_read: usize,
}

#[macro_export]
macro_rules! new_diag_entry {
    ($pid: literal, $field: expr) => {{
        use core::ptr::addr_of;
        let addr = addr_of!($field) as usize;
        let size = size_of_val(&$field);
        DiagEntry {
            pid: $pid,
            addr_to_read: addr,
            bytes_to_read: size,
        }
    }};
}

type ServerResult = Result<usize, KwpError>;

impl KwpServer {
    pub fn new(dsu: &'static Arbiter<Dsu>, qspi: &'static Arbiter<QspiStorage>) -> Self {
        Self {
            last_cmd_time: 0,
            buf: DiagBuffer::new(),
            pending_op: PendingOp::None,
            dsu,
            qspi,
            mode: KwpSessionType::Normal,
            flash_cfg: MemCfg {
                flash_size: 1024 * 1024,
                qspi_size: 0,
            },
            security_level: SecurityLevel::FullUnlocked,
        }
    }

    pub async fn process_cmd(
        &mut self,
        cmd: &[u8],
        _now_ms: u64,
        shared: &mut diag_task::SharedResources<'_>,
    ) -> &[u8] {
        self.last_cmd_time = Mono::now().duration_since_epoch().to_millis();
        let r = match KwpCommand::try_from(cmd[0]).ok() {
            Some(KwpCommand::StartDiagnosticSession) => self.start_diag_session(cmd).await,
            Some(KwpCommand::ReadMemoryByAddress) => self.read_mem_by_address(cmd).await,
            Some(KwpCommand::ReadDataByLocalIdentifier) => self.read_data_local(cmd, shared).await,
            Some(KwpCommand::InputOutputControlByLocalIdentifier) => self.ioctl(cmd).await,
            _ => Err(KwpError::ServiceNotSupported),
        };

        let reply_len = r.unwrap_or_else(|nrc| self.buf.make_negative_reply(cmd[0], nrc));
        &self.buf.buf()[..reply_len]
    }

    async fn read_mem_by_address(&mut self, cmd: &[u8]) -> ServerResult {
        if cmd.len() != 6 {
            // 1 byte for len and 4 bytes for addr
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else {
            let len: usize = cmd[1] as usize;
            let addr = u32::from_le_bytes(cmd[2..6].try_into().unwrap());
            check_mem_addr(self.flash_cfg, self.security_level, addr, true)?;
            check_mem_addr(
                self.flash_cfg,
                self.security_level,
                addr + len as u32 - 1,
                true,
            )?;
            // QSPI handling
            if (0x04000000u32..0x05000000).contains(&addr) {
                let mut v = [0; 0xFF];
                let b_addr = addr - 0x04000000u32;
                self.qspi.access().await.read(b_addr, &mut v[..len]);
                Ok(self
                    .buf
                    .make_positive_reply(KwpCommand::ReadMemoryByAddress.into(), |buf| {
                        buf[..len].copy_from_slice(&v[..len]);
                        len
                    }))
            } else {
                unsafe {
                    Ok(self.buf.make_positive_reply(
                        KwpCommand::ReadMemoryByAddress.into(),
                        |buf| {
                            let dest_ptr = buf.as_mut_ptr();
                            let ptr = core::ptr::NonNull::new_unchecked(addr as *mut u8);
                            ptr.copy_to_nonoverlapping(NonNull::new_unchecked(dest_ptr), len);
                            len
                        },
                    ))
                }
            }
        }
    }

    async fn read_data_local(
        &mut self,
        cmd: &[u8],
        shared: &mut diag_task::SharedResources<'_>,
    ) -> ServerResult {
        if cmd.len() != 2 {
            return Err(KwpError::SubFunctionNotSupportedInvalidFormat);
        }

        fn copy_payload<T: Sized, const N: usize>(pid: u8, value: &T, buf: &mut [u8; N]) -> usize {
            const {
                assert!(N >= core::mem::size_of::<T>() + 1);
            }
            buf[0] = pid;
            unsafe {
                core::ptr::copy_nonoverlapping(
                    value as *const T as *const u8,
                    buf[1..].as_mut_ptr(),
                    core::mem::size_of::<T>() + 1,
                )
            }
            core::mem::size_of::<T>() + 1
        }

        match cmd[1] {
            0x00 => Ok(self.buf.make_positive_reply(cmd[0], |buf| {
                buf[0] = 0x00;
                shared.perf_stats.lock(|lck| {
                    buf[1..3].copy_from_slice(&lck.cpu_percentage.to_be_bytes());
                    buf[3..5].copy_from_slice(&lck.wakeups.to_be_bytes());
                    buf[5..7].copy_from_slice(&lck.hw_interrupts.to_be_bytes());
                    buf[7..9].copy_from_slice(&lck.us_input_funcs.to_be_bytes());
                    buf[9..11].copy_from_slice(&lck.us_output_funcs.to_be_bytes());
                });
                shared.gearbox.lock(|lck| {
                    let times = &lck.internal_vars().stat_times;
                    buf[11..13].copy_from_slice(&times.input_processing.to_be_bytes());
                    buf[13..15].copy_from_slice(&times.shift_actuation_logic.to_be_bytes());
                    buf[15..17].copy_from_slice(&times.torque_requests.to_be_bytes());
                    buf[17..19].copy_from_slice(&times.adaptation.to_be_bytes());
                    buf[19..21].copy_from_slice(&times.safety.to_be_bytes());
                    buf[21..23].copy_from_slice(&times.tcc_update.to_be_bytes());
                    buf[23..25].copy_from_slice(&times.shift_point_calc_logic.to_be_bytes());
                    buf[25..27].copy_from_slice(&times.output_processing.to_be_bytes());
                });
                27
            })),
            0x01 => Ok(self.buf.make_positive_reply(cmd[0], |buf| {
                buf[0] = 0x01;
                shared.gearbox.lock(|lck| {
                    let outputs = lck.outputs();
                    let vars = lck.internal_vars();
                    buf[1..3].copy_from_slice(&vars.mpc_pressure.to_be_bytes());
                    buf[3..5].copy_from_slice(&vars.spc_pressure.to_be_bytes());
                    buf[5..7].copy_from_slice(&outputs.mpc_current.to_be_bytes());
                    buf[7..9].copy_from_slice(&outputs.spc_current.to_be_bytes());
                });
                9
            })),
            0x02 => Ok(self.buf.make_positive_reply(cmd[0], |buf| {
                buf[0] = 0x02;
                shared.sensor_data.lock(|lck| {
                    buf[1] = (lck.t_pcb + 50) as u8;
                    buf[2] = (lck.t_tle + 50) as u8;
                    buf[3] = match lck.tft {
                        TftReading::ParkOrNeutral => 0xFF,
                        TftReading::Temperature(grad) => (grad + 50) as u8,
                    };
                    buf[4..6].copy_from_slice(&lck.vkl15.to_be_bytes());
                    buf[6..8].copy_from_slice(&lck.vkl87.to_be_bytes());
                    buf[8..10].copy_from_slice(&lck.vsense.to_be_bytes());
                    buf[10..12].copy_from_slice(&lck.ikl87.to_be_bytes());
                    buf[12..14].copy_from_slice(&lck.core_mv.to_be_bytes());
                    buf[14..16].copy_from_slice(&lck.io_mv.to_be_bytes());
                });
                16
            })),
            0x03 => shared.gearbox.lock(|lck| {
                Ok(self
                    .buf
                    .make_positive_reply(cmd[0], |buf| copy_payload(0x03, lck.outputs(), buf)))
            }),
            0x04 => shared.gearbox.lock(|lck| {
                Ok(self.buf.make_positive_reply(cmd[0], |buf| {
                    copy_payload(0x04, lck.internal_vars(), buf)
                }))
            }),
            0x05 => shared.solenoid_statuses.lock(|lck| {
                Ok(self
                    .buf
                    .make_positive_reply(cmd[0], |buf| copy_payload(0x05, lck, buf)))
            }),

            _ => {
                let mut egs_buf = [0; 0xFF];
                if let Some(resp_len) = shared
                    .gearbox
                    .lock(|lck| lck.diag_read_data_by_local_ident(cmd[1], &mut egs_buf[1..]))
                {
                    Ok(self.buf.make_positive_reply(cmd[0], |buf| {
                        buf[0] = cmd[1];
                        buf[1..1 + resp_len].copy_from_slice(&egs_buf[..resp_len]);
                        resp_len + 1
                    }))
                } else {
                    Err(KwpError::SubFunctionNotSupportedInvalidFormat)
                }
            }
        }
    }

    async fn ioctl(&mut self, cmd: &[u8]) -> ServerResult {
        if cmd.len() < 3 {
            return Err(KwpError::SubFunctionNotSupportedInvalidFormat);
        }
        match cmd[1] {
            // IOCTL ID
            // 0x01 => Variant coding (Siemens)
            // 0x10 => Device mode (Siemens)
            // 0x30 => Solenoid inspection (Siemens)

            // 0xF0 => Log mode (UN52)
            0xF0 => {
                match cmd[2] {
                    // Rpt type
                    // 0x00 => Return ctrl to ecu
                    // 0x01 => Report state
                    // 0x04 => Reset to default
                    // 0x07 => Short term adjust
                    0x01 => {
                        let log_ty = defmt_multi_output::get_current_defmt_log_mode() as u8;
                        Ok(self.buf.copy_positive_reply(cmd[0], &[0xF0, 0x01, log_ty]))
                    }
                    0x07 | 0x08 => {
                        if cmd.len() != 4 {
                            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
                        } else {
                            let log_mode = if cmd[3] == 0x00 {
                                DefmtTarget::Rtt
                            } else if cmd[3] == 0x01 {
                                DefmtTarget::Can
                            } else if cmd[3] == 0x02 {
                                DefmtTarget::Serial
                            } else {
                                return Err(KwpError::SubFunctionNotSupportedInvalidFormat);
                            };
                            if defmt_multi_output::set_defmt_log_mode(log_mode).is_err() {
                                Err(KwpError::ConditionsNotCorrectRequestSequenceError)
                            } else {
                                Ok(self.buf.copy_positive_reply(cmd[0], &[0xF0, cmd[3]]))
                            }
                        }
                    } // 0x08 => Long term adjust
                    _ => Err(KwpError::RequestOutOfRange),
                }
            }
            _ => Err(KwpError::SubFunctionNotSupportedInvalidFormat),
        }
    }

    async fn start_diag_session(&mut self, cmd: &[u8]) -> ServerResult {
        if cmd.len() != 2 && cmd.len() != 4 {
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else {
            match KwpSessionType::try_from(cmd[1]).ok() {
                Some(KwpSessionType::Reprogramming) => {
                    let mut dsu = self.dsu.access().await;
                    ram_info::modify_bootloader_info(&mut dsu, |inf| {
                        inf.diag_request_bootloader.0 = true;
                        if cmd.len() == 4 {
                            inf.diag_request_bootloader.1 = Some((cmd[3], cmd[2]));
                        } else {
                            inf.diag_request_bootloader.1 = None
                        }
                    });
                    drop(dsu);
                    self.pending_op = PendingOp::Reboot;
                    Ok(self.buf.copy_positive_reply(cmd[0], &[cmd[1]]))
                }
                Some(KwpSessionType::Normal) => {
                    self.mode = KwpSessionType::Normal;
                    Ok(self.buf.copy_positive_reply(cmd[0], &[cmd[1]]))
                }
                Some(KwpSessionType::ExtendedDiagnostics) => {
                    self.mode = KwpSessionType::ExtendedDiagnostics;
                    Ok(self.buf.copy_positive_reply(cmd[0], &[cmd[1]]))
                }
                _ => Err(KwpError::SubFunctionNotSupportedInvalidFormat),
            }
        }
    }

    pub async fn update(&mut self, _now_ms: u64) -> Option<&[u8]> {
        if self.flash_cfg.qspi_size == 0 {
            if let Some(qspi) = self.qspi.try_access() {
                if let Some(flash_size) = qspi.size_bytes() {
                    println!("Setting QSPI size to {} bytes", flash_size);
                    self.flash_cfg.qspi_size = flash_size;
                }
            }
        }
        match self.pending_op {
            PendingOp::None => None,
            PendingOp::Reboot => {
                Mono::delay(100u64.millis()).await;
                SCB::sys_reset();
            }
        }
    }
}
