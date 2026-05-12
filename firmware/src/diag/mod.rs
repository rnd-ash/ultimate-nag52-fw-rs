use atsamd_hal::{fugit::ExtU64, pac::SCB, rtic_time::Monotonic};
use automotive_diag::kwp2000::{KwpCommand, KwpError, KwpSessionType};
use diag_common::{DefmtTarget, defmt_multi_output, hal_extensions::dsu::Dsu, ram_info};
use rtic_sync::arbiter::Arbiter;

pub mod dev_mode;

use crate::Mono;

#[derive(Copy, Clone)]
pub enum PendingOp {
    None,
    Reboot,
}

pub struct KwpServer {
    last_cmd_time: u64,
    buf: [u8; 4096],
    pending_op: PendingOp,
    dsu: &'static Arbiter<Dsu>,
    mode: KwpSessionType,
}

type ServerResult = Result<usize, KwpError>;

impl KwpServer {
    pub fn new(dsu: &'static Arbiter<Dsu>) -> Self {
        Self {
            last_cmd_time: 0,
            buf: [0; 4096],
            pending_op: PendingOp::None,
            dsu,
            mode: KwpSessionType::Normal,
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

    pub async fn process_cmd(&mut self, cmd: &[u8], _now_ms: u64) -> &[u8] {
        self.last_cmd_time = Mono::now().duration_since_epoch().to_millis();
        let r = match KwpCommand::try_from(cmd[0]).ok() {
            Some(KwpCommand::StartDiagnosticSession) => self.start_diag_session(cmd).await,
            Some(KwpCommand::InputOutputControlByLocalIdentifier) => self.ioctl(cmd).await,
            _ => Err(KwpError::ServiceNotSupported),
        };

        let reply_len = r.unwrap_or_else(|nrc| self.make_nrc(cmd[0], nrc));
        &self.buf[..reply_len]
    }

    async fn ioctl(&mut self, cmd: &[u8]) -> ServerResult {
        if cmd.len() < 3 {
            return Err(KwpError::SubFunctionNotSupportedInvalidFormat)
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
                        Ok(self.make_positive_reply(cmd[0], &[0xF0, 0x01, log_ty]))
                    },
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
                                Ok(self.make_positive_reply(cmd[0], &[0xF0, cmd[3]]))
                            }
                        }
                    }, // 0x08 => Long term adjust
                    _ => Err(KwpError::RequestOutOfRange)
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
                    Ok(self.make_positive_reply(cmd[0], &[cmd[1]]))
                }
                Some(KwpSessionType::Normal) => {
                    self.mode = KwpSessionType::Normal;
                    Ok(self.make_positive_reply(cmd[0], &[cmd[1]]))
                }
                Some(KwpSessionType::ExtendedDiagnostics) => {
                    self.mode = KwpSessionType::ExtendedDiagnostics;
                    Ok(self.make_positive_reply(cmd[0], &[cmd[1]]))
                }
                _ => Err(KwpError::SubFunctionNotSupportedInvalidFormat),
            }
        }
    }

    pub async fn update(&mut self, _now_ms: u64) -> Option<&[u8]> {
        match self.pending_op {
            PendingOp::None => None,
            PendingOp::Reboot => {
                Mono::delay(100u64.millis()).await;
                SCB::sys_reset();
            }
        }
    }
}
