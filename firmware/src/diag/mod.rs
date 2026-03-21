use atsamd_hal::{fugit::ExtU64, pac::SCB, rtic_time::Monotonic};
use automotive_diag::kwp2000::{KwpCommand, KwpError, KwpSessionType};
use diag_common::ram_info;

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
}

type ServerResult = core::result::Result<usize, KwpError>;

impl KwpServer {
    pub fn new() -> Self {
        Self {
            last_cmd_time: 0,
            buf: [0; 4096],
            pending_op: PendingOp::None,
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

    pub fn process_cmd<'a>(&'a mut self, cmd: &[u8], _now_ms: u64) -> &'a [u8] {
        self.last_cmd_time = Mono::now().duration_since_epoch().to_millis();
        let r = match KwpCommand::try_from(cmd[0]).ok() {
            Some(KwpCommand::StartDiagnosticSession) => self.start_diag_session(cmd),
            _ => Err(KwpError::ServiceNotSupported),
        };

        let reply_len = r.unwrap_or_else(|nrc| self.make_nrc(cmd[0], nrc));
        &self.buf[..reply_len]
    }

    fn start_diag_session(&mut self, cmd: &[u8]) -> ServerResult {
        if cmd.len() != 2 && cmd.len() != 4 {
            Err(KwpError::SubFunctionNotSupportedInvalidFormat)
        } else {
            match KwpSessionType::try_from(cmd[1]).ok() {
                Some(KwpSessionType::Reprogramming) => {
                    ram_info::modify_bootloader_info(|inf| {
                        inf.diag_request_bootloader.0 = true;
                        if cmd.len() == 4 {
                            inf.diag_request_bootloader.1 = Some((cmd[3], cmd[2]));
                        } else {
                            inf.diag_request_bootloader.1 = None
                        }
                    });
                    self.pending_op = PendingOp::Reboot;
                    Ok(self.make_positive_reply(cmd[0], &[cmd[1]]))
                }
                Some(KwpSessionType::Normal) => {
                    //self.mode = KwpSessionType::Normal;
                    Ok(self.make_positive_reply(cmd[0], &[cmd[1]]))
                }
                _ => return Err(KwpError::SubFunctionNotSupportedInvalidFormat),
            }
        }
    }

    pub async fn update(&mut self, _now_ms: u64) -> Option<&[u8]> {
        match self.pending_op {
            PendingOp::None => None,
            PendingOp::Reboot => {
                Mono::delay(20u64.millis()).await;
                SCB::sys_reset();
            }
        }
    }
}
