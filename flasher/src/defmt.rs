use std::{
    fmt,
    sync::mpsc::{Receiver, channel},
    time::Duration,
};

use color_eyre::eyre::{Error, Report};
use defmt_decoder::{Frame, Location, Locations, Table};
use diag_common::CAN_ID_DEFMT_LOG;
use ecu_diagnostics::channel::{CanFrame, Packet};
use ecu_diagnostics::hardware::{Hardware, HardwareScanner, socketcan::SocketCanScanner};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LogLevel {
    Info,
    Warn,
    Error,
    Debug,
}

#[derive(Debug, Clone)]
pub struct MicrosFormattedDuration(pub Duration);

#[derive(Debug, Clone, Default)]
pub struct DefmtTcuMsg {
    pub ts: Option<Duration>,
    pub msg: String,
    pub level: Option<defmt_parser::Level>,
    pub loc: Option<Location>,
}

impl fmt::Display for MicrosFormattedDuration {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut t = self.0.as_secs();
        let micros = self.0.as_micros() % 1_000_000;
        let seconds = t % 60;
        t /= 60;
        let minutes = t % 60;
        t /= 60;
        let hours = t % 24;
        t /= 24;
        if t > 0 {
            let days = t;
            write!(
                f,
                "{days}d {hours:02}:{minutes:02}:{seconds:02}.{micros:06}"
            )
        } else {
            write!(f, "{hours:02}:{minutes:02}:{seconds:02}.{micros:06}")
        }
    }
}

pub struct DefmtCanIf {
    rx: Receiver<Vec<u8>>,
}

pub fn decode_msg(bytes: &[u8], tab: &Table, loc: &Option<Locations>) -> Option<DefmtTcuMsg> {
    if let Ok((frame, _)) = tab.decode(bytes) {
        let mut msg = DefmtTcuMsg::default();
        if let Some(locations) = &loc {
            msg.loc = locations.get(&frame.index()).cloned()
        }

        msg.msg = frame.display_message().to_string();
        msg.level = frame.level();

        if let Some(time) = frame.display_timestamp() {
            if let Ok(seconds) = time.to_string().parse::<f32>() {
                let micros = (seconds * 1000000.0) as u64;
                msg.ts = Some(Duration::from_micros(micros));
            }
        }
        Some(msg)
    } else {
        None
    }
}

pub trait DefmtLogEndpoint {
    fn read_msg(&self) -> Option<Vec<u8>>;
}

impl DefmtCanIf {
    pub fn new(iface: &str) -> Result<Self, Report> {
        let mut dev = SocketCanScanner::new().open_device_by_name(iface)?;
        let mut can = dev.create_can_channel()?;
        let (tx, rx) = channel();
        can.set_can_cfg(500_000, false)?;
        can.open()?;

        std::thread::spawn(move || {
            let mut frame_buf = Vec::new();
            loop {
                for p in can
                    .read_packets(1000, 0)
                    .unwrap_or_default()
                    .iter()
                    .filter(|x| x.get_address() == CAN_ID_DEFMT_LOG as u32)
                {
                    let data = p.get_data();
                    if data[0] == 0xFF {
                        frame_buf.extend_from_slice(&data[1..]);
                        tx.send(frame_buf.clone());
                        frame_buf.clear();
                        // EOF
                    } else if data[0] == 0x00 {
                        // SOF
                        frame_buf.clear();
                        frame_buf.extend_from_slice(&data[1..]);
                    } else if data[0] < 0xF0 {
                        frame_buf.extend_from_slice(&data[1..]);
                    }
                }
                std::thread::sleep(Duration::from_millis(10));
            }
        });

        Ok(Self { rx })
    }
}

impl DefmtLogEndpoint for DefmtCanIf {
    fn read_msg(&self) -> Option<Vec<u8>> {
        self.rx.try_recv().ok()
    }
}
