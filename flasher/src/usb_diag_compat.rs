use std::{
    io::{self, ErrorKind},
    sync::{
        Arc, RwLock,
        atomic::AtomicBool,
        mpsc::{self, channel},
    },
    time::Duration,
};

use color_eyre::eyre::Report;
use ecu_diagnostics::channel::{ChannelError, ChannelResult, IsoTPChannel, PayloadChannel};
use serialport::{SerialPort, SerialPortType};

use crate::defmt::{DefmtLogEndpoint, DefmtTcuMsg};

pub const UN52_USB_VID: u16 = 0x16c0;
pub const UN52_USB_PID: u16 = 0x27de;

pub struct SerialInner {
    response_diag: mpsc::Receiver<Vec<u8>>,
    defmt_queue: mpsc::Receiver<Vec<u8>>,
    sender_diag: mpsc::Sender<Vec<u8>>,
    alive: Arc<AtomicBool>,
}

impl SerialInner {
    pub fn new(mut port: Box<dyn SerialPort>) -> std::io::Result<Self> {
        let mut port_c = port.try_clone().unwrap();
        let (tx_req, rx_req) = channel::<Vec<u8>>();
        let (tx_defmt, rx_defmt) = channel::<Vec<u8>>();
        let (tx_resp, rx_resp) = channel::<Vec<u8>>();
        port.clear(serialport::ClearBuffer::All)?;

        let alive: Arc<AtomicBool> = Arc::new(AtomicBool::new(true));
        let alive_c = alive.clone();
        let alive_cc = alive.clone();

        std::thread::spawn(move || {
            let mut defmt_frame_buf = Vec::new();
            loop {
                if !alive_c.load(std::sync::atomic::Ordering::Relaxed) {
                    break;
                }
                let mut tmp = [0; 2];
                if let Err(e) = port.read_exact(&mut tmp) {
                    alive_c.store(false, std::sync::atomic::Ordering::Relaxed);
                    break;
                }
                let len = u16::from_le_bytes(tmp);
                let mut buf = vec![0; len as usize];
                if port.read_exact(&mut buf).is_err() {
                    alive_c.store(false, std::sync::atomic::Ordering::Relaxed);
                    break;
                }
                if buf[0] == diag_common::USB_PACKET_TY_ISOTP {
                    let _ = tx_resp.send(buf[1..].to_vec());
                } else if buf[0] == diag_common::USB_PACKET_TY_DEFMT {
                    defmt_frame_buf.extend_from_slice(&buf[2..]);
                    if buf[1] == 0xFF {
                        // End of frame
                        let _ = tx_defmt.send(defmt_frame_buf.clone());
                        defmt_frame_buf.clear();
                    }
                } else {
                    log::warn!("Unknown USB Packet {:02X?}", buf)
                }
            }
            alive_c.store(false, std::sync::atomic::Ordering::Relaxed);
        });
        std::thread::spawn(move || {
            loop {
                if let Ok(req) = rx_req.recv_timeout(Duration::from_secs(1)) {
                    // Header (Len)
                    let mut buf = (req.len() as u16).to_le_bytes().to_vec();
                    // Data to write
                    buf.extend_from_slice(&req);
                    if port_c.write_all(&buf).is_err() {
                        alive_cc.store(false, std::sync::atomic::Ordering::Relaxed);
                        break;
                    }
                } else if !alive_cc.load(std::sync::atomic::Ordering::Relaxed) {
                    break;
                }
            }
        });
        Ok(Self {
            response_diag: rx_resp,
            defmt_queue: rx_defmt,
            sender_diag: tx_req,
            alive: alive,
        })
    }
}

impl Drop for SerialInner {
    fn drop(&mut self) {
        self.alive
            .store(false, std::sync::atomic::Ordering::Relaxed);
    }
}

pub struct UsbDiagIface {
    pending_reboot: bool,
    last_req: Option<Vec<u8>>,
    cached_port: Option<SerialInner>,
}

unsafe impl Send for UsbDiagIface {}
unsafe impl Sync for UsbDiagIface {}

impl UsbDiagIface {
    pub fn new() -> Result<Self, Report> {
        let mut s = Self {
            pending_reboot: false,
            last_req: None,
            cached_port: None,
        };
        // To trigger port open
        s.with_serial(|_| Ok(()))?;
        Ok(s)
    }

    fn scan_and_open() -> std::io::Result<Box<dyn SerialPort>> {
        let ser: Box<dyn SerialPort>;
        'outer: loop {
            let ports = serialport::available_ports().unwrap();
            for p in ports {
                if let SerialPortType::UsbPort(usb_inf) = p.port_type {
                    // Wait for all descriptor info to be ready!
                    if usb_inf.vid == 0x16c0
                        && usb_inf.pid == 0x27de
                        && usb_inf.serial_number.is_some()
                        && usb_inf.product.is_some()
                        && usb_inf.manufacturer.is_some()
                    {
                        if let Ok(serial) = serialport::new(&p.port_name, 115200)
                            .timeout(Duration::from_millis(500))
                            .open()
                        {
                            ser = serial;
                            break 'outer;
                        }
                    }
                }
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        Ok(ser)
    }

    fn with_serial<R, F: FnMut(&mut SerialInner) -> ChannelResult<R>>(
        &mut self,
        mut f: F,
    ) -> ChannelResult<R> {
        // Try with the port we used last time, this makes ops like Reading/Writing memory (lots of successive calls)
        // much faster
        if let Some(cached_port) = self.cached_port.as_mut() {
            if !cached_port.alive.load(std::sync::atomic::Ordering::Relaxed) {
                self.cached_port = None;
            } else {
                let res = f(cached_port);
                match res {
                    Ok(success) => return Ok(success),
                    Err(_) => {
                        println!("Uncaching port");
                        self.cached_port = None;
                    }
                }
            }
        }
        // If the cached port failed, then try to re-open it, this result is final
        let port = Self::scan_and_open()?;
        self.cached_port = Some(SerialInner::new(port)?);
        f(&mut self.cached_port.as_mut().unwrap())
    }
}

impl IsoTPChannel for UsbDiagIface {
    fn set_iso_tp_cfg(
        &mut self,
        _cfg: ecu_diagnostics::channel::IsoTPSettings,
    ) -> ecu_diagnostics::channel::ChannelResult<()> {
        Ok(())
    }
}

impl DefmtLogEndpoint for UsbDiagIface {
    fn read_msg(&self) -> Option<Vec<u8>> {
        self.cached_port
            .as_ref()
            .map(|x| x.defmt_queue.recv().ok())
            .flatten()
    }
}

impl PayloadChannel for UsbDiagIface {
    fn open(&mut self) -> ecu_diagnostics::channel::ChannelResult<()> {
        Ok(())
    }

    fn close(&mut self) -> ecu_diagnostics::channel::ChannelResult<()> {
        Ok(())
    }

    fn set_ids(&mut self, _send: u32, _recv: u32) -> ecu_diagnostics::channel::ChannelResult<()> {
        Ok(())
    }

    fn read_bytes(&mut self, timeout_ms: u32) -> ecu_diagnostics::channel::ChannelResult<Vec<u8>> {
        let cache = self.last_req.clone();
        let output = self.with_serial(|serial| {
            if let Some(r) = &cache {
                let _ = serial.sender_diag.send(r.to_vec());
            }

            let r = serial
                .response_diag
                .recv_timeout(Duration::from_millis(timeout_ms as u64))
                .ok();
            if let Some(r) = r {
                Ok(r)
            } else {
                Err(ChannelError::ReadTimeout)
            }
        });
        if output.is_ok() {
            self.last_req = None;
        }

        output
    }

    fn write_bytes(
        &mut self,
        _addr: u32,
        _ext_id: Option<u8>,
        buffer: &[u8],
        _timeout_ms: u32,
    ) -> ecu_diagnostics::channel::ChannelResult<()> {
        // IMPORTANT: Since we know this TCU only ever does call and response
        // requests (Write followed by reading), we can just cache what to write
        // and report OK, and then when read is called, perform both write and read
        // with one access to the Serial port, to make it more reliable
        self.last_req = Some(buffer.to_vec());
        Ok(())
    }

    fn clear_rx_buffer(&mut self) -> ecu_diagnostics::channel::ChannelResult<()> {
        Ok(())
    }

    fn clear_tx_buffer(&mut self) -> ecu_diagnostics::channel::ChannelResult<()> {
        Ok(())
    }
}
