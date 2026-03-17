use std::{io::ErrorKind, sync::Arc, time::Duration};

use color_eyre::eyre::Report;
use ecu_diagnostics::channel::{ChannelError, ChannelResult, IsoTPChannel, PayloadChannel};
use serialport::{SerialPort, SerialPortType};

pub const UN52_USB_VID: u16 = 0x16c0;
pub const UN52_USB_PID: u16 = 0x27de;

pub struct UsbDiagIface {
    pending_reboot: bool,
    last_req: Option<Vec<u8>>,
    cached_port: Option<Box<dyn SerialPort>>
}

unsafe impl Send for UsbDiagIface{}
unsafe impl Sync for UsbDiagIface{}

impl UsbDiagIface {
    pub fn new() -> Result<Self, Report> {
        Ok(Self {
            pending_reboot: false,
            last_req: None,
            cached_port: None
        })
    }

    fn scan_and_open() -> std::io::Result<Box<dyn SerialPort>>  {
        let ser: Box<dyn SerialPort>;
        'outer: loop {
            let ports = serialport::available_ports().unwrap();
            for p in ports {
                if let SerialPortType::UsbPort(usb_inf) = p.port_type {
                    // Wait for all descriptor info to be ready!
                    if usb_inf.vid == 0x16c0 && usb_inf.pid == 0x27de && usb_inf.serial_number.is_some() && usb_inf.product.is_some() && usb_inf.manufacturer.is_some() {
                        if let Ok(serial) = serialport::new(&p.port_name, 115200)
                            .timeout(Duration::from_millis(500))
                            .open() {
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

    fn with_serial<R, F: FnMut(&mut Box<dyn SerialPort>) -> std::io::Result<R>>(&mut self, mut f: F) -> ChannelResult<R> {
        // Try with the port we used last time, this makes ops like Reading/Writing memory (lots of successive calls)
        // much faster
        if let Some(cached_port) = self.cached_port.as_mut() {
            let res = f(cached_port);
            match res {
                Ok(success) => return Ok(success),
                Err(_) => {
                    self.cached_port = None;
                }
            }
        }
        // If the cached port failed, then try to re-open it, this result is final
        let port  = Self::scan_and_open().map_err(|e| ChannelError::Other(e.to_string()))?;
        self.cached_port = Some(port);
        f(&mut self.cached_port.as_mut().unwrap()).map_err(|e| ChannelError::IOError(Arc::new(e)))
    }
}

impl IsoTPChannel for UsbDiagIface {
    fn set_iso_tp_cfg(&mut self, _cfg: ecu_diagnostics::channel::IsoTPSettings) -> ecu_diagnostics::channel::ChannelResult<()> {
        Ok(())
    }
}

impl PayloadChannel for UsbDiagIface {
    fn open(&mut self) -> ecu_diagnostics::channel::ChannelResult<()> {
        self.with_serial(|serial| {
            serial.clear(serialport::ClearBuffer::All)?;
            Ok(())
        })
    }

    fn close(&mut self) -> ecu_diagnostics::channel::ChannelResult<()> {
        Ok(())
    }

    fn set_ids(&mut self, _send: u32, _recv: u32) -> ecu_diagnostics::channel::ChannelResult<()> {
        Ok(())
    }

    fn read_bytes(&mut self, _timeout_ms: u32) -> ecu_diagnostics::channel::ChannelResult<Vec<u8>> {
        // Cached write request
        let req = self.last_req.take();
        // Everything here is executed with access to the serial port. If something failed,
        // then the app will try to re-connect to the port once before trying again. If that fails,
        // then we return an error
        self.with_serial(|serial| {
            if let Some(to_tx) = req.clone() {
                // Header (Len)
                let mut buf = (to_tx.len() as u16).to_le_bytes().to_vec();
                // Data to write
                buf.extend_from_slice(&to_tx);
                serial.write_all(&buf)?;
            }
            // Try reading the response
            let mut tmp = [0; 2];
            serial.read_exact(&mut tmp)?;
            let len = u16::from_le_bytes(tmp);
            let mut buf = vec![0; len as usize];
            serial.read_exact(&mut  buf)?;
            Ok(buf)
        })
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
