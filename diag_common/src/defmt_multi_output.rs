use core::{
    cell::RefCell,
    sync::atomic::{AtomicBool, AtomicU8},
};

use atsamd_hal::{usb::UsbBus};
use cortex_m::interrupt::{CriticalSection, Mutex, free};
use defmt::{Encoder, global_logger};
use mcan::{
    embedded_can::{Id, StandardId},
    message::tx::{FrameType, MessageBuilder},
    tx_buffers::DynTx,
};
use rtic_sync::arbiter::{Arbiter, ExclusiveAccess};
use rtt_target::{UpChannel, rtt_init};
use usbd_serial::SerialPort;

use crate::DefmtTarget;

const LOG_MODE_RTT: u8 = 0;
const LOG_MODE_CAN: u8 = 1;
const LOG_MODE_SER: u8 = 2;

static IN_USE_LOGGGER: Mutex<RefCell<Option<InUseLogger>>> = Mutex::new(RefCell::new(None));
static MODE: AtomicU8 = AtomicU8::new(LOG_MODE_RTT);
static RTT_CHANNEL_INIT: AtomicBool = AtomicBool::new(false);
static RTT_CHANNEL: Mutex<RefCell<Option<UpChannel>>> = Mutex::new(RefCell::new(None));
static mut CAN_LOGGGER: Option<&'static Arbiter<bsp::can_deps::Can0Tx>> = None;
static mut SER_LOGGGER: Option<&'static Arbiter<SerialPort<'static, UsbBus>>> = None;

pub enum InUseLogger {
    Rtt((defmt::Encoder, UpChannel)),
    Can(
        (
            ExclusiveAccess<'static, bsp::can_deps::Can0Tx>,
            BufferedDefmtWriter<256, 8>,
        ),
    ),
    Serial(
        (
            ExclusiveAccess<'static, SerialPort<'static, UsbBus>>,
            BufferedDefmtWriter<256, 32>,
        ),
    ),
}

#[derive(Copy, Clone)]
pub enum Error {
    EndpointNotPresent,
}

pub struct BufferedDefmtWriter<const BUF_LEN: usize, const PKT_MAX: usize> {
    inner: [u8; BUF_LEN],
    pos: usize,
    pci: u8,
}

impl<const BUF_LEN: usize, const PKT_MAX: usize> Default for BufferedDefmtWriter<BUF_LEN, PKT_MAX> {
    fn default() -> Self {
        Self {
            inner: [0; BUF_LEN],
            pos: Default::default(),
            pci: Default::default(),
        }
    }
}

impl<const BUF_LEN: usize, const PKT_MAX: usize> BufferedDefmtWriter<BUF_LEN, PKT_MAX> {
    pub fn write<F: FnMut(&[u8]) -> Option<usize>>(
        &mut self,
        end: bool,
        data: &[u8],
        mut write_fn: F,
    ) {
        if self.pos + data.len() > BUF_LEN {
            // TODO - Handle Overflow case
            return;
        }
        self.inner[self.pos..self.pos + data.len()].copy_from_slice(data);
        self.pos += data.len();

        if self.pos > PKT_MAX || end {
            let mut out_pos = 0;
            let data_max = PKT_MAX - 1;
            let mut buf = [0; PKT_MAX];
            loop {
                let max = core::cmp::min(self.pos - out_pos, data_max);
                if end && self.pos - out_pos <= data_max {
                    self.pci = 0xFF;
                }

                buf[0] = self.pci;
                buf[1..max + 1].copy_from_slice(&self.inner[out_pos..out_pos + max]);
                if let Some(size) = (write_fn)(&buf[..1 + max]) {
                    out_pos += size - 1 // -1 since data[0] is PCI
                } else {
                    break;
                }

                self.pci += 1;
                if self.pci == 0xF0 {
                    self.pci = 1;
                }
                let left = self.pos - out_pos;
                if (end && left == 0) || (!end && left < PKT_MAX) {
                    break;
                }
            }
            if !end {
                self.pos -= out_pos;
                self.inner.rotate_left(out_pos);
            }
        }
    }
}

const CAN_ID_DEFMT: StandardId = unsafe { StandardId::new_unchecked(crate::CAN_ID_DEFMT_LOG) };
fn write_data_can(tx: &mut bsp::can_deps::Can0Tx, buf: &[u8]) -> Option<usize> {
    let mb = MessageBuilder {
        id: Id::Standard(CAN_ID_DEFMT),
        frame_type: FrameType::Classic(mcan::message::tx::ClassicFrameType::Data(buf)),
        store_tx_event: None,
    }
    .build()
    .unwrap();
    tx.transmit_queued(mb).ok().map(|_| buf.len())
}

fn write_data_serial(serial: &mut SerialPort<'static, UsbBus>, buf: &[u8]) -> Option<usize> {
    if serial.dtr() {
        let size = (buf.len() as u16 + 1).to_le_bytes();
        serial.write(&size).ok()?;
        serial.write(&[crate::USB_PACKET_TY_DEFMT]).ok()?;
        serial.write(buf).ok()
    } else {
        None
    }
}

impl InUseLogger {
    pub fn start(&mut self) {
        match self {
            InUseLogger::Rtt((encoder, channel)) => {
                encoder.start_frame(|w| {
                    channel.write(w);
                });
            }
            _ => {}
        }
    }

    pub fn write(&mut self, bytes: &[u8]) {
        match self {
            InUseLogger::Rtt((encoder, channel)) => {
                encoder.write(bytes, |w| {
                    channel.write(w);
                });
            }
            InUseLogger::Can((can, buffer)) => {
                buffer.write(false, bytes, |data| write_data_can(can, data));
            }
            InUseLogger::Serial((ser, buffer)) => {
                buffer.write(false, bytes, |data| write_data_serial(ser, data));
            }
        }
    }

    pub fn release(self, cs: &CriticalSection) {
        match self {
            InUseLogger::Rtt((mut encoder, mut channel)) => {
                encoder.end_frame(|w| {
                    channel.write(w);
                });
                // Put the channel back
                *RTT_CHANNEL.borrow(cs).borrow_mut() = Some(channel);
            }
            InUseLogger::Can((mut can, mut buffer)) => {
                buffer.write(true, &[], |data| write_data_can(&mut can, data));
            }
            InUseLogger::Serial((mut ser, mut buffer)) => {
                buffer.write(true, &[], |data| write_data_serial(&mut ser, data));
            }
        }
    }
}

fn can_defmt_logger_present() -> bool {
    free(|_| {
        let raw = unsafe { *&*&raw const CAN_LOGGGER };
        raw.is_some()
    })
}

fn serial_defmt_logger_present() -> bool {
    free(|_| {
        let raw = unsafe { *&*&raw const SER_LOGGGER };
        raw.is_some()
    })
}

pub fn set_defmt_log_mode(mode: DefmtTarget) -> Result<(), Error> {
    match mode {
        DefmtTarget::Rtt => {
            // Always OK
        }
        DefmtTarget::Can => {
            if !can_defmt_logger_present() {
                return Err(Error::EndpointNotPresent);
            }
        }
        DefmtTarget::Serial => {
            if !serial_defmt_logger_present() {
                return Err(Error::EndpointNotPresent);
            }
        }
    }
    MODE.store(mode as u8, core::sync::atomic::Ordering::Relaxed);
    Ok(())
}

pub fn get_current_defmt_log_mode() -> DefmtTarget {
    unsafe {
        let n = MODE.load(core::sync::atomic::Ordering::Relaxed);
        // Safety - We guarantee with constants this is OK
        core::mem::transmute(n)
    }
}

pub fn set_defmt_can_logger(can: &'static Arbiter<bsp::can_deps::Can0Tx>) {
    free(|_| unsafe { CAN_LOGGGER = Some(can) })
}

pub fn set_defmt_serial_logger(ser: &'static Arbiter<SerialPort<'static, UsbBus>>) {
    free(|_| unsafe { SER_LOGGGER = Some(ser) })
}

pub fn init() {
    if RTT_CHANNEL_INIT.load(core::sync::atomic::Ordering::Relaxed) {
        panic!("RTT Channel alread initialized")
    }
    let c = rtt_init! {
        up: {
            0: {
                size: 512,
                mode: rtt_target::ChannelMode::NoBlockSkip,
                name: "defmt"
            }
        }
    };
    free(|cs| {
        *RTT_CHANNEL.borrow(cs).borrow_mut() = Some(c.up.0);
    });
    RTT_CHANNEL_INIT.store(true, core::sync::atomic::Ordering::Relaxed);
}

#[global_logger]
pub struct DefmtMutliOutputLogger;

unsafe impl defmt::Logger for DefmtMutliOutputLogger {
    fn acquire() {
        free(|cs| {
            let logger = IN_USE_LOGGGER.borrow(cs);
            if logger.borrow().is_some() {
                panic!("Logger already in use!")
            } else {
                let mode = MODE.load(core::sync::atomic::Ordering::Relaxed);
                let mut msg_logger = if mode == LOG_MODE_CAN
                    // SAFETY - In critical section
                    && let Some(can_logger) = unsafe { CAN_LOGGGER }
                    && let Some(aquired) = can_logger.try_access()
                {
                    InUseLogger::Can((aquired, BufferedDefmtWriter::default()))
                } else if mode == LOG_MODE_SER
                    // SAFETY - In critical section
                    && let Some(ser_logger) = unsafe { SER_LOGGGER }
                    && let Some(aquired) = ser_logger.try_access()
                {
                    InUseLogger::Serial((aquired, BufferedDefmtWriter::default()))
                } else {
                    // Use RTT as fallback
                    if let Some(rtt_channel) = RTT_CHANNEL.borrow(cs).take() {
                        InUseLogger::Rtt((Encoder::new(), rtt_channel))
                    } else {
                        panic!("RTT Channel not initialized")
                    }
                };
                msg_logger.start();
                *logger.borrow_mut() = Some(msg_logger);
            }
        });
    }

    unsafe fn flush() {}

    unsafe fn release() {
        free(|cs| {
            if let Some(logger) = IN_USE_LOGGGER.borrow(cs).borrow_mut().take() {
                logger.release(cs);
            }
        })
    }

    unsafe fn write(bytes: &[u8]) {
        free(|cs| {
            if let Some(logger) = IN_USE_LOGGGER.borrow(cs).borrow_mut().as_mut() {
                logger.write(bytes);
            }
        })
    }
}
