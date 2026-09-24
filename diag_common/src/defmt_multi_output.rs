use core::{
    cell::UnsafeCell,
    sync::atomic::{AtomicBool, AtomicU8, compiler_fence},
};

use atsamd_hal::usb::UsbBus;
use critical_section::RestoreState;
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

static MODE: AtomicU8 = AtomicU8::new(LOG_MODE_RTT);
static RTT_CHANNEL_INIT: AtomicBool = AtomicBool::new(false);

static mut CAN_LOGGGER: Option<&'static Arbiter<bsp::can_deps::Can0Tx>> = None;
static mut SER_LOGGGER: Option<&'static Arbiter<SerialPort<'static, UsbBus>>> = None;

pub enum InUseLogger {
    Can(
        (
            ExclusiveAccess<'static, bsp::can_deps::Can0Tx>,
            BufferedDefmtWriter<256>,
        ),
    ),
    Serial(
        (
            ExclusiveAccess<'static, SerialPort<'static, UsbBus>>,
            BufferedDefmtWriter<256>,
        ),
    ),
}

#[derive(Copy, Clone)]
pub enum Error {
    EndpointNotPresent,
}

pub struct BufferedDefmtWriter<const BUF_LEN: usize> {
    inner: [u8; BUF_LEN],
    pos: usize,
    pci: u8,
}

impl<const BUF_LEN: usize> Default for BufferedDefmtWriter<BUF_LEN> {
    fn default() -> Self {
        Self {
            inner: [0; BUF_LEN],
            pos: Default::default(),
            pci: Default::default(),
        }
    }
}

impl<const BUF_LEN: usize> BufferedDefmtWriter<BUF_LEN> {
    pub fn write<const PKT_MAX: usize, F: FnMut(&[u8]) -> Option<usize>>(
        &mut self,
        data: &[u8],
        mut write_fn: F,
    ) {
        if self.pos + data.len() > BUF_LEN {
            // TODO - Handle Overflow case
            return;
        }
        self.inner[self.pos..self.pos + data.len()].copy_from_slice(data);
        self.pos += data.len();

        if self.pos > PKT_MAX || data.is_empty() {
            let mut out_pos = 0;
            let data_max = PKT_MAX - 1;
            let mut buf = [0; PKT_MAX];
            loop {
                let max = core::cmp::min(self.pos - out_pos, data_max);
                if data.is_empty() && self.pos - out_pos <= data_max {
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
                if (data.is_empty() && left == 0) || (!data.is_empty() && left < PKT_MAX) {
                    break;
                }
            }
            if !data.is_empty() {
                self.pos -= out_pos;
                self.inner.rotate_left(out_pos);
            }
        }
    }
}

const CAN_ID_DEFMT: StandardId = unsafe { StandardId::new_unchecked(crate::CAN_ID_DEFMT_LOG) };

fn write_data_can(
    can_tx: &mut ExclusiveAccess<bsp::can_deps::Can0Tx>,
    buf: &[u8],
) -> Option<usize> {
    let mb = MessageBuilder {
        id: Id::Standard(CAN_ID_DEFMT),
        frame_type: FrameType::Classic(mcan::message::tx::ClassicFrameType::Data(buf)),
        store_tx_event: None,
    }
    .build()
    .unwrap();
    can_tx.transmit_queued(mb).ok().map(|_| buf.len())
}

fn write_data_serial(
    serial: &mut ExclusiveAccess<'_, SerialPort<'static, UsbBus>>,
    buf: &[u8],
) -> Option<usize> {
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
    pub fn write(&mut self, bytes: &[u8]) {
        match self {
            InUseLogger::Can((can, buffer)) => {
                buffer.write::<8, _>(bytes, |data| write_data_can(can, data));
            }
            InUseLogger::Serial((ser, buffer)) => {
                buffer.write::<32, _>(bytes, |data| write_data_serial(ser, data));
            }
        }
    }

    pub fn release(self) {
        match self {
            InUseLogger::Can((mut can, mut buffer)) => {
                buffer.write::<8, _>(&[], |data| write_data_can(&mut can, data));
            }
            InUseLogger::Serial((mut ser, mut buffer)) => {
                buffer.write::<32, _>(&[], |data| write_data_serial(&mut ser, data));
            }
        }
    }
}

fn can_defmt_logger_present() -> bool {
    critical_section::with(|_| {
        let raw = unsafe { *&*&raw const CAN_LOGGGER };
        raw.is_some()
    })
}

fn serial_defmt_logger_present() -> bool {
    critical_section::with(|_| {
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
    critical_section::with(|_| unsafe { CAN_LOGGGER = Some(can) })
}

pub fn set_defmt_serial_logger(ser: &'static Arbiter<SerialPort<'static, UsbBus>>) {
    critical_section::with(|_| unsafe { SER_LOGGGER = Some(ser) })
}

pub fn init() {
    if RTT_CHANNEL_INIT.load(core::sync::atomic::Ordering::Relaxed) {
        panic!("RTT Channel already initialized")
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

    critical_section::with(|_| unsafe {
        LOGGER_STATE
            .rtt_logger
            .get()
            .write(Some((Encoder::new(), c.up.0)));
    });
    RTT_CHANNEL_INIT.store(true, core::sync::atomic::Ordering::Relaxed);
}

pub struct LoggerState {
    inner: UnsafeCell<RestoreState>,
    rtt_logger: UnsafeCell<Option<(Encoder, UpChannel)>>,
    alt_logger: UnsafeCell<Option<InUseLogger>>,
}

unsafe impl Sync for LoggerState {}

static LOGGER_STATE: LoggerState = LoggerState {
    inner: UnsafeCell::new(RestoreState::invalid()),
    rtt_logger: UnsafeCell::new(None),
    alt_logger: UnsafeCell::new(None),
};

#[global_logger]
pub struct DefmtMutliOutputLogger;

/// Safety notes
///
/// 1. On acquire, all interrupts are disabled
/// 2. Write and flush get called (Potentially with multiple write calls)
/// 3. Release is called, interrupts are re-enabled
///
/// The reason for doing this manually is to prevent a potential deadlock,
/// where one of the writer's Arbiters initially is free, but then between
/// acquire and write, gets locked by another task.
///
/// This way, we ensure that during the whole write of the message, the endpoints
/// are either free, or in use (Which causes a cancelled write)
unsafe impl defmt::Logger for DefmtMutliOutputLogger {
    fn acquire() {
        let restore = unsafe { critical_section::acquire() };
        compiler_fence(core::sync::atomic::Ordering::SeqCst);
        unsafe {
            LOGGER_STATE.inner.get().write(restore);
            // Write to RTT (All messages)
            if let Some((encoder, rtt)) = &mut *LOGGER_STATE.rtt_logger.get() {
                encoder.start_frame(|bytes| {
                    rtt.write(bytes);
                });
            }
            // Try to gain access to the alt endpoints
            let mode = MODE.load(core::sync::atomic::Ordering::Relaxed);
            if mode == LOG_MODE_CAN
                && let Some(Some(can_tx)) = CAN_LOGGGER.map(|x| x.try_access())
            {
                // We have access to CAN Tx, so we can write this message
                *(&mut *LOGGER_STATE.alt_logger.get()) =
                    Some(InUseLogger::Can((can_tx, BufferedDefmtWriter::default())));
            } else if mode == LOG_MODE_SER
                && let Some(Some(ser)) = SER_LOGGGER.map(|x| x.try_access())
            {
                *(&mut *LOGGER_STATE.alt_logger.get()) =
                    Some(InUseLogger::Serial((ser, BufferedDefmtWriter::default())));
            }
        }
    }

    unsafe fn write(bytes: &[u8]) {
        // Safety - we are inside a CS
        if let Some((encoder, rtt)) = unsafe { &mut *LOGGER_STATE.rtt_logger.get() } {
            encoder.write(bytes, |bytes| {
                rtt.write(bytes);
            });
        }

        if let Some(writer) = unsafe { &mut *LOGGER_STATE.alt_logger.get() } {
            writer.write(bytes);
        }
    }

    unsafe fn flush() {
        // Safety - We are inside a CS
    }

    unsafe fn release() {
        if let Some((encoder, rtt)) = unsafe { &mut *LOGGER_STATE.rtt_logger.get() } {
            encoder.end_frame(|bytes| {
                rtt.write(bytes);
            });
        }

        if let Some(writer) = unsafe { &mut *LOGGER_STATE.alt_logger.get() }.take() {
            writer.release();
        }

        compiler_fence(core::sync::atomic::Ordering::SeqCst);
        unsafe { critical_section::release(LOGGER_STATE.inner.get().read()) };
    }
}
