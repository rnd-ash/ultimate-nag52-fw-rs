//! ISO 15765-2 ISOTP Layer for MCAN CAN devices
//!
//! Assumptions
//! * Padding of each frame to 8 bytes
//! * Non extended CAN or extended ISO-TP addressing

use mcan::{
    core::CanId,
    embedded_can::Id,
    message::tx::{AnyMessage, ClassicFrameType, FrameType, MessageBuilder},
    messageram::Capacities,
    tx_buffers::DynTx,
};
use rtic_sync::{
    arbiter::Arbiter,
    signal::{Signal, SignalReader, SignalWriter},
};

use futures::FutureExt;

use crate::isotp_endpoints::SharedIsoTpBuf;

const fn make_fc_ok(stmin: u8, bs: u8) -> [u8; 8] {
    [0x30, bs, stmin, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC]
}

const fn make_fc_reject() -> [u8; 8] {
    [0x32, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC]
}

fn write<'a, ID: CanId + 'a, C: Capacities>(
    id: Id,
    data: [u8; 8],
    tx: &mut mcan::tx_buffers::Tx<'a, ID, C>,
    mailbox: Option<usize>
) -> nb::Result<(), mcan::tx_buffers::Error> {
    let msg = C::TxMessage::new(MessageBuilder {
        id,
        frame_type: FrameType::Classic(ClassicFrameType::Data(&data)),
        store_tx_event: None,
    })
    .unwrap();
    if let Some(mailbox) = mailbox {
        tx.transmit_dedicated(mailbox, msg)
    } else {
        tx.transmit_queued(msg)
    }
}

pub fn make_isotp_endpoint<'a, ID: CanId + 'a, C: Capacities, const N: usize>(
    tx_id: Id,
    rx_id: Id,
    tx_mailbox: Option<usize>,
    can_tx: &'a Arbiter<mcan::tx_buffers::Tx<'a, ID, C>>,
    ready_signal: &'a Signal<IsotpCtsMsg>,
    rx_signal: &'a Signal<SharedIsoTpBuf<N>>,
) -> (
    IsoTpInterruptHandler<'a, ID, C, N>,
    IsotpConsumer<'a, ID, C, N>,
) {
    let (tx_tx_signal, rx_tx_signal) = ready_signal.split();
    let (tx_rx_ready, rx_rx_ready) = rx_signal.split();
    (
        IsoTpInterruptHandler {
            rx_id,
            tx_id,
            mailbox_dedicated: tx_mailbox,
            rx_ready: tx_rx_ready,
            rx_state: IsoTpMode::Idle,
            can_tx,
            tx_clear_to_send: tx_tx_signal,
        },
        IsotpConsumer {
            tx_id,
            rx_ready: rx_rx_ready,
            can_tx,
            rx_clear_to_send: rx_tx_signal,
            mailbox_dedicated: tx_mailbox,
        },
    )
}

/// Part of the ISOTP handler that is used for MCAN interrupts:
///
/// Usage with RTIC:
/// ```
/// #[task(priority = 1, binds=CAN0, local=[can0_interrupts, can0_fifo0, isotp_isr])]
/// fn can0(cx: can0::Context) {
///     for interrupt in cx.local.can0_interrupts.iter_flagged() {
///         match interrupt {
///             Interrupt::RxFifo0NewMessage => {
///                 for msg in cx.local.can0_fifo0.into_iter() {
///                     if msg.id() == Id::Standard(cx.local.isotp_isr.rx_id) {
///                         const BS: u8 = 8;
///                         const STMIN: u8 = 20;
///                         cx.local.isotp_isr.on_frame_rx(msg.data(), STMIN, BS);
///                     }
///                 }
///             }
///             _ => {}
///         }
///     }
/// }
/// ```
pub struct IsoTpInterruptHandler<'a, ID: CanId + 'a, C: Capacities + 'a, const N: usize> {
    pub rx_id: Id,
    pub mailbox_dedicated: Option<usize>,
    tx_id: Id,
    rx_ready: SignalWriter<'a, SharedIsoTpBuf<N>>,
    rx_state: IsoTpMode<N>,
    can_tx: &'a Arbiter<mcan::tx_buffers::Tx<'a, ID, C>>,
    tx_clear_to_send: SignalWriter<'a, IsotpCtsMsg>,
}

impl<'a, ID: CanId + 'a, C: Capacities + 'a, const N: usize> IsoTpInterruptHandler<'a, ID, C, N> {
    /// Called when each frame is received
    pub fn on_frame_rx(&mut self, data: &[u8], ecu_stmin: u8, ecu_bs: u8) {
        if data.len() == 8 {
            match data[0] {
                0x00..=0x07 => {
                    // Single frame handling
                    let size = data[0] as usize;
                    let buf = &data[1..1 + size];
                    let mut tmp = SharedIsoTpBuf::new();
                    tmp.data[..size].copy_from_slice(buf);
                    tmp.size = size;
                    self.rx_ready.write(tmp);
                }
                0x10..=0x1F => {
                    // Start of a large payload
                    if let Some(mut can_tx) = self.can_tx.try_access() {
                        // Accept
                        let size = (((data[0] & 0x0F) as u16) << 8 | (data[1] as u16)) as usize;

                        let mut shared_buffer = SharedIsoTpBuf::new();
                        shared_buffer.data[..6].copy_from_slice(&data[2..]);
                        shared_buffer.size = 6;

                        if write(self.tx_id, make_fc_ok(ecu_stmin, ecu_bs), &mut can_tx, self.mailbox_dedicated).is_ok() {
                            self.rx_state = IsoTpMode::Rx {
                                stmin: ecu_stmin,
                                bs: ecu_bs,
                                buf: shared_buffer,
                                rx_count: 0,
                                targ_size: size,
                            };
                        }
                    }
                }
                0x20..=0x2F => {
                    if let IsoTpMode::Rx {
                        stmin,
                        bs,
                        buf,
                        rx_count,
                        targ_size,
                    } = &mut self.rx_state
                    {
                        // Multi-frame
                        *rx_count += 1;
                        let max = core::cmp::min(7, *targ_size - buf.size);
                        buf.data[buf.size..buf.size + max].copy_from_slice(&data[1..1 + max]);
                        buf.size += max;
                        if buf.size == *targ_size {
                            // Completed reception of  data
                            self.rx_ready.write(*buf);
                            self.rx_state = IsoTpMode::Idle;
                        } else if *bs != 0 && rx_count == bs {
                            // Try to transmit a FC OK CAN message
                            if self
                                .can_tx
                                .try_access()
                                .and_then(|mut tx| {
                                    write(self.tx_id, make_fc_ok(*stmin, *bs), &mut tx, self.mailbox_dedicated).ok()
                                })
                                .is_none()
                            {
                                // Error
                                self.rx_state = IsoTpMode::Idle;
                            } else {
                                // Ok
                                *rx_count = 0;
                            }
                        }
                    }
                }
                0x30 => self.tx_clear_to_send.write(IsotpCtsMsg::Ok {
                    stmin: data[2],
                    bs: data[1],
                }),
                0x31 => self.tx_clear_to_send.write(IsotpCtsMsg::Wait),
                0x32 => self.tx_clear_to_send.write(IsotpCtsMsg::Reject),
                _ => {}
            }
        }
    }
}

#[derive(Clone, Copy)]
/// Used by the ISR Handler to determine
/// what state to be in
pub enum IsoTpMode<const N: usize> {
    /// In reception mode.
    /// Timing parameters are from the
    /// other ends flow-control msg
    Rx {
        stmin: u8,
        bs: u8,
        buf: SharedIsoTpBuf<N>,
        rx_count: u8,
        targ_size: usize,
    },
    /// No message reception in progress
    Idle,
}

#[derive(Clone, Copy)]
/// Message sent to the Thread handler
pub enum IsotpCtsMsg {
    /// Thread handler can continue with sending
    Ok { stmin: u8, bs: u8 },
    /// Wait for another flow control msg
    Wait,
    /// Receiver rejected
    Reject,
}

/// ISOTP Transmission error
pub enum IsoTpTxErr {
    /// Receiver rejected the request
    Rejected,
    /// Timeout waiting for receipient flow control
    Timeout,
    /// Internal CAN error
    McanErr(nb::Error<mcan::tx_buffers::Error>),
}

impl From<nb::Error<mcan::tx_buffers::Error>> for IsoTpTxErr {
    fn from(value: nb::Error<mcan::tx_buffers::Error>) -> Self {
        Self::McanErr(value)
    }
}

/// ISOTP thread part
///
/// This is designed to be used in an async context:
///
/// ```
/// let rx =  isotp_thread.read_payload().await;
/// // Do something
/// let tx_result = isotp_thread.write_payload(&[0x7F, 0x21, 0x12]).await;
/// ```
pub struct IsotpConsumer<'a, ID: CanId + 'a, C: Capacities + 'a, const N: usize> {
    tx_id: Id,
    rx_ready: SignalReader<'a, SharedIsoTpBuf<N>>,
    can_tx: &'a Arbiter<mcan::tx_buffers::Tx<'a, ID, C>>,
    rx_clear_to_send: SignalReader<'a, IsotpCtsMsg>,
    mailbox_dedicated: Option<usize>
}

impl<'a, ID: CanId + 'a, C: Capacities + 'a, const N: usize> IsotpConsumer<'a, ID, C, N> {
    /// Attempts to write an ISOTP payload to the CAN network
    pub async fn write_payload<M: embedded_hal_async::delay::DelayNs>(
        &mut self,
        mono: &mut M,
        buf: &[u8],
    ) -> Result<(), IsoTpTxErr> {
        let mut tx_buf = [0; 8];
        if buf.len() < 8 {
            tx_buf[0] = buf.len() as u8;
            tx_buf[1..1 + buf.len()].copy_from_slice(buf);
            write(self.tx_id, tx_buf, &mut *self.can_tx.access().await, self.mailbox_dedicated)?;
            Ok(())
        } else {
            // We can send
            tx_buf[0] = 0x10u8 | ((buf.len() >> 8) & 0x0F) as u8;
            tx_buf[1] = (buf.len() & 0xFF) as u8;
            tx_buf[2..].copy_from_slice(&buf[..6]);
            write(self.tx_id, tx_buf, &mut *self.can_tx.access().await, self.mailbox_dedicated)?;
            // Wait for clear to send a block
            let mut pci = 0x21;
            let mut buf_pos = 6;
            'outer: loop {
                futures::select_biased! {
                    // Timeout waiting for flow control, abort the transmission
                    _ = mono.delay_ms(1000).fuse() => {
                        break Err(IsoTpTxErr::Timeout)
                    },
                    // Receieved a flow control msg
                    fc_result = self.rx_clear_to_send.wait().fuse() => {
                        match fc_result {
                            IsotpCtsMsg::Ok { stmin, bs } => {
                                // Calculate the delay between sending frames, in microseconds (us)
                                let micros_sleep = match stmin {
                                    0 => 100, // No time delay, but wait 100us so we don't lock up the CPU
                                    1..=0x7F => stmin as u32 * 100, // Milliseconds
                                    0xF1..=0xF9 => {
                                        // Microseconds
                                        100*(stmin-0xF0) as u32
                                    },
                                    _ => {
                                        defmt::error!("Invalid STMIN received {}", stmin);
                                        100
                                    }
                                };
                                // Send the block
                                let mut bs_count = bs;
                                while bs == 0 || bs_count != 0 {
                                    // Send frame
                                    tx_buf[0] = pci;
                                    let max_copy = core::cmp::min(7, buf.len() - buf_pos);
                                    tx_buf[1..1 + max_copy]
                                        .copy_from_slice(&buf[buf_pos..buf_pos + max_copy]);
                                    write(self.tx_id, tx_buf, &mut *self.can_tx.access().await, self.mailbox_dedicated)?;
                                    buf_pos += max_copy;
                                    if buf_pos == buf.len() {
                                        // Tx complete
                                        break 'outer Ok(());
                                    }
                                    mono.delay_us(micros_sleep).await;
                                    pci = 0x20 | (pci + 1) & 0x0F;
                                    bs_count = bs_count.wrapping_sub(1);
                                }
                            }
                            IsotpCtsMsg::Wait => {
                                // Do nothing (Loop resets, waiting for the next flow control)
                            }
                            IsotpCtsMsg::Reject => {
                                write(
                                    self.tx_id,
                                    make_fc_reject(),
                                    &mut *self.can_tx.access().await,
                                    self.mailbox_dedicated
                                )?;
                                break 'outer Err(IsoTpTxErr::Rejected);
                            }
                        }
                    }
                }
            }
        }
    }

    /// Reads a received ISOTP payload
    pub async fn read_payload(&mut self) -> SharedIsoTpBuf<N> {
        self.rx_ready.wait().await
    }
}
