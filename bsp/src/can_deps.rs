use atsamd_hal::{can::Dependencies, pac};
use mcan::{message::{rx, tx}, messageram::SharedMemory, rx_fifo::{Fifo0, RxFifo}};

use crate::{CanRx, CanTx};
use mcan::generic_array::typenum::{*};
use atsamd_hal::clock::v2::types::Can0;
pub struct Capacities;

// rx::Message<8> since EGS does not deal with CANFD (So max 8 byte message)
impl mcan::messageram::Capacities for Capacities {
    type StandardFilters = U1; // TODO (Add filters as needed - Can be up to U128)
    type ExtendedFilters = U0; // No extended CAN support (Vehicles don't use it)
    type RxBufferMessage = rx::Message<8>;
    type DedicatedRxBuffers = U0;
    // Use RxFIFO0 with 64 slots (each for an 8 byte msg)
    type RxFifo0Message = rx::Message<8>;
    type RxFifo0 = U64;
    // We don't use FIFO1 (Set size to 0)
    type RxFifo1Message = rx::Message<8>;
    type RxFifo1 = U0;
    type TxMessage = tx::Message<8>;
    // Maximum size of any EGS CAN layer is 6 CAN Frames outputted (+1 for diag)
    // We can therefore optimize RAM better by having only 10 Tx msgs
    type TxBuffers = U10;  // Up to 10 messages to Tx...
    // ...of which 7 frames have their own dedicated 'slots'
    type DedicatedTxBuffers = U7;
    type TxEventFifo = U32;
}

pub const CAN_MEM_ADDR: usize = 0x2000_0000;
pub const CAN_MEM_RAM_SIZE: usize = 2048; // Based on size of Capacities

pub const CAN_TX_MAILBOX_DIAG: usize = 0;
pub const CAN_RX_MAILBOX_DIAG: usize = 0;

pub type RxFifo0 =
    RxFifo<'static, Fifo0, Can0, <Capacities as mcan::messageram::Capacities>::RxFifo0Message>;

pub type Can0Tx = mcan::tx_buffers::Tx<'static, Can0, Capacities>;

pub type Can0TxEventFifo = mcan::tx_event_fifo::TxEventFifo<'static, Can0>;

pub type Can0Aux<GclkId> = mcan::bus::Aux<
    'static,
    Can0,
    Dependencies<Can0, GclkId, CanRx, CanTx, pac::Can0>,
>;