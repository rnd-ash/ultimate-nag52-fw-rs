use atsamd_hal::{clock::v2::pclk, nb, rtic_time::Monotonic};
use bsp::can_deps::Capacities;
use mcan::embedded_can;

use crate::{can::egs52::Egs52Can, Mono};

pub mod data;
pub mod egs52;

/// Creates a default [RxFrame]
#[macro_export]
macro_rules! rxframe_default {
    ($frame_ty:ident) => {
        RxFrame::<$frame_ty> {
            frame: $frame_ty::ZERO,
            timestamp_ms: 0,
            seen: false,
        }
    };
}

/// Rx frame with timeout
///
/// ECU uses these to verify that the
/// CAN data is not stagnent
#[derive(Copy, Clone)]
pub struct RxFrame<T: Copy> {
    frame: T,
    timestamp_ms: u64,
    seen: bool,
}

impl<T: Copy> RxFrame<T> {
    /// Returns [None] if the frame has never been seen on the bus, or is stagnent
    /// otherwise, returns the CAN frame
    pub fn get(&self, max_ms: u64) -> Option<T> {
        if self.seen {
            if Mono::now().duration_since_epoch().to_millis() - self.timestamp_ms > max_ms {
                None
            } else {
                Some(self.frame)
            }
        } else {
            None
        }
    }

    /// Logs a new incomming frame
    pub fn write(&mut self, v: T) {
        self.seen = true;
        self.frame = v;
        self.timestamp_ms = Mono::now().duration_since_epoch().to_millis()
    }

    /// Returns true if the frame has been seen on the bus at some point
    /// in the past
    pub fn has_been_seen(&self) -> bool {
        self.seen
    }
}

pub trait CanLayer {
    fn on_frame(&mut self, id: embedded_can::Id, data: &[u8; 8]);
    fn read_signals(&self);
    fn write_signals(&mut self);
    fn transmit(
        &self,
        can_tx: &mut mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>,
    ) -> nb::Result<(), mcan::tx_buffers::Error>;
}

pub enum CanLayerTy {
    Egs52(Egs52Can),
}

impl CanLayerTy {
    pub fn on_frame(&mut self, id: embedded_can::Id, data: &[u8; 8]) {
        self.as_can_layer_mut().on_frame(id, data);
    }

    pub fn read_signals(&self) {
        self.as_can_layer().read_signals();
    }

    pub fn write_signals(&mut self) {
        self.as_can_layer_mut().write_signals();
    }

    pub fn transmit(
        &self,
        can_tx: &mut mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>,
    ) -> nb::Result<(), mcan::tx_buffers::Error> {
        self.as_can_layer().transmit(can_tx)
    }

    fn as_can_layer(&self) -> &impl CanLayer {
        match self {
            CanLayerTy::Egs52(egs52_can) => egs52_can,
        }
    }

    fn as_can_layer_mut(&mut self) -> &mut impl CanLayer {
        match self {
            CanLayerTy::Egs52(egs52_can) => egs52_can,
        }
    }
}
