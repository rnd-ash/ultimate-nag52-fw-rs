use atsamd_hal::clock::v2::pclk;
use bsp::can_deps::Capacities;
use mcan::tx_buffers::DynTx;

pub use super::data::slave_mode::*;
use crate::{
    can::{CanLayer, RxFrame},
    handle_frames, rxframe_default,
};

pub struct SlaveCan {
    sol_rpt: SolenoidReport,
    sol_ctrl: RxFrame<SolenoidControl>,
}

impl SlaveCan {
    pub fn new() -> Self {
        let s = Self {
            sol_rpt: SolenoidReport::ZERO,
            sol_ctrl: rxframe_default!(SolenoidControl),
        };
        s
    }
}

impl CanLayer<SolenoidReport, SolenoidControl> for SlaveCan {
    fn transmit(
        &self,
        can_tx: &mut mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>,
    ) -> atsamd_hal::nb::Result<(), mcan::tx_buffers::Error> {
        can_tx.transmit_queued(self.sol_rpt.as_tx_can_msg())?;
        Ok(())
    }

    fn read_signals(&self, dest: &mut SolenoidControl) {
        if let Some(frame) = self.sol_ctrl.get(100)  {
            *dest = frame;
        } else {
            *dest = SolenoidControl::ZERO
        }
    }

    fn write_signals(&mut self, sigs: &SolenoidReport) {
        self.sol_rpt = *sigs;
    }

    fn on_frame(&mut self, id: mcan::embedded_can::Id, data: &[u8; 8]) {
        handle_frames!(self, id, data, (sol_ctrl, SolenoidControl),);
    }
}
