use atsamd_hal::clock::v2::pclk;
use bsp::can_deps::Capacities;
use mcan::tx_buffers::DynTx;

pub use super::data::egs_52::*;
use crate::{
    can::{
        input_output::{CanRxSignals, CanTxSignals},
        CanLayer, RxFrame,
    },
    handle_frames, rxframe_default,
};

pub struct Egs52Can {
    gs218: Gs218H,
    gs418: Gs418H,
    gs338: Gs338H,
    // Reading frames
    ewm_230: RxFrame<Ewm230H>,
    ms_210: RxFrame<Ms210H>,
    ms_308: RxFrame<Ms308H>,
    ms_608: RxFrame<Ms608H>,
}

impl Egs52Can {
    pub fn new() -> Self {
        let mut s = Self {
            gs218: Gs218H::ZERO,
            gs418: Gs418H::ZERO,
            gs338: Gs338H::ZERO,
            ewm_230: rxframe_default!(Ewm230H),
            ms_210: rxframe_default!(Ms210H),
            ms_308: rxframe_default!(Ms308H),
            ms_608: rxframe_default!(Ms608H),
        };
        s.gs218.set_gic(EnumGic::GSnv);
        s.gs218.set_gzc(EnumGzc::GSnv);
        s.gs418.set_whst(EnumWhst::Snv);

        s
    }
}

impl CanLayer<CanTxSignals, CanRxSignals> for Egs52Can {
    fn transmit(
        &self,
        can_tx: &mut mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>,
    ) -> atsamd_hal::nb::Result<(), mcan::tx_buffers::Error> {
        // Set toggle bits and counters
        can_tx.transmit_queued(self.gs218.as_tx_can_msg())?;
        can_tx.transmit_queued(self.gs418.as_tx_can_msg())?;
        can_tx.transmit_queued(self.gs338.as_tx_can_msg())?;
        Ok(())
    }

    fn read_signals(&self, _dest: &mut CanRxSignals) {
        if let Some(ewm230) = self.ewm_230.get(1000) {
            let _whc = ewm230.whc();
            //defmt::info!("Valid ewm230: {}  {:?}", ewm230, whc);
        }
    }

    fn write_signals(&mut self, _sigs: &CanTxSignals) {
        // Finally, calculate parity and counters
        self.gs218.set_mtgl_egs(!self.gs218.mtgl_egs());
        //self.gs218.set_mpar_egs();
    }

    fn on_frame(&mut self, id: mcan::embedded_can::Id, data: &[u8; 8]) {
        handle_frames!(
            self,
            id,
            data,
            (ewm_230, Ewm230H),
            (ms_210, Ms210H),
            (ms_308, Ms308H),
            (ms_608, Ms608H),
        );
    }
}
