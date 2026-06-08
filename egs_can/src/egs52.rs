use embedded_can::{Id, StandardId};

pub use crate::can_matrix::egs_52::*;
use crate::{
    CanError, CanLayer, CanResult, CanRxData, CanTxData, RxFrame, TipTronicShifterPos,
    rxframe_default,
};

#[derive(Copy, Clone)]
pub struct Egs52Can {
    gs218: Gs218H,
    gs418: Gs418H,
    gs338: Gs338H,
    // Reading frames
    bs_200: RxFrame<Bs200H>,
    bs_208: RxFrame<Bs208H>,
    ms_210: RxFrame<Ms210H>,
    ms_212: RxFrame<Ms212H>,
    ewm_230: RxFrame<Ewm230H>,
    ms_308: RxFrame<Ms308H>,
    ms_312: RxFrame<Ms312H>,
    kla_410: RxFrame<Kla410H>,
    vg_428: RxFrame<Vg428H>,
    ms_608: RxFrame<Ms608H>,
    ezs_240: RxFrame<Ezs240H>,
    ki_408: RxFrame<Kombi408H>,
}

impl Default for Egs52Can {
    fn default() -> Self {
        let mut s = Self {
            gs218: Gs218H::ZERO,
            gs418: Gs418H::ZERO,
            gs338: Gs338H::ZERO,
            bs_200: rxframe_default!(Bs200H),
            bs_208: rxframe_default!(Bs208H),
            ms_210: rxframe_default!(Ms210H),
            ms_212: rxframe_default!(Ms212H),
            ewm_230: rxframe_default!(Ewm230H),
            ms_308: rxframe_default!(Ms308H),
            ms_312: rxframe_default!(Ms312H),
            kla_410: rxframe_default!(Kla410H),
            vg_428: rxframe_default!(Vg428H),
            ms_608: rxframe_default!(Ms608H),
            ezs_240: rxframe_default!(Ezs240H),
            ki_408: rxframe_default!(Kombi408H),
        };
        s.gs218.set_gic(EnumGic::GSnv);
        s.gs218.set_gzc(EnumGzc::GSnv);
        s.gs418.set_whst(EnumWhst::Snv);

        s
    }
}

impl Egs52Can {
    fn process_ewm_230(&self, now_ms: u32) -> CanResult<TipTronicShifterPos> {
        let ewm_230 = self.ewm_230.get(100, now_ms)?;
        match ewm_230.whc()? {
            EnumWhc::D => Ok(TipTronicShifterPos::D),
            EnumWhc::N => Ok(TipTronicShifterPos::N),
            EnumWhc::R => Ok(TipTronicShifterPos::R),
            EnumWhc::P => Ok(TipTronicShifterPos::P),
            EnumWhc::Plus => Ok(TipTronicShifterPos::Plus),
            EnumWhc::Minus => Ok(TipTronicShifterPos::Minus),
            EnumWhc::NZwD => Ok(TipTronicShifterPos::ND),
            EnumWhc::RZwN => Ok(TipTronicShifterPos::NR),
            EnumWhc::PZwR => Ok(TipTronicShifterPos::PR),
            EnumWhc::Snv => Err(CanError::SignalInvalid),
        }
    }
}

impl CanLayer<CanTxData, CanRxData> for Egs52Can {
    fn filters(&self) -> &[StandardId] {
        &[]
    }

    fn read_signals(&self, now_ms: u32, dest: &mut CanRxData) {
        dest.ewm_position = self.process_ewm_230(now_ms)
    }

    fn write_signals(&mut self, sigs: &CanTxData) {
        // Finally, calculate parity and counters
        self.gs218.set_mtgl_egs(!self.gs218.mtgl_egs());
        //self.gs218.set_mpar_egs();

        // Gear status
        //self.gs418.set_gic(match sigs.gear_actual {
        //    Some(Gear::PowerFreeInD) => EnumGic::GKraftfrei,
        //    None => EnumGic::GSnv,
        //});
    }

    fn on_frame(&mut self, now_ms: u32, id: embedded_can::Id, dlc: u8, data: &[u8; 8]) {
        match id {
            Id::Standard(Ewm230H::CAN_ID) => {
                self.ewm_230.write(Ewm230H::from_can_msg(dlc, data), now_ms)
            }
            _ => {}
        }
    }

    fn transmit<E, F: FnMut(embedded_can::StandardId, &[u8]) -> nb::Result<(), E>>(
        &self,
        mut f: F,
    ) -> nb::Result<(), E> {
        use super::SignalFrame;

        macro_rules! tx_frame {
            ($name: ident, $value: ident) => {
                f(
                    $name::CAN_ID,
                    &self.$value.to_u64_bytes()[..$name::LEN_BYTES],
                )?;
            };
        }
        tx_frame!(Gs218H, gs218);
        tx_frame!(Gs338H, gs338);
        tx_frame!(Gs418H, gs418);
        Ok(())
    }
}
