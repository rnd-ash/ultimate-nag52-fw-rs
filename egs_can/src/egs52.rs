use core::num::NonZeroU8;

use arbitrary_int::{UInt, traits::Integer, u5, u13};
use embedded_can::{StandardId};
use num_traits::clamp;

use crate::bit_checking::SafeCanValue;
pub use crate::can_matrix::egs_52::*;
use crate::{
    CanError, CanLayer, CanResult, CanRxData, CanTxData, ShiftPaddlePos,
    TipTronicShifterPos, TorqueOutputInfo,
};

trait TorqueToFloat {
    fn value_nm(self) -> f32;
}

impl TorqueToFloat for UInt<u16, 13> {
    fn value_nm(self) -> f32 {
        (self.value() as f32 / 4.0) - 500.0
    }
}

crate::make_rx_frames!(Egs52Rx {
    // ABS/ESP/SBC
    Bs200H,
    Bs208H,
    // Engine
    Ms210H,
    Ms212H,
    Ms308H,
    Ms312H,
    Ms608H,
    // EWM Shifter
    Ewm230H,
    // Transfer case
    Vg428H,
    // Shift-by-wire (Paddles)
    Sbw232H,
    // Cluster
    Kombi408H,
    // EZS
    Ezs240H,
    // AC information (For torque lost due to AC on)
    Kla410H
});

#[derive(Copy, Clone)]
pub struct Egs52Can {
    gs218: Gs218H,
    gs418: Gs418H,
    gs338: Gs338H,
    // Reading frames
    rx_frames: Egs52Rx,
    m_esp_m_sta_delta: f32,
    freeze_sta_trq: bool,
    cvn_counter: u5,
}

impl Default for Egs52Can {
    fn default() -> Self {
        let mut s = Self {
            gs218: Gs218H::ZERO,
            gs418: Gs418H::ZERO,
            gs338: Gs338H::ZERO,
            rx_frames: Default::default(),
            m_esp_m_sta_delta: 0.0,
            freeze_sta_trq: false,
            cvn_counter: u5::default(),
        };
        s.gs218.set_gic(EnumGic::GSnv);
        s.gs218.set_gzc(EnumGzc::GSnv);

        s.gs218.set_calid_cvn_akt(true);

        s.gs418.set_whst(EnumWhst::Snv);

        s
    }
}

impl Egs52Can {
    fn process_ewm_230(&self, now_ms: u32) -> CanResult<TipTronicShifterPos> {
        let ewm_230 = self.rx_frames.ewm230_h.get(100, now_ms)?;
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

    fn process_engine_torque(&self, now_ms: u32) -> TorqueOutputInfo {
        let mut ret = TorqueOutputInfo::default();

        let ms312 = self.rx_frames.ms312_h.get(100, now_ms);

        // Transformer function for 13 bit torque values to float
        let torque_to_f32 = |x: UInt<u16, 13>| (x.value() as f32 / 4.0) - 500.0;

        ret.max_torque_nm = ms312.and_then(|ms| ms.m_max().with_valid(torque_to_f32));
        ret.min_torque_nm = ms312.and_then(|ms| ms.m_min().with_valid(torque_to_f32));
        ret.driver_req_torque_nm = self
            .rx_frames
            .ms212_h
            .get(100, now_ms)
            .and_then(|x| x.m_espv().with_valid(torque_to_f32));

        let static_trq = ms312.and_then(|ms| ms.m_sta().with_valid(torque_to_f32));
        //if let Ok(static) = static_trq && let Ok(driver) = ret.driver_req_torque_nm {
        //    let should_freeze = self.gs218.mmax_egs() | self.gs218.mmin_egs();
        //}
        

        // FMOTMAX compensation for max torque based on altitude
        if let Ok(m_max) = &mut ret.max_torque_nm {
            let factor = self.rx_frames.ms210_h.get(100, now_ms).map(|x| x.fmmotmax()).unwrap_or(100);
            *m_max *= (factor as f32 / 100.0);
        }

        ret
    }

    fn process_engine_rpm(&self, now_ms: u32) -> CanResult<u16> {
        self.rx_frames
            .ms308_h
            .get(100, now_ms)
            .map(|x| x.nmot())
            .check_valid()
    }

    fn process_engine_m_sta(&self, now_ms: u32) -> CanResult<f32> {
        let raw = self
            .rx_frames
            .ms312_h
            .get(100, now_ms)
            .map(|x| x.m_sta())
            .check_valid()?;
        Ok(((raw.as_::<u16>() as f32) / 4.0) - 500.0)
    }

    fn process_paddle_pos(&self, now_ms: u32) -> CanResult<ShiftPaddlePos> {
        let pos = match self.rx_frames.sbw232_h.get(100, now_ms)?.lrt_pm_3() {
            EnumLrtPm3::Plus => ShiftPaddlePos::Plus,
            EnumLrtPm3::Minus => ShiftPaddlePos::Minus,
            EnumLrtPm3::PlusMinus => ShiftPaddlePos::Both,
            _ => ShiftPaddlePos::None,
        };
        Ok(pos)
    }

    /// Gear limitation processing. The powertrain ECUs can request that the gearbox
    /// limits its gearing between either a min, max, or range of gears.
    ///
    /// The order of priority (Highest to lowest) is:
    /// 1. ESP
    /// 2. MS
    fn process_req_gear_limits(&self, now_ms: u32) -> (Option<NonZeroU8>, Option<NonZeroU8>) {
        // Order of priority
        //
        // 1. ESP/ABS
        // 2. MS (Engine)

        // Safety - gmin and gmax signals from either ECUs are the same layout (0 = Passive -> 7 = D7)
        let (esp_min, esp_max) = self
            .rx_frames
            .bs208_h
            .get(100, now_ms)
            .map(|x| (x.gmin_esp() as u8, x.gmax_esp() as u8))
            .unwrap_or_default();
        let (ms_min, ms_max) = self
            .rx_frames
            .ms210_h
            .get(100, now_ms)
            .map(|x| (x.gmin_ms() as u8, x.gmax_ms() as u8))
            .unwrap_or_default();

        let mut min = esp_min;
        if min == 0 {
            min = ms_min;
        }

        let mut max = ms_max;
        if esp_max < max && esp_max != 0 {
            max = esp_max;
        }

        (NonZeroU8::new(min), NonZeroU8::new(max))
    }
}

impl CanLayer<CanTxData, CanRxData> for Egs52Can {
    fn filters(&self) -> &[StandardId] {
        Egs52Rx::filters()
    }

    fn read_signals(&self, now_ms: u32, dest: &mut CanRxData) {
        dest.engine_rpm = self.process_engine_rpm(now_ms);
        dest.ewm_position = self.process_ewm_230(now_ms);
        dest.engine_static_torque_nm = self.process_engine_m_sta(now_ms);
    }

    fn write_signals(&mut self, now_ms: u32, sigs: &CanTxData) {
        // Torque request processing
        match sigs.torque_req {
            None => {
                self.gs218.set_dyn_0_amr_egs(false);
                self.gs218.set_dyn_1_egs(false);
                self.gs218.set_mmin_egs(false);
                self.gs218.set_mmax_egs(false);
                self.gs218.set_m_egs(u13::default());
                // IMPORTANT - Calculate Torque delta (Required to compute real static torque)
                let sta = self
                    .rx_frames
                    .ms312_h
                    .get(100, now_ms)
                    .and_then(|x| x.m_sta().with_valid(|t| t.value() as f32 / 4.0));
                let esp = self
                    .rx_frames
                    .ms212_h
                    .get(100, now_ms)
                    .and_then(|x| x.m_espv().with_valid(|t| t.value() as f32 / 4.0));
                if let Ok(m_sta) = sta
                    && let Ok(m_esp) = esp
                {
                    // Compute delta
                    self.m_esp_m_sta_delta = m_esp - m_sta;
                } else {
                    self.m_esp_m_sta_delta = 0.0;
                }
                self.freeze_sta_trq = false;
            }
            Some(req) => {
                self.freeze_sta_trq = true;
                self.gs218.set_dyn_0_amr_egs(true);
                self.gs218.set_dyn_1_egs(req.ramp_end);
                self.gs218.set_mmin_egs(true);
                let amount = u13::try_new(((req.amount_nm * 4.0) as u16) + 500).unwrap_or_default();
                self.gs218.set_m_egs(amount);
            }
        }
        self.gs218.set_mtgl_egs(!self.gs218.mtgl_egs());
        // Parity calculations
        let gs218_bytes01 = {
            let bytes = self.gs218.to_u64_bytes();
            u16::from_le_bytes(bytes[0..2].try_into().unwrap())
        };
        self.gs218.set_mpar_egs(gs218_bytes01.count_ones() % 2 != 0);
        // Error status bits
        self.gs218.set_fehler(self.cvn_counter);
        self.cvn_counter = self.cvn_counter.wrapping_add(u5::new(1));

        // TFT related signals
        self.gs218.set_alf(sigs.can_start);
        self.gs418
            .set_t_get(clamp(sigs.gearbox_temperature_c + 50, 0, 254) as u8);

        // Gearbox status outputs
        self.gs218.set_gs_notl(!sigs.gearbox_ok);
        self.gs218.set_get_ok(!sigs.gearbox_ok);

        // Finally, calculate parity and counters

        //self.gs218.set_mpar_egs();

        // Gear status
        //self.gs418.set_gic(match sigs.gear_actual {
        //    Some(Gear::PowerFreeInD) => EnumGic::GKraftfrei,
        //    None => EnumGic::GSnv,
        //});
    }

    fn on_frame(&mut self, now_ms: u32, id: embedded_can::Id, dlc: u8, data: &[u8; 8]) {
        self.rx_frames.on_rx_frame(now_ms, id, dlc, data);
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
