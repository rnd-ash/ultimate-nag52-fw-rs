use crate::{
    storage::{QspiStorageCmd, QspiStorageResp},
    tasks::elapsed_dwt_ticks,
};

use atsamd_hal::pac::DWT;
use defmt::println;
use egs_logic::{
    calbrations::{hydr::HydrCal, mech::MechCal, shift::ShiftMapCal, tcc_pump::TccPumpCal},
    storage::{
        GearboxAdaptStorage, GearboxCalibStorage, GearboxDtcStorage, GearboxMapStorage,
        StorageBacking, StoragePoll,
    },
};
use rtic_sync::channel::{Receiver, Sender};

#[derive(Default)]
pub struct EgsDtcStorage;

impl StorageBacking for EgsDtcStorage {
    fn init_poll(&mut self) -> StoragePoll {
        StoragePoll::Ready
    }
}

impl GearboxDtcStorage for EgsDtcStorage {}

#[derive(Default)]
pub struct EgsAdpStorage;

impl StorageBacking for EgsAdpStorage {
    fn init_poll(&mut self) -> StoragePoll {
        StoragePoll::Ready
    }
}

impl GearboxAdaptStorage for EgsAdpStorage {}

#[derive(Default)]
pub struct EgsCalStorage {
    hydr_cal: (HydrCal, bool),
    mech_cal: (MechCal, bool),
    tcc_pump_cal: (TccPumpCal, bool),
    shift_map_cal: (ShiftMapCal, bool),
    channel: Option<(
        Sender<'static, QspiStorageCmd, 2>,
        Receiver<'static, QspiStorageResp, 2>,
    )>,
    state: u8,
}

impl EgsCalStorage {
    pub fn set_channels(
        &mut self,
        tx: Sender<'static, QspiStorageCmd, 2>,
        rx: Receiver<'static, QspiStorageResp, 2>,
    ) {
        self.channel = Some((tx, rx))
    }
}

impl StorageBacking for EgsCalStorage {
    fn init_poll(&mut self) -> StoragePoll {
        if let Some((tx, rx)) = self.channel.as_mut() {
            // FSM to request calibration loading
            if self.state == 0 && tx.try_send(QspiStorageCmd::ReadMechCal).is_ok() {
                self.state = 1;
            } else if self.state == 1 && tx.try_send(QspiStorageCmd::ReadHydrCal).is_ok() {
                self.state = 2;
            } else if self.state == 2 && tx.try_send(QspiStorageCmd::ReadTccPumpCal).is_ok() {
                self.state = 3;
            } else if self.state == 3 && tx.try_send(QspiStorageCmd::ReadShiftMapCal).is_ok() {
                self.state = 4;
            }
            if let Ok(resp) = rx.try_recv() {
                // Query response
                match resp {
                    QspiStorageResp::MechCal(cal) => {
                        defmt::debug!("MECH CAL resp received");
                        self.mech_cal.0 = cal.unwrap_or_default();
                        self.mech_cal.1 = true;
                    }
                    QspiStorageResp::HydrCal(cal) => {
                        defmt::debug!("HYDR CAL resp received");
                        self.hydr_cal.0 = cal.unwrap_or_default();
                        self.hydr_cal.1 = true;
                    }
                    QspiStorageResp::ShiftMapCal(cal) => {
                        defmt::debug!("Shift map CAL resp received");
                        self.shift_map_cal.0 = cal.unwrap_or_default();
                        self.shift_map_cal.1 = true;
                    }
                    QspiStorageResp::TccPumpCal(cal) => {
                        defmt::debug!("TCC Pump CAL resp received");
                        self.tcc_pump_cal.0 = cal.unwrap_or_default();
                        self.tcc_pump_cal.1 = true;
                    }
                }
            }
            // All calibrations have been loaded out of DB, check validity
            if self.hydr_cal.1 && self.mech_cal.1 && self.shift_map_cal.1 && self.tcc_pump_cal.1 {
                if self.hydr_cal.0.is_valid() && self.mech_cal.0.is_valid() {
                    defmt::error!("Calibrations ready and valid!");
                    StoragePoll::Ready
                } else {
                    defmt::error!("Cannot load calibrations. Invalid data");
                    StoragePoll::Error
                }
            } else {
                StoragePoll::Waiting
            }
        } else {
            StoragePoll::Waiting
        }
    }
}

impl GearboxCalibStorage for EgsCalStorage {
    fn hydr_cal(&self) -> &egs_logic::calbrations::hydr::HydrCal {
        &self.hydr_cal.0
    }

    fn mech_cal(&self) -> &MechCal {
        &self.mech_cal.0
    }

    fn tcc_pump_cal(&self) -> &egs_logic::calbrations::tcc_pump::TccPumpCal {
        todo!()
    }

    fn shift_map_cal(&self) -> &egs_logic::calbrations::shift::ShiftMapCal {
        todo!()
    }
}

#[derive(Default)]
pub struct EgsMapStorage;

impl StorageBacking for EgsMapStorage {
    fn init_poll(&mut self) -> StoragePoll {
        StoragePoll::Ready
    }
}

impl GearboxMapStorage for EgsMapStorage {}

pub type V2Gearbox<'a> =
    egs_logic::Gearbox<'a, EgsDtcStorage, EgsCalStorage, EgsMapStorage, EgsAdpStorage>;

pub struct TickCounter {
    old: u32,
}

impl TickCounter {
    pub fn new() -> Self {
        Self {
            old: DWT::cycle_count(),
        }
    }
}

const DWT_TICKS_PER_500NS: u32 = 50;

impl egs_logic::TcuTickCounter for TickCounter {
    // 10ns per clock cycle -  500ns per tick
    fn start(&mut self) {
        self.old = DWT::cycle_count();
    }

    fn half_micros(&mut self) -> u32 {
        // Processor runs at 100Mhz (10ns per tick)
        // Divide by 50 to get 500ns resolution
        let (ticks, delta) = elapsed_dwt_ticks(self.old);
        self.old = ticks;
        delta / DWT_TICKS_PER_500NS
    }

    fn value_now_raw(&self) -> u32 {
        DWT::cycle_count()
    }

    fn half_micros_since_raw(&self, v: u32) -> u32 {
        let ticks = elapsed_dwt_ticks(v).1;
        ticks / DWT_TICKS_PER_500NS
    }
}
