use atsamd_hal::{fugit::ExtU64, rtic_time::Monotonic};
use egs_logic::calbrations::mech::MechCal;
use rtic::Mutex;
use rtic_sync::channel::{Receiver, Sender};

use crate::{
    Mono, app,
    storage::{
        EKV_KEY_HYDR_CAL, EKV_KEY_MECH_CAL, EKV_KEY_SHIFT_MAP_CAL, EKV_KEY_TCC_PUMP_CAL,
        QspiStorageCmd, QspiStorageResp,
    },
};

pub async fn qspi_commander_thread(
    mut cx: app::qspi_flash_commander::Context<'_>,
    mut rx_cmd: Receiver<'static, QspiStorageCmd, 2>,
    mut tx_resp: Sender<'static, QspiStorageResp, 2>,
) {
    let seed = cx.shared.trng.lock(|lck| lck.random_u32());
    let qspi = *cx.shared.qspi;
    let mut db = crate::storage::qspi::QspiStorageDb::new(qspi, seed).await;
    loop {
        if let Ok(req) = rx_cmd.recv().await {
            match req {
                QspiStorageCmd::ReadMechCal => {
                    defmt::debug!("Asked to read MECH CAL");
                    let cal = db.get_key(EKV_KEY_MECH_CAL).await;
                    tx_resp.send(QspiStorageResp::MechCal(cal)).await;
                }
                QspiStorageCmd::ReadHydrCal => {
                    defmt::debug!("Asked to read HYDR CAL");
                    let cal = db.get_key(EKV_KEY_HYDR_CAL).await;
                    tx_resp.send(QspiStorageResp::HydrCal(cal)).await;
                }
                QspiStorageCmd::ReadShiftMapCal => {
                    defmt::debug!("Asked to read Shift map CAL");
                    let cal = db.get_key(EKV_KEY_SHIFT_MAP_CAL).await;
                    tx_resp.send(QspiStorageResp::ShiftMapCal(cal)).await;
                }
                QspiStorageCmd::ReadTccPumpCal => {
                    defmt::debug!("Asked to read TCC Pump CAL");
                    let cal = db.get_key(EKV_KEY_TCC_PUMP_CAL).await;
                    tx_resp.send(QspiStorageResp::TccPumpCal(cal)).await;
                }
            }
        }
    }
}
