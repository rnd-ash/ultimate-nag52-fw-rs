use core::sync::atomic::Ordering;

use atsamd_hal::{clock::v2::pclk, dmac, fugit::ExtU64, rtic_time::Monotonic};
use bsp::can_deps::Capacities;
use rtic::Mutex;
use rtic_sync::arbiter::Arbiter;

use crate::{DEVICE_MODE, Mono, app, can::{input_output::{CanRxSignals, CanTxSignals}, slave::{FullReport, SensorReport, SolenoidControl, SolenoidReport}}, diag::dev_mode::EgsDeviceMode, sensors::TftState, solenoids::SolenoidControler};



pub async fn gearbox_task(
    mut cx: app::gearbox_task::Context<'_>,
    can_tx: &'static Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>,
    mut solenoid_controller: SolenoidControler<dmac::Ch0, dmac::Ch1>,
) {
    // Gearbox has started, set device mode
    DEVICE_MODE.store(EgsDeviceMode::SLAVE.bits(), Ordering::Relaxed);
    let mut can_input = CanRxSignals::default();
    let can_output = CanTxSignals::default();
    loop {
        let now = Mono::now();
        let mode = EgsDeviceMode::from_bits_retain(DEVICE_MODE.load(Ordering::Relaxed));
        let sensor_data = cx.shared.sensor_data.lock(|l| l.clone());

        if !mode.contains(EgsDeviceMode::SLAVE) {
            cx.shared
                .can_layer
                .lock(|lck| lck.read_signals(&mut can_input));
            // Gearbox stuff
            let mut can = can_tx.access().await;
            let _ = cx.shared.can_layer.lock(|lck| {
                lck.write_signals(&can_output);
                lck.transmit(&mut can)
            });
        } else {
            // Slave mode has special logic
            use crate::can::CanLayer;
            let mut control_frame = SolenoidControl::ZERO;
            cx.shared
                .slave_can
                .lock(|lck| lck.read_signals(&mut control_frame));

            let spc_req = control_frame.spc_req().swap_bytes();
            let mpc_req = control_frame.mpc_req().swap_bytes();
            let tcc_req = control_frame.tcc_req();

            solenoid_controller.set_mpc_current(mpc_req).await;
            solenoid_controller.set_spc_current(spc_req).await;
            solenoid_controller.set_y3(control_frame.y_3_en()).await;
            solenoid_controller.set_y4(control_frame.y_4_en()).await;
            solenoid_controller.set_y5(control_frame.y_5_en()).await;
            solenoid_controller.update_task().await;

            let observed_pwm = cx.shared.soltcc.lock(|tcc| {
                solenoid_controller.set_tcc_pwm((tcc_req as u16) << 8, tcc);
                solenoid_controller.get_observed_tcc_pwm(tcc)
            }) >> 8;

            // Actuate solenoids
            // Todo - Only do delay and second query if first failed
            let mut rpt = SolenoidReport::ZERO;
            let mut sensor_rpt = SensorReport::ZERO;
            let spc_current = solenoid_controller.read_spc_current();
            let mpc_current = solenoid_controller.read_mpc_current();

            rpt.set_mpc_curr(mpc_current.swap_bytes());
            rpt.set_spc_curr(spc_current.swap_bytes());
            rpt.set_tcc_pwm(observed_pwm as u8);
            rpt.set_y_3_on(solenoid_controller.read_y3_current() > 100);
            rpt.set_y_4_on(solenoid_controller.read_y4_current() > 100);
            rpt.set_y_5_on(solenoid_controller.read_y5_current() > 100);

            match sensor_data.tft {
                TftState::Pll => sensor_rpt.set_tft(0xFF),
                TftState::Temperature(temp) => {
                    sensor_rpt.set_tft((temp + 50) as u8);
                }
            }
            sensor_rpt.set_vbatt((sensor_data.vkl15 / 100) as u8);
            sensor_rpt.set_n_2_raw(sensor_data.ikl87);
            //sensor_rpt.set_n_3_raw(sensor_data.n3_rpm * 50);

            let fr = FullReport {
                solenoids: rpt,
                sensors: sensor_rpt,
            };

            let mut can = can_tx.access().await;
            let _ = cx.shared.slave_can.lock(|lck| {
                lck.write_signals(&fr);
                lck.transmit(&mut can)
            });
        }

        Mono::delay_until(now + 20u64.millis()).await;
    }
}