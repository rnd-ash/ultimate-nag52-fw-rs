use atsamd_hal::{clock::v2::pclk, dmac, fugit::ExtU64, pac::DWT, rtic_time::Monotonic};
use bsp::can_deps::Capacities;
use egs_logic::{
    TcuTickCounter,
    egs_can::{
        CanLayer,
        slave::{SlaveReq, SlaveStatus},
    },
    errors::DeviceMode,
    vars::GearboxOutputs,
};
use mcan::embedded_can::StandardId;
use mcan::message::tx::Message;
use mcan::{embedded_can::Id, message::tx::MessageBuilder, tx_buffers::DynTx};
use rtic::Mutex;
use rtic_sync::arbiter::Arbiter;

use crate::{
    Mono, app, egs_logic_impl::TickCounter, solenoids::SolenoidController, tasks::elapsed_dwt_ticks,
};
use crate::{diag::SolenoidReport, storage::eeprom::Eeprom};

fn build_can_msg(id: StandardId, data: &[u8]) -> Message<8> {
    MessageBuilder {
        id: Id::Standard(id),
        frame_type: mcan::message::tx::FrameType::Classic(
            mcan::message::tx::ClassicFrameType::Data(data),
        ),
        store_tx_event: None,
    }
    .build()
    .unwrap()
}

pub async fn gearbox_init(
    cx: &mut app::gearbox_task::Context<'_>,
    sol: &mut SolenoidController<dmac::Ch0, dmac::Ch1>,
    mut eeprom: Eeprom<dmac::Ch2>,
) {
    eeprom.init().await;

    match Mono::timeout_after(1000u64.millis(), sol.init()).await {
        Ok(false) | Err(_) => {
            cx.shared
                .gearbox
                .lock(|lck| lck.set_dev_mode(DeviceMode::HARDWARE_ERR));
        }
        Ok(true) => {}
    }
}

pub async fn write_all_solenoids(
    solenoid_controller: &mut SolenoidController<dmac::Ch0, dmac::Ch1>,
    outputs: &GearboxOutputs,
) -> bool {
    solenoid_controller.en_high_side_power(outputs.solenoid_pwr_en);
    solenoid_controller.set_y3(outputs.y3_en).await;
    solenoid_controller.set_y4(outputs.y4_en).await;
    solenoid_controller.set_y5(outputs.y5_en).await;

    if solenoid_controller
        .set_mpc_current(outputs.mpc_current)
        .await
        .is_err()
        || solenoid_controller
            .set_spc_current(outputs.spc_current)
            .await
            .is_err()
    {
        solenoid_controller.en_high_side_power(false);
        false
    } else {
        true
    }
}

pub async fn gearbox_task(
    mut cx: app::gearbox_task::Context<'_>,
    can_tx: &'static Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>,
    eeprom: Eeprom<dmac::Ch2>,
    mut solenoid_controller: SolenoidController<dmac::Ch0, dmac::Ch1>,
) {
    gearbox_init(&mut cx, &mut solenoid_controller, eeprom).await;

    let mut slave_req = SlaveReq::default();
    let slave_res = SlaveStatus::default();

    let mut tick_counter = TickCounter::new();
    let mut last_dwt_time = DWT::cycle_count();
    let mut us_since_boot: u64 = 0;
    loop {
        // Calculate time to now that the loop actually took
        let (new_dwt, delta) = elapsed_dwt_ticks(last_dwt_time);
        last_dwt_time = new_dwt;
        us_since_boot += delta as u64 / 100;
        tick_counter.start();
        let now = Mono::now();
        let now_ms = now.duration_since_epoch().to_millis() as u32;

        let sensor_data_now = cx.shared.sensor_data.lock(|x| **x);

        // Set pulse information up
        let pulse_data = cx.local.speed_sensors.update();

        let mut perf_ticks_input = 0;
        let (gearbox_mode, outputs) = cx.shared.gearbox.lock(|gearbox| {
            // Process signals
            cx.shared.can_layer.lock(|can| {
                can.read_signals(now_ms, &mut gearbox.inputs_mut().can_vars);
            });

            gearbox.inputs_mut().n2_pulses_raw = pulse_data.pulses_n2 as u16;
            gearbox.inputs_mut().n3_pulses_raw = pulse_data.pulses_n3 as u16;

            gearbox.inputs_mut().atf = Some(sensor_data_now.tft);
            gearbox.inputs_mut().clock_time_us = us_since_boot;
            //gearbox.inputs_mut().n2_rpm = Some(sensor_data_now.)

            perf_ticks_input = tick_counter.half_micros() as u16;
            gearbox.update(&mut tick_counter);
            (gearbox.dev_mode(), *gearbox.outputs())
        });

        // Write to CAN logic
        {
            let mut can_tx = can_tx.access().await;

            if gearbox_mode.contains(DeviceMode::SLAVE) {
                // Slave logic
                cx.shared.slave_can.lock(|lck| {
                    // Reduce locking by reading here, data will be
                    // applied in the next EGS cycle
                    lck.read_signals(now_ms, &mut slave_req);
                    lck.write_signals(now_ms, &slave_res);
                    let _ =
                        lck.transmit(|id, data| can_tx.transmit_queued(build_can_msg(id, data)));
                });
            } else {
                // EGS logic
                cx.shared.can_layer.lock(|can_layer| {
                    can_layer.write_signals(now_ms, &outputs.can_outputs);
                    let _ = can_layer
                        .transmit(|id, data| can_tx.transmit_queued(build_can_msg(id, data)));
                });
            }
        }

        // Write to solenoids
        match Mono::timeout_after(
            20u64.millis(),
            write_all_solenoids(&mut solenoid_controller, &outputs),
        )
        .await
        {
            Ok(res) => {
                if !res {
                    cx.shared
                        .gearbox
                        .lock(|lck| lck.set_dev_mode(DeviceMode::HARDWARE_ERR));
                }
            }
            Err(_) => {
                defmt::error!("Write to solenoids timeout")
            }
        }

        cx.shared.soltcc.lock(|tcc| {
            solenoid_controller.set_tcc_pwm(outputs.tcc_pwm, tcc);
        });
        solenoid_controller.update_task().await;
        let rpt = SolenoidReport {
            tcc_pwm_target: outputs.tcc_pwm,
            tcc_pwm_recorded: cx
                .shared
                .soltcc
                .lock(|lck| solenoid_controller.get_observed_tcc_pwm(lck)),
            pwm_target_y3: solenoid_controller.get_y3_pwm(),
            pwm_target_y4: solenoid_controller.get_y4_pwm(),
            pwm_target_y5: solenoid_controller.get_y5_pwm(),
            current_target_mpc: outputs.mpc_current,
            current_target_spc: outputs.spc_current,
            current_measured_y3: solenoid_controller.read_y3_current(),
            current_measured_y4: solenoid_controller.read_y4_current(),
            current_measured_y5: solenoid_controller.read_y5_current(),
            current_measured_spc: solenoid_controller.read_spc_current(),
            current_measured_mpc: solenoid_controller.read_mpc_current(),
            current_measured_trrs: 0,
            current_measured_gpio: 0,
        };
        cx.shared.solenoid_statuses.lock(|lck| *lck = rpt);
        let perf_ticks_output = tick_counter.half_micros() as u16;
        cx.shared.perf_stats.lock(|lck| {
            lck.us_output_funcs = perf_ticks_output;
            lck.us_input_funcs = perf_ticks_input;
        });
        Mono::delay_until(now + 20u64.millis()).await;
    }
}
