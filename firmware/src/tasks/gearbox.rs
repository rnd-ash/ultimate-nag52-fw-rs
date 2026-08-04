use core::ops::Sub;

use atsamd_hal::{
    clock::v2::pclk,
    dmac,
    fugit::{ExtU64, Instant},
    nb,
    rtic_time::{Monotonic, monotonic::TimerQueueBasedDuration},
    timer::TimerCounter7,
    timer_traits::InterruptDrivenTimer,
};
use bsp::can_deps::Capacities;
use cortex_m::prelude::_embedded_hal_watchdog_Watchdog;
use egs_logic::{
    TcuTickCounter,
    egs_can::{
        CanLayer,
        slave::{SlaveReq, SlaveStatus},
    },
    errors::DeviceMode,
};
use mcan::{
    embedded_can::{self, Id},
    message::tx::MessageBuilder,
    tx_buffers::DynTx,
};
use rtic::Mutex;
use rtic_sync::arbiter::Arbiter;

use crate::{Mono, app, egs_logic_impl::V2Gearbox, solenoids::SolenoidControler};

pub async fn gearbox_task(
    mut cx: app::gearbox_task::Context<'_>,
    can_tx: &'static Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>,
    mut solenoid_controller: SolenoidControler<dmac::Ch0, dmac::Ch1>,
) {
    let mut gearbox = V2Gearbox::default();
    let mut inputs = egs_logic::GearboxInputs::default();
    let mut outputs = egs_logic::GearboxOutputs::default();
    let mut slave_req = SlaveReq::default();
    let mut slave_res = SlaveStatus::default();

    //gearbox.set_dev_mode(DeviceMode::SLAVE);
    loop {
        cx.local.hpet.start();
        let mut perf_timer_tmp = 0;
        let now = Mono::now();
        let now_ms = now.duration_since_epoch().to_millis() as u32;

        // Or EGS device mode with one that is global
        let extern_dev_mode = cx
            .shared
            .device_mode
            .load(core::sync::atomic::Ordering::Relaxed);
        gearbox.set_dev_mode(DeviceMode::from_bits_truncate(extern_dev_mode));

        // Process signals
        if !gearbox.dev_mode().contains(DeviceMode::SLAVE) {
            cx.shared.can_layer.lock(|can| {
                can.read_signals(now_ms, &mut inputs.can_vars);
            })
        }

        cx.shared
            .sensor_data
            .lock(|sensors| inputs.atf = Some(sensors.tft));
        let perf_ticks_input = cx.local.hpet.ticks();
        gearbox.update(&inputs, &mut outputs).await;
        let perf_timer_tmp = cx.local.hpet.ticks();
        let perf_ticks_gearbox = perf_timer_tmp - perf_ticks_input;

        // Set the Device mode based on EGS layer
        cx.shared.device_mode.store(
            gearbox.dev_mode().bits(),
            core::sync::atomic::Ordering::Relaxed,
        );

        // Write to CAN logic
        {
            let mut can_tx = can_tx.access().await;

            if gearbox.dev_mode().contains(DeviceMode::SLAVE) {
                // Slave logic
                cx.shared.slave_can.lock(|lck| {
                    // Reduce locking by reading here, data will be
                    // applied in the next EGS cycle
                    lck.read_signals(now_ms, &mut slave_req);
                    lck.write_signals(now_ms, &slave_res);
                    let _ = lck.transmit(|id, data| {
                        let msg = MessageBuilder {
                            id: Id::Standard(id),
                            frame_type: mcan::message::tx::FrameType::Classic(
                                mcan::message::tx::ClassicFrameType::Data(data),
                            ),
                            store_tx_event: None,
                        }
                        .build()
                        .unwrap();
                        can_tx.transmit_queued(msg)
                    });
                });
            } else {
                // EGS logic
                cx.shared.can_layer.lock(|can_layer| {
                    can_layer.write_signals(now_ms, &outputs.can_outputs);
                    let _ = can_layer.transmit(|id, data| {
                        let msg = MessageBuilder {
                            id: Id::Standard(id),
                            frame_type: mcan::message::tx::FrameType::Classic(
                                mcan::message::tx::ClassicFrameType::Data(data),
                            ),
                            store_tx_event: None,
                        }
                        .build()
                        .unwrap();
                        can_tx.transmit_queued(msg)
                    });
                });
            }
        }
        // Write to solenoids

        solenoid_controller.set_y3(outputs.y3_en).await;
        solenoid_controller.set_y3(outputs.y4_en).await;
        solenoid_controller.set_y3(outputs.y5_en).await;
        solenoid_controller
            .set_mpc_current(outputs.mpc_current)
            .await;
        solenoid_controller
            .set_spc_current(outputs.spc_current)
            .await;
        cx.shared.soltcc.lock(|tcc| {
            solenoid_controller.set_tcc_pwm(outputs.tcc_pwm, tcc);
        });
        solenoid_controller.update_task().await;
        cx.shared.outputs.lock(|out| *out = outputs);

        // Poll the watchdog in the main gearbox task
        cx.shared.wdt.lock(|wdt| wdt.feed());

        let perf_ticks_output = cx.local.hpet.ticks() - perf_timer_tmp;
        cx.shared.perf_stats.lock(|lck| {
            lck.us_output_funcs = perf_ticks_output;
            lck.us_input_funcs = perf_ticks_input;
            lck.us_process_func = perf_ticks_gearbox;
        });
        Mono::delay_until(now + 20u64.millis()).await;
    }
}
