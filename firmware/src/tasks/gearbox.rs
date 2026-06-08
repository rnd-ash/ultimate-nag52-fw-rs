use atsamd_hal::{clock::v2::pclk, dmac, fugit::ExtU64, nb, rtic_time::Monotonic};
use bsp::can_deps::Capacities;
use egs_logic::{egs_can::{CanLayer, slave::{SlaveReq, SlaveStatus}}, errors::DeviceMode};
use mcan::{embedded_can::{self, Id}, message::tx::MessageBuilder, tx_buffers::DynTx};
use rtic::Mutex;
use rtic_sync::arbiter::Arbiter;

use crate::{Mono, app, egs_logic_impl::V2Gearbox, solenoids::SolenoidControler};

pub async fn gearbox_task(
    mut cx: app::gearbox_task::Context<'_>,
    can_tx: &'static Arbiter<mcan::tx_buffers::Tx<'static, pclk::ids::Can0, Capacities>>,
    mut solenoid_controller: SolenoidControler<dmac::Ch0, dmac::Ch1>,
) {
    let mut gearbox = V2Gearbox::default();
    let mut inputs = egs_logic::GearboxPhysicalInputs::default();
    let mut outputs = egs_logic::GearboxOutputs::default();
    let mut slave_req = SlaveReq::default();
    let mut slave_res = SlaveStatus::default();
    gearbox.set_dev_mode(DeviceMode::SLAVE);
    loop {
        let now = Mono::now();
        let now_ms = now.duration_since_epoch().to_millis() as u32;

        // Or EGS device mode with one that is global
        let extern_dev_mode = cx.shared.device_mode.load(core::sync::atomic::Ordering::Relaxed);
        gearbox.set_dev_mode(DeviceMode::from_bits_truncate(extern_dev_mode));

        // Process signals
        if !gearbox.dev_mode().contains(DeviceMode::SLAVE) {
            cx.shared.can_layer.lock(|can| {
                can.read_signals(now_ms, &mut inputs.can_vars);
            })
        }

        gearbox.update(&inputs, &mut outputs).await;


        // Set the Device mode based on EGS layer
        cx.shared.device_mode.store(gearbox.dev_mode().bits(), core::sync::atomic::Ordering::Relaxed);


        // Write to CAN logic
        {
            let mut can_tx = can_tx.access().await;

            if gearbox.dev_mode().contains(DeviceMode::SLAVE) {
                // Slave logic
                cx.shared.slave_can.lock(|lck| {
                    // Reduce locking by reading here, data will be 
                    // applied in the next EGS cycle
                    lck.read_signals(now_ms, &mut slave_req);
                    lck.write_signals(&slave_res);
                    let _ = lck.transmit(|id, data| {
                        let msg = MessageBuilder {
                            id: Id::Standard(id),
                            frame_type: mcan::message::tx::FrameType::Classic(mcan::message::tx::ClassicFrameType::Data(data)),
                            store_tx_event: None
                        }.build().unwrap();
                        can_tx.transmit_queued(msg)
                    });
                });
            } else {
                // EGS logic
                cx.shared.can_layer.lock(|can_layer| {
                    can_layer.write_signals(&outputs.can_outputs);
                    let _ = can_layer.transmit(|id, data| {
                        let msg = MessageBuilder {
                            id: Id::Standard(id),
                            frame_type: mcan::message::tx::FrameType::Classic(mcan::message::tx::ClassicFrameType::Data(data)),
                            store_tx_event: None
                        }.build().unwrap();
                        can_tx.transmit_queued(msg)
                    });
                });
            }
        }
        // Write to solenoids
        //solenoid_controller.set_tcc_pwm(duty, sol_tcc);

        Mono::delay_until(now + 20u64.millis()).await;
    }
}