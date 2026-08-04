use embedded_can::{StandardId};

pub use crate::can_matrix::slave_mode::*;
use crate::{CanError, CanLayer, CanResult};

crate::make_rx_frames!(SlaveRx {
    SolenoidControl
});

#[derive(Copy, Clone)]
pub struct SlaveCan {
    sensor_rpt: SensorReport,
    solenoid_rpt: SolenoidReport,
    rx_frames: SlaveRx,
}

impl Default for SlaveCan {
    fn default() -> Self {
        Self {
            sensor_rpt: SensorReport::ZERO,
            solenoid_rpt: SolenoidReport::ZERO,
            rx_frames: SlaveRx::default(),
        }
    }
}

#[derive(Clone, Copy)]
pub struct SlaveReq(pub CanResult<SolenoidControl>);

impl Default for SlaveReq {
    fn default() -> Self {
        Self(Err(CanError::MissingMsg))
    }
}

#[derive(Clone, Copy)]
pub struct SlaveStatus {
    pub sensors: SensorReport,
    pub solenoids: SolenoidReport,
}

impl Default for SlaveStatus {
    fn default() -> Self {
        Self {
            sensors: SensorReport::ZERO,
            solenoids: SolenoidReport::ZERO,
        }
    }
}

impl CanLayer<SlaveStatus, SlaveReq> for SlaveCan {
    fn filters(&self) -> &[StandardId] {
        SlaveRx::filters()
    }

    fn read_signals(&self, now_ms: u32, dest: &mut SlaveReq) {
        dest.0 = self.rx_frames.solenoid_control.get(100, now_ms)
    }

    fn write_signals(&mut self, now_ms: u32, sigs: &SlaveStatus) {
        self.sensor_rpt = sigs.sensors;
        self.solenoid_rpt = sigs.solenoids;
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
        tx_frame!(SolenoidReport, solenoid_rpt);
        tx_frame!(SensorReport, sensor_rpt);
        Ok(())
    }
}
