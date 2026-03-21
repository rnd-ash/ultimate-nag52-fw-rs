use atsamd_hal::{fugit::ExtU64, rtic_time::Monotonic};
use rtic::Mutex;

use crate::{Mono, app};

pub async fn sensor_query(
    mut cx: app::sensor_query::Context<'_>
) {
    loop {
        let now = Mono::now();
        let _speed_sensors = cx.local.speed_sensors.update();
        let data = cx.local.adc_data.update().await;
        cx.shared.sensor_data.lock(|l| *l = data);
        Mono::delay_until(now + 10u64.millis()).await;
    }
}