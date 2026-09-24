use atsamd_hal::{fugit::ExtU64, rtic_time::Monotonic};
use rtic::Mutex;

use crate::{Mono, app};

const INTERVAL_POLL: u64 = 10;

pub async fn sensor_query(mut cx: app::sensor_query::Context<'_>) {
    loop {
        let now = Mono::now();

        let data = cx.local.adc_data.update().await;

        if data.vkl15 < 7000 {
            // Less than 7V, disable outputs
        }

        cx.shared.sensor_data.lock(|l| {
            **l = data;
            //**l.speed_sensor
        });
        Mono::delay_until(now + INTERVAL_POLL.millis()).await;
    }
}
