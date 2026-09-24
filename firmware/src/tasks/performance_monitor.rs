use atsamd_hal::{fugit::ExtU64, rtic_time::Monotonic};
use cortex_m::prelude::_embedded_hal_watchdog_Watchdog;
use rtic::Mutex;

use crate::{Mono, app};

#[derive(Copy, Clone, Default)]
pub struct PerformanceInfo {
    pub cpu_idle_ticks: u32,
    pub hw_interrupts: u32,
    pub wakeups: u32,
}

pub async fn performance_monitor(mut ctx: app::perf_monitor::Context<'_>, cpu_ticks_per_sec: u32) {
    const SECOND_MILLIS: u32 = 1000;

    let mut last_mono = Mono::now().duration_since_epoch().to_millis();
    loop {
        let now = Mono::now();
        let delta_ms = now.duration_since_epoch().to_millis() - last_mono;
        if delta_ms != 0 {
            // < 1 second -> > 1.0
            let mul = SECOND_MILLIS as f32 / delta_ms as f32;
            // Reset
            let info = ctx.shared.perf_info.lock(|lck| {
                let old = *lck;
                *lck = Default::default();
                old
            });

            let asleep_ticks_per_sec = (info.cpu_idle_ticks as f32 * mul) as u32;
            let interrupts_per_sec = (info.hw_interrupts as f32 * mul) as u32;
            let wakeups_per_sec = (info.wakeups as f32 * mul) as u32;
            let percentage: f32 = ((cpu_ticks_per_sec.saturating_sub(asleep_ticks_per_sec)) * 100)
                as f32
                / cpu_ticks_per_sec as f32;
            ctx.shared.perf_stats.lock(|stats| {
                stats.cpu_percentage = (percentage * 10.0) as u16;
                stats.hw_interrupts = interrupts_per_sec as u16;
                stats.wakeups = wakeups_per_sec as u16;
            });
        }
        last_mono = now.duration_since_epoch().to_millis();
        // Poll watchdog
        ctx.local.wdt.feed();
        Mono::delay_until(now + ((250) as u64).millis()).await;
    }
}
