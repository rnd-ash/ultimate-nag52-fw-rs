use core::sync::atomic::Ordering;

use atsamd_hal::{fugit::ExtU64, rtic_time::Monotonic};
use rtic::Mutex;

use crate::{Mono, app};

pub async fn performance_monitor(mut ctx: app::perf_monitor::Context<'_>, tps: u32) {
    const SECOND_MILLIS: u32 = 1000;
    const UPDATES_PER_SEC: u32 = 10;
    let max_ticks: u32 = tps / UPDATES_PER_SEC;
    loop {
        let now = Mono::now();
        // Reset
        let asleep_ticks = ctx.shared.cpu_idle_ticks.swap(0, Ordering::Relaxed);
        let interrupts_per_sec =
            ctx.shared.hw_interrupts.swap(0, Ordering::Relaxed) * UPDATES_PER_SEC;
        let wakeups_per_sec = ctx.shared.wakeups.swap(0, Ordering::Relaxed) * UPDATES_PER_SEC;
        let percentage: f32 =
            ((max_ticks.saturating_sub(asleep_ticks)) * 100) as f32 / max_ticks as f32;
        //println!("Sleep count: {}", asleep_ticks);
        defmt::info!(
            "CPU: {:02}% - HW Interrupts: {}/sec, Wakeups: {}/sec",
            percentage,
            interrupts_per_sec,
            wakeups_per_sec
        );
        ctx.shared.perf_stats.lock(|stats| {
            stats.cpu_percentage = (percentage * 10.0) as u16;
            stats.hw_interrupts = interrupts_per_sec as u16;
            stats.wakeups = wakeups_per_sec as u16;
        });
        Mono::delay_until(now + ((SECOND_MILLIS / UPDATES_PER_SEC) as u64).millis()).await;
    }
}
