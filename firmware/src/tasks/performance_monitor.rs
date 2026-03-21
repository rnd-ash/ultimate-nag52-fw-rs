use core::sync::atomic::Ordering;

use atsamd_hal::{fugit::ExtU64, rtic_time::Monotonic};
use cortex_m::prelude::_embedded_hal_watchdog_Watchdog;
use rtic::Mutex;

use crate::{Mono, app};

pub async fn performance_monitor(mut ctx: app::perf_monitor::Context<'_>, tps: u32) {
    const SECOND_MILLIS: u32 = 1000;
    const UPDATES_PER_SEC: u32 = 4;
    let max_ticks: u32 = tps / UPDATES_PER_SEC;
    loop {
        let now = Mono::now();
        // Reset
        let asleep_ticks = ctx.shared.cpu_idle_ticks.swap(0, Ordering::Relaxed);
        let interrupts_per_sec = ctx.shared.hw_interrupts.swap(0, Ordering::Relaxed)*UPDATES_PER_SEC;
        let percentage: f32 =
            ((max_ticks.saturating_sub(asleep_ticks)) * 100) as f32 / max_ticks as f32;
        defmt::info!("CPU: {:02}% - HW Interrupts: {}/sec", percentage, interrupts_per_sec);
        ctx.shared.wdt.lock(|wdt| wdt.feed());
        Mono::delay_until(now + ((SECOND_MILLIS / UPDATES_PER_SEC) as u64).millis()).await;
    }
}