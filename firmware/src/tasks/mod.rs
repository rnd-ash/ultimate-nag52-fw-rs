mod init;
mod diag;
mod gearbox;
mod performance_monitor;
mod sensors;

pub use init::*;
pub use diag::*;
pub use gearbox::*;
pub use performance_monitor::*;
pub use sensors::*;

use core::{ptr::addr_of, sync::atomic::Ordering};

use crate::{app, ram_test};
use atsamd_hal::pac::{DWT, SCB};
use cortex_m::asm::wfi;
use diag_common::{hal_extensions::dsu::{self, MemoryTestResult}, ram_info::modify_bootloader_info};



pub fn idle(ctx: &app::idle::Context) -> ! {
    
    let (mut dwt, mut dcb) = unsafe {
        let p = cortex_m::Peripherals::steal();
        (p.DWT, p.DCB)
    };
    dcb.enable_trace();
    dwt.enable_cycle_counter();
    // Size of each RAM test
    let ram_test_size: usize = ram_test::ram_buf_size();
    let ram_test_buf_addr: *mut u8 = ram_test::ram_buf_ptr();
    let ram_start = ram_test::ram_start();

    // Max address in RAM to test (Ensuring it doesn't
    // overwrite the idle task stack)
    let mut ram_test_end: u32 = 0x2000_0000 + (256*1024);
    let mut ram_offset = 0;

    #[inline(always)]
    fn set_stack_watermark(watermark: &mut u32) {
        let dummy = 0;
        let addr = addr_of!(dummy).addr() as u32;
        // since stack decreases
        if addr < *watermark {
            // Round down to nearest 4KB
            *watermark = (addr/4096)*4096;
        }
    }
    // Pesimistic view of stack watermark,
    // so we don't write into the stack
    set_stack_watermark(&mut ram_test_end);
    let mut ram_test_done = false;
    loop {
        // Stop interrupts from context switch
        let count = cortex_m::interrupt::free(|_| {
            // Wait for something to wake up the CPU
            if !ram_test_done && let Some(mut lock) = ctx.shared.dsu.try_access() {
                unsafe {
                    // Copy RAM to temp buffer
                    let ram_ptr = (ram_start+ram_offset) as *mut u8;
                    ram_ptr.copy_to(ram_test_buf_addr, ram_test_size);
                    // Guaranteed alignment
                    let test = lock.polling_memory_test(ram_ptr.addr() as u32, ram_test_size as u32).unwrap();
                    dwt.set_cycle_count(0);
                    wfi();
                    let dwt_count = DWT::cycle_count();
                    let test_res = test.finish_now();
                    // Copy ram back
                    ram_test_buf_addr.copy_to(ram_ptr, ram_test_size);
                    match test_res {
                        MemoryTestResult::Ok=> {
                            ram_offset += ram_test_size as u32;
                            if (ram_start+ram_offset) > ram_test_end {
                                ram_test_done = true;
                                ram_offset = 0;
                            }
                        }
                        MemoryTestResult::Aborted => {
                            // Aborted, so don't increase counter
                        },
                        MemoryTestResult::Error(error) => {
                            if let dsu::Error::RamTestFailed { addr, phase, bit } = error {
                                modify_bootloader_info(|info| {
                                    info.ram_failure = Some((addr, bit, phase));
                                });
                                SCB::sys_reset();
                            }
                        },
                    }
                    dwt_count
                }
            } else {
                dwt.set_cycle_count(0);
                wfi();
                DWT::cycle_count()
            }
        });
        // Write down CPU usage after interrupt performed (Lowers latency to interrupt)
        ctx.shared
            .cpu_idle_ticks
            .fetch_add(count, Ordering::Relaxed);

        let nvic = unsafe {
            cortex_m::Peripherals::steal().NVIC
        };

        let pending: u32 = nvic.ispr
            .iter()
            .map(|x| x.read().count_ones())
            .sum();

        ctx.shared.hw_interrupts.fetch_add(pending, Ordering::Relaxed);
    }
}