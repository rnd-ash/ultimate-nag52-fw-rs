#![no_std]
#![no_main]

use core::{panic::PanicInfo};

use atsamd_hal::{
    clock::v2::{clock_system_at_reset, dpll::Dpll, gclk::{Gclk, GclkDiv16}, pclk::Pclk}, dsu::Dsu, ehal::digital::OutputPin, nvm::{Nvm, smart_eeprom::SmartEepromMode}, pac::Peripherals
};
use bootloader::bl_info::{
    CodeSectionInfo, MemoryRegion, mutate_smarteeprom_info, parse_git_sha, parse_u8,
    region_crc,
};
use cortex_m_rt::entry;
use diag_common::ram_info;

unsafe extern "C" {
    static mut _can_ram_addr: u8;
    static mut _can_ram_end_addr: u8;
}

pub fn can_ram_start() -> u32 {
    (&raw mut _can_ram_addr as *mut u8).addr() as u32
}

pub fn can_ram_end() -> u32 {
    (&raw mut _can_ram_end_addr as *mut u8).addr() as u32
}

pub const fn create_code_info(name: [u8; 10]) -> CodeSectionInfo {
    CodeSectionInfo {
        name,
        git_sha: parse_git_sha(env!("VERGEN_GIT_SHA")),
        version_major: parse_u8(env!("CARGO_PKG_VERSION_MAJOR")),
        version_minor: parse_u8(env!("CARGO_PKG_VERSION_MINOR")),
        version_patch: parse_u8(env!("CARGO_PKG_VERSION_PATCH")),
        compile_year: parse_u8(env!("BUILD_YEAR")),
        compile_month: parse_u8(env!("BUILD_MONTH")),
        compile_week: parse_u8(env!("BUILD_WEEK")),
        compile_day: parse_u8(env!("BUILD_DAY")),
        rustc_version_major: parse_u8(env!("RUSTC_VER_MAJOR")),
        rustc_version_minor: parse_u8(env!("RUSTC_VER_MINOR")),
        rustc_version_patch: parse_u8(env!("RUSTC_VER_PATCH")),
        #[cfg(debug_assertions)]
        is_debug: 1,
        #[cfg(not(debug_assertions))]
        is_debug: 0,
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    let port = unsafe { Peripherals::steal().port };
    let pins = bsp::Pins::new(port);
    pins.led_stat_ok.into_push_pull_output().set_low().unwrap();
    let mut err = pins.led_stat_err.into_push_pull_output();
    loop {
        err.set_high().unwrap();
        cortex_m::asm::delay(24_000_000);
        err.set_low().unwrap();
        cortex_m::asm::delay(24_000_000);
    }
}

#[entry]
fn main() -> ! {
    let mut bsp_peripherals = Peripherals::take().unwrap();
    let pins = bsp::Pins::new(bsp_peripherals.port);
    let mut stat = pins.led_stat_ok.into_push_pull_output();
    stat.set_high().unwrap();

    let (_bus, clocks, tokens) = clock_system_at_reset(bsp_peripherals.oscctrl, bsp_peripherals.osc32kctrl, bsp_peripherals.gclk, bsp_peripherals.mclk, &mut bsp_peripherals.nvmctrl);

    let mut nvm = Nvm::new(bsp_peripherals.nvmctrl);
    let mut dsu = Dsu::new(bsp_peripherals.dsu, &bsp_peripherals.pac).unwrap();
    // Ram test
    unsafe {
        // Clock to 120Mhz briefly so we can run RAM test as fast as possible
        let (gclk1, dfll) = Gclk::from_source(tokens.gclks.gclk1, clocks.dfll);
        let gclk1 = gclk1.div(GclkDiv16::Div(24)).enable(); // Gclk1 is now at 2Mhz
        let (clk_dpll0, _gclk1) = Pclk::enable(tokens.pclks.dpll0, gclk1);
        // DPLL0 at 120Mhz (2*60)
        let dpll0 = Dpll::from_pclk(tokens.dpll0, clk_dpll0)
            .loop_div(60, 0)
            .enable();
        let (gclk0_100, dfll, dpll0) = clocks.gclk0.swap_sources(dfll, dpll0);
        // Test MCAN RAM region (As we can't test it when MCAN is running)
        ram_info::modify_bootloader_info(|f| {
            if f.ram_failure.is_none() {
                let len = can_ram_end()-can_ram_start();
                if let Err(atsamd_hal::dsu::Error::RamTestFailed { addr, phase, bit }) = dsu.memory_test(can_ram_start(), len) {
                    f.ram_failure = Some((addr, bit, phase));
                }       
            }
        });

        // Revert clocks
        let (_gclk0_48, dpll0, _dfll) = gclk0_100.swap_sources(dpll0, dfll);
        let _disabled_dpll0 = dpll0.disable();
    }
    let bl_info = if let Ok(smart_eeprom) = nvm.smart_eeprom() {
        // Start smart EEPROM in locked mode
        let mut smart_eeprom = match smart_eeprom {
            SmartEepromMode::Locked(smart_eeprom) => smart_eeprom.unlock(),
            SmartEepromMode::Unlocked(smart_eeprom) => smart_eeprom,
        };
        mutate_smarteeprom_info(&mut smart_eeprom, |info| {
            const SECTION_INFO: CodeSectionInfo = create_code_info(*b"UN52PICPRE");
            if info.preloader_info != SECTION_INFO {
                info.preloader_info = SECTION_INFO
            }
        });

        // Check smart eeprom
        Some(bootloader::bl_info::get_smarteeprom_info(&smart_eeprom))
    } else {
        None
    };
    if let Some(info) = bl_info {
        if info.bl_flashing_pending == 0 {
            unsafe {
                // We have to copy the bootloader portions
                let scratch_crc =
                    region_crc(MemoryRegion::BootloaderScratch.range_exclusive(), &mut dsu);
                if scratch_crc == info.crc32_bl {
                    // Can copy (Sig. valid)
                    let bootloader_region = MemoryRegion::Bootloader;
                    let copy_region = MemoryRegion::BootloaderScratch;
                    // Erase bootloader
                    let _ = nvm.erase_flash(
                        bootloader_region.start_addr() as *mut u32,
                        bootloader_region.blocks_8k(),
                    );
                    //// Copy over the scratch area to the bootloader
                    let _ = nvm.write_flash(
                        bootloader_region.start_addr() as *mut u32,
                        copy_region.start_addr() as *const u32,
                        bootloader_region.range_exclusive().len() as u32 / 4,
                        atsamd_hal::nvm::WriteGranularity::Page,
                    );
                }
                let mut unlocked_eeprom = match nvm.smart_eeprom().unwrap() {
                    SmartEepromMode::Locked(smart_eeprom) => smart_eeprom.unlock(),
                    SmartEepromMode::Unlocked(smart_eeprom) => smart_eeprom,
                };
                // Set the flags so that we don't do this on next boot
                let _ =
                    bootloader::bl_info::mutate_smarteeprom_info(&mut unlocked_eeprom, |info| {
                        info.bl_flashing_pending = 0xFF;
                        info.crc32_bl =
                            region_crc(MemoryRegion::Bootloader.range_exclusive(), &mut dsu);
                    });
            }
        }
    }
    // Jump to bootloader
    stat.set_low().unwrap();
    unsafe {
        let core_peripehrals = cortex_m::Peripherals::steal();
        let bootloader_addr = MemoryRegion::Bootloader.start_addr();
        core_peripehrals.SCB.vtor.write(bootloader_addr);
        cortex_m::asm::bootload(bootloader_addr as *const u32)
    }
}
