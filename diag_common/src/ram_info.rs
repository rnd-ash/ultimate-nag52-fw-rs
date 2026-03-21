use crate::dyn_panic::AppPanicInfo;

pub const MAX_RESET_COUNT: u8 = 5;

#[derive(Clone, Copy)]
#[repr(C, packed(4))]
struct BootloaderCommInfo {
    crc: u32,
    state: BootloaderRamInfo,
}

#[derive(Clone, Copy, Default)]
#[repr(C, packed(4))]
pub struct BootloaderRamInfo {
    /// ## Bootloader -> Application
    /// If this counter goes above [`MAX_RESET_COUNT`]
    /// it will trigger the bootloader to not start the app
    /// essentially an emergency recovery mode. User can trigger
    /// this by quick pressing the reset button 5 times rapidly
    pub reset_counter: u8,
    /// ## Application -> Bootloader
    /// Diagnostic session
    /// has requested the bootloader stays active
    /// for an application update
    /// (stay_in_diag, (stmin override, bs override))
    pub diag_request_bootloader: (bool, Option<(u8, u8)>),
    /// Panic information from the app if the app panicked
    pub app_panic: Option<AppPanicInfo>,
    /// Ram failure error (Address, bit#, test stage)
    pub ram_failure: Option<(u32, u8, u8)>
}

// Ensure we don't overflow in RAM
static_assertions::const_assert!(core::mem::size_of::<BootloaderCommInfo>() < 512);

const BOOTLOADER_COMM_ADDR: *mut BootloaderCommInfo = 0x20010000 as *mut BootloaderCommInfo;

// Returns bootloader Comm Info, only if CRC is valid
pub fn get_bootloader_comm_info() -> Option<BootloaderRamInfo> {
    let bl_ram: &BootloaderCommInfo = unsafe { BOOTLOADER_COMM_ADDR.as_ref().unwrap() };
    let crc = crc_of_bootloader_state(&bl_ram.state);
    if crc == bl_ram.crc {
        Some(bl_ram.state)
    } else {
        None
    }
}

pub fn create_default_comm_info() {
    let pre = unsafe { BOOTLOADER_COMM_ADDR.as_mut().unwrap() };
    pre.state = Default::default();
    let crc = crc_of_bootloader_state(&pre.state);
    pre.crc = crc;
}

pub fn modify_bootloader_info<F: FnOnce(&mut BootloaderRamInfo)>(f: F) {
    let pre = unsafe { BOOTLOADER_COMM_ADDR.as_mut().unwrap() };
    f(&mut pre.state);
    let crc = crc_of_bootloader_state(&pre.state);
    pre.crc = crc;
}

fn crc_of_bootloader_state(state: &BootloaderRamInfo) -> u32 {
    unsafe {
        let as_bytes = core::slice::from_raw_parts(
            (state as *const BootloaderRamInfo) as *const u8,
            core::mem::size_of::<BootloaderRamInfo>(),
        );
        embedded_crc32c::crc32c(as_bytes)
    }
}
