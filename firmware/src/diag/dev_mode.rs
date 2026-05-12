//! Device mode
//!
//! This is compatible with the EGS device mode flags

use bitflags::bitflags;

bitflags! {
    pub struct EgsDeviceMode: u16 {
        /// Normal operation of the ECU
        const NORMAL = 0x0001;
        /// Self-Test of the ECU (via diagnostics)
        const MONTAGE = 0x0002;
        /// Special drive cycle simulation (TBA)
        const ROLLER = 0x0004;
        /// Normal control of the solenoids and CAN is disabled
        /// and the device operates in a special 'slave' mode
        /// where the solenoids can be manipulated directly over
        /// CAN (see [crate::can::data:slave_mode])
        const SLAVE = 0x0008;
        /// Outputs are disabled. This
        /// mode can be disabled automatically if
        /// conditions are correct
        const TEMP_EMERGENCY = 0x0010;
        /// Hardware error, everything is disabled, including
        /// normal transmission on CAN (Diagnostics can still
        /// function)
        const HARDWARE_ERROR = 0x0020;
        /// Outputs are disabled. This
        /// mode can be only be disabled via diagnostics
        const PERM_EMERGENCY = 0x0040;
        /// Outputs are disabled. This mode only occurs
        /// when voltage falls below 9V
        const UNDERVOLTAGE_EMERGENCY = 0x0080;
        /// Device is initializing. Cleared after initializing
        const INITIALIZATION = 0x0100;
        /// End of line testing (TBA)
        const TEST = 0x0200;
        /// Unknown fully what this does
        const WEP = 0x0400;
    }
}

impl EgsDeviceMode {
    /// If this function returns false, the solenoid
    /// high side power is turned off, shutting off
    /// power to all the solenoids.
    pub fn solenoid_output_allowed(&self) -> bool {
        let banned_modes = Self::TEMP_EMERGENCY
            | Self::HARDWARE_ERROR
            | Self::PERM_EMERGENCY
            | Self::UNDERVOLTAGE_EMERGENCY;
        !self.intersects(banned_modes)
    }
}
