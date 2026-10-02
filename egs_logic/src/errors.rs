use bitflags::bitflags;

pub enum CanValueResult<T> {
    Ok(T),
    MissingMsg,
    InvalidMsgLen,
}

pub enum CanState {
    Ok,
    BusErr,
}

bitflags! {
    struct SolenoidErrors: u16 {
        const Y3_SHORT = 1;
        const Y4_SHORT = 1 << 1;
        const Y5_SHORT = 1 << 2;
        const Y3_OPEN = 1 << 3;
        const Y4_OPEN = 1 << 4;
        const Y5_OPEN = 1 << 5;

        const MPC_SHORT = 1 << 6;
        const SPC_SHORT = 1 << 7;
        const TCC_SHORT = 1 << 8;
        const MPC_OPEN = 1 << 9;
        const SPC_OPEN = 1 << 10;
        const TCC_OPEN = 1 << 11;
    }
}

bitflags! {
    /// Device mode, compatible with original EGS Siemens layer
    /// so that device mode shows up correctly in DAS.
    ///
    /// These definitions can be found in EGS5x CBF file
    #[derive(Copy, Clone)]
    pub struct DeviceMode: u16 {
        /// Device is running normally
        const NORMAL = 1;
        /// Montage mode active (Solenoid self-test)
        const MONTAGE = 1 << 1;
        /// Roller mode active (Test bench drive cycle -
        /// Never used by this code base)
        const ROLLER = 1 << 2;
        /// Slave mode active (Solenoid control on bench)
        const SLAVE = 1 << 3;
        /// Emergency mode active, which can be reset after reboot
        const TEMP_EMERGENCY = 1 << 4;
        /// Hardware internal error - Cannot be cleared
        const HARDWARE_ERR = 1 << 5;
        /// Emergency mode active, which can only be reset
        /// via DTC clearing
        const PERM_EMERGENCY = 1 << 6;
        /// Undervoltage detected (Will be cleared once voltage
        /// returns to normal)
        const UNDERVOLTAGE_EMERGENCY = 1 << 7;
        /// Device is initializing. This is cleared once
        /// EGS has booted up, and never
        /// set again during operation
        const INIT = 1 << 8;
        /// Some test mode, not 100% sure
        const WEP = 1 << 10;
        /// Solenoids are switched off only if allowed by
        /// gearbox software
        const CONDITIONAL_EMERGENCY = 1 << 11;
    }
}

impl DeviceMode {
    pub fn has_error(&self) -> bool {
        self.intersects(
            Self::TEMP_EMERGENCY
                | Self::PERM_EMERGENCY
                | Self::HARDWARE_ERR
                | Self::UNDERVOLTAGE_EMERGENCY
                | Self::CONDITIONAL_EMERGENCY,
        )
    }
}
