// Tle messages can be one of the following 3 formats:
//
// 1. Global message (No channel specified)
// 2. Message for channels 0-3, and Message for channels 4-7
// 3. Message per channel

use arbitrary_int::{u11, u12, u14, u18, u19, u2, u20, u4, u5, u6};

const CMD_MATCH_MASK: u32 = 0b0111_1111_0000_0000_0000_0000_0000_0000;
const WRITE_MATCH_MASK: u32 = 0b1000_0000_0000_0000_0000_0000_0000_0000;

pub trait TleMsg: Into<u32> + From<u32> + Copy {
    fn id_match(&self, other: u32) -> bool;
    fn is_read(&self) -> bool;
}

macro_rules! tle_message {
    ($msg:ident, $msg_id:literal) => {
        impl $msg {
            /// Creates a new message with the correct Message ID
            pub const fn new_with_id() -> Self {
                Self::new_with_raw_value((($msg_id as u32) << 24) & CMD_MATCH_MASK)
            }
        }

        impl TleMsg for $msg {
            fn id_match(&self, other: u32) -> bool {
                self.raw_value() & CMD_MATCH_MASK == other & CMD_MATCH_MASK
            }

            fn is_read(&self) -> bool {
                self.raw_value() & WRITE_MATCH_MASK == 0
            }
        }

        impl Into<u32> for $msg {
            fn into(self) -> u32 {
                self.raw_value()
            }
        }

        impl From<u32> for $msg {
            fn from(val: u32) -> Self {
                Self::new_with_raw_value(val)
            }
        }
    };
    ($msg:ident, $msg_id_03:literal, $msg_id_47:literal) => {
        impl $msg {
            /// Creates a new message with a message ID for channels 0 to 3
            pub const fn new_for_channels_0_3() -> Self {
                Self::new_with_raw_value((($msg_id_03 as u32) << 24) & CMD_MATCH_MASK)
            }

            /// Creates a new message with a message ID for channels 4 to 7
            pub const fn new_for_channels_4_7() -> Self {
                Self::new_with_raw_value((($msg_id_47 as u32) << 24) & CMD_MATCH_MASK)
            }
        }

        impl TleMsg for $msg {
            fn id_match(&self, other: u32) -> bool {
                self.raw_value() & CMD_MATCH_MASK == other & CMD_MATCH_MASK
            }

            fn is_read(&self) -> bool {
                self.raw_value() & WRITE_MATCH_MASK == 0
            }
        }

        impl Into<u32> for $msg {
            fn into(self) -> u32 {
                self.raw_value()
            }
        }

        impl From<u32> for $msg {
            fn from(val: u32) -> Self {
                Self::new_with_raw_value(val)
            }
        }
    };
}

/// TLE Channel number
#[bitbybit::bitenum(u3, exhaustive = true)]
#[derive(PartialEq, Eq, PartialOrd, Ord)]
pub enum TleChannel {
    /// Channel 0 (OUT0 - Pin 60)
    _0 = 0,
    /// Channel 1 (OUT1 - Pin 59)
    _1 = 1,
    /// Channel 2 (OUT2 - Pin 55)
    _2 = 2,
    /// Channel 3 (OUT3 - Pin 54)
    _3 = 3,
    /// Channel 4 (OUT4 - Pin 21)
    _4 = 4,
    /// Channel 5 (OUT5 - Pin 22)
    _5 = 5,
    /// Channel 6 (OUT6 - Pin 26)
    _6 = 6,
    /// Channel 7 (OUT7 - Pin 27)
    _7 = 7,
}

/// Channel control mode
///
/// See [CtrlMethodFaultMaskCfg]
#[bitbybit::bitenum(u1, exhaustive = true)]
pub enum ControlMode {
    /// Current control mode
    CurrentControl = 0,
    /// Direct PWM mode
    Pwm = 1,
}

/// Diagnostic timer configuration
///
/// See []
#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(Default)]
pub enum DiagnosticTimer {
    /// Pre divider of 128, nFault of 10..11
    #[default]
    Div128nFault10_11 = 0b00,
    /// Pre divider of 192, nFault of 10..11
    Div192nFault10_11 = 0b01,
    /// Pre divider of 128, nFault of 2..3
    Div128nFault02_03 = 0b10,
    /// Pre divider of 256, nFault of 10..11
    Div256nFault10_11 = 0b11,
}

/// Short to vBatt threshold value
///
/// See []
#[bitbybit::bitenum(u2, exhaustive = true)]
pub enum ShortToBatThreshold {
    /// 0.7V
    _0_7V = 0b00,
    /// 1.9V
    _0_9V = 0b01,
    /// 1.1V
    _1_1V = 0b10,
    /// 1.3V
    _1_3V = 0b11,
}

#[bitbybit::bitenum(u2, exhaustive = true)]
#[derive(defmt::Format)]
pub enum DividerM {
    /// 32 A/D samples per PWM Period
    _32 = 0b00,
    /// 64 A/D samples per PWM Period
    _64 = 0b01,
    /// 128 A/D samples per PWM period
    _128 = 0b10,
    /// 512 A/D samples per PWM period
    /// in direct PWM mode, or 128 in current control mode
    _512Or128 = 0b11,
}

impl DividerM {
    pub const fn get_value(&self, is_cc_mode: bool) -> u16 {
        match self {
            DividerM::_32 => 32,
            DividerM::_64 => 64,
            DividerM::_128 => 128,
            DividerM::_512Or128 => {
                if is_cc_mode {
                    128
                } else {
                    512
                }
            }
        }
    }
}

#[bitbybit::bitenum(u1, exhaustive = true)]
pub enum SummationMethod {
    /// Use all values of samples
    UseAllSamples = 0,
    /// Throw out first ADC samples after an OUTx pin transition
    /// and use previous samples twice in the average calculation
    ThrowOut = 1,
}

/// IC Version
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct IcVersion {
    #[bit(31)]
    write: bool,
    #[bits(24..=30)]
    msg_id: u7,
    /// IC Manufacturer (MUST BE 0b1100_0001 for Infineon)
    #[bits(16..=23, r)]
    ic_manf_id: u8,
    /// IC Version number
    #[bits(8..=15, r)]
    version_number: u8,
}

tle_message!(IcVersion, 0b000_0000);

/// Control Method and Fault Mask Configuration
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct CtrlMethodFaultMaskCfg {
    /// Write rather than read
    #[bit(31, rw)]
    write: bool,
    /// Message ID
    #[bits(24..=30)]
    msg_id: u7,
    /// Control mode for Channel x
    #[bit(16, rw)]
    cmx: [ControlMode; 8],
    /// Fault on channel x triggers the FAULT pin
    #[bit(8, rw)]
    fmx: [bool; 8],
    /// Fault mask enable for RESET_B pin
    #[bit(7, rw)]
    fmr: bool,
    /// Fault mask enable for ENABLE pin
    #[bit(6, rw)]
    fme: bool,
    /// Diagnostic timer
    #[bits(4..=5, rw)]
    diag_timer: DiagnosticTimer,
}

tle_message!(CtrlMethodFaultMaskCfg, 0b000_0001);

// Diagnostic configuration
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct DiagnosticConfiguration {
    /// Write rather than read
    #[bit(31, rw)]
    write: bool,
    /// Message ID
    #[bits(24..=30)]
    msg_id: u7,
    /// Short to battery Threshold
    #[bits(4..=5, stride = 6, rw)]
    sb: [ShortToBatThreshold; 4],
    /// Short to battery retry time
    ///
    /// Retry after 16*x periods
    ///
    /// Retry period = (16*[sb_retry])/f_pwm
    #[bits(0..=3, stride = 6, rw)]
    sb_retry: [u4; 4],
}

tle_message!(DiagnosticConfiguration, 0b000_0010, 0b000_0011);

// Channels 0-3 or 4-7  (SPI msg 4/5)
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct DiagnosticRead {
    /// Write rather than read
    #[bit(31)]
    write: bool,
    /// Message ID
    #[bits(24..=30)]
    msg_id: u7,
    /// Short to Ground - Fault
    #[bit(5, stride = 6, r)]
    sg: [bool; 4],
    /// Short to Ground & Open Load (Gate Off) - Tested
    #[bit(4, stride = 6, r)]
    off_tst: [bool; 4],
    /// Short to Battery - Fault
    #[bit(3, stride = 6, r)]
    sb: [bool; 4],
    /// Short to Battery - Tested
    #[bit(2, stride = 6, r)]
    sb_tst: [bool; 4],
    /// Open Load (Gate Off)
    #[bit(1, stride = 6, r)]
    ol_off: [bool; 4],
    /// Open Load (Gate On)
    #[bit(0, stride = 6, r)]
    ol_on: [bool; 4],
}

tle_message!(DiagnosticRead, 0b000_0100, 0b000_0101);

// Channels 0-3 or 4-7  (SPI msg 6/7)
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct PwmOffset {
    /// Write rather than read
    #[bit(31, rw)]
    write: bool,
    /// Message ID
    #[bits(24..=30)]
    msg_id: u7,
    /// ChannelX Pulse Offset
    /// 1/32 of PWM period set by N and M values
    ///
    /// Note: After exiting reset, a pulse on the PHASE_SYNC pin is
    ///       needed to synchronize the channels
    #[bits(0..=4, rw)]
    offset: [u5; 4],
}

tle_message!(PwmOffset, 0b000_0110, 0b000_0111);

// Per channel (SPI msg 8)
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct MainPeriodSet {
    /// Write rather than read
    #[bit(31, rw)]
    write: bool,
    /// Message ID
    #[bits(27..=30)]
    msg_id: u4,
    /// Channel ID
    #[bits(24..=26, rw)]
    channel_id: TleChannel,
    /// Sample summation method
    #[bit(16, rw)]
    sam: SummationMethod,
    /// Divider M (Number of A/D samples per PWM Period)
    #[bits(14..=15, rw)]
    divider_m: DividerM,
    /// Divider N (Number of main CLK Periods between A/D samnples)
    ///
    /// `T_pwm  = N*M*T_clk`  & `T_adc = N*T_clk`
    #[bits(0..=13, rw)]
    divider_n: u14,
}

tle_message!(MainPeriodSet, 0b000_1000);

// Per channel (SPI msg 9)
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct ControlVarsSet {
    /// Write rather than read
    #[bit(31, rw)]
    write: bool,
    /// Message ID
    #[bits(27..=30)]
    msg_id: u4,
    /// Channel ID
    #[bits(24..=26, rw)]
    channel_id: TleChannel,
    /// Control loop proportional coefficient
    #[bits(12..=23, rw)]
    kp: u12,
    /// Control loop integral coefficient
    #[bits(0..=11, rw)]
    ki: u12,
}

tle_message!(ControlVarsSet, 0b001_0000);

// Per channel (SPI msg 10)
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct CurrentDitherAmpSet {
    /// Write rather than read
    #[bit(31, rw)]
    write: bool,
    /// Message ID
    #[bits(27..=30)]
    msg_id: u4,
    /// Channel ID
    #[bits(24..=26, rw)]
    channel_id: TleChannel,
    /// Operation during ENABLE deactivation
    #[bit(23, rw)]
    en: bool,
    /// Dither Step Size (LSb's value is 2^-2 of the current setpoint LSb)
    ///
    /// `Dither Amplitude [mA pp] = ((2*[dither_step_size]*dither steps)/2^13) / (320mV/rSense(Ohm))`
    #[bits(11..=21, rw)]
    dither_step_size: u11,
    /// Current set point
    ///
    /// `Setpoint [mA] = ([current_setpoint]/2^11) * (320mV/rSense(Ohm))`
    #[bits(0..=10, rw)]
    current_setpoint: u11,
}

tle_message!(CurrentDitherAmpSet, 0b001_1000);

// Per channel (SPI msg 11)
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct DitherPeriodSet {
    /// Write rather than read
    #[bit(31, rw)]
    write: bool,
    /// Message ID
    #[bits(27..=30)]
    msg_id: u4,
    /// Channel ID
    #[bits(24..=26, rw)]
    channel_id: TleChannel,
    /// Number of dither steps in 1/4 waveform (0 disables the dither function)
    ///
    /// `Dither period = (4*[number_of_steps]) / f_pwm`
    #[bits(0..=4, rw)]
    number_of_steps: u5,
}

tle_message!(DitherPeriodSet, 0b010_0000);

// Per channel (SPI msg 12)
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct MaxMinCurrentRead {
    /// Write rather than read
    #[bit(31)]
    write: bool,
    /// Message ID
    #[bits(27..=30)]
    msg_id: u4,
    /// Channel ID
    #[bits(24..=26, rw)]
    channel_id: TleChannel,
    /// Set when new data is available
    #[bit(22, r)]
    valid: bool,
    /// The largest summation of 'M' A/D samples within one PWM  period
    /// during the pervious dither cycle
    ///
    /// `Max current feedback [mA] = (max/2^11) * (320mV/Rsense[Ohm])`
    #[bits(11..=21, r)]
    max: u11,
    /// The smallest summation of 'M' A/D samples within one PWM  period
    /// during the pervious dither cycle
    ///
    /// `Min current feedback [mA] = (min/2^11) * (320mV/Rsense[Ohm])`
    #[bits(0..=10, r)]
    min: u11,
}

tle_message!(MaxMinCurrentRead, 0b010_1000);

// Per channel (SPI msg 13)
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct AverageCurrentRead {
    /// Write rather than read
    #[bit(31)]
    write: bool,
    /// Message ID
    #[bits(27..=30)]
    msg_id: u4,
    /// Channel ID
    #[bits(24..=26, rw)]
    channel_id: TleChannel,
    /// Set when new data is available
    #[bit(20, r)]
    valid: bool,
    /// 20 bit summation of the total current over a dither period
    ///
    /// Consult the datasheet for more information
    #[bits(0..=19, r)]
    avg: u20,
}

tle_message!(AverageCurrentRead, 0b011_0000);

// Per channel (SPI msg 14)
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct AutoZeroTriggerRead {
    /// Write rather than read
    #[bit(31, rw)]
    write: bool,
    /// Message ID
    #[bits(27..=30)]
    msg_id: u4,
    /// Channel ID
    #[bits(24..=26, rw)]
    channel_id: TleChannel,
    /// AutoZero - Gate on has occured since the last read
    #[bit(17, r)]
    az_on: bool,
    /// AutoZero - Gate off has occured since the last read
    #[bit(16, r)]
    az_off: bool,
    /// AutoZero value - Gate on
    #[bits(8..=15, r)]
    az_on_val: u8,
    /// AutoZero value - Gate off
    #[bits(0..=7, r)]
    az_off_val: u8,
}

tle_message!(AutoZeroTriggerRead, 0b011_1000);

// Per channel (SPI msg 15)
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct PwmDutyCycle {
    /// Write rather than read
    #[bit(31, rw)]
    write: bool,
    /// Message ID
    #[bits(27..=30)]
    msg_id: u4,
    /// Channel ID
    #[bits(24..=26, rw)]
    channel_id: TleChannel,
    /// PWM Duty cycle
    #[bits(0..=18, rw)]
    pwm: u19,
}

tle_message!(PwmDutyCycle, 0b100_0000);

// Per channel (SPI msg 16)
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct CurrentProfileSetup1 {
    /// Write rather than read
    #[bit(31, rw)]
    write: bool,
    /// Message ID
    #[bits(27..=30)]
    msg_id: u4,
    /// Channel ID
    #[bits(24..=26, rw)]
    channel_id: TleChannel,
    /// Threshold Zone n
    #[bits(12..=15, rw)]
    threshold: [u4; 3],
    /// Count Zone n
    #[bits(0..=3, rw)]
    count: [u4; 3],
}

tle_message!(CurrentProfileSetup1, 0b100_1000);

// Per channel (SPI msg 17)
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct CurrentProfileSetup2 {
    /// Write rather than read
    #[bit(31, rw)]
    write: bool,
    /// Message ID
    #[bits(27..=30)]
    msg_id: u4,
    /// Channel ID
    #[bits(24..=26, rw)]
    channel_id: TleChannel,
    /// Current profile time out in steps of 16 ADC sample periods
    #[bits(4..=9, rw)]
    timeout: u6,
    /// Zone 3 A/D setup (Consult the datasheet)
    #[bits(0..=1, rw)]
    zone_3_set: u2,
}

tle_message!(CurrentProfileSetup2, 0b101_0000);

// Per channel (SPI msg 18)
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct CurrentProfileDetectionFeedback {
    /// Write rather than read
    #[bit(31)]
    write: bool,
    /// Message ID
    #[bits(27..=30)]
    msg_id: u4,
    /// Channel ID
    #[bits(24..=26, rw)]
    channel_id: TleChannel,
    /// Detect interrupt bit
    #[bit(2, r)]
    detection_interrupted: bool,
    /// Current profile timeout
    #[bit(1, r)]
    timeout: bool,
    /// Passed since last read
    #[bit(0, r)]
    pass: bool,
}

tle_message!(CurrentProfileDetectionFeedback, 0b101_1000);

// Per channel (SPI msg 19)
#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct ReadGenericFlags {
    /// Write rather than read
    #[bit(31)]
    write: bool,
    /// Message ID
    #[bits(24..=30)]
    msg_id: u7,
    /// Overvoltage has occured since last read
    #[bit(3, r)]
    ov: bool,
    /// Phase sync has occured since last read
    #[bit(2, r)]
    ps: bool,
    /// Enable latch bit
    #[bit(1, r)]
    en_l: bool,
    /// RESET_B latch bit
    #[bit(0, r)]
    rb_l: bool,
}

tle_message!(ReadGenericFlags, 0b111_1000);
