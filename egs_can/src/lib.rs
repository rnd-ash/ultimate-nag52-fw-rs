#![no_std]

mod can_matrix;
use arbitrary_int::traits::{BuiltinInteger, UnsignedInteger};
pub use can_matrix::*;
use embedded_can::StandardId;

use crate::egs52::Egs52Can;

mod bit_checking;
pub mod egs52;
pub mod slave;

/// Rx frame with timeout
///
/// ECU uses these to verify that the
/// CAN data is not stagnent
#[derive(Copy, Clone)]
pub struct RxFrame<T: SignalFrame> {
    /// Stored as Some(T) if message length
    /// is correct, otherwise
    frame: CanResult<T>,
    timestamp_ms: u32,
    seen: bool,
}

#[macro_export]
macro_rules! make_rx_frames {
    ($struct_name:ident { $($frame:ident),* }) => {
        pastey::paste! {

#[derive(Copy, Clone)]
pub struct $struct_name {
    $(
        pub [<$frame:snake>]: crate::RxFrame<$frame>,
    )*
}

impl Default for $struct_name {
    fn default() -> Self {
        Self {
            $(
                [<$frame:snake>]: crate::RxFrame::<$frame> {
                    frame: Err(CanError::MissingMsg),
                    timestamp_ms: 0,
                    seen: false,
                },
            )*
        }
    }
}

impl $struct_name {
    #[inline(always)]
    pub const fn filters() -> &'static [embedded_can::StandardId] {
        &[$(
            [<$frame>]::CAN_ID,
        )*]
    }

    #[inline(always)]
    pub fn on_rx_frame(&mut self, now_ms: u32, id: embedded_can::Id, dlc: u8, data: &[u8; 8]) {
        match id {
            $(
                embedded_can::Id::Standard([<$frame>]::CAN_ID) => {
                    self.[<$frame:snake>].write([<$frame>]::from_can_msg(dlc, data), now_ms)
                }
            )*
            _ => {}
        }
    }
}

        }
    }
}

impl<T: SignalFrame> RxFrame<T>
where
    T: Copy + Clone,
{
    /// Returns [None] if the frame has never been seen on the bus, or is stagnent
    /// otherwise, returns the CAN frame
    pub fn get(&self, max_ms: u32, now_ms: u32) -> CanResult<T> {
        if self.seen {
            if now_ms - self.timestamp_ms > max_ms {
                Err(CanError::MissingMsg)
            } else {
                self.frame
            }
        } else {
            Err(CanError::MissingMsg)
        }
    }

    /// Logs a new incomming frame
    pub fn write(&mut self, v: CanResult<T>, now_ms: u32) {
        self.seen = true;
        self.frame = v;
        self.timestamp_ms = now_ms
    }

    /// Returns true if the frame has been seen on the bus at some point
    /// in the past
    pub fn has_been_seen(&self) -> bool {
        self.seen
    }
}

#[derive(Default, Debug, Clone, Copy)]
pub enum CanError {
    Unsupported,
    #[default]
    MissingMsg,
    InvalidFrameLen,
    SignalInvalid,
}

// Logic so that Invalid CAN Enums get thrown as SignalInvalid errors.
// The type here is returned by bitbybit if an enum was not valid
impl<T: UnsignedInteger + BuiltinInteger, const N: usize> From<arbitrary_int::UInt<T, N>>
    for CanError
{
    fn from(_value: arbitrary_int::UInt<T, N>) -> Self {
        Self::SignalInvalid
    }
}

pub type CanResult<T> = core::result::Result<T, CanError>;

pub trait CanLayer<I, O> {
    fn filters(&self) -> &[StandardId] {
        &[]
    }
    fn on_frame(&mut self, now_ms: u32, id: embedded_can::Id, dlc: u8, data: &[u8; 8]);
    fn read_signals(&self, now_ms: u32, dest: &mut O);
    fn write_signals(&mut self, now_ms: u32, sigs: &I);
    fn transmit<E, F: FnMut(StandardId, &[u8]) -> nb::Result<(), E>>(
        &self,
        f: F,
    ) -> nb::Result<(), E>;
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum CanTargGear {
    N,
    _1,
    _2,
    _3,
    _4,
    _5,
    _6,
    _7,
    R,
    R2,
    P,
    Abort,
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum ShifterPosSimple {
    P,
    R,
    N,
    D,
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum TipTronicShifterPos {
    P,
    R,
    N,
    D,
    ND,
    NR,
    PR,
    Plus,
    Minus,
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum ShiftPaddlePos {
    None,
    Plus,
    Both,
    Minus,
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum CanActualGear {
    N,
    _1,
    _2,
    _3,
    _4,
    _5,
    _6,
    _7,
    R,
    R2,
    P,
    PowerFree,
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum DisplayGear {
    Blank,
    _1,
    _2,
    _3,
    _4,
    _5,
    _6,
    _7,
    A,
    F,
    N,
    P,
    R,
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum EcuReqGear {
    _1,
    _2,
    _3,
    _4,
    _5,
}

#[derive(Default, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum DisplayProfile {
    A,
    C,
    R,
    F,
    M,
    S,
    W,
    Underscore,
    #[default]
    Blank,
    Upshift,
    Downshift,
}

#[derive(Default, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum TccState {
    #[default]
    Open,
    OpenSlipping,
    SlippingOpen,
    Slipping,
    SlippingClosed,
    ClosedSlipping,
    Closed,
}

#[derive(Default, Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum AgileMode {
    Sports,
    Comfort,
    Unknown,
    #[default]
    Snv,
}

#[derive(Copy, Clone, Debug)]
pub struct TorqueRequest {
    pub amount_nm: f32,
    pub ramp_end: bool,
}

#[derive(Copy, Clone)]
pub enum CanLayerTy {
    Egs52(Egs52Can),
}

impl CanLayerTy {
    fn as_can_layer(&self) -> &impl CanLayer<CanTxData, CanRxData> {
        match self {
            CanLayerTy::Egs52(egs52_can) => egs52_can,
        }
    }

    fn as_can_layer_mut(&mut self) -> &mut impl CanLayer<CanTxData, CanRxData> {
        match self {
            CanLayerTy::Egs52(egs52_can) => egs52_can,
        }
    }

    pub fn filters(&self) -> &[StandardId] {
        self.as_can_layer().filters()
    }

    pub fn on_frame(&mut self, now_ms: u32, id: embedded_can::Id, dlc: u8, data: &[u8; 8]) {
        self.as_can_layer_mut().on_frame(now_ms, id, dlc, data);
    }

    pub fn read_signals(&self, now_ms: u32, dest: &mut CanRxData) {
        self.as_can_layer().read_signals(now_ms, dest);
    }

    pub fn write_signals(&mut self, now_ms: u32, sigs: &CanTxData) {
        self.as_can_layer_mut().write_signals(now_ms, sigs);
    }

    pub fn transmit<E, F: FnMut(StandardId, &[u8]) -> nb::Result<(), E>>(
        &self,
        f: F,
    ) -> nb::Result<(), E> {
        self.as_can_layer().transmit(f)
    }
}

/// Data that is sent over CAN to the vehicle
#[derive(Default, Copy, Clone, Debug)]
pub struct CanTxData {
    pub can_start: bool,
    pub gearbox_ok: bool,
    pub manual_shifting: bool,
    pub high_resistance: bool,
    pub garage_shifting: bool,
    pub overtemperature: bool,
    pub kickdown_active: bool,
    pub req_mil_light: bool,
    pub fourmatic: bool,
    pub large_nag: bool,
    pub activate_brake_when_shifting: bool,

    pub display_info: Option<(DisplayGear, DisplayProfile)>,
    pub gear_info: Option<(CanTargGear, CanActualGear)>,
    pub shifter_pos: Option<ShifterPosSimple>,
    pub tcc_position: TccState,
    pub torque_req: Option<TorqueRequest>,
    pub agile_mode: AgileMode,
    pub creep_torque_nm: u16,
    pub loss_torque_nm: u8,
    pub input_rpm: u16,
    pub output_rpm: u16,
    pub gearbox_temperature_c: i16,
}

impl CanTxData {
    pub const fn new() -> Self {
        Self {
            can_start: false,
            gearbox_ok: false,
            manual_shifting: false,
            high_resistance: false,
            garage_shifting: false,
            overtemperature: false,
            kickdown_active: false,
            req_mil_light: false,
            fourmatic: false,
            large_nag: false,
            activate_brake_when_shifting: false,
            display_info: None,
            gear_info: None,
            shifter_pos: None,
            tcc_position: TccState::Open,
            torque_req: None,
            agile_mode: AgileMode::Snv,
            creep_torque_nm: 0,
            loss_torque_nm: 0,
            input_rpm: 0,
            output_rpm: 0,
            gearbox_temperature_c: 0,
        }
    }
}

#[derive(Copy, Clone)]
pub struct WheelSpeeds {
    pub fr: CanResult<u16>,
    pub fl: CanResult<u16>,
    pub rr: CanResult<u16>,
    pub rl: CanResult<u16>,
}

impl WheelSpeeds {
    pub const fn new() -> Self {
        Self {
            fr: Err(CanError::MissingMsg),
            fl: Err(CanError::MissingMsg),
            rr: Err(CanError::MissingMsg),
            rl: Err(CanError::MissingMsg),
        }
    }
}

#[derive(Copy, Clone)]
pub struct TorqueOutputInfo {
    pub trq_req_ack: bool,
    pub driver_req_torque_nm: CanResult<f32>,
    pub engine_output_torque_nm: CanResult<f32>,
    pub min_torque_nm: CanResult<f32>,
    pub max_torque_nm: CanResult<f32>,
}

impl Default for TorqueOutputInfo {
    fn default() -> Self {
        Self {
            trq_req_ack: false,
            driver_req_torque_nm: Err(CanError::MissingMsg),
            engine_output_torque_nm: Err(CanError::MissingMsg),
            min_torque_nm: Err(CanError::MissingMsg),
            max_torque_nm: Err(CanError::MissingMsg),
        }
    }
}

/// Data that is received over CAN from the vehicle
#[derive(Copy, Clone)]
pub struct CanRxData {
    pub engine_rpm: CanResult<u16>,
    pub wheel_speeds: WheelSpeeds,
    pub ewm_position: CanResult<TipTronicShifterPos>,
    pub pedal_pos: CanResult<u8>,
    pub kickdown: CanResult<bool>,
    pub oil_temperature_c: CanResult<i16>,
    pub coolant_temperature_c: CanResult<i16>,
    pub intake_temperature_c: CanResult<i16>,
    pub driver_demand_torque_nm: CanResult<f32>,
    pub engine_static_torque_nm: CanResult<f32>,
    pub engine_indicated_torque_nm: CanResult<f32>,
}

impl CanRxData {
    pub const fn new() -> Self {
        Self {
            engine_rpm: Err(CanError::MissingMsg),
            wheel_speeds: WheelSpeeds::new(),
            ewm_position: Err(CanError::MissingMsg),
            pedal_pos: Err(CanError::MissingMsg),
            kickdown: Err(CanError::MissingMsg),
            oil_temperature_c: Err(CanError::MissingMsg),
            coolant_temperature_c: Err(CanError::MissingMsg),
            intake_temperature_c: Err(CanError::MissingMsg),
            driver_demand_torque_nm: Err(CanError::MissingMsg),
            engine_static_torque_nm: Err(CanError::MissingMsg),
            engine_indicated_torque_nm: Err(CanError::MissingMsg),
        }
    }
}
