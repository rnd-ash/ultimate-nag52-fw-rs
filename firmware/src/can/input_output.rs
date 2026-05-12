use crate::gearbox_control::Gear;



#[repr(u8)]
#[derive(Copy, Clone, defmt::Format, Default)]
pub enum TorqueReqTy {
    #[default]
    Min,
    Max,
}

#[derive(Copy, Clone, defmt::Format, Default)]
pub enum TorqueReqPhase {
    #[default]
    Decend,
    Hold,
    RampToEnd
}

#[derive(Copy, Clone, defmt::Format, Default)]
pub struct TorqueRequest {
    /// The actual value of torque (Nm)
    pub m_req: f32,
    pub ty: TorqueReqTy
}


#[derive(Copy, Clone, defmt::Format, Default)]
pub struct CanTxSignals {
    pub torque_request: Option<TorqueRequest>,
    /// ATF Temperature in Celcius
    pub atf_temperature: i16,
    pub gear_target: Option<Gear>,
    pub gear_actual: Option<Gear>
}

#[derive(Copy, Clone, defmt::Format, Default)]
pub struct CanRxSignals {

}

