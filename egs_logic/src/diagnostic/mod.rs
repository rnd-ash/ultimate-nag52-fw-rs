use core::u16;

use crate::{
    Gearbox,
    storage::{GearboxAdaptStorage, GearboxCalibStorage, GearboxDtcStorage, GearboxMapStorage},
};

impl<
    'a,
    DS: GearboxDtcStorage,
    CS: GearboxCalibStorage,
    MS: GearboxMapStorage,
    AS: GearboxAdaptStorage,
> Gearbox<'a, DS, CS, MS, AS>
{
    /// Response buffer is just for the data itself, no SID/PID
    pub fn diag_read_data_by_local_ident(
        &self,
        pid: u8,
        response_buffer: &mut [u8],
    ) -> Option<usize> {
        match pid {
            0x31 => {
                // Speed information
                response_buffer[0..2].copy_from_slice(&self.vars.n2_rpm.to_be_bytes());
                response_buffer[2..4].copy_from_slice(&self.vars.n3_rpm.to_be_bytes());
                response_buffer[4..6]
                    .copy_from_slice(&self.vars.calc_input_rpm.unwrap_or(u16::MAX).to_be_bytes());
                response_buffer[6..8].copy_from_slice(&self.vars.engine_rpm.to_be_bytes());
                // Wheel speed FL (u16)
                response_buffer[8..10].copy_from_slice(
                    &self
                        .inputs
                        .can_vars
                        .wheel_speeds
                        .fl
                        .unwrap_or(u16::MAX)
                        .to_be_bytes(),
                );
                response_buffer[10..12].copy_from_slice(
                    &self
                        .inputs
                        .can_vars
                        .wheel_speeds
                        .fr
                        .unwrap_or(u16::MAX)
                        .to_be_bytes(),
                );
                response_buffer[12..14].copy_from_slice(
                    &self
                        .inputs
                        .can_vars
                        .wheel_speeds
                        .rl
                        .unwrap_or(u16::MAX)
                        .to_be_bytes(),
                );
                response_buffer[14..16].copy_from_slice(
                    &self
                        .inputs
                        .can_vars
                        .wheel_speeds
                        .rr
                        .unwrap_or(u16::MAX)
                        .to_be_bytes(),
                );
                // Vehicle speed from Rear wheels (u16)
                // Vehicle speed from Front wheels (u16)
                Some(20)
            }
            0x60 => {
                // Device mode
                response_buffer[..2].copy_from_slice(&self.dev_mode().bits().to_be_bytes());
                Some(2)
            }
            0x86 => {
                // N2 raw
                response_buffer[..2].copy_from_slice(&self.vars.n2_rpm.to_be_bytes());
                Some(2)
            }
            0x87 => {
                // N3 raw
                response_buffer[..2].copy_from_slice(&self.vars.n3_rpm.to_be_bytes());
                Some(2)
            }
            0x88 => {
                // N_OUT raw
                response_buffer[..2].copy_from_slice(&self.vars.n_out_rpm.to_be_bytes());
                Some(2)
            }

            _ => None,
        }
    }
}
