use embedded_can::StandardId;

pub mod custom_can;
pub mod egs_51;
pub mod egs_52;
pub mod egs_53;
pub mod hfm_can;

pub mod slave_mode;

pub trait SignalFrame {
    const CAN_ID: StandardId;
    const LEN_BYTES: usize;
}

#[macro_use]
mod macros {
    #[macro_export]
    macro_rules! can_impl_code_gen {
        ($name: ident) => {
            impl $name {
                pub fn from_can_msg(dlc: u8, data: &[u8]) -> crate::CanResult<Self> {
                    if dlc as usize != Self::LEN_BYTES {
                        Err(crate::CanError::InvalidFrameLen)
                    } else {
                        #[cfg(target_endian = "little")]
                        let raw = u64::from_le_bytes(data.try_into().unwrap());
                        #[cfg(target_endian = "big")]
                        let raw = u64::from_be_bytes(data.try_into().unwrap());

                        Ok(Self::new_with_raw_value(raw))
                    }
                }

                pub fn to_u64_bytes(&self) -> [u8; 8] {
                    #[cfg(target_endian = "little")]
                    {
                        self.raw_value().to_le_bytes()
                    }
                    #[cfg(target_endian = "big")]
                    {
                        self.raw_value().to_be_bytes()
                    }
                }
            }
        };
    }
}
