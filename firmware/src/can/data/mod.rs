use mcan::embedded_can;

pub mod egs_51;
pub mod egs_52;
pub mod egs_53;
pub mod hfm_can;
pub mod slave_mode;

pub trait SignalFrame: Copy {
    const CAN_ID: embedded_can::StandardId;

    fn as_tx_can_msg(&self) -> mcan::message::tx::Message<8>;
}
