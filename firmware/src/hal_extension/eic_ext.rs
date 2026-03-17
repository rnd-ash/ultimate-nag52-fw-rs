//! Wrapper around EIC Channel from atsamd_hal crate
//! that can work with the Event system implementation

use core::{ops::{Deref, DerefMut}};

use atsamd_hal::{
    eic::{ChId as EvChId, EicPin},
    pac::Peripherals,
};

use crate::hal_extension::evsys::{
    ChId as EvsysChId, EvSysChannel, EvSysGenerator, GenReady, Uninitialized,
};

pub struct EicEvGen<P: EicPin, C: EvChId>(atsamd_hal::eic::ExtInt<P, C>);

impl<P, C> Deref for EicEvGen<P, C>
where
    P: EicPin<ChId = C>,
    C: EvChId,
{
    type Target = atsamd_hal::eic::ExtInt<P, C>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<P, C> DerefMut for EicEvGen<P, C>
where
    P: EicPin<ChId = C>,
    C: EvChId,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<P, C> EicEvGen<P, C>
where
    P: EicPin<ChId = C>,
    C: EvChId,
{
    pub fn new(pin: P, channel: atsamd_hal::eic::Channel<C>) -> Self {
        let c = channel.with_pin(pin);
        Self(c)
    }

    pub fn enable_evsys<EC: EvsysChId>(
        &mut self,
        evsys_channel: EvSysChannel<EC, Uninitialized>,
    ) -> EvSysChannel<EC, GenReady<Self>> {
        // atsamd crate enables the EIC channel event wrongly (Doesn't disable EIC first)
        // so we must do it manually here

        
        let eic = unsafe { Peripherals::steal().eic };
        // Turn off the EIC peripheral
        eic.ctrla().modify(|_, w| w.enable().clear_bit());
        while eic.syncbusy().read().enable().bit_is_set() {
            core::hint::spin_loop();
        }
        // Set the appropriate event system bit
        eic.evctrl().modify(|r, w| unsafe {
            w.extinteo().bits(r.extinteo().bits() | (1 << P::ChId::ID))
        });

        // Re-enable the EIC peripheral
        eic.ctrla().modify(|_, w| w.enable().set_bit());
        evsys_channel.register_generator()
    }
}

impl<P, C> EvSysGenerator for EicEvGen<P, C>
where
    P: EicPin<ChId = C>,
    C: EvChId,
{
    // For SAM5x series ONLY
    // EIC EXTIN 0-15 = EVGEN 0x12-0x21
    const GENERATOR_ID: u8 = P::ChId::ID as u8 + 0x12;
}
