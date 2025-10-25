//! Speed sensor module
//!
//! # Mapping (TCU):
//!
//! * INTN2_RPM (Gearbox N2 SPD sensor - 60 pulses/rev) - PA16 - EIC Channel [0]
//! * INTN3_RPM (Gearbox N3 SPD sensor - 60 pulses/rev) - PA14 - EIC Channel [14]
//!
//! * EXT1_RPM (Extra SPD Sensor TCU Pin 23) - PB08 - EIC Channel [8]
//! * EXT2_RPM (Extra SPD Sensor TCU Pin 5)  - PA06 - EIC Channel [6]
//! * EXT3_RPM (Extra SPD Sensor TCU Pin 6)  - REMOVED (Conflicting EIC channel)
//!
//! NOTE: EXT1_RPM is required for some Mercedes factory vehicles (G-Class). It is
//!       used as an output shaft speed sensor, with a count of 48 pulses/rev.
//!
//! # Mapping (Peripherals)
//!
//! * INTN2_RPM - EIC[0]  - EVSYS[0] - TC0
//! * INTN3_RPM - EIC[14] - EVSYS[1] - TC1
//! * EXT1_RPM  - EIC[8]  - EVSYS[2] - TC2
//! * EXT2_RPM  - EIC[6]  - EVSYS[3] - TC3
//!
//! # Implementation
//!
//! Each Speed sensor has its own EIC channel, Event-System channel, and TC (Timer/Counter)
//! peripheral. The EIC channel sends an event via the Event-System to the TC peripheral
//! every time a high pulse is detected on the pin. The TC then increments its counter value
//! by 1.
//!
//! This way, the peripherals are counting all speed sensors in the background, without any
//! CPU interrupts. (Worse case is 40,000 pulses/sec total). The CPU can just read the
//! COUNT register of each TC peripheral to grab the current pulse count.
//!
//! The EIC peripheral is driven by the low speed 32Khz clock, which allows for a maximum
//! pulse frequency of ~16Khz, which is more than enough. If a speed sensor reports over
//! 10Khz, something is terribly wrong.

use atsamd_hal::{
    clock::v2::pclk::Pclk,
    eic::{self},
    gpio::{Pin, PullDownInterrupt, PA06, PA14, PA16, PB08},
    pac::Mclk,
};

use crate::hal_extension::{
    eic_ext::{self, EicEvGen},
    evsys,
    pcnt::{
        self, PulseCounter, Tc0PulseCounter, Tc1PulseCounter, Tc2PulseCounter, Tc3PulseCounter,
    },
};

pub type IntN2RpmPc =
    PulseCounter<Tc0PulseCounter, evsys::Ch0, EicEvGen<Pin<PA16, PullDownInterrupt>, eic::Ch0>>;
pub type IntN3RpmPc =
    PulseCounter<Tc1PulseCounter, evsys::Ch1, EicEvGen<Pin<PA14, PullDownInterrupt>, eic::Ch14>>;

pub type Ext1RpmPc =
    PulseCounter<Tc2PulseCounter, evsys::Ch2, EicEvGen<Pin<PB08, PullDownInterrupt>, eic::Ch8>>;
pub type Ext2RpmPc =
    PulseCounter<Tc3PulseCounter, evsys::Ch3, EicEvGen<Pin<PA06, PullDownInterrupt>, eic::Ch6>>;

pub struct SpeedSensorSettings {
    pulses_rev_n2: usize,
    pulses_rev_n3: usize,
    pulses_rev_ext1: usize,
    pulses_rev_ext2: usize,
}

pub struct AllSpeedSensors {
    n2: IntN2RpmPc,
    n3: IntN3RpmPc,
    ext1: Ext1RpmPc,
    ext2: Ext2RpmPc,
    //settings: SpeedSensorSettings,
}

impl AllSpeedSensors {
    pub fn new(
        n2: IntN2RpmPc,
        n3: IntN3RpmPc,
        ext1: Ext1RpmPc,
        ext2: Ext2RpmPc,
        //settings: SpeedSensorSettings,
    ) -> Self {
        n2.clear();
        n3.clear();
        ext1.clear();
        ext2.clear();
        Self { n2, n3, ext1, ext2 }
    }

    pub fn update(&self) -> (u16, u16) {
        macro_rules! count_and_clear {
            ($sensor: ident) => {{
                let res = self.$sensor.count();
                self.$sensor.clear();
                res
            }};
        }

        let n2_res = count_and_clear!(n2);
        let n3_res = count_and_clear!(n3);
        let ext1_res = count_and_clear!(ext1);
        let ext2_res = count_and_clear!(ext2);
        (n2_res, n3_res)
    }
}

/// Initializes a speed sensor
///
/// ## Generics
/// * `PS` - Pclk source for the TC peripheral
/// * `TC` - Pulse counter Timer/Counter peripheral to use
/// * `P` - GPIO pin to use, must be linked to `EicId`
/// * `EicId` - EIC Channel ID
/// * `EvsysId` - Event System channel ID
#[allow(clippy::too_many_arguments)]
pub fn init_speed_sensor<
    PS: atsamd_hal::clock::v2::pclk::PclkSourceId,
    TC: pcnt::CounterInstance,
    P: eic::EicPin<ChId = EicId>,
    EicId: eic::ChId,
    EvsysId: evsys::ChId,
>(
    pin: impl Into<P>,
    eic_channel: eic::Channel<EicId>,
    evsys_channel: evsys::EvSysChannel<EvsysId, evsys::Uninitialized>,
    mclk: &mut Mclk,
    tc: TC::Instance,
    tc_clock: &Pclk<TC::ClockId, PS>,
) -> PulseCounter<TC, EvsysId, EicEvGen<P, EicId>> {
    let mut eic_wrapper = eic_ext::EicEvGen::new(pin.into(), eic_channel);
    eic_wrapper.enable_interrupt();
    eic_wrapper.sense(atsamd_hal::pac::eic::config::Sense0select::High);

    let evsys_channel_ready = eic_wrapper.enable_evsys(evsys_channel);
    let pcnt = pcnt::PulseCounterBuilder::default()
        .stop_on_overflow(true)
        .run_in_standby(true)
        .build(tc, tc_clock, mclk, evsys_channel_ready);
    pcnt.start();
    pcnt
}
