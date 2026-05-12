//! # Speed sensor module
//!
//! The TCU has multiple inputs that require continuous pulse counting. These are mainly
//! for speed sensors within or outside the gearbox.
//!
//! Each speed sensor signal requires multiple peripherals to function together in order
//! to count pulses without any CPU intervension. The mapping of signals to peripherals is
//! as follows:
//!
//! |*Signal*|*Description*|*Pulses/rev*|*GPIO Pin*|*EIC Channel*|*EVSYS Channel*|*TC peripheral*|
//! |:-:|:-:|:-:|:-:|:-:|:-:|:-:|
//! |INTN2_RPM|Gearbox N2 speed sensor|60|PB18|2|0|TC0|
//! |INTN3_RPM|Gearbox N3 speed sensor|60|PB19|3|1|TC1|
//! |EXT1_RPM|Optional user speed sensor[^note]|Any[^note]|PB05|5|2|TC2|
//! |EXT2_RPM|Optional user speed sensor|Any|PD00|0|3|TC3|
//! |EXT2_RPM|Optional user speed sensor|Any|PD01|1|4|TC4|
//!
//! [^note]: On G-Class, this is used as an output shaft speed sensor with 48 pulses/rev
//!
//! ## Implementation
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
    eic::{self, Sense},
    gpio::{PB05, PB18, PB19, PD00, PD01, Pin, PullDownInterrupt},
    pac::Mclk,
};

use crate::hal_extension::{
    eic_ext::{self, EicEvGen},
    evsys,
    pulse_count::{
        self, PulseCounter, Tc0PulseCounter, Tc1PulseCounter, Tc2PulseCounter, Tc3PulseCounter,
        Tc4PulseCounter,
    },
};

/// Internal gearbox N2 speed sensor pulse counter
pub type IntN2RpmPc =
    PulseCounter<Tc0PulseCounter, evsys::Ch0, EicEvGen<Pin<PB18, PullDownInterrupt>, eic::Ch2>>;

/// Internal gearbox N3 speed sensor pulse counter
pub type IntN3RpmPc =
    PulseCounter<Tc1PulseCounter, evsys::Ch1, EicEvGen<Pin<PB19, PullDownInterrupt>, eic::Ch3>>;

/// GPIO 1 speed sensor pulse counter
pub type Ext1RpmPc =
    PulseCounter<Tc2PulseCounter, evsys::Ch2, EicEvGen<Pin<PB05, PullDownInterrupt>, eic::Ch5>>;

/// GPIO 2 speed sensor pulse counter
pub type Ext2RpmPc =
    PulseCounter<Tc3PulseCounter, evsys::Ch3, EicEvGen<Pin<PD00, PullDownInterrupt>, eic::Ch0>>;

/// GPIO 3 speed sensor pulse counter
pub type Ext3RpmPc =
    PulseCounter<Tc4PulseCounter, evsys::Ch4, EicEvGen<Pin<PD01, PullDownInterrupt>, eic::Ch1>>;

pub struct AllPulseReadings {
    pulses_n2: u16,
    pulses_n3: u16,
    pulses_ext1: Option<u16>,
    pulses_ext2: Option<u16>,
    pulses_ext3: Option<u16>,
}

pub struct AllSpeedSensors {
    n2: IntN2RpmPc,
    n3: IntN3RpmPc,
    ext1: Option<Ext1RpmPc>,
    ext2: Option<Ext2RpmPc>,
    ext3: Option<Ext3RpmPc>,
}

impl AllSpeedSensors {
    pub fn new(
        n2: IntN2RpmPc,
        n3: IntN3RpmPc,
        ext1: Option<Ext1RpmPc>,
        ext2: Option<Ext2RpmPc>,
        ext3: Option<Ext3RpmPc>,
        //settings: SpeedSensorSettings,
    ) -> Self {
        n2.clear();
        n3.clear();
        if let Some(ext1) = ext1.as_ref() {
            ext1.clear();
        }
        if let Some(ext2) = ext2.as_ref() {
            ext2.clear();
        }
        if let Some(ext3) = ext3.as_ref() {
            ext3.clear();
        }
        Self {
            n2,
            n3,
            ext1,
            ext2,
            ext3,
        }
    }

    pub fn update(&self) -> AllPulseReadings {
        let n2_res = self.n2.count_and_clear();
        let n3_res = self.n3.count_and_clear();

        let ext1_res = self.ext1.as_ref().map(|x| x.count_and_clear());
        let ext2_res = self.ext2.as_ref().map(|x| x.count_and_clear());
        let ext3_res = self.ext3.as_ref().map(|x| x.count_and_clear());

        AllPulseReadings {
            pulses_n2: n2_res,
            pulses_n3: n3_res,
            pulses_ext1: ext1_res,
            pulses_ext2: ext2_res,
            pulses_ext3: ext3_res,
        }
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
    TC: pulse_count::CounterInstance,
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
    //eic_wrapper.enable_interrupt();
    eic_wrapper.sense(Sense::High);

    let evsys_channel_ready = eic_wrapper.enable_evsys(evsys_channel);
    let pcnt = pulse_count::PulseCounterBuilder::default()
        .stop_on_overflow(true)
        .run_in_standby(true)
        .build(tc, tc_clock, mclk, evsys_channel_ready);
    pcnt.start();
    pcnt
}
