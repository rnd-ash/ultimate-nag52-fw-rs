//! The torque converter solenoid on the 722.6 operates using special characteristics:
//!
//! There is an electrical PWM of 1000Hz driving the solenoid, but this wave is turned on and off
//! at 100Hz to create a hydraulic PWM. This hydraulic PWM acts as a pump to push fluid into the torque
//! converter clutch. This is done using a second input for a Zener shutoff MOSFET, which when activated
//! causes the torque converter solenoid to rapidly slam shut, rather than slowly close due to magnetic field decay.
//!
//! The solenoid is controled using a finite state machine, which can have the following modes of operation:
//!
//! ## Below PWM 14/255 (Off)
//! In this state, the electrical PWM of the solenoid is off, and the Zener pin is off
//!
//! ## Above PWM 244/255 (Full on)
//! In this state, the electrical PWM of the solenoid is on at 25%, the Zener pin is off
//!
//! ## Between PWM 14-244/255 (Zener control)
//! This phase requires the use of the state machine. It is split into 3 distinct phases:
//!
//! 1 (Inrush phase). Electrical PWM to the solenoid is switched on at 99%, Zener pin is off
//! 2 (Hold phase). Electrical PWM to the solenoid is reduced to 25%, Zener pin is off
//! 3 (Zener off phase). Electrical PWM to the solenoid is turned off, the Zener pin is switched on, causing the solenoid to slam shut
//!
//! The inrush phase is at maximum 2.5ms in duration. Therefore, if the total on time is less than 2.5ms,
//! phase (2 - Hold) is skipped, and the solenoid is switched off.

// IO required:
// TCC PWM   - PB18 TCC1[0]
// TCC Zener - PB16 Tcc3[0]/TC6[0]
// TCC FDBK  - PB17 EIC[1]/TC6[1]/TCC3[1]

use core::u32;

use atsamd_hal::{
    clock::Tcc2Tcc3Clock,
    eic::{self, Sense},
    fugit::ExtU32,
    gpio::{AlternateF, PB16, PD21, Pin, PullDownInterrupt},
    pac::{Mclk, Peripherals, Port, Tcc2, Tcc3, port::group::evctrl::Evact0select},
    prelude::_embedded_hal_Pwm,
    pwm::{Channel, TCC3Pinout, Tcc3Pwm},
    time::Hertz,
    timer_params::TimerParams,
};
use bsp::{TccCutoff, TccFdbk, TccPwm};
use defmt::println;

use crate::hal_extension::{
    self,
    eic_ext::{self, EicEvGen},
    evsys::{self, EvSysChannel, GenReady, Uninitialized},
};

/// TCC channel for the PWM wave
const TCC1_CHANNEL_PWM: Channel = Channel::_0;
/// TCC solenoid resistance (Ohms)
const RESISTANCE_TCC_SOL: f32 = 2.5;

/// Nanoseconds per Hydraulic PWM Period
const NS_FULL_PERIOD: u32 = 10_000_000;

pub struct Tcc2Mc1Ev;
pub struct Tcc2Mc2Ev;

pub struct Tcc2OvfEv;

pub struct ZenerPinOnEv;
pub struct ZenerPinOffEv;

pub struct ZenerSpikeEv;

impl hal_extension::evsys::EvSysGenerator for Tcc2Mc1Ev {
    const GENERATOR_ID: u8 = 0x3D; // TCC MC[1]
}

impl hal_extension::evsys::EvSysGenerator for Tcc2OvfEv {
    const GENERATOR_ID: u8 = 0x39; // TCC2 OVF
}

impl hal_extension::evsys::EvSysUser for ZenerPinOnEv {
    const USER_ID: usize = 0x1; // Port event 0
}

impl hal_extension::evsys::EvSysUser for ZenerPinOffEv {
    const USER_ID: usize = 0x2; // Port event 1
}

impl hal_extension::evsys::EvSysUser for ZenerSpikeEv {
    const USER_ID: usize = 33; // TCC2 MC0
}

/// Torque converter clutch Timer capture counter
///
/// A thin wrapper around TCC2 for adding multiple
/// watchpoints which are required for the 3 phase
/// control of the torque converter solenoid.
pub struct TccTcc {
    inner: Tcc2,
    // Cycles for a full 10ms window
    pub full_window_cycles: u32,
    port: Port,
}

impl TccTcc {
    ///
    /// Interrupt sequence:
    /// OVF -> OVF (Continuous PWM or Continuous Off)
    /// OVF -> MC1 -> OVF (Inrush, Off)
    /// OVF -> MC2 -> MC1 -> OVF (Inrush, Hold, Off)
    ///
    /// MC1 will always trigger Zener pin to 1 (EVSYS)
    /// OVF will always trigger Zener pin to 0 (EVSYS)
    ///
    ///
    /// P0->P0 (Continuous PWM or Continuous Off)
    /// P0->P1 (Inrush, Off)
    /// P0->P1->P2 (Inrush, Hold, Off)
    ///
    ///
    /// Configuration of the Torque converter system
    ///
    /// * TCC2 controls timing of the torque converter subsystem
    ///     * MC0 - Stores the STAMP value for TCC solenoid feedback (Generated via EVSYS)
    ///         * CPU queries this value against the requested PWM hydraulic PWM to check
    ///           the status of the TCC solenoid
    ///     * MC1 - Comparator for P1 time (Emits interrupt and EVSYS event)
    ///     * MC2 - Comparator for P2 time (Emits interrupt)
    ///     * OVF - Used at end of phase, for CPU to reset the subsystem
    /// * TCC3 controls the electrical PWM of the torque converter solenoid (1000Hz PWM)
    /// * EIC Channel 11 is used to generate feedback events via event system
    ///     * Each event triggers a STAMP command for TCC2 (Stamp is stored in TCC MC2)
    ///         -> The CPU can then calculate PWM duration using the stamp COUNT val
    pub fn setup<ZonChId: evsys::ChId, ZoffChId: evsys::ChId, PdChId: evsys::ChId>(
        tcc: Tcc2,
        clk: &Tcc2Tcc3Clock,
        mclk: &mut Mclk,
        evsys_channel_zener_spike: EvSysChannel<
            PdChId,
            GenReady<EicEvGen<Pin<PD21, PullDownInterrupt>, eic::Ch11>>,
        >,
        evsys_channel_zener_on: EvSysChannel<ZonChId, Uninitialized>,
        evsys_channel_zener_off: EvSysChannel<ZoffChId, Uninitialized>,
    ) -> Self {
        // Must be done here before we wire up MCLK
        let evsys_channel_fdbk = evsys_channel_zener_spike.register_user::<ZenerSpikeEv>();

        mclk.apbcmask().modify(|_, w| w.tcc2_().set_bit());
        tcc.ctrla().write(|w| w.enable().clear_bit());
        while tcc.syncbusy().read().enable().bit_is_set() {}
        tcc.ctrla().write(|w| w.swrst().set_bit());
        while tcc.syncbusy().read().swrst().bit_is_set() {}

        // Enable input event capturing on CH0
        tcc.ctrla().write(|w| w.cpten0().set_bit());

        let maximum = TimerParams::new_ns(NS_FULL_PERIOD.nanos(), clk.freq());

        // Enable period at 10ms (Complete cycle)
        unsafe { tcc.per().modify(|_, w| w.per().bits(maximum.cycles)) };
        // CC0 is used for Stamping the Zener pulse
        // Enable CC1 and CC2, but set their watchpoints above the Period value,
        // so the threshold is never reached to trigger the interrupt
        unsafe { tcc.cc(0).modify(|_, w| w.cc().bits(0)) };
        unsafe { tcc.cc(1).modify(|_, w| w.cc().bits(u32::MAX)) };
        unsafe { tcc.cc(2).modify(|_, w| w.cc().bits(u32::MAX)) };
        // Enable our interrupts
        tcc.intenset().write(|s| {
            // For phase 1->2
            s.mc1().set_bit();
            // For phase 0->1
            s.mc2().set_bit();
            // For end of cycle
            s.ovf().set_bit()
        });
        // Enable event generation and receiving
        tcc.evctrl().modify(|_, w| {
            // Event for overflow (Zener pin goes low)
            w.ovfeo().set_bit();
            // Event for MC1 (Zener pin goes on)
            w.mceo1().set_bit();

            // Enable input events
            w.tcei0().set_bit();
            // Stamp to MC0
            w.evact0().stamp();
            w.mcei0().set_bit()
        });

        // Period value dicates the max of the counter
        tcc.wave().modify(|_, w| w.wavegen().nfrq());

        tcc.ctrlbclr().write(|w| {
            // Count up
            w.dir().set_bit();
            // Reset on hitting period value (To 0)
            w.oneshot().set_bit()
        });
        // Set timer divider for counting
        tcc.ctrla().modify(|_, w| {
            match maximum.divider {
                1 => w.prescaler().div1(),
                2 => w.prescaler().div2(),
                4 => w.prescaler().div4(),
                8 => w.prescaler().div8(),
                16 => w.prescaler().div16(),
                64 => w.prescaler().div64(),
                256 => w.prescaler().div256(),
                1024 => w.prescaler().div1024(),
                _ => unreachable!(),
            };
            // Start timer
            w.enable().set_bit();
            w.runstdby().set_bit()
        });

        // Wire up evsys generators
        let evsys_channel_zener_on = evsys_channel_zener_on.register_generator::<Tcc2Mc1Ev>();
        let evsys_channel_zener_off = evsys_channel_zener_off.register_generator::<Tcc2OvfEv>();
        let evsys_channel_zener_on = evsys_channel_zener_on.register_user::<ZenerPinOnEv>();
        let evsys_channel_zener_off = evsys_channel_zener_off.register_user::<ZenerPinOffEv>();

        // Setup channels for event system
        let port = unsafe { Peripherals::steal().port };
        // Zener pin is PB17, so group 1 index 17
        port.group(1).evctrl().modify(|_, w| {
            // Enable events 0 and 1
            w.portei0().set_bit();
            w.portei1().set_bit();
            // Event 0 action - Set high
            w.evact0().variant(Evact0select::Set);
            // Event 0 should modify pin 17
            unsafe {
                w.pid0().bits(17);
            }
            // Event 1 action - Set low
            unsafe {
                w.evact1().bits(Evact0select::Clr as u8);
            }
            // Event 1 should modify pin 17
            unsafe { w.pid1().bits(17) }
        });

        Self {
            inner: tcc,
            full_window_cycles: maximum.cycles,
            port,
        }
    }

    #[inline]
    /// Sets the watchpoints for P1 and P2 (Inrush and Hold times)
    /// * Set to u32::MAX to disable the watch point
    pub fn set_watchpoints(&mut self, p1: u32, p2: u32) {
        unsafe {
            self.inner.cc(1).modify(|_, w| w.cc().bits(p1));
            self.inner.cc(2).modify(|_, w| w.cc().bits(p2));
        };
    }
}

pub struct TccSol {
    tcc_pwm: Tcc3Pwm<PB16, AlternateF>,
    tcctcc: TccTcc,
    max_duty: u32,
    zener_pin: TccCutoff,
    /// Phase 1 timer watchpoint value
    phase_1_count: u32,
    /// Phase 2 timer watchpoint value
    phase_2_count: u32,
    /// Phase 1 duty cycle
    phase_1_pwm: u32,
    /// Phase 2 duty cycle
    phase_2_pwm: u32,
    /// Set when feedback spike is detected
    fdbk_timer_val: u32,
    //fdbk_eic_channel: ExtInt<Pin<PD21, Interrupt<PullDown>>, eic::Ch11>,
    req_pwm: u16,
}

impl TccSol {
    /*
    pub fn new<P: eic::EicPin<ChId = EicId>, EicId: eic::ChId, EvsysId: evsys::ChId>(
        pin: impl Into<P>,
        eic_channel: eic::Channel<EicId>, */
    pub fn new<ZonChId: evsys::ChId, ZoffChId: evsys::ChId, PdChId: evsys::ChId>(
        tcc2: Tcc2,
        tcc3: Tcc3,
        tcc2_3_clock: &Tcc2Tcc3Clock,
        pwm: TccPwm,
        zener: TccCutoff,
        fdbk: TccFdbk,
        eic_fdbk: eic::Channel<eic::Ch11>,
        mclk: &mut Mclk,
        evsys_channel_pulse_detect: EvSysChannel<PdChId, Uninitialized>,
        evsys_channel_zener_on: EvSysChannel<ZonChId, Uninitialized>,
        evsys_channel_zener_off: EvSysChannel<ZoffChId, Uninitialized>,
    ) -> Self {
        let pinout = TCC3Pinout::Pb16(pwm);
        let mut tcc_pwm = Tcc3Pwm::new(tcc2_3_clock, Hertz::Hz(1000), tcc3, pinout, mclk);

        // Setup the EIC trigger for feedback
        let mut ext = eic_ext::EicEvGen::new(fdbk.into_pull_down_interrupt(), eic_fdbk);
        ext.sense(Sense::High);
        let evsys_pusle_detect_gen_rdy = ext.enable_evsys(evsys_channel_pulse_detect);

        tcc_pwm.enable(TCC1_CHANNEL_PWM);
        tcc_pwm.set_duty(TCC1_CHANNEL_PWM, 0);
        let max_duty = tcc_pwm.get_max_duty();
        let tcctcc = TccTcc::setup(
            tcc2,
            tcc2_3_clock,
            mclk,
            evsys_pusle_detect_gen_rdy,
            evsys_channel_zener_on,
            evsys_channel_zener_off,
        );

        Self {
            tcc_pwm,
            max_duty,
            tcctcc,
            zener_pin: zener,
            phase_1_count: u32::MAX,
            phase_2_count: u32::MAX,
            phase_1_pwm: 0,
            phase_2_pwm: 0,
            fdbk_timer_val: 0,
            //fdbk_eic_channel: eic,
            req_pwm: 0,
        }
    }

    pub fn get_observed_pwm(&self) -> u16 {
        if self.req_pwm < 3000 || self.req_pwm > 62000 {
            // No Zener spike, so diagnostics is off
            self.req_pwm
        } else {
            println!("{}", self.fdbk_timer_val);
            // Diagnostics is on
            let percent = self.fdbk_timer_val as f32 / self.tcctcc.full_window_cycles as f32;
            (percent * (u16::MAX as f32)) as u16
        }
    }

    /// Writes Hydraulic PWM to the solenoid.
    ///
    /// * pwm - Hydraulic PWM duty, in the range 0 - [u16::MAX]
    /// * mv - Battery voltage in mV
    /// * temp - ATF Temperature in C
    pub fn write_tcc_sol(&mut self, pwm: u16) {
        self.req_pwm = pwm;
        if pwm < 3000 {
            // Solid off
            self.phase_1_count = u32::MAX;
            self.phase_2_count = u32::MAX;
            self.phase_1_pwm = 0;
            self.phase_2_pwm = 0;
        } else if pwm > 62000 {
            //  Solid on
            self.phase_1_count = u32::MAX;
            self.phase_2_count = u32::MAX;
            self.phase_1_pwm = self.max_duty / 4;
            self.phase_2_pwm = 0;
        } else {
            // Off -> Inrush -> (Hold)
            let cycles_on =
                ((pwm as f32 / u16::MAX as f32) * self.tcctcc.full_window_cycles as f32) as u32;
            let cycles_inrush_max = ((2_500_000.0 / NS_FULL_PERIOD as f32)
                * self.tcctcc.full_window_cycles as f32) as u32;

            let cycles_inrush = core::cmp::min(cycles_on, cycles_inrush_max);
            let cycles_hold = cycles_on.saturating_sub(cycles_inrush);

            self.phase_1_count = cycles_inrush;
            self.phase_1_pwm = (self.max_duty as f32 * 0.99) as u32;
            if 0 != cycles_hold {
                // Inrush  + Hold
                self.phase_2_count = cycles_inrush + cycles_hold;
                self.phase_2_pwm = self.max_duty / 4;
            } else {
                // Just Inrush
                self.phase_2_count = u32::MAX;
                self.phase_2_pwm = 0;
            }
        }
    }

    /// On a full period elapsed interrupt
    /// (Overflow)
    #[inline]
    pub fn on_tcc_ovf(&mut self) {
        // Clear Interrupt flag
        self.tcctcc.inner.intflag().write(|w| w.ovf().set_bit());
        // As its a fresh period, set the watchpoints
        if self.phase_2_count != u32::MAX {
            // Only MC1 (Inrush->Off)
            self.tcctcc.set_watchpoints(self.phase_1_count, u32::MAX);
        } else {
            // MC2 (->Hold), and then MC1 (->Off)
            self.tcctcc
                .set_watchpoints(self.phase_2_count, self.phase_1_count);
        }
        self.tcc_pwm.set_duty(TCC1_CHANNEL_PWM, self.phase_1_pwm);
        // Get the value from zener monitor (MC0)
        self.fdbk_timer_val = self.tcctcc.inner.cc(0).read().cc().bits();
    }

    #[inline]
    pub fn on_tcc_mc1(&mut self) {
        // Clear Interrupt flag
        self.tcctcc.inner.intflag().write(|w| w.mc1().set_bit());
        // Always going to off from this phase (Zener pin will also via EVSYS)
        self.tcc_pwm.set_duty(TCC1_CHANNEL_PWM, 0);
    }

    /// On P1 time (Transition to P2)
    #[inline]
    pub fn on_tcc_mc2(&mut self) {
        // Clear Interrupt flag
        self.tcctcc.inner.intflag().write(|w| w.mc2().set_bit());
        // Hold phase requested
        self.tcc_pwm.set_duty(TCC1_CHANNEL_PWM, self.phase_2_pwm);
    }
}
