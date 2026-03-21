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

use core::{u16, u32};

use atsamd_hal::{
    clock::Tcc2Tcc3Clock,
    ehal::digital::OutputPin,
    eic::{self},
    fugit::ExtU32,
    gpio::{AlternateF, PB16},
    pac::{port::group::evctrl::Evact0select, Mclk, Peripherals, Tcc2, Tcc3},
    prelude::_embedded_hal_Pwm,
    pwm::{Channel, TCC3Pinout, Tcc3Pwm},
    time::Hertz,
    timer_params::TimerParams,
};
use bsp::{TccCutoff, TccFdbk, TccPwm};
use defmt::println;

use crate::hal_extension::{
    self,
    eic_ext::{self},
    evsys::{self, EvSysChannel, Uninitialized},
};

/// TCC channel for the PWM wave
const TCC1_CHANNEL_PWM: Channel = Channel::_0;
/// TCC solenoid resistance (Ohms)
const RESISTANCE_TCC_SOL: f32 = 2.5;

/// Nanoseconds per Hydraulic PWM Period
const NS_FULL_PERIOD: u32 = 10_000_000;

pub struct Tcc2Mc0Ev {}
pub struct Tcc2Mc1Ev {}

pub struct Tcc2OvfEv {}

pub struct ZenerPinOnEv {}
pub struct ZenerPinOffEv {}

impl hal_extension::evsys::EvSysGenerator for Tcc2Mc0Ev {
    const GENERATOR_ID: u8 = 0x3C; // match compare 0 (Transition to P1)
}

impl hal_extension::evsys::EvSysGenerator for Tcc2Mc1Ev {
    const GENERATOR_ID: u8 = 0x3D; // match compare 1 (Zener switch off)
}

impl hal_extension::evsys::EvSysGenerator for Tcc2OvfEv {
    const GENERATOR_ID: u8 = 0x39; // overflow (End of phase)
}

impl hal_extension::evsys::EvSysUser for ZenerPinOnEv {
    const USER_ID: usize = 0x1; // Port event 0
}

impl hal_extension::evsys::EvSysUser for ZenerPinOffEv {
    const USER_ID: usize = 0x2; // Port event 1
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
}

impl TccTcc {
    ///
    /// Configuration of the Torque converter system
    ///
    /// * TCC2 controls timing of the torque converter subsystem
    ///     * MC0 generates CPU interrupt for end of Inrush phase (RW)
    ///         * CPU interrupt transitions to phase 1 or phase 2
    ///           (Inrush->Off or Inrush->Hold->Off)
    ///     * MC1 generates CPU interrupt and Event system output for end of hold phase (RW)
    ///         * Event system output goes to [Zener] pin to turn it off
    ///         * CPU interrupt transitions to phase 2
    ///     * MC2 stores the feedback pulse detect timestamp (See below) (R)
    ///     * OVF generates CPU interrupt Every 10ms (RW)
    ///         * CPU interrupt transitions back to phase 0 (Start cycle again)
    /// * TCC3 controls the electrical PWM of the torque converter solenoid (1000Hz PWM)
    /// * EIC Channel 11 is used to generate feedback events via event system
    ///     * Each event triggers a STAMP command for TCC2 (Stamp is stored in TCC MC2)
    ///         -> The CPU can then calculate PWM duration using Tcc2 COUNT val + Stamp COUNT val
    pub fn setup<ZonChId: evsys::ChId, ZoffChId: evsys::ChId>(
        tcc: Tcc2,
        clk: &Tcc2Tcc3Clock,
        mclk: &mut Mclk,
        evsys_channel_zener_on: EvSysChannel<ZonChId, Uninitialized>,
        evsys_channel_zener_off: EvSysChannel<ZoffChId, Uninitialized>,
    ) -> Self {
        mclk.apbcmask().modify(|_, w| w.tcc2_().set_bit());
        tcc.ctrla().write(|w| w.enable().clear_bit());
        while tcc.syncbusy().read().enable().bit_is_set() {}
        tcc.ctrla().write(|w| w.swrst().set_bit());
        while tcc.syncbusy().read().swrst().bit_is_set() {}

        let maximum = TimerParams::new_ns(NS_FULL_PERIOD.nanos(), clk.freq());

        // Enable period at 10ms (Complete cycle)
        unsafe { tcc.per().modify(|_, w| w.per().bits(maximum.cycles)) };
        // Enable CC0 and CC1, but set their watchpoints above the Period value,
        // so the threshold is never reached to trigger the interrupt
        unsafe { tcc.cc(0).modify(|_, w| w.cc().bits(u32::MAX)) };
        unsafe { tcc.cc(1).modify(|_, w| w.cc().bits(u32::MAX)) };
        // Enable our interrupts
        tcc.intenset().write(|s| {
            // For phase 0->1
            s.mc0().set_bit();
            // For phase 1->2
            s.mc1().set_bit();
            // For end of cycle
            s.ovf().set_bit()
        });
        // Enable event generation for all 3 periods
        tcc.evctrl().modify(|_, w| {
            w.mceo0().set_bit();
            w.mceo1().set_bit();
            w.ovfeo().set_bit()
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

        println!(
            "ON: {} {} {:032b}",
            evsys_channel_zener_on.user_ready(),
            evsys_channel_zener_on.busy(),
            port.group(1).evctrl().read().bits()
        );

        println!(
            "OFF: {} {}",
            evsys_channel_zener_off.user_ready(),
            evsys_channel_zener_off.busy()
        );
        //mclk.apbbmask().modify(|_, w| w.port_().set_bit());

        Self {
            inner: tcc,
            full_window_cycles: maximum.cycles,
        }
    }

    #[inline]
    /// Sets the watchpoints for P1 and P2 (Inrush and Hold times)
    /// * Set to u32::MAX to disable the watch point
    pub fn set_watchpoints(&mut self, p1: u32, p2: u32) {
        unsafe {
            self.inner.cc(0).modify(|_, w| w.cc().bits(p1));
            self.inner.cc(1).modify(|_, w| w.cc().bits(p2));
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
        _evsys_channel_pulse_detect: EvSysChannel<PdChId, Uninitialized>,
        evsys_channel_zener_on: EvSysChannel<ZonChId, Uninitialized>,
        evsys_channel_zener_off: EvSysChannel<ZoffChId, Uninitialized>,
    ) -> Self {
        let pinout = TCC3Pinout::Pb16(pwm);
        let mut tcc_pwm = Tcc3Pwm::new(&tcc2_3_clock, Hertz::Hz(1000), tcc3, pinout, mclk);
        tcc_pwm.enable(TCC1_CHANNEL_PWM);
        tcc_pwm.set_duty(TCC1_CHANNEL_PWM, 0);
        let max_duty = tcc_pwm.get_max_duty();
        let tcctcc = TccTcc::setup(
            tcc2,
            &tcc2_3_clock,
            mclk,
            evsys_channel_zener_on,
            evsys_channel_zener_off,
        );

        // Setup the EIC trigger
        let _ext = eic_ext::EicEvGen::new(fdbk.into_pull_down_interrupt(), eic_fdbk);

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
            //println!("{}", self.fdbk_timer_val);
            // Diagnostics is on
            let percent = self.fdbk_timer_val as f32 / self.tcctcc.full_window_cycles as f32;
            (percent * (u16::MAX as f32)) as u16
        }
    }

    /// Writes Hydraulic PWM to the solenoid.
    ///
    /// * pwm - Hydraulic PWM duty, in the range 0 - [u16::MAX]
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
            let cycles_inrush_max = ((2_500_000 as f32 / NS_FULL_PERIOD as f32)
                * self.tcctcc.full_window_cycles as f32) as u32;

            let cycles_inrush = core::cmp::min(cycles_on, cycles_inrush_max);
            let cycles_hold = cycles_on.saturating_sub(cycles_inrush);

            self.phase_1_count = cycles_inrush;
            self.phase_1_pwm = (self.max_duty as f32 * 0.9) as u32;
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
        self.tcctcc
            .set_watchpoints(self.phase_1_count, self.phase_2_count);
        self.tcc_pwm.set_duty(TCC1_CHANNEL_PWM, self.phase_1_pwm);
        //self.zener_pin.set_low().unwrap();
    }

    /// On P1 time (Transition to P2)
    #[inline]
    pub fn on_tcc_mc0(&mut self) {
        // Clear Interrupt flag
        self.tcctcc.inner.intflag().write(|w| w.mc0().set_bit());
        // Could be going to Hold (Inrush -> Hold -> Off) or directly to off (Inrush -> Off)
        // Hence the expression for the zener pin
        self.tcc_pwm.set_duty(TCC1_CHANNEL_PWM, self.phase_2_pwm);
        self.zener_pin
            .set_state((self.phase_2_pwm == 0).into())
            .unwrap();
    }

    /// On P2 time (Transition to P0)
    #[inline]
    pub fn on_tcc_mc1(&mut self) {
        // Clear Interrupt flag
        self.tcctcc.inner.intflag().write(|w| w.mc1().set_bit());
        // Always going to off from this phase (Hold -> Off)
        self.tcc_pwm.set_duty(TCC1_CHANNEL_PWM, 0);
        //self.zener_pin.set_high().unwrap();
    }

    /// On EIC Trigger of Zener off spike
    #[inline]
    pub fn on_fdbk_spike(&mut self) {
        //self.fdbk_eic_channel.clear_interrupt();
        //self.tcctcc.inner.ctrlbset().write(|w| {
        //    w.cmd()
        //        .variant(atsamd_hal::pac::tcc0::ctrlbset::Cmdselect::Readsync)
        //});
        //while self.tcctcc.inner.syncbusy().read().ctrlb().bit_is_set() {}
        //while self.tcctcc.inner.ctrlbset().read().cmd().bits() != 0 {}
        //self.fdbk_timer_val = self.tcctcc.inner.count().read().bits();
    }
}
