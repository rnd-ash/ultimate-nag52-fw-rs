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
    clock::{Tcc0Tcc1Clock, Tcc2Tcc3Clock},
    ehal::digital::OutputPin,
    fugit::ExtU32,
    gpio::{AlternateF, PB18},
    pac::{Mclk, Tcc1, Tcc2},
    prelude::_embedded_hal_Pwm,
    pwm::{Channel, TCC1Pinout, Tcc1Pwm},
    time::Hertz,
    timer_params::TimerParams,
};
use bsp::{TccCutoff, TccPwm};

/// TCC channel for the PWM wave
const TCC1_CHANNEL_PWM: Channel = Channel::_0;
/// TCC solenoid resistance (Ohms)
const RESISTANCE_TCC_SOL: f32 = 2.5;

/// Nanoseconds per Hydraulic PWM Period
const NS_FULL_PERIOD: u32 = 10_000_000;

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
    pub fn setup(tcc: Tcc2, clk: &Tcc2Tcc3Clock, mclk: &mut Mclk) -> Self {
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
        // Period value dicates the max of the counter
        tcc.wave().modify(|_, w| w.wavegen().nfrq());

        tcc.ctrlbset().write(|w| {
            // Count up
            w.dir().clear_bit();
            // Reset on hitting period value (To 0)
            w.oneshot().clear_bit()
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
    tcc_pwm: Tcc1Pwm<PB18, AlternateF>,
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
}

impl TccSol {
    pub fn new(
        tcc1: Tcc1,
        tcc2: Tcc2,
        tcc1_clock: &Tcc0Tcc1Clock,
        tcc2_clock: &Tcc2Tcc3Clock,
        pwm: TccPwm,
        zener: TccCutoff,
        mclk: &mut Mclk,
    ) -> Self {
        let pinout = TCC1Pinout::Pb18(pwm);
        let mut tcc_pwm = Tcc1Pwm::new(&tcc1_clock, Hertz::Hz(1000), tcc1, pinout, mclk);
        tcc_pwm.enable(TCC1_CHANNEL_PWM);
        tcc_pwm.set_duty(TCC1_CHANNEL_PWM, 0);
        let max_duty = tcc_pwm.get_max_duty();
        let tcctcc = TccTcc::setup(tcc2, &tcc2_clock, mclk);

        Self {
            tcc_pwm,
            max_duty,
            tcctcc,
            zener_pin: zener,
            phase_1_count: u32::MAX,
            phase_2_count: u32::MAX,
            phase_1_pwm: 0,
            phase_2_pwm: 0,
        }
    }

    /// Writes Hydraulic PWM to the solenoid.
    ///
    /// * pwm - Hydraulic PWM duty, in the range 0 - [u16::MAX]
    pub fn write_tcc_sol(&mut self, pwm: u16) {
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
        self.zener_pin.set_low().unwrap();
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
        self.zener_pin.set_high().unwrap();
    }
}
