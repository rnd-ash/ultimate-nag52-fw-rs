// The torque converter solenoid on the 722.6 operates using special characteristics:
//
// There is an electrical PWM of 1000Hz driving the solenoid, but this wave is turned on and off
// at 100Hz to create a hydraulic PWM in the solenoid. This is done using a second input for a Zener shutoff MOSFET
//
// between PWM 0-14/255, the TCC solenoid is turned off, and after 244/255, the Zener shutoff is permenetly off.
//
// Between these PWM ranges, the TCC solenoid operates as follows.
//
// 1. TCC solenoid 1000Hz wave is activated, the Zener shutoff is off. (100Hz PWM is done at 99% duty) - Inrush phase
// 2. After 2.5ms, the 1000Hz wave is reduced to a 40% duty (Approx) - Hold phase
// 3. After n/255 duty time, the 1000Hz wave is stopped, and the Zener shutoff output is activated

// Inputs:
// TCC PWM   - PB18 TCC1[0]
// TCC Zener - PB16 Tcc3[0]/TC6[0]
// TCC FDBK  - PB17 EIC[1]/TC6[1]/TCC3[1]

use atsamd_hal::{
    async_hal::interrupts::TC7,
    clock::{Tc6Tc7Clock, Tcc0Tcc1Clock},
    ehal::digital::OutputPin,
    gpio::{AlternateF, PB18},
    pac::{Mclk, Tc7, Tcc1},
    prelude::{InterruptDrivenTimer, _embedded_hal_Pwm},
    pwm::{Channel, TCC1Pinout, Tcc1Pwm},
    time::Hertz,
    timer::TimerCounter,
};
use bsp::{TccCutoff, TccPwm};

const TCC1_CHANNEL_PWM: Channel = Channel::_1;

const NS_FULL_PERIOD: u32 = 10000000; // Nanoseconds per full Hydraulic PWM period

#[derive(Copy, Clone, PartialEq)]
pub enum CurrentMode {
    Disabled,
    OffInrush {
        inrush_pwm: u32,
        state: u8,

        off_time_ns: u32,
        inrush_time_ns: u32,
    },
    OffInrushHold {
        inrush_pwm: u32,
        hold_pwm: u32,
        state: u8,

        off_time_ns: u32,
        inrush_time_ns: u32,
        hold_time_ns: u32,
    },
}

pub struct TccSol {
    tcc_pwm: Tcc1Pwm<PB18, AlternateF>,
    timer: TimerCounter<Tc7>,
    max_duty: u32,

    hydraulic_pwm: u16,
    on_elec_pwm: u16,
    hold_elec_pwm: u16,

    zener_pin: TccCutoff,
    stage: CurrentMode,
    stage_isr: CurrentMode,
}

impl TccSol {
    pub fn new(
        tcc1: Tcc1,
        tc7: Tc7,
        tcc1_clock: &Tcc0Tcc1Clock,
        tc7_clock: &Tc6Tc7Clock,
        pwm: TccPwm,
        zener: TccCutoff,
        mclk: &mut Mclk,
    ) -> Self {
        let pinout = TCC1Pinout::Pb18(pwm);
        let mut tcc_pwm = Tcc1Pwm::new(&tcc1_clock, Hertz::Hz(1000), tcc1, pinout, mclk);
        tcc_pwm.enable(TCC1_CHANNEL_PWM);
        tcc_pwm.set_duty(TCC1_CHANNEL_PWM, 0);
        let max_duty = tcc_pwm.get_max_duty();

        let timer = TimerCounter::tc7_(tc7_clock, tc7, mclk);

        Self {
            tcc_pwm,
            max_duty,
            timer,
            hydraulic_pwm: 0,
            on_elec_pwm: 0,
            hold_elec_pwm: 0,
            zener_pin: zener,
            stage: CurrentMode::Disabled,
            stage_isr: CurrentMode::Disabled,
        }
    }

    pub fn write_tcc_sol(&mut self, pwm: u16) {
        if pwm < 3000 {
            // Off (PWM too low)
            self.timer.disable_interrupt();
            self.tcc_pwm.set_duty(TCC1_CHANNEL_PWM, 0);
            self.zener_pin.set_low().unwrap();
            self.stage = CurrentMode::Disabled;
        } else if pwm > 62000 {
            // Hold on (PWM too high)
            self.timer.disable_interrupt();
            self.tcc_pwm.set_duty(TCC1_CHANNEL_PWM, self.max_duty / 4);
            self.zener_pin.set_low().unwrap();
            self.stage = CurrentMode::Disabled;
        } else {
            // Off, Inrush, Hold, Off
            self.timer.enable_interrupt();
            self.tcc_pwm.set_duty(TCC1_CHANNEL_PWM, 0);
            self.zener_pin.set_low().unwrap();
            self.stage = CurrentMode::Disabled;
            let hydr_pwm_on_time = (NS_FULL_PERIOD as f32 * (pwm as f32 / u16::MAX as f32)) as u32;
            // > 2ms, we can add a hold phase
            if hydr_pwm_on_time > 2 * 1000 * 1000 {
                let inrush_time = 2 * 1000 * 1000;
                let hold_time = hydr_pwm_on_time - inrush_time;
                let off_time = NS_FULL_PERIOD - (inrush_time + hold_time);
                self.stage = CurrentMode::OffInrushHold {
                    inrush_pwm: self.max_duty,
                    hold_pwm: self.max_duty / 4,
                    state: 0,
                    off_time_ns: off_time,
                    inrush_time_ns: inrush_time,
                    hold_time_ns: hold_time,
                }
            } else {
                // Too short for hold phase
                let off_time = NS_FULL_PERIOD - (hydr_pwm_on_time);
                self.stage = CurrentMode::OffInrush {
                    inrush_pwm: self.max_duty,
                    state: 0,
                    off_time_ns: off_time,
                    inrush_time_ns: hydr_pwm_on_time,
                }
            }
        }
    }

    pub fn on_timer_interrupt(&mut self) {
        match &mut self.stage_isr {
            CurrentMode::Disabled => {}
            CurrentMode::OffInrush {
                inrush_pwm,
                state,
                off_time_ns,
                inrush_time_ns,
            } => todo!(),
            CurrentMode::OffInrushHold {
                inrush_pwm,
                hold_pwm,
                state,
                off_time_ns,
                inrush_time_ns,
                hold_time_ns,
            } => todo!(),
        }
    }
}
