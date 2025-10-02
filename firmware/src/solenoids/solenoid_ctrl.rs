use atsamd_hal::time::Hertz;

use crate::solenoids::commands::{ControlMode, DividerM};

pub struct SolenoidLims {
    ki_lim: f32,
    kp_lim: f32,
    n: u16,
}

const fn floor_unsigned(f: f32) -> u16 {
    let ret = (f as u16) as f32;
    if f < ret {
        (f - 1.0) as u16
    } else {
        f as u16
    }
}

#[derive(Copy, Clone, defmt::Format)]
pub struct DitherOpts {
    pub step_size: u16,
    pub steps: u8,
}

pub struct DitherSettings {
    // Frequency of the dither signal
    freq: Hertz,
    // Amplitude of the dither signal in mA/pp
    amplitude: u16,
}

impl DitherSettings {
    pub const fn new(freq: Hertz, amplitude: u16) -> Self {
        Self { freq, amplitude }
    }
}

#[derive(Copy, Clone, defmt::Format)]
pub enum Mode {
    Pwm {
        divm: DividerM,
        divn: u16,
    },
    ConstantCurrent {
        kp: u16,
        ki: u16,
        divm: DividerM,
        divn: u16,
        dither_opts: Option<DitherOpts>,
    },
}

#[derive(defmt::Format, Clone, Copy)]
pub enum ModeError {
    /// Wn equation value is invalid.
    InvalidWn,
    /// KP overflow or underflow
    InvalidKp,
    /// KI overflow or  underflow
    InvalidKi,
    /// Dither step size overflow
    DitherStepSizeOverflow,
    /// Dither step overflow
    DitherStepOverflow,
    /// Either Dither amp or freq is 0.0,
    InvalidDitherOpts,
}

impl Mode {
    pub fn to_ctrl_mode(&self) -> ControlMode {
        match self {
            Mode::Pwm { .. } => ControlMode::Pwm,
            Mode::ConstantCurrent { .. } => ControlMode::CurrentControl,
        }
    }

    pub const fn pwm(f_pwm: Hertz, f_clk: Hertz, m: DividerM) -> Self {
        let div_m = m.get_value(false) as f32;
        let n = floor_unsigned((f_clk.raw() as f32 / f_pwm.raw() as f32 / div_m) + 0.5);
        Self::Pwm { divm: m, divn: n }
    }

    /// Calculate KP' and KI' limits in a const context
    const fn calc_lims(r_sense: f32, m: DividerM, f_pwm: Hertz, f_clk: Hertz) -> SolenoidLims {
        let div_m = m.get_value(true) as f32;
        let n = floor_unsigned((f_clk.raw() as f32 / f_pwm.raw() as f32 / div_m) + 0.5);
        let ki_lim: f32 = 4095.0 * r_sense / (0.04 * div_m * n as f32) * f_pwm.raw() as f32;
        let kp_lim: f32 = 4095.0 * r_sense / (0.04 * div_m * n as f32);
        SolenoidLims { ki_lim, kp_lim, n }
    }

    const fn calc_dither_settings(
        r_sense: f32,
        f_pwm: Hertz,
        settings: DitherSettings,
    ) -> Result<DitherOpts, ModeError> {
        if settings.amplitude == 0 || settings.freq.raw() == 0 {
            Err(ModeError::InvalidDitherOpts)
        } else {
            let steps =
                floor_unsigned((f_pwm.to_Hz() as f32 / settings.freq.raw() as f32 / 4.0) + 0.5);
            if steps > 32 {
                Err(ModeError::DitherStepOverflow)
            } else {
                let step_size = floor_unsigned(
                    (settings.amplitude as f32 / 1000.0 * 4096.0 * r_sense / (0.32 * steps as f32))
                        + 0.5,
                );
                if step_size > 1023 {
                    Err(ModeError::DitherStepSizeOverflow)
                } else {
                    Ok(DitherOpts {
                        step_size: step_size,
                        steps: steps as u8,
                    })
                }
            }
        }
    }

    /// Sets Kp and Ki based on the characteristics of the solenoid,
    /// and also calculates dither parameters if requested
    ///
    /// ## Parameters
    /// * `r_sol` - Resistance of the solenoid in Ohms
    /// * `r_sense` - Resistance of shunt resistor in Ohms
    /// * `l_sol` - Inductance of the solenoid in H
    /// * `v_batt` - Supply voltage in volts
    /// * `f_pwm` - Solenoid PWM frequency
    /// * `f_clk` - TLE8242 CLK frequency
    /// * `c` - Control variable 'C'. Default is 0.707
    /// * `ratio` - Ratio of `f_sol/C*wn`. Default is 5, higher values can be used
    ///           to increase bandwidth of the control loop.
    /// * `dither_opts` - Dither parameters if specified
    pub fn calculated_cc_mode(
        r_sol: f32,
        r_sense: f32,
        l_sol: f32,
        v_batt: f32,
        f_pwm: Hertz,
        f_clk: Hertz,
        c: f32,
        ratio: f32,
        dither_settings: Option<DitherSettings>,
    ) -> Result<Self, ModeError> {
        let divm = DividerM::_128;
        let lims = Self::calc_lims(r_sense, divm, f_pwm, f_clk);
        let wn = f_pwm.raw() as f32 / (ratio * c); // 0.707 should be configurable
        let wc = (r_sense + r_sol) / l_sol;

        let wn_lim_ki = libm::sqrtf(lims.ki_lim * v_batt / l_sol);
        let wn_lim_kp = (lims.kp_lim * v_batt / l_sol + r_sol / l_sol) / (2.0 * c);

        let wn_actual = wn.min(wn_lim_ki.min(wn_lim_kp));
        if wn_actual < wc / 2.0 / c {
            Err(ModeError::InvalidWn)
        } else {
            let ki_tick = l_sol / v_batt * (wn_actual * wn_actual);
            let kp_tick = (2.0 * c * wn_actual - r_sol / l_sol) * l_sol / v_batt;

            let ki =
                ki_tick * 0.04 * f_clk.raw() as f32 / (r_sense * (f_pwm.raw().pow(2) as f32)) + 0.5;
            let kp = kp_tick * 0.04 * f_clk.raw() as f32 / (r_sense * f_pwm.raw() as f32) + 0.5;

            if ki < 0.0 || ki > 4095.0 {
                Err(ModeError::InvalidKi)
            } else if kp < 0.0 || kp > 4095.0 {
                Err(ModeError::InvalidKp)
            } else {
                let dither_opts = match dither_settings {
                    None => None,
                    Some(settings) => Some(Self::calc_dither_settings(r_sense, f_pwm, settings)?),
                };

                Ok(Self::ConstantCurrent {
                    kp: floor_unsigned(kp),
                    ki: floor_unsigned(ki),
                    divn: lims.n,
                    divm: DividerM::_128,
                    dither_opts: dither_opts,
                })
            }
        }
    }

    pub fn div_m(&self) -> DividerM {
        match self {
            Mode::Pwm { divm, .. } => *divm,
            Mode::ConstantCurrent { divm, .. } => *divm,
        }
    }

    pub fn div_n(&self) -> u16 {
        match self {
            Mode::Pwm { divn, .. } => *divn,
            Mode::ConstantCurrent { divn, .. } => *divn,
        }
    }
}
