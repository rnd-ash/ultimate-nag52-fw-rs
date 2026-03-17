use atsamd_hal::{
    adc::{Adc0, Adc1, FutureAdc},
    pac::Supc,
};
use bsp::{
    AccelP, PmonKl15, PmonKl87, PmonKl87Diag,
    PmonSensors, Tft, TsenPcb, TsenTle8242,
};

use crate::{
    sensors::variable_adc_input::{VariableAcMBrk, VariableGpio1, VariableGpio2, VariableGpio3},
    Adc0Irqs, Adc1Irqs,
};

pub struct Adc0Pins {
    pub tsen_tl8242: TsenTle8242,
    pub tsen_pcb: TsenPcb,
}

#[derive(Clone, Copy, defmt::Format)]
pub struct Adc0Result {
    /// Tsen ADC reading (0-4095)
    pub tsen_tle82423: u16,
    /// Tsen ADC reading (0-4095)
    pub tsen_pcb: u16,
    /// Core voltage in mV
    pub core_mv: u16,
    /// IO voltage in mV
    pub io_mv: u16,
    /// CPU Temp in 1/10th C
    pub t_cpu: i16,
}

impl Adc0Pins {
    pub async fn poll_all(
        &mut self,
        adc0: &mut FutureAdc<Adc0, Adc0Irqs>,
        supc: &mut Supc,
    ) -> Adc0Result {
        Adc0Result {
            tsen_tle82423: adc0.read(&mut self.tsen_tl8242).await,
            tsen_pcb: adc0.read(&mut self.tsen_pcb).await,
            core_mv: adc0
                .read_cpu_voltage(atsamd_hal::adc::CpuVoltageSource::Core)
                .await,
            io_mv: adc0
                .read_cpu_voltage(atsamd_hal::adc::CpuVoltageSource::Io)
                .await,
            t_cpu: (adc0.read_cpu_temperature(supc).await * 10.0) as i16,
        }
    }
}

pub struct Adc1Pins {
    pub ac_p: AccelP,
    pub tft: Tft,
    pub pmon_kl15: PmonKl15,
    pub pmon_kl87: PmonKl87,
    pub pmon_sens: PmonSensors,
    pub pmon_kl87_diag: PmonKl87Diag,
}

pub struct Adc1VariableInputs {
    pub gpio_1: Option<VariableGpio1>,
    pub gpio_2: Option<VariableGpio2>,
    pub gpio_3: Option<VariableGpio3>,
    pub ac_m_brk: VariableAcMBrk,
}

#[derive(Clone, Copy, defmt::Format)]
pub struct Adc1Result {
    pub ac_p: u16,
    pub tft: u16,
    pub pmon_kl15: u16,
    pub pmon_kl87: u16,
    pub pmon_sens: u16,
    pub pmon_kl87_diag: u16,
}

impl Adc1Pins {
    pub async fn poll_all(&mut self, adc1: &mut FutureAdc<Adc1, Adc1Irqs>) -> Adc1Result {
        Adc1Result {
            ac_p: adc1.read(&mut self.ac_p).await,
            tft: adc1.read(&mut self.tft).await,
            pmon_kl15: adc1.read(&mut self.pmon_kl15).await,
            pmon_kl87: adc1.read(&mut self.pmon_kl87).await,
            pmon_sens: adc1.read(&mut self.pmon_sens).await,
            pmon_kl87_diag: adc1.read(&mut self.pmon_kl87_diag).await,
        }
    }
}

impl Adc1VariableInputs {
    pub async fn poll_all(
        &mut self,
        adc1: &mut FutureAdc<Adc1, Adc1Irqs>,
    ) -> VariableInputVoltages {
        let gpio_1_mv = match self.gpio_1.as_mut() {
            Some(vai) => Some(vai.get_voltage_mv(adc1).await),
            None => None,
        };

        let gpio_2_mv = match self.gpio_2.as_mut() {
            Some(vai) => Some(vai.get_voltage_mv(adc1).await),
            None => None,
        };

        let gpio_3_mv = match self.gpio_3.as_mut() {
            Some(vai) => Some(vai.get_voltage_mv(adc1).await),
            None => None,
        };

        VariableInputVoltages {
            gpio_1_mv,
            gpio_2_mv,
            gpio_3_mv,
            accel_m_brk_mv: self.ac_m_brk.get_voltage_mv(adc1).await,
        }
    }
}

#[derive(Clone, Copy, defmt::Format)]

pub struct VariableInputVoltages {
    gpio_1_mv: Option<u16>,
    gpio_2_mv: Option<u16>,
    gpio_3_mv: Option<u16>,
    accel_m_brk_mv: u16,
}
