//! Variable input for ADC
//!
//! This Input has 2 voltage scales. (0-5.0V and 0-15V)
//! To widen the voltage range, pull scale_drop_out to 0V
//!
//! Specifications (See schematics)
//!
//! SCALE_DROP low  : R1 = 12000 Ohm, R2 = 23300 Ohm
//! SCALE_DROP high : R1 = 12000 Ohm, R2 = 3300 Ohm

use atsamd_hal::{
    adc::{Adc1, AdcPin, FutureAdc},
    ehal::digital::{OutputPin, StatefulOutputPin},
    gpio::{AlternateB, Pin, PinId, PushPullOutput},
};

use crate::Adc1Irqs;

/// Variable input for GPIO1
pub type VariableGpio1 = VariableAdcInput<bsp::GpioAnalog1Id, bsp::ScaleDrpGpio1Id>;
/// Variable input for GPIO2
pub type VariableGpio2 = VariableAdcInput<bsp::GpioAnalog2Id, bsp::ScaleDrpGpio2Id>;
/// Variable input for GPIO3
pub type VariableGpio3 = VariableAdcInput<bsp::GpioAnalog3Id, bsp::ScaleDrpGpio3Id>;
/// Variable input for Accelerator- or Brake input
pub type VariableAcMBrk = VariableAdcInput<bsp::AccelMOrBrakeId, bsp::ScaleDrpAcmId>;

pub struct VariableAdcInput<I: PinId, S: PinId>
where
    Pin<I, AlternateB>: AdcPin<Adc1>,
{
    scale_drop_out: Pin<S, PushPullOutput>,
    adc_pin: Pin<I, AlternateB>,
}

impl<I: PinId, S: PinId> VariableAdcInput<I, S>
where
    Pin<I, AlternateB>: AdcPin<Adc1>,
{
    pub fn new(
        pin: impl Into<Pin<I, AlternateB>>,
        scale_drop: impl Into<Pin<S, PushPullOutput>>,
    ) -> Self {
        // Set system to wide range by default
        let mut s_drop = scale_drop.into();
        s_drop.set_high().unwrap();
        Self {
            scale_drop_out: s_drop,
            adc_pin: pin.into(),
        }
    }

    pub async fn get_voltage_mv(&mut self, adc: &mut FutureAdc<Adc1, Adc1Irqs>) -> u16 {
        let mut adc_res = adc.read(&mut self.adc_pin).await;
        // High = MOSFET pulled up => Wide range active
        let is_wide_range = self.scale_drop_out.is_set_high().unwrap();

        // Bounds between 1-99%
        adc_res = if adc_res > 4040 {
            4095
        } else if adc_res < 40 {
            0
        } else {
            adc_res
        };

        match is_wide_range {
            true => super::adc_reading_to_source(adc_res, 12_000, 3_300),
            false => super::adc_reading_to_source(adc_res, 12_000, 23_300),
        }
    }
}
