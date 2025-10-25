use atsamd_hal::adc::{AdcPin, CpuVoltageSource, SampleCount};
use atsamd_hal::gpio::{AnyPin, PA00, PB03};
use atsamd_hal::rtic_time::Monotonic;
use atsamd_hal::{
    adc::{
        Accumulation, Adc0, Adc1, AdcBuilder, AdcResolution, FutureAdc, InterruptHandler,
        Prescaler, Reference,
    },
    clock::{
        self,
        v2::{
            apb::ApbClk,
            gclk::GclkId,
            pclk::{Pclk, PclkId},
        },
    },
    pac::{self, Supc},
};
use bsp::{AccelMSense, AccelPSense, SolPwrSense, Tft, VBattSense, VSensorSense, VsolSense};
use defmt::{info, println};

use crate::maths::FirstOrderAverage;
use crate::{maths, Adc0Irqs, Adc1Irqs, Mono};

pub mod speed_sensors;

// All values here are from 0-4095 (12bit ADC reading)
pub struct AnalogSensors {
    /// Feedback of the 5V sensor supply
    vsense_feedback: FirstOrderAverage<u16, 10>,
    /// Feedback of the KL15 terminal
    vbatt_feedback: FirstOrderAverage<u16, 10>,
    /// Accelerator + Input voltage
    accel_p_sense: FirstOrderAverage<u16, 10>,
    /// Accelerator - Input voltage
    accel_m_sense: FirstOrderAverage<u16, 10>,
    /// TFT Sensor on the valve body
    tft: FirstOrderAverage<u16, 10>,
    /// Linear feedback from high side switch for solenoids
    sol_pwr_sense: FirstOrderAverage<u16, 10>,
    /// Voltage feedback of the solenoid supply (KL87)
    vsol_sense: FirstOrderAverage<u16, 10>,
}


pub struct AdcPins {
    pub vbatt_sense: VBattSense,
    pub vsensor_sense: VSensorSense,
    pub accel_plus: AccelPSense,
    pub accel_minus: AccelMSense,
    pub tft: Tft,
    pub sol_pwr_sense: SolPwrSense,
    pub vsol_sense: VsolSense,
}

#[derive(Default, Copy, Clone)]
pub struct SensorData {
    pub tft: TftSensorReading,
    /// Units: millivolts
    pub vbatt: u16,
    pub n2_rpm: u16,
    pub n3_rpm: u16,
}

pub struct AdcData {
    adc0: FutureAdc<Adc0, Adc0Irqs>,
    adc1: FutureAdc<Adc1, Adc1Irqs>,
    /// Required for reading the board temperature
    supc: Supc,
    /// Pins required
    pins: AdcPins,
}

impl AdcData {
    pub fn new<P: GclkId>(
        adc0: pac::Adc0,
        adc1: pac::Adc1,
        supc: Supc,
        pins: AdcPins,
        apb_adc0: ApbClk<clock::v2::pclk::ids::Adc0>,
        apb_adc1: ApbClk<clock::v2::pclk::ids::Adc1>,
        pclk_adc0: Pclk<clock::v2::pclk::ids::Adc0, P>,
        pclk_adc1: Pclk<clock::v2::pclk::ids::Adc1, P>,
    ) -> Self {
        // ADC0 has access to the TFT Sensor, therefore, we cannot do any sample averaging at a hardware level!
        // This is to prevent a situation where the parking lock comes on mid-reading, which in tern causes the voltage
        // to jump to 3.3V, which (When averaged with valid Temperature readings), would cause a spike in voltage that
        // isn't quite high enough to be considered Parking lock on state
        let adc_adc0 = AdcBuilder::new(Accumulation::Single(AdcResolution::_12))
            .with_vref(Reference::Intvcc1)
            .with_clock_divider(Prescaler::Div8)
            .with_clock_cycles_per_sample(16)
            .enable(adc0, apb_adc0, &pclk_adc0)
            .unwrap()
            .into_future(Adc0Irqs);
        let adc_adc1 = AdcBuilder::new(Accumulation::Single(AdcResolution::_12))
            .with_vref(Reference::Intvcc1)
            .with_clock_divider(Prescaler::Div8)
            .with_clock_cycles_per_sample(16)
            .enable(adc1, apb_adc1, &pclk_adc1)
            .unwrap()
            .into_future(Adc1Irqs);
        Self {
            adc0: adc_adc0,
            adc1: adc_adc1,
            supc,
            pins,
        }
    }

    /// Assume ADC reading is 0-4095 (=0-3.3V)
    /// R1 and R2 are in Ohms
    pub fn adc_reading_to_source(adc: u16, r1: u32, r2: u32) -> u16 {
        let div = (r2 * 1000) / (r1 + r2);
        let v_out = 3300 * (((adc as u32) * 1000) / 4096);
        (v_out / div) as u16
    }

    pub async fn update(&mut self) -> SensorData {
        let now = Mono::now().duration_since_epoch().to_micros();
        let mcu_core_supply = self.adc0.read_cpu_voltage(CpuVoltageSource::Core).await;
        // What 4095 means from ADC
        let mcu_io_supply = self.adc0.read_cpu_voltage(CpuVoltageSource::Io).await;

        let tft_adc = self.adc0.read(&mut self.pins.tft).await;
        let tft_reading = match tft_adc {
            r if r > 4090 => TftSensorReading::ParkingLock,
            _ => {
                let voltage = (tft_adc as u32 * mcu_core_supply as u32) / 4095;
                let resistance = (voltage * 2000) / (mcu_core_supply as u32 - voltage);
                let temperature = maths::interp(resistance as i32, TFT_LOOKUP);
                TftSensorReading::Temperature(temperature as i16)
            }
        };

        // Voltage supplies (0-12V) (R1 = 10KOhm, R2 = 2.2KOhm)
        let board_supply = Self::adc_reading_to_source(
            self.adc1.read(&mut self.pins.vbatt_sense).await,
            10_000,
            2_200,
        );
        let solenoid_supply = Self::adc_reading_to_source(
            self.adc1.read(&mut self.pins.vsol_sense).await,
            10_000,
            2_200,
        );
        // Voltage supplies (0-5V)
        let sensor_supply = Self::adc_reading_to_source(
            self.adc1.read(&mut self.pins.vsensor_sense).await,
            10_000,
            15_000,
        );

        // Current draw
        let solenoid_current = self.adc0.read(&mut self.pins.sol_pwr_sense).await;
        // CPU Temperature
        let cpu_temp = self.adc0.read_cpu_temperature(&mut self.supc).await as i16;
        let time = Mono::now().duration_since_epoch().to_micros() - now;
        //info!(
        //    "{}us, TFT: {}, CPU: {} C. [V_KL30: {} V V_KL87: {} V] MCU Core: {}mV - MCU IO: {}mV",
        //    time,
        //    tft_reading,
        //    cpu_temp,
        //    board_supply as f32 / 1000.0,
        //    solenoid_supply as f32 / 1000.0,
        //    mcu_core_supply,
        //    mcu_io_supply
        //);
        SensorData {
            tft: tft_reading,
            vbatt: board_supply,
            n2_rpm: 0,
            n3_rpm: 0,
        }
    }
}

#[derive(Default, Clone, Copy)]
pub enum TftSensorReading {
    #[default]
    ParkingLock,
    Temperature(i16),
}

// https://www.nxp.com/docs/en/data-sheet/KTY83_SER.pdf
// KTY83/110
// Resistance (Ohm), Temperature
const TFT_LOOKUP: &[(i32, i32)] = &[
    (500, -55),
    (525, -50),
    (577, -40),
    (632, -30),
    (691, -20),
    (754, -10),
    (820, 0),
    (889, 10),
    (962, 20),
    (1000, 25),
    (1039, 30),
    (1118, 40),
    (1202, 50),
    (1288, 60),
    (1379, 70),
    (1472, 80),
    (1569, 90),
    (1670, 100),
    (1774, 110),
    (1882, 120),
    (1937, 125),
    (1993, 130),
    (2107, 140),
    (2225, 150),
    (2346, 160),
    (2471, 170),
    (2535, 175),
];
