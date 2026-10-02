use crate::hal_extensions::qspi_async::{
    Command::{self},
    OneShot, Qspi,
};
use atsamd_hal::prelude::_atsamd_hal_embedded_hal_digital_v2_OutputPin;
use bsp::LedQspi;
use rtic_sync::arbiter::Arbiter;

pub struct QspiStorage {
    inner: Qspi<OneShot>,
    led: LedQspi,
    id: [u8; 3],
}

const CLK_CYCLES_SLOW: u8 = 20;
const CLK_CYCLES_FAST: u8 = 2;

impl QspiStorage {
    pub fn new(spi: Qspi<OneShot>, led: LedQspi) -> Self {
        // Chip reset done here
        Self {
            inner: spi,
            led,
            id: [0xFF, 0xFF, 0xFF],
        }
    }

    pub fn size_bytes(&self) -> Option<u32> {
        if self.id != [0xFF, 0xFF, 0xFF] {
            match self.id[2] {
                0x40 => Some(16 * 1024 * 1024),
                _ => None,
            }
        } else {
            None
        }
    }

    pub async fn erase_chip<T: embedded_hal_async::delay::DelayNs>(&mut self, tim: &mut T) {
        self.led.set_high().unwrap();
        self.inner.set_clk_divider(CLK_CYCLES_SLOW);
        if let Some(handle) = self.inner.erase_chip() {
            loop {
                tim.delay_ms(10).await;
                if handle.is_erase_complete() {
                    break;
                }
            }
        }
        self.inner.set_clk_divider(CLK_CYCLES_FAST);
        self.led.set_low().unwrap();
        self.init_chip(tim).await;
    }

    pub async fn erase_4k_sector<T: embedded_hal_async::delay::DelayNs>(
        &mut self,
        addr: u32,
        tim: &mut T,
    ) -> bool {
        self.led.set_high().unwrap();
        self.inner.set_clk_divider(CLK_CYCLES_SLOW);
        defmt::debug!("QSPI 4K erase from 0x{:08X}", addr);
        let ret = if let Some(handle) = self.inner.erase_sector(addr) {
            loop {
                tim.delay_ms(10).await;
                if handle.is_erase_complete() {
                    break;
                }
            }
            true
        } else {
            false
        };
        self.inner.set_clk_divider(CLK_CYCLES_FAST);
        self.led.set_low().unwrap();
        ret
    }

    pub async fn erase_32k_block<T: embedded_hal_async::delay::DelayNs>(
        &mut self,
        addr: u32,
        tim: &mut T,
    ) -> bool {
        self.led.set_high().unwrap();
        self.inner.set_clk_divider(CLK_CYCLES_SLOW);
        defmt::debug!("QSPI 32K erase from 0x{:08X}", addr);
        let ret = if let Some(handle) = self.inner.erase_block(addr) {
            loop {
                tim.delay_ms(10).await;
                if handle.is_erase_complete() {
                    break;
                }
            }
            true
        } else {
            false
        };
        self.inner.set_clk_divider(CLK_CYCLES_FAST);
        self.led.set_low().unwrap();
        ret
    }

    pub async fn write<T: embedded_hal_async::delay::DelayNs>(
        &mut self,
        addr: u32,
        data: &[u8],
        tim: &mut T,
    ) {
        defmt::debug!("QSPI write {} bytes at 0x{:08X}", data.len(), addr);
        self.with_flash(|qspi| {
            // Enable QSPI for writing
            qspi.write_command(Command::WriteStatus2, &[0x02]).unwrap();
        });
        let mut off = 0;
        for block in data.chunks(256) {
            self.with_flash(|qspi| {
                qspi.run_command(Command::WriteEnable).unwrap();
                qspi.write_memory(addr + off, block);
            });
            while !self.ready() {
                tim.delay_ms(10).await;
            }
            off += block.len() as u32;
        }
    }

    pub fn read(&mut self, addr: u32, data: &mut [u8]) {
        if data.is_empty() {
            return;
        }
        self.with_flash(|qspi| {
            qspi.read_memory(addr, data);
        })
    }

    pub fn ready(&mut self) -> bool {
        // S0 - Busy
        self.state(Command::ReadStatus) & 0x01 == 0 &&
        // S15 - Suspend status
        self.state(Command::ReadStatus2) & 0x80 == 0
    }

    pub fn state(&mut self, s: Command) -> u8 {
        let mut r = [0];
        self.with_flash(|qspi| qspi.read_command(s, &mut r).unwrap());
        r[0]
    }

    fn with_flash<R, F: FnOnce(&mut Qspi<OneShot>) -> R>(&mut self, f: F) -> R {
        self.led.set_high().unwrap();
        let r = f(&mut self.inner);
        self.led.set_low().unwrap();
        r
    }

    pub async fn init_chip<T: embedded_hal_async::delay::DelayNs>(&mut self, tim: &mut T) {
        // Wait for pending ops to complete
        while !self.ready() {
            tim.delay_ms(1).await;
        }
        // Reset the chip
        self.with_flash(|qspi| {
            qspi.run_command(Command::EnableReset).unwrap();
            qspi.run_command(Command::Reset).unwrap();
        });
        // Nothing will be accepted for the next 30us
        tim.delay_ms(1).await;
        // Read JDEC ID
        self.id = self.with_flash(|qspi| {
            let mut buf = [0, 0, 0];
            qspi.read_command(Command::ReadId, &mut buf).unwrap();
            buf
        });
        if self.id[1] != 0xEF {
            panic!("Fatal. Flash type is not supported (0x{:02X})", self.id[1]);
        }
        self.inner.set_clk_divider(CLK_CYCLES_FAST);
        // Enable Quad SPI mode
        self.inner
            .write_command(Command::WriteStatus2, &[0x02])
            .unwrap();
        defmt::info!("QSPI init complete");
    }
}
