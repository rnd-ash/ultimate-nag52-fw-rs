//! EEPROM storage module
//!
//! EEPROM is broken down into 64 byte pages.
//!
//! Page 0 is reserved for version information and any other information (TBA)
//!
//! Page 1-512 are reserved for data
//!
//! Each page contains a 4 byte header, followed by 60 bytes of data.
//! The header contains:
//!     * Version (7 bits - 1-128)
//!     * Block ID (1-512 - 0 denotes an unused page)
//!     * CRC16 of the 60 bytes of data
//!
//! ## EEPRROM initialization
//!
//!
//! ## EEPROM upgrading
//!
//! EEPROM Upgrading is done 1 of 2 ways:
//!     * Major change - This is done when the header major version changes.
//!                      Here, the entire EEPROM is erased and re-initialized
//!     * Minor change - This is done if a structure on EEPROM is modified.
//!                      Here, only blocks with new versions are erased and
//!                      re-initialized
//!
//!

use arbitrary_int::{u7, u9};
use atsamd_hal::{
    dmac, ehal_async::i2c::I2c, fugit::ExtU64, rtic_time::Monotonic, sercom::i2c::{self, I2cFutureDma}
};
use diag_common::hal_extensions::dsu::Dsu;
use bsp::EepromPads;
use defmt::println;
use diag_common::embedded_crc32c;
use rtic_sync::arbiter::Arbiter;

use crate::Mono;

pub const EEPROM_VER_MAJOR: u16 = 1;
pub const EEPROM_VER_MINOR: u16 = 0;
pub const EEPROM_I2C_ADDR: u8 = 0x50;
const EEPROM_BLOCKS: usize = (32*1024)/64;

pub struct EepromHeader {
    /// Major version (If changed, the entire
    /// EEPROM must be wiped and re-initialized)
    version_major: u16,
    /// Minor version (If changed, selective
    /// blocks might need to be re-initialized
    /// or changed)
    version_minor: u16,
}

#[bitbybit::bitfield(u32, defmt_bitfields)]
pub struct EepromBlkHeader {
    /// EEPROM Block version (1-128)
    #[bits(0..=6, rw)]
    version: u7,
    /// EEPROM Block ID (1-512)
    #[bits(7..=15, rw)]
    blk_id: u9,
    /// CRC16 of the block
    #[bits(16..=31, rw)]
    crc: u16,
}

// Assert less than 64 bytes (1 EEPROM page)
pub struct EepromBlock {
    header: EepromBlkHeader,
    data: [u8; 60],
}

impl EepromBlock {
    pub fn pack(&self) -> [u8; 64] {
        let mut buf = [0; 64];
        buf[0..4].copy_from_slice(&self.header.raw_value.to_le_bytes());
        buf[4..].copy_from_slice(&self.data);
        buf
    }

    pub fn unpack(buf: [u8; 64]) -> Self {
        Self {
            header: EepromBlkHeader::new_with_raw_value(u32::from_le_bytes(buf[..4].try_into().unwrap())),
            data: buf[4..].try_into().unwrap()
        }
    }
}

pub struct Eeprom<C: dmac::ChId> {
    i2c: I2cFutureDma<i2c::Config<EepromPads>, C>,
    arbiter: &'static Arbiter<Dsu>,
}

impl<C: dmac::ChId> Eeprom<C> {
    pub fn new(i2c: I2cFutureDma<i2c::Config<EepromPads>, C>, arbiter: &'static Arbiter<Dsu>) -> Self {
        Self { i2c, arbiter }
    }

    pub fn crc16(&mut self, data: &[u8]) -> u16 {
        (embedded_crc32c::crc32c(data) & 0xFFFF_FFFF) as u16
    }

    async fn init_block(&mut self, id: u16) {
        let mut blk = EepromBlock {
            header: EepromBlkHeader::new_with_raw_value(0)
                .with_blk_id(u9::new(id))
                .with_version(u7::new(0)),
            data: [0; 60],
        };
        let crc = self.crc16(&blk.data);
        blk.header.set_crc(crc);
        let addr = id*64;
        let mut buf = [0; 66];
        buf[..2].copy_from_slice(&addr.to_be_bytes());
        buf[2..].copy_from_slice(&blk.pack());
        self.i2c.write(EEPROM_I2C_ADDR, &buf).await;
        Mono::delay(6u64.millis()).await;
    }

    async fn read_block(&mut self, id: u16) -> Option<EepromBlock> {
        let addr = id.to_be_bytes();
        let mut read_buf = [0; 64];
        let _res = self
            .i2c
            .write_read(EEPROM_I2C_ADDR, &addr, &mut read_buf)
            .await
            .ok()?;
        Some(EepromBlock::unpack(read_buf))
    }

    pub async fn init(&mut self) {
        defmt::info!("EEPROM init");
        self.init_block(0).await;
        defmt::info!("EEPROM init2");
        let mut read = [0; 64];
        let res = self
            .i2c
            .write_read(EEPROM_I2C_ADDR, &[0, 0], &mut read)
            .await;

        println!("I2C R: {:02?} {:02X}", defmt::Debug2Format(&res), read);
    }
}
