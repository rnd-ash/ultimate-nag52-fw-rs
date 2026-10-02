use core::mem::MaybeUninit;

use diag_common::qspi_driver::QspiStorage;
use ekv::Config;
use rtic_sync::arbiter::Arbiter;

const KB: usize = 1024;
const _4KB: usize = 4 * 1024;

const EKV_START: usize = 1024 * KB;
const EKV_END: usize = 1536 * KB;

pub struct EkvQspiStorageWrapper {
    inner: &'static Arbiter<QspiStorage>,
    qspi_start: usize,
}

pub struct EkvMutex {}

unsafe impl embassy_sync::blocking_mutex::raw::RawMutex for EkvMutex {
    const INIT: Self = EkvMutex {};

    fn lock<R>(&self, f: impl FnOnce() -> R) -> R {
        f()
    }
}

impl ekv::flash::Flash for EkvQspiStorageWrapper {
    type Error = ();

    fn page_count(&self) -> usize {
        (EKV_END - EKV_START) / _4KB
    }

    async fn erase(&mut self, page_id: ekv::flash::PageID) -> Result<(), Self::Error> {
        let flash_offset = (page_id.index() * _4KB) + EKV_START;
        defmt::debug!("EKV Erase page at 0x{:08X}", &flash_offset);
        self.inner
            .access()
            .await
            .erase_4k_sector(flash_offset as u32, &mut crate::Mono)
            .await;
        Ok(())
    }

    async fn read(
        &mut self,
        page_id: ekv::flash::PageID,
        offset: usize,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        let flash_offset = (page_id.index() * _4KB) + self.qspi_start + offset;
        self.inner.access().await.read(flash_offset as u32, data);
        Ok(())
    }

    async fn write(
        &mut self,
        page_id: ekv::flash::PageID,
        offset: usize,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        let flash_offset = (page_id.index() * _4KB) + self.qspi_start + offset;

        self.inner
            .access()
            .await
            .write(flash_offset as u32, data, &mut crate::Mono)
            .await;

        Ok(())
    }
}

pub struct QspiStorageDb {
    map: ekv::Database<EkvQspiStorageWrapper, EkvMutex>,
}

impl QspiStorageDb {
    pub async fn new(arb: &'static Arbiter<QspiStorage>, seed: u32) -> Self {
        defmt::debug!("QSPI EKV DB init. Seed: 0x{:08X}", seed);
        let mut cfg = Config::default();
        cfg.random_seed = seed;
        let s = Self {
            map: ekv::Database::new(
                EkvQspiStorageWrapper {
                    inner: arb,
                    qspi_start: 1024 * 1024,
                },
                cfg,
            ),
        };
        if let Err(e) = s.map.mount().await {
            defmt::error!("EKV mount failed. Formatting");
            if s.map.format().await.is_err() {
                defmt::error!("EKV format failed");
            } else {
                defmt::info!("EKV format OK!");
            }
            if let Err(e) = s.map.mount().await {
                defmt::error!("EKV mount failed after format!");
            }
        } else {
            defmt::info!("EKV mount OK!");
        }
        s
    }

    pub async fn set_key<T>(&mut self, k: &str, v: &T) {
        let dest_bytes = v as *const T as *const u8;
        let buf = unsafe { core::slice::from_raw_parts(dest_bytes, core::mem::size_of::<T>()) };

        self.map
            .write_transaction()
            .await
            .write(k.as_bytes(), buf)
            .await;
    }

    pub async fn get_key<T>(&mut self, k: &str) -> Option<T>
    where
        T: Sized,
    {
        let mut dest: T = unsafe { MaybeUninit::zeroed().assume_init() };

        let dest_bytes = &mut dest as *mut T as *mut u8;
        let buf = unsafe { core::slice::from_raw_parts_mut(dest_bytes, core::mem::size_of::<T>()) };

        let size = self
            .map
            .read_transaction()
            .await
            .read(k.as_bytes(), buf)
            .await
            .ok();
        if size == Some(core::mem::size_of::<T>()) {
            // Init OK!
            Some(dest)
        } else {
            None
        }
    }

    pub async fn erase_key(&mut self, k: &str) {
        self.map
            .write_transaction()
            .await
            .delete(k.as_bytes())
            .await;
    }

    pub async fn commit(&mut self) {
        self.map.write_transaction().await.commit().await;
    }
}
