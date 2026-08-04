use atsamd_hal::usb::{
    UsbBus,
    usb_device::device::{UsbDevice, UsbDeviceState},
};
use bsp::LedUsb;
use diag_common::isotp_endpoints::usb_isotp::UsbIsoTpInterruptHandler;
use embedded_hal::digital::OutputPin;
use usbd_serial::DefaultBufferStore;

pub struct UsbData<'a> {
    pub led: LedUsb,
    pub dev: UsbDevice<'a, UsbBus>,
    pub isotp: UsbIsoTpInterruptHandler<'a, UsbBus, DefaultBufferStore, DefaultBufferStore, 4096>,
}

impl<'a> UsbData<'a> {
    #[inline(always)]
    pub fn poll(&mut self) {
        self.isotp
            .with_serial(|serial| self.dev.poll(&mut [serial]));
        if self.dev.state() == UsbDeviceState::Configured {
            let _ = self.led.set_high();
            self.isotp.poll();
            let _ = self.led.set_low();
        }
    }
}
