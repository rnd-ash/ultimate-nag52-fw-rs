use core::{ffi::CStr, panic::PanicInfo, ptr::slice_from_raw_parts};

#[derive(Copy, Clone)]
#[repr(C, packed(4))]
pub struct LocationInfo {
    file_str_ptr: *const u8,
    file_str_len: u32,
    col: u32,
    line: u32,
}

#[derive(Copy, Clone, defmt::Format)]
pub struct LocationDisplayInfo<'a> {
    pub file_name: &'a str,
    pub col: u32,
    pub line: u32,
}

/// Panic message type
#[derive(Copy, Clone)]
#[allow(clippy::large_enum_variant)]
pub enum AppPanicMsgTy {
    /// Dynamic panic message (Created using fmt! macro)
    Dynamic(DynAppPanicMsg),
    /// Static panic message (Stored in application flash at a const address)
    Static { addr: *const u8, len: usize },
}

impl AppPanicMsgTy {
    pub fn as_bytes(&self) -> &[u8] {
        match self {
            AppPanicMsgTy::Dynamic(dyn_app_panic_msg) => dyn_app_panic_msg.as_bytes(),
            AppPanicMsgTy::Static { addr, len } => {
                let ptr = slice_from_raw_parts(*addr, *len);
                unsafe { &*ptr }
            }
        }
    }
}

const PANIC_MSG_BUF_SIZE: usize = 256;

/// Dynamic message buffer,
/// Limited to 256 bytes for the panic message
/// (255 bytes + 1 byte null)
#[derive(Copy, Clone)]
#[repr(C, packed(1))]
pub struct DynAppPanicMsg {
    buf: [u8; PANIC_MSG_BUF_SIZE],
}

impl Default for DynAppPanicMsg {
    fn default() -> Self {
        Self { buf: [0; PANIC_MSG_BUF_SIZE] }
    }
}

impl core::fmt::Write for DynAppPanicMsg {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let pos = self.buf.iter().position(|x| *x == 0).unwrap_or(PANIC_MSG_BUF_SIZE-1);
        //defmt::info!("{}  - {}", pos, s);
        let maximum = core::cmp::min(PANIC_MSG_BUF_SIZE-1 - pos, s.len());
        self.buf[pos..pos + maximum].copy_from_slice(&s.as_bytes()[..maximum]);
        self.buf[pos + maximum] = 0;
        Ok(())
    }
}

impl DynAppPanicMsg {


    pub fn as_bytes(&self) -> &[u8] {
        unsafe {
            let cstr = CStr::from_bytes_with_nul_unchecked(&self.buf);
            cstr.to_bytes()
        }
    }
}

#[derive(Copy, Clone)]
pub struct AppPanicInfo {
    msg: AppPanicMsgTy,
    location: Option<LocationInfo>,
}

impl AppPanicInfo {
    pub fn new(panic: &PanicInfo) -> Self {
        let msg = match panic.message().as_str() {
            Some(flash_string) => AppPanicMsgTy::Static {
                addr: flash_string.as_ptr(),
                len: flash_string.len(),
            },
            None => {
                use core::fmt::Write;
                let mut buffer = DynAppPanicMsg::default();
                // Dynamic string
                let _ = write!(&mut buffer, "{}", panic.message());
                AppPanicMsgTy::Dynamic(buffer)
            }
        };

        let location = panic.location().map(|loc| LocationInfo {
            file_str_ptr: loc.file().as_ptr(),
            file_str_len: loc.file().len() as u32,
            col: loc.column(),
            line: loc.line(),
        });

        Self { msg, location }
    }

    pub fn msg(&self) -> &str {
        match &self.msg {
            AppPanicMsgTy::Dynamic(dyn_app_panic_msg) => unsafe {
                str::from_utf8_unchecked(dyn_app_panic_msg.as_bytes())
            },
            AppPanicMsgTy::Static { addr, len } => {
                let slice = slice_from_raw_parts(*addr, *len);
                unsafe { str::from_utf8_unchecked(&*slice) }
            }
        }
    }

    pub fn file<'a>(&'a self) -> Option<LocationDisplayInfo<'a>> {
        self.location.map(|loc| {
            let slice = slice_from_raw_parts(loc.file_str_ptr, loc.file_str_len as usize);
            let file_name = unsafe { str::from_utf8_unchecked(&*slice) };
            LocationDisplayInfo {
                file_name,
                col: loc.col,
                line: loc.line,
            }
        })
    }
}
