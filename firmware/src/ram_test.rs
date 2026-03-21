


unsafe extern "C" {
    static mut _ram_test_buffer_addr: u8;
    static mut _ram_test_buffer_end_addr: u8;
    static mut _ram_start_addr: u8;
}

pub fn ram_buf_size() -> usize {
    let start = (&raw mut _ram_test_buffer_addr).addr();
    let end = (&raw mut _ram_test_buffer_end_addr).addr();
    end-start
    
    //128
}

pub fn ram_buf_ptr() -> *mut u8 {
    &raw mut _ram_test_buffer_addr
}

pub fn ram_start() -> u32 {
    (&raw mut _ram_start_addr).addr() as u32
}