use automotive_diag::kwp2000::KwpError;

pub struct DiagBuffer([u8; 4096]);

impl DiagBuffer {
    pub const fn new() -> Self {
        Self([0; 4096])
    }

    pub fn make_positive_reply<F: FnOnce(&mut [u8; 4095]) -> usize>(
        &mut self,
        sid: u8,
        f: F,
    ) -> usize {
        self.0[0] = sid + 0x40;
        f((&mut self.0[1..]).try_into().unwrap()) + 1
    }

    pub fn copy_positive_reply(&mut self, sid: u8, buf: &[u8]) -> usize {
        self.0[0] = sid;
        self.0[1..1 + buf.len()].copy_from_slice(buf);
        buf.len() + 1
    }

    pub fn make_negative_reply(&mut self, sid: u8, nrc: KwpError) -> usize {
        self.0[0] = 0x7F;
        self.0[1] = sid;
        self.0[2] = nrc.into();
        3
    }

    pub fn buf(&self) -> &[u8] {
        &self.0
    }
}
