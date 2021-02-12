pub const FE_INVALID: u64 = 0x01;

extern "C" {
    pub fn feenableexcept(x: u64);
}
