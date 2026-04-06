/// Signed 32x32 -> lower 32 bits
pub fn mpy(a: u32, b: u32) -> u32 {
    ((a as i32 as i64).wrapping_mul(b as i32 as i64)) as u32
}

/// Signed 32x32 -> upper 32 bits
pub fn mpyh(a: u32, b: u32) -> u32 {
    (((a as i32 as i64).wrapping_mul(b as i32 as i64)) >> 32) as u32
}

/// Unsigned 32x32 -> lower 32 bits
pub fn mpyu(a: u32, b: u32) -> u32 {
    ((a as u64).wrapping_mul(b as u64)) as u32
}

/// Unsigned 32x32 -> upper 32 bits
pub fn mpyhu(a: u32, b: u32) -> u32 {
    (((a as u64).wrapping_mul(b as u64)) >> 32) as u32
}
