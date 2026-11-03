/// Extract bits [hi:lo] from a 32-bit value (inclusive)
#[inline]
pub fn extract_bits(val: u32, hi: u8, lo: u8) -> u32 {
    let width = hi - lo + 1;
    (val >> lo) & ((1u32 << width) - 1)
}

/// Extract the major opcode (bits [31:27]) from the first halfword or full word
#[inline]
pub fn major_opcode(first_half: u16) -> u8 {
    (first_half >> 11) as u8 & 0x1F
}

/// Whether a major opcode indicates a 32-bit instruction (0x00..0x0B)
#[inline]
pub fn is_32bit_instruction(major: u8) -> bool {
    major < 0x0C
}

/// Sign-extend a value of `bits` width to 32 bits
#[inline]
pub fn sign_extend(val: u32, bits: u8) -> i32 {
    let shift = 32 - bits;
    ((val as i32) << shift) >> shift
}

/// Sign-extend a value of `bits` width to u32 (preserving bit pattern)
#[inline]
pub fn sign_extend_u32(val: u32, bits: u8) -> u32 {
    sign_extend(val, bits) as u32
}

/// Extract the B register field from a 32-bit instruction
/// B[2:0] at bits[26:24], B[5:3] at bits[14:12]
#[inline]
pub fn extract_b_reg(word: u32) -> u8 {
    let low = extract_bits(word, 26, 24) as u8;  // B[2:0]
    let high = extract_bits(word, 14, 12) as u8; // B[5:3]
    (high << 3) | low
}

/// Extract the C register field from a 32-bit instruction (bits[11:6])
#[inline]
pub fn extract_c_reg(word: u32) -> u8 {
    extract_bits(word, 11, 6) as u8
}

/// Extract the A register field from a 32-bit instruction (bits[5:0])
#[inline]
pub fn extract_a_reg(word: u32) -> u8 {
    extract_bits(word, 5, 0) as u8
}

/// Extract the F (flag setting) bit from a 32-bit instruction (bit 15)
#[inline]
pub fn extract_f_bit(word: u32) -> bool {
    (word >> 15) & 1 != 0
}

/// Extract the P (operand format) field (bits[23:22])
#[inline]
pub fn extract_p_field(word: u32) -> u8 {
    extract_bits(word, 23, 22) as u8
}

/// Extract sub-opcode for major 0x04 (bits[21:16])
#[inline]
pub fn extract_subopcode_04(word: u32) -> u8 {
    extract_bits(word, 21, 16) as u8
}

/// Extract unsigned 6-bit immediate (bits[11:6]) for P=01 format
#[inline]
pub fn extract_u6(word: u32) -> u32 {
    extract_bits(word, 11, 6)
}

/// Extract signed 12-bit immediate for P=10 format
/// bits[11:6] = S[5:0] (lower 6 bits), bits[5:0] = S[11:6] (upper 6 bits)
#[inline]
pub fn extract_s12(word: u32) -> i32 {
    let s_low = extract_bits(word, 11, 6);   // S[5:0]
    let s_high = extract_bits(word, 5, 0);   // S[11:6]
    let raw = (s_high << 6) | s_low;
    sign_extend(raw, 12)
}

/// Extract condition code Q[4:0] for P=11 format (bits[4:0])
#[inline]
pub fn extract_condition_q(word: u32) -> u8 {
    extract_bits(word, 4, 0) as u8
}

/// Extract delay slot mode N bit (bit 5)
#[inline]
pub fn extract_n_bit(word: u32) -> bool {
    (word >> 5) & 1 != 0
}

/// Map 3-bit 16-bit register encoding to actual register index
/// 0->r0, 1->r1, 2->r2, 3->r3, 4->r12, 5->r13, 6->r14, 7->r15
#[inline]
pub fn map_16bit_reg(encoded: u8) -> u8 {
    match encoded & 0x07 {
        0 => 0,
        1 => 1,
        2 => 2,
        3 => 3,
        4 => 12,
        5 => 13,
        6 => 14,
        7 => 15,
        _ => unreachable!(),
    }
}

/// Extract H register (6-bit) from 16-bit instruction format 0x0E
/// h[2:0] at bits[7:5], h[5:3] at bits[2:0]
#[inline]
pub fn extract_h_reg_16(half: u16) -> u8 {
    let low = ((half >> 5) & 0x07) as u8;  // h[2:0]
    let high = (half & 0x07) as u8;        // h[5:3]
    (high << 3) | low
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_extract_bits() {
        assert_eq!(extract_bits(0xDEADBEEF, 31, 24), 0xDE);
        assert_eq!(extract_bits(0xDEADBEEF, 7, 0), 0xEF);
        assert_eq!(extract_bits(0b1010_0000, 7, 4), 0b1010);
    }

    #[test]
    fn test_major_opcode() {
        // 0x20 = 0b00100_000_00000000 -> major = 0b00100 = 4
        assert_eq!(major_opcode(0x2000), 0x04);
        // 0x00 = major 0
        assert_eq!(major_opcode(0x0000), 0x00);
        // 0xF8 = 0b11111_000_00000000 -> major = 0x1F
        assert_eq!(major_opcode(0xF800), 0x1F);
    }

    #[test]
    fn test_sign_extend() {
        assert_eq!(sign_extend(0b11111111, 8), -1);
        assert_eq!(sign_extend(0b01111111, 8), 127);
        assert_eq!(sign_extend(0b100000000000, 12), -2048);
    }

    #[test]
    fn test_extract_b_reg() {
        // B[2:0] at bits[26:24] = 0b001, B[5:3] at bits[14:12] = 0b000
        // -> B = 0b000_001 = 1
        let word: u32 = 0b00100_001_00_000000_0_000_000010_000000;
        assert_eq!(extract_b_reg(word), 1);
    }

    #[test]
    fn test_map_16bit_reg() {
        assert_eq!(map_16bit_reg(0), 0);
        assert_eq!(map_16bit_reg(3), 3);
        assert_eq!(map_16bit_reg(4), 12);
        assert_eq!(map_16bit_reg(7), 15);
    }

    #[test]
    fn test_extract_s12() {
        // bits[11:6] = S[5:0] (lower), bits[5:0] = S[11:6] (upper)
        // value = 1 => S[11:6]=0, S[5:0]=1 => bits[11:6]=1, bits[5:0]=0
        let word = (0b000001u32 << 6) | 0b000000u32;
        assert_eq!(extract_s12(word), 1);

        // value = 0x72 (114) => S[11:6]=0b000001 (1), S[5:0]=0b110010 (50)
        // bits[11:6]=50, bits[5:0]=1
        let word2 = (50u32 << 6) | 1u32;
        assert_eq!(extract_s12(word2), 0x72);
    }
}
