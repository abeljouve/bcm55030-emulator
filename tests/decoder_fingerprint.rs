//! Golden fingerprint of the decoder and the formatter.
//!
//! One million pseudo-random encodings (fixed seed, every major opcode, with a
//! random long immediate after each) are decoded and formatted; an FNV-1a hash
//! over the results is compared with a constant. Any change to what an
//! encoding decodes to, or to how it prints, moves the hash. A deliberate
//! change updates the constant in the same commit and says why.

use bcm55030_emulator::decoder::decode_bytes;
use bcm55030_emulator::decoder::format::format_instruction;

const CASES: u32 = 1_000_000;
const GOLDEN: u64 = 0x971b_fa29_0af7_53e4;

struct XorShift64Star(u64);
impl XorShift64Star {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
}

fn fnv1a(h: &mut u64, bytes: &[u8]) {
    for &b in bytes {
        *h ^= b as u64;
        *h = h.wrapping_mul(0x0000_0100_0000_01B3);
    }
}

#[test]
fn decoder_and_formatter_fingerprint() {
    let mut rng = XorShift64Star(0x9E37_79B9_7F4A_7C15);
    let mut h: u64 = 0xCBF2_9CE4_8422_2325;
    let base = 0x1000u32;
    for _ in 0..CASES {
        let r = rng.next();
        let bytes = r.to_be_bytes();
        let line = match decode_bytes(base, &bytes, base) {
            Ok(d) => format!("{:?}|{}", d, format_instruction(&d)),
            Err(e) => format!("err {:?}", e),
        };
        fnv1a(&mut h, line.as_bytes());
    }
    println!("decoder fingerprint over {CASES} encodings: {h:#018x}");
    assert_eq!(h, GOLDEN, "decoder/formatter output changed (fingerprint {h:#018x})");
}
