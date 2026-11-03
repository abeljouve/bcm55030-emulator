//! Differential tests: the generic semantics of `arc_isa::sem`, run on
//! `EmuDom`, against the frozen executor in `executor::legacy`. Every case
//! compares the result (exception variant and address included) and the
//! whole `CpuState`, field by field.

use arc_isa::condition::ConditionCode;
use arc_isa::dom::{Dom, Flag};
use arc_isa::sem::regs;

use super::emu_dom::EmuDom;
use super::legacy;
use crate::cpu::registers::CpuState;
use crate::memory::Memory;

pub(crate) struct Rng(u64);
impl Rng {
    pub(crate) fn new(seed: u64) -> Self {
        Self(seed | 1)
    }
    pub(crate) fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    pub(crate) fn bool(&mut self) -> bool {
        self.next() & 1 != 0
    }
    /// Half uniform, half from values that sit on edges.
    pub(crate) fn word(&mut self) -> u32 {
        const EDGES: [u32; 16] = [
            0, 1, 2, 31, 32, 33, 0x7FFF_FFFF, 0x8000_0000, 0xFFFF_FFFF, 0xFFFF, 0x8000, 0xFF,
            0x80, 3, 7, 0x1F,
        ];
        let r = self.next();
        if r & 1 == 0 { (r >> 32) as u32 } else { EDGES[(r >> 8) as usize % EDGES.len()] }
    }
}

/// A state with random registers, flags (user mode included) and PC.
pub(crate) fn random_state(rng: &mut Rng) -> CpuState {
    let mut s = CpuState::new();
    for r in s.core_regs.iter_mut() {
        *r = rng.word();
    }
    s.flag_z = rng.bool();
    s.flag_n = rng.bool();
    s.flag_c = rng.bool();
    s.flag_v = rng.bool();
    s.flag_e1 = rng.bool();
    s.flag_e2 = rng.bool();
    s.flag_a1 = rng.bool();
    s.flag_a2 = rng.bool();
    s.flag_l = rng.bool();
    s.flag_u = rng.next().is_multiple_of(4);
    s.pc = (rng.next() as u32) & 0x0007_FFFE;
    s
}

/// Lines of the two states' `Debug` output that differ.
pub(crate) fn state_diff(a: &CpuState, b: &CpuState) -> Vec<String> {
    let fa = format!("{a:#?}");
    let fb = format!("{b:#?}");
    fa.lines()
        .zip(fb.lines())
        .filter(|(x, y)| x != y)
        .map(|(x, y)| format!("legacy {} | generic {}", x.trim(), y.trim()))
        .collect()
}

#[test]
fn conditions_exhaustive() {
    let mut s = CpuState::new();
    let mut m = Memory::new(0x100);
    for q in 0..16u8 {
        let cc = ConditionCode::from_u8(q).unwrap();
        for bits in 0..16u8 {
            let (z, n, c, v) = (bits & 8 != 0, bits & 4 != 0, bits & 2 != 0, bits & 1 != 0);
            (s.flag_z, s.flag_n, s.flag_c, s.flag_v) = (z, n, c, v);
            let want = legacy::cond::evaluate(cc, z, n, c, v);
            let got = regs::cond_holds(&mut EmuDom::new(&mut s, &mut m), cc);
            assert_eq!(got, want, "{cc:?} Z{z} N{n} C{c} V{v}");
        }
    }
}

#[test]
fn core_register_reads_and_writes() {
    let mut rng = Rng::new(0xC0DE_0001);
    let mut m = Memory::new(0x100);
    for _ in 0..200 {
        let base = random_state(&mut rng);
        for user in [false, true] {
            for r in 0..64u8 {
                let mut a = base.clone();
                let mut b = base.clone();
                a.flag_u = user;
                b.flag_u = user;
                assert_eq!(
                    regs::read_core_reg(&mut EmuDom::new(&mut b, &mut m), r),
                    legacy::regs::read_core_reg(&a, r),
                    "read r{r} U={user}"
                );
                let v = rng.word();
                let want = legacy::regs::write_core_reg(&mut a, r, v);
                let got = regs::write_core_reg(&mut EmuDom::new(&mut b, &mut m), r, v);
                assert_eq!(got, want, "write r{r} U={user}");
                let d = state_diff(&a, &b);
                assert!(d.is_empty(), "write r{r} U={user}: {d:?}");
            }
        }
    }
}

#[test]
fn status32_read_and_write() {
    let mut rng = Rng::new(0xC0DE_0002);
    let mut m = Memory::new(0x100);
    for _ in 0..20_000 {
        let base = random_state(&mut rng);
        let mut a = base.clone();
        let mut b = base.clone();
        assert_eq!(
            regs::status32(&mut EmuDom::new(&mut b, &mut m)),
            legacy::regs::status32(&a),
        );
        let v = rng.word();
        legacy::regs::set_status32(&mut a, v);
        regs::set_status32(&mut EmuDom::new(&mut b, &mut m), v);
        let d = state_diff(&a, &b);
        assert!(d.is_empty(), "set_status32({v:#x}): {d:?}");
        // Every flag the trait names maps onto the field the executor uses.
        let mut e = EmuDom::new(&mut b, &mut m);
        for f in Flag::ALL {
            let bit = e.bit(v, f.status32_bit() as u8);
            assert_eq!(e.flag(f), bit, "{f:?}");
        }
    }
}
