//! Differential test of one step: the generic sequencing (`arc_isa::sem::step`)
//! against the frozen `Cpu::step`, on random code and on hand-built loops and
//! delay-slot chains. Each case runs both CPUs from the same state for a few
//! steps and compares, after every step, the result, the whole `CpuState`,
//! memory, the exception record and the shadow call stack.

use super::Cpu;
use crate::executor::differential::{random_state, state_diff, Rng};

const CODE: u32 = 0x1000;
const MEM: usize = 0x10000;

fn alu(sub: u32, b: u32, c: u32, a: u32, p: u32, f: bool) -> u32 {
    (0x04 << 27) | ((b & 7) << 24) | ((p & 3) << 22) | ((sub & 0x3F) << 16) | ((f as u32) << 15)
        | (((b >> 3) & 7) << 12) | ((c & 0x3F) << 6) | (a & 0x3F)
}
/// `b[.d] s21` / `bl[.d] s25`, relative to PCL.
fn b(off: i32, delay: bool, link: bool) -> u32 {
    let d = off as u32;
    if link {
        (1 << 16) | ((d >> 2) & 0x1FF) << 18 | ((d >> 11) & 0x3FF) << 6 | (delay as u32) << 5 | (1 << 17)
    } else {
        ((d >> 1) & 0x3FF) << 17 | ((d >> 11) & 0x3FF) << 6 | (delay as u32) << 5
    }
}

/// One random instruction word, biased towards what sequencing cares about.
fn random_word(rng: &mut Rng) -> u32 {
    match rng.below(8) {
        // Branches and jumps with a short reach, delayed or not, linking or not.
        0 | 1 => b((rng.below(32) as i32 - 16) * 2, rng.bool(), rng.below(4) == 0),
        // LP with a short body.
        2 => alu(0x28, 0, rng.below(12) as u32, 0, 0b01, false),
        // mov lp_count, u6.
        3 => alu(0x0A, 60, 1 + rng.below(4) as u32, 0, 0b01, false),
        // j [blink] / j.d [blink] / jl [r1].
        4 => alu(0x20 + rng.below(4) as u32, 0, if rng.bool() { 31 } else { 1 }, 0, 0, false),
        // ALU, loads, stores, anything else.
        _ => rng.next() as u32,
    }
}

fn run_case(rng: &mut Rng, code: &[u8], steps: u32) {
    let mut base = random_state(rng);
    base.pc = CODE + 2 * rng.below(8) as u32;
    base.aux_lp_start = CODE + 2 * rng.below(16) as u32;
    base.aux_lp_end = CODE + 2 * rng.below(16) as u32;
    base.core_regs[60] = rng.below(4) as u32;
    base.flag_l = rng.below(4) == 0;
    base.aux_int_vector_base = 0x2000;
    base.halted = false;
    base.sleeping = false;
    for r in [1, 2, 3, 31] {
        base.core_regs[r] = CODE + 2 * rng.below(64) as u32;
    }
    let mut a = Cpu::new(MEM);
    let mut b = Cpu::new(MEM);
    for cpu in [&mut a, &mut b] {
        cpu.mem.load_binary(CODE, code);
        cpu.state = base.clone();
    }
    for s in 0..steps {
        let ra = a.legacy_step();
        let rb = b.step();
        assert_eq!(ra, rb, "step {s}: result");
        if a.state != b.state {
            panic!("step {s}: state differs: {:?}", state_diff(&a.state, &b.state));
        }
        assert_eq!(a.exceptions_taken, b.exceptions_taken, "step {s}: exceptions");
        assert_eq!(a.last_exception, b.last_exception, "step {s}: last exception");
        assert_eq!(a.shadow_call_stack, b.shadow_call_stack, "step {s}: shadow call stack");
        if ra.is_err() || a.state.halted || a.state.sleeping {
            break;
        }
    }
    assert!(a.mem.sram_snapshot() == b.mem.sram_snapshot(), "memory differs");
}

#[test]
fn step_matches_legacy_on_random_code() {
    let mut rng = Rng::new(0x57E9_0001);
    let n = std::env::var("ARC_DIFF_CASES").ok().and_then(|s| s.parse().ok()).unwrap_or(200_000u64) / 4;
    for _ in 0..n {
        let len = 1 + rng.below(8) as usize;
        let mut code = Vec::new();
        for _ in 0..len {
            code.extend_from_slice(&random_word(&mut rng).to_be_bytes());
        }
        let steps = 1 + rng.below(16) as u32;
        run_case(&mut rng, &code, steps);
    }
}

#[test]
fn step_matches_legacy_on_loops_and_delay_chains() {
    let mut rng = Rng::new(0x57E9_0002);
    let add1 = |r: u32| alu(0x00, r, 1, r, 0b01, false);
    let mut cases: Vec<Vec<u32>> = Vec::new();
    for count in 0..5 {
        for body in 1..4u32 {
            // mov lp_count, n ; lp end ; body x add ; (end) add
            let mut c = vec![alu(0x0A, 60, count, 0, 0b01, false), alu(0x28, 0, 2 + 2 * body, 0, 0b01, false)];
            c.extend((0..body).map(|_| add1(4)));
            c.push(add1(5));
            cases.push(c);
        }
    }
    // Nested delayed branches, a delayed link with a faulting slot, LP_END by a jump.
    cases.push(vec![b(8, true, false), b(8, true, false), add1(4), add1(5), add1(6)]);
    cases.push(vec![b(8, true, true), 0x246F_003F, add1(4), add1(5)]);
    cases.push(vec![alu(0x0A, 60, 3, 0, 0b01, false), alu(0x28, 0, 6, 0, 0b01, false), add1(4), add1(5),
                    b(-8, false, false), add1(6)]);
    for c in &cases {
        let code: Vec<u8> = c.iter().flat_map(|w| w.to_be_bytes()).collect();
        for _ in 0..2_000 {
            run_case(&mut rng, &code, 24);
        }
    }
}
