//! Golden fingerprints of the instruction semantics. Fixed-seed corpora per
//! instruction class run through the executor, and through `Cpu::step` for
//! sequencing; an FNV-1a hash over a fixed projection of the outcome must
//! match a constant. Any change in behaviour moves a hash: a deliberate
//! change updates the constant in the same commit and says why.
//!
//! The constants were computed with the previous, non-generic implementation
//! and checked equal on the generic one before that implementation was
//! removed.

use arc_isa::condition::ConditionCode;
use arc_isa::decoder::instruction::*;

use crate::cpu::exception::Exception;
use crate::cpu::registers::CpuState;
use crate::cpu::Cpu;
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
    fn below(&mut self, n: u64) -> u64 {
        ((self.next() >> 32) * n) >> 32
    }
    fn bool(&mut self) -> bool {
        self.next() >> 63 != 0
    }
    fn word(&mut self) -> u32 {
        const EDGES: [u32; 16] = [
            0, 1, 2, 31, 32, 33, 0x7FFF_FFFF, 0x8000_0000, 0xFFFF_FFFF, 0xFFFF, 0x8000, 0xFF,
            0x80, 3, 7, 0x1F,
        ];
        let r = self.next();
        if r >> 63 == 0 { (r >> 16) as u32 } else { EDGES[((r >> 32) % EDGES.len() as u64) as usize] }
    }
}

pub(crate) struct Fnv(u64);
impl Fnv {
    fn new() -> Self {
        Self(0xCBF2_9CE4_8422_2325)
    }
    fn bytes(&mut self, b: &[u8]) {
        for &x in b {
            self.0 ^= x as u64;
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01B3);
        }
    }
    fn u32(&mut self, v: u32) {
        self.bytes(&v.to_le_bytes());
    }
    fn bool(&mut self, v: bool) {
        self.bytes(&[v as u8]);
    }
    /// The outcome of one case: the result and the architectural state the
    /// semantics can touch. A fixed list, so adding an unrelated field to
    /// `CpuState` does not move the hashes.
    fn outcome(&mut self, r: &Result<(), Exception>, s: &CpuState) {
        self.bytes(format!("{r:?}").as_bytes());
        for v in s.core_regs {
            self.u32(v);
        }
        for f in [s.flag_z, s.flag_n, s.flag_c, s.flag_v, s.flag_e1, s.flag_e2, s.flag_a1, s.flag_a2,
                  s.flag_ae, s.flag_de, s.flag_u, s.flag_l, s.flag_h, s.pc_written, s.link_executed,
                  s.halted, s.sleeping] {
            self.bool(f);
        }
        self.u32(s.pc);
        self.bytes(format!("{:?}", s.delay_state).as_bytes());
        for v in [s.aux_semaphore, s.aux_lp_start, s.aux_lp_end, s.aux_status32_l1, s.aux_status32_l2,
                  s.aux_control0, s.aux_limit0, s.aux_int_vector_base, s.aux_mulhi, s.aux_macmode,
                  s.aux_irq_lv12, s.aux_control1, s.aux_limit1, s.aux_irq_lev, s.aux_irq_hint,
                  s.aux_itrigger, s.aux_xpu, s.aux_irq_pulse_cancel, s.aux_irq_pending, s.aux_dc_ctrl,
                  s.aux_bta, s.aux_bta_l1, s.aux_bta_l2] {
            self.u32(v);
        }
    }
}

pub(crate) fn state(rng: &mut Rng) -> CpuState {
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
    s.flag_l = rng.bool();
    s.flag_u = rng.below(8) == 0;
    s.pc = (rng.next() as u32) & 0x0007_FFFE;
    s.aux_status32_l1 = rng.word();
    s.aux_status32_l2 = rng.word();
    s.aux_bta_l1 = rng.word();
    s.aux_bta_l2 = rng.word();
    s.aux_lp_start = rng.word() & !1;
    s.aux_lp_end = rng.word() & !1;
    s
}

fn reg(rng: &mut Rng) -> u8 {
    const SPECIAL: [u8; 10] = [26, 27, 28, 29, 30, 31, 60, 61, 62, 63];
    if rng.bool() { SPECIAL[rng.below(10) as usize] } else { rng.below(26) as u8 }
}
fn opnd(rng: &mut Rng) -> Operand {
    match rng.below(3) {
        0 => Operand::Reg(reg(rng)),
        1 => Operand::Imm(rng.word()),
        _ => Operand::None,
    }
}
fn cc(rng: &mut Rng) -> Option<ConditionCode> {
    let q = rng.below(17);
    if q == 16 { None } else { ConditionCode::from_u8(q as u8) }
}
fn delay(rng: &mut Rng) -> DelayMode {
    if rng.bool() { DelayMode::Delay } else { DelayMode::NoDelay }
}

/// One random register-level instruction (no memory, no AUX).
pub(crate) fn register_insn(rng: &mut Rng) -> Instruction {
    const SINGLE: [SingleOp; 13] = [
        SingleOp::Asl, SingleOp::Asr, SingleOp::Lsr, SingleOp::Ror, SingleOp::Rrc, SingleOp::Sexb,
        SingleOp::Sexw, SingleOp::Extb, SingleOp::Extw, SingleOp::Abs, SingleOp::Not, SingleOp::Rlc,
        SingleOp::Ex,
    ];
    const EXT: [ExtArithOp; 20] = [
        ExtArithOp::Asl, ExtArithOp::Lsr, ExtArithOp::Asr, ExtArithOp::Ror, ExtArithOp::Adds,
        ExtArithOp::Subs, ExtArithOp::Divaw, ExtArithOp::Asls, ExtArithOp::Asrs, ExtArithOp::Addsdw,
        ExtArithOp::Subsdw, ExtArithOp::Swap, ExtArithOp::Norm, ExtArithOp::Sat16, ExtArithOp::Rnd16,
        ExtArithOp::Abssw, ExtArithOp::Abss, ExtArithOp::Negsw, ExtArithOp::Negs, ExtArithOp::Normw,
    ];
    const KINDS: [BrCompareKind; 8] = [
        BrCompareKind::Breq, BrCompareKind::Brne, BrCompareKind::Brlt, BrCompareKind::Brge,
        BrCompareKind::Brlo, BrCompareKind::Brhs, BrCompareKind::Bbit0, BrCompareKind::Bbit1,
    ];
    let alu_ops: Vec<AluOp> = (0..64u8).filter_map(AluOp::from_u8).collect();
    match rng.below(9) {
        0 | 1 => Instruction::Alu {
            op: alu_ops[rng.below(alu_ops.len() as u64) as usize], dst: opnd(rng), src1: opnd(rng),
            src2: opnd(rng), set_flags: rng.bool(), cc: cc(rng),
        },
        2 => Instruction::SingleOp {
            op: SINGLE[rng.below(13) as usize], dst: opnd(rng), src: opnd(rng), set_flags: rng.bool(),
            cc: cc(rng),
        },
        3 => Instruction::ExtArith {
            op: EXT[rng.below(20) as usize], dst: opnd(rng), src1: opnd(rng), src2: opnd(rng),
            set_flags: rng.bool(), cc: cc(rng),
        },
        4 => Instruction::Branch {
            offset: ((rng.next() as i32) >> 10) & !1, cc: cc(rng), delay: delay(rng), link: rng.bool(),
        },
        5 => Instruction::BranchCompare {
            kind: KINDS[rng.below(8) as usize], src1: opnd(rng), src2: opnd(rng),
            offset: ((rng.next() as i32) >> 10) & !1, delay: delay(rng),
        },
        6 => Instruction::Jump {
            target: opnd(rng), cc: cc(rng), delay: delay(rng), link: rng.bool(), flag_restore: rng.bool(),
        },
        7 => Instruction::ZeroOp(match rng.below(7) {
            0 => ZeroOp::Nop,
            1 => ZeroOp::Sleep { u6: rng.below(64) as u8 },
            2 => ZeroOp::Swi,
            3 => ZeroOp::Brk,
            4 => ZeroOp::Trap { param: rng.below(64) as u8 },
            5 => ZeroOp::Rtie,
            _ => ZeroOp::Sync,
        }),
        _ => {
            if rng.bool() {
                Instruction::Loop { offset: (rng.next() as u32) & 0x0FFE, cc: cc(rng) }
            } else {
                Instruction::Flag { src: opnd(rng), cc: cc(rng) }
            }
        }
    }
}

type Exec<'a> = &'a dyn Fn(&DecodedInstruction, &mut CpuState, &mut Memory) -> Result<(), Exception>;

/// Register-level instructions, one per case.
pub(crate) fn fingerprint_registers(exec: Exec) -> u64 {
    let mut rng = Rng::new(0x6011_0001);
    let mut h = Fnv::new();
    let mut m = Memory::new(0x100);
    for _ in 0..200_000 {
        let mut s = state(&mut rng);
        let size = if rng.bool() { 2 } else { 4 };
        let dec = DecodedInstruction { inst: register_insn(&mut rng), size, has_limm: size == 4 && rng.bool(), pc: s.pc };
        let r = exec(&dec, &mut s, &mut m);
        h.outcome(&r, &s);
    }
    h.0
}

/// Loads, stores, `lr` and `sr` on one SoC memory over the whole corpus.
pub(crate) fn fingerprint_memory(exec: Exec) -> u64 {
    use crate::memory::SRAM_SIZE;
    use crate::soc::bank::BootMode;
    let mut rng = Rng::new(0x6011_0002);
    let mut h = Fnv::new();
    let mut m = Memory::new_soc(SRAM_SIZE, BootMode::Cold);
    let sram: Vec<u8> = (0..SRAM_SIZE).map(|_| rng.next() as u8).collect();
    m.load_binary(0, &sram);
    let sizes = [DataSize::Word, DataSize::HalfWord, DataSize::Byte];
    let wbs = [WritebackMode::None, WritebackMode::PreWrite, WritebackMode::PostWrite, WritebackMode::Scaled];
    for _ in 0..100_000 {
        let mut s = state(&mut rng);
        s.core_regs[1] = match rng.below(3) {
            0 => 0x0001_0000 + rng.below(0x0006_0000) as u32,
            1 => rng.below(0x80) as u32,
            _ => 0x0300_0000 + rng.below(0x100) as u32,
        };
        s.core_regs[2] = rng.below(64) as u32;
        let off = if rng.bool() { Operand::Reg(2) } else { Operand::Imm(rng.below(64) as u32) };
        let r3 = Operand::Reg(3 + rng.below(20) as u8);
        let a = rng.word() & 0x7FF;
        let inst = match rng.below(4) {
            0 => Instruction::Load {
                dst: r3, base: Operand::Reg(1), offset: off, data_size: sizes[rng.below(3) as usize],
                sign_extend: rng.bool(), writeback: wbs[rng.below(4) as usize], cache_bypass: rng.bool(),
            },
            1 => Instruction::Store {
                src: r3, base: Operand::Reg(1), offset: off, data_size: sizes[rng.below(3) as usize],
                writeback: wbs[rng.below(4) as usize], cache_bypass: rng.bool(),
            },
            2 => Instruction::LoadAux { dst: r3, addr: Operand::Imm(a) },
            _ => Instruction::StoreAux { src: r3, addr: Operand::Imm(a) },
        };
        let dec = DecodedInstruction { inst, size: 4, has_limm: false, pc: s.pc };
        let r = exec(&dec, &mut s, &mut m);
        h.outcome(&r, &s);
        h.u32(m.misaligned_data.count as u32);
    }
    h.bytes(&m.sram_snapshot());
    h.bytes(format!("{:?}", m.dcache_snapshot()).as_bytes());
    h.0
}

/// Whole steps (`step` is `Cpu::step` or its frozen copy) over short random
/// programs, hashing the state after every step.
pub(crate) fn fingerprint_steps(step: &dyn Fn(&mut Cpu) -> Result<(), Exception>) -> u64 {
    let mut rng = Rng::new(0x6011_0003);
    let mut h = Fnv::new();
    for _ in 0..20_000 {
        let mut cpu = Cpu::new(0x10000);
        let code: Vec<u8> = (0..1 + rng.below(8)).flat_map(|_| (rng.next() as u32).to_be_bytes()).collect();
        cpu.mem.load_binary(0x1000, &code);
        let mut s = state(&mut rng);
        s.pc = 0x1000 + 2 * rng.below(8) as u32;
        s.aux_lp_start = 0x1000 + 2 * rng.below(16) as u32;
        s.aux_lp_end = 0x1000 + 2 * rng.below(16) as u32;
        s.core_regs[60] = rng.below(4) as u32;
        s.aux_int_vector_base = 0x2000;
        cpu.state = s;
        for _ in 0..1 + rng.below(16) {
            let r = step(&mut cpu);
            h.outcome(&r, &cpu.state);
            if r.is_err() || cpu.state.halted || cpu.state.sleeping {
                break;
            }
        }
    }
    h.0
}

const REGISTERS: u64 = 0xbe62_974d_92c4_42c3;
const MEMORY: u64 = 0x00a4_ecc5_21e9_aba9;
const STEPS: u64 = 0x1067_32c0_ab8a_6330;

#[test]
fn register_semantics_fingerprint() {
    let h = fingerprint_registers(&|d, s, m| super::execute(d, s, m));
    assert_eq!(h, REGISTERS, "register-level semantics changed (fingerprint {h:#018x})");
}

#[test]
fn memory_semantics_fingerprint() {
    let h = fingerprint_memory(&|d, s, m| super::execute(d, s, m));
    assert_eq!(h, MEMORY, "memory/AUX semantics changed (fingerprint {h:#018x})");
}

#[test]
fn step_fingerprint() {
    let h = fingerprint_steps(&|c| c.step());
    assert_eq!(h, STEPS, "step sequencing changed (fingerprint {h:#018x})");
}

/// One-off: the frozen implementation gives the same fingerprints.
#[test]
fn legacy_fingerprints() {
    println!("legacy registers {:#018x}", fingerprint_registers(&|d, s, m| super::legacy::execute(d, s, m)));
    println!("legacy memory    {:#018x}", fingerprint_memory(&|d, s, m| super::legacy::execute(d, s, m)));
    println!("legacy steps     {:#018x}", fingerprint_steps(&|c| c.legacy_step()));
}
