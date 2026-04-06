use bcm55030_emulator::cpu::Cpu;

// === Instruction encoding helpers ===

fn word_bytes(w: u32) -> [u8; 4] {
    [(w >> 24) as u8, (w >> 16) as u8, (w >> 8) as u8, w as u8]
}

fn half_bytes(h: u16) -> [u8; 2] {
    [(h >> 8) as u8, h as u8]
}

/// 16-bit MOV_S b_enc, u8   (major 0x1B)
fn mov_s(b_enc: u8, val: u8) -> u16 {
    (0x1Bu16 << 11) | ((b_enc as u16 & 7) << 8) | (val as u16)
}

/// 16-bit BRK_S  (major 0x0F, i=0x1F)
fn brk_s() -> u16 {
    // bits[15:11]=01111, bits[10:8]=000, bits[7:5]=000, bits[4:0]=11111
    0x781F
}

/// 16-bit NOP_S  (major 0x0F, i=0, c_enc=7, b_enc=0)
fn nop_s() -> u16 {
    // bits[15:11]=01111, bits[10:8]=000, bits[7:5]=111, bits[4:0]=00000
    0x78E0
}

/// 16-bit B_S +offset (major 0x1E, sub=0 unconditional, s10)
fn b_s(byte_offset: i32) -> u16 {
    let s9 = ((byte_offset >> 1) & 0x1FF) as u16;
    (0x1Eu16 << 11) | (0b00u16 << 9) | s9
}

/// 16-bit BEQ_S +offset (major 0x1E, sub=1, s10)
fn beq_s(byte_offset: i32) -> u16 {
    let s9 = ((byte_offset >> 1) & 0x1FF) as u16;
    (0x1Eu16 << 11) | (0b01u16 << 9) | s9
}

/// 32-bit ALU: A = B op C (P=00) or A = B op U6 (P=01)
/// For P=00: pass c_or_u6 as register number
/// For P=01: pass c_or_u6 as immediate value
fn alu32(sub: u8, b: u8, c_or_u6: u8, a: u8, p: u8, f: bool) -> u32 {
    let b_low = (b & 0x07) as u32;
    let b_high = ((b >> 3) & 0x07) as u32;
    (0x04u32 << 27)
        | (b_low << 24)
        | ((p as u32 & 3) << 22)
        | ((sub as u32 & 0x3F) << 16)
        | ((f as u32) << 15)
        | (b_high << 12)
        | ((c_or_u6 as u32 & 0x3F) << 6)
        | (a as u32 & 0x3F)
}

/// 32-bit special op: sub-opcode 0x20-0x3F
#[allow(dead_code)]
fn special32(sub: u8, b: u8, c_or_u6: u8, p: u8, f: bool) -> u32 {
    alu32(sub, b, c_or_u6, 0, p, f)
}

/// 32-bit LD r_a, [r_b, s9_offset] (major 0x02)
fn ld32(a: u8, b: u8, offset: i16, zz: u8, x: bool) -> u32 {
    let b_low = (b & 0x07) as u32;
    let b_high = ((b >> 3) & 0x07) as u32;
    let s9 = offset as u16 as u32 & 0x1FF;
    let s_low = s9 & 0xFF;
    let s8 = (s9 >> 8) & 1;
    (0x02u32 << 27)
        | (b_low << 24)
        | (s_low << 16)
        | (s8 << 15)
        | (b_high << 12)
        | ((zz as u32 & 3) << 7)
        | ((x as u32) << 6)
        | (a as u32 & 0x3F)
}

/// 32-bit ST r_c, [r_b, s9_offset] (major 0x03)
fn st32(c: u8, b: u8, offset: i16, zz: u8) -> u32 {
    let b_low = (b & 0x07) as u32;
    let b_high = ((b >> 3) & 0x07) as u32;
    let s9 = offset as u16 as u32 & 0x1FF;
    let s_low = s9 & 0xFF;
    let s8 = (s9 >> 8) & 1;
    (0x03u32 << 27)
        | (b_low << 24)
        | (s_low << 16)
        | (s8 << 15)
        | (b_high << 12)
        | ((c as u32 & 0x3F) << 6)
        | ((zz as u32 & 3) << 1)
}

/// 32-bit LP u6_offset (major 0x04, sub=0x28, P=01)
fn lp32(u6_offset: u8) -> u32 {
    alu32(0x28, 0, u6_offset, 0, 0b01, false)
}

// === Program builder ===

struct Program {
    bytes: Vec<u8>,
}

impl Program {
    fn new() -> Self {
        Self { bytes: Vec::new() }
    }
    fn emit16(&mut self, h: u16) -> &mut Self {
        self.bytes.extend_from_slice(&half_bytes(h));
        self
    }
    fn emit32(&mut self, w: u32) -> &mut Self {
        self.bytes.extend_from_slice(&word_bytes(w));
        self
    }
    fn run(&self, max_steps: u64) -> Cpu {
        let mut cpu = Cpu::new(65536);
        cpu.mem.load_binary(0, &self.bytes);
        let _ = cpu.run(max_steps);
        cpu
    }
}

// === Tests ===

#[test]
fn test_nop_and_brk() {
    let cpu = Program::new()
        .emit16(nop_s())
        .emit16(brk_s())
        .run(10);

    assert!(cpu.state.halted);
    assert_eq!(cpu.state.instruction_count, 2);
    assert_eq!(cpu.state.pc, 0x04);
}

#[test]
fn test_mov_s_immediate() {
    let cpu = Program::new()
        .emit16(mov_s(0, 42))   // r0 = 42
        .emit16(mov_s(1, 100))  // r1 = 100
        .emit16(mov_s(2, 255))  // r2 = 255
        .emit16(brk_s())
        .run(10);

    assert!(cpu.state.halted);
    assert_eq!(cpu.state.core_regs[0], 42);
    assert_eq!(cpu.state.core_regs[1], 100);
    assert_eq!(cpu.state.core_regs[2], 255);
}

#[test]
fn test_add_reg_reg() {
    // ADD r0, r1, r2 (P=00, sub=0x00, F=0)
    let cpu = Program::new()
        .emit16(mov_s(1, 10))       // r1 = 10
        .emit16(mov_s(2, 20))       // r2 = 20
        .emit32(alu32(0x00, 1, 2, 0, 0b00, false))  // r0 = r1 + r2
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 30);
}

#[test]
fn test_add_with_flags() {
    // ADD.F r0, r1, r2 (F=1)
    let cpu = Program::new()
        .emit16(mov_s(1, 10))
        .emit16(mov_s(2, 20))
        .emit32(alu32(0x00, 1, 2, 0, 0b00, true))
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 30);
    assert!(!cpu.state.flag_z);  // 30 != 0
    assert!(!cpu.state.flag_n);  // 30 > 0
    assert!(!cpu.state.flag_c);  // no carry
    assert!(!cpu.state.flag_v);  // no overflow
}

#[test]
fn test_sub_produces_zero() {
    // SUB.F r0, r1, r2 where r1 == r2
    let cpu = Program::new()
        .emit16(mov_s(1, 42))
        .emit16(mov_s(2, 42))
        .emit32(alu32(0x02, 1, 2, 0, 0b00, true))  // SUB.F r0, r1, r2
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 0);
    assert!(cpu.state.flag_z);   // result is 0
    assert!(!cpu.state.flag_n);  // not negative
    assert!(cpu.state.flag_c);   // no borrow (42 >= 42)
    assert!(!cpu.state.flag_v);  // no overflow
}

#[test]
fn test_sub_u6_underflow() {
    // r0 = 0, then SUB.F r0, r0, 1 → 0xFFFFFFFF
    let cpu = Program::new()
        .emit16(mov_s(0, 0))
        .emit32(alu32(0x02, 0, 1, 0, 0b01, true))  // SUB.F r0, r0, #1
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 0xFFFFFFFF);
    assert!(!cpu.state.flag_z);
    assert!(cpu.state.flag_n);   // bit 31 set
    assert!(!cpu.state.flag_c);  // borrow occurred
}

#[test]
fn test_cmp_equal() {
    // CMP.F r0, r1 (same values) → Z=1
    let cpu = Program::new()
        .emit16(mov_s(0, 5))
        .emit16(mov_s(1, 5))
        .emit32(alu32(0x0C, 0, 1, 0, 0b00, true))  // CMP.F r0, r1
        .emit16(brk_s())
        .run(10);

    assert!(cpu.state.flag_z);
    assert!(cpu.state.flag_c);   // no borrow (5 >= 5)
    // r0 should be unchanged (CMP doesn't write dst)
    assert_eq!(cpu.state.core_regs[0], 5);
}

#[test]
fn test_cmp_less_than() {
    // CMP.F r0, r1 where r0 < r1 → Z=0, C=0 (borrow), N=1
    let cpu = Program::new()
        .emit16(mov_s(0, 3))
        .emit16(mov_s(1, 10))
        .emit32(alu32(0x0C, 0, 1, 0, 0b00, true))  // CMP.F r0, r1
        .emit16(brk_s())
        .run(10);

    assert!(!cpu.state.flag_z);
    assert!(!cpu.state.flag_c);  // borrow (3 < 10)
    assert!(cpu.state.flag_n);   // result is negative
}

#[test]
fn test_and_or_xor() {
    // AND, OR, XOR
    let cpu = Program::new()
        .emit16(mov_s(1, 0xFF))
        .emit16(mov_s(2, 0x0F))
        .emit32(alu32(0x04, 1, 2, 0, 0b00, false))  // AND r0, r1, r2 → 0x0F
        .emit32(alu32(0x05, 1, 2, 3, 0b00, false))  // OR r3, r1, r2  → 0xFF
        // XOR needs r12-r15 in 16-bit or use 32-bit MOV for higher regs
        // Store XOR result in r0 (overwrite)
        .emit32(alu32(0x07, 1, 2, 0, 0b00, false))  // XOR r0, r1, r2 → 0xF0
        .emit16(brk_s())
        .run(20);

    // After AND r0=0x0F is overwritten by XOR r0=0xF0
    assert_eq!(cpu.state.core_regs[0], 0xF0);
    assert_eq!(cpu.state.core_regs[3], 0xFF);
}

#[test]
fn test_mov_u6() {
    // MOV r0, #42 (P=01, sub=0x0A)
    let cpu = Program::new()
        .emit32(alu32(0x0A, 0, 42, 0, 0b01, false))  // MOV r0, #42
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 42);
}

#[test]
fn test_branch_unconditional() {
    // B_S +6 should skip the next 2-byte instruction
    // At address 0x02, PCL = 0x00, so target = 0x00 + 6 = 0x06
    let cpu = Program::new()
        .emit16(mov_s(0, 10))     // 0x00: r0 = 10
        .emit16(b_s(6))           // 0x02: B_S +6 → PCL=0x00, target 0x06
        .emit16(mov_s(0, 99))     // 0x04: r0 = 99 (SKIPPED)
        .emit16(brk_s())          // 0x06: halt
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 10); // should NOT be 99
    assert!(cpu.state.halted);
}

#[test]
fn test_branch_conditional_taken() {
    // CMP.F r0, r1 (equal) then BEQ_S (should branch)
    let cpu = Program::new()
        .emit16(mov_s(0, 5))               // 0x00: r0 = 5
        .emit16(mov_s(1, 5))               // 0x02: r1 = 5
        .emit32(alu32(0x0C, 0, 1, 0, 0b00, true))  // 0x04: CMP.F r0, r1 → Z=1
        .emit16(beq_s(4))                  // 0x08: BEQ_S +4 → target 0x0C
        .emit16(mov_s(0, 99))              // 0x0A: r0 = 99 (SKIPPED)
        .emit16(brk_s())                   // 0x0C: halt
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 5);
    assert!(cpu.state.halted);
}

#[test]
fn test_branch_conditional_not_taken() {
    // CMP.F r0, r1 (not equal) then BEQ_S (should NOT branch)
    let cpu = Program::new()
        .emit16(mov_s(0, 5))
        .emit16(mov_s(1, 10))
        .emit32(alu32(0x0C, 0, 1, 0, 0b00, true))  // CMP.F r0, r1 → Z=0
        .emit16(beq_s(4))                  // BEQ_S not taken
        .emit16(mov_s(0, 99))              // r0 = 99 (EXECUTED)
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 99);
}

#[test]
fn test_store_and_load_word() {
    // Store a value to memory, load it back
    let cpu = Program::new()
        .emit16(mov_s(1, 0x80))              // 0x00: r1 = 0x80 (base addr)
        .emit16(mov_s(2, 0x42))              // 0x02: r2 = 0x42
        .emit32(st32(2, 1, 0, 0))           // 0x04: ST r2, [r1, 0] (word)
        .emit32(ld32(0, 1, 0, 0, false))    // 0x08: LD r0, [r1, 0] (word)
        .emit16(brk_s())                     // 0x0C: halt
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 0x42);
}

#[test]
fn test_store_and_load_byte() {
    let cpu = Program::new()
        .emit16(mov_s(1, 0x80))              // r1 = 0x80
        .emit16(mov_s(2, 0xAB))             // r2 = 0xAB
        .emit32(st32(2, 1, 0, 1))           // STB r2, [r1, 0] (byte, zz=1)
        .emit32(ld32(0, 1, 0, 1, false))    // LDB r0, [r1, 0] (byte, zero-extend)
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 0xAB);
}

#[test]
fn test_load_byte_sign_extend() {
    let cpu = Program::new()
        .emit16(mov_s(1, 0x80))             // r1 = 0x80
        .emit16(mov_s(2, 0x80))             // r2 = 0x80 (negative if sign-extended as byte)
        .emit32(st32(2, 1, 0, 1))           // STB r2, [r1, 0]
        .emit32(ld32(0, 1, 0, 1, true))     // LDB.X r0, [r1, 0] (sign-extend)
        .emit16(brk_s())
        .run(10);

    // 0x80 sign-extended from byte = 0xFFFFFF80
    assert_eq!(cpu.state.core_regs[0], 0xFFFFFF80);
}

#[test]
fn test_store_load_with_offset() {
    let cpu = Program::new()
        .emit16(mov_s(1, 0x80))
        .emit16(mov_s(2, 0x55))
        .emit32(st32(2, 1, 4, 0))           // ST r2, [r1, 4]
        .emit32(ld32(0, 1, 4, 0, false))    // LD r0, [r1, 4]
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 0x55);
}

#[test]
fn test_zero_overhead_loop() {
    // MOV_S r0, 0
    // MOV r60, 5        (LP_COUNT = 5)
    // LP 8              (LP_END = LP_PC + 8)
    // ADD r0, r0, #1    (loop body, 4 bytes)
    // BRK_S             (at LP_END address)
    let mov_lp_count = alu32(0x0A, 60, 5, 0, 0b01, false); // MOV r60, #5
    let lp = lp32(8);                                         // LP 8
    let add_r0_1 = alu32(0x00, 0, 1, 0, 0b01, false);       // ADD r0, r0, #1

    let cpu = Program::new()
        .emit16(mov_s(0, 0))     // 0x00: r0 = 0
        .emit32(mov_lp_count)    // 0x02: r60 = LP_COUNT = 5
        .emit32(lp)              // 0x06: LP 8 → LP_START=0x0A, LP_END=0x0E
        .emit32(add_r0_1)        // 0x0A: r0 += 1  (loop body)
        .emit16(brk_s())         // 0x0E: halt (at LP_END)
        .run(100);

    assert!(cpu.state.halted);
    assert_eq!(cpu.state.core_regs[0], 5); // loop ran 5 times
    assert_eq!(cpu.state.core_regs[60], 0); // LP_COUNT decremented to 0
}

#[test]
fn test_multiply() {
    // MPY r0, r1, r2 (sub=0x1A)
    let cpu = Program::new()
        .emit16(mov_s(1, 7))
        .emit16(mov_s(2, 6))
        .emit32(alu32(0x1A, 1, 2, 0, 0b00, false))  // MPY r0, r1, r2
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 42); // 7 * 6 = 42
}

#[test]
fn test_bic_bset_bclr() {
    // BIC: a & !b
    let cpu = Program::new()
        .emit16(mov_s(1, 0xFF))
        .emit16(mov_s(2, 0x0F))
        .emit32(alu32(0x06, 1, 2, 0, 0b00, false))  // BIC r0, r1, r2 → 0xFF & !0x0F = 0xF0
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 0xF0);
}

#[test]
fn test_add_u6_immediate() {
    // ADD r0, r1, #15 (P=01)
    let cpu = Program::new()
        .emit16(mov_s(1, 100))
        .emit32(alu32(0x00, 1, 15, 0, 0b01, false))  // ADD r0, r1, #15
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 115);
}

#[test]
fn test_mixed_16_32_execution() {
    // Interleave 16-bit and 32-bit instructions
    let cpu = Program::new()
        .emit16(mov_s(0, 1))                          // 16-bit
        .emit32(alu32(0x00, 0, 10, 0, 0b01, false))  // 32-bit ADD r0, r0, #10
        .emit16(mov_s(1, 5))                           // 16-bit
        .emit32(alu32(0x00, 0, 1, 0, 0b00, false))   // 32-bit ADD r0, r0, r1
        .emit16(brk_s())
        .run(20);

    assert_eq!(cpu.state.core_regs[0], 16); // 1 + 10 + 5 = 16
}

/// 16-bit major 0x17: shift/sub/bit B,B,u5
/// Format: [15:11]=10111, [10:8]=b_enc, [7:5]=i, [4:0]=u5
fn shift_u5_16(b_enc: u8, i: u8, u5: u8) -> u16 {
    (0x17u16 << 11) | ((b_enc as u16 & 7) << 8) | ((i as u16 & 7) << 5) | (u5 as u16 & 0x1F)
}

/// 16-bit major 0x0F general ops: OP_S b,b,c
/// Format: [15:11]=01111, [10:8]=b_enc, [7:5]=c_enc, [4:0]=i
fn gen_op_16(b_enc: u8, c_enc: u8, i: u8) -> u16 {
    (0x0Fu16 << 11) | ((b_enc as u16 & 7) << 8) | ((c_enc as u16 & 7) << 5) | (i as u16 & 0x1F)
}

/// 16-bit major 0x0D: ADD_S/SUB_S/ASL_S/ASR_S C,B,u3
/// Format: [15:11]=01101, [10:8]=b_enc, [7:5]=c_enc, [4:3]=i, [2:0]=u3
fn arith_u3_16(b_enc: u8, c_enc: u8, i: u8, u3: u8) -> u16 {
    (0x0Du16 << 11) | ((b_enc as u16 & 7) << 8) | ((c_enc as u16 & 7) << 5)
        | ((i as u16 & 3) << 3) | (u3 as u16 & 7)
}

#[test]
fn test_asl_s_u5() {
    // ASL_S r0, r0, #3 → r0 = 5 << 3 = 40
    let cpu = Program::new()
        .emit16(mov_s(0, 5))
        .emit16(shift_u5_16(0, 0x00, 3))  // ASL_S r0, r0, #3
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 40);
}

#[test]
fn test_lsr_s_u5() {
    // LSR_S r0, r0, #2 → r0 = 0x80 >> 2 = 0x20
    let cpu = Program::new()
        .emit16(mov_s(0, 0x80))
        .emit16(shift_u5_16(0, 0x01, 2))  // LSR_S r0, r0, #2
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 0x20);
}

#[test]
fn test_asr_s_u5() {
    // ASR_S r0, r0, #4 → 128 >> 4 = 8
    let cpu = Program::new()
        .emit16(mov_s(0, 0x80))            // r0 = 128
        .emit16(shift_u5_16(0, 0x02, 4))   // ASR_S r0, r0, #4
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 8);
}

#[test]
fn test_asl_s_reg_reg() {
    // ASL_S r0, r0, r1 (major 0x0F, i=0x18)
    // r0 = 3, r1 = 4 → r0 = 3 << 4 = 48
    let cpu = Program::new()
        .emit16(mov_s(0, 3))
        .emit16(mov_s(1, 4))
        .emit16(gen_op_16(0, 1, 0x18))   // ASL_S r0, r0, r1
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 48);
}

#[test]
fn test_lsr_s_reg_reg() {
    // LSR_S r0, r0, r1 (major 0x0F, i=0x19)
    // r0 = 0x80, r1 = 3 → r0 = 0x80 >> 3 = 0x10
    let cpu = Program::new()
        .emit16(mov_s(0, 0x80))
        .emit16(mov_s(1, 3))
        .emit16(gen_op_16(0, 1, 0x19))   // LSR_S r0, r0, r1
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 0x10);
}

#[test]
fn test_asr_s_reg_reg() {
    // ASR_S r0, r0, r1 (major 0x0F, i=0x1A)
    // r0 = 0x40, r1 = 2 → r0 = 0x40 >> 2 = 0x10
    let cpu = Program::new()
        .emit16(mov_s(0, 0x40))
        .emit16(mov_s(1, 2))
        .emit16(gen_op_16(0, 1, 0x1A))   // ASR_S r0, r0, r1
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[0], 0x10);
}

#[test]
fn test_asl_s_u3() {
    // ASL_S r1, r0, #2 (major 0x0D, i=2)
    // r0 = 7 → r1 = 7 << 2 = 28
    let cpu = Program::new()
        .emit16(mov_s(0, 7))
        .emit16(arith_u3_16(0, 1, 2, 2))  // ASL_S r1, r0, #2
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[1], 28);
}

#[test]
fn test_asr_s_u3() {
    // ASR_S r1, r0, #1 (major 0x0D, i=3)
    // r0 = 16 → r1 = 16 >> 1 = 8
    let cpu = Program::new()
        .emit16(mov_s(0, 16))
        .emit16(arith_u3_16(0, 1, 3, 1))  // ASR_S r1, r0, #1
        .emit16(brk_s())
        .run(10);

    assert_eq!(cpu.state.core_regs[1], 8);
}
