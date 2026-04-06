pub mod decode16;
pub mod decode32;
pub mod fields;
pub mod instruction;

use crate::cpu::exception::Exception;
use crate::memory::Memory;
use fields::{is_32bit_instruction, major_opcode};
use instruction::DecodedInstruction;

pub fn decode(pc: u32, mem: &Memory) -> Result<DecodedInstruction, Exception> {
    let first_half = mem.fetch_half(pc)?;
    let major = major_opcode(first_half);

    if is_32bit_instruction(major) {
        let second_half = mem.fetch_half(pc + 2)?;
        let word = ((first_half as u32) << 16) | (second_half as u32);
        decode32::decode_32bit(word, pc, mem)
    } else {
        decode16::decode_16bit(first_half, pc, mem)
    }
}
