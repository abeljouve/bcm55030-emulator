//! ARCompact decoder: raw bytes to `DecodedInstruction`, through any
//! `InstructionFetch` source.

pub mod decode16;
pub mod decode32;
pub mod fields;
pub mod format;
pub mod instruction;

use crate::exception::Exception;
use fields::{is_32bit_instruction, major_opcode};
use instruction::DecodedInstruction;

pub trait InstructionFetch {
    fn fetch_half(&self, addr: u32) -> Result<u16, Exception>;
    fn fetch_word(&self, addr: u32) -> Result<u32, Exception>;
}

pub struct ByteSliceFetch<'a> {
    pub bytes: &'a [u8],
    pub base: u32,
}

impl<'a> ByteSliceFetch<'a> {
    pub fn new(bytes: &'a [u8], base: u32) -> Self {
        Self { bytes, base }
    }
}

impl<'a> InstructionFetch for ByteSliceFetch<'a> {
    fn fetch_half(&self, addr: u32) -> Result<u16, Exception> {
        if addr & 1 != 0 {
            return Err(Exception::MisalignedAccess { address: addr });
        }
        let off = addr
            .checked_sub(self.base)
            .ok_or(Exception::MemoryError { address: addr, is_write: false })?
            as usize;
        if off + 1 >= self.bytes.len() {
            return Err(Exception::MemoryError { address: addr, is_write: false });
        }
        Ok(((self.bytes[off] as u16) << 8) | (self.bytes[off + 1] as u16))
    }

    fn fetch_word(&self, addr: u32) -> Result<u32, Exception> {
        if addr & 1 != 0 {
            return Err(Exception::MisalignedAccess { address: addr });
        }
        let off = addr
            .checked_sub(self.base)
            .ok_or(Exception::MemoryError { address: addr, is_write: false })?
            as usize;
        if off + 3 >= self.bytes.len() {
            return Err(Exception::MemoryError { address: addr, is_write: false });
        }
        Ok(((self.bytes[off] as u32) << 24)
            | ((self.bytes[off + 1] as u32) << 16)
            | ((self.bytes[off + 2] as u32) << 8)
            | (self.bytes[off + 3] as u32))
    }
}

/// Decode the instruction at `pc` from a raw byte slice rooted at
/// `base`. Used by the UI / MCP disassembler that cannot reach into
/// `Memory`.
pub fn decode_bytes(
    pc: u32,
    bytes: &[u8],
    base: u32,
) -> Result<DecodedInstruction, Exception> {
    let fetch = ByteSliceFetch::new(bytes, base);
    decode_with(pc, &fetch)
}

/// Decode the instruction at `pc` from any fetch source.
#[inline]
pub fn decode_with(pc: u32, fetch: &dyn InstructionFetch) -> Result<DecodedInstruction, Exception> {
    let first_half = fetch.fetch_half(pc)?;
    let major = major_opcode(first_half);

    if is_32bit_instruction(major) {
        let second_half = fetch.fetch_half(pc + 2)?;
        let word = ((first_half as u32) << 16) | (second_half as u32);
        decode32::decode_32bit(word, pc, fetch)
    } else {
        decode16::decode_16bit(first_half, pc, fetch)
    }
}
