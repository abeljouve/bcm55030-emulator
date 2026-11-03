//! The decoder lives in `arc_isa::decoder`; it is re-exported here under its
//! historical paths. What stays is the fetch path from the live `Memory`.

pub use arc_isa::decoder::{
    decode16, decode32, decode_bytes, decode_with, fields, format, instruction, ByteSliceFetch,
    InstructionFetch,
};

use crate::cpu::exception::Exception;
use crate::memory::Memory;
use instruction::DecodedInstruction;

impl InstructionFetch for Memory {
    #[inline]
    fn fetch_half(&self, addr: u32) -> Result<u16, Exception> {
        Memory::fetch_half(self, addr)
    }
    #[inline]
    fn fetch_word(&self, addr: u32) -> Result<u32, Exception> {
        Memory::fetch_word(self, addr)
    }
}

/// Zero-cost wrapper that serves `fetch_half` / `fetch_word` out of
/// a borrowed byte slice. Big-endian (MSB first), matching the ARC
/// ARCompact fetch semantics. `base` is the virtual address of
/// `bytes[0]`.
/// Decode the instruction at `pc` from the live `Memory` (CPU path).
#[inline]
pub fn decode(pc: u32, mem: &Memory) -> Result<DecodedInstruction, Exception> {
    decode_with(pc, mem)
}
