//! Instruction semantics, generic over a [`crate::dom::Dom`].

pub mod alu;
pub mod branch;
pub mod extended;
pub mod regs;
pub mod single_op;
pub mod special;
