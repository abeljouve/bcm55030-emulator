//! Instruction semantics, generic over a [`crate::dom::Dom`].

pub mod alu;
pub mod aux;
pub mod branch;
pub mod extended;
pub mod load_store;
pub mod regs;
pub mod single_op;
pub mod special;
