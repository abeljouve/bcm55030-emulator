//! ARCompact instruction set of the BCM55030 core: decoder, formatter,
//! condition codes, exceptions, architectural register numbers, and the
//! instruction semantics written once over a value domain ([`dom::Dom`]).
//!
//! No dependencies, tests included.

pub mod concrete;
pub mod condition;
pub mod decoder;
pub mod dom;
pub mod exception;
pub mod flat;
pub mod regs;
pub mod sem;
