//! The concrete (`u32`/`bool`) primitives, shared by every concrete domain so
//! that two of them cannot disagree on an operator.

/// Implements the primitive part of [`crate::dom::Dom`] for `V = u32`,
/// `B = bool`, `as_const` and the guarded effects. Expand inside the
/// `impl Dom for ... { ... }` block.
#[macro_export]
macro_rules! concrete_primitives {
    () => {
        type V = u32;
        type B = bool;

        #[inline(always)]
        fn k(&mut self, v: u32) -> u32 {
            v
        }
        #[inline(always)]
        fn kb(&mut self, b: bool) -> bool {
            b
        }
        #[inline(always)]
        fn add(&mut self, a: u32, b: u32) -> u32 {
            a.wrapping_add(b)
        }
        #[inline(always)]
        fn sub(&mut self, a: u32, b: u32) -> u32 {
            a.wrapping_sub(b)
        }
        #[inline(always)]
        fn and(&mut self, a: u32, b: u32) -> u32 {
            a & b
        }
        #[inline(always)]
        fn or(&mut self, a: u32, b: u32) -> u32 {
            a | b
        }
        #[inline(always)]
        fn xor(&mut self, a: u32, b: u32) -> u32 {
            a ^ b
        }
        #[inline(always)]
        fn not(&mut self, a: u32) -> u32 {
            !a
        }
        #[inline(always)]
        fn neg(&mut self, a: u32) -> u32 {
            a.wrapping_neg()
        }
        #[inline(always)]
        fn shl(&mut self, a: u32, n: u32) -> u32 {
            a.checked_shl(n).unwrap_or(0)
        }
        #[inline(always)]
        fn lshr(&mut self, a: u32, n: u32) -> u32 {
            a.checked_shr(n).unwrap_or(0)
        }
        #[inline(always)]
        fn ashr(&mut self, a: u32, n: u32) -> u32 {
            ((a as i32) >> n.min(31)) as u32
        }
        #[inline(always)]
        fn eq(&mut self, a: u32, b: u32) -> bool {
            a == b
        }
        #[inline(always)]
        fn ult(&mut self, a: u32, b: u32) -> bool {
            a < b
        }
        #[inline(always)]
        fn slt(&mut self, a: u32, b: u32) -> bool {
            (a as i32) < (b as i32)
        }
        #[inline(always)]
        fn ite(&mut self, c: bool, a: u32, b: u32) -> u32 {
            if c { a } else { b }
        }
        #[inline(always)]
        fn ite_b(&mut self, c: bool, a: bool, b: bool) -> bool {
            if c { a } else { b }
        }
        #[inline(always)]
        fn band(&mut self, a: bool, b: bool) -> bool {
            a & b
        }
        #[inline(always)]
        fn bor(&mut self, a: bool, b: bool) -> bool {
            a | b
        }
        #[inline(always)]
        fn bxor(&mut self, a: bool, b: bool) -> bool {
            a ^ b
        }
        #[inline(always)]
        fn bnot(&mut self, a: bool) -> bool {
            !a
        }
        #[inline(always)]
        fn bit(&mut self, a: u32, i: u8) -> bool {
            (a >> i) & 1 != 0
        }
        #[inline(always)]
        fn b2v(&mut self, b: bool) -> u32 {
            b as u32
        }
        #[inline(always)]
        fn sext(&mut self, a: u32, bits: u8) -> u32 {
            let s = 32 - bits as u32;
            (((a << s) as i32) >> s) as u32
        }
        #[inline(always)]
        fn zext(&mut self, a: u32, bits: u8) -> u32 {
            a & (u32::MAX >> (32 - bits as u32))
        }
        #[inline(always)]
        fn clz(&mut self, a: u32) -> u32 {
            a.leading_zeros()
        }
        #[inline(always)]
        fn as_const(&self, v: u32) -> Option<u32> {
            Some(v)
        }
        #[inline(always)]
        fn when(
            &mut self,
            g: bool,
            body: impl FnOnce(&mut Self) -> Result<(), $crate::exception::Exception>,
        ) -> Result<(), $crate::exception::Exception> {
            if g { body(self) } else { Ok(()) }
        }
        #[inline(always)]
        fn fault_if(
            &mut self,
            c: bool,
            e: $crate::exception::Exception,
        ) -> Result<(), $crate::exception::Exception> {
            if c { Err(e) } else { Ok(()) }
        }
    };
}
