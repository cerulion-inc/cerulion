// SPDX-License-Identifier: MIT OR Apache-2.0
//! The single canonical signing-byte encoder.
//!
//! We hand-roll ONE deterministic encoding for signing (rather than reuse the
//! serde/postcard container format) for three reasons:
//!
//! 1. **Stability for a firmware signing format.** A hand-rolled, explicit byte
//!    layout has no hidden coupling to a serde field order or a postcard version
//!    — it is what the bytes literally are, forever, pinned by byte-exact test
//!    vectors.
//! 2. **Explicit domain separation.** Each signable type prepends a distinct,
//!    length-prefixed domain tag, so a signature over a device cert can never be
//!    replayed as a grant (or vice versa).
//! 3. **Unambiguous parsing.** Every variable-length field is length-prefixed
//!    (u32-LE), so no two distinct field tuples can ever produce the same bytes.
//!
//! Fixed-width, structurally-known fields (32-byte keys, u8/u16/u64 scalars) are
//! written raw in a fixed order; variable-length fields are length-prefixed.

use super::{AccountId, PrincipalKind, PublicKey, RobotId, Scope, Validity};
use crate::error::PairingError;

/// Panic message used when a canonical length overflows `u32` (see
/// [`checked_len_u32`]). This is a should-never-happen invariant: no signable
/// type has a `> 4 GiB` field (keys are 32 bytes, domains are short constants,
/// and the only length-prefixed collection is a bounded account list).
const OVERFLOW_MSG: &str =
    "CanonicalWriter: field length exceeds u32::MAX (canonical-encoding domain-confusion guard)";

/// Convert a `usize` length to `u32`, rejecting (rather than silently
/// truncating) a value above `u32::MAX`. A truncated length prefix in signing
/// bytes is a domain-confusion risk (two distinct inputs → identical encoding),
/// so this returns an error at the boundary.
pub(crate) fn checked_len_u32(len: usize) -> Result<u32, PairingError> {
    u32::try_from(len).map_err(|_| PairingError::CanonicalOverflow)
}

/// Appends canonical signing bytes to an internal buffer. Every method returns
/// `&mut Self` for chaining.
pub struct CanonicalWriter {
    buf: Vec<u8>,
}

impl CanonicalWriter {
    /// Start a new signing payload for the given domain tag. The tag is written
    /// length-prefixed as the very first bytes.
    pub fn new(domain: &[u8]) -> Self {
        let mut w = CanonicalWriter { buf: Vec::new() };
        w.bytes(domain);
        w
    }

    /// One raw byte.
    pub fn u8(&mut self, v: u8) -> &mut Self {
        self.buf.push(v);
        self
    }

    /// A u16, little-endian.
    pub fn u16(&mut self, v: u16) -> &mut Self {
        self.buf.extend_from_slice(&v.to_le_bytes());
        self
    }

    /// A u64, little-endian.
    pub fn u64(&mut self, v: u64) -> &mut Self {
        self.buf.extend_from_slice(&v.to_le_bytes());
        self
    }

    /// Fixed-length bytes written raw (no length prefix). Use only for fields
    /// whose length is structurally known (e.g. 32-byte keys).
    pub fn fixed(&mut self, b: &[u8]) -> &mut Self {
        self.buf.extend_from_slice(b);
        self
    }

    /// Variable-length bytes written as `u32-LE length || bytes`. Panics loudly
    /// (never silently truncates) if `b.len() > u32::MAX` (a should-never-happen
    /// invariant — the length prefix would otherwise be corrupted).
    pub fn bytes(&mut self, b: &[u8]) -> &mut Self {
        let len = checked_len_u32(b.len()).expect(OVERFLOW_MSG);
        self.buf.extend_from_slice(&len.to_le_bytes());
        self.buf.extend_from_slice(b);
        self
    }

    /// A `u32-LE` count prefix (for collections written element-by-element).
    /// Panics loudly (never silently truncates) if `n > u32::MAX`.
    pub fn count(&mut self, n: usize) -> &mut Self {
        let len = checked_len_u32(n).expect(OVERFLOW_MSG);
        self.buf.extend_from_slice(&len.to_le_bytes());
        self
    }

    // -- domain-specific field writers (keep every payload consistent) --------

    /// A 32-byte public key, raw.
    pub fn key(&mut self, k: &PublicKey) -> &mut Self {
        self.fixed(&k.0)
    }

    /// A 32-byte account id, raw.
    pub fn account(&mut self, a: &AccountId) -> &mut Self {
        self.fixed(&a.0)
    }

    /// A 32-byte robot id, raw.
    pub fn robot(&mut self, r: &RobotId) -> &mut Self {
        self.fixed(&r.0)
    }

    /// A validity window: `not_before_ns || not_after_ns` (both u64-LE).
    pub fn validity(&mut self, v: &Validity) -> &mut Self {
        self.u64(v.not_before_ns).u64(v.not_after_ns)
    }

    /// A scope: `role (u16-LE) || caps (u64-LE)`.
    pub fn scope(&mut self, s: &Scope) -> &mut Self {
        self.u16(s.role.0).u64(s.caps)
    }

    /// A principal kind, as its u8 discriminant.
    pub fn principal(&mut self, p: PrincipalKind) -> &mut Self {
        self.u8(p as u8)
    }

    /// Consume the writer and return the accumulated signing bytes.
    pub fn finish(self) -> Vec<u8> {
        self.buf
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Independent oracle: build the same bytes by hand and confirm the writer's
    // primitives lay out exactly as documented (NOT a self-compare of the
    // production encoders — those are pinned in tests/format_vectors_test.rs).
    #[test]
    fn primitives_lay_out_as_documented() {
        let mut w = CanonicalWriter::new(b"dom");
        w.u8(0xAB)
            .u16(0x1234)
            .u64(0x0102_0304_0506_0708)
            .bytes(&[0xDE, 0xAD])
            .fixed(&[0xBE, 0xEF]);
        let got = w.finish();

        let mut want = Vec::new();
        want.extend_from_slice(&3u32.to_le_bytes()); // domain length
        want.extend_from_slice(b"dom"); // domain
        want.push(0xAB); // u8
        want.extend_from_slice(&0x1234u16.to_le_bytes()); // u16 LE
        want.extend_from_slice(&0x0102_0304_0506_0708u64.to_le_bytes()); // u64 LE
        want.extend_from_slice(&2u32.to_le_bytes()); // bytes length
        want.extend_from_slice(&[0xDE, 0xAD]); // bytes
        want.extend_from_slice(&[0xBE, 0xEF]); // fixed (no prefix)

        assert_eq!(got, want);
    }

    #[test]
    fn count_is_u32_le() {
        let mut w = CanonicalWriter::new(b"");
        w.count(5);
        // domain "" => length prefix 0, then count 5.
        assert_eq!(w.finish(), [0, 0, 0, 0, 5, 0, 0, 0]);
    }

    #[test]
    fn checked_len_u32_rejects_overflow_at_the_boundary() {
        // In-range values pass through unchanged.
        assert_eq!(checked_len_u32(0).unwrap(), 0);
        assert_eq!(checked_len_u32(u32::MAX as usize).unwrap(), u32::MAX);
        // Just over u32::MAX is rejected (no silent truncation). Constructible
        // without allocating: we feed the length value directly. Only meaningful
        // where usize is wider than u32 (64-bit); on 32-bit usize == u32 so the
        // boundary is unrepresentable and the check is vacuously correct.
        #[cfg(target_pointer_width = "64")]
        {
            assert!(matches!(
                checked_len_u32(u32::MAX as usize + 1),
                Err(PairingError::CanonicalOverflow)
            ));
        }
    }
}
