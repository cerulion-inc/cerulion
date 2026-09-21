// SPDX-License-Identifier: AGPL-3.0-only
//! The ONE implementation of the canonical Cerulion **message body**
//! encoding — `[fixed][offset table][variable payloads]`.
//!
//! # Why this module exists
//!
//! This body shape is written in three places and read in four, and until
//! this module existed it had **two independent hand-written implementations**: the
//! schema-driven [`CdrCodec`](super::CdrCodec) (the `ros2 attach` /
//! `dds_bridge` ingress path) and the rmw introspection bridge
//! (`rmw_cerulion::type_bridge` + its `type_bridge_cpp` twin). They agreed at
//! the top level — both build it from the same [`WireLayout`] — and they
//! agreed on the array framing for `Nested[]` / `string[]`. They disagreed on
//! the **body of a variable-nested array element**, where rmw wrote a bespoke
//! `[packed fixed][u32 count-of-variable-members][per var: u32 len + payload]`
//! with no offset table.
//!
//! That divergence is invisible to rmw's own tests, which round-trip the
//! bridge against ITSELF: a symmetric encoder+decoder pair is self-consistent
//! no matter what convention it invents. Without one shared codec the
//! consequence is user-visible — `nav_msgs/Path`, `tf2_msgs/TFMessage` and
//! every `Detection*Array` published by an rmw robot stay
//! [`NestedArrayOpaque`](super::FrameValueKind::NestedArrayOpaque) to the
//! [`FrameWalker`](super::FrameWalker), so viz draws a text dump instead of a
//! polyline.
//!
//! Fixing rmw's bytes alone would have left the duplication — and the same
//! drift free to recur. So the convention now lives HERE, once, and every
//! producer that ASSEMBLES a body calls it: [`CdrCodec`](super::CdrCodec) and
//! both rmw introspection bridges.
//!
//! # The one producer that does NOT call it, and why
//!
//! The generated SHM writers (`codegen::generator::structs`) implement the
//! offset-table and payload-alignment rule INDEPENDENTLY, and deliberately so:
//! they write in place into a loaned shared-memory slot, so there is no
//! `Vec<u8>` to assemble and nothing for [`CanonicalBodyBuilder`] to hand back
//! (Principle #10). The design is fine; the DUPLICATION is the hazard. Anyone
//! editing [`variable_payload_align`], the entry width, or the payload
//! placement rule here must make the same edit in `generator/structs.rs`, or
//! the native writers silently skew from every other producer — the original
//! failure mode, one file over. (The end-to-end guard against that skew is
//! `frame_walker_production_test`, which walks frames the REAL generated writer
//! produced.)
//!
//! # The encoding
//!
//! ```text
//! [ fixed section, repr(C) padded, size = layout.fixed_size ]
//! [ offset table: N entries × (u32 offset, u32 length) ]      ← 8N bytes
//! [ variable payloads at the recorded offsets ]
//! ```
//!
//! Offsets are relative to the START of the body. Entry `i` corresponds to
//! `layout.variable_fields[i]` (declaration order). Each payload is aligned to
//! [`variable_payload_align`] of its field type before being placed, so a
//! `float64[]` payload lands 8-byte aligned and can be read in place.
//!
//! This is EXACTLY the shape of a top-level frame payload, which is what makes
//! the walker's recursion free: a variable-nested field's payload is a
//! *headerless sub-frame*, decoded by the same code that decodes the frame
//! containing it.
//!
//! # Relationship to the `FrameWalker`
//!
//! The walker is deliberately NOT built on this module. It stays an
//! independent reader with its own degrade discipline (non-canonical bytes
//! become `NestedArrayOpaque`, never a guessed element list). That
//! independence is worth keeping: it makes "encode with this module, decode
//! with the walker" a genuine cross-check rather than a self-compare, which is
//! precisely the test that did not exist and would have caught that skew on day
//! one.
//!
//! Two consequences of the walker's `PayloadAudit::Element` rules that this
//! writer must respect, and does:
//!
//! - alignment padding between variable payloads is tolerated only up to
//!   `MAX_ELEMENT_VAR_PAD` (7 bytes) — satisfied because the widest alignment
//!   [`variable_payload_align`] returns is 8;
//! - an element body must account for every one of its bytes, so payloads are
//!   emitted in table order with no gaps beyond that alignment padding.

use super::layout::WireLayout;
use super::schema::FieldType;
use crate::shm_runtime::{read_offset_entry, write_offset_entry};

/// Hard ceiling on one encoded body (1 GiB). A corrupt or hostile length must
/// produce an [`ElementBodyError`], never an OOM abort. Mirrors
/// `CdrCodec::MAX_CDR_FRAME_BYTES` and
/// `rmw_cerulion::type_bridge::MAX_FRAME_BYTES`.
pub const MAX_BODY_BYTES: usize = 1 << 30;

/// A failure building or reading a canonical body.
///
/// Every variant maps to a `&'static str` via [`ElementBodyError::as_str`] so
/// the rmw bridges (whose internal codec returns `Result<_, &'static str>`)
/// can propagate it without allocating.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum ElementBodyError {
    /// The caller pushed a different number of variable payloads than the
    /// layout declares. A programming error in the caller's field walk, not
    /// bad input — the two MUST stay in lockstep or every offset is wrong.
    #[error("variable-payload count does not match the layout's variable-field count")]
    VariableCountMismatch,
    /// The body would exceed [`MAX_BODY_BYTES`], or an offset/length does not
    /// fit in the `u32` the offset table stores.
    #[error("encoded body exceeds the size cap or overflows a u32 offset/length")]
    BodyTooLarge,
    /// The body is shorter than `fixed_size + 8 * variable_field_count`, so it
    /// cannot even contain its own offset table.
    #[error("body is too short to contain its fixed section and offset table")]
    BodyTruncated,
    /// An offset-table entry points outside the body, overflows, or points
    /// below the data floor (into the fixed section or the table itself).
    #[error("offset-table entry is out of bounds for the body")]
    EntryOutOfBounds,
}

impl ElementBodyError {
    /// Static description, for callers that propagate `&'static str`.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::VariableCountMismatch => {
                "variable-payload count does not match the layout's variable-field count"
            }
            Self::BodyTooLarge => {
                "encoded body exceeds the size cap or overflows a u32 offset/length"
            }
            Self::BodyTruncated => {
                "body is too short to contain its fixed section and offset table"
            }
            Self::EntryOutOfBounds => "offset-table entry is out of bounds for the body",
        }
    }
}

/// Alignment a variable field's payload is placed at within the body.
///
/// Primitive arrays align to their ELEMENT size so a reader can view the
/// payload as `&[T]` in place; everything else (strings, byte blobs, nested
/// sub-frames, counted element arrays) is byte-aligned.
///
/// The widest value is 8, which is what bounds genuine inter-payload padding
/// at 7 bytes — the constant the `FrameWalker`'s element audit tolerates.
pub fn variable_payload_align(ft: &FieldType) -> usize {
    match ft {
        FieldType::DynamicArray { element_type } => match element_type.as_ref() {
            FieldType::I16 | FieldType::U16 => 2,
            FieldType::I32 | FieldType::U32 | FieldType::F32 => 4,
            FieldType::I64 | FieldType::U64 | FieldType::F64 => 8,
            _ => 1,
        },
        _ => 1,
    }
}

/// Assembles one canonical body: write the fixed section through
/// [`fixed_mut`](Self::fixed_mut), push each variable payload in DECLARATION
/// order, then [`finish`](Self::finish).
///
/// The fixed section starts fully zeroed, which is also the correct value for
/// `repr(C)` padding bytes — deterministic frames (Principle #7) require them
/// zeroed rather than left as whatever the source struct held.
pub struct CanonicalBodyBuilder<'a> {
    layout: &'a WireLayout,
    fixed: Vec<u8>,
    vars: Vec<Vec<u8>>,
}

impl<'a> CanonicalBodyBuilder<'a> {
    /// Start a body for `layout`, fixed section zeroed.
    pub fn new(layout: &'a WireLayout) -> Self {
        Self {
            layout,
            fixed: vec![0u8; layout.fixed_size],
            vars: Vec::with_capacity(layout.variable_fields.len()),
        }
    }

    /// The zeroed fixed section, to be written at
    /// `layout.fixed_fields[i].offset`. Exactly `layout.fixed_size` bytes.
    #[inline]
    pub fn fixed_mut(&mut self) -> &mut [u8] {
        &mut self.fixed
    }

    /// Append the next variable field's payload. Call once per entry in
    /// `layout.variable_fields`, in that order.
    pub fn push_variable(&mut self, bytes: Vec<u8>) -> Result<(), ElementBodyError> {
        if self.vars.len() >= self.layout.variable_fields.len() {
            return Err(ElementBodyError::VariableCountMismatch);
        }
        if bytes.len() > MAX_BODY_BYTES {
            return Err(ElementBodyError::BodyTooLarge);
        }
        self.vars.push(bytes);
        Ok(())
    }

    /// Number of variable payloads pushed so far — lets a caller assert its
    /// own field walk stayed in lockstep with the layout.
    #[inline]
    pub fn pushed(&self) -> usize {
        self.vars.len()
    }

    /// Assemble `[fixed][table][var]`.
    ///
    /// Errors if the pushed count does not match the layout (the lockstep
    /// invariant) or the result would overflow the size cap.
    pub fn finish(self) -> Result<Vec<u8>, ElementBodyError> {
        let n = self.layout.variable_fields.len();
        if self.vars.len() != n {
            return Err(ElementBodyError::VariableCountMismatch);
        }
        let fixed_size = self.layout.fixed_size;
        debug_assert_eq!(self.fixed.len(), fixed_size, "fixed section was resized");

        let mut body = self.fixed;
        // Reserve the offset table (zeroed); entries are back-patched below.
        body.resize(
            fixed_size
                .checked_add(
                    8usize
                        .checked_mul(n)
                        .ok_or(ElementBodyError::BodyTooLarge)?,
                )
                .ok_or(ElementBodyError::BodyTooLarge)?,
            0,
        );

        for (i, vfl) in self.layout.variable_fields.iter().enumerate() {
            let bytes = &self.vars[i];
            // Align the payload start (see `variable_payload_align`); the pad
            // bytes are zero, so the body stays byte-deterministic.
            let align = variable_payload_align(&vfl.field_type);
            while !body.len().is_multiple_of(align) {
                body.push(0);
            }
            let off = body.len();
            let len = bytes.len();
            if off > u32::MAX as usize
                || len > u32::MAX as usize
                || off.checked_add(len).is_none_or(|end| end > MAX_BODY_BYTES)
            {
                return Err(ElementBodyError::BodyTooLarge);
            }
            write_offset_entry(&mut body, fixed_size, i, off as u32, len as u32);
            body.extend_from_slice(bytes);
        }
        Ok(body)
    }
}

/// Reads one canonical body: bounds-checked access to the fixed section and to
/// each variable payload by declaration index.
///
/// Validation here is the BOUNDS contract (nothing may point outside the body
/// or below the data floor) — it deliberately does NOT require exact byte
/// accounting. Exactness is the [`FrameWalker`](super::FrameWalker)'s
/// discriminator for refusing a foreign element convention that happens to
/// look plausible; a decoder reading a peer it already trusts by schema hash
/// only needs the bounds to be sound.
#[derive(Debug)]
pub struct CanonicalBodyReader<'a> {
    layout: &'a WireLayout,
    body: &'a [u8],
}

impl<'a> CanonicalBodyReader<'a> {
    /// Validate `body`'s offset table against `layout`.
    ///
    /// EVERY entry must lie inside the body and at or above the data floor
    /// (`fixed_size + 8 * n`) — an entry pointing into the fixed section or
    /// the table itself is structurally impossible in a canonical body and is
    /// exactly what a foreign framing produces (an rmw-packed
    /// `std_msgs/Header` body puts the variable-member COUNT `1` where the
    /// offset belongs, which lands far below the floor).
    ///
    /// # `(0, 0)` is NOT carved out
    ///
    /// This reader used to accept a `(0, 0)` entry as the "unwritten field"
    /// idiom. It was deleted, for three compounding reasons:
    ///
    /// - **Unreachable from every canonical writer.** [`CanonicalBodyBuilder`]
    ///   places each payload at `body.len()`, which is never below the floor,
    ///   so it cannot emit `(0, 0)` even for an empty payload — it emits
    ///   `(floor, 0)`. Nothing that writes this encoding produces the bytes
    ///   the carve-out admitted.
    /// - **The idiom it named does not produce these bytes, and is at another
    ///   level anyway.** `set_<f>_bytes(&[])` on the generated SHM writers is
    ///   an EXPLICIT empty write: the setter records the running payload cursor,
    ///   so it emits `(cursor, 0)` — well-formed and in bounds — and marks the
    ///   field WRITTEN. `(0, 0)` is what an *untouched* table slot holds, and a
    ///   frame with one of those is refused at publish by the unwritten-field
    ///   gate, so it never reaches a reader at all. Separately, both are
    ///   TOP-LEVEL frame concerns, and a top-level frame is never read through
    ///   this type (rmw's `unflatten` has its own entry reader). At this
    ///   reader's two call sites — decoding a nested sub-frame or an array
    ///   element body — an empty top-level entry has already become an EMPTY
    ///   BODY, rejected by the `body.len() < floor` check above regardless of
    ///   what this loop does.
    /// - **It inverted the rule the reader-side states for the same bytes.**
    ///   The [`FrameWalker`](super::FrameWalker)'s element audit refuses a
    ///   `(0, 0)` pair precisely BECAUSE it sits below the data floor (it is
    ///   the discriminator that rejects a bespoke `/tf` blob whose entries land
    ///   in an identity quaternion's zero bytes). A writer-side reader that
    ///   accepted what the walker refuses is the divergence shape this
    ///   module exists to remove.
    pub fn new(layout: &'a WireLayout, body: &'a [u8]) -> Result<Self, ElementBodyError> {
        let n = layout.variable_fields.len();
        let floor = layout
            .fixed_size
            .checked_add(
                8usize
                    .checked_mul(n)
                    .ok_or(ElementBodyError::BodyTruncated)?,
            )
            .ok_or(ElementBodyError::BodyTruncated)?;
        if body.len() < floor {
            return Err(ElementBodyError::BodyTruncated);
        }
        for i in 0..n {
            let (off, len) = read_offset_entry(body, layout.fixed_size, i);
            let off = off as usize;
            let len = len as usize;
            if off < floor {
                return Err(ElementBodyError::EntryOutOfBounds);
            }
            match off.checked_add(len) {
                Some(end) if end <= body.len() => {}
                _ => return Err(ElementBodyError::EntryOutOfBounds),
            }
        }
        Ok(Self { layout, body })
    }

    /// The fixed section (`layout.fixed_size` bytes from the body start).
    #[inline]
    pub fn fixed(&self) -> &'a [u8] {
        &self.body[..self.layout.fixed_size]
    }

    /// Variable payload `idx` (declaration order). Returns `None` past the
    /// last variable field. Bounds were validated in [`Self::new`].
    pub fn variable(&self, idx: usize) -> Option<&'a [u8]> {
        if idx >= self.layout.variable_fields.len() {
            return None;
        }
        let (off, len) = read_offset_entry(self.body, self.layout.fixed_size, idx);
        Some(&self.body[off as usize..off as usize + len as usize])
    }

    /// Number of variable fields this body carries.
    #[inline]
    pub fn variable_count(&self) -> usize {
        self.layout.variable_fields.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codegen::layout::{FieldLayout, VariableFieldLayout};

    /// `std_msgs/Header`-shaped layout: fixed `{i32 sec; u32 nanosec}` = 8
    /// bytes, one variable field `frame_id`.
    fn header_layout() -> WireLayout {
        WireLayout {
            qualified_name: "std_msgs/Header".to_string(),
            schema_hash: 0xDEAD_BEEF,
            fixed_size: 8,
            fixed_align: 4,
            fixed_fields: vec![
                FieldLayout {
                    name: "sec".to_string(),
                    offset: 0,
                    size: 4,
                    align: 4,
                    field_type: FieldType::I32,
                },
                FieldLayout {
                    name: "nanosec".to_string(),
                    offset: 4,
                    size: 4,
                    align: 4,
                    field_type: FieldType::U32,
                },
            ],
            variable_fields: vec![VariableFieldLayout {
                name: "frame_id".to_string(),
                field_type: FieldType::String,
            }],
        }
    }

    /// THE original worked example, byte-for-byte against a HAND-WRITTEN
    /// oracle (never a self-compare): `Header { stamp: 0.0, frame_id: "odom" }`.
    ///
    /// Both the canonical and the old rmw-packed body are 20 bytes and differ
    /// in exactly one `u32` — canonical carries the offset `16`, rmw carried
    /// the variable-member count `1`. That is why "teach the decoder both" was
    /// rejected: there is no structural discriminator, only a value.
    #[test]
    fn header_worked_example_matches_the_hand_oracle_byte_for_byte() {
        let layout = header_layout();
        let mut b = CanonicalBodyBuilder::new(&layout);
        b.fixed_mut()[0..4].copy_from_slice(&0i32.to_le_bytes());
        b.fixed_mut()[4..8].copy_from_slice(&0u32.to_le_bytes());
        b.push_variable(b"odom".to_vec()).unwrap();
        let body = b.finish().unwrap();

        let mut oracle = Vec::new();
        oracle.extend_from_slice(&0i32.to_le_bytes()); // stamp.sec
        oracle.extend_from_slice(&0u32.to_le_bytes()); // stamp.nanosec
        oracle.extend_from_slice(&16u32.to_le_bytes()); // entry.offset = 8 + 8
        oracle.extend_from_slice(&4u32.to_le_bytes()); // entry.length
        oracle.extend_from_slice(b"odom");
        assert_eq!(body, oracle);
        assert_eq!(body.len(), 20);

        // The rmw-packed body of the same value: same length, one differing
        // u32. Byte-length identity is the whole reason this must converge.
        let mut rmw_packed = Vec::new();
        rmw_packed.extend_from_slice(&0i32.to_le_bytes());
        rmw_packed.extend_from_slice(&0u32.to_le_bytes());
        rmw_packed.extend_from_slice(&1u32.to_le_bytes()); // count-of-variables
        rmw_packed.extend_from_slice(&4u32.to_le_bytes());
        rmw_packed.extend_from_slice(b"odom");
        assert_eq!(rmw_packed.len(), body.len());
        assert_ne!(rmw_packed, body);
        // And the reader REFUSES it: offset 1 is below the data floor 16.
        assert_eq!(
            CanonicalBodyReader::new(&layout, &rmw_packed).unwrap_err(),
            ElementBodyError::EntryOutOfBounds
        );
    }

    /// Round-trip through the reader, with the values checked against the
    /// hand-built inputs (not against a second encode).
    #[test]
    fn builder_output_reads_back_field_for_field() {
        let layout = header_layout();
        let mut b = CanonicalBodyBuilder::new(&layout);
        b.fixed_mut()[0..4].copy_from_slice(&7i32.to_le_bytes());
        b.fixed_mut()[4..8].copy_from_slice(&42u32.to_le_bytes());
        b.push_variable(b"base_link".to_vec()).unwrap();
        let body = b.finish().unwrap();

        let r = CanonicalBodyReader::new(&layout, &body).unwrap();
        assert_eq!(r.fixed(), &[7, 0, 0, 0, 42, 0, 0, 0]);
        assert_eq!(r.variable(0), Some(&b"base_link"[..]));
        assert_eq!(r.variable(1), None);
        assert_eq!(r.variable_count(), 1);
    }

    /// An empty variable payload is legal and round-trips as empty.
    #[test]
    fn empty_variable_payload_round_trips() {
        let layout = header_layout();
        let mut b = CanonicalBodyBuilder::new(&layout);
        b.push_variable(Vec::new()).unwrap();
        let body = b.finish().unwrap();
        // 8 fixed + 8 table, no payload.
        assert_eq!(body.len(), 16);
        // Offset still points AT the floor with length 0 (not the (0,0) idiom).
        assert_eq!(&body[8..12], &16u32.to_le_bytes());
        assert_eq!(&body[12..16], &0u32.to_le_bytes());
        let r = CanonicalBodyReader::new(&layout, &body).unwrap();
        assert_eq!(r.variable(0), Some(&[][..]));
    }

    /// A schema with NO fixed fields (e.g. `tf2_msgs/TFMessage`) puts the
    /// table at offset 0 — the floor is then just `8 * n`.
    #[test]
    fn zero_fixed_size_layout_places_the_table_first() {
        let layout = WireLayout {
            qualified_name: "tf2_msgs/TFMessage".to_string(),
            schema_hash: 1,
            fixed_size: 0,
            fixed_align: 1,
            fixed_fields: vec![],
            variable_fields: vec![VariableFieldLayout {
                name: "transforms".to_string(),
                field_type: FieldType::DynamicArray {
                    element_type: Box::new(FieldType::Nested {
                        schema_name: "TransformStamped".to_string(),
                        package: Some("geometry_msgs".to_string()),
                        fixed: None,
                    }),
                },
            }],
        };
        let mut b = CanonicalBodyBuilder::new(&layout);
        assert!(b.fixed_mut().is_empty());
        b.push_variable(vec![0xAB, 0xCD]).unwrap();
        let body = b.finish().unwrap();
        let mut oracle = Vec::new();
        oracle.extend_from_slice(&8u32.to_le_bytes());
        oracle.extend_from_slice(&2u32.to_le_bytes());
        oracle.extend_from_slice(&[0xAB, 0xCD]);
        assert_eq!(body, oracle);
        let r = CanonicalBodyReader::new(&layout, &body).unwrap();
        assert!(r.fixed().is_empty());
        assert_eq!(r.variable(0), Some(&[0xAB, 0xCD][..]));
    }

    /// A `float64[]` payload is 8-byte aligned, so the writer inserts padding
    /// after an odd-length predecessor. Hand oracle on the exact offsets, and
    /// the pad must be at most 7 bytes (the walker's `MAX_ELEMENT_VAR_PAD`).
    #[test]
    fn primitive_array_payload_is_aligned_and_padding_is_bounded() {
        let layout = WireLayout {
            qualified_name: "test/Aligned".to_string(),
            schema_hash: 2,
            fixed_size: 0,
            fixed_align: 1,
            fixed_fields: vec![],
            variable_fields: vec![
                VariableFieldLayout {
                    name: "label".to_string(),
                    field_type: FieldType::String,
                },
                VariableFieldLayout {
                    name: "values".to_string(),
                    field_type: FieldType::DynamicArray {
                        element_type: Box::new(FieldType::F64),
                    },
                },
            ],
        };
        let mut b = CanonicalBodyBuilder::new(&layout);
        b.push_variable(b"abc".to_vec()).unwrap(); // 3 bytes → ends at 19
        b.push_variable(1.5f64.to_le_bytes().to_vec()).unwrap();
        let body = b.finish().unwrap();

        // Table is 2×8 = 16 bytes, so "abc" sits at [16..19]; the f64 payload
        // must start at the next multiple of 8 = 24 (5 pad bytes, ≤ 7).
        let (off0, len0) = read_offset_entry(&body, 0, 0);
        let (off1, len1) = read_offset_entry(&body, 0, 1);
        assert_eq!((off0, len0), (16, 3));
        assert_eq!((off1, len1), (24, 8));
        assert_eq!(off1 as usize % 8, 0, "f64 payload must be 8-byte aligned");
        let pad = off1 as usize - (off0 + len0) as usize;
        assert_eq!(pad, 5);
        assert!(
            pad <= 7,
            "padding must stay within the walker's MAX_ELEMENT_VAR_PAD"
        );
        // Padding bytes are ZERO — byte-determinism (Principle #7).
        assert_eq!(&body[19..24], &[0, 0, 0, 0, 0]);
        assert_eq!(&body[24..32], &1.5f64.to_le_bytes());
    }

    /// Lockstep is enforced in BOTH directions: too few pushes fails at
    /// `finish`, too many fails at `push_variable`.
    #[test]
    fn variable_count_lockstep_is_enforced_both_ways() {
        let layout = header_layout();

        let b = CanonicalBodyBuilder::new(&layout);
        assert_eq!(
            b.finish().unwrap_err(),
            ElementBodyError::VariableCountMismatch,
            "too few payloads must not silently produce a short body"
        );

        let mut b = CanonicalBodyBuilder::new(&layout);
        b.push_variable(b"one".to_vec()).unwrap();
        assert_eq!(
            b.push_variable(b"two".to_vec()).unwrap_err(),
            ElementBodyError::VariableCountMismatch
        );
        assert_eq!(b.pushed(), 1);
    }

    /// Hostile / truncated bodies are refused, never read out of bounds.
    #[test]
    fn reader_refuses_truncated_and_out_of_bounds_bodies() {
        let layout = header_layout();

        // Shorter than fixed(8) + table(8).
        assert_eq!(
            CanonicalBodyReader::new(&layout, &[0u8; 15]).unwrap_err(),
            ElementBodyError::BodyTruncated
        );
        // A `(0, 0)` entry is REFUSED like any other sub-floor entry.
        // It used to be carved out as the "unwritten field" idiom, but no
        // canonical writer emits it (the builder places even an EMPTY payload
        // at the floor, below; the `set_<f>_bytes(&[])` idiom it named writes
        // `(cursor, 0)`, not this), it is a TOP-LEVEL frame concern this reader
        // never sees, and the `FrameWalker` refuses the same bytes — a reader
        // accepting what the walker rejects is the writer/reader divergence
        // shape this module exists to remove.
        let mut all_zero = vec![0u8; 16];
        assert_eq!(
            CanonicalBodyReader::new(&layout, &all_zero).unwrap_err(),
            ElementBodyError::EntryOutOfBounds,
            "a (0, 0) entry points into the fixed section and must be refused"
        );
        // The CANONICAL spelling of an empty payload — offset AT the floor,
        // length 0 — is accepted, and reads back as the empty slice. This is
        // what `CanonicalBodyBuilder` actually emits, and it is the arm that
        // makes the refusal above a real distinction rather than a ban on
        // empty payloads.
        write_offset_entry(&mut all_zero, 8, 0, 16, 0);
        let reader = CanonicalBodyReader::new(&layout, &all_zero)
            .expect("(floor, 0) is the canonical empty payload");
        assert_eq!(reader.variable(0), Some(&[][..]));

        // Length runs past the end.
        let mut body = vec![0u8; 16];
        write_offset_entry(&mut body, 8, 0, 16, 4);
        assert_eq!(
            CanonicalBodyReader::new(&layout, &body).unwrap_err(),
            ElementBodyError::EntryOutOfBounds
        );

        // Offset below the data floor (into the table).
        let mut body = vec![0u8; 24];
        write_offset_entry(&mut body, 8, 0, 8, 0);
        assert_eq!(
            CanonicalBodyReader::new(&layout, &body).unwrap_err(),
            ElementBodyError::EntryOutOfBounds
        );

        // A hostile offset that would overflow on `off + len`.
        let mut body = vec![0u8; 24];
        write_offset_entry(&mut body, 8, 0, u32::MAX, u32::MAX);
        assert_eq!(
            CanonicalBodyReader::new(&layout, &body).unwrap_err(),
            ElementBodyError::EntryOutOfBounds
        );
    }

    /// The builder cannot emit the `(0, 0)` entry the reader used to
    /// carve out — an EMPTY payload is spelled `(floor, 0)`.
    ///
    /// This is the "unreachable from every canonical writer" half of the
    /// carve-out's deletion, asserted rather than argued: the reader's refusal
    /// above can only be wrong if some writer produces those bytes.
    #[test]
    fn the_builder_spells_an_empty_payload_at_the_floor_never_zero_zero() {
        let layout = header_layout();
        let mut b = CanonicalBodyBuilder::new(&layout);
        b.push_variable(Vec::new()).expect("empty payload");
        let body = b.finish().expect("finish");

        let floor = layout.fixed_size + 8 * layout.variable_fields.len();
        assert_eq!(read_offset_entry(&body, layout.fixed_size, 0), (16, 0));
        assert_eq!(floor, 16, "header floor = fixed(8) + one entry(8)");
        // And it round-trips through the reader that now refuses (0, 0).
        let reader = CanonicalBodyReader::new(&layout, &body).expect("reader");
        assert_eq!(reader.variable(0), Some(&[][..]));
    }

    /// Every error variant's `as_str` matches its `Display` — the rmw bridges
    /// propagate the static form, so the two must not drift apart.
    #[test]
    fn error_static_str_matches_display() {
        for e in [
            ElementBodyError::VariableCountMismatch,
            ElementBodyError::BodyTooLarge,
            ElementBodyError::BodyTruncated,
            ElementBodyError::EntryOutOfBounds,
        ] {
            assert_eq!(e.as_str(), e.to_string());
        }
    }

    /// `variable_payload_align` never exceeds 8 — the property the walker's
    /// `MAX_ELEMENT_VAR_PAD = 7` depends on.
    #[test]
    fn alignment_never_exceeds_eight() {
        let cases = [
            (FieldType::String, 1usize),
            (FieldType::Bytes, 1),
            (
                FieldType::DynamicArray {
                    element_type: Box::new(FieldType::U8),
                },
                1,
            ),
            (
                FieldType::DynamicArray {
                    element_type: Box::new(FieldType::U16),
                },
                2,
            ),
            (
                FieldType::DynamicArray {
                    element_type: Box::new(FieldType::F32),
                },
                4,
            ),
            (
                FieldType::DynamicArray {
                    element_type: Box::new(FieldType::F64),
                },
                8,
            ),
            (
                FieldType::DynamicArray {
                    element_type: Box::new(FieldType::String),
                },
                1,
            ),
        ];
        for (ft, expect) in cases {
            let a = variable_payload_align(&ft);
            assert_eq!(a, expect, "alignment for {ft:?}");
            assert!(a <= 8);
        }
    }
}
