// SPDX-License-Identifier: AGPL-3.0-only
//! Generic wire-frame walker.
//!
//! [`FrameWalker`] decodes a raw Cerulion wire frame into a typed
//! [`FrameValue`] tree, driven ENTIRELY by the schema IR — no per-message
//! code. It is designed to replace the hand-written, two-hash decoders in
//! `cerulion_cli_engine::topic_cmd` (`decode_string_payload` /
//! `decode_image_meta`) — those decoders still exist today; rewriting
//! `topic echo` onto the walker is a tracked follow-on — and it is the
//! substrate for schema-driven sinks such as the Rerun bridge (`examples/go2`).
//!
//! # What it does
//!
//! Given a resolved schema set (typically parsed from
//! `native_ros2_messages::BUILTIN_MSGS` + any workspace schemas) and a frame
//! (`&[u8]` — the exact bytes a subscriber sees, WireHeader included), it
//! walks the wire layout and yields every field as a borrowed
//! [`FrameValueKind`]. Reads are zero-copy: strings and byte slices borrow
//! directly out of the frame.
//!
//! # Wire layout it decodes (see [`crate::shm_runtime`] / [`crate::wire`])
//!
//! ```text
//! [WireHeader (32 B)][fixed section][offset table (8·N B)][variable payload]
//! ```
//!
//! - Fixed-section fields are read at the `#[repr(C)]` offsets computed by
//!   [`crate::codegen::layout::LayoutResolver`] (the SAME engine the rmw
//!   bridge uses, so decode == the generated writer's encode).
//! - Variable fields are located via the offset table
//!   ([`crate::shm_runtime::read_offset_entry`]): each entry is an
//!   `(offset, length)` pair, `offset` measured from `payload[0]` (i.e.
//!   after the WireHeader).
//! - Nested **fixed** messages are inlined in the fixed section and decoded
//!   recursively. Nested **variable** messages live in the offset table as a
//!   *headerless* sub-frame (`[fixed][table][var]` of the target — the exact
//!   shape `set_<f>_bytes` writes) and are decoded recursively.
//!
//! # Canonical element framing for arrays of nested messages
//!
//! A `DynamicArray<Nested>` / `FixedArray<Nested-variable>` / `string[]`
//! (e.g. `geometry_msgs/PoseArray.poses`, `nav_msgs/Path.poses`,
//! `sensor_msgs/PointField[] fields`) carries its elements under the
//! **canonical element encoding** documented by
//! [`CdrCodec`](crate::codegen::CdrCodec) (`cdr_codec.rs`, "Cerulion
//! variable-payload encodings"):
//!
//! | element | variable-payload encoding | producers |
//! |---|---|---|
//! | `Nested`, recursively FIXED | back-to-back fixed sections, stride = the target's padded fixed size, **no count** | the `ros2 attach` / `dds_bridge` CDR codec; the rmw introspection bridge |
//! | `Nested`, VARIABLE | `u32 count` + per element `u32 len` + headerless sub-frame | the CDR codec; the rmw introspection bridge |
//! | `string` | `u32 count` + per element `u32 len` + UTF-8 | the CDR codec; the rmw introspection bridge |
//!
//! Every producer now writes the SAME element body. The rmw
//! bridge used to agree on the ARRAY framing but not on a VARIABLE element's
//! BODY (packed fixed members + a `u32` count of variable members + per-variable
//! `[u32 len]`, with NO offset table), so an rmw robot's `Path` / `TFMessage` /
//! `Detection*Array` stayed opaque here. Both bridges now build the body
//! through the shared [`element_codec`](crate::codegen::element_codec), and
//! `crates/rmw_cerulion/tests/canonical_element_body_test.rs` cross-validates rmw
//! frames by decoding them with THIS walker. The bespoke framing is still
//! REFUSED, on its own hand-built bytes rather than as a live producer's
//! output — pinned by `legacy_packed_element_body_stays_opaque` (kept: a
//! recorded bag or an un-upgraded peer can still carry it, and it must degrade
//! rather than mis-decode).
//!
//! The walker decodes that framing RECURSIVELY (an element body is just the
//! headerless sub-frame shape [`FrameWalker::walk`] already decodes, so a nested
//! array inside an element composes for free) and yields
//! [`FrameValueKind::NestedArray`].
//!
//! **This is decoded, not merely surfaced.** The `cerulion_viz` viz ladder now
//! CONSUMES the decoded elements to DRAW geometry — a nav2 robot's `/plan`
//! (`nav_msgs/Path`) renders as a polyline, a `PoseArray` as points, a
//! `Detection3DArray` as boxes — instead of the earlier text dump. Decoding
//! inside the walker (rather than exposing the raw payload for every consumer to
//! re-parse) keeps exactly one implementation of the element convention on the
//! read side.
//!
//! Decoding is **total and strictly validated** on TWO levels:
//!
//! - the ARRAY's own framing — exact byte consumption of the blob, a count
//!   budget checked before any allocation, and per-element bounds;
//! - each ELEMENT's own INTERNAL accounting (`PayloadAudit::Element`) —
//!   `fixed section + offset table + the bytes its entries describe` must
//!   consume the element body EXACTLY, with entries in order, non-overlapping,
//!   in bounds, and separated by at most alignment padding. Zero-length
//!   entries are audited like any other (they carry a real offset in the
//!   canonical form), and the rule applies recursively to a variable-nested
//!   sub-frame INSIDE an element.
//!
//! On ANY inconsistency the field degrades to
//! [`FrameValueKind::NestedArrayOpaque`] — the earlier behaviour — rather
//! than yielding a plausible-but-wrong element list. Both levels are needed to
//! keep producer-defined conventions opaque: `cerulion_viz::go2_tf`'s 84-byte
//! `/tf` blob PASSES the array level for exactly one stamp value — its
//! `stamp.sec` field sits exactly where the canonical per-element `u32 len`
//! does, so `sec == 76` makes `4 + 4 + 76 == 84` exact, and a robot's
//! monotonic uptime clock passes through 76 seconds every boot. Only the
//! ELEMENT audit refuses it (the 76-byte "element" carries two all-zero
//! offset-table entries in the identity quaternion's bytes, and even the
//! canonical `(72, 0)` pair would leave its last 4 bytes unaccounted) — see
//! `go2_tf_bespoke_blob_stays_opaque_at_every_stamp`.
//!
//! Two facts worth knowing when consuming these values:
//!
//! - **No alignment guarantee.** A complex variable field's payload starts
//!   wherever the writer's cursor happened to be (the generated writers
//!   align-up only for primitive arrays), so an element blob can begin at an
//!   odd offset. Decoding is unaligned-safe (`from_le_bytes` over slices),
//!   but a consumer can never cast the blob to a typed slice.
//! - **Element-schema skew is not hash-covered.** The wire `schema_hash`
//!   folds a *fixed*-resolved nested target's hash into the parent but NOT a
//!   variable one (it is self-describing on the wire), so a producer/consumer
//!   schema skew *inside* a variable element type is not detectable from the
//!   header. This is a pre-existing property of single variable-nested fields
//!   (the walker resolves the element layout from its OWN schema set); arrays
//!   inherit it rather than introducing it.
//!
//! # What it does NOT decode
//!
//! These shapes have no canonical v1 element encoding (or no resolvable
//! element layout) and still surface as
//! [`FrameValueKind::NestedArrayOpaque`]:
//!
//! - a `FixedArray<StringFixed(n)>` in the fixed section (back-to-back
//!   NUL-padded strings);
//! - `bool[]` — `decode_variable`'s `DynamicArray` arm special-cases only
//!   `u8`/`i8` as raw bytes and `Bool` is not a [`PrimType`], so it falls to
//!   the opaque arm (its bytes ARE one-per-element, but v1 defines no framing
//!   for them);
//! - `DynamicArray<StringFixed(n)>` and `DynamicArray<DynamicArray<..>>`;
//! - any `Nested[]` whose element schema is not in the walker's set (the
//!   walker cannot know the element shape, so it must not guess one).
//!
//! # Error discipline (Principle: loud over silent)
//!
//! Every structural problem — unknown schema, frame shorter than the fixed
//! section, a payload that ends inside the declared offset table, an
//! offset-table entry pointing outside the payload — returns a
//! [`WalkError`]. The walker is an UNTRUSTED-INPUT parser and is therefore
//! stricter than the writer-side compat fallback in
//! [`crate::shm_runtime::read_offset_entry`] (which serves `(0, 0)` for a
//! short buffer): a table-truncated frame REFUSES with
//! [`WalkError::OffsetTableTruncated`] instead of silently decoding
//! all-empty variable fields. The walker NEVER panics on adversarial input
//! and NEVER fabricates a value. Two deliberate degradations that are NOT
//! errors: invalid UTF-8 in a string field degrades to
//! [`FrameValueKind::Bytes`] (the raw bytes are real; only the `&str` view
//! failed), and an EMPTY variable-nested blob — the `set_<f>_bytes(&[])`
//! producer idiom for an intentionally empty nested field (e.g. an empty
//! `Header`) — decodes as a present [`FrameValueKind::Nested`] value with
//! zero fields (any NON-empty blob shorter than the target's fixed section
//! + offset table still errors).

use std::collections::BTreeMap;

use super::layout::{LayoutResolver, WireLayout};
use super::schema::{FieldType, MessageSchema};
use crate::shm_runtime::read_offset_entry;
use crate::wire::WireHeader;

/// A decoded scalar / composite value borrowed out of a wire frame.
///
/// The `'a` lifetime ties string / byte / array views back to the frame
/// bytes passed to [`FrameWalker::walk`], so decoding allocates only the
/// field-name `String`s and the `Vec` spine of the tree — never the payload.
#[derive(Debug, Clone, PartialEq)]
pub enum FrameValueKind<'a> {
    Bool(bool),
    I8(i8),
    U8(u8),
    I16(i16),
    U16(u16),
    I32(i32),
    U32(u32),
    I64(i64),
    U64(u64),
    F32(f32),
    F64(f64),
    /// A `string` / `string_fixed[n]` field, UTF-8-validated. A fixed
    /// string is trimmed at its first NUL.
    Str(&'a str),
    /// A `bytes` / `uint8[]` / `int8[]` field, or a string field whose
    /// bytes failed UTF-8 validation (the bytes are still real).
    Bytes(&'a [u8]),
    /// A `FixedArray` or `DynamicArray` of a numeric primitive
    /// (`float32[9]`, `float64[]`, `uint32[]`, …). Elements are LE-packed in
    /// [`PrimArray::bytes`]; use the typed iterators to read them.
    PrimArray(PrimArray<'a>),
    /// A recursively decoded nested message (fixed-inlined OR variable
    /// sub-frame).
    Nested(Box<FrameValue<'a>>),
    /// A `FixedArray` of nested-fixed messages, decoded element-by-element.
    Array(Vec<FrameValueKind<'a>>),
    /// A `DynamicArray` / `FixedArray` of nested messages or strings, decoded
    /// element-by-element under the CANONICAL ELEMENT FRAMING (see the
    /// module docs). Elements are [`FrameValueKind::Nested`] (message
    /// elements) or [`FrameValueKind::Str`] / [`FrameValueKind::Bytes`]
    /// (`string[]`, the latter when an element failed UTF-8 validation).
    ///
    /// DELIBERATELY distinct from [`FrameValueKind::Array`]: `Array` means "a
    /// fixed-length array of fixed-nested messages, split by stride" and
    /// consumers already expand it per index (e.g. into plot series). A
    /// 5 000-pose `/plan` must not silently take that path, so every consumer
    /// is compiler-forced to decide how it handles a dynamic element array.
    NestedArray {
        /// The decoded elements.
        elements: Vec<FrameValueKind<'a>>,
        /// The field's RAW bytes — byte-for-byte what
        /// [`FrameValueKind::NestedArrayOpaque`] would have carried for the
        /// same field.
        ///
        /// Present so that DECODING A FIELD CAN NEVER TAKE INFORMATION AWAY
        /// from a consumer: an older consumer with its own element
        /// convention (`cerulion_viz::tf`, `cerulion_viz::pointcloud`) keeps
        /// reading `raw` and behaves exactly as it did, while a consumer that
        /// wants the decoded tree reads `elements`. Without it, teaching the
        /// walker a new framing silently changed those consumers' inputs from
        /// "the bytes" to "nothing", which is how the first diff turned a
        /// truthful "undecodable TF transforms blob" warning into a FALSE
        /// "frame has no `transforms` array".
        ///
        /// For an EMPTY array this is the field's own empty slice, so
        /// `raw.is_empty()` and `elements.is_empty()` agree and a
        /// bespoke-convention consumer sees the same "no elements" blob it
        /// saw before.
        raw: &'a [u8],
    },
    /// A `DynamicArray<Nested>` / `FixedArray<Nested-variable>` / `string[]`
    /// whose bytes are NOT canonically framed (a producer-defined convention,
    /// or a corrupt/foreign blob): surfaced verbatim so a consumer can apply
    /// its own convention. See the module docs — this is the explicit
    /// degradation, never a guessed element list.
    NestedArrayOpaque(&'a [u8]),
}

/// The numeric element type of a [`PrimArray`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrimType {
    I16,
    U16,
    I32,
    U32,
    I64,
    U64,
    F32,
    F64,
}

impl PrimType {
    /// Byte width of one element.
    pub const fn size(self) -> usize {
        match self {
            PrimType::I16 | PrimType::U16 => 2,
            PrimType::I32 | PrimType::U32 | PrimType::F32 => 4,
            PrimType::I64 | PrimType::U64 | PrimType::F64 => 8,
        }
    }
}

/// A borrowed view over a LE-packed array of a numeric primitive.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PrimArray<'a> {
    /// Element type.
    pub elem: PrimType,
    /// Raw little-endian element bytes (length == `count * elem.size()`).
    pub bytes: &'a [u8],
    /// Number of complete elements the array holds.
    pub count: usize,
}

impl<'a> PrimArray<'a> {
    /// Iterate the elements as `f32`, widening/reinterpreting from the
    /// stored primitive (`F32` verbatim; `F64` narrowed; integer types cast
    /// to `f32`). Trailing partial bytes (never present for a well-formed
    /// frame) are ignored.
    pub fn iter_f32(&self) -> impl Iterator<Item = f32> + '_ {
        (0..self.count).map(move |i| {
            let s = i * self.elem.size();
            let b = &self.bytes[s..s + self.elem.size()];
            match self.elem {
                PrimType::F32 => f32::from_le_bytes([b[0], b[1], b[2], b[3]]),
                PrimType::F64 => {
                    f64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]) as f32
                }
                PrimType::I16 => i16::from_le_bytes([b[0], b[1]]) as f32,
                PrimType::U16 => u16::from_le_bytes([b[0], b[1]]) as f32,
                PrimType::I32 => i32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f32,
                PrimType::U32 => u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f32,
                PrimType::I64 => {
                    i64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]) as f32
                }
                PrimType::U64 => {
                    u64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]) as f32
                }
            }
        })
    }

    /// Iterate the elements as `f64` (`F64` verbatim; everything else
    /// widened/cast).
    pub fn iter_f64(&self) -> impl Iterator<Item = f64> + '_ {
        (0..self.count).map(move |i| {
            let s = i * self.elem.size();
            let b = &self.bytes[s..s + self.elem.size()];
            match self.elem {
                PrimType::F64 => {
                    f64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]])
                }
                PrimType::F32 => f32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f64,
                PrimType::I16 => i16::from_le_bytes([b[0], b[1]]) as f64,
                PrimType::U16 => u16::from_le_bytes([b[0], b[1]]) as f64,
                PrimType::I32 => i32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f64,
                PrimType::U32 => u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as f64,
                PrimType::I64 => {
                    i64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]) as f64
                }
                PrimType::U64 => {
                    u64::from_le_bytes([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]]) as f64
                }
            }
        })
    }
}

/// One `(field name, value)` pair in a decoded frame, in wire order (fixed
/// fields first, then variable fields — each group in declaration order).
#[derive(Debug, Clone, PartialEq)]
pub struct NamedValue<'a> {
    pub name: String,
    pub value: FrameValueKind<'a>,
}

/// A fully decoded wire frame: the qualified schema name plus every field.
#[derive(Debug, Clone, PartialEq)]
pub struct FrameValue<'a> {
    /// Qualified schema name (`"pkg/Name"` or bare).
    pub schema_name: String,
    /// Decoded fields (fixed group then variable group; see [`NamedValue`]).
    pub fields: Vec<NamedValue<'a>>,
}

impl<'a> FrameValue<'a> {
    /// Look a field up by name.
    pub fn field(&self, name: &str) -> Option<&FrameValueKind<'a>> {
        self.fields
            .iter()
            .find(|f| f.name == name)
            .map(|f| &f.value)
    }
}

/// A structural decode failure. Every variant names the offending element;
/// the walker returns these instead of panicking or guessing.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum WalkError {
    /// The requested qualified schema name is not in the walker's set.
    #[error("unknown schema '{0}' (not in the walker's schema set)")]
    UnknownSchema(String),

    /// No schema in the set has this `schema_hash`.
    #[error("unknown schema_hash 0x{0:016X} (no schema in the set matches)")]
    UnknownSchemaHash(u64),

    /// The frame is shorter than the 32-byte WireHeader.
    #[error("frame too short: have {have} bytes, need at least {need} for the WireHeader")]
    FrameTooShort { have: usize, need: usize },

    /// A fixed-section field extends past the end of the payload (a
    /// truncated frame).
    #[error(
        "fixed field '{field}' out of bounds: [{offset}, {offset}+{size}) exceeds payload len {payload_len}"
    )]
    FixedFieldOutOfBounds {
        field: String,
        offset: usize,
        size: usize,
        payload_len: usize,
    },

    /// The payload ends inside (or before) the declared offset table — a
    /// truncated frame. The writer-side
    /// [`crate::shm_runtime::read_offset_entry`] fallback would serve
    /// `(0, 0)` for the missing entries; as an untrusted-input parser the
    /// walker refuses instead of silently decoding all-empty variable
    /// fields. Boundary: `have == fixed_size + table_bytes` is VALID (every
    /// entry present; all may legitimately be empty).
    #[error(
        "offset table truncated in '{schema}': payload has {have} bytes, needs at least \
         {fixed_size} (fixed section) + {table_bytes} (offset table)"
    )]
    OffsetTableTruncated {
        schema: String,
        have: usize,
        fixed_size: usize,
        table_bytes: usize,
    },

    /// An offset-table entry points outside the variable payload region (a
    /// corrupt or truncated frame).
    #[error(
        "variable field '{field}' out of bounds: offset {offset} len {length} vs payload len {payload_len}"
    )]
    VariableFieldOutOfBounds {
        field: String,
        offset: usize,
        length: usize,
        payload_len: usize,
    },

    /// A nested reference could not be resolved to a schema in the set (the
    /// nested target was omitted when the walker was built).
    #[error("nested field '{field}' references unresolvable schema '{target}'")]
    NestedResolutionFailed { field: String, target: String },

    /// A canonical ELEMENT body's own accounting is not exact:
    /// its fixed section + offset table + the bytes its entries describe
    /// do not consume the body exactly, or an entry is out of order /
    /// overlapping / separated from its predecessor by more than alignment
    /// padding. Only reachable under `PayloadAudit::Element`; the array
    /// decode maps it (like every other walk error) to
    /// [`FrameValueKind::NestedArrayOpaque`].
    #[error(
        "element body of '{schema}' is not exactly consumed: accounted {consumed} of {body_len} \
         bytes (a bespoke element convention, or a corrupt frame)"
    )]
    ElementBodyNotExactlyConsumed {
        schema: String,
        consumed: usize,
        body_len: usize,
    },
}

/// How strictly ONE payload's own offset-table accounting is audited.
///
/// The two modes exist because the two payload classes have genuinely
/// different producers: a top-level frame is written field-by-field by a
/// generated writer that leaves an UNWRITTEN variable field's entry at
/// `(0, 0)` (which [`read_offset_entry`] serves as an empty slice), whereas a
/// canonical element body is emitted whole — by
/// [`CdrCodec`](crate::codegen::CdrCodec)'s `decode_payload`, the ONLY
/// producer of this shape — with a real offset for EVERY entry and no slack.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PayloadAudit {
    /// A top-level frame, or a variable-nested sub-frame reached from one.
    /// Each entry is bounds-checked on its own; a `(0, 0)` entry is the
    /// unwritten-field idiom and trailing slack is tolerated.
    ///
    /// # Arbitrary in-bounds top-level placement is INTENDED legal wire
    ///
    /// Adopted frames depend on this: a top-level offset-table entry may point
    /// anywhere in `[data_floor, payload.len())` — out of declaration order,
    /// page-aligned, with dead gap bytes between fields and trailing slack,
    /// all inside `total_size`. The ONLY constraints are per-entry:
    /// `off >= data_floor` (an entry may never alias the fixed section or
    /// the offset table itself) and `off + len <= payload.len()`.
    /// Ordering / contiguity / exact accounting are deliberately NOT
    /// required here — an adopting publisher places its big variable field
    /// page-aligned in the slot tail, and any future tightening of
    /// this audit would break every adopted frame. Pinned by
    /// `crates/cerulion_core/tests/wire_gap_frame_test.rs` (plus bagd / gateway /
    /// replay byte-fidelity twins); the strict twin below stays strict for
    /// element bodies only.
    Frame,
    /// A canonical element body (or a variable-nested sub-frame INSIDE one).
    /// EXACT internal accounting is required — the discriminator that keeps a
    /// bespoke element convention whose bytes happen to satisfy the array
    /// framing from decoding by accident.
    Element,
}

/// The largest gap [`PayloadAudit::Element`] tolerates between one variable
/// field's end and the next one's offset. `CdrCodec::decode_payload` aligns a
/// variable payload to its element size (`variable_payload_align`), and the
/// widest such alignment is 8 bytes, so genuine padding is at most 7 bytes.
const MAX_ELEMENT_VAR_PAD: usize = 7;

/// A schema-driven wire-frame decoder over a resolved schema set.
///
/// Build once from a schema set (e.g. every `native_ros2_messages::BUILTIN_MSGS`
/// entry), then [`walk`](Self::walk) any number of frames.
///
/// `Clone` deep-copies the precomputed layout maps (a one-time cost at build
/// time) so a single parse can be shared by two owners — e.g. the vizd daemon
/// builds one walker and hands a clone to its render worker, guaranteeing both
/// halves resolve against the identical schema set without re-parsing.
#[derive(Clone)]
pub struct FrameWalker {
    /// Qualified name → wire layout, precomputed for every schema in the set
    /// (including nested targets, so recursion is a map lookup).
    layouts: BTreeMap<String, WireLayout>,
    /// `schema_hash` → qualified name, built once in [`Self::new`]. Turns
    /// [`Self::walk_by_hash`] / [`Self::schema_name_for_hash`] /
    /// [`Self::layout_for_hash`] from an `O(n_schemas)` linear scan (over the
    /// hundreds of built-ins) into an `O(log n)` lookup with NO per-frame
    /// `String` clone — the fixed per-frame overhead that scales with topic
    /// fan-out. On the (pathological) chance two schemas share a hash, the
    /// lexicographically SMALLEST qualified name wins, exactly matching a
    /// first-hit `layouts.iter().find()` scan.
    hash_index: BTreeMap<u64, String>,
}

impl FrameWalker {
    /// Build a walker over `schemas`. Runs nested-fixed resolution + layout
    /// computation via [`LayoutResolver`]; returns any resolution warnings
    /// alongside (callers should surface them — a warning means a nested
    /// reference could not be resolved to a fixed target).
    pub fn new(schemas: Vec<MessageSchema>) -> (Self, Vec<String>) {
        // Resolve first so we can capture the qualified-name set (the
        // resolver consumes `schemas`). `resolve_fixed_nested` only computes
        // warnings; it does not log — so no double-logging with the resolver's
        // own (idempotent, no-op-on-resolved) internal pass below.
        let mut schemas = schemas;
        let warnings = super::resolve_fixed_nested(&mut schemas);
        let qnames: Vec<String> = schemas.iter().map(|s| s.qualified_name()).collect();
        // The resolver re-runs resolution (idempotent — finds nothing on an
        // already-resolved set, so it logs no duplicate warnings) and gives
        // us the memoized layouts.
        let (mut resolver, _resolver_warnings) = LayoutResolver::new(schemas);
        let mut layouts = BTreeMap::new();
        for q in qnames {
            if let Some(layout) = resolver.layout_of(&q) {
                layouts.insert(q, layout);
            }
        }
        // Index `schema_hash → qualified name` once so hash-driven resolution
        // is a map lookup, not a per-frame linear scan + clone. `layouts`
        // iterates in ascending qualified-name order, so `or_insert` keeps the
        // smallest name on a (pathological) hash collision — byte-identical to
        // a first-hit `.iter().find()`.
        let mut hash_index = BTreeMap::new();
        for (qname, layout) in &layouts {
            hash_index
                .entry(layout.schema_hash)
                .or_insert_with(|| qname.clone());
        }
        (
            Self {
                layouts,
                hash_index,
            },
            warnings,
        )
    }

    /// True if `qualified_name` is decodable by this walker.
    pub fn knows(&self, qualified_name: &str) -> bool {
        self.layouts.contains_key(qualified_name)
    }

    /// Resolve a frame's `WireHeader.schema_hash` to the qualified schema
    /// name this walker would decode it as, if any. `O(log n)` via the
    /// `hash → name` index built at construction — no per-frame scan, no
    /// `String` clone (the returned `&str` borrows the index).
    pub fn schema_name_for_hash(&self, schema_hash: u64) -> Option<&str> {
        self.hash_index.get(&schema_hash).map(String::as_str)
    }

    /// The pre-index linear scan, kept as a test-only reference so the
    /// `hash_index`-backed [`schema_name_for_hash`](Self::schema_name_for_hash)
    /// can be pinned equivalent to it over the whole built-in corpus (a
    /// genuine cross-algorithm check — `BTreeMap` lookup vs `.iter().find` —
    /// not a self-compare).
    #[cfg(test)]
    fn schema_name_for_hash_linear(&self, schema_hash: u64) -> Option<&str> {
        self.layouts
            .iter()
            .find(|(_, l)| l.schema_hash == schema_hash)
            .map(|(k, _)| k.as_str())
    }

    /// Resolve a frame's `WireHeader.schema_hash` directly to the cached
    /// [`WireLayout`] this walker decodes it with — the SAME layout
    /// [`walk_by_hash`](Self::walk_by_hash) uses, exposed so a caller can
    /// cache it per topic (a topic's hash is stable) and skip even the index
    /// lookup on subsequent frames. `O(log n)`, no allocation.
    pub fn layout_for_hash(&self, schema_hash: u64) -> Option<&WireLayout> {
        let qname = self.hash_index.get(&schema_hash)?;
        self.layouts.get(qname)
    }

    /// Resolve a qualified schema NAME directly to its layout. This is the
    /// collision-proof lookup: the hash index keeps one winner on a hash
    /// collision, so a NAME request must never be routed through it.
    pub fn layout_for_name(&self, qualified_name: &str) -> Option<&WireLayout> {
        self.layouts.get(qualified_name)
    }

    /// Resolve a qualified schema NAME (`pkg/Type`) to the `schema_hash` this
    /// walker would stamp / expect for it, if the schema is in the set. The
    /// inverse of [`schema_name_for_hash`](Self::schema_name_for_hash): the viz
    /// daemon uses it to turn a controller-provided ROS type name (a remote
    /// topic's schema, discovered out-of-band) into the wire `schema_hash` that
    /// `register_ingress_topic` validates against. `O(log n)`,
    /// no allocation.
    pub fn schema_hash_for(&self, qualified_name: &str) -> Option<u64> {
        self.layouts.get(qualified_name).map(|l| l.schema_hash)
    }

    /// Decode `frame` as the schema named `qualified_name`.
    ///
    /// `frame` is the whole wire frame including the 32-byte WireHeader
    /// (exactly what a subscriber's `payload()` / an `InputView`'s frame
    /// bytes return).
    pub fn walk<'a>(
        &self,
        qualified_name: &str,
        frame: &'a [u8],
    ) -> Result<FrameValue<'a>, WalkError> {
        let layout = self
            .layouts
            .get(qualified_name)
            .ok_or_else(|| WalkError::UnknownSchema(qualified_name.to_string()))?;
        if frame.len() < WireHeader::SIZE {
            return Err(WalkError::FrameTooShort {
                have: frame.len(),
                need: WireHeader::SIZE,
            });
        }
        let payload = &frame[WireHeader::SIZE..];
        self.walk_payload(layout, payload, 0, PayloadAudit::Frame)
    }

    /// Decode `frame` using its own `WireHeader.schema_hash` to pick the
    /// schema (the general "topic echo" entry point). Errors if the hash is
    /// unknown or the frame is truncated.
    pub fn walk_by_hash<'a>(&self, frame: &'a [u8]) -> Result<FrameValue<'a>, WalkError> {
        if frame.len() < WireHeader::SIZE {
            return Err(WalkError::FrameTooShort {
                have: frame.len(),
                need: WireHeader::SIZE,
            });
        }
        let header = WireHeader::read_from_buf(frame).ok_or(WalkError::FrameTooShort {
            have: frame.len(),
            need: WireHeader::SIZE,
        })?;
        // Both `qname` (borrowing `hash_index`) and `walk` (borrowing
        // `layouts`) are shared borrows of `self`; the returned `FrameValue`
        // is tied to `frame`, not `self`, so no clone is needed to break a
        // borrow — the per-frame qname `String` allocation is gone.
        let qname = self
            .schema_name_for_hash(header.schema_hash)
            .ok_or(WalkError::UnknownSchemaHash(header.schema_hash))?;
        self.walk(qname, frame)
    }

    /// Decode a headerless payload (`[fixed][table][var]`) against a layout.
    /// Used both for the top-level frame (after stripping the header) and for
    /// nested sub-frames.
    ///
    /// `audit` selects how strictly the payload's OWN offset-table accounting
    /// is checked — see [`PayloadAudit`].
    fn walk_payload<'a>(
        &self,
        layout: &WireLayout,
        payload: &'a [u8],
        array_depth: u32,
        audit: PayloadAudit,
    ) -> Result<FrameValue<'a>, WalkError> {
        // Untrusted-input strictness (findings 1+5): the WHOLE declared
        // offset table must be present up front. `read_offset_entry`'s
        // (0, 0) short-buffer fallback exists for the production writer's
        // compat path; silently decoding a table-truncated frame as
        // all-empty variable fields would fabricate values. The boundary
        // (payload == fixed + table exactly) is valid — all entries
        // present, each may be legitimately empty. Fixed-only schemas
        // (no variable fields) keep the per-field bounds checks below as
        // their only truncation gate.
        if !layout.variable_fields.is_empty() {
            let need = layout.fixed_size + layout.offset_table_bytes();
            if payload.len() < need {
                return Err(WalkError::OffsetTableTruncated {
                    schema: layout.qualified_name.clone(),
                    have: payload.len(),
                    fixed_size: layout.fixed_size,
                    table_bytes: layout.offset_table_bytes(),
                });
            }
        }

        let mut fields =
            Vec::with_capacity(layout.fixed_fields.len() + layout.variable_fields.len());

        // Fixed section: each field at its computed #[repr(C)] offset.
        for fl in &layout.fixed_fields {
            let end = fl.offset.checked_add(fl.size);
            let bytes = end.and_then(|e| payload.get(fl.offset..e)).ok_or_else(|| {
                WalkError::FixedFieldOutOfBounds {
                    field: fl.name.clone(),
                    offset: fl.offset,
                    size: fl.size,
                    payload_len: payload.len(),
                }
            })?;
            let value =
                self.decode_fixed(&fl.field_type, bytes, &layout.qualified_name, array_depth)?;
            fields.push(NamedValue {
                name: fl.name.clone(),
                value,
            });
        }

        // Variable section: locate each field via the offset table.
        let data_floor = layout.data_floor();
        // `Element` audit only: the running end of the accounted-for variable
        // region. Starts at the data floor (nothing past the table is
        // accounted yet) and must finish EXACTLY at `payload.len()`.
        let mut accounted = data_floor;
        for (idx, vf) in layout.variable_fields.iter().enumerate() {
            let (off, len) = read_offset_entry(payload, layout.fixed_size, idx);
            let (off, len) = (off as usize, len as usize);
            let bytes: &[u8] = match audit {
                // An unwritten / genuinely empty field: `read_offset_entry`
                // returns (0, 0). An empty slice is the correct value, not a
                // corruption.
                PayloadAudit::Frame if len == 0 => &[],
                PayloadAudit::Frame => {
                    let end = off.checked_add(len);
                    let in_bounds = off >= data_floor && end.is_some_and(|e| e <= payload.len());
                    if !in_bounds {
                        return Err(WalkError::VariableFieldOutOfBounds {
                            field: vf.name.clone(),
                            offset: off,
                            length: len,
                            payload_len: payload.len(),
                        });
                    }
                    &payload[off..off + len]
                }
                // EVERY entry — zero-length included — must be
                // accounted at its own offset, in order, with at most
                // alignment padding since the previous field's end. The
                // canonical producer always writes a real offset (never
                // `(0, 0)`), so accepting the unwritten-field idiom here would
                // leave the bytes between the table and the first accounted
                // field unvalidated — the escape a bespoke blob's aliasing
                // zeros walk through.
                PayloadAudit::Element => {
                    let end = off.checked_add(len);
                    let in_bounds = off >= accounted
                        && off - accounted <= MAX_ELEMENT_VAR_PAD
                        && end.is_some_and(|e| e <= payload.len());
                    if !in_bounds {
                        return Err(WalkError::VariableFieldOutOfBounds {
                            field: vf.name.clone(),
                            offset: off,
                            length: len,
                            payload_len: payload.len(),
                        });
                    }
                    accounted = off + len;
                    &payload[off..off + len]
                }
            };
            let value = self.decode_variable(
                &vf.field_type,
                bytes,
                &layout.qualified_name,
                array_depth,
                audit,
            )?;
            fields.push(NamedValue {
                name: vf.name.clone(),
                value,
            });
        }

        // Element bodies are exactly consumed: no interior hole beyond
        // alignment padding (checked per entry above) and no trailing slack.
        // For a FIXED element (no variable fields) this reduces to
        // `payload.len() == fixed_size`, which the caller already guarantees.
        if audit == PayloadAudit::Element && accounted != payload.len() {
            return Err(WalkError::ElementBodyNotExactlyConsumed {
                schema: layout.qualified_name.clone(),
                consumed: accounted,
                body_len: payload.len(),
            });
        }

        Ok(FrameValue {
            schema_name: layout.qualified_name.clone(),
            fields,
        })
    }

    /// Decode one fixed-section field. `bytes` is exactly the field's slice
    /// (already sized to `field_type`'s fixed size by the caller).
    fn decode_fixed<'a>(
        &self,
        ft: &FieldType,
        bytes: &'a [u8],
        parent_qname: &str,
        array_depth: u32,
    ) -> Result<FrameValueKind<'a>, WalkError> {
        Ok(match ft {
            FieldType::Bool => FrameValueKind::Bool(bytes[0] != 0),
            FieldType::I8 => FrameValueKind::I8(bytes[0] as i8),
            FieldType::U8 => FrameValueKind::U8(bytes[0]),
            FieldType::I16 => FrameValueKind::I16(i16::from_le_bytes([bytes[0], bytes[1]])),
            FieldType::U16 => FrameValueKind::U16(u16::from_le_bytes([bytes[0], bytes[1]])),
            FieldType::I32 => {
                FrameValueKind::I32(i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
            }
            FieldType::U32 => {
                FrameValueKind::U32(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
            }
            FieldType::I64 => FrameValueKind::I64(i64::from_le_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
            ])),
            FieldType::U64 => FrameValueKind::U64(u64::from_le_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
            ])),
            FieldType::F32 => {
                FrameValueKind::F32(f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
            }
            FieldType::F64 => FrameValueKind::F64(f64::from_le_bytes([
                bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
            ])),
            FieldType::StringFixed(_) => decode_str_or_bytes(trim_at_nul(bytes)),
            FieldType::FixedArray {
                element_type,
                length,
            } => self.decode_array(element_type, *length, bytes, parent_qname, array_depth)?,
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                // Fixed-inlined nested: `bytes` is exactly the target's fixed
                // section (the target is recursively fixed, so no offset table).
                // `PayloadAudit::Frame` regardless of the caller's mode: the
                // slice is already exactly `fl.size == target.fixed_size` bytes
                // and there is no offset table, so the Element audit has
                // nothing to add — while a `fl.size`-vs-`fixed_size` skew
                // between the two nested resolvers would make it reject a
                // sound frame.
                let target = self.resolve_nested(schema_name, package, parent_qname, "<nested>")?;
                FrameValueKind::Nested(Box::new(self.walk_payload(
                    target,
                    bytes,
                    array_depth,
                    PayloadAudit::Frame,
                )?))
            }
            // Variable types never appear in the fixed section.
            FieldType::String | FieldType::Bytes | FieldType::DynamicArray { .. } => {
                FrameValueKind::Bytes(bytes)
            }
        })
    }

    /// Decode a `FixedArray<T>` whose bytes are the whole array slice.
    fn decode_array<'a>(
        &self,
        element_type: &FieldType,
        length: usize,
        bytes: &'a [u8],
        parent_qname: &str,
        array_depth: u32,
    ) -> Result<FrameValueKind<'a>, WalkError> {
        if let Some(pt) = prim_type_of(element_type) {
            return Ok(FrameValueKind::PrimArray(PrimArray {
                elem: pt,
                bytes,
                count: length,
            }));
        }
        match element_type {
            FieldType::U8 | FieldType::Bool => Ok(FrameValueKind::Bytes(bytes)),
            FieldType::I8 => Ok(FrameValueKind::Bytes(bytes)),
            FieldType::Nested {
                fixed: Some(info), ..
            } => {
                // Array of fixed-nested: split into `length` equal strides
                // and decode each element.
                let stride = info.fixed_size;
                let mut out = Vec::with_capacity(length);
                for i in 0..length {
                    let s = i.checked_mul(stride);
                    let e = s.and_then(|s| s.checked_add(stride));
                    let elem = s.zip(e).and_then(|(s, e)| bytes.get(s..e)).ok_or_else(|| {
                        WalkError::FixedFieldOutOfBounds {
                            field: "<fixed-array-element>".to_string(),
                            offset: i.saturating_mul(stride),
                            size: stride,
                            payload_len: bytes.len(),
                        }
                    })?;
                    out.push(self.decode_fixed(element_type, elem, parent_qname, array_depth)?);
                }
                Ok(FrameValueKind::Array(out))
            }
            // FixedArray of a variable element is itself variable and never
            // reaches the fixed-section decode path; treat any other element
            // conservatively as opaque bytes.
            _ => Ok(FrameValueKind::NestedArrayOpaque(bytes)),
        }
    }

    /// Decode one variable (offset-table) field. `bytes` is the field's
    /// payload slice (possibly empty).
    fn decode_variable<'a>(
        &self,
        ft: &FieldType,
        bytes: &'a [u8],
        parent_qname: &str,
        array_depth: u32,
        audit: PayloadAudit,
    ) -> Result<FrameValueKind<'a>, WalkError> {
        Ok(match ft {
            FieldType::String => decode_str_or_bytes(bytes),
            FieldType::Bytes => FrameValueKind::Bytes(bytes),
            FieldType::DynamicArray { element_type } => match element_type.as_ref() {
                FieldType::U8 | FieldType::I8 => FrameValueKind::Bytes(bytes),
                other => {
                    if let Some(pt) = prim_type_of(other) {
                        // `PrimType::size()` is never 0 (min 2), but clippy's
                        // manual-checked-division lint (and defense-in-depth)
                        // want the checked form.
                        let count = bytes.len().checked_div(pt.size()).unwrap_or(0);
                        FrameValueKind::PrimArray(PrimArray {
                            elem: pt,
                            bytes,
                            count,
                        })
                    } else {
                        // `Nested[]` / `string[]` under the canonical
                        // element framing; degrades to
                        // `NestedArrayOpaque` when the bytes do not satisfy it.
                        self.decode_element_array(other, None, bytes, parent_qname, array_depth)
                    }
                }
            },
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                let target = self.resolve_nested(schema_name, package, parent_qname, "<nested>")?;
                if bytes.is_empty() && audit == PayloadAudit::Frame {
                    // The `set_<f>_bytes(&[])` producer idiom: an
                    // INTENTIONALLY empty nested field (e.g. the empty
                    // `Header` every in-repo Image producer writes).
                    // Decodes as a PRESENT nested value with zero fields —
                    // distinct from a truncated sub-frame (any NON-empty
                    // blob shorter than the target's fixed section + offset
                    // table still errors via `walk_payload`'s checks).
                    //
                    // FRAME-ONLY: the idiom is a
                    // generated-WRITER convention. `CdrCodec::decode_payload`
                    // — the only producer of canonical element bodies — always
                    // emits a nested field's full `[fixed][table]`, so a
                    // zero-length nested entry inside an element body is NOT
                    // canonical and must degrade the array rather than
                    // fabricate a present-but-empty value.
                    FrameValueKind::Nested(Box::new(FrameValue {
                        schema_name: target.qualified_name.clone(),
                        fields: Vec::new(),
                    }))
                } else {
                    // Variable nested: `bytes` is a headerless sub-frame of
                    // the target ([fixed][table][var]). The audit mode is
                    // INHERITED — a sub-frame inside an element body is held to
                    // the element's exact-accounting rule (the module docs'
                    // "recursively strict"), one inside a frame is not.
                    FrameValueKind::Nested(Box::new(self.walk_payload(
                        target,
                        bytes,
                        array_depth,
                        audit,
                    )?))
                }
            }
            // A FixedArray-of-variable-element is itself variable and lands
            // here. Its Cerulion bytes STILL carry the `u32 count` (only the
            // CDR-side count is suppressed for a fixed array), so it decodes
            // through the same canonical path with the declared length as an
            // extra requirement.
            FieldType::FixedArray {
                element_type,
                length,
            } => self.decode_element_array(
                element_type,
                Some(*length),
                bytes,
                parent_qname,
                array_depth,
            ),
            // Fixed primitives never land in the variable section.
            _ => FrameValueKind::Bytes(bytes),
        })
    }

    /// Decode a `Nested[]` / `Nested[N]` / `string[]` variable payload under
    /// the CANONICAL ELEMENT FRAMING (see the module docs for the
    /// three encodings and [`CdrCodec`](crate::codegen::CdrCodec) for the
    /// producer side).
    ///
    /// `declared_length` is `Some(N)` for a `FixedArray` (whose Cerulion bytes
    /// still carry the `u32 count` — only the CDR-side count is suppressed —
    /// so the count must equal `N`) and `None` for a `DynamicArray`.
    ///
    /// TOTAL: returns [`FrameValueKind::NestedArrayOpaque`] on ANY
    /// inconsistency instead of erroring or fabricating elements. It is the
    /// caller's already-bounds-checked field slice that is handed back, so a
    /// degrade loses nothing an older consumer had.
    fn decode_element_array<'a>(
        &self,
        element_type: &FieldType,
        declared_length: Option<usize>,
        bytes: &'a [u8],
        parent_qname: &str,
        array_depth: u32,
    ) -> FrameValueKind<'a> {
        // An EMPTY blob is the `set_<f>_bytes(&[])` producer idiom for an
        // intentionally empty complex field (the same rule the single
        // variable-nested arm applies) → zero elements, distinct from opaque.
        // Checked FIRST so it is also the answer for a fieldless (stride-0)
        // element schema, where no count/stride arithmetic is possible — but
        // NOT ahead of a `FixedArray`'s declared arity: an
        // `N > 0` array whose blob is empty has a KNOWABLY wrong arity, so
        // reporting a successful zero-element decode there would fabricate the
        // one fact the declared length pins.
        if bytes.is_empty() {
            return match declared_length {
                Some(n) if n != 0 => FrameValueKind::NestedArrayOpaque(bytes),
                _ => FrameValueKind::NestedArray {
                    elements: Vec::new(),
                    raw: bytes,
                },
            };
        }
        // Explicit recursion bound. Element bodies are strictly shorter than
        // the parent blob (each level costs at least a count + a length
        // prefix), so termination never depends on this — it is a cheap,
        // total answer to "how deep can a hostile frame drive the walker".
        if array_depth >= MAX_NESTED_ARRAY_DEPTH {
            return FrameValueKind::NestedArrayOpaque(bytes);
        }
        match element_type {
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                // Unresolvable element schema → opaque (the walker cannot know
                // the element shape, so it must not guess one).
                let Some(target) = self.lookup_nested(schema_name, package, parent_qname) else {
                    return FrameValueKind::NestedArrayOpaque(bytes);
                };
                if target.is_fixed() {
                    self.decode_fixed_stride_elements(target, declared_length, bytes, array_depth)
                } else {
                    self.decode_counted_elements(
                        CountedElement::SubFrame(target),
                        declared_length,
                        bytes,
                        array_depth,
                    )
                }
            }
            FieldType::String => self.decode_counted_elements(
                CountedElement::Utf8,
                declared_length,
                bytes,
                array_depth,
            ),
            // `DynamicArray<DynamicArray<..>>`, `StringFixed` elements, and any
            // other element shape canonical v1 does not define: opaque.
            _ => FrameValueKind::NestedArrayOpaque(bytes),
        }
    }

    /// The COUNTED element cases — a VARIABLE nested element or a `string[]`:
    /// `u32 count` + per element (`u32 len`, `len` body bytes).
    ///
    /// Strictness, in the order it is applied (any failure ⇒ opaque):
    ///
    /// 1. the `u32 count` must be readable;
    /// 2. `count` is checked against the remaining byte budget BEFORE the
    ///    `Vec` is sized from it — every element costs at least its own 4-byte
    ///    length prefix, so `count > remaining / 4` is impossible (this is the
    ///    guard that makes a hostile `0xFFFFFFFF` prefix cheap, mirroring
    ///    `cdr_codec`'s `guard_count`);
    /// 3. a `FixedArray`'s `declared_length` must equal `count`;
    /// 4. every per-element `len` must be readable and in bounds;
    /// 5. every element's OWN internal accounting must be exact
    ///    ([`PayloadAudit::Element`]);
    /// 6. the cursor must land EXACTLY on the end of the blob.
    ///
    /// Rules 5 and 6 together are the discriminator against a bespoke element
    /// convention decoding by accident, and NEITHER is sufficient alone:
    /// `cerulion_viz::go2_tf`'s 84-byte `/tf` blob satisfies rule 6 whenever
    /// its `stamp.sec` happens to equal 76 (the field sits exactly where the
    /// canonical per-element `u32 len` does), and the resulting 76-byte body
    /// clears `TransformStamped`'s `56 + 16` floor with 4 bytes to spare — it
    /// is rule 5 that refuses it (its offset-table entries land in the identity
    /// quaternion's zero bytes and read `(0, 0)`, below the body's data floor;
    /// even the canonical `(72, 0)` pair would leave 4 bytes unaccounted).
    fn decode_counted_elements<'a>(
        &self,
        element: CountedElement<'_>,
        declared_length: Option<usize>,
        bytes: &'a [u8],
        array_depth: u32,
    ) -> FrameValueKind<'a> {
        let opaque = FrameValueKind::NestedArrayOpaque(bytes);
        let mut cur = ElementCursor::new(bytes);
        let Some(count) = cur.read_u32() else {
            return opaque;
        };
        let count = count as usize;
        // Budget check BEFORE the allocation (rule 2).
        if count > cur.remaining() / 4 {
            return opaque;
        }
        if declared_length.is_some_and(|n| n != count) {
            return opaque;
        }
        let mut out = Vec::with_capacity(count.min(ELEMENT_PREALLOC_CAP));
        for _ in 0..count {
            let Some(len) = cur.read_u32() else {
                return opaque;
            };
            let Some(body) = cur.take(len as usize) else {
                return opaque;
            };
            match element {
                // An empty UTF-8 element is a legitimate empty string (CDR
                // `u32(1) | NUL` carries zero content bytes).
                CountedElement::Utf8 => out.push(decode_str_or_bytes(body)),
                CountedElement::SubFrame(target) => {
                    if body.is_empty() {
                        // A VARIABLE element schema has at least one
                        // offset-table entry, so its sub-frame is never
                        // shorter than 8 bytes — a zero-length element body
                        // cannot come from the canonical encoding.
                        //
                        // SCOPE: this is a LOCAL statement of
                        // the rule, not the enforcement. `walk_payload` refuses
                        // an empty body on its own (a `SubFrame` target has
                        // `variable_fields` non-empty ⇒
                        // `OffsetTableTruncated`), so deleting this guard
                        // changes no output — the `Err(_) => opaque` arm below
                        // catches the same blob. Kept because the reason is
                        // not obvious at the `Err` arm, and because it holds
                        // even if a future resolver skew ever handed this path
                        // a fixed target.
                        return opaque;
                    }
                    match self.walk_payload(target, body, array_depth + 1, PayloadAudit::Element) {
                        Ok(fv) => out.push(FrameValueKind::Nested(Box::new(fv))),
                        // ALL-OR-NOTHING (see `decode_fixed_stride_elements`).
                        Err(_) => return opaque,
                    }
                }
            }
        }
        // Exact consumption (rule 5).
        if !cur.is_exhausted() {
            return opaque;
        }
        FrameValueKind::NestedArray {
            elements: out,
            raw: bytes,
        }
    }

    /// The recursively-FIXED element case: back-to-back fixed sections at
    /// `stride = target.fixed_size`, NO count prefix — `count` is recovered as
    /// `bytes.len() / stride`, which makes `bytes.len() % stride == 0` the
    /// EXACT-CONSUMPTION requirement (the strongest discriminator available
    /// without a count, and the same arithmetic
    /// [`CdrCodec::encode`](crate::codegen::CdrCodec) validates on this
    /// encoding).
    fn decode_fixed_stride_elements<'a>(
        &self,
        target: &WireLayout,
        declared_length: Option<usize>,
        bytes: &'a [u8],
        array_depth: u32,
    ) -> FrameValueKind<'a> {
        let stride = target.fixed_size;
        // stride == 0 (a fieldless element schema, e.g. `std_msgs/Empty[]`)
        // makes the count unrecoverable — and is the division-by-zero guard.
        if stride == 0 || !bytes.len().is_multiple_of(stride) {
            return FrameValueKind::NestedArrayOpaque(bytes);
        }
        // `count <= bytes.len()` (stride >= 1), so the element loop below is
        // bounded by the already-validated field slice.
        let count = bytes.len() / stride;
        if declared_length.is_some_and(|n| n != count) {
            return FrameValueKind::NestedArrayOpaque(bytes);
        }
        let mut out = Vec::with_capacity(count.min(ELEMENT_PREALLOC_CAP));
        for i in 0..count {
            let elem = &bytes[i * stride..(i + 1) * stride];
            // A fixed element schema has no offset table, so its body cannot
            // hold another array — the depth counter does not advance here.
            // The Element audit reduces to `elem.len() == target.fixed_size`,
            // which the stride arithmetic already guarantees; passed for
            // uniformity (an element is audited as an element).
            match self.walk_payload(target, elem, array_depth, PayloadAudit::Element) {
                Ok(fv) => out.push(FrameValueKind::Nested(Box::new(fv))),
                // ALL-OR-NOTHING: a half-decoded array is exactly the
                // fabricated value the walker forbids.
                Err(_) => return FrameValueKind::NestedArrayOpaque(bytes),
            }
        }
        FrameValueKind::NestedArray {
            elements: out,
            raw: bytes,
        }
    }

    /// Resolve a nested reference to its target layout using the SAME
    /// precedence as [`LayoutResolver`]: qualified → same-package →
    /// bare `Header`→`std_msgs/Header` → unambiguous bare suffix.
    fn resolve_nested(
        &self,
        schema_name: &str,
        package: &Option<String>,
        parent_qname: &str,
        field: &str,
    ) -> Result<&WireLayout, WalkError> {
        let target = self.lookup_nested(schema_name, package, parent_qname);
        target.ok_or_else(|| WalkError::NestedResolutionFailed {
            field: field.to_string(),
            target: match package {
                Some(pkg) => format!("{pkg}/{schema_name}"),
                None => schema_name.to_string(),
            },
        })
    }

    fn lookup_nested(
        &self,
        schema_name: &str,
        package: &Option<String>,
        parent_qname: &str,
    ) -> Option<&WireLayout> {
        if let Some(pkg) = package {
            return self.layouts.get(&format!("{pkg}/{schema_name}"));
        }
        // Same package as the parent (or bare key for a package-less parent).
        match parent_qname.rsplit_once('/') {
            Some((parent_pkg, _)) => {
                if let Some(l) = self.layouts.get(&format!("{parent_pkg}/{schema_name}")) {
                    return Some(l);
                }
            }
            None => {
                if let Some(l) = self.layouts.get(schema_name) {
                    return Some(l);
                }
            }
        }
        // Bare `Header` → std_msgs (rosidl legacy special case).
        if schema_name == "Header" {
            if let Some(l) = self.layouts.get("std_msgs/Header") {
                return Some(l);
            }
        }
        // Unambiguous bare-suffix fallback.
        let mut it = self
            .layouts
            .iter()
            .filter(|(k, _)| k.rsplit('/').next() == Some(schema_name));
        match (it.next(), it.next()) {
            (Some((_, l)), None) => Some(l),
            _ => None,
        }
    }
}

/// How deep canonically-framed element ARRAYS may nest before the walker
/// stops decoding and hands the bytes back opaque. Element bodies
/// shrink strictly at every level, so termination does not rest on this — it
/// is a cheap, explicit ceiling on how much work an adversarial frame can
/// drive THROUGH THE ARRAY PATH. 8 is far past any real ROS 2 shape
/// (`Detection3DArray` → `detections[]` → `results[]` is 2).
///
/// SCOPE: only [`FrameWalker::decode_element_array`] consults it. The
/// SINGULAR variable-nested arm (`decode_variable`'s `Nested` case) recurses
/// through `walk_payload` at the same depth with no cap, so a *recursive*
/// schema (`Node { Node child, string tag }`) can still drive walker recursion
/// as deep as the frame's bytes allow. That hole predates element framing and is not
/// widened by it (the same reachability class — a frame can only drive depth at
/// all under a recursive schema); it is called out here so the cap is not read
/// as a total answer.
const MAX_NESTED_ARRAY_DEPTH: u32 = 8;

/// Upper bound on the element `Vec`'s UP-FRONT capacity.
///
/// The count budget bounds the element COUNT to `remaining / 4`, but
/// `FrameValueKind` is ~40 bytes, so sizing the `Vec` straight from a
/// budget-passing count would reserve up to ~10× the blob length BEFORE the
/// first element is validated — paid even by a blob that immediately degrades
/// to opaque, and an allocation failure aborts the process (which the module's
/// "NEVER panics on adversarial input" contract does not allow). Capping the
/// reservation keeps the common case (any real ROS 2 array) at one allocation
/// while a hostile count pays only amortized growth for elements that actually
/// validate.
const ELEMENT_PREALLOC_CAP: usize = 256;

/// What one element of a COUNTED canonical array
/// (`u32 count` + per element `u32 len` + body) decodes into.
#[derive(Debug, Clone, Copy)]
enum CountedElement<'w> {
    /// `string[]` — the element body is UTF-8 content (no NUL).
    Utf8,
    /// `Nested[]` with a VARIABLE element — the body is a headerless
    /// sub-frame of `'w`'s layout.
    SubFrame(&'w WireLayout),
}

/// A tiny bounds-checked cursor over a canonical element-array payload.
///
/// Deliberately NOT shared with `cdr_codec`'s `ComplexReader`, which reads the
/// same bytes: the two have opposite ERROR POLICIES (the codec returns a loud
/// `Err` for a frame it was told to transcode; the walker degrades a single
/// field to opaque and keeps decoding the rest of the message). Fusing them
/// would force one policy on both.
struct ElementCursor<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> ElementCursor<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, pos: 0 }
    }

    /// Bytes not yet consumed.
    fn remaining(&self) -> usize {
        self.bytes.len().saturating_sub(self.pos)
    }

    fn is_exhausted(&self) -> bool {
        self.pos == self.bytes.len()
    }

    fn read_u32(&mut self) -> Option<u32> {
        let s = self.take(4)?;
        Some(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
    }

    /// `n` bytes, or `None` if that would run past the end (checked add — a
    /// hostile `u32` length can never wrap the cursor).
    fn take(&mut self, n: usize) -> Option<&'a [u8]> {
        let end = self.pos.checked_add(n)?;
        let slice = self.bytes.get(self.pos..end)?;
        self.pos = end;
        Some(slice)
    }
}

/// The [`PrimType`] for a numeric-primitive field type (excluding `u8`/`i8`,
/// which are surfaced as raw [`FrameValueKind::Bytes`]). `None` for
/// non-numeric types.
fn prim_type_of(ft: &FieldType) -> Option<PrimType> {
    Some(match ft {
        FieldType::I16 => PrimType::I16,
        FieldType::U16 => PrimType::U16,
        FieldType::I32 => PrimType::I32,
        FieldType::U32 => PrimType::U32,
        FieldType::I64 => PrimType::I64,
        FieldType::U64 => PrimType::U64,
        FieldType::F32 => PrimType::F32,
        FieldType::F64 => PrimType::F64,
        _ => return None,
    })
}

/// Trim a fixed-string slice at its first NUL byte (C-string semantics).
fn trim_at_nul(bytes: &[u8]) -> &[u8] {
    match bytes.iter().position(|&b| b == 0) {
        Some(n) => &bytes[..n],
        None => bytes,
    }
}

/// UTF-8-validate a byte slice into [`FrameValueKind::Str`], degrading to
/// [`FrameValueKind::Bytes`] on invalid UTF-8 (never erroring the walk — the
/// bytes are real, only the `&str` view failed).
fn decode_str_or_bytes(bytes: &[u8]) -> FrameValueKind<'_> {
    match std::str::from_utf8(bytes) {
        Ok(s) => FrameValueKind::Str(s),
        Err(_) => FrameValueKind::Bytes(bytes),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codegen::schema::{FieldDef, FieldType, MessageSchema};
    use crate::shm_runtime::write_offset_entry;
    use crate::wire::WireHeader;

    // ---- Frame-building test helpers (independent of the writer path) ----
    //
    // These lay out `[WireHeader][fixed][table][var]` by hand from KNOWN
    // offsets so the walker is checked against an oracle, not a re-run of the
    // production writer.

    /// Prepend a 32-byte WireHeader (schema_hash only — the walker does not
    /// require the counts for `walk`) to a hand-built payload.
    fn frame(schema_hash: u64, payload: &[u8]) -> Vec<u8> {
        let mut f = vec![0u8; WireHeader::SIZE];
        let header = WireHeader::with_schema(schema_hash);
        header.write_to_buf(&mut f);
        f.extend_from_slice(payload);
        f
    }

    fn vec3_schema() -> MessageSchema {
        let mut s = MessageSchema::new_in_package("Vec3", "geometry_msgs");
        for n in ["x", "y", "z"] {
            s.add_field(FieldDef::new(n, FieldType::F64));
        }
        s
    }

    #[test]
    fn walks_a_fixed_only_frame() {
        let (walker, warns) = FrameWalker::new(vec![vec3_schema()]);
        assert!(warns.is_empty());

        // Fixed section: x=1.0, y=2.0, z=3.0 (three f64 LE, 24 bytes).
        let mut payload = Vec::new();
        payload.extend_from_slice(&1.0f64.to_le_bytes());
        payload.extend_from_slice(&2.0f64.to_le_bytes());
        payload.extend_from_slice(&3.0f64.to_le_bytes());
        let f = frame(0, &payload);

        let fv = walker.walk("geometry_msgs/Vec3", &f).expect("walk");
        assert_eq!(fv.schema_name, "geometry_msgs/Vec3");
        assert_eq!(fv.fields.len(), 3);
        assert_eq!(fv.field("x"), Some(&FrameValueKind::F64(1.0)));
        assert_eq!(fv.field("y"), Some(&FrameValueKind::F64(2.0)));
        assert_eq!(fv.field("z"), Some(&FrameValueKind::F64(3.0)));
    }

    /// Image-like: fixed {height:u32, width:u32} + variable {encoding:string,
    /// data:bytes}. Fixed size = 8; offset table starts at payload[8], two
    /// entries (16 B), variable data starts at payload[24].
    fn imglike_schema() -> MessageSchema {
        let mut s = MessageSchema::new_in_package("Imglike", "test_msgs");
        s.add_field(FieldDef::new("height", FieldType::U32));
        s.add_field(FieldDef::new("width", FieldType::U32));
        s.add_field(FieldDef::new("encoding", FieldType::String));
        s.add_field(FieldDef::new("data", FieldType::Bytes));
        s
    }

    #[test]
    fn walks_variable_string_and_bytes() {
        let (walker, _) = FrameWalker::new(vec![imglike_schema()]);

        let encoding = b"rgb8";
        let data = b"\x01\x02\x03\x04\x05";
        // Layout: fixed(8) + table(16) = 24. encoding at 24 (len 4), data at
        // 28 (len 5).
        let fixed_size = 8usize;
        let table_bytes = 16usize;
        let enc_off = (fixed_size + table_bytes) as u32; // 24
        let data_off = enc_off + encoding.len() as u32; // 28

        let mut payload = vec![0u8; fixed_size + table_bytes];
        payload[0..4].copy_from_slice(&480u32.to_le_bytes()); // height
        payload[4..8].copy_from_slice(&640u32.to_le_bytes()); // width
        write_offset_entry(&mut payload, fixed_size, 0, enc_off, encoding.len() as u32);
        write_offset_entry(&mut payload, fixed_size, 1, data_off, data.len() as u32);
        payload.extend_from_slice(encoding);
        payload.extend_from_slice(data);

        let f = frame(0, &payload);
        let fv = walker.walk("test_msgs/Imglike", &f).expect("walk");
        assert_eq!(fv.field("height"), Some(&FrameValueKind::U32(480)));
        assert_eq!(fv.field("width"), Some(&FrameValueKind::U32(640)));
        assert_eq!(fv.field("encoding"), Some(&FrameValueKind::Str("rgb8")));
        assert_eq!(
            fv.field("data"),
            Some(&FrameValueKind::Bytes(&[1, 2, 3, 4, 5]))
        );
    }

    #[test]
    fn empty_variable_field_is_empty_not_error() {
        let (walker, _) = FrameWalker::new(vec![imglike_schema()]);
        // Both offset-table entries left (0,0): empty string + empty bytes.
        let payload = vec![0u8; 8 + 16];
        let f = frame(0, &payload);
        let fv = walker.walk("test_msgs/Imglike", &f).expect("walk");
        assert_eq!(fv.field("encoding"), Some(&FrameValueKind::Str("")));
        assert_eq!(fv.field("data"), Some(&FrameValueKind::Bytes(&[])));
    }

    #[test]
    fn walks_dynamic_array_of_primitives() {
        let mut s = MessageSchema::new_in_package("Vals", "test_msgs");
        s.add_field(FieldDef::new(
            "vals",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::F64),
            },
        ));
        let (walker, _) = FrameWalker::new(vec![s]);

        // All-variable schema: fixed_size 0, one table entry (8 B), data at 8.
        let vals = [1.5f64, -2.5, 3.25];
        let mut payload = vec![0u8; 8];
        let data_off = 8u32;
        let mut data = Vec::new();
        for v in vals {
            data.extend_from_slice(&v.to_le_bytes());
        }
        write_offset_entry(&mut payload, 0, 0, data_off, data.len() as u32);
        payload.extend_from_slice(&data);

        let f = frame(0, &payload);
        let fv = walker.walk("test_msgs/Vals", &f).expect("walk");
        match fv.field("vals") {
            Some(FrameValueKind::PrimArray(pa)) => {
                assert_eq!(pa.elem, PrimType::F64);
                assert_eq!(pa.count, 3);
                let got: Vec<f64> = pa.iter_f64().collect();
                assert_eq!(got, vec![1.5, -2.5, 3.25]);
            }
            other => panic!("expected PrimArray, got {other:?}"),
        }
    }

    #[test]
    fn walks_fixed_nested() {
        // Pose { position: geometry_msgs/Vec3 } — Vec3 is recursively fixed,
        // so it inlines at offset 0 (24 bytes).
        let mut pose = MessageSchema::new_in_package("PoseLike", "test_msgs");
        pose.add_field(FieldDef::new(
            "position",
            FieldType::Nested {
                schema_name: "Vec3".into(),
                package: Some("geometry_msgs".into()),
                fixed: None,
            },
        ));
        let (walker, warns) = FrameWalker::new(vec![vec3_schema(), pose]);
        assert!(warns.is_empty(), "{warns:?}");

        let mut payload = Vec::new();
        payload.extend_from_slice(&7.0f64.to_le_bytes());
        payload.extend_from_slice(&8.0f64.to_le_bytes());
        payload.extend_from_slice(&9.0f64.to_le_bytes());
        let f = frame(0, &payload);

        let fv = walker.walk("test_msgs/PoseLike", &f).expect("walk");
        match fv.field("position") {
            Some(FrameValueKind::Nested(inner)) => {
                assert_eq!(inner.schema_name, "geometry_msgs/Vec3");
                assert_eq!(inner.field("x"), Some(&FrameValueKind::F64(7.0)));
                assert_eq!(inner.field("z"), Some(&FrameValueKind::F64(9.0)));
            }
            other => panic!("expected Nested, got {other:?}"),
        }
    }

    #[test]
    fn walks_variable_nested_subframe() {
        // Outer { label: string, inner: InnerVar } where InnerVar has a
        // string (so it is variable). The nested field's bytes are a
        // headerless sub-frame [fixed][table][var] of InnerVar.
        let mut inner = MessageSchema::new_in_package("InnerVar", "test_msgs");
        inner.add_field(FieldDef::new("id", FieldType::U32));
        inner.add_field(FieldDef::new("name", FieldType::String));

        let mut outer = MessageSchema::new_in_package("OuterVar", "test_msgs");
        outer.add_field(FieldDef::new("label", FieldType::String));
        outer.add_field(FieldDef::new(
            "inner",
            FieldType::Nested {
                schema_name: "InnerVar".into(),
                package: Some("test_msgs".into()),
                fixed: None,
            },
        ));
        let (walker, _) = FrameWalker::new(vec![inner, outer]);

        // Build the InnerVar sub-frame: fixed {id:u32}=4, table(8)=... one var
        // field `name`. fixed_size 4, table 8 => data floor 12.
        let name = b"lidar";
        let mut sub = vec![0u8; 4 + 8];
        sub[0..4].copy_from_slice(&42u32.to_le_bytes());
        write_offset_entry(&mut sub, 4, 0, 12, name.len() as u32);
        sub.extend_from_slice(name);

        // Build OuterVar: all-variable (label string + inner nested-var) →
        // fixed_size 0, table 16 (2 entries), data floor 16.
        let label = b"cloud";
        let mut payload = vec![0u8; 16];
        let label_off = 16u32;
        let inner_off = label_off + label.len() as u32;
        write_offset_entry(&mut payload, 0, 0, label_off, label.len() as u32);
        write_offset_entry(&mut payload, 0, 1, inner_off, sub.len() as u32);
        payload.extend_from_slice(label);
        payload.extend_from_slice(&sub);

        let f = frame(0, &payload);
        let fv = walker.walk("test_msgs/OuterVar", &f).expect("walk");
        assert_eq!(fv.field("label"), Some(&FrameValueKind::Str("cloud")));
        match fv.field("inner") {
            Some(FrameValueKind::Nested(inner)) => {
                assert_eq!(inner.field("id"), Some(&FrameValueKind::U32(42)));
                assert_eq!(inner.field("name"), Some(&FrameValueKind::Str("lidar")));
            }
            other => panic!("expected Nested, got {other:?}"),
        }
    }

    #[test]
    fn dynamic_array_of_nested_with_non_canonical_bytes_is_opaque() {
        // Fields { entries: SomeMsg[] } where SomeMsg is variable. Note: the
        // walker DOES decode the canonical element framing for this shape, so
        // the reason these 4 junk bytes stay opaque is now specific — the
        // leading `u32` reads as a count far beyond the remaining byte budget,
        // which the pre-allocation guard refuses.
        let mut some = MessageSchema::new_in_package("SomeMsg", "test_msgs");
        some.add_field(FieldDef::new("name", FieldType::String));
        let mut holder = MessageSchema::new_in_package("Holder", "test_msgs");
        holder.add_field(FieldDef::new(
            "entries",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::Nested {
                    schema_name: "SomeMsg".into(),
                    package: Some("test_msgs".into()),
                    fixed: None,
                }),
            },
        ));
        let (walker, _) = FrameWalker::new(vec![some, holder]);

        let opaque = b"\xDE\xAD\xBE\xEF";
        let mut payload = vec![0u8; 8];
        write_offset_entry(&mut payload, 0, 0, 8, opaque.len() as u32);
        payload.extend_from_slice(opaque);
        let f = frame(0, &payload);
        let fv = walker.walk("test_msgs/Holder", &f).expect("walk");
        assert_eq!(
            fv.field("entries"),
            Some(&FrameValueKind::NestedArrayOpaque(&[
                0xDE, 0xAD, 0xBE, 0xEF
            ]))
        );
    }

    // ---- Canonical element framing, FIXED-stride elements --------
    //
    // All frames below are hand-built from KNOWN offsets over the REAL
    // built-in corpus (so the strides are production strides), and every
    // asserted value is a hand-written oracle — never a re-encode of the
    // bytes under test. Each builder carries a layout DRIFT GUARD so a
    // codegen layout change fails loudly at the guard rather than silently
    // shifting the oracle.

    /// The layout the walker will decode `qname` with (in-module test access).
    fn layout_of<'w>(w: &'w FrameWalker, qname: &str) -> &'w WireLayout {
        w.layouts
            .get(qname)
            .unwrap_or_else(|| panic!("{qname} missing from the walker's schema set"))
    }

    /// Assert `qname`'s payload shape, then return `(fixed_size, table_bytes)`.
    fn assert_shape(
        w: &FrameWalker,
        qname: &str,
        fixed_size: usize,
        var_names: &[&str],
    ) -> (usize, usize) {
        let l = layout_of(w, qname);
        assert_eq!(l.fixed_size, fixed_size, "{qname} fixed_size drifted");
        let got: Vec<&str> = l.variable_fields.iter().map(|v| v.name.as_str()).collect();
        assert_eq!(got, var_names, "{qname} variable fields drifted");
        (l.fixed_size, l.offset_table_bytes())
    }

    /// One `geometry_msgs/Pose` fixed section, hand-laid: `Point{x,y,z}` then
    /// `Quaternion{x,y,z,w}`, seven f64 LE = 56 bytes.
    fn pose_fixed_section(pos: [f64; 3], quat: [f64; 4]) -> Vec<u8> {
        let mut v = Vec::with_capacity(56);
        for c in pos {
            v.extend_from_slice(&c.to_le_bytes());
        }
        for c in quat {
            v.extend_from_slice(&c.to_le_bytes());
        }
        assert_eq!(v.len(), 56);
        v
    }

    /// Build a `geometry_msgs/PoseArray` frame carrying `poses_blob` verbatim
    /// and an intentionally-empty `header` (entry `(0, 0)`).
    fn pose_array_frame(w: &FrameWalker, poses_blob: &[u8]) -> Vec<u8> {
        // Header is variable and Pose[] is variable → fixed_size 0, table 16,
        // variable data floor 16.
        let (fixed_size, table_bytes) =
            assert_shape(w, "geometry_msgs/PoseArray", 0, &["header", "poses"]);
        assert_eq!((fixed_size, table_bytes), (0, 16));
        assert_eq!(
            layout_of(w, "geometry_msgs/Pose").fixed_size,
            56,
            "Pose stride drifted"
        );
        let mut payload = vec![0u8; fixed_size + table_bytes];
        // entry 0 = `header`: left (0, 0) — an UNTOUCHED table slot, which
        // the walker reads as a zero-length payload. (This is NOT
        // what `set_<f>_bytes(&[])` writes — that records the running cursor
        // and emits `(cursor, 0)`. A real producer's frame never carries a
        // (0, 0) entry: an untouched variable field is refused at publish by
        // the unwritten-field gate. This arm pins the walker's DEFENSIVE
        // handling of a slot nothing wrote.)
        write_offset_entry(
            &mut payload,
            fixed_size,
            1,
            (fixed_size + table_bytes) as u32,
            poses_blob.len() as u32,
        );
        payload.extend_from_slice(poses_blob);
        frame(0, &payload)
    }

    /// Build a `geometry_msgs/Polygon` frame carrying `points_blob` verbatim.
    fn polygon_frame(w: &FrameWalker, points_blob: &[u8]) -> Vec<u8> {
        let (fixed_size, table_bytes) = assert_shape(w, "geometry_msgs/Polygon", 0, &["points"]);
        assert_eq!((fixed_size, table_bytes), (0, 8));
        assert_eq!(
            layout_of(w, "geometry_msgs/Point32").fixed_size,
            12,
            "Point32 stride drifted"
        );
        let mut payload = vec![0u8; fixed_size + table_bytes];
        write_offset_entry(
            &mut payload,
            fixed_size,
            0,
            (fixed_size + table_bytes) as u32,
            points_blob.len() as u32,
        );
        payload.extend_from_slice(points_blob);
        frame(0, &payload)
    }

    fn expect_nested_array<'v, 'a>(
        v: Option<&'v FrameValueKind<'a>>,
        what: &str,
    ) -> &'v [FrameValueKind<'a>] {
        match v {
            Some(FrameValueKind::NestedArray { elements, .. }) => elements,
            other => panic!("expected NestedArray for {what}, got {other:?}"),
        }
    }

    fn expect_nested<'v, 'a>(v: &'v FrameValueKind<'a>, what: &str) -> &'v FrameValue<'a> {
        match v {
            FrameValueKind::Nested(inner) => inner,
            other => panic!("expected Nested for {what}, got {other:?}"),
        }
    }

    /// #1 — `geometry_msgs/PoseArray`: three back-to-back 56-byte `Pose`
    /// sections (stride form, NO count prefix) decode element-by-element into
    /// the full recursive tree, every leaf equal to a hand-written value.
    #[test]
    fn pose_array_fixed_stride_elements_decode_to_hand_oracle() {
        let walker = builtin_walker_for_test();
        // Hand oracle: three distinct poses (asymmetric across x/y/z and the
        // quaternion, so a swapped/short span shows up as a wrong value).
        let oracle: [([f64; 3], [f64; 4]); 3] = [
            ([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]),
            ([-4.5, 5.25, 6.125], [0.5, -0.5, 0.25, 0.75]),
            ([7.0, 8.0, 9.0], [1.0, 2.0, 3.0, 4.0]),
        ];
        let mut blob = Vec::new();
        for (pos, quat) in oracle {
            blob.extend_from_slice(&pose_fixed_section(pos, quat));
        }
        assert_eq!(
            blob.len(),
            3 * 56,
            "no count prefix in the fixed-stride form"
        );

        let f = pose_array_frame(&walker, &blob);
        let fv = walker.walk("geometry_msgs/PoseArray", &f).expect("walk");

        let elems = expect_nested_array(fv.field("poses"), "poses");
        assert_eq!(elems.len(), 3, "count == len / stride");
        for (i, (pos, quat)) in oracle.iter().enumerate() {
            let pose = expect_nested(&elems[i], "pose element");
            assert_eq!(pose.schema_name, "geometry_msgs/Pose");
            let position = expect_nested(
                pose.field("position").expect("position present"),
                "position",
            );
            assert_eq!(position.schema_name, "geometry_msgs/Point");
            assert_eq!(position.field("x"), Some(&FrameValueKind::F64(pos[0])));
            assert_eq!(position.field("y"), Some(&FrameValueKind::F64(pos[1])));
            assert_eq!(position.field("z"), Some(&FrameValueKind::F64(pos[2])));
            let orientation = expect_nested(
                pose.field("orientation").expect("orientation present"),
                "orientation",
            );
            assert_eq!(orientation.schema_name, "geometry_msgs/Quaternion");
            assert_eq!(orientation.field("x"), Some(&FrameValueKind::F64(quat[0])));
            assert_eq!(orientation.field("y"), Some(&FrameValueKind::F64(quat[1])));
            assert_eq!(orientation.field("z"), Some(&FrameValueKind::F64(quat[2])));
            assert_eq!(orientation.field("w"), Some(&FrameValueKind::F64(quat[3])));
        }
    }

    /// #1b — a SINGLE element is decoded (not mistaken for a scalar), and the
    /// untouched sibling `header` still reads as a zero-length payload.
    #[test]
    fn single_fixed_stride_element_decodes_and_sibling_is_unaffected() {
        let walker = builtin_walker_for_test();
        let blob = pose_fixed_section([0.5, -1.5, 2.5], [0.0, 0.0, 0.25, 0.875]);
        let f = pose_array_frame(&walker, &blob);
        let fv = walker.walk("geometry_msgs/PoseArray", &f).expect("walk");

        let elems = expect_nested_array(fv.field("poses"), "poses");
        assert_eq!(elems.len(), 1);
        let position = expect_nested(
            expect_nested(&elems[0], "pose")
                .field("position")
                .expect("position"),
            "position",
        );
        assert_eq!(position.field("y"), Some(&FrameValueKind::F64(-1.5)));

        let header = expect_nested(fv.field("header").expect("header present"), "header");
        assert_eq!(header.schema_name, "std_msgs/Header");
        assert!(
            header.fields.is_empty(),
            "empty-blob nested idiom preserved"
        );
    }

    /// #2 — `geometry_msgs/Polygon`: two 12-byte `Point32` elements, exact
    /// f32 oracle (a narrower stride than Pose, so the stride really comes
    /// from the element layout and is not hardcoded).
    #[test]
    fn polygon_point32_stride_decodes_exact_f32_oracle() {
        let walker = builtin_walker_for_test();
        let oracle: [[f32; 3]; 2] = [[1.5, -2.25, 3.75], [-0.5, 0.125, 64.0]];
        let mut blob = Vec::new();
        for p in oracle {
            for c in p {
                blob.extend_from_slice(&c.to_le_bytes());
            }
        }
        assert_eq!(blob.len(), 2 * 12);

        let f = polygon_frame(&walker, &blob);
        let fv = walker.walk("geometry_msgs/Polygon", &f).expect("walk");
        let elems = expect_nested_array(fv.field("points"), "points");
        assert_eq!(elems.len(), 2);
        for (i, p) in oracle.iter().enumerate() {
            let pt = expect_nested(&elems[i], "point element");
            assert_eq!(pt.schema_name, "geometry_msgs/Point32");
            assert_eq!(pt.field("x"), Some(&FrameValueKind::F32(p[0])));
            assert_eq!(pt.field("y"), Some(&FrameValueKind::F32(p[1])));
            assert_eq!(pt.field("z"), Some(&FrameValueKind::F32(p[2])));
        }
    }

    /// #3 — a RAGGED blob (`len % stride != 0`) must degrade to opaque, never
    /// decode `floor(len / stride)` elements and swallow the remainder. This
    /// is the exact-consumption requirement for the stride form, and the
    /// discriminator against any producer convention whose record size is not
    /// a multiple of the element's padded fixed size.
    #[test]
    fn ragged_stride_blob_degrades_to_opaque() {
        let walker = builtin_walker_for_test();
        // 25 bytes: two whole Point32s (24) + one trailing byte.
        for ragged_len in [1usize, 11, 13, 25] {
            let blob: Vec<u8> = (0..ragged_len).map(|i| (i as u8).wrapping_add(1)).collect();
            let f = polygon_frame(&walker, &blob);
            let fv = walker.walk("geometry_msgs/Polygon", &f).expect("walk");
            assert_eq!(
                fv.field("points"),
                Some(&FrameValueKind::NestedArrayOpaque(blob.as_slice())),
                "ragged blob of {ragged_len} bytes must stay opaque"
            );
        }
    }

    /// #4 — an EMPTY blob is zero elements, DISTINCT from opaque: the
    /// `set_<f>_bytes(&[])` producer idiom for an intentionally empty array.
    /// The variant's `raw` slice is the field's own (empty)
    /// bytes, so a bespoke-convention consumer reading `raw` sees exactly the
    /// "no elements" blob it saw before the walker decoded elements.
    #[test]
    fn empty_element_blob_is_zero_elements_not_opaque() {
        let walker = builtin_walker_for_test();
        let f = polygon_frame(&walker, &[]);
        let fv = walker.walk("geometry_msgs/Polygon", &f).expect("walk");
        assert_eq!(
            fv.field("points"),
            Some(&FrameValueKind::NestedArray {
                elements: Vec::new(),
                raw: &[]
            }),
            "empty blob → zero elements, not opaque"
        );
        // And the same for the wider PoseArray shape (both entries empty).
        let f = pose_array_frame(&walker, &[]);
        let fv = walker.walk("geometry_msgs/PoseArray", &f).expect("walk");
        assert_eq!(
            fv.field("poses"),
            Some(&FrameValueKind::NestedArray {
                elements: Vec::new(),
                raw: &[]
            })
        );
    }

    /// #5 — a FIELDLESS element schema has `stride == 0`, so the count is
    /// unrecoverable: opaque, and no division-by-zero panic. (An empty blob
    /// for the same field is still zero elements — the empty-blob rule is
    /// checked before any stride arithmetic.)
    #[test]
    fn zero_stride_element_schema_degrades_to_opaque_no_panic() {
        // `Nothing` has no fields at all → fixed_size 0, no variable fields ⇒
        // recursively fixed with a zero stride.
        let nothing = MessageSchema::new_in_package("Nothing", "test_msgs");
        let mut holder = MessageSchema::new_in_package("NothingHolder", "test_msgs");
        holder.add_field(FieldDef::new(
            "items",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::Nested {
                    schema_name: "Nothing".into(),
                    package: Some("test_msgs".into()),
                    fixed: None,
                }),
            },
        ));
        let (walker, _) = FrameWalker::new(vec![nothing, holder]);
        {
            let l = layout_of(&walker, "test_msgs/Nothing");
            assert_eq!(l.fixed_size, 0, "fieldless schema has a zero stride");
            assert!(l.is_fixed(), "fieldless schema is recursively fixed");
        }

        // Non-empty blob → opaque (count unrecoverable), no panic.
        let junk = b"\x01\x02\x03";
        let mut payload = vec![0u8; 8];
        write_offset_entry(&mut payload, 0, 0, 8, junk.len() as u32);
        payload.extend_from_slice(junk);
        let f = frame(0, &payload);
        let fv = walker.walk("test_msgs/NothingHolder", &f).expect("walk");
        assert_eq!(
            fv.field("items"),
            Some(&FrameValueKind::NestedArrayOpaque(&junk[..]))
        );

        // Empty blob → zero elements (rule order pin).
        let payload = vec![0u8; 8];
        let f = frame(0, &payload);
        let fv = walker.walk("test_msgs/NothingHolder", &f).expect("walk");
        assert_eq!(
            fv.field("items"),
            Some(&FrameValueKind::NestedArray {
                elements: Vec::new(),
                raw: &[]
            })
        );
    }

    /// #6 — REGRESSION PIN: a FIXED-LENGTH array of fixed-nested elements in
    /// the FIXED section still decodes through the untouched `decode_array`
    /// stride path as [`FrameValueKind::Array`] — element decoding did not widen or
    /// reroute that variant (consumers expand `Array` per index).
    #[test]
    fn fixed_length_nested_array_still_yields_array_variant() {
        let mut pt = MessageSchema::new_in_package("Pt", "test_msgs");
        for n in ["x", "y", "z"] {
            pt.add_field(FieldDef::new(n, FieldType::F64));
        }
        let mut quad = MessageSchema::new_in_package("Quad", "test_msgs");
        quad.add_field(FieldDef::new(
            "corners",
            FieldType::FixedArray {
                element_type: Box::new(FieldType::Nested {
                    schema_name: "Pt".into(),
                    package: Some("test_msgs".into()),
                    fixed: None,
                }),
                length: 4,
            },
        ));
        let (walker, warns) = FrameWalker::new(vec![pt, quad]);
        assert!(warns.is_empty(), "{warns:?}");
        // The whole array is FIXED → it lives in the fixed section (4 × 24 B),
        // no offset table at all.
        let (fixed_size, table_bytes) = assert_shape(&walker, "test_msgs/Quad", 96, &[]);
        assert_eq!((fixed_size, table_bytes), (96, 0));

        let mut payload = Vec::new();
        for i in 0..4u32 {
            for c in 0..3u32 {
                payload.extend_from_slice(&f64::from(i * 10 + c).to_le_bytes());
            }
        }
        let f = frame(0, &payload);
        let fv = walker.walk("test_msgs/Quad", &f).expect("walk");
        match fv.field("corners") {
            Some(FrameValueKind::Array(elems)) => {
                assert_eq!(elems.len(), 4);
                let third = expect_nested(&elems[2], "corners[2]");
                assert_eq!(third.field("x"), Some(&FrameValueKind::F64(20.0)));
                assert_eq!(third.field("z"), Some(&FrameValueKind::F64(22.0)));
            }
            other => panic!("expected the untouched Array variant, got {other:?}"),
        }
    }

    /// #6b — `nav_msgs/GridCells.cells` (`geometry_msgs/Point[]`): the THIRD
    /// fixed-stride shape, which was named as fixed but never tested.
    /// Its `header` entry stays `(0, 0)` — the unwritten-field idiom, which is
    /// legal in a FRAME (unlike inside an element body).
    #[test]
    fn grid_cells_point_stride_decodes_to_hand_oracle() {
        let walker = builtin_walker_for_test();
        let (fixed_size, table_bytes) =
            assert_shape(&walker, "nav_msgs/GridCells", 8, &["header", "cells"]);
        assert_eq!((fixed_size, table_bytes), (8, 16));
        assert_eq!(
            layout_of(&walker, "geometry_msgs/Point").fixed_size,
            24,
            "Point stride drifted"
        );

        let oracle = [[1.0f64, 2.0, 3.0], [-4.5, 5.25, 6.125]];
        let mut blob = Vec::new();
        for p in oracle {
            for c in p {
                blob.extend_from_slice(&c.to_le_bytes());
            }
        }
        let mut payload = vec![0u8; fixed_size + table_bytes];
        payload[0..4].copy_from_slice(&0.5f32.to_le_bytes());
        payload[4..8].copy_from_slice(&0.25f32.to_le_bytes());
        write_offset_entry(
            &mut payload,
            fixed_size,
            1,
            (fixed_size + table_bytes) as u32,
            blob.len() as u32,
        );
        payload.extend_from_slice(&blob);
        let f = frame(0, &payload);

        let fv = walker.walk("nav_msgs/GridCells", &f).expect("walk");
        assert_eq!(fv.field("cell_width"), Some(&FrameValueKind::F32(0.5)));
        assert_eq!(fv.field("cell_height"), Some(&FrameValueKind::F32(0.25)));
        let elems = expect_nested_array(fv.field("cells"), "cells");
        assert_eq!(elems.len(), 2);
        for (i, p) in oracle.iter().enumerate() {
            let pt = expect_nested(&elems[i], "cells element");
            assert_eq!(pt.field("x"), Some(&FrameValueKind::F64(p[0])));
            assert_eq!(pt.field("y"), Some(&FrameValueKind::F64(p[1])));
            assert_eq!(pt.field("z"), Some(&FrameValueKind::F64(p[2])));
        }
    }

    // ---- Canonical element framing, COUNTED elements -------------
    //
    // The `u32 count` + per element (`u32 len`, body) form: a VARIABLE nested
    // element or a `string[]`. Same discipline as above — hand-built frames
    // over the REAL corpus, hand-written oracles, a layout drift guard per
    // builder.

    /// `u32 count` + per element (`u32 len`, body) — the canonical counted
    /// framing, built from element bodies.
    fn counted_blob(elements: &[Vec<u8>]) -> Vec<u8> {
        let mut v = Vec::new();
        v.extend_from_slice(&(elements.len() as u32).to_le_bytes());
        for e in elements {
            v.extend_from_slice(&(e.len() as u32).to_le_bytes());
            v.extend_from_slice(e);
        }
        v
    }

    /// A `std_msgs/Header` sub-frame: fixed `Time{sec, nanosec}` (8 B) |
    /// entry[0] `frame_id` (8 B) | UTF-8. Length = 16 + frame_id.len().
    fn header_body(sec: i32, nanosec: u32, frame_id: &str) -> Vec<u8> {
        let mut v = vec![0u8; 16];
        v[0..4].copy_from_slice(&sec.to_le_bytes());
        v[4..8].copy_from_slice(&nanosec.to_le_bytes());
        write_offset_entry(&mut v, 8, 0, 16, frame_id.len() as u32);
        v.extend_from_slice(frame_id.as_bytes());
        v
    }

    /// Assert the `std_msgs/Header` shape [`header_body`] hand-lays.
    fn assert_header_shape(w: &FrameWalker) {
        let (fixed_size, table_bytes) = assert_shape(w, "std_msgs/Header", 8, &["frame_id"]);
        assert_eq!((fixed_size, table_bytes), (8, 8));
    }

    /// A `geometry_msgs/PoseStamped` element body: fixed `pose` (56 B) |
    /// entry[0] `header` (8 B) | the header sub-frame.
    fn pose_stamped_body(
        pos: [f64; 3],
        quat: [f64; 4],
        sec: i32,
        nanosec: u32,
        frame_id: &str,
    ) -> Vec<u8> {
        let hdr = header_body(sec, nanosec, frame_id);
        let mut v = pose_fixed_section(pos, quat);
        v.extend_from_slice(&[0u8; 8]); // entry[0] placeholder
        write_offset_entry(&mut v, 56, 0, 64, hdr.len() as u32);
        v.extend_from_slice(&hdr);
        v
    }

    /// #7 — THE ISSUE'S ACCEPTANCE CASE: `nav_msgs/Path` with two
    /// `PoseStamped` elements (variable elements: each carries its own
    /// `Header`, so each element body has its OWN offset table) decodes into
    /// the full recursive tree.
    #[test]
    fn path_variable_elements_decode_full_recursive_tree() {
        let walker = builtin_walker_for_test();
        assert_header_shape(&walker);
        let (path_fixed, path_table) =
            assert_shape(&walker, "nav_msgs/Path", 0, &["header", "poses"]);
        assert_eq!((path_fixed, path_table), (0, 16));
        let (ps_fixed, ps_table) =
            assert_shape(&walker, "geometry_msgs/PoseStamped", 56, &["header"]);
        assert_eq!((ps_fixed, ps_table), (56, 8));

        // Hand oracle: two poses with distinct everything.
        let oracle = [
            (
                [1.0f64, 2.0, 3.0],
                [0.0f64, 0.0, 0.0, 1.0],
                11i32,
                12u32,
                "map",
            ),
            (
                [-4.5f64, 5.25, 6.125],
                [0.5f64, -0.5, 0.25, 0.75],
                21i32,
                22u32,
                "odom_frame",
            ),
        ];
        let bodies: Vec<Vec<u8>> = oracle
            .iter()
            .map(|(p, q, s, n, f)| pose_stamped_body(*p, *q, *s, *n, f))
            .collect();
        let blob = counted_blob(&bodies);

        let mut payload = vec![0u8; path_fixed + path_table];
        write_offset_entry(
            &mut payload,
            path_fixed,
            1,
            (path_fixed + path_table) as u32,
            blob.len() as u32,
        );
        payload.extend_from_slice(&blob);
        let f = frame(0, &payload);

        let fv = walker.walk("nav_msgs/Path", &f).expect("walk");
        let elems = expect_nested_array(fv.field("poses"), "poses");
        assert_eq!(elems.len(), 2);
        for (i, (pos, quat, sec, nanosec, frame_id)) in oracle.iter().enumerate() {
            let ps = expect_nested(&elems[i], "PoseStamped element");
            assert_eq!(ps.schema_name, "geometry_msgs/PoseStamped");
            let pose = expect_nested(ps.field("pose").expect("pose"), "pose");
            let position = expect_nested(pose.field("position").expect("position"), "position");
            assert_eq!(position.field("x"), Some(&FrameValueKind::F64(pos[0])));
            assert_eq!(position.field("y"), Some(&FrameValueKind::F64(pos[1])));
            assert_eq!(position.field("z"), Some(&FrameValueKind::F64(pos[2])));
            let orientation = expect_nested(
                pose.field("orientation").expect("orientation"),
                "orientation",
            );
            assert_eq!(orientation.field("w"), Some(&FrameValueKind::F64(quat[3])));
            // The element's OWN variable field: a nested Header sub-frame.
            let hdr = expect_nested(ps.field("header").expect("header"), "header");
            assert_eq!(hdr.schema_name, "std_msgs/Header");
            assert_eq!(hdr.field("frame_id"), Some(&FrameValueKind::Str(frame_id)));
            let stamp = expect_nested(hdr.field("stamp").expect("stamp"), "stamp");
            assert_eq!(stamp.field("sec"), Some(&FrameValueKind::I32(*sec)));
            assert_eq!(stamp.field("nanosec"), Some(&FrameValueKind::U32(*nanosec)));
        }
    }

    /// A `geometry_msgs/TransformStamped` element body: fixed `transform`
    /// (56 B) | entry[0] `header` | entry[1] `child_frame_id` | the two
    /// payloads. TWO variable fields per element, so the element's own
    /// offset table is genuinely exercised.
    fn transform_stamped_body(
        translation: [f64; 3],
        rotation: [f64; 4],
        frame_id: &str,
        child_frame_id: &str,
    ) -> Vec<u8> {
        let hdr = header_body(0, 0, frame_id);
        let mut v = pose_fixed_section(translation, rotation);
        v.extend_from_slice(&[0u8; 16]); // two entry placeholders
        write_offset_entry(&mut v, 56, 0, 72, hdr.len() as u32);
        write_offset_entry(
            &mut v,
            56,
            1,
            72 + hdr.len() as u32,
            child_frame_id.len() as u32,
        );
        v.extend_from_slice(&hdr);
        v.extend_from_slice(child_frame_id.as_bytes());
        v
    }

    /// Build a `tf2_msgs/TFMessage` frame carrying `blob` verbatim.
    fn tf_message_frame(w: &FrameWalker, blob: &[u8]) -> Vec<u8> {
        let (fixed_size, table_bytes) = assert_shape(w, "tf2_msgs/TFMessage", 0, &["transforms"]);
        assert_eq!((fixed_size, table_bytes), (0, 8));
        let (ts_fixed, ts_table) = assert_shape(
            w,
            "geometry_msgs/TransformStamped",
            56,
            &["header", "child_frame_id"],
        );
        assert_eq!((ts_fixed, ts_table), (56, 16));
        let mut payload = vec![0u8; fixed_size + table_bytes];
        write_offset_entry(
            &mut payload,
            fixed_size,
            0,
            (fixed_size + table_bytes) as u32,
            blob.len() as u32,
        );
        payload.extend_from_slice(blob);
        frame(0, &payload)
    }

    /// #8 — `tf2_msgs/TFMessage`: two `TransformStamped` elements, each with
    /// TWO variable fields (its `Header` AND `child_frame_id`).
    #[test]
    fn tf_message_elements_with_two_variable_fields_decode() {
        let walker = builtin_walker_for_test();
        assert_header_shape(&walker);
        let oracle = [
            (
                [1.0f64, 0.0, 0.25],
                [0.0f64, 0.0, 0.0, 1.0],
                "odom",
                "base_link",
            ),
            (
                [-2.0f64, 3.5, 0.0],
                [0.25f64, 0.5, 0.125, 0.875],
                "base_link",
                "lidar",
            ),
        ];
        let bodies: Vec<Vec<u8>> = oracle
            .iter()
            .map(|(t, r, f, c)| transform_stamped_body(*t, *r, f, c))
            .collect();
        let f = tf_message_frame(&walker, &counted_blob(&bodies));

        let fv = walker.walk("tf2_msgs/TFMessage", &f).expect("walk");
        let elems = expect_nested_array(fv.field("transforms"), "transforms");
        assert_eq!(elems.len(), 2);
        for (i, (translation, rotation, frame_id, child)) in oracle.iter().enumerate() {
            let ts = expect_nested(&elems[i], "TransformStamped element");
            let tr = expect_nested(ts.field("transform").expect("transform"), "transform");
            let t = expect_nested(tr.field("translation").expect("translation"), "translation");
            assert_eq!(t.field("x"), Some(&FrameValueKind::F64(translation[0])));
            assert_eq!(t.field("z"), Some(&FrameValueKind::F64(translation[2])));
            let r = expect_nested(tr.field("rotation").expect("rotation"), "rotation");
            assert_eq!(r.field("w"), Some(&FrameValueKind::F64(rotation[3])));
            assert_eq!(
                ts.field("child_frame_id"),
                Some(&FrameValueKind::Str(child))
            );
            let hdr = expect_nested(ts.field("header").expect("header"), "header");
            assert_eq!(hdr.field("frame_id"), Some(&FrameValueKind::Str(frame_id)));
        }
    }

    /// #9 — `vision_msgs/Detection3DArray`: an ARRAY INSIDE AN ELEMENT (the
    /// recursion composing). `detections[]` elements are variable and each
    /// carries its OWN `results[]` array of variable elements, each of which
    /// carries a variable nested `hypothesis` holding a string — four levels
    /// below the top-level array.
    #[test]
    fn detection3d_array_nested_element_arrays_compose() {
        let walker = builtin_walker_for_test();
        // Layout drift guards for every level the bodies below hand-lay.
        let (da_fixed, da_table) = assert_shape(
            &walker,
            "vision_msgs/Detection3DArray",
            0,
            &["header", "detections"],
        );
        assert_eq!((da_fixed, da_table), (0, 16));
        let (d_fixed, d_table) = assert_shape(
            &walker,
            "vision_msgs/Detection3D",
            80, // bbox = Pose(56) + Vector3(24)
            &["header", "results", "id"],
        );
        assert_eq!((d_fixed, d_table), (80, 24));
        let (ohwp_fixed, ohwp_table) = assert_shape(
            &walker,
            "vision_msgs/ObjectHypothesisWithPose",
            344, // PoseWithCovariance = Pose(56) + float64[36](288)
            &["hypothesis"],
        );
        assert_eq!((ohwp_fixed, ohwp_table), (344, 8));
        let (oh_fixed, oh_table) =
            assert_shape(&walker, "vision_msgs/ObjectHypothesis", 8, &["class_id"]);
        assert_eq!((oh_fixed, oh_table), (8, 8));

        // Hand oracle.
        const SCORE: f64 = 0.875;
        const CLASS_ID: &str = "cone";
        const DET_ID: &str = "det-7";
        let center_pos = [10.5f64, -20.25, 30.125];
        let size = [1.5f64, 2.5, 3.5];

        // ObjectHypothesis: fixed {score} | entry[0] class_id | UTF-8.
        let mut oh = vec![0u8; 16];
        oh[0..8].copy_from_slice(&SCORE.to_le_bytes());
        write_offset_entry(&mut oh, 8, 0, 16, CLASS_ID.len() as u32);
        oh.extend_from_slice(CLASS_ID.as_bytes());

        // ObjectHypothesisWithPose: fixed {pose: PoseWithCovariance} (zeroed —
        // not under test) | entry[0] hypothesis | the sub-frame.
        let mut ohwp = vec![0u8; 344 + 8];
        write_offset_entry(&mut ohwp, 344, 0, 352, oh.len() as u32);
        ohwp.extend_from_slice(&oh);

        let results_blob = counted_blob(&[ohwp]);

        // Detection3D: fixed {bbox: Pose center + Vector3 size} | entries
        // [header, results, id]. `header` carries a MINIMAL Header sub-frame
        // (zero stamp, empty `frame_id`) — exactly what
        // `CdrCodec::decode_payload` writes for an empty header.
        // The `(0, 0)` unwritten-entry idiom is a generated-WRITER
        // convention and is NOT canonical inside an element body, so an element
        // that used it would (correctly) degrade the whole array to opaque.
        let empty_hdr = header_body(0, 0, "");
        assert_eq!(empty_hdr.len(), 16, "minimal Header sub-frame");
        let mut det = Vec::new();
        det.extend_from_slice(&pose_fixed_section(center_pos, [0.0, 0.0, 0.0, 1.0]));
        for c in size {
            det.extend_from_slice(&c.to_le_bytes());
        }
        assert_eq!(det.len(), 80, "bbox fixed section");
        det.extend_from_slice(&[0u8; 24]); // three entry placeholders
        write_offset_entry(&mut det, 80, 0, 104, empty_hdr.len() as u32);
        let results_off = 104 + empty_hdr.len() as u32;
        write_offset_entry(&mut det, 80, 1, results_off, results_blob.len() as u32);
        write_offset_entry(
            &mut det,
            80,
            2,
            results_off + results_blob.len() as u32,
            DET_ID.len() as u32,
        );
        det.extend_from_slice(&empty_hdr);
        det.extend_from_slice(&results_blob);
        det.extend_from_slice(DET_ID.as_bytes());

        let detections_blob = counted_blob(&[det]);
        let mut payload = vec![0u8; da_fixed + da_table];
        write_offset_entry(
            &mut payload,
            da_fixed,
            1,
            (da_fixed + da_table) as u32,
            detections_blob.len() as u32,
        );
        payload.extend_from_slice(&detections_blob);
        let f = frame(0, &payload);

        let fv = walker
            .walk("vision_msgs/Detection3DArray", &f)
            .expect("walk");
        let dets = expect_nested_array(fv.field("detections"), "detections");
        assert_eq!(dets.len(), 1);
        let det = expect_nested(&dets[0], "Detection3D");
        assert_eq!(det.field("id"), Some(&FrameValueKind::Str(DET_ID)));
        // The fixed nested bbox rode along untouched.
        let bbox = expect_nested(det.field("bbox").expect("bbox"), "bbox");
        let center = expect_nested(bbox.field("center").expect("center"), "center");
        let cpos = expect_nested(center.field("position").expect("position"), "position");
        assert_eq!(cpos.field("x"), Some(&FrameValueKind::F64(center_pos[0])));
        let bsize = expect_nested(bbox.field("size").expect("size"), "size");
        assert_eq!(bsize.field("y"), Some(&FrameValueKind::F64(size[1])));
        // THE COMPOSITION: an array inside an element, whose element carries a
        // variable nested holding a string.
        let results = expect_nested_array(det.field("results"), "results");
        assert_eq!(results.len(), 1);
        let ohwp = expect_nested(&results[0], "ObjectHypothesisWithPose");
        let hyp = expect_nested(ohwp.field("hypothesis").expect("hypothesis"), "hypothesis");
        assert_eq!(hyp.field("class_id"), Some(&FrameValueKind::Str(CLASS_ID)));
        assert_eq!(hyp.field("score"), Some(&FrameValueKind::F64(SCORE)));
    }

    /// Build a `sensor_msgs/JointState` frame carrying `name_blob` verbatim in
    /// its `string[] name` field (the other arrays stay empty).
    fn joint_state_frame(w: &FrameWalker, name_blob: &[u8]) -> Vec<u8> {
        let (fixed_size, table_bytes) = assert_shape(
            w,
            "sensor_msgs/JointState",
            0,
            &["header", "name", "position", "velocity", "effort"],
        );
        assert_eq!((fixed_size, table_bytes), (0, 40));
        let mut payload = vec![0u8; fixed_size + table_bytes];
        write_offset_entry(
            &mut payload,
            fixed_size,
            1,
            (fixed_size + table_bytes) as u32,
            name_blob.len() as u32,
        );
        payload.extend_from_slice(name_blob);
        frame(0, &payload)
    }

    /// #10 — `string[]`: `u32 count` + per element `u32 len` + UTF-8, over the
    /// real `sensor_msgs/JointState.name`. An EMPTY element is a legitimate
    /// empty string (CDR spells it `u32(1) | NUL` — zero content bytes); a
    /// NON-UTF-8 element degrades PER ELEMENT to `Bytes`, matching the
    /// walker's existing string policy, without failing the array.
    #[test]
    fn string_array_decodes_with_empty_and_non_utf8_elements() {
        let walker = builtin_walker_for_test();
        let blob = counted_blob(&[
            b"joint_a".to_vec(),
            Vec::new(),
            vec![0xFF, 0xFE],
            b"joint_d".to_vec(),
        ]);
        let f = joint_state_frame(&walker, &blob);
        let fv = walker.walk("sensor_msgs/JointState", &f).expect("walk");
        let elems = expect_nested_array(fv.field("name"), "name");
        assert_eq!(
            elems,
            &[
                FrameValueKind::Str("joint_a"),
                FrameValueKind::Str(""),
                FrameValueKind::Bytes(&[0xFF, 0xFE]),
                FrameValueKind::Str("joint_d"),
            ]
        );
    }

    /// #11-#13 — the adversarial count/length arms, all over the real
    /// `string[]` field so the shape is production-realistic. Each must
    /// degrade to opaque: no panic, no OOM, no partial decode.
    #[test]
    fn counted_array_adversarial_counts_and_lengths_degrade() {
        let walker = builtin_walker_for_test();
        let mut cases: Vec<(&str, Vec<u8>)> = Vec::new();

        // #11 hostile count: 0xFFFFFFFF elements claimed over a few bytes.
        let mut hostile = u32::MAX.to_le_bytes().to_vec();
        hostile.extend_from_slice(&[1, 0, 0, 0, b'a']);
        cases.push(("count = u32::MAX", hostile));

        // Count too big for the budget even without overflow: 3 elements
        // claimed but only 4 payload bytes remain.
        let mut over = 3u32.to_le_bytes().to_vec();
        over.extend_from_slice(&1u32.to_le_bytes());
        cases.push(("count over the byte budget", over));

        // #12 the FINAL element's length overruns the blob.
        let mut overrun = 2u32.to_le_bytes().to_vec();
        overrun.extend_from_slice(&1u32.to_le_bytes());
        overrun.push(b'a');
        overrun.extend_from_slice(&99u32.to_le_bytes()); // claims 99 bytes
        overrun.extend_from_slice(b"bb");
        cases.push(("final element length overruns", overrun));

        // A length prefix that would overflow the cursor.
        let mut huge = 1u32.to_le_bytes().to_vec();
        huge.extend_from_slice(&u32::MAX.to_le_bytes());
        huge.extend_from_slice(b"x");
        cases.push(("element length = u32::MAX", huge));

        // #13 TRAILING SLACK after the declared elements — the
        // exact-consumption pin (a lenient decoder would return ["a"] and
        // silently swallow the tail).
        let mut slack = 1u32.to_le_bytes().to_vec();
        slack.extend_from_slice(&1u32.to_le_bytes());
        slack.push(b'a');
        slack.extend_from_slice(b"UNCONSUMED");
        cases.push(("trailing bytes after the last element", slack));

        // Truncated count prefix (fewer than 4 bytes in the whole blob).
        cases.push(("count prefix truncated", vec![1, 0, 0]));

        for (label, blob) in cases {
            let f = joint_state_frame(&walker, &blob);
            let fv = walker.walk("sensor_msgs/JointState", &f).expect("walk");
            assert_eq!(
                fv.field("name"),
                Some(&FrameValueKind::NestedArrayOpaque(blob.as_slice())),
                "{label} must degrade to opaque"
            );
        }

        // ANTI-TAUTOLOGY control: the same builder + a WELL-FORMED blob does
        // decode (so the arms above are not all failing for some unrelated
        // reason).
        let good = counted_blob(&[b"a".to_vec(), b"bb".to_vec()]);
        let f = joint_state_frame(&walker, &good);
        let fv = walker.walk("sensor_msgs/JointState", &f).expect("walk");
        assert_eq!(
            expect_nested_array(fv.field("name"), "name"),
            &[FrameValueKind::Str("a"), FrameValueKind::Str("bb")]
        );
    }

    /// ONE record in the `cerulion_viz::go2_tf` TF convention (`u32 count`
    /// then FLAT records with NO per-element length —
    /// `sec | nanosec | u32+frame_id | u32+child | 3×f64 | 4×f64`; that
    /// module's docs are the source of truth). Hand-laid rather than imported:
    /// `cerulion_core` must not depend on `cerulion_viz`.
    fn go2_tf_blob(
        sec: i32,
        nanosec: u32,
        frame_id: &str,
        child: &str,
        translation: [f64; 3],
        rotation: [f64; 4],
    ) -> Vec<u8> {
        let mut blob = 1u32.to_le_bytes().to_vec();
        blob.extend_from_slice(&sec.to_le_bytes());
        blob.extend_from_slice(&nanosec.to_le_bytes());
        blob.extend_from_slice(&(frame_id.len() as u32).to_le_bytes());
        blob.extend_from_slice(frame_id.as_bytes());
        blob.extend_from_slice(&(child.len() as u32).to_le_bytes());
        blob.extend_from_slice(child.as_bytes());
        for c in translation {
            blob.extend_from_slice(&c.to_le_bytes());
        }
        for c in rotation {
            blob.extend_from_slice(&c.to_le_bytes());
        }
        blob
    }

    /// The `stamp.sec` value at which a go2_tf blob of `blob_len` bytes ALIASES
    /// the canonical counted framing: `sec` occupies the 4 bytes the canonical
    /// form uses for the first element's `u32 len`, so the blob is exactly
    /// consumed iff `4 (count) + 4 (len) + sec == blob_len`.
    ///
    /// DERIVED, never hardcoded — a go2_tf layout change (or a different
    /// frame-id length) moves the alias, and a test that pinned 76 would
    /// silently stop probing the case it exists for.
    fn go2_tf_alias_sec(blob_len: usize) -> i32 {
        i32::try_from(blob_len - 8).expect("a go2_tf record is far below i32::MAX")
    }

    /// #14a — BESPOKE DISCRIMINATOR (hardened): the go2_tf TF
    /// element convention must NOT decode as canonical v1 at ANY stamp
    /// magnitude — including the ONE value per record shape where the ARRAY
    /// framing is satisfied EXACTLY.
    ///
    /// For the REAL production record (`go2_tf_source` + `identity_odom_base`
    /// ⇒ 84 bytes) the alias is `sec == 76`: the resulting 76-byte "element"
    /// clears `TransformStamped`'s `56 + 16` floor, its two offset-table entries
    /// land in the identity quaternion's zero bytes, and without the
    /// discriminator the walker would decode a bogus 1-element array.
    /// `go2_tf_source` stamps `Time::from_ns(self.now_ns())`
    /// off a MONOTONIC clock, so a robot's uptime passes through 76 s every
    /// boot (~10 frames at `period_ms = 100`).
    #[test]
    fn go2_tf_bespoke_blob_stays_opaque_at_every_stamp() {
        let walker = builtin_walker_for_test();
        // Both the REAL production shape and the wider one the earlier test
        // used, so each shape's own alias is swept.
        let shapes: [(&str, &str, &str); 2] = [
            (
                "production (go2_tf_source + identity_odom_base)",
                "odom",
                "base",
            ),
            ("wider child frame", "odom", "base_link"),
        ];
        for (label, frame_id, child) in shapes {
            let probe = go2_tf_blob(0, 7, frame_id, child, [1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]);
            let alias = go2_tf_alias_sec(probe.len());
            // The alias is a genuine NEAR MISS, not a trivially-rejected blob:
            // the array level accepts it (exact outer consumption) and the
            // element body clears TransformStamped's fixed + table floor. If a
            // layout change ever breaks either fact this test stops being
            // probative, so both are asserted.
            let (ts_fixed, ts_table) = assert_shape(
                &walker,
                "geometry_msgs/TransformStamped",
                56,
                &["header", "child_frame_id"],
            );
            assert_eq!(
                8 + alias as usize,
                probe.len(),
                "{label}: the alias must consume the blob exactly"
            );
            assert!(
                alias as usize > ts_fixed + ts_table,
                "{label}: the alias body must clear the element's fixed + table floor \
                 (else the array level, not the element audit, does the refusing)"
            );

            // Sweep every uptime second a robot passes through in its first 5
            // minutes, plus the derived alias (inside the range for these
            // shapes, listed explicitly so the arm survives a range change).
            for sec in (0..=300).chain(std::iter::once(alias)) {
                let blob = go2_tf_blob(
                    sec,
                    7,
                    frame_id,
                    child,
                    [1.0, 2.0, 3.0],
                    [0.0, 0.0, 0.0, 1.0],
                );
                let f = tf_message_frame(&walker, &blob);
                let fv = walker.walk("tf2_msgs/TFMessage", &f).expect("walk");
                assert_eq!(
                    fv.field("transforms"),
                    Some(&FrameValueKind::NestedArrayOpaque(blob.as_slice())),
                    "{label}: a go2_tf blob (sec = {sec}) must stay opaque"
                );
            }
            // And a far-future wall-clock stamp (the shape a robot with a real
            // time source publishes).
            let blob = go2_tf_blob(
                1_700_000_000,
                7,
                frame_id,
                child,
                [1.0, 2.0, 3.0],
                [0.0, 0.0, 0.0, 1.0],
            );
            let f = tf_message_frame(&walker, &blob);
            let fv = walker.walk("tf2_msgs/TFMessage", &f).expect("walk");
            assert_eq!(
                fv.field("transforms"),
                Some(&FrameValueKind::NestedArrayOpaque(blob.as_slice())),
                "{label}: a wall-clock-stamped go2_tf blob must stay opaque"
            );
        }
    }

    /// #14a' — the EXACT production vector, as a named regression test: the
    /// bytes `go2_tf_source` publishes on `/tf` 76 seconds after boot
    /// (`identity_odom_base(76, _)` — odom→base, zero translation, identity
    /// quaternion). Without strict validation this decodes into a 1-element `NestedArray`
    /// whose translation reads `(8.49e-314, 9.40e-314, 8.41e-315)` — garbage
    /// floats reinterpreted from the record's frame-id/length bytes.
    #[test]
    fn go2_tf_production_blob_at_the_alias_stamp_stays_opaque() {
        let walker = builtin_walker_for_test();
        let blob = go2_tf_blob(76, 0, "odom", "base", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]);
        assert_eq!(blob.len(), 84, "the production go2_tf record is 84 bytes");
        assert_eq!(go2_tf_alias_sec(blob.len()), 76, "the alias stamp is 76 s");
        let f = tf_message_frame(&walker, &blob);
        let fv = walker.walk("tf2_msgs/TFMessage", &f).expect("walk");
        assert_eq!(
            fv.field("transforms"),
            Some(&FrameValueKind::NestedArrayOpaque(blob.as_slice())),
            "the production /tf blob must stay opaque AT the alias stamp"
        );

        // ANTI-TAUTOLOGY: the same frame builder + a CANONICAL blob decodes, so
        // the arm above fails on the element convention, not on the harness.
        let canonical = counted_blob(&[transform_stamped_body(
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            "odom",
            "base",
        )]);
        let f = tf_message_frame(&walker, &canonical);
        let fv = walker.walk("tf2_msgs/TFMessage", &f).expect("walk");
        assert_eq!(
            expect_nested_array(fv.field("transforms"), "transforms").len(),
            1
        );
    }

    /// #14b — BESPOKE DISCRIMINATOR: the packed `PointCloud2.fields`
    /// convention `dds_bridge::mapping::encode_point_fields` writes (records
    /// until end-of-blob, NO count at all) must stay opaque.
    #[test]
    fn packed_point_fields_bespoke_blob_stays_opaque() {
        let walker = builtin_walker_for_test();
        let (fixed_size, table_bytes) = {
            let l = layout_of(&walker, "sensor_msgs/PointCloud2");
            (l.fixed_size, l.offset_table_bytes())
        };
        let fields_idx = layout_of(&walker, "sensor_msgs/PointCloud2")
            .variable_fields
            .iter()
            .position(|v| v.name == "fields")
            .expect("PointCloud2 has a `fields` variable field");

        // The packed layout: per record `u32 name_len | name | u32 offset |
        // u8 datatype | u32 count`. No count prefix, no per-element length.
        let mut blob = Vec::new();
        for (name, offset) in [("x", 0u32), ("y", 4), ("z", 8)] {
            blob.extend_from_slice(&(name.len() as u32).to_le_bytes());
            blob.extend_from_slice(name.as_bytes());
            blob.extend_from_slice(&offset.to_le_bytes());
            blob.push(7); // FLOAT32
            blob.extend_from_slice(&1u32.to_le_bytes());
        }

        let mut payload = vec![0u8; fixed_size + table_bytes];
        write_offset_entry(
            &mut payload,
            fixed_size,
            fields_idx,
            (fixed_size + table_bytes) as u32,
            blob.len() as u32,
        );
        payload.extend_from_slice(&blob);
        let f = frame(0, &payload);
        let fv = walker.walk("sensor_msgs/PointCloud2", &f).expect("walk");
        assert_eq!(
            fv.field("fields"),
            Some(&FrameValueKind::NestedArrayOpaque(blob.as_slice())),
            "the packed point-fields blob must stay opaque"
        );
    }

    /// #14c — BESPOKE DISCRIMINATOR: an element BODY written as
    /// `[packed fixed][u32 count-of-variable-members][per var: u32 len +
    /// payload]` — NO offset table — must NOT decode, even though it agrees
    /// with canonical v1 on the array's count+length framing. For
    /// `std_msgs/Header` with `frame_id = "odom"` both bodies are 20 bytes,
    /// but bytes `[8..12]` carry the variable-member count `1` where canonical
    /// carries the offset `16`, so the canonical read sees an offset-table
    /// entry pointing below the data floor and the whole array degrades.
    ///
    /// **This test was renamed** (was `rmw_packed_element_body_stays_opaque`).
    /// The rmw bridge no longer PRODUCES this shape — both bridges now build
    /// element bodies through the shared `element_codec`, cross-validated by
    /// `crates/rmw_cerulion/tests/canonical_element_body_test.rs`. The pin is KEPT
    /// rather than inverted because its contract was never "rmw writes this":
    /// it is "these bytes must degrade rather than mis-decode", which still
    /// holds and still matters — a recorded bag or an un-upgraded peer can
    /// carry them, and the two framings are byte-length-identical at N = 1, so
    /// a lenient reader would silently mis-decode rather than fail.
    #[test]
    fn legacy_packed_element_body_stays_opaque() {
        let walker = header_list_walker();
        assert_header_shape(&walker);

        // The legacy packed Header body for frame_id = "odom".
        let mut rmw = Vec::new();
        rmw.extend_from_slice(&0i32.to_le_bytes()); // stamp.sec
        rmw.extend_from_slice(&0u32.to_le_bytes()); // stamp.nanosec
        rmw.extend_from_slice(&1u32.to_le_bytes()); // count of variable members
        rmw.extend_from_slice(&4u32.to_le_bytes()); // len("odom")
        rmw.extend_from_slice(b"odom");
        assert_eq!(rmw.len(), 20);
        // Same length as the canonical body — only the middle u32 differs.
        assert_eq!(header_body(0, 0, "odom").len(), rmw.len());

        let blob = counted_blob(&[rmw]);
        let mut payload = vec![0u8; 8];
        write_offset_entry(&mut payload, 0, 0, 8, blob.len() as u32);
        payload.extend_from_slice(&blob);
        let f = frame(0, &payload);
        let fv = walker.walk("test_msgs/HeaderList", &f).expect("walk");
        assert_eq!(
            fv.field("items"),
            Some(&FrameValueKind::NestedArrayOpaque(blob.as_slice())),
            "a legacy packed element body must not decode as canonical v1"
        );

        // ANTI-TAUTOLOGY control: the CANONICAL body of the same message, in
        // the same array framing, DOES decode — so the arm above fails on the
        // body layout, not on the harness.
        let blob = counted_blob(&[header_body(0, 0, "odom")]);
        let mut payload = vec![0u8; 8];
        write_offset_entry(&mut payload, 0, 0, 8, blob.len() as u32);
        payload.extend_from_slice(&blob);
        let f = frame(0, &payload);
        let fv = walker.walk("test_msgs/HeaderList", &f).expect("walk");
        let elems = expect_nested_array(fv.field("items"), "items");
        assert_eq!(elems.len(), 1);
        assert_eq!(
            expect_nested(&elems[0], "Header").field("frame_id"),
            Some(&FrameValueKind::Str("odom"))
        );
    }

    /// Build a `test_msgs/HeaderList { std_msgs/Header[] items }` walker over
    /// the real corpus (no built-in type has a bare `Header[]`), plus a frame
    /// carrying `blob` verbatim in `items`.
    fn header_list_walker() -> FrameWalker {
        let mut holder = MessageSchema::new_in_package("HeaderList", "test_msgs");
        holder.add_field(FieldDef::new(
            "items",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::Nested {
                    schema_name: "Header".into(),
                    package: Some("std_msgs".into()),
                    fixed: None,
                }),
            },
        ));
        let mut schemas = resolved_builtin_schemas();
        schemas.push(holder);
        let (walker, _) = FrameWalker::new(schemas);
        walker
    }

    fn header_list_frame(blob: &[u8]) -> Vec<u8> {
        let mut payload = vec![0u8; 8];
        write_offset_entry(&mut payload, 0, 0, 8, blob.len() as u32);
        payload.extend_from_slice(blob);
        frame(0, &payload)
    }

    /// #14d — an element only decodes if its OWN internal
    /// accounting is EXACT. Each arm below satisfies the ARRAY framing
    /// perfectly (count + per-element length + exact outer consumption) and is
    /// refused purely on the element body — the class the `sec == 76` go2_tf
    /// alias lives in.
    #[test]
    fn element_internal_accounting_is_exact() {
        let walker = header_list_walker();
        assert_header_shape(&walker);
        // `std_msgs/Header`: fixed 8 (stamp) + table 8 (frame_id) ⇒ data floor
        // 16.
        const FLOOR: usize = 16;

        // CONTROL (anti-tautology): the exactly-accounted body DOES decode.
        let good = header_body(1, 2, "odom");
        assert_eq!(good.len(), FLOOR + 4);
        let good_frame = header_list_frame(&counted_blob(&[good]));
        let fv = walker
            .walk("test_msgs/HeaderList", &good_frame)
            .expect("walk");
        let elems = expect_nested_array(fv.field("items"), "items");
        assert_eq!(
            expect_nested(&elems[0], "Header").field("frame_id"),
            Some(&FrameValueKind::Str("odom"))
        );

        let mut cases: Vec<(&str, Vec<Vec<u8>>)> = Vec::new();

        // TRAILING SLACK inside the element — the go2_tf alias's signature
        // (its 76-byte body clears the floor and leaves 4 bytes unaccounted).
        let mut slack = header_body(1, 2, "odom");
        slack.extend_from_slice(b"HOLE");
        cases.push(("element body with trailing slack", vec![slack]));

        // The `(0, 0)` UNWRITTEN-ENTRY idiom: legal in a FRAME (a generated
        // writer leaves an untouched variable field there), NOT canonical
        // inside an element body. Tolerated, it would decode as `frame_id: ""`
        // and leave every byte past the table unvalidated.
        cases.push(("all-zero offset table", vec![vec![0u8; FLOOR]]));

        // CONTAINMENT: element 0's entry reaches PAST its own body into the
        // sibling's bytes (which DO exist in the enclosing blob, so a
        // base-relative implementation would happily read them).
        let mut leak = header_body(1, 2, "odom");
        write_offset_entry(&mut leak, 8, 0, FLOOR as u32, 12);
        cases.push((
            "entry reaching past the body into the next element",
            vec![leak, header_body(3, 4, "base")],
        ));

        // An entry BELOW the data floor (pointing into the element's own fixed
        // section / table).
        let mut below = header_body(1, 2, "odom");
        write_offset_entry(&mut below, 8, 0, 4, 4);
        cases.push(("entry below the element's data floor", vec![below]));

        for (label, bodies) in cases {
            let blob = counted_blob(&bodies);
            let f = header_list_frame(&blob);
            let fv = walker.walk("test_msgs/HeaderList", &f).expect("walk");
            assert_eq!(
                fv.field("items"),
                Some(&FrameValueKind::NestedArrayOpaque(blob.as_slice())),
                "{label} must degrade the whole array to opaque"
            );
        }
    }

    /// #14e — the INTERIOR-GAP rule, over `TransformStamped` (TWO variable
    /// fields, so a gap between them is expressible). `CdrCodec` aligns a
    /// variable payload to at most 8 bytes, so a gap of up to
    /// `MAX_ELEMENT_VAR_PAD` is genuine padding and MUST be accepted; anything
    /// wider is unaccounted space.
    #[test]
    fn element_interior_gap_is_bounded_by_alignment_padding() {
        let walker = builtin_walker_for_test();
        assert_header_shape(&walker);
        let (ts_fixed, ts_table) = assert_shape(
            &walker,
            "geometry_msgs/TransformStamped",
            56,
            &["header", "child_frame_id"],
        );
        let floor = ts_fixed + ts_table;

        /// A `TransformStamped` body with `gap` unaccounted bytes between its
        /// `header` and `child_frame_id` payloads.
        fn body_with_gap(floor: usize, frame_id: &str, child: &str, gap: usize) -> Vec<u8> {
            let hdr = header_body(0, 0, frame_id);
            let mut v = pose_fixed_section([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]);
            v.extend_from_slice(&[0u8; 16]);
            assert_eq!(v.len(), floor);
            write_offset_entry(&mut v, 56, 0, floor as u32, hdr.len() as u32);
            write_offset_entry(
                &mut v,
                56,
                1,
                (floor + hdr.len() + gap) as u32,
                child.len() as u32,
            );
            v.extend_from_slice(&hdr);
            v.extend_from_slice(&vec![0u8; gap]);
            v.extend_from_slice(child.as_bytes());
            v
        }

        // Accepted: no gap, and the widest genuine alignment padding.
        for gap in [0, MAX_ELEMENT_VAR_PAD] {
            let blob = counted_blob(&[body_with_gap(floor, "odom", "base", gap)]);
            let f = tf_message_frame(&walker, &blob);
            let fv = walker.walk("tf2_msgs/TFMessage", &f).expect("walk");
            let elems = expect_nested_array(fv.field("transforms"), "transforms");
            assert_eq!(elems.len(), 1, "gap {gap} is genuine alignment padding");
            let ts = expect_nested(&elems[0], "TransformStamped");
            assert_eq!(
                ts.field("child_frame_id"),
                Some(&FrameValueKind::Str("base"))
            );
        }
        // Refused: one byte wider than any alignment could produce.
        let blob = counted_blob(&[body_with_gap(
            floor,
            "odom",
            "base",
            MAX_ELEMENT_VAR_PAD + 1,
        )]);
        let f = tf_message_frame(&walker, &blob);
        let fv = walker.walk("tf2_msgs/TFMessage", &f).expect("walk");
        assert_eq!(
            fv.field("transforms"),
            Some(&FrameValueKind::NestedArrayOpaque(blob.as_slice())),
            "an interior gap wider than alignment padding must degrade to opaque"
        );
    }

    /// #14f — the shapes canonical v1 defines NO element encoding for stay
    /// opaque (the module docs' "What it does NOT decode" list). Unpinned
    /// before this fix.
    #[test]
    fn undefined_element_shapes_stay_opaque() {
        // `bool[]` — one byte per element on the wire, but v1 defines no
        // framing for it, so it must NOT be guessed at.
        let mut flags = MessageSchema::new_in_package("Flags", "test_msgs");
        flags.add_field(FieldDef::new(
            "flags",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::Bool),
            },
        ));
        // `Nested[]` whose element schema is absent from the walker's set.
        let mut orphan = MessageSchema::new_in_package("Orphan", "test_msgs");
        orphan.add_field(FieldDef::new(
            "items",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::Nested {
                    schema_name: "NotInTheSet".into(),
                    package: Some("test_msgs".into()),
                    fixed: None,
                }),
            },
        ));
        // `DynamicArray<StringFixed(n)>`.
        let mut tags = MessageSchema::new_in_package("Tags", "test_msgs");
        tags.add_field(FieldDef::new(
            "tags",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::StringFixed(4)),
            },
        ));
        let (walker, _) = FrameWalker::new(vec![flags, orphan, tags]);

        // A blob that WOULD satisfy the canonical counted framing if the
        // element shape were decodable — so each arm fails on the element
        // TYPE, not on the bytes.
        let blob = counted_blob(&[b"ab".to_vec(), b"cd".to_vec()]);
        for (qname, field) in [
            ("test_msgs/Flags", "flags"),
            ("test_msgs/Orphan", "items"),
            ("test_msgs/Tags", "tags"),
        ] {
            let mut payload = vec![0u8; 8];
            write_offset_entry(&mut payload, 0, 0, 8, blob.len() as u32);
            payload.extend_from_slice(&blob);
            let f = frame(0, &payload);
            let fv = walker.walk(qname, &f).expect("walk");
            assert_eq!(
                fv.field(field),
                Some(&FrameValueKind::NestedArrayOpaque(blob.as_slice())),
                "{qname}.{field} has no canonical v1 element encoding"
            );
        }
    }

    /// #14g — the FIXED-STRIDE path's `declared_length` arity check. It is not
    /// reachable through `walk` on a well-resolved schema set (a `FixedArray`
    /// only reaches `decode_element_array` when it was classified VARIABLE,
    /// i.e. its element is variable, i.e. the stride path is not taken), so it
    /// is pinned directly at the seam rather than shipped unproven.
    #[test]
    fn fixed_stride_declared_length_is_enforced_at_the_seam() {
        let walker = builtin_walker_for_test();
        let point = layout_of(&walker, "geometry_msgs/Point");
        assert_eq!(point.fixed_size, 24, "Point stride drifted");
        assert!(point.is_fixed());
        // Three back-to-back Point sections.
        let bytes = vec![0u8; 72];

        // Arity matches ⇒ decodes.
        match walker.decode_fixed_stride_elements(point, Some(3), &bytes, 0) {
            FrameValueKind::NestedArray { elements, .. } => assert_eq!(elements.len(), 3),
            other => panic!("expected 3 decoded elements, got {other:?}"),
        }
        // Arity disagrees ⇒ opaque (never a silently-different count).
        for declared in [2usize, 4] {
            assert_eq!(
                walker.decode_fixed_stride_elements(point, Some(declared), &bytes, 0),
                FrameValueKind::NestedArrayOpaque(bytes.as_slice()),
                "declared {declared} vs 3 real strides must degrade"
            );
        }
    }

    /// #15 — DETERMINISM: the same bytes decoded twice yield an identical
    /// tree (the walker is a pure function of `(bytes, schema set)`).
    #[test]
    fn element_decode_is_deterministic() {
        let walker = builtin_walker_for_test();
        // Layout drift guards for every shape hand-laid below (this
        // builder was the one test claiming a guard without carrying one).
        assert_header_shape(&walker);
        let (path_fixed, path_table) =
            assert_shape(&walker, "nav_msgs/Path", 0, &["header", "poses"]);
        assert_eq!((path_fixed, path_table), (0, 16));
        let (ps_fixed, ps_table) =
            assert_shape(&walker, "geometry_msgs/PoseStamped", 56, &["header"]);
        assert_eq!((ps_fixed, ps_table), (56, 8));
        let bodies = vec![
            pose_stamped_body([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0], 1, 2, "map"),
            pose_stamped_body([4.0, 5.0, 6.0], [0.5, 0.5, 0.5, 0.5], 3, 4, "odom"),
        ];
        let blob = counted_blob(&bodies);
        let mut payload = vec![0u8; 16];
        write_offset_entry(&mut payload, 0, 1, 16, blob.len() as u32);
        payload.extend_from_slice(&blob);
        let f = frame(0, &payload);

        let a = walker.walk("nav_msgs/Path", &f).expect("walk a");
        let b = walker.walk("nav_msgs/Path", &f).expect("walk b");
        assert_eq!(a, b, "two decodes of one frame must be identical");
        // Not vacuous: the field really decoded.
        assert_eq!(expect_nested_array(a.field("poses"), "poses").len(), 2);
    }

    /// #16 — a `FixedArray` of VARIABLE elements: the Cerulion bytes still
    /// carry the `u32 count` (only the CDR-side count is suppressed), and the
    /// count must EQUAL the declared length.
    #[test]
    fn fixed_array_of_variable_elements_requires_the_declared_count() {
        let mut holder = MessageSchema::new_in_package("Pair", "test_msgs");
        holder.add_field(FieldDef::new(
            "items",
            FieldType::FixedArray {
                element_type: Box::new(FieldType::Nested {
                    schema_name: "Header".into(),
                    package: Some("std_msgs".into()),
                    fixed: None,
                }),
                length: 2,
            },
        ));
        let mut schemas = resolved_builtin_schemas();
        schemas.push(holder);
        let (walker, _) = FrameWalker::new(schemas);
        // The array is VARIABLE (its element is), so it lives in the table.
        let (fixed_size, table_bytes) = assert_shape(&walker, "test_msgs/Pair", 0, &["items"]);
        assert_eq!((fixed_size, table_bytes), (0, 8));

        let build = |blob: &[u8]| {
            let mut payload = vec![0u8; 8];
            write_offset_entry(&mut payload, 0, 0, 8, blob.len() as u32);
            payload.extend_from_slice(blob);
            frame(0, &payload)
        };

        // count == 2 == the declared length → decodes.
        let ok = counted_blob(&[header_body(0, 0, "a"), header_body(0, 0, "bb")]);
        let ok_frame = build(&ok);
        let fv = walker.walk("test_msgs/Pair", &ok_frame).expect("walk");
        let elems = expect_nested_array(fv.field("items"), "items");
        assert_eq!(elems.len(), 2);
        assert_eq!(
            expect_nested(&elems[1], "items[1]").field("frame_id"),
            Some(&FrameValueKind::Str("bb"))
        );

        // count == 1 != 2 → opaque (a well-formed blob of the WRONG arity).
        let wrong = counted_blob(&[header_body(0, 0, "a")]);
        let wrong_frame = build(&wrong);
        let fv = walker.walk("test_msgs/Pair", &wrong_frame).expect("walk");
        assert_eq!(
            fv.field("items"),
            Some(&FrameValueKind::NestedArrayOpaque(wrong.as_slice())),
            "count != declared length must degrade"
        );

        // An EMPTY blob against a declared length of 2 is the
        // same wrong-arity fact, and the empty-blob rule must not short-circuit
        // ahead of it (a short-circuit reports an authoritative ZERO-element decode
        // — the arity the declared length exists to pin).
        let empty_frame = build(&[]);
        let fv = walker.walk("test_msgs/Pair", &empty_frame).expect("walk");
        assert_eq!(
            fv.field("items"),
            Some(&FrameValueKind::NestedArrayOpaque(&[][..])),
            "an empty blob for a declared length of 2 must degrade, not report 0 elements"
        );
    }

    /// #17 — the explicit DEPTH CEILING. A self-referential
    /// `Node { Node[] kids, string tag }` lets a frame nest element arrays
    /// arbitrarily; the walker decodes exactly `MAX_NESTED_ARRAY_DEPTH`
    /// levels and hands the next one back opaque (never unbounded recursion,
    /// never an error).
    #[test]
    fn element_array_depth_is_bounded() {
        let mut node = MessageSchema::new_in_package("Node", "test_msgs");
        node.add_field(FieldDef::new(
            "kids",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::Nested {
                    schema_name: "Node".into(),
                    package: Some("test_msgs".into()),
                    fixed: None,
                }),
            },
        ));
        node.add_field(FieldDef::new("tag", FieldType::String));
        let (walker, _) = FrameWalker::new(vec![node]);
        let (fixed_size, table_bytes) =
            assert_shape(&walker, "test_msgs/Node", 0, &["kids", "tag"]);
        assert_eq!((fixed_size, table_bytes), (0, 16));

        /// One `Node` body: entry[0] `kids` | entry[1] `tag` | payloads.
        fn node_body(kids_blob: &[u8], tag: &str) -> Vec<u8> {
            let mut v = vec![0u8; 16];
            write_offset_entry(&mut v, 0, 0, 16, kids_blob.len() as u32);
            write_offset_entry(&mut v, 0, 1, 16 + kids_blob.len() as u32, tag.len() as u32);
            v.extend_from_slice(kids_blob);
            v.extend_from_slice(tag.as_bytes());
            v
        }

        // Nest LEVELS arrays: the leaf has an empty `kids`.
        const LEVELS: u32 = MAX_NESTED_ARRAY_DEPTH + 2;
        let mut body = node_body(&[], "leaf");
        for i in 0..LEVELS {
            body = node_body(&counted_blob(&[body]), &format!("lvl{i}"));
        }
        let f = frame(0, &body);
        let mut fv = walker.walk("test_msgs/Node", &f).expect("walk");

        // Descend: the array at depth d decodes for d < MAX, and the array at
        // depth MAX is handed back opaque.
        for depth in 0..MAX_NESTED_ARRAY_DEPTH {
            let elems = expect_nested_array(fv.field("kids"), &format!("kids at depth {depth}"));
            assert_eq!(elems.len(), 1, "one child per level at depth {depth}");
            fv = expect_nested(&elems[0], "child").clone();
        }
        match fv.field("kids") {
            Some(FrameValueKind::NestedArrayOpaque(b)) => {
                assert!(!b.is_empty(), "the ceiling handed back the real bytes")
            }
            other => panic!("expected the depth ceiling to degrade, got {other:?}"),
        }
    }

    #[test]
    fn unknown_schema_errors() {
        let (walker, _) = FrameWalker::new(vec![vec3_schema()]);
        let f = frame(0, &[0u8; 24]);
        assert_eq!(
            walker.walk("nope/Nope", &f),
            Err(WalkError::UnknownSchema("nope/Nope".to_string()))
        );
    }

    #[test]
    fn frame_shorter_than_header_errors() {
        let (walker, _) = FrameWalker::new(vec![vec3_schema()]);
        let short = [0u8; 10];
        assert_eq!(
            walker.walk("geometry_msgs/Vec3", &short),
            Err(WalkError::FrameTooShort {
                have: 10,
                need: WireHeader::SIZE
            })
        );
    }

    #[test]
    fn truncated_fixed_section_errors() {
        let (walker, _) = FrameWalker::new(vec![vec3_schema()]);
        // Header present but only 8 payload bytes where 24 are needed.
        let f = frame(0, &[0u8; 8]);
        match walker.walk("geometry_msgs/Vec3", &f) {
            Err(WalkError::FixedFieldOutOfBounds { field, .. }) => {
                // The first field past the 8 available bytes fails.
                assert!(field == "y" || field == "x" || field == "z");
            }
            other => panic!("expected FixedFieldOutOfBounds, got {other:?}"),
        }
    }

    #[test]
    fn variable_offset_out_of_bounds_errors() {
        let (walker, _) = FrameWalker::new(vec![imglike_schema()]);
        // Offset-table entry for `encoding` claims 100 bytes at offset 24 but
        // the payload ends at 24 → out of bounds.
        let mut payload = vec![0u8; 8 + 16];
        write_offset_entry(&mut payload, 8, 0, 24, 100);
        let f = frame(0, &payload);
        match walker.walk("test_msgs/Imglike", &f) {
            Err(WalkError::VariableFieldOutOfBounds {
                field,
                offset,
                length,
                ..
            }) => {
                assert_eq!(field, "encoding");
                assert_eq!((offset, length), (24, 100));
            }
            other => panic!("expected VariableFieldOutOfBounds, got {other:?}"),
        }
    }

    #[test]
    fn string_fixed_trims_at_nul_and_bad_utf8_degrades_to_bytes() {
        let mut s = MessageSchema::new_in_package("Tag", "test_msgs");
        s.add_field(FieldDef::new("name", FieldType::StringFixed(8)));
        let (walker, _) = FrameWalker::new(vec![s]);

        // "hi\0\0\0\0\0\0" → "hi"
        let mut payload = vec![0u8; 8];
        payload[0] = b'h';
        payload[1] = b'i';
        let f = frame(0, &payload);
        let fv = walker.walk("test_msgs/Tag", &f).expect("walk");
        assert_eq!(fv.field("name"), Some(&FrameValueKind::Str("hi")));

        // Invalid UTF-8 (0xFF) with no NUL → degrades to Bytes, never panics.
        let payload2 = vec![0xFFu8; 8];
        let f2 = frame(0, &payload2);
        let fv2 = walker.walk("test_msgs/Tag", &f2).expect("walk");
        assert!(matches!(fv2.field("name"), Some(FrameValueKind::Bytes(_))));
    }

    #[test]
    fn walk_by_hash_resolves_via_header() {
        let schema = vec3_schema();
        let hash = schema.schema_hash();
        let (walker, _) = FrameWalker::new(vec![schema]);

        let mut payload = Vec::new();
        for v in [1.0f64, 2.0, 3.0] {
            payload.extend_from_slice(&v.to_le_bytes());
        }
        let f = frame(hash, &payload);
        let fv = walker.walk_by_hash(&f).expect("walk_by_hash");
        assert_eq!(fv.schema_name, "geometry_msgs/Vec3");

        // A frame with an unregistered hash errors loudly.
        let bad = frame(0xDEAD_BEEF, &payload);
        assert_eq!(
            walker.walk_by_hash(&bad),
            Err(WalkError::UnknownSchemaHash(0xDEAD_BEEF))
        );
    }

    // ---- hash_index / layout_for_hash equivalence ----
    //
    // The `hash → schema` index must resolve every frame hash to EXACTLY what
    // the pre-index linear scan resolved. These pin it over the REAL built-in
    // corpus (`native_ros2_messages::BUILTIN_MSGS` — the exact `.msg` text the
    // generated types compiled against; the same set the viz sink walks).

    /// Build a walker over the entire embedded built-in ROS 2 corpus, exactly
    /// as `cerulion_viz::schema_registry::builtin_walker` does.
    fn builtin_walker_for_test() -> FrameWalker {
        use crate::codegen::parse_rosmsg;
        let mut schemas = Vec::new();
        for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
            if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
                schemas.push(s);
            }
        }
        let (walker, _warnings) = FrameWalker::new(schemas);
        walker
    }

    /// The built-in schemas AFTER `resolve_fixed_nested` — so `schema_hash()`
    /// is the RESOLVED (recipe-3, nested-folded) hash the walker actually
    /// stores, not the raw parse-time hash.
    fn resolved_builtin_schemas() -> Vec<MessageSchema> {
        use crate::codegen::{parse_rosmsg, resolve_fixed_nested};
        let mut schemas = Vec::new();
        for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
            if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
                schemas.push(s);
            }
        }
        resolve_fixed_nested(&mut schemas);
        schemas
    }

    #[test]
    fn hash_index_matches_linear_scan_over_whole_builtin_corpus() {
        let walker = builtin_walker_for_test();
        let resolved = resolved_builtin_schemas();
        assert!(!resolved.is_empty(), "built-in corpus parsed empty");

        // Every resolved built-in hash must resolve through the index to
        // EXACTLY what the pre-index linear scan resolves — a cross-algorithm
        // equivalence pin, and (`resolved.len()` of them) proof it is not
        // vacuously all-`None`.
        let mut resolved_via_index = 0usize;
        for schema in &resolved {
            let hash = schema.schema_hash();
            let via_index = walker.schema_name_for_hash(hash);
            let via_linear = walker.schema_name_for_hash_linear(hash);
            assert_eq!(
                via_index,
                via_linear,
                "index vs linear disagree for {}",
                schema.qualified_name()
            );
            // `layout_for_hash` resolves to the SAME schema and carries the
            // hash it was keyed on.
            match walker.layout_for_hash(hash) {
                Some(layout) => {
                    assert_eq!(Some(layout.qualified_name.as_str()), via_linear);
                    assert_eq!(layout.schema_hash, hash);
                }
                None => assert_eq!(via_linear, None),
            }
            if via_index.is_some() {
                resolved_via_index += 1;
            }
        }
        assert_eq!(
            resolved_via_index,
            resolved.len(),
            "every resolved built-in hash should resolve via the index"
        );
    }

    #[test]
    fn layout_for_name_matches_every_builtin_qualified_name() {
        let walker = builtin_walker_for_test();
        let names: Vec<String> = walker.layouts.keys().cloned().collect();
        assert!(!names.is_empty(), "built-in corpus parsed empty");
        for qualified_name in names {
            let layout = walker
                .layout_for_name(&qualified_name)
                .expect("name lookup");
            assert_eq!(layout.qualified_name, qualified_name);
        }
    }

    #[test]
    fn schema_hash_for_is_the_inverse_of_schema_name_for_hash() {
        // `schema_hash_for(name)` turns a controller-provided
        // ROS type name into the wire hash. Pin it as the exact inverse of
        // `schema_name_for_hash` over the whole corpus (a round-trip, not a
        // self-compare — each half is derived independently), plus a hand oracle
        // and the unknown-name None arm.
        let walker = builtin_walker_for_test();
        let resolved = resolved_builtin_schemas();
        assert!(!resolved.is_empty(), "built-in corpus parsed empty");

        let mut checked = 0usize;
        for schema in &resolved {
            let name = schema.qualified_name();
            let hash = schema.schema_hash();
            // name → hash → name round-trips to the same name.
            assert_eq!(
                walker.schema_hash_for(&name),
                Some(hash),
                "schema_hash_for disagrees with the schema's own hash for {name}"
            );
            assert_eq!(
                walker.schema_name_for_hash(hash),
                Some(name.as_str()),
                "the hash resolves back to the same name for {name}"
            );
            checked += 1;
        }
        assert_eq!(checked, resolved.len(), "every built-in name round-trips");

        // Hand oracle: a known builtin resolves to a non-zero hash.
        assert!(
            walker.schema_hash_for("geometry_msgs/Vector3").is_some(),
            "a known builtin (Vector3) resolves to a hash"
        );
        // An unknown name is None (not a fabricated hash).
        assert_eq!(walker.schema_hash_for("no_such_pkg/NoSuchType"), None);
    }

    #[test]
    fn hash_index_resolves_known_builtins_to_their_literal_names() {
        let walker = builtin_walker_for_test();
        // Resolved-hash lookup by qualified name (Twist folds its fixed-nested
        // Vector3 hashes, so its RESOLVED hash is what the walker stores).
        let by_name: BTreeMap<String, u64> = resolved_builtin_schemas()
            .iter()
            .map(|s| (s.qualified_name(), s.schema_hash()))
            .collect();

        for qname in [
            "geometry_msgs/Vector3",
            "geometry_msgs/Twist",
            "std_msgs/Header",
            "sensor_msgs/PointCloud2",
        ] {
            let hash = *by_name
                .get(qname)
                .unwrap_or_else(|| panic!("{qname} missing from built-in corpus"));
            assert_eq!(
                walker.schema_name_for_hash(hash),
                Some(qname),
                "schema_name_for_hash for {qname}"
            );
            let layout = walker
                .layout_for_hash(hash)
                .unwrap_or_else(|| panic!("layout_for_hash None for {qname}"));
            assert_eq!(layout.qualified_name, qname);
            assert_eq!(layout.schema_hash, hash);
        }

        // A hash no built-in carries resolves to `None` on BOTH accessors.
        let bogus = 0xDEAD_BEEF_DEAD_BEEFu64;
        assert!(
            !by_name.values().any(|&h| h == bogus),
            "test's bogus hash accidentally collided with a real built-in"
        );
        assert_eq!(walker.schema_name_for_hash(bogus), None);
        assert!(walker.layout_for_hash(bogus).is_none());
    }

    /// Findings 1+5: a payload that ends INSIDE (or before) the declared
    /// offset table must REFUSE — never silently decode all-empty variable
    /// fields off `read_offset_entry`'s writer-compat (0, 0) fallback. The
    /// exact boundary (fixed + full table, zeroed) stays Ok with genuinely
    /// empty fields.
    #[test]
    fn truncated_offset_table_refuses_boundary_is_ok() {
        let (walker, _) = FrameWalker::new(vec![imglike_schema()]);
        // Imglike: fixed 8, 2 variable fields → table 16, need = 24.

        // Fixed-section-only payload (8 bytes): the table is entirely
        // missing → Err, with exact accounting.
        let f = frame(0, &[0u8; 8]);
        assert_eq!(
            walker.walk("test_msgs/Imglike", &f),
            Err(WalkError::OffsetTableTruncated {
                schema: "test_msgs/Imglike".to_string(),
                have: 8,
                fixed_size: 8,
                table_bytes: 16,
            })
        );

        // Truncated MID-table (one full entry + 1 byte of the second) →
        // still Err — a partial table is as untrustworthy as none.
        let f = frame(0, &[0u8; 8 + 9]);
        assert_eq!(
            walker.walk("test_msgs/Imglike", &f),
            Err(WalkError::OffsetTableTruncated {
                schema: "test_msgs/Imglike".to_string(),
                have: 17,
                fixed_size: 8,
                table_bytes: 16,
            })
        );

        // BOUNDARY: exactly fixed + table (24 bytes, zeroed) → Ok; every
        // entry present and legitimately empty.
        let f = frame(0, &[0u8; 24]);
        let fv = walker
            .walk("test_msgs/Imglike", &f)
            .expect("boundary walks");
        assert_eq!(fv.field("encoding"), Some(&FrameValueKind::Str("")));
        assert_eq!(fv.field("data"), Some(&FrameValueKind::Bytes(&[])));
    }

    /// The `set_<f>_bytes(&[])` producer idiom: an EMPTY variable-nested
    /// blob decodes as a PRESENT nested value with zero fields; a NON-empty
    /// blob shorter than the target's fixed section + table still errors.
    #[test]
    fn empty_variable_nested_decodes_as_zero_field_nested() {
        // Holder { inner: InnerVar } — InnerVar { id: u32, name: string }
        // (fixed 4 + table 8 → a non-empty sub-frame needs >= 12 bytes).
        let mut inner = MessageSchema::new_in_package("InnerVar", "test_msgs");
        inner.add_field(FieldDef::new("id", FieldType::U32));
        inner.add_field(FieldDef::new("name", FieldType::String));
        let mut holder = MessageSchema::new_in_package("Holder1", "test_msgs");
        holder.add_field(FieldDef::new(
            "inner",
            FieldType::Nested {
                schema_name: "InnerVar".into(),
                package: Some("test_msgs".into()),
                fixed: None,
            },
        ));
        let (walker, _) = FrameWalker::new(vec![inner, holder]);

        // Entry (0, 0) — an untouched table slot, read as a zero-length
        // payload (not the `set_<f>_bytes(&[])` idiom, which emits
        // `(cursor, 0)`; this pins the defensive read of an unwritten slot).
        let payload = vec![0u8; 8]; // all-variable holder: table only
        let f = frame(0, &payload);
        let fv = walker.walk("test_msgs/Holder1", &f).expect("walk");
        match fv.field("inner") {
            Some(FrameValueKind::Nested(n)) => {
                assert_eq!(n.schema_name, "test_msgs/InnerVar");
                assert!(n.fields.is_empty(), "empty blob → zero decoded fields");
            }
            other => panic!("expected empty Nested, got {other:?}"),
        }

        // A NON-empty but table-truncated sub-frame (6 bytes < 12) errors.
        let mut payload = vec![0u8; 8];
        write_offset_entry(&mut payload, 0, 0, 8, 6);
        payload.extend_from_slice(&[0u8; 6]);
        let f = frame(0, &payload);
        assert_eq!(
            walker.walk("test_msgs/Holder1", &f),
            Err(WalkError::OffsetTableTruncated {
                schema: "test_msgs/InnerVar".to_string(),
                have: 6,
                fixed_size: 4,
                table_bytes: 8,
            })
        );
    }
}
