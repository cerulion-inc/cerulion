// SPDX-License-Identifier: AGPL-3.0-only
//! Generic, schema-driven CDR ⇄ Cerulion-wire codec.
//!
//! [`CdrCodec`] transcodes between the DDS/ROS 2 **CDR** body encoding
//! (XCDR1) and the **Cerulion wire payload** (`[fixed][offset table][var]`)
//! for ANY schema the codec's [`MessageSchema`] set can describe — no
//! per-message-type Rust struct, no per-type mapping code. It is driven by
//! the SAME schema IR + [`WireLayout`] machinery that already powers codegen
//! ([`crate::codegen::generate_schema`]), the frame walker
//! ([`crate::codegen::FrameWalker`]), the rmw introspection bridge
//! (`rmw_cerulion::type_bridge`), and schema hashing
//! ([`MessageSchema::schema_hash`]). It exists so a `ros2 attach`-style
//! ingress (`examples/go2/nodes/dds_bridge`) flows an arbitrary
//! standard-registry type with ZERO per-type code: the hand-written serde
//! structs + CDR codecs become anti-regression ORACLES, not the mechanism.
//!
//! # Two directions
//!
//! - [`CdrCodec::decode`] — ingress (sensor direction): a CDR body →
//!   a complete Cerulion wire frame (32-byte [`WireHeader`] + payload),
//!   ready for `publish_raw`.
//! - [`CdrCodec::encode`] — egress/actuator direction: a Cerulion wire
//!   frame → a CDR body, ready to prepend a DDS encapsulation header and
//!   publish to a DDS peer.
//!
//! The two are true inverses at the frame level (`encode(decode(cdr)) ==
//! cdr` modulo the recoverable trailing CDR alignment pad, which XCDR1
//! never emits at the top level).
//!
//! **Known limit — an ARRAY of a member-less message is decode-only.** A
//! member-less element contributes NO Cerulion payload, so `Empty[]` decodes
//! to a zero-byte variable region from which the element count cannot be
//! recovered (the stride is 0), and [`CdrCodec::encode`] refuses it with
//! [`CdrCodecError::BadWirePayload`] instead of guessing — loud, never a
//! silently wrong count. Nothing in the vendored ROS 2 corpus declares such
//! an array (`std_msgs/Empty` is its only member-less message, and no message
//! takes an array of it), so this is recorded rather than fixed.
//!
//! # CDR rules implemented (XCDR1 / plain CDR)
//!
//! Alignment is relative to the START of the CDR body (byte 0 AFTER the
//! 4-byte DDS encapsulation header, which the caller strips — see
//! [`split_encapsulation`]). Each primitive aligns to its own size (1/2/4/8);
//! `bool` is one byte; a `string` is `u32(len+1) | utf8 | 0x00` (STRICT: the
//! terminal NUL is verified and a zero length prefix is rejected — a lenient
//! read would corrupt the last char and break the `encode(decode(cdr)) ==
//! cdr` inverse); a dynamic
//! sequence `T[]` is `u32(count) | elements` (each element aligned per its
//! type); a fixed array `T[N]` is `N` elements with NO count; a nested
//! struct is its fields inline with NO alignment reset and no length prefix.
//! Both little- and big-endian bodies decode (dispatched from the
//! encapsulation header). Every read is bounds-checked: hostile lengths,
//! truncation, non-UTF-8, and huge counts are `Err`, never a panic or an
//! out-of-bounds read (this input arrives from an arbitrary network peer).
//!
//! # Cerulion variable-payload encodings (produced/consumed)
//!
//! These match the native generated writers + [`FrameWalker`](crate::codegen::FrameWalker) (so decoded
//! frames are consumable by every schema-driven Cerulion sink) and the rmw
//! introspection bridge, so an rmw-bridged and a dds-bridged copy of the same
//! message agree wherever both are framework-defined.
//!
//! **That agreement is now structural, not asserted.** This module
//! and both rmw bridges build a message body through the ONE
//! [`element_codec`](crate::codegen::element_codec) routine. It is worth
//! stating plainly why the wording changed: the previous version of this
//! paragraph claimed the same agreement while it was FALSE — the rmw bridge
//! wrote a variable-nested ELEMENT body as `[packed fixed][u32 count-of-
//! variable-members][per var: u32 len + payload]` with no offset table, and
//! had done since the day it was written. Nothing detected it because each
//! side only ever round-tripped against itself. The cross-validation that
//! would have caught it — encode through rmw, decode with the walker — now
//! exists at `crates/rmw_cerulion/tests/canonical_element_body_test.rs`.
//!
//! | field | Cerulion variable payload | offset-table alignment |
//! |---|---|---|
//! | `string` | UTF-8 content (no NUL) | 1 |
//! | `bytes` / `uint8[]`/`int8[]`/`bool[]` | raw bytes | 1 |
//! | `T[]` primitive (≥2-byte) | LE element bytes | `size_of::<T>()` |
//! | single variable nested (e.g. `Header`) | headerless sub-frame `[fixed][table][var]` | 1 |
//! | `Nested[]` (recursively fixed element) | back-to-back fixed sections (stride = padded size), NO count | 1 |
//! | `Nested[]` (variable element) | `u32 count` + per element `u32 len`+sub-frame | 1 |
//! | `string[]` | `u32 count` + per element `u32 len`+UTF-8 | 1 |
//!
//! The `Nested[]`/`string[]` cases have no framing on the *generated
//! producer* API (which exposes only `set_<f>_bytes`), so this codec picks the
//! self-describing, round-trippable canonical encoding above for them and
//! documents it here. **Note:** the frame walker now DECODES all three of
//! those rows element-by-element (into `FrameValueKind::NestedArray`),
//! recursively. Bytes written under some OTHER element convention fail the
//! walker's validation and stay `NestedArrayOpaque`; see
//! [`FrameWalker`](crate::codegen::FrameWalker)'s module docs for the decode
//! rules and the degrade discipline. The `cerulion_viz` viz ladder CONSUMES
//! those decoded arrays to DRAW them — a dds-bridged nav2 `/plan` renders as a
//! `LineStrips3D` polyline (a `PoseArray` as points, a `Detection3DArray` as
//! N-instance boxes) instead of a text dump — so this codec's encoding choice
//! is what makes a never-seen robot's `/plan` "just draw".
//!
//! **Empty variable-nested fields (encode):** a single variable nested field
//! whose offset-table entry has length 0 — how the native writers spell an
//! UNWRITTEN nested field (`set_header_bytes(&[])`; the frame walker reads
//! it as present-with-zero-fields) — encodes as the nested schema's
//! ZERO-DEFAULT fields (every fixed field zero, every string/array empty,
//! recursively). Deterministic; a legitimate native frame is never rejected.

use std::collections::BTreeMap;

use super::element_codec::{CanonicalBodyBuilder, ElementBodyError};
use super::layout::{LayoutResolver, WireLayout};
use super::resolve::resolve_fixed_nested;
use super::schema::{FieldType, MessageSchema};
use crate::shm_runtime::read_offset_entry;
use crate::wire::WireHeader;

/// Hard ceiling on one transcoded frame (1 GiB) — a corrupt/hostile length
/// prefix must produce an [`CdrCodecError`], never an OOM abort. Mirrors
/// `rmw_cerulion::type_bridge::MAX_FRAME_BYTES`.
pub const MAX_CDR_FRAME_BYTES: usize = 1 << 30;

/// The most trailing bytes a legitimately-framed CDR body can carry beyond its
/// last field: the CONSUMPTION allowance enforced by [`CdrCodec::decode`].
///
/// An RTPS `SerializedPayload` is padded up to the enclosing submessage's
/// 4-byte alignment, so a conformant writer can leave at most **3** bytes the
/// field walk does not read. Anything more cannot be alignment padding, so it
/// is structural evidence that the schema used to decode is not the schema the
/// writer used — see [`CdrCodecError::TrailingBytes`] for why that matters and
/// what the residual is.
pub const MAX_CDR_TRAILING_PAD: usize = 3;

/// Byte order of a CDR body, dispatched from the DDS representation
/// identifier ([`split_encapsulation`]). Cerulion's own fixed section is
/// ALWAYS little-endian, so decode normalizes big-endian primitives to LE
/// and encode denormalizes back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CdrEndianness {
    /// CDR_LE (`rep_id` 0x0001) — what the go2 side always emits.
    Little,
    /// CDR_BE (`rep_id` 0x0000).
    Big,
}

impl CdrEndianness {
    #[inline]
    fn is_big(self) -> bool {
        matches!(self, CdrEndianness::Big)
    }
}

/// Octets a MEMBER-LESS message occupies on the CDR wire.
///
/// IDL forbids an empty struct, so ROS 2's IDL generation gives every
/// member-less message one placeholder member —
/// `uint8 structure_needs_at_least_one_member` — and `std_msgs/Empty`,
/// `statistics_msgs/StatisticDataType` and friends therefore serialize to ONE
/// octet, not zero. Cerulion models such a message as genuinely field-less (an
/// `Empty` frame carries no payload), so the codec has to bridge the
/// difference explicitly: decode SKIPS the octet, encode re-emits it as zero
/// (rosidl zero-initializes it and nothing ever writes it, so it carries no
/// information).
///
/// This is not cosmetic. A member-less message NESTED inside another shifts
/// every following field by one byte if it is unaccounted for, and the
/// leftover lands at 1 byte — inside [`MAX_CDR_TRAILING_PAD`], where
/// [`CdrCodecError::TrailingBytes`] cannot see it. Skipping it here is what
/// keeps that from being a silent mis-decode.
const IDL_EMPTY_STRUCT_OCTETS: usize = 1;

/// A transcode failure. Total (never panics); every variant names the schema
/// (and, where meaningful, the field) so a mis-decode is loud, not silent.
/// `Clone + PartialEq + Eq` so oracle tests can assert an exact variant.
///
/// `#[non_exhaustive]` because this enum grows whenever the codec learns to
/// name a new failure class (as it did with [`Self::TrailingBytes`]); a
/// downstream crate must therefore carry a wildcard arm rather than break on
/// the next one. Enum-level only — every existing variant is still
/// constructible outside this crate (the go2 bridge's tests assert exact
/// variants), and adding the attribute is only free while the crate is
/// unpublished, which is why it lands with the variant that motivated it.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum CdrCodecError {
    /// No [`MessageSchema`] in the codec's set has this qualified name — the
    /// acceptance-critical "no schema" (never "no codec") signal.
    #[error("unknown schema '{0}' (no MessageSchema in the codec's set)")]
    UnknownSchema(String),

    /// A read ran past the end of the CDR body (a truncated frame).
    #[error(
        "CDR body for '{schema}' field '{field}' truncated: need {need} more bytes at \
         offset {pos}, but only {have} remain"
    )]
    Truncated {
        schema: String,
        field: String,
        pos: usize,
        need: usize,
        have: usize,
    },

    /// A length/count prefix claims more elements than the remaining body
    /// could possibly hold — a corrupt or hostile frame. Rejected BEFORE any
    /// allocation is sized from the count (the DoS guard).
    #[error(
        "CDR body for '{schema}' field '{field}' has a hostile length/count prefix {claimed} \
         over only {available} remaining bytes"
    )]
    HostileLength {
        schema: String,
        field: String,
        claimed: u64,
        available: usize,
    },

    /// A `string` field's bytes are not valid UTF-8 — the same rejection the
    /// serde-CDR engine makes (identical Err class for the anti-regression
    /// oracle).
    #[error("CDR string in '{schema}' field '{field}' is not valid UTF-8")]
    InvalidUtf8 { schema: String, field: String },

    /// A `string` field's framing is malformed: a zero length prefix (a
    /// well-formed CDR string always contains at least its NUL terminator)
    /// or a final byte that is not the NUL terminator. Rejected loudly —
    /// silently dropping the last byte would corrupt the value AND break the
    /// documented `encode(decode(cdr)) == cdr`
    /// inverse.
    #[error("CDR string in '{schema}' field '{field}' is malformed: {detail}")]
    MalformedCdrString {
        schema: String,
        field: String,
        detail: String,
    },

    /// A field type the codec cannot map to/from CDR (e.g. `string_fixed[n]`,
    /// which has no standard CDR encoding).
    #[error("field '{field}' in '{schema}' is unsupported by the CDR codec: {detail}")]
    Unsupported {
        schema: String,
        field: String,
        detail: String,
    },

    /// A nested reference did not resolve to any schema in the codec's set
    /// (the nested target was omitted when the codec was built).
    #[error("nested reference '{target}' in '{schema}' field '{field}' resolves to no schema in the codec's set")]
    NestedResolutionFailed {
        schema: String,
        field: String,
        target: String,
    },

    /// The DDS encapsulation header is shorter than its 4 bytes.
    #[error("DDS encapsulation header too short: need {need} bytes, have {have}")]
    MissingEncapsulation { need: usize, have: usize },

    /// The DDS representation identifier is neither CDR_LE nor CDR_BE (e.g.
    /// PL_CDR / a parameter-list encapsulation, or garbage).
    #[error("unsupported CDR representation id {rep_id:#06x} (only CDR_LE 0x0001 and CDR_BE 0x0000 are supported)")]
    UnsupportedRepresentation { rep_id: u16 },

    /// The Cerulion wire frame/payload handed to [`CdrCodec::encode`] is
    /// malformed (shorter than its header/fixed section+table, or an
    /// offset-table entry out of bounds).
    #[error("malformed Cerulion wire payload for '{schema}': {detail}")]
    BadWirePayload { schema: String, detail: String },

    /// The transcoded frame would exceed [`MAX_CDR_FRAME_BYTES`].
    #[error("transcoded frame for '{schema}' exceeds the {cap}-byte cap")]
    FrameTooLarge { schema: String, cap: usize },

    /// The field walk SUCCEEDED but left more unconsumed trailing bytes than a
    /// CDR alignment pad could explain ([`MAX_CDR_TRAILING_PAD`]) — the
    /// wrong-schema detector.
    ///
    /// # Why this variant has to exist
    ///
    /// On the `ros2 attach` / `dds_bridge` ingress path the schema is resolved
    /// from the LOCAL corpus (workspace `.msg` store + built-in registry)
    /// before the robot's own served type text, and on divergence the robot's
    /// text is discarded. Nothing downstream can catch a wrong corpus entry:
    /// the publisher STAMPS the wire `schema_hash` from that same entry and
    /// the subscriber EXPECTS it from that same entry, so both sides of the
    /// hash gate are wrong together and it never fires. A definition that is
    /// merely SHORTER than the writer's therefore decodes "successfully" into
    /// garbage — every field after the divergence read at the wrong offset —
    /// as long as the byte lengths happen to work out.
    ///
    /// A real incident was exactly this shape: the community `unitree_go/
    /// Go2FrontVideoData` definition disagreed with what the Go2 firmware
    /// publishes, and the video payload was read at the wrong offset producing
    /// plausible-looking values. It was caught only because a length prefix
    /// went hostile ([`CdrCodecError::HostileLength`]) — an accident of that
    /// particular divergence, not a guarantee.
    ///
    /// # Known residual
    ///
    /// A wrong schema whose walk stops 1..=3 bytes short is INDISTINGUISHABLE
    /// from alignment padding and is NOT caught.
    ///
    /// Two tightenings look attractive and neither one works:
    ///
    /// - **The encapsulation header's declared pad count** (XTypes 1.3
    ///   §7.4.3.5 puts it in the low 2 bits of the options field). It never
    ///   reaches this seam: the production ingress hands the codec a BARE
    ///   body, because rustdds parses the representation identifier AND the
    ///   options off upstream (`dds_bridge`'s `RawSample`), and even the
    ///   [`CdrCodec::decode_dds_payload`] convenience goes through
    ///   [`split_encapsulation`], which returns `&payload[4..]` and discards
    ///   the options bytes outright. Threading it would mean re-plumbing raw
    ///   options through every ingress caller.
    ///
    ///   Unavailability is the WHOLE argument, and TWO things are
    ///   deliberately NOT claimed alongside it. The field is not dead on
    ///   XCDR1 traffic: the three committed Go2 captures each declare an
    ///   EXACT, NON-ZERO pad (3, 2 and 3 bytes) matching their real leftover
    ///   — see `examples/go2/nodes/dds_bridge/tests/frontvideostream_wire_test.rs`,
    ///   which cross-checks the two and pins the counts per fixture. Nor is
    ///   it redundant with [`MAX_CDR_TRAILING_PAD`]: those same fixtures
    ///   refute that too, since the 720p capture declares 2, which WOULD
    ///   sharpen the allowance from 3 to 2 on that sample. Threading it is a
    ///   real tightening this codec forgoes for reach — not an idea that buys
    ///   nothing.
    /// - **A congruence rule** (`remaining == (4 - consumed % 4) % 4`, since
    ///   the 4-byte header means submessage padding rounds the BODY to 4).
    ///   Padding the LAST submessage is writer-discretionary, and a
    ///   fragmented sample reassembles to the writer-declared `sampleSize`
    ///   with no rounding at all — and fragmented means large, i.e. exactly
    ///   the camera and pointcloud topics. A congruence rule would
    ///   false-positive per-vendor on the highest-value streams.
    #[error(
        "CDR body for '{schema}' was NOT fully consumed: the schema's fields account for \
         {consumed} of {} body bytes, leaving {remaining} unconsumed — more than the 3 bytes \
         a CDR alignment pad can explain. This is structural evidence that the schema used to \
         decode is NOT the one the writer used (a wrong or stale local definition decodes \
         silently into garbage). Fix, if this type came from the workspace \
         store: delete the stale schemas/<pkg>/msg/<Type>.msg and re-run `cerulion ros2 \
         attach`, which materializes the robot's own type description over the wire — UNLESS \
         a COMPILED-IN definition of the same name exists, which then resolves the type and \
         the wire rung never runs, so use the next fix instead. Fix, if it came from a \
         COMPILED-IN corpus (native_ros2_messages BUILTIN_MSGS, or the go2 bridge's \
         UNITREE_MSGS) where there is no file to delete: WRITE a corrected \
         schemas/<pkg>/msg/<Type>.msg — a workspace store definition SHADOWS a compiled one \
         of the same qualified name (the store wins, loudly) — and then make the bridge LOAD \
         that store, which it does ONLY when its config carries msg_dirs: either re-run \
         `cerulion ros2 attach` (the store is now non-empty, so the regenerated config emits \
         the key) or add `msg_dirs: [../schemas]` to graphs/<graph>.bridge.yaml by hand",
        consumed + remaining
    )]
    TrailingBytes {
        schema: String,
        /// Bytes the walk accounted for (`body.len() - remaining`). Derived,
        /// not the raw cursor: a trailing `align()` can leave the cursor past
        /// the end of the body, where the raw value would over-report.
        consumed: usize,
        /// Bytes left over after the walk (`body.len() - consumed`).
        remaining: usize,
    },
}

/// Split a full DDS `SerializedPayload` (`rep_id:u16(BE) | options:u16 |
/// body`) into `(endianness, body)`. This mirrors the encapsulation-header
/// handling in `examples/go2/lib/cerulion_go2_dds/src/cdr.rs` (the format is
/// RTPS 9.4.2: the 16-bit representation id is stored big-endian; CDR_LE =
/// `[0x00, 0x01]`); it is re-implemented here (4 trivial bytes) so this core
/// module is self-contained and directly usable without a `cerulion_go2_dds`
/// dependency (which would be a cycle — that crate depends on core).
pub fn split_encapsulation(dds_payload: &[u8]) -> Result<(CdrEndianness, &[u8]), CdrCodecError> {
    if dds_payload.len() < 4 {
        return Err(CdrCodecError::MissingEncapsulation {
            need: 4,
            have: dds_payload.len(),
        });
    }
    let rep_id = u16::from_be_bytes([dds_payload[0], dds_payload[1]]);
    let endian = match rep_id {
        0x0001 => CdrEndianness::Little,
        0x0000 => CdrEndianness::Big,
        other => return Err(CdrCodecError::UnsupportedRepresentation { rep_id: other }),
    };
    Ok((endian, &dds_payload[4..]))
}

// ===========================================================================
// CDR read/write cursors
// ===========================================================================

/// Bounds-checked reader over a CDR body. `pos` is body-relative (byte 0 is
/// the field-aligned origin — the byte after the encapsulation header).
struct CdrReader<'a> {
    body: &'a [u8],
    pos: usize,
    big_endian: bool,
}

impl<'a> CdrReader<'a> {
    fn new(body: &'a [u8], endian: CdrEndianness) -> Self {
        Self {
            body,
            pos: 0,
            big_endian: endian.is_big(),
        }
    }

    #[inline]
    fn remaining(&self) -> usize {
        self.body.len().saturating_sub(self.pos)
    }

    /// Advance `pos` to the next multiple of `a` (CDR alignment padding). A
    /// subsequent [`Self::take`] bounds-checks any over-run — align itself
    /// never reads, so padding past the end is harmless until a read follows.
    #[inline]
    fn align(&mut self, a: usize) {
        let rem = self.pos % a;
        if rem != 0 {
            self.pos += a - rem;
        }
    }

    fn take(&mut self, n: usize, schema: &str, field: &str) -> Result<&'a [u8], CdrCodecError> {
        let end = self.pos.checked_add(n);
        match end {
            Some(end) if end <= self.body.len() => {
                let slice = &self.body[self.pos..end];
                self.pos = end;
                Ok(slice)
            }
            _ => Err(CdrCodecError::Truncated {
                schema: schema.to_string(),
                field: field.to_string(),
                pos: self.pos,
                need: n,
                have: self.remaining(),
            }),
        }
    }

    fn read_u32(&mut self, schema: &str, field: &str) -> Result<u32, CdrCodecError> {
        self.align(4);
        let b = self.take(4, schema, field)?;
        Ok(if self.big_endian {
            u32::from_be_bytes([b[0], b[1], b[2], b[3]])
        } else {
            u32::from_le_bytes([b[0], b[1], b[2], b[3]])
        })
    }

    /// Read a primitive of byte width `size` (1/2/4/8), returning its
    /// LITTLE-ENDIAN byte representation (Cerulion's fixed section is always
    /// LE). Big-endian bodies are byte-reversed here.
    fn read_prim_le(
        &mut self,
        size: usize,
        schema: &str,
        field: &str,
    ) -> Result<[u8; 8], CdrCodecError> {
        self.align(size);
        let b = self.take(size, schema, field)?;
        let mut out = [0u8; 8];
        if self.big_endian {
            for (i, dst) in out.iter_mut().take(size).enumerate() {
                *dst = b[size - 1 - i];
            }
        } else {
            out[..size].copy_from_slice(b);
        }
        Ok(out)
    }
}

/// CDR body writer (encode direction). `buf.len()` is the body-relative
/// cursor, so alignment padding is `buf.len() % a`.
struct CdrWriter {
    buf: Vec<u8>,
    big_endian: bool,
}

impl CdrWriter {
    fn new(endian: CdrEndianness) -> Self {
        Self {
            buf: Vec::new(),
            big_endian: endian.is_big(),
        }
    }

    #[inline]
    fn align(&mut self, a: usize) {
        while !self.buf.len().is_multiple_of(a) {
            self.buf.push(0);
        }
    }

    /// Write a primitive whose LITTLE-ENDIAN bytes are `le` (as stored in the
    /// Cerulion fixed section), byte-reversing for a big-endian body.
    fn write_prim_le(&mut self, le: &[u8]) {
        let size = le.len();
        self.align(size);
        if self.big_endian {
            for &b in le.iter().rev() {
                self.buf.push(b);
            }
        } else {
            self.buf.extend_from_slice(le);
        }
    }

    fn write_u32(&mut self, v: u32) {
        self.align(4);
        if self.big_endian {
            self.buf.extend_from_slice(&v.to_be_bytes());
        } else {
            self.buf.extend_from_slice(&v.to_le_bytes());
        }
    }

    /// Write a CDR string: `u32(content.len()+1) | content | 0x00`.
    fn write_string(&mut self, content: &[u8]) {
        self.write_u32(content.len() as u32 + 1);
        self.buf.extend_from_slice(content);
        self.buf.push(0);
    }
}

// ===========================================================================
// The codec
// ===========================================================================

/// A schema-driven CDR ⇄ Cerulion-wire transcoder.
///
/// Build once from a schema set (e.g. every `native_ros2_messages::BUILTIN_MSGS`
/// entry parsed via [`crate::codegen::parse_rosmsg`], the same set
/// [`FrameWalker::new`](crate::codegen::FrameWalker::new) takes), then
/// [`decode`](Self::decode) / [`encode`](Self::encode) any number of frames.
pub struct CdrCodec {
    /// Qualified name → resolved schema (declaration order drives the CDR
    /// walk; the `fixed` flags on nested fields are resolved).
    schemas: BTreeMap<String, MessageSchema>,
    /// Qualified name → wire layout (drives Cerulion fixed offsets + the
    /// offset table).
    layouts: BTreeMap<String, WireLayout>,
}

impl CdrCodec {
    /// Build a codec over `schemas`. Runs nested-fixed resolution + layout
    /// computation (the SAME pipeline codegen runs at build time) and returns
    /// any resolution warnings alongside — callers should surface them (a
    /// warning means a nested reference could not be resolved to a target).
    pub fn new(schemas: Vec<MessageSchema>) -> (Self, Vec<String>) {
        let mut schemas = schemas;
        let warnings = resolve_fixed_nested(&mut schemas);
        // Snapshot the RESOLVED schemas (LayoutResolver::new re-resolves
        // idempotently, so this snapshot matches the layouts' classification).
        let schema_map: BTreeMap<String, MessageSchema> = schemas
            .iter()
            .map(|s| (s.qualified_name(), s.clone()))
            .collect();
        let qnames: Vec<String> = schemas.iter().map(|s| s.qualified_name()).collect();
        let (mut resolver, _resolver_warnings) = LayoutResolver::new(schemas);
        let mut layouts = BTreeMap::new();
        for q in qnames {
            if let Some(layout) = resolver.layout_of(&q) {
                layouts.insert(q, layout);
            }
        }
        (
            Self {
                schemas: schema_map,
                layouts,
            },
            warnings,
        )
    }

    /// True if `qualified_name` can be transcoded by this codec.
    pub fn knows(&self, qualified_name: &str) -> bool {
        self.layouts.contains_key(qualified_name)
    }

    /// The wire `schema_hash` for a known schema (for stamping/validation).
    pub fn schema_hash(&self, qualified_name: &str) -> Option<u64> {
        self.layouts.get(qualified_name).map(|l| l.schema_hash)
    }

    /// Narrow `configured` to this schema's own per-route SLICE
    /// budget — the number that decides how deep a dynamically-created ingress
    /// route's receive queue can be
    /// ([`crate::transport::ingress_route_buffer_depth`]).
    ///
    /// `None` for a schema this codec does not know: a caller that cannot
    /// resolve the type must keep the slice it was configured with rather than
    /// invent one. See [`crate::codegen::route_budget`] for the rule, both arms,
    /// and the residual it leaves.
    pub fn route_slice_budget(
        &self,
        qualified_name: &str,
        configured: crate::wire::MaxSliceLen,
    ) -> Option<crate::wire::MaxSliceLen> {
        self.layouts.get(qualified_name).map(|layout| {
            crate::codegen::route_budget::route_slice_budget_capped(layout, configured)
        })
    }

    /// Decode a CDR body into a complete Cerulion wire frame (32-byte
    /// [`WireHeader`] + payload). `sequence` / `timestamp_ns` stamp the header
    /// (the publisher's counters/clock — deterministic under replay).
    pub fn decode(
        &self,
        qualified_name: &str,
        endian: CdrEndianness,
        cdr_body: &[u8],
        sequence: u32,
        timestamp_ns: u64,
    ) -> Result<Vec<u8>, CdrCodecError> {
        let layout = self
            .layouts
            .get(qualified_name)
            .ok_or_else(|| CdrCodecError::UnknownSchema(qualified_name.to_string()))?;
        let mut reader = CdrReader::new(cdr_body, endian);
        let payload = self.decode_payload(qualified_name, &mut reader)?;

        // The NOT-FULLY-CONSUMED gate. The walk above
        // succeeding proves only that the body was long enough for THIS
        // schema, not that it IS this schema; leftover bytes beyond the
        // alignment allowance say the writer serialized something else. Loud
        // here, because no gate downstream can see it (both sides of the
        // schema-hash check are minted from the same local corpus entry — see
        // `CdrCodecError::TrailingBytes`).
        //
        // Placed at the TOP-LEVEL entry only: `decode_payload` recurses for
        // variable-nested sub-frames off the SAME reader, so a nested walk
        // legitimately stops mid-body and only the outermost cursor is
        // meaningful.
        let remaining = reader.remaining();
        if remaining > MAX_CDR_TRAILING_PAD {
            return Err(CdrCodecError::TrailingBytes {
                schema: qualified_name.to_string(),
                consumed: cdr_body.len() - remaining,
                remaining,
            });
        }

        let total = WireHeader::SIZE
            .checked_add(payload.len())
            .filter(|&n| n <= MAX_CDR_FRAME_BYTES)
            .ok_or_else(|| CdrCodecError::FrameTooLarge {
                schema: qualified_name.to_string(),
                cap: MAX_CDR_FRAME_BYTES,
            })?;

        let header = WireHeader {
            schema_hash: layout.schema_hash,
            total_size: total as u32,
            // FRAME-relative (byte offset from MESSAGE start — the wire.rs
            // contract, and exactly what the native publisher stamps:
            // `WireHeader::SIZE + T::WIRE_FIXED_SIZE`, publisher.rs). A
            // payload-relative value here made a dds-bridged frame differ
            // from a natively-published one at header bytes [12..16]
            // (a confirmed defect class). Note: the offset-table
            // ENTRIES remain payload-relative (the `read_offset_entry`
            // convention shared with the generated readers + FrameWalker) —
            // only this header FIELD is frame-relative.
            offset_table_offset: (WireHeader::SIZE + layout.fixed_size) as u32,
            offset_table_count: layout.variable_fields.len() as u32,
            sequence,
            timestamp_ns,
        };
        let mut frame = vec![0u8; WireHeader::SIZE];
        header.write_to_buf(&mut frame);
        frame.extend_from_slice(&payload);
        Ok(frame)
    }

    /// Convenience: split a full DDS payload (with the 4-byte encapsulation
    /// header) and [`decode`](Self::decode) its body.
    pub fn decode_dds_payload(
        &self,
        qualified_name: &str,
        dds_payload: &[u8],
        sequence: u32,
        timestamp_ns: u64,
    ) -> Result<Vec<u8>, CdrCodecError> {
        let (endian, body) = split_encapsulation(dds_payload)?;
        self.decode(qualified_name, endian, body, sequence, timestamp_ns)
    }

    /// Encode a complete Cerulion wire frame (32-byte header + payload) into a
    /// CDR body (the egress/actuator direction). The header is stripped and
    /// ignored — only the schema (`qualified_name`) + the payload drive the
    /// encoding, so `encode(decode(cdr)) == cdr`.
    pub fn encode(
        &self,
        qualified_name: &str,
        endian: CdrEndianness,
        cerulion_frame: &[u8],
    ) -> Result<Vec<u8>, CdrCodecError> {
        if !self.layouts.contains_key(qualified_name) {
            return Err(CdrCodecError::UnknownSchema(qualified_name.to_string()));
        }
        if cerulion_frame.len() < WireHeader::SIZE {
            return Err(CdrCodecError::BadWirePayload {
                schema: qualified_name.to_string(),
                detail: format!(
                    "frame is {} bytes, shorter than the {}-byte WireHeader",
                    cerulion_frame.len(),
                    WireHeader::SIZE
                ),
            });
        }
        let payload = &cerulion_frame[WireHeader::SIZE..];
        let mut writer = CdrWriter::new(endian);
        self.encode_payload(qualified_name, payload, &mut writer)?;
        Ok(writer.buf)
    }

    // -- decode (CDR → Cerulion) --------------------------------------------

    /// Build the Cerulion payload `[fixed][offset table][var]` for `qname` by
    /// walking its fields in declaration order over the CDR reader. This is
    /// the recursive workhorse: the top-level frame and every variable-nested
    /// sub-frame flow through it.
    fn decode_payload(
        &self,
        qname: &str,
        reader: &mut CdrReader<'_>,
    ) -> Result<Vec<u8>, CdrCodecError> {
        let layout = self
            .layouts
            .get(qname)
            .ok_or_else(|| CdrCodecError::UnknownSchema(qname.to_string()))?;
        let schema = self
            .schemas
            .get(qname)
            .ok_or_else(|| CdrCodecError::UnknownSchema(qname.to_string()))?;

        let mut fixed = vec![0u8; layout.fixed_size];
        let mut var_bufs: Vec<Vec<u8>> = Vec::with_capacity(layout.variable_fields.len());
        let mut fixed_idx = 0usize;

        // A member-less message is one placeholder octet on the wire and zero
        // fields here — step over it before the (empty) walk.
        self.skip_idl_placeholder(qname, reader);

        for field in &schema.fields {
            if field.field_type.is_variable() {
                let bytes =
                    self.decode_cdr_variable(qname, &field.name, &field.field_type, reader)?;
                var_bufs.push(bytes);
            } else {
                let fl = &layout.fixed_fields[fixed_idx];
                debug_assert_eq!(fl.name, field.name, "fixed-field lockstep desync");
                self.decode_cdr_fixed(
                    qname,
                    &field.name,
                    &field.field_type,
                    reader,
                    &mut fixed,
                    fl.offset,
                )?;
                fixed_idx += 1;
            }
        }

        // Assemble [fixed][table][var] through the SHARED canonical body
        // codec — the same routine the rmw introspection bridges
        // now call, so there is exactly ONE implementation of this convention
        // instead of the two that silently disagreed for a year.
        let mut builder = CanonicalBodyBuilder::new(layout);
        builder.fixed_mut().copy_from_slice(&fixed);
        for bytes in var_bufs {
            builder
                .push_variable(bytes)
                .map_err(|e| Self::body_err(qname, e))?;
        }
        builder.finish().map_err(|e| Self::body_err(qname, e))
    }

    /// True when `qname` names a MEMBER-LESS message (no fields — a
    /// comment-only or constants-only `.msg`), which occupies
    /// [`IDL_EMPTY_STRUCT_OCTETS`] on the CDR wire but nothing in a Cerulion
    /// payload.
    fn is_member_less(&self, qname: &str) -> bool {
        self.schemas.get(qname).is_some_and(|s| s.fields.is_empty())
    }

    /// Step the reader over a member-less message's IDL placeholder octet.
    ///
    /// TOLERANT of its absence by design — but the tolerance is only free at
    /// the TOP level, and the scope is worth stating exactly:
    ///
    /// - **Top level.** The octet carries no information, so a writer that
    ///   omits it (a bare DDS app that never went through rosidl) costs us
    ///   nothing to accept: nothing follows it, so the `remaining >= 1` guard
    ///   simply declines to move a cursor that is already at the end. Refusing
    ///   such a body would trade this fix for a fresh dead-route class, the
    ///   very failure mode the upstream-drift gate exists to prevent.
    /// - **Nested.** Here absence is NOT free. The guard sees the ENCLOSING
    ///   message's remaining bytes, so it cannot tell "the placeholder is
    ///   there" from "the next field starts here", and skipping when the
    ///   writer omitted the octet eats the first byte of the following field.
    ///   That is a MIS-DECODE, and it is accepted deliberately: the writer is
    ///   non-conforming (rosidl emits the member unconditionally), and the
    ///   damage is not silent in practice — the one-byte shift desynchronizes
    ///   the rest of the walk. The stolen byte makes the walk consume MORE
    ///   than it should, so on an all-FIXED enclosing message the only
    ///   possible outcome is `Truncated` (over-running the body end); once a
    ///   VARIABLE field's length prefix is read at the wrong offset it is
    ///   `HostileLength` instead. [`CdrCodecError::TrailingBytes`] is
    ///   reachable only in that second position, and only if the misread
    ///   prefix happens to come out SMALL enough to under-read the body — it
    ///   is structurally unreachable on a fixed-only message. None of the
    ///   three is *guaranteed*, which is why the case is recorded here rather
    ///   than claimed away.
    ///
    /// What must not happen in either position is silently LEAVING a present
    /// placeholder in the stream, which mis-aligns every following field of an
    /// enclosing message by exactly one byte — inside the trailing-pad
    /// allowance, where the gate structurally cannot see it.
    fn skip_idl_placeholder(&self, qname: &str, reader: &mut CdrReader<'_>) {
        if self.is_member_less(qname) && reader.remaining() >= IDL_EMPTY_STRUCT_OCTETS {
            reader.pos += IDL_EMPTY_STRUCT_OCTETS;
        }
    }

    /// Write a member-less message's IDL placeholder octet (the inverse of
    /// [`Self::skip_idl_placeholder`]), so an egress frame is the length a ROS
    /// 2 reader expects.
    fn write_idl_placeholder(&self, qname: &str, writer: &mut CdrWriter) {
        if self.is_member_less(qname) {
            writer
                .buf
                .extend(std::iter::repeat_n(0u8, IDL_EMPTY_STRUCT_OCTETS));
        }
    }

    /// Map a shared-codec body error into this codec's error type.
    fn body_err(schema: &str, e: ElementBodyError) -> CdrCodecError {
        CdrCodecError::BadWirePayload {
            schema: schema.to_string(),
            detail: e.as_str().to_string(),
        }
    }

    /// Read one fixed-section field from CDR and write its little-endian bytes
    /// into `fixed` at `offset` (recursing through fixed arrays + fixed nested
    /// with their `#[repr(C)]` sub-offsets).
    fn decode_cdr_fixed(
        &self,
        schema: &str,
        field: &str,
        ft: &FieldType,
        reader: &mut CdrReader<'_>,
        fixed: &mut [u8],
        offset: usize,
    ) -> Result<(), CdrCodecError> {
        match ft {
            FieldType::Bool
            | FieldType::I8
            | FieldType::U8
            | FieldType::I16
            | FieldType::U16
            | FieldType::I32
            | FieldType::U32
            | FieldType::I64
            | FieldType::U64
            | FieldType::F32
            | FieldType::F64 => {
                let size = ft.fixed_size().expect("primitive has a fixed size");
                let le = reader.read_prim_le(size, schema, field)?;
                let dst = fixed.get_mut(offset..offset + size).ok_or_else(|| {
                    CdrCodecError::BadWirePayload {
                        schema: schema.to_string(),
                        detail: format!("fixed field '{field}' write out of bounds"),
                    }
                })?;
                dst.copy_from_slice(&le[..size]);
                Ok(())
            }
            FieldType::StringFixed(_) => Err(CdrCodecError::Unsupported {
                schema: schema.to_string(),
                field: field.to_string(),
                detail: "string_fixed[n] has no standard CDR encoding".to_string(),
            }),
            FieldType::FixedArray {
                element_type,
                length,
            } => {
                let stride =
                    element_type
                        .fixed_size()
                        .ok_or_else(|| CdrCodecError::BadWirePayload {
                            schema: schema.to_string(),
                            detail: format!("fixed array '{field}' has a non-fixed element"),
                        })?;
                for i in 0..*length {
                    self.decode_cdr_fixed(
                        schema,
                        field,
                        element_type,
                        reader,
                        fixed,
                        offset + i * stride,
                    )?;
                }
                Ok(())
            }
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                let nq = self.resolve_nested_qname(schema, field, schema_name, package)?;
                let nlayout = self
                    .layouts
                    .get(&nq)
                    .ok_or_else(|| CdrCodecError::UnknownSchema(nq.clone()))?;
                // A member-less nested target contributes its placeholder
                // octet and no fields; skipping it here is what keeps the
                // FOLLOWING field of this message aligned.
                self.skip_idl_placeholder(&nq, reader);
                // A fixed nested has every field in `fixed_fields`, in CDR
                // declaration order — walk those directly at the base offset.
                for nfl in &nlayout.fixed_fields {
                    self.decode_cdr_fixed(
                        &nq,
                        &nfl.name,
                        &nfl.field_type,
                        reader,
                        fixed,
                        offset + nfl.offset,
                    )?;
                }
                Ok(())
            }
            FieldType::String | FieldType::Bytes | FieldType::DynamicArray { .. } => {
                Err(CdrCodecError::BadWirePayload {
                    schema: schema.to_string(),
                    detail: format!("variable field '{field}' reached the fixed-section decoder"),
                })
            }
        }
    }

    /// Read one variable field from CDR and return its Cerulion variable-
    /// payload bytes (the offset-table entry's slice).
    fn decode_cdr_variable(
        &self,
        schema: &str,
        field: &str,
        ft: &FieldType,
        reader: &mut CdrReader<'_>,
    ) -> Result<Vec<u8>, CdrCodecError> {
        match ft {
            FieldType::String => self.read_cdr_string(schema, field, reader),
            FieldType::Bytes => {
                let count = reader.read_u32(schema, field)? as usize;
                guard_count(schema, field, count, reader.remaining())?;
                Ok(reader.take(count, schema, field)?.to_vec())
            }
            FieldType::DynamicArray { element_type } => {
                let count = reader.read_u32(schema, field)? as usize;
                self.decode_cdr_seq(schema, field, element_type, count, reader)
            }
            FieldType::FixedArray {
                element_type,
                length,
            } => {
                // Reaches here only when the element is VARIABLE (a variable
                // fixed-array). CDR has NO count for a fixed array.
                self.decode_cdr_seq(schema, field, element_type, *length, reader)
            }
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                // A single variable nested field → headerless sub-frame.
                let nq = self.resolve_nested_qname(schema, field, schema_name, package)?;
                self.decode_payload(&nq, reader)
            }
            FieldType::StringFixed(_)
            | FieldType::Bool
            | FieldType::I8
            | FieldType::U8
            | FieldType::I16
            | FieldType::U16
            | FieldType::I32
            | FieldType::U32
            | FieldType::I64
            | FieldType::U64
            | FieldType::F32
            | FieldType::F64 => Err(CdrCodecError::BadWirePayload {
                schema: schema.to_string(),
                detail: format!("fixed field '{field}' reached the variable decoder"),
            }),
        }
    }

    /// Decode `count` elements of `element_type` from CDR into the Cerulion
    /// variable-payload encoding for a sequence/array of that element.
    fn decode_cdr_seq(
        &self,
        schema: &str,
        field: &str,
        element_type: &FieldType,
        count: usize,
        reader: &mut CdrReader<'_>,
    ) -> Result<Vec<u8>, CdrCodecError> {
        guard_count(schema, field, count, reader.remaining())?;
        match element_type {
            FieldType::Bool | FieldType::I8 | FieldType::U8 => {
                // raw bytes (1-byte elements, no per-element alignment).
                Ok(reader.take(count, schema, field)?.to_vec())
            }
            FieldType::I16
            | FieldType::U16
            | FieldType::I32
            | FieldType::U32
            | FieldType::I64
            | FieldType::U64
            | FieldType::F32
            | FieldType::F64 => {
                let size = element_type.fixed_size().expect("primitive");
                let mut out = Vec::new();
                for _ in 0..count {
                    let le = reader.read_prim_le(size, schema, field)?;
                    out.extend_from_slice(&le[..size]);
                }
                Ok(out)
            }
            FieldType::String => {
                // Cerulion: u32 count + per element [u32 len][UTF-8].
                let mut out = Vec::new();
                out.extend_from_slice(&(count as u32).to_le_bytes());
                for _ in 0..count {
                    let content = self.read_cdr_string(schema, field, reader)?;
                    out.extend_from_slice(&(content.len() as u32).to_le_bytes());
                    out.extend_from_slice(&content);
                }
                Ok(out)
            }
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                let nq = self.resolve_nested_qname(schema, field, schema_name, package)?;
                let nlayout = self
                    .layouts
                    .get(&nq)
                    .ok_or_else(|| CdrCodecError::UnknownSchema(nq.clone()))?;
                if nlayout.is_fixed() {
                    // Back-to-back fixed sections (stride = padded size), NO
                    // count prefix — the reader recovers count from len/stride.
                    let mut out = Vec::new();
                    for _ in 0..count {
                        let sub = self.decode_payload(&nq, reader)?;
                        out.extend_from_slice(&sub);
                    }
                    Ok(out)
                } else {
                    // u32 count + per element [u32 len][sub-frame].
                    let mut out = Vec::new();
                    out.extend_from_slice(&(count as u32).to_le_bytes());
                    for _ in 0..count {
                        let sub = self.decode_payload(&nq, reader)?;
                        out.extend_from_slice(&(sub.len() as u32).to_le_bytes());
                        out.extend_from_slice(&sub);
                    }
                    Ok(out)
                }
            }
            other => Err(CdrCodecError::Unsupported {
                schema: schema.to_string(),
                field: field.to_string(),
                detail: format!("array of {} is not supported", other.canonical_str()),
            }),
        }
    }

    /// Read one CDR string (`u32(len+1) | utf8 | NUL`) and return its UTF-8
    /// content bytes (no NUL). Strict: a zero
    /// length prefix and a non-NUL final byte are loud
    /// [`CdrCodecError::MalformedCdrString`]s — leniently dropping the last
    /// byte would corrupt the value and break the `encode(decode(cdr)) ==
    /// cdr` inverse. UTF-8 is validated (matching the serde-CDR engine).
    fn read_cdr_string(
        &self,
        schema: &str,
        field: &str,
        reader: &mut CdrReader<'_>,
    ) -> Result<Vec<u8>, CdrCodecError> {
        let len = reader.read_u32(schema, field)? as usize;
        if len == 0 {
            return Err(CdrCodecError::MalformedCdrString {
                schema: schema.to_string(),
                field: field.to_string(),
                detail: "length prefix 0 (a CDR string always contains at least its NUL \
                         terminator)"
                    .to_string(),
            });
        }
        guard_count(schema, field, len, reader.remaining())?;
        let bytes = reader.take(len, schema, field)?;
        let (content, terminator) = bytes.split_at(len - 1);
        if terminator != [0] {
            return Err(CdrCodecError::MalformedCdrString {
                schema: schema.to_string(),
                field: field.to_string(),
                detail: format!("missing NUL terminator (final byte {:#04x})", terminator[0]),
            });
        }
        std::str::from_utf8(content).map_err(|_| CdrCodecError::InvalidUtf8 {
            schema: schema.to_string(),
            field: field.to_string(),
        })?;
        Ok(content.to_vec())
    }

    // -- encode (Cerulion → CDR) --------------------------------------------

    /// Emit the CDR body for `qname` from a Cerulion payload
    /// `[fixed][table][var]`. The recursive workhorse for the top-level frame
    /// and every nested sub-frame.
    fn encode_payload(
        &self,
        qname: &str,
        payload: &[u8],
        writer: &mut CdrWriter,
    ) -> Result<(), CdrCodecError> {
        let layout = self
            .layouts
            .get(qname)
            .ok_or_else(|| CdrCodecError::UnknownSchema(qname.to_string()))?;
        let schema = self
            .schemas
            .get(qname)
            .ok_or_else(|| CdrCodecError::UnknownSchema(qname.to_string()))?;

        let head = layout.fixed_size + layout.offset_table_bytes();
        if payload.len() < head {
            return Err(CdrCodecError::BadWirePayload {
                schema: qname.to_string(),
                detail: format!(
                    "payload is {} bytes, shorter than fixed({}) + offset table({})",
                    payload.len(),
                    layout.fixed_size,
                    layout.offset_table_bytes()
                ),
            });
        }

        // Mirror of the decode side: a member-less message must leave its
        // placeholder octet on the wire or a ROS 2 reader sees a short body.
        self.write_idl_placeholder(qname, writer);

        let mut fixed_idx = 0usize;
        let mut var_idx = 0usize;
        for field in &schema.fields {
            if field.field_type.is_variable() {
                let (off, len) = read_offset_entry(payload, layout.fixed_size, var_idx);
                let (off, len) = (off as usize, len as usize);
                let vbytes: &[u8] = if len == 0 {
                    &[]
                } else {
                    let end = off.checked_add(len);
                    let ok = off >= head && end.is_some_and(|e| e <= payload.len());
                    if !ok {
                        return Err(CdrCodecError::BadWirePayload {
                            schema: qname.to_string(),
                            detail: format!(
                                "variable field '{}' offset {off} len {len} out of bounds (payload {})",
                                field.name,
                                payload.len()
                            ),
                        });
                    }
                    &payload[off..off + len]
                };
                self.encode_cdr_variable(qname, &field.name, &field.field_type, vbytes, writer)?;
                var_idx += 1;
            } else {
                let fl = &layout.fixed_fields[fixed_idx];
                debug_assert_eq!(fl.name, field.name, "fixed-field lockstep desync (encode)");
                self.encode_cdr_fixed(
                    qname,
                    &field.name,
                    &field.field_type,
                    payload,
                    fl.offset,
                    writer,
                )?;
                fixed_idx += 1;
            }
        }
        Ok(())
    }

    /// Emit CDR for one fixed-section field, reading its LE bytes from the
    /// payload at `offset` (recursing through fixed arrays + fixed nested).
    fn encode_cdr_fixed(
        &self,
        schema: &str,
        field: &str,
        ft: &FieldType,
        payload: &[u8],
        offset: usize,
        writer: &mut CdrWriter,
    ) -> Result<(), CdrCodecError> {
        match ft {
            FieldType::Bool
            | FieldType::I8
            | FieldType::U8
            | FieldType::I16
            | FieldType::U16
            | FieldType::I32
            | FieldType::U32
            | FieldType::I64
            | FieldType::U64
            | FieldType::F32
            | FieldType::F64 => {
                let size = ft.fixed_size().expect("primitive has a fixed size");
                let le = payload.get(offset..offset + size).ok_or_else(|| {
                    CdrCodecError::BadWirePayload {
                        schema: schema.to_string(),
                        detail: format!("fixed field '{field}' read out of bounds"),
                    }
                })?;
                writer.write_prim_le(le);
                Ok(())
            }
            FieldType::StringFixed(_) => Err(CdrCodecError::Unsupported {
                schema: schema.to_string(),
                field: field.to_string(),
                detail: "string_fixed[n] has no standard CDR encoding".to_string(),
            }),
            FieldType::FixedArray {
                element_type,
                length,
            } => {
                let stride =
                    element_type
                        .fixed_size()
                        .ok_or_else(|| CdrCodecError::BadWirePayload {
                            schema: schema.to_string(),
                            detail: format!("fixed array '{field}' has a non-fixed element"),
                        })?;
                for i in 0..*length {
                    self.encode_cdr_fixed(
                        schema,
                        field,
                        element_type,
                        payload,
                        offset + i * stride,
                        writer,
                    )?;
                }
                Ok(())
            }
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                let nq = self.resolve_nested_qname(schema, field, schema_name, package)?;
                let nlayout = self
                    .layouts
                    .get(&nq)
                    .ok_or_else(|| CdrCodecError::UnknownSchema(nq.clone()))?;
                self.write_idl_placeholder(&nq, writer);
                for nfl in &nlayout.fixed_fields {
                    self.encode_cdr_fixed(
                        &nq,
                        &nfl.name,
                        &nfl.field_type,
                        payload,
                        offset + nfl.offset,
                        writer,
                    )?;
                }
                Ok(())
            }
            FieldType::String | FieldType::Bytes | FieldType::DynamicArray { .. } => {
                Err(CdrCodecError::BadWirePayload {
                    schema: schema.to_string(),
                    detail: format!("variable field '{field}' reached the fixed-section encoder"),
                })
            }
        }
    }

    /// Emit CDR for one variable field, reading the Cerulion variable-payload
    /// slice `vbytes`.
    fn encode_cdr_variable(
        &self,
        schema: &str,
        field: &str,
        ft: &FieldType,
        vbytes: &[u8],
        writer: &mut CdrWriter,
    ) -> Result<(), CdrCodecError> {
        match ft {
            FieldType::String => {
                std::str::from_utf8(vbytes).map_err(|_| CdrCodecError::InvalidUtf8 {
                    schema: schema.to_string(),
                    field: field.to_string(),
                })?;
                writer.write_string(vbytes);
                Ok(())
            }
            FieldType::Bytes => {
                writer.write_u32(vbytes.len() as u32);
                writer.buf.extend_from_slice(vbytes);
                Ok(())
            }
            FieldType::DynamicArray { element_type } => {
                self.encode_cdr_seq(schema, field, element_type, None, vbytes, writer)
            }
            FieldType::FixedArray {
                element_type,
                length,
            } => self.encode_cdr_seq(schema, field, element_type, Some(*length), vbytes, writer),
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                let nq = self.resolve_nested_qname(schema, field, schema_name, package)?;
                if vbytes.is_empty() {
                    // A ZERO-LENGTH offset-table entry is how the native
                    // writers spell an UNWRITTEN variable-nested field (the
                    // `set_header_bytes(&[])` idiom; the frame walker reads
                    // it as present-with-zero-fields). Encode it as the
                    // nested schema's ZERO-DEFAULT fields — deterministic,
                    // and a legitimate native frame is never rejected
                    // (a confirmed defect class). A zero-filled
                    // `[fixed][table]` buffer IS that value: every fixed
                    // field zero, every offset entry (0,0) → empty
                    // strings/arrays (recursively, for nested-in-nested).
                    let nlayout = self
                        .layouts
                        .get(&nq)
                        .ok_or_else(|| CdrCodecError::UnknownSchema(nq.clone()))?;
                    let zero = vec![0u8; nlayout.fixed_size + nlayout.offset_table_bytes()];
                    return self.encode_payload(&nq, &zero, writer);
                }
                // vbytes is a headerless sub-frame [fixed][table][var].
                self.encode_payload(&nq, vbytes, writer)
            }
            _ => Err(CdrCodecError::BadWirePayload {
                schema: schema.to_string(),
                detail: format!("fixed field '{field}' reached the variable encoder"),
            }),
        }
    }

    /// Emit CDR for a sequence/array field from its Cerulion variable-payload
    /// bytes. `fixed_count` is `Some(N)` for a fixed array (no CDR count
    /// prefix — write exactly N) or `None` for a dynamic array (emit the count
    /// the Cerulion bytes imply).
    fn encode_cdr_seq(
        &self,
        schema: &str,
        field: &str,
        element_type: &FieldType,
        fixed_count: Option<usize>,
        vbytes: &[u8],
        writer: &mut CdrWriter,
    ) -> Result<(), CdrCodecError> {
        match element_type {
            FieldType::Bool | FieldType::I8 | FieldType::U8 => {
                if fixed_count.is_none() {
                    writer.write_u32(vbytes.len() as u32);
                }
                writer.buf.extend_from_slice(vbytes);
                Ok(())
            }
            FieldType::I16
            | FieldType::U16
            | FieldType::I32
            | FieldType::U32
            | FieldType::I64
            | FieldType::U64
            | FieldType::F32
            | FieldType::F64 => {
                let size = element_type.fixed_size().expect("primitive");
                if !vbytes.len().is_multiple_of(size) {
                    return Err(CdrCodecError::BadWirePayload {
                        schema: schema.to_string(),
                        detail: format!(
                            "array '{field}' payload {} not a multiple of element size {size}",
                            vbytes.len()
                        ),
                    });
                }
                let count = vbytes.len() / size;
                if fixed_count.is_none() {
                    writer.write_u32(count as u32);
                }
                for chunk in vbytes.chunks(size) {
                    writer.write_prim_le(chunk);
                }
                Ok(())
            }
            FieldType::String => {
                // Cerulion: u32 count + per element [u32 len][UTF-8].
                let mut cur = ComplexReader::new(vbytes, schema, field);
                let count = cur.read_u32()? as usize;
                if fixed_count.is_none() {
                    writer.write_u32(count as u32);
                }
                for _ in 0..count {
                    let len = cur.read_u32()? as usize;
                    let content = cur.take(len)?;
                    std::str::from_utf8(content).map_err(|_| CdrCodecError::InvalidUtf8 {
                        schema: schema.to_string(),
                        field: field.to_string(),
                    })?;
                    writer.write_string(content);
                }
                Ok(())
            }
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                let nq = self.resolve_nested_qname(schema, field, schema_name, package)?;
                let nlayout = self
                    .layouts
                    .get(&nq)
                    .ok_or_else(|| CdrCodecError::UnknownSchema(nq.clone()))?;
                if nlayout.is_fixed() {
                    // Back-to-back fixed sections; count = len / stride.
                    let stride = nlayout.fixed_size;
                    if stride == 0 || !vbytes.len().is_multiple_of(stride) {
                        return Err(CdrCodecError::BadWirePayload {
                            schema: schema.to_string(),
                            detail: format!(
                                "fixed-nested array '{field}' payload {} not a multiple of stride {stride}",
                                vbytes.len()
                            ),
                        });
                    }
                    let count = vbytes.len() / stride;
                    if fixed_count.is_none() {
                        writer.write_u32(count as u32);
                    }
                    for chunk in vbytes.chunks(stride) {
                        self.encode_payload(&nq, chunk, writer)?;
                    }
                    Ok(())
                } else {
                    // u32 count + per element [u32 len][sub-frame].
                    let mut cur = ComplexReader::new(vbytes, schema, field);
                    let count = cur.read_u32()? as usize;
                    if fixed_count.is_none() {
                        writer.write_u32(count as u32);
                    }
                    for _ in 0..count {
                        let len = cur.read_u32()? as usize;
                        let sub = cur.take(len)?;
                        self.encode_payload(&nq, sub, writer)?;
                    }
                    Ok(())
                }
            }
            other => Err(CdrCodecError::Unsupported {
                schema: schema.to_string(),
                field: field.to_string(),
                detail: format!("array of {} is not supported", other.canonical_str()),
            }),
        }
    }

    /// Resolve a nested reference to its qualified name using the SAME
    /// precedence as [`LayoutResolver`] / [`FrameWalker`](crate::codegen::FrameWalker): qualified →
    /// same-package → bare `Header`→`std_msgs/Header` → unambiguous bare
    /// suffix.
    fn resolve_nested_qname(
        &self,
        parent_qname: &str,
        field: &str,
        schema_name: &str,
        package: &Option<String>,
    ) -> Result<String, CdrCodecError> {
        let target = match package {
            Some(pkg) => format!("{pkg}/{schema_name}"),
            None => schema_name.to_string(),
        };
        self.lookup_nested_qname(schema_name, package, parent_qname)
            .ok_or_else(|| CdrCodecError::NestedResolutionFailed {
                schema: parent_qname.to_string(),
                field: field.to_string(),
                target,
            })
    }

    fn lookup_nested_qname(
        &self,
        schema_name: &str,
        package: &Option<String>,
        parent_qname: &str,
    ) -> Option<String> {
        // Delegate to the SINGLE shared ladder (see
        // [`resolve_nested_qname_ladder`]) over this codec's loaded layouts, so
        // the decoder's nested resolution can never drift from the
        // `cerulion ros2 attach` acquisition-completeness walk that must predict
        // it.
        resolve_nested_qname_ladder(
            schema_name,
            package.as_deref(),
            parent_qname,
            |k| self.layouts.contains_key(k),
            self.layouts.keys().map(String::as_str),
        )
    }
}

/// Cerulion's canonical nested-`.msg`-reference resolution ladder — the SINGLE
/// source of truth shared by the CDR codec's private `lookup_nested_qname`
/// (which delegates here) and the `cerulion ros2 attach` acquisition-completeness
/// walk (`cerulion_cli_engine::ros_cmd::nested_dependencies`), so the decoder's
/// resolution and the walk that must PREDICT it can never silently drift (a
/// drift-guard test feeds cases through here and asserts these documented
/// outcomes).
///
/// Resolves a nested reference — a bare `schema_name` with an optional source
/// `package`, appearing inside the message `parent_qname` — to its qualified
/// `pkg/Type` name against a RESOLVABLE UNIVERSE, in this precedence:
///
/// 1. **qualified** (`package = Some(pkg)`) → `pkg/schema_name`;
/// 2. **same-package** → `<parent_pkg>/schema_name` (or the bare `schema_name`
///    when `parent_qname` is itself unqualified — a flat-namespace schema);
/// 3. **bare `Header`** → `std_msgs/Header`;
/// 4. **unambiguous bare suffix** — exactly ONE universe member whose bare type
///    (the part after the final `/`) equals `schema_name`; two or more is
///    ambiguous and resolves to `None`.
///
/// `contains` tests membership in the universe; `keys` enumerates it (consumed
/// ONLY by rung 4). Returns `None` when no rung resolves — the caller decides
/// whether that is a hard decode error (the codec) or a missing dependency (the
/// completeness walk).
pub fn resolve_nested_qname_ladder<'a>(
    schema_name: &str,
    package: Option<&str>,
    parent_qname: &str,
    contains: impl Fn(&str) -> bool,
    keys: impl Iterator<Item = &'a str>,
) -> Option<String> {
    if let Some(pkg) = package {
        let key = format!("{pkg}/{schema_name}");
        return contains(&key).then_some(key);
    }
    match parent_qname.rsplit_once('/') {
        Some((parent_pkg, _)) => {
            let key = format!("{parent_pkg}/{schema_name}");
            if contains(&key) {
                return Some(key);
            }
        }
        None => {
            if contains(schema_name) {
                return Some(schema_name.to_string());
            }
        }
    }
    if schema_name == "Header" {
        let key = "std_msgs/Header";
        if contains(key) {
            return Some(key.to_string());
        }
    }
    let mut it = keys.filter(|k| k.rsplit('/').next() == Some(schema_name));
    match (it.next(), it.next()) {
        (Some(k), None) => Some(k.to_string()),
        _ => None,
    }
}

/// Reject a length/count prefix that cannot fit in the remaining body (each
/// CDR element is ≥ 1 byte, so a valid `count` never exceeds `available`).
/// The DoS guard: a hostile `0xFFFFFFFF` prefix fails HERE, before any
/// allocation or element loop is sized from it.
fn guard_count(
    schema: &str,
    field: &str,
    count: usize,
    available: usize,
) -> Result<(), CdrCodecError> {
    if count > available {
        Err(CdrCodecError::HostileLength {
            schema: schema.to_string(),
            field: field.to_string(),
            claimed: count as u64,
            available,
        })
    } else {
        Ok(())
    }
}

/// A tiny bounds-checked cursor over a Cerulion complex-field payload
/// (`u32 count` + length-delimited entries) used by the encode side to walk
/// the string-array / variable-nested-array encodings.
struct ComplexReader<'a> {
    bytes: &'a [u8],
    pos: usize,
    schema: &'a str,
    field: &'a str,
}

impl<'a> ComplexReader<'a> {
    fn new(bytes: &'a [u8], schema: &'a str, field: &'a str) -> Self {
        Self {
            bytes,
            pos: 0,
            schema,
            field,
        }
    }

    fn read_u32(&mut self) -> Result<u32, CdrCodecError> {
        let slice = self.take(4)?;
        Ok(u32::from_le_bytes([slice[0], slice[1], slice[2], slice[3]]))
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], CdrCodecError> {
        let end = self.pos.checked_add(n);
        match end {
            Some(end) if end <= self.bytes.len() => {
                let slice = &self.bytes[self.pos..end];
                self.pos = end;
                Ok(slice)
            }
            _ => Err(CdrCodecError::BadWirePayload {
                schema: self.schema.to_string(),
                detail: format!(
                    "complex field '{}' truncated: need {n} bytes at {}, have {}",
                    self.field,
                    self.pos,
                    self.bytes.len().saturating_sub(self.pos)
                ),
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::codegen::schema::{FieldDef, FieldType, MessageSchema};
    use crate::codegen::{FrameValueKind, FrameWalker};

    // ---- schema fixtures --------------------------------------------------

    fn f64_field(n: &str) -> FieldDef {
        FieldDef::new(n, FieldType::F64)
    }
    fn nested(name: &str, pkg: &str) -> FieldType {
        FieldType::Nested {
            schema_name: name.to_string(),
            package: Some(pkg.to_string()),
            fixed: None,
        }
    }

    fn vec3() -> MessageSchema {
        let mut s = MessageSchema::new_in_package("Vector3", "geometry_msgs");
        for n in ["x", "y", "z"] {
            s.add_field(f64_field(n));
        }
        s
    }
    fn twist() -> MessageSchema {
        let mut s = MessageSchema::new_in_package("Twist", "geometry_msgs");
        s.add_field(FieldDef::new("linear", nested("Vector3", "geometry_msgs")));
        s.add_field(FieldDef::new("angular", nested("Vector3", "geometry_msgs")));
        s
    }
    fn time() -> MessageSchema {
        let mut s = MessageSchema::new_in_package("Time", "builtin_interfaces");
        s.add_field(FieldDef::new("sec", FieldType::I32));
        s.add_field(FieldDef::new("nanosec", FieldType::U32));
        s
    }
    fn header() -> MessageSchema {
        let mut s = MessageSchema::new_in_package("Header", "std_msgs");
        s.add_field(FieldDef::new("stamp", nested("Time", "builtin_interfaces")));
        s.add_field(FieldDef::new("frame_id", FieldType::String));
        s
    }

    fn codec(schemas: Vec<MessageSchema>) -> CdrCodec {
        let (c, warns) = CdrCodec::new(schemas);
        assert!(warns.is_empty(), "resolution warnings: {warns:?}");
        c
    }

    /// A Cerulion frame whose 32-byte header is a placeholder (all zeros) —
    /// [`CdrCodec::encode`] strips and ignores the header, so this is a
    /// sufficient encode input built purely from a hand payload.
    fn headerless_input(payload: &[u8]) -> Vec<u8> {
        let mut f = vec![0u8; WireHeader::SIZE];
        f.extend_from_slice(payload);
        f
    }

    // ---- decode: fixed-only + endianness ---------------------------------

    #[test]
    fn test_decode_fixed_only_matches_le_oracle() {
        let c = codec(vec![vec3()]);
        // CDR body: x=1.0, y=2.0, z=3.0 (3 f64 LE, no padding).
        let mut body = Vec::new();
        for v in [1.0f64, 2.0, 3.0] {
            body.extend_from_slice(&v.to_le_bytes());
        }
        let frame = c
            .decode("geometry_msgs/Vector3", CdrEndianness::Little, &body, 7, 99)
            .expect("decode");
        // Oracle: fixed section == the three LE f64, payload == fixed (no vars).
        assert_eq!(&frame[WireHeader::SIZE..], &body[..]);
        // Header is well-formed.
        let hdr = WireHeader::read_from_buf(&frame).expect("hdr");
        assert_eq!(
            hdr.schema_hash,
            c.schema_hash("geometry_msgs/Vector3").unwrap()
        );
        assert_eq!(hdr.total_size as usize, frame.len());
        assert_eq!(hdr.offset_table_count, 0);
        assert_eq!(hdr.sequence, 7);
        assert_eq!(hdr.timestamp_ns, 99);
    }

    #[test]
    fn test_decode_big_endian_yields_identical_cerulion_frame() {
        let c = codec(vec![vec3()]);
        let mut le = Vec::new();
        let mut be = Vec::new();
        for v in [1.5f64, -2.5, 3.25] {
            le.extend_from_slice(&v.to_le_bytes());
            be.extend_from_slice(&v.to_be_bytes());
        }
        // Anti-tautology: the two CDR bodies DIFFER byte-for-byte...
        assert_ne!(le, be);
        let lf = c
            .decode("geometry_msgs/Vector3", CdrEndianness::Little, &le, 0, 0)
            .unwrap();
        let bf = c
            .decode("geometry_msgs/Vector3", CdrEndianness::Big, &be, 0, 0)
            .unwrap();
        // ...yet decode to the SAME Cerulion frame (LE fixed section).
        assert_eq!(lf, bf);
        assert_eq!(&lf[WireHeader::SIZE..], &le[..]);
    }

    #[test]
    fn test_decode_nested_fixed_walks_correctly() {
        let c = codec(vec![vec3(), twist()]);
        // Twist = linear{1,2,3} + angular{4,5,6}, all inline (no reset).
        let mut body = Vec::new();
        for v in [1.0f64, 2.0, 3.0, 4.0, 5.0, 6.0] {
            body.extend_from_slice(&v.to_le_bytes());
        }
        let frame = c
            .decode("geometry_msgs/Twist", CdrEndianness::Little, &body, 0, 0)
            .unwrap();
        assert_eq!(&frame[WireHeader::SIZE..], &body[..]); // 48-byte fixed section
                                                           // Cross-check via the frame walker (independent decoder).
        let (walker, _) = FrameWalker::new(vec![vec3(), twist()]);
        let fv = walker.walk("geometry_msgs/Twist", &frame).unwrap();
        match fv.field("linear") {
            Some(FrameValueKind::Nested(inner)) => {
                assert_eq!(inner.field("x"), Some(&FrameValueKind::F64(1.0)));
                assert_eq!(inner.field("z"), Some(&FrameValueKind::F64(3.0)));
            }
            other => panic!("expected Nested, got {other:?}"),
        }
        match fv.field("angular") {
            Some(FrameValueKind::Nested(inner)) => {
                assert_eq!(inner.field("y"), Some(&FrameValueKind::F64(5.0)));
            }
            other => panic!("expected Nested, got {other:?}"),
        }
    }

    // ---- decode: variable string + bytes with a hand oracle --------------

    fn imglike() -> MessageSchema {
        let mut s = MessageSchema::new_in_package("Imglike", "test_msgs");
        s.add_field(FieldDef::new("height", FieldType::U32));
        s.add_field(FieldDef::new("width", FieldType::U32));
        s.add_field(FieldDef::new("encoding", FieldType::String));
        s.add_field(FieldDef::new(
            "data",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::U8),
            },
        ));
        s
    }

    #[test]
    fn test_decode_string_and_bytes_matches_hand_oracle() {
        let c = codec(vec![imglike()]);
        // CDR body (LE): height=480, width=640, encoding="rgb8", data=[1,2,3].
        let mut body = Vec::new();
        body.extend_from_slice(&480u32.to_le_bytes());
        body.extend_from_slice(&640u32.to_le_bytes());
        body.extend_from_slice(&5u32.to_le_bytes()); // len incl NUL
        body.extend_from_slice(b"rgb8\0");
        // data u8[]: u32 count aligns to 4 — the "rgb8\0" ended at body offset
        // 17, so 3 pad bytes then the count.
        body.extend_from_slice(&[0, 0, 0]);
        body.extend_from_slice(&3u32.to_le_bytes());
        body.extend_from_slice(&[1, 2, 3]);

        // Hand oracle Cerulion payload: fixed[height,width]=8, table(16),
        // encoding@24 len 4 "rgb8", data@28 len 3.
        let mut payload = Vec::new();
        payload.extend_from_slice(&480u32.to_le_bytes());
        payload.extend_from_slice(&640u32.to_le_bytes());
        payload.extend_from_slice(&24u32.to_le_bytes()); // encoding offset
        payload.extend_from_slice(&4u32.to_le_bytes()); // encoding len
        payload.extend_from_slice(&28u32.to_le_bytes()); // data offset
        payload.extend_from_slice(&3u32.to_le_bytes()); // data len
        payload.extend_from_slice(b"rgb8");
        payload.extend_from_slice(&[1, 2, 3]);

        let frame = c
            .decode("test_msgs/Imglike", CdrEndianness::Little, &body, 7, 99)
            .unwrap();
        assert_eq!(&frame[WireHeader::SIZE..], &payload[..]);
        // Header fields are stamped from the layout. offset_table_offset is
        // FRAME-relative (32-byte header + the 8-byte fixed section = 40) —
        // the wire.rs "byte offset from message start" contract the native
        // publisher stamps.
        let hdr = WireHeader::read_from_buf(&frame).unwrap();
        assert_eq!(hdr.offset_table_offset, 40);
        assert_eq!(hdr.offset_table_count, 2);

        // Independent cross-check via the walker.
        let (walker, _) = FrameWalker::new(vec![imglike()]);
        let fv = walker.walk("test_msgs/Imglike", &frame).unwrap();
        assert_eq!(fv.field("height"), Some(&FrameValueKind::U32(480)));
        assert_eq!(fv.field("encoding"), Some(&FrameValueKind::Str("rgb8")));
        assert_eq!(fv.field("data"), Some(&FrameValueKind::Bytes(&[1, 2, 3])));
    }

    #[test]
    fn test_decode_padding_before_f64_matches_repr_c() {
        let mut s = MessageSchema::new_in_package("Pad", "test_msgs");
        s.add_field(FieldDef::new("flag", FieldType::Bool));
        s.add_field(FieldDef::new("val", FieldType::F64));
        let c = codec(vec![s]);
        // CDR: flag@0 (1 byte), pad to 8, val@8.
        let mut body = vec![1u8];
        body.extend_from_slice(&[0u8; 7]); // CDR alignment pad
        body.extend_from_slice(&2.0f64.to_le_bytes());
        let frame = c
            .decode("test_msgs/Pad", CdrEndianness::Little, &body, 0, 0)
            .unwrap();
        // Cerulion fixed: flag@0, pad 1..8 (zeroed), val@8. size 16.
        let mut expect = vec![1u8, 0, 0, 0, 0, 0, 0, 0];
        expect.extend_from_slice(&2.0f64.to_le_bytes());
        assert_eq!(&frame[WireHeader::SIZE..], &expect[..]);
    }

    #[test]
    fn test_decode_fixed_field_after_string_is_reordered_to_fixed_section() {
        // Declaration order: string s THEN f64 v. In Cerulion, v (fixed) comes
        // FIRST (fixed section), s (variable) is in the offset region.
        let mut s = MessageSchema::new_in_package("SthenV", "test_msgs");
        s.add_field(FieldDef::new("s", FieldType::String));
        s.add_field(FieldDef::new("v", FieldType::F64));
        let c = codec(vec![s]);
        // CDR: s="hi" (u32 3, "hi\0"), then v aligns to 8 (pad), v=9.0.
        let mut body = Vec::new();
        body.extend_from_slice(&3u32.to_le_bytes());
        body.extend_from_slice(b"hi\0"); // ends at offset 7
        body.push(0); // pad to 8
        body.extend_from_slice(&9.0f64.to_le_bytes());
        let frame = c
            .decode("test_msgs/SthenV", CdrEndianness::Little, &body, 0, 0)
            .unwrap();
        // Cerulion: fixed[v]=8, table(8), s@16 len 2. payload len 8+8+2=18.
        let mut payload = Vec::new();
        payload.extend_from_slice(&9.0f64.to_le_bytes());
        payload.extend_from_slice(&16u32.to_le_bytes());
        payload.extend_from_slice(&2u32.to_le_bytes());
        payload.extend_from_slice(b"hi");
        assert_eq!(&frame[WireHeader::SIZE..], &payload[..]);
    }

    // ---- decode: variable nested (Header) --------------------------------

    #[test]
    fn test_decode_variable_nested_header_subframe() {
        let mut stamped = MessageSchema::new_in_package("Stamped", "test_msgs");
        stamped.add_field(FieldDef::new("header", nested("Header", "std_msgs")));
        stamped.add_field(FieldDef::new("value", FieldType::U32));
        let set = vec![time(), header(), stamped];
        let c = codec(set.clone());
        // CDR body: Header{ Time{sec=12,nanosec=34}, frame_id="lidar" }, value=7.
        // Declaration order: header (inline: sec,nanosec,frame_id-string) then
        // value (u32, aligns to 4).
        let mut body = Vec::new();
        body.extend_from_slice(&12i32.to_le_bytes());
        body.extend_from_slice(&34u32.to_le_bytes());
        body.extend_from_slice(&6u32.to_le_bytes()); // "lidar" + NUL
        body.extend_from_slice(b"lidar\0"); // ends at offset 8+4+6=18
        body.extend_from_slice(&[0, 0]); // pad to 20 for value u32
        body.extend_from_slice(&7u32.to_le_bytes());

        let frame = c
            .decode("test_msgs/Stamped", CdrEndianness::Little, &body, 0, 0)
            .unwrap();

        // Independent cross-check: the walker decodes the nested sub-frame.
        let (walker, _) = FrameWalker::new(set);
        let fv = walker.walk("test_msgs/Stamped", &frame).unwrap();
        assert_eq!(fv.field("value"), Some(&FrameValueKind::U32(7)));
        match fv.field("header") {
            Some(FrameValueKind::Nested(h)) => {
                assert_eq!(h.field("frame_id"), Some(&FrameValueKind::Str("lidar")));
                match h.field("stamp") {
                    Some(FrameValueKind::Nested(t)) => {
                        assert_eq!(t.field("sec"), Some(&FrameValueKind::I32(12)));
                        assert_eq!(t.field("nanosec"), Some(&FrameValueKind::U32(34)));
                    }
                    other => panic!("expected Time nested, got {other:?}"),
                }
            }
            other => panic!("expected Header nested, got {other:?}"),
        }
    }

    // ---- decode: primitive dynamic array ---------------------------------

    #[test]
    fn test_decode_dynamic_f64_array_aligns_and_matches() {
        let mut s = MessageSchema::new_in_package("Vals", "test_msgs");
        s.add_field(FieldDef::new(
            "vals",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::F64),
            },
        ));
        let c = codec(vec![s.clone()]);
        // CDR: u32 count=3, pad to 8, then 3 f64.
        let vals = [1.5f64, -2.5, 3.25];
        let mut body = Vec::new();
        body.extend_from_slice(&3u32.to_le_bytes());
        body.extend_from_slice(&[0, 0, 0, 0]); // pad count(4) → 8
        for v in vals {
            body.extend_from_slice(&v.to_le_bytes());
        }
        let frame = c
            .decode("test_msgs/Vals", CdrEndianness::Little, &body, 0, 0)
            .unwrap();
        // Cerulion: all-variable, fixed 0, table(8). vals payload aligns to 8:
        // table ends at 8 (already 8-aligned), data @8, 24 bytes.
        let (walker, _) = FrameWalker::new(vec![s]);
        let fv = walker.walk("test_msgs/Vals", &frame).unwrap();
        match fv.field("vals") {
            Some(FrameValueKind::PrimArray(pa)) => {
                let got: Vec<f64> = pa.iter_f64().collect();
                assert_eq!(got, vec![1.5, -2.5, 3.25]);
            }
            other => panic!("expected PrimArray, got {other:?}"),
        }
        // Empty sequence → present, zero elements (a zero-count PrimArray).
        let mut empty = Vec::new();
        empty.extend_from_slice(&0u32.to_le_bytes());
        let ef = c
            .decode("test_msgs/Vals", CdrEndianness::Little, &empty, 0, 0)
            .unwrap();
        let fv = walker.walk("test_msgs/Vals", &ef).unwrap();
        match fv.field("vals") {
            Some(FrameValueKind::PrimArray(pa)) => assert_eq!(pa.count, 0),
            other => panic!("expected empty PrimArray, got {other:?}"),
        }
    }

    // ---- decode: a "new" type the hand registry never had (JointState) ---

    fn jointstate_set() -> Vec<MessageSchema> {
        let mut js = MessageSchema::new_in_package("JointState", "sensor_msgs");
        js.add_field(FieldDef::new("header", nested("Header", "std_msgs")));
        js.add_field(FieldDef::new(
            "name",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::String),
            },
        ));
        for f in ["position", "velocity", "effort"] {
            js.add_field(FieldDef::new(
                f,
                FieldType::DynamicArray {
                    element_type: Box::new(FieldType::F64),
                },
            ));
        }
        vec![time(), header(), js]
    }

    #[test]
    fn test_decode_new_type_jointstate_to_hand_oracle() {
        let set = jointstate_set();
        let c = codec(set.clone());
        // Hand-built CDR: header{Time{0,0}, frame_id=""}, name=["a","bb"],
        // position=[1.0,2.0], velocity=[], effort=[].
        let mut body = Vec::new();
        // header.stamp
        body.extend_from_slice(&0i32.to_le_bytes());
        body.extend_from_slice(&0u32.to_le_bytes());
        // header.frame_id = "" → len 1, NUL
        body.extend_from_slice(&1u32.to_le_bytes());
        body.push(0); // NUL — body offset now 13
                      // name: u32 count=2 (aligns to 4 → pad 3 bytes at 13..16)
        body.extend_from_slice(&[0, 0, 0]);
        body.extend_from_slice(&2u32.to_le_bytes());
        // "a" → len 2, "a\0"
        body.extend_from_slice(&2u32.to_le_bytes());
        body.extend_from_slice(b"a\0");
        // "bb" → len 3, "bb\0" (u32 aligns to 4: prev ended at 24+... compute
        // lazily by padding to 4)
        while body.len() % 4 != 0 {
            body.push(0);
        }
        body.extend_from_slice(&3u32.to_le_bytes());
        body.extend_from_slice(b"bb\0");
        // position: u32 count=2, pad to 8, then 2 f64
        while body.len() % 4 != 0 {
            body.push(0);
        }
        body.extend_from_slice(&2u32.to_le_bytes());
        while body.len() % 8 != 0 {
            body.push(0);
        }
        body.extend_from_slice(&1.0f64.to_le_bytes());
        body.extend_from_slice(&2.0f64.to_le_bytes());
        // velocity: count 0
        body.extend_from_slice(&0u32.to_le_bytes());
        // effort: count 0
        body.extend_from_slice(&0u32.to_le_bytes());

        let frame = c
            .decode("sensor_msgs/JointState", CdrEndianness::Little, &body, 0, 0)
            .unwrap();

        // LITERAL BYTE ORACLE for the Cerulion-side `string[]` framing (restored
        // on purpose: the walker assertion below is the production
        // loop closure, but on its own a COORDINATED change to both the codec
        // writer and the walker reader would pass, so the bytes themselves are
        // pinned too). `name` is variable field index 1 of `JointState`
        // (`header`, `name`, …) with `fixed_size == 0`, so its offset entry is
        // at payload[8..16].
        {
            let payload = &frame[WireHeader::SIZE..];
            let (off, len) = crate::shm_runtime::read_offset_entry(payload, 0, 1);
            let name_bytes = &payload[off as usize..(off + len) as usize];
            assert_eq!(&name_bytes[0..4], &2u32.to_le_bytes(), "u32 count = 2");
            assert_eq!(&name_bytes[4..8], &1u32.to_le_bytes(), "element 0 len = 1");
            assert_eq!(&name_bytes[8..9], b"a", "element 0 content (no NUL)");
            assert_eq!(&name_bytes[9..13], &2u32.to_le_bytes(), "element 1 len = 2");
            assert_eq!(&name_bytes[13..15], b"bb", "element 1 content");
            assert_eq!(name_bytes.len(), 15, "exactly consumed, no slack");
        }

        let (walker, _) = FrameWalker::new(set);
        let fv = walker.walk("sensor_msgs/JointState", &frame).unwrap();
        // `name` is a `DynamicArray<String>`: this codec writes it in the
        // canonical `u32 count` + per element `u32 len` + UTF-8 form, which
        // the walker DECODES. So the assertion is now the
        // real production loop closure — codec-produced bytes read back as the
        // SAME strings the hand-built CDR body carried, with no hand byte
        // arithmetic in between (a genuine cross-check, not a self-compare:
        // the oracle is the CDR body above).
        match fv.field("name") {
            Some(FrameValueKind::NestedArray {
                elements: elems, ..
            }) => {
                assert_eq!(
                    elems,
                    &vec![FrameValueKind::Str("a"), FrameValueKind::Str("bb")],
                    "codec-written string[] decodes to the CDR body's strings"
                );
            }
            other => panic!("expected a decoded name array, got {other:?}"),
        }
        match fv.field("position") {
            Some(FrameValueKind::PrimArray(pa)) => {
                assert_eq!(pa.iter_f64().collect::<Vec<_>>(), vec![1.0, 2.0]);
            }
            other => panic!("expected position PrimArray, got {other:?}"),
        }
        // Round-trip: encode back to CDR == the original body (trailing pad
        // aside — none here since the last field is a count with no elements).
        let re = c
            .encode("sensor_msgs/JointState", CdrEndianness::Little, &frame)
            .unwrap();
        assert_eq!(re, body, "encode(decode(cdr)) == cdr");
    }

    // ---- decode: dynamic array of variable nested (PointField-like) ------

    fn pointfield_set() -> Vec<MessageSchema> {
        let mut pf = MessageSchema::new_in_package("PointField", "sensor_msgs");
        pf.add_field(FieldDef::new("name", FieldType::String));
        pf.add_field(FieldDef::new("offset", FieldType::U32));
        pf.add_field(FieldDef::new("datatype", FieldType::U8));
        pf.add_field(FieldDef::new("count", FieldType::U32));
        let mut holder = MessageSchema::new_in_package("Fields", "sensor_msgs");
        holder.add_field(FieldDef::new(
            "fields",
            FieldType::DynamicArray {
                element_type: Box::new(nested("PointField", "sensor_msgs")),
            },
        ));
        vec![pf, holder]
    }

    #[test]
    fn test_decode_variable_nested_array_round_trips() {
        let set = pointfield_set();
        let c = codec(set);
        // CDR: fields = [ {name="x", offset=0, datatype=7, count=1} ].
        let mut body = Vec::new();
        body.extend_from_slice(&1u32.to_le_bytes()); // count 1
                                                     // element PointField inline: name string "x"(len2), offset u32 (aligns
                                                     // 4), datatype u8, count u32 (aligns 4).
        body.extend_from_slice(&2u32.to_le_bytes());
        body.extend_from_slice(b"x\0"); // ends at 4+4+2=10
        while body.len() % 4 != 0 {
            body.push(0);
        }
        body.extend_from_slice(&0u32.to_le_bytes()); // offset
        body.push(7u8); // datatype
        while body.len() % 4 != 0 {
            body.push(0);
        }
        body.extend_from_slice(&1u32.to_le_bytes()); // count

        let frame = c
            .decode("sensor_msgs/Fields", CdrEndianness::Little, &body, 0, 0)
            .unwrap();
        let re = c
            .encode("sensor_msgs/Fields", CdrEndianness::Little, &frame)
            .unwrap();
        assert_eq!(re, body);
        // And a second decode is byte-identical (determinism).
        let frame2 = c
            .decode("sensor_msgs/Fields", CdrEndianness::Little, &body, 0, 0)
            .unwrap();
        assert_eq!(frame, frame2);

        // The production loop closure for the
        // VARIABLE-ELEMENT row (`PointField` carries a `string name`, so the
        // codec writes `u32 count` + per element `u32 len` + headerless
        // sub-frame). Previously this test's only oracle was the codec against
        // ITSELF, with a comment claiming "the array is opaque to the walker" —
        // no longer true, and the two sides resolve the element layout through
        // INDEPENDENT resolvers (`resolve_nested_qname` here vs the walker's
        // `lookup_nested`), so their agreement needs an assertion, not
        // inspection. The oracle is the hand-built CDR body above.
        let (walker, _) = FrameWalker::new(pointfield_set());
        let fv = walker.walk("sensor_msgs/Fields", &frame).unwrap();
        match fv.field("fields") {
            Some(FrameValueKind::NestedArray {
                elements: elems, ..
            }) => {
                assert_eq!(elems.len(), 1, "one PointField element");
                match &elems[0] {
                    FrameValueKind::Nested(pf) => {
                        assert_eq!(pf.schema_name, "sensor_msgs/PointField");
                        assert_eq!(pf.field("name"), Some(&FrameValueKind::Str("x")));
                        assert_eq!(pf.field("offset"), Some(&FrameValueKind::U32(0)));
                        assert_eq!(pf.field("datatype"), Some(&FrameValueKind::U8(7)));
                        assert_eq!(pf.field("count"), Some(&FrameValueKind::U32(1)));
                    }
                    other => panic!("expected a decoded PointField, got {other:?}"),
                }
            }
            other => panic!("expected a decoded fields array, got {other:?}"),
        }
    }

    /// The fixed-stride row's production loop closure — a
    /// `DynamicArray<Nested-fixed>` the codec writes as back-to-back fixed
    /// sections with NO count, read back by the walker element-by-element. The
    /// oracle is the hand-built CDR body.
    #[test]
    fn test_decode_fixed_nested_array_walker_cross_check() {
        let mut pt = MessageSchema::new_in_package("Point", "geometry_msgs");
        for n in ["x", "y", "z"] {
            pt.add_field(FieldDef::new(n, FieldType::F64));
        }
        let mut holder = MessageSchema::new_in_package("Cells", "test_msgs");
        holder.add_field(FieldDef::new(
            "cells",
            FieldType::DynamicArray {
                element_type: Box::new(nested("Point", "geometry_msgs")),
            },
        ));
        let set = vec![pt, holder];
        let c = codec(set.clone());

        // CDR: cells = [ (1.0, 2.0, 3.0), (-4.5, 5.25, 6.125) ].
        let oracle = [[1.0f64, 2.0, 3.0], [-4.5, 5.25, 6.125]];
        let mut body = 2u32.to_le_bytes().to_vec();
        while !body.len().is_multiple_of(8) {
            body.push(0);
        }
        for p in oracle {
            for v in p {
                body.extend_from_slice(&v.to_le_bytes());
            }
        }

        let frame = c
            .decode("test_msgs/Cells", CdrEndianness::Little, &body, 0, 0)
            .unwrap();
        let (walker, _) = FrameWalker::new(set);
        let fv = walker.walk("test_msgs/Cells", &frame).unwrap();
        match fv.field("cells") {
            Some(FrameValueKind::NestedArray {
                elements: elems, ..
            }) => {
                assert_eq!(elems.len(), 2);
                for (i, p) in oracle.iter().enumerate() {
                    match &elems[i] {
                        FrameValueKind::Nested(point) => {
                            assert_eq!(point.field("x"), Some(&FrameValueKind::F64(p[0])));
                            assert_eq!(point.field("y"), Some(&FrameValueKind::F64(p[1])));
                            assert_eq!(point.field("z"), Some(&FrameValueKind::F64(p[2])));
                        }
                        other => panic!("expected a decoded Point, got {other:?}"),
                    }
                }
            }
            other => panic!("expected a decoded cells array, got {other:?}"),
        }
        // And the codec's own inverse still holds on the same frame.
        let re = c
            .encode("test_msgs/Cells", CdrEndianness::Little, &frame)
            .unwrap();
        assert_eq!(re, body, "encode(decode(cdr)) == cdr");
    }

    // ---- decode: fixed array of primitives + fixed array of fixed nested --

    #[test]
    fn test_decode_fixed_arrays() {
        // Imu-cov-like: { float64[4] cov } fixed section.
        let mut s = MessageSchema::new_in_package("Cov", "test_msgs");
        s.add_field(FieldDef::new(
            "cov",
            FieldType::FixedArray {
                element_type: Box::new(FieldType::F64),
                length: 4,
            },
        ));
        let c = codec(vec![s]);
        let mut body = Vec::new();
        for v in [1.0f64, 2.0, 3.0, 4.0] {
            body.extend_from_slice(&v.to_le_bytes());
        }
        let frame = c
            .decode("test_msgs/Cov", CdrEndianness::Little, &body, 0, 0)
            .unwrap();
        assert_eq!(&frame[WireHeader::SIZE..], &body[..]);
        let re = c
            .encode("test_msgs/Cov", CdrEndianness::Little, &frame)
            .unwrap();
        assert_eq!(re, body);
    }

    #[test]
    fn test_decode_fixed_array_of_fixed_nested() {
        // Corners { Vector3[2] pts } — fixed array of a recursively-fixed
        // nested → all fixed section, stride 24.
        let mut s = MessageSchema::new_in_package("Corners", "test_msgs");
        s.add_field(FieldDef::new(
            "pts",
            FieldType::FixedArray {
                element_type: Box::new(nested("Vector3", "geometry_msgs")),
                length: 2,
            },
        ));
        let c = codec(vec![vec3(), s]);
        let mut body = Vec::new();
        for v in [1.0f64, 2.0, 3.0, 4.0, 5.0, 6.0] {
            body.extend_from_slice(&v.to_le_bytes());
        }
        let frame = c
            .decode("test_msgs/Corners", CdrEndianness::Little, &body, 0, 0)
            .unwrap();
        assert_eq!(&frame[WireHeader::SIZE..], &body[..]); // 48 bytes, 2×24
        let re = c
            .encode("test_msgs/Corners", CdrEndianness::Little, &frame)
            .unwrap();
        assert_eq!(re, body);
    }

    // ---- encode: inverse + round-trip anchored by hand oracles -----------

    #[test]
    fn test_encode_from_hand_cerulion_frame_matches_cdr_oracle() {
        let c = codec(vec![vec3(), twist()]);
        // Hand Cerulion frame for Twist (fixed 48 bytes: 6 f64).
        let mut payload = Vec::new();
        for v in [0.5f64, 0.0, 0.0, 0.0, 0.0, 0.25] {
            payload.extend_from_slice(&v.to_le_bytes());
        }
        let frame = headerless_input(&payload);
        let cdr = c
            .encode("geometry_msgs/Twist", CdrEndianness::Little, &frame)
            .unwrap();
        // Oracle: 6 f64 LE, no padding.
        assert_eq!(cdr, payload);
        // And decode(encode) is identity.
        let round = c
            .decode("geometry_msgs/Twist", CdrEndianness::Little, &cdr, 0, 0)
            .unwrap();
        assert_eq!(round[WireHeader::SIZE..], frame[WireHeader::SIZE..]);
    }

    #[test]
    fn test_encode_decode_identity_for_stamped() {
        let set = vec![time(), header(), {
            let mut s = MessageSchema::new_in_package("Stamped", "test_msgs");
            s.add_field(FieldDef::new("header", nested("Header", "std_msgs")));
            s.add_field(FieldDef::new("value", FieldType::U32));
            s
        }];
        let c = codec(set);
        let mut body = Vec::new();
        body.extend_from_slice(&5i32.to_le_bytes());
        body.extend_from_slice(&6u32.to_le_bytes());
        body.extend_from_slice(&4u32.to_le_bytes());
        body.extend_from_slice(b"map\0");
        // value u32 already 4-aligned here (8+4+4=16).
        body.extend_from_slice(&42u32.to_le_bytes());
        let frame = c
            .decode("test_msgs/Stamped", CdrEndianness::Little, &body, 0, 0)
            .unwrap();
        let re = c
            .encode("test_msgs/Stamped", CdrEndianness::Little, &frame)
            .unwrap();
        assert_eq!(re, body, "encode(decode(cdr)) == cdr");
    }

    #[test]
    fn test_encode_big_endian_round_trips() {
        let c = codec(vec![vec3()]);
        let mut body = Vec::new();
        for v in [1.0f64, 2.0, 3.0] {
            body.extend_from_slice(&v.to_be_bytes());
        }
        let frame = c
            .decode("geometry_msgs/Vector3", CdrEndianness::Big, &body, 0, 0)
            .unwrap();
        let re = c
            .encode("geometry_msgs/Vector3", CdrEndianness::Big, &frame)
            .unwrap();
        assert_eq!(re, body);
    }

    // ---- hostile input -----------------------------------------------------

    #[test]
    fn test_truncated_body_is_err_not_panic() {
        let c = codec(vec![vec3()]);
        // Only 3 bytes where 24 are needed.
        let r = c.decode(
            "geometry_msgs/Vector3",
            CdrEndianness::Little,
            &[1, 2, 3],
            0,
            0,
        );
        assert!(matches!(r, Err(CdrCodecError::Truncated { .. })), "{r:?}");
        // Header-only-ish: empty body.
        let r = c.decode("geometry_msgs/Vector3", CdrEndianness::Little, &[], 0, 0);
        assert!(matches!(r, Err(CdrCodecError::Truncated { .. })), "{r:?}");
    }

    #[test]
    fn test_hostile_length_prefix_is_err_no_ooom() {
        let c = codec(vec![imglike()]);
        // height, width, then a string len 0xFFFFFFFF over 3 bytes.
        let mut body = Vec::new();
        body.extend_from_slice(&0u32.to_le_bytes());
        body.extend_from_slice(&0u32.to_le_bytes());
        body.extend_from_slice(&0xFFFF_FFFFu32.to_le_bytes());
        body.extend_from_slice(&[0x41, 0x42, 0x43]); // 3 bytes, not 4 GiB
        assert!(body.len() < 50, "guard buffer stays tiny");
        let r = c.decode("test_msgs/Imglike", CdrEndianness::Little, &body, 0, 0);
        assert!(
            matches!(r, Err(CdrCodecError::HostileLength { .. })),
            "{r:?}"
        );
    }

    #[test]
    fn test_hostile_sequence_count_is_err() {
        let mut s = MessageSchema::new_in_package("Vals", "test_msgs");
        s.add_field(FieldDef::new(
            "vals",
            FieldType::DynamicArray {
                element_type: Box::new(FieldType::F64),
            },
        ));
        let c = codec(vec![s]);
        let mut body = Vec::new();
        body.extend_from_slice(&0xFFFF_FFFFu32.to_le_bytes()); // huge count
        body.extend_from_slice(&[0u8; 8]); // one f64 of room
        let r = c.decode("test_msgs/Vals", CdrEndianness::Little, &body, 0, 0);
        assert!(
            matches!(r, Err(CdrCodecError::HostileLength { .. })),
            "{r:?}"
        );
    }

    #[test]
    fn test_non_utf8_string_is_err() {
        let c = codec(vec![imglike()]);
        let mut body = Vec::new();
        body.extend_from_slice(&0u32.to_le_bytes()); // height
        body.extend_from_slice(&0u32.to_le_bytes()); // width
        body.extend_from_slice(&2u32.to_le_bytes()); // encoding len 2 (1 char + NUL)
        body.extend_from_slice(&[0xFF, 0x00]); // 0xFF invalid UTF-8, then NUL
        body.extend_from_slice(&0u32.to_le_bytes()); // data count 0
        let r = c.decode("test_msgs/Imglike", CdrEndianness::Little, &body, 0, 0);
        assert!(matches!(r, Err(CdrCodecError::InvalidUtf8 { .. })), "{r:?}");
    }

    #[test]
    fn test_string_missing_nul_terminator_is_err() {
        // A string body `len=3|"ab"|0x41` must not silently yield
        // "ab" (corrupting the last char + breaking encode(decode)==cdr).
        // Loud Err carrying schema + field + the offending byte.
        let c = codec(vec![imglike()]);
        let mut body = Vec::new();
        body.extend_from_slice(&0u32.to_le_bytes()); // height
        body.extend_from_slice(&0u32.to_le_bytes()); // width
        body.extend_from_slice(&3u32.to_le_bytes()); // encoding len 3
        body.extend_from_slice(b"abA"); // final byte 0x41 — NOT the NUL
        body.push(0); // pad 15 → 16
        body.extend_from_slice(&0u32.to_le_bytes()); // data count 0
        let r = c.decode("test_msgs/Imglike", CdrEndianness::Little, &body, 0, 0);
        match r {
            Err(CdrCodecError::MalformedCdrString {
                schema,
                field,
                detail,
            }) => {
                assert_eq!(schema, "test_msgs/Imglike");
                assert_eq!(field, "encoding");
                assert!(detail.contains("0x41"), "names the byte: {detail}");
            }
            other => panic!("expected MalformedCdrString, got {other:?}"),
        }
    }

    #[test]
    fn test_string_zero_length_prefix_is_err() {
        // A 0 length prefix is invalid CDR (a
        // well-formed string always contains its NUL) — loud Err, never a
        // lenient empty string.
        let c = codec(vec![imglike()]);
        let mut body = Vec::new();
        body.extend_from_slice(&0u32.to_le_bytes()); // height
        body.extend_from_slice(&0u32.to_le_bytes()); // width
        body.extend_from_slice(&0u32.to_le_bytes()); // encoding len 0 — invalid
        body.extend_from_slice(&0u32.to_le_bytes()); // data count 0
        let r = c.decode("test_msgs/Imglike", CdrEndianness::Little, &body, 0, 0);
        assert!(
            matches!(r, Err(CdrCodecError::MalformedCdrString { .. })),
            "{r:?}"
        );
    }

    #[test]
    fn test_encode_empty_variable_nested_emits_zero_default_fields() {
        // A zero-length offset entry for a variable-
        // nested field (the native `set_header_bytes(&[])` idiom — present-
        // with-zero-fields to the frame walker) must encode as the nested
        // schema's zero-default fields, never reject the frame.
        let mut stamped = MessageSchema::new_in_package("Stamped", "test_msgs");
        stamped.add_field(FieldDef::new("header", nested("Header", "std_msgs")));
        stamped.add_field(FieldDef::new("value", FieldType::U32));
        let c = codec(vec![time(), header(), stamped]);

        // Hand Cerulion payload: fixed [value=7] + table [(0,0)] + no var.
        let mut payload = Vec::new();
        payload.extend_from_slice(&7u32.to_le_bytes());
        payload.extend_from_slice(&0u32.to_le_bytes()); // entry offset 0
        payload.extend_from_slice(&0u32.to_le_bytes()); // entry len 0 — EMPTY
        let cdr = c
            .encode(
                "test_msgs/Stamped",
                CdrEndianness::Little,
                &headerless_input(&payload),
            )
            .expect("an empty variable-nested field encodes as zero-defaults");

        // Hand oracle: Header zero-default (Time{0,0} + empty string), then
        // value=7 (u32 aligned 13→16).
        let mut expect = Vec::new();
        expect.extend_from_slice(&0i32.to_le_bytes()); // stamp.sec
        expect.extend_from_slice(&0u32.to_le_bytes()); // stamp.nanosec
        expect.extend_from_slice(&1u32.to_le_bytes()); // frame_id len (NUL only)
        expect.push(0); // the NUL — body ends at 13
        expect.extend_from_slice(&[0, 0, 0]); // pad to 16
        expect.extend_from_slice(&7u32.to_le_bytes());
        assert_eq!(cdr, expect);

        // The zero-default CDR round-trips: decode materializes a FULL
        // zero-valued sub-frame (decode never emits len-0 nested entries),
        // and re-encoding reproduces the same CDR bytes.
        let frame = c
            .decode("test_msgs/Stamped", CdrEndianness::Little, &cdr, 0, 0)
            .expect("zero-default CDR decodes");
        let re = c
            .encode("test_msgs/Stamped", CdrEndianness::Little, &frame)
            .expect("re-encode");
        assert_eq!(re, cdr, "encode(decode(x)) == x holds for the zero-default");
    }

    #[test]
    fn test_unknown_schema_is_err() {
        let c = codec(vec![vec3()]);
        let r = c.decode("nope/Nope", CdrEndianness::Little, &[], 0, 0);
        assert_eq!(
            r,
            Err(CdrCodecError::UnknownSchema("nope/Nope".to_string()))
        );
    }

    #[test]
    fn test_split_encapsulation() {
        assert_eq!(
            split_encapsulation(&[0x00, 0x01, 0x00, 0x00, 0xAA]),
            Ok((CdrEndianness::Little, &[0xAAu8][..]))
        );
        assert_eq!(
            split_encapsulation(&[0x00, 0x00, 0x00, 0x00, 0xBB]),
            Ok((CdrEndianness::Big, &[0xBBu8][..]))
        );
        assert!(matches!(
            split_encapsulation(&[0x00, 0x03, 0x00, 0x00]), // PL_CDR_LE
            Err(CdrCodecError::UnsupportedRepresentation { rep_id: 0x0003 })
        ));
        assert!(matches!(
            split_encapsulation(&[0x00, 0x01]),
            Err(CdrCodecError::MissingEncapsulation { need: 4, have: 2 })
        ));
    }

    #[test]
    fn test_decode_is_deterministic() {
        let c = codec(vec![imglike()]);
        let mut body = Vec::new();
        body.extend_from_slice(&1u32.to_le_bytes());
        body.extend_from_slice(&2u32.to_le_bytes());
        body.extend_from_slice(&4u32.to_le_bytes());
        body.extend_from_slice(b"abc\0");
        body.extend_from_slice(&2u32.to_le_bytes());
        body.extend_from_slice(&[9, 8]);
        let a = c
            .decode("test_msgs/Imglike", CdrEndianness::Little, &body, 3, 4)
            .unwrap();
        let b = c
            .decode("test_msgs/Imglike", CdrEndianness::Little, &body, 3, 4)
            .unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn test_decode_dds_payload_convenience() {
        let c = codec(vec![vec3()]);
        let mut dds = vec![0x00, 0x01, 0x00, 0x00]; // CDR_LE header
        for v in [1.0f64, 2.0, 3.0] {
            dds.extend_from_slice(&v.to_le_bytes());
        }
        let frame = c
            .decode_dds_payload("geometry_msgs/Vector3", &dds, 0, 0)
            .unwrap();
        assert_eq!(&frame[WireHeader::SIZE..], &dds[4..]);
    }

    // ---- The NOT-FULLY-CONSUMED gate ---------------------------------------
    //
    // Every oracle below is a HAND-BUILT body with a hand-computed
    // consumed/remaining split — never a self-compare against another decode.

    /// The allowance is the RTPS submessage alignment ceiling, and the
    /// `TrailingBytes` Display text says "3" as a literal (a const cannot be
    /// interpolated into a `thiserror` format string). Pin the two together so
    /// the message cannot rot away from the check.
    #[test]
    fn the_trailing_pad_allowance_is_the_rtps_alignment_ceiling() {
        assert_eq!(
            MAX_CDR_TRAILING_PAD, 3,
            "a SerializedPayload is padded to the submessage's 4-byte alignment, so 3 is the \
             most a conformant writer can leave; changing this must also change the \
             TrailingBytes Display text, which spells it literally"
        );
        let text = CdrCodecError::TrailingBytes {
            schema: "x/Y".to_string(),
            consumed: 24,
            remaining: 4,
        }
        .to_string();
        assert!(
            text.contains("more than the 3 bytes"),
            "Display must name the allowance: {text}"
        );
    }

    /// Exact consumption passes; 1..=3 leftover bytes pass (indistinguishable
    /// from a real alignment pad); 4 fails. The boundary is pinned on BOTH
    /// sides, so neither a stricter nor a looser gate survives.
    #[test]
    fn consumption_gate_accepts_the_alignment_pad_and_rejects_one_byte_past_it() {
        let c = codec(vec![vec3()]);
        // geometry_msgs/Vector3 == 3 × f64 == exactly 24 CDR bytes.
        let mut exact = Vec::new();
        for v in [1.0f64, 2.0, 3.0] {
            exact.extend_from_slice(&v.to_le_bytes());
        }
        assert_eq!(exact.len(), 24);

        // LITERAL 0..=3, deliberately NOT derived from MAX_CDR_TRAILING_PAD:
        // an accept-loop bounded by the constant under test shrinks with it,
        // so an over-strict regression (allowance 0 — which would refuse every
        // frame from a conformant, submessage-padding robot) would silently
        // stop being tested. These three are the RTPS ceiling, independently.
        for pad in [0usize, 1, 2, 3] {
            let mut body = exact.clone();
            body.extend(std::iter::repeat_n(0u8, pad));
            let frame = c
                .decode("geometry_msgs/Vector3", CdrEndianness::Little, &body, 0, 0)
                .unwrap_or_else(|e| panic!("{pad} trailing byte(s) is a legal pad, got {e:?}"));
            // The pad is DROPPED, never smuggled into the payload.
            assert_eq!(
                &frame[WireHeader::SIZE..],
                &exact[..],
                "pad {pad} must not reach the Cerulion payload"
            );
        }

        // One byte past the ceiling: rejected, with an exact hand split.
        let mut over = exact.clone();
        over.extend_from_slice(&[0, 0, 0, 0]);
        assert_eq!(
            c.decode("geometry_msgs/Vector3", CdrEndianness::Little, &over, 0, 0),
            Err(CdrCodecError::TrailingBytes {
                schema: "geometry_msgs/Vector3".to_string(),
                consumed: 24,
                remaining: 4,
            })
        );
    }

    /// Consumption SHORT by one byte is a truncation, not a trailing-bytes
    /// finding — the two directions of "this is not the writer's schema" keep
    /// their distinct, separately-actionable variants.
    #[test]
    fn a_body_one_byte_short_is_truncated_never_trailing_bytes() {
        let c = codec(vec![vec3()]);
        let mut short = Vec::new();
        for v in [1.0f64, 2.0, 3.0] {
            short.extend_from_slice(&v.to_le_bytes());
        }
        short.pop(); // 23 bytes — the last f64 cannot be read
        let err = c
            .decode("geometry_msgs/Vector3", CdrEndianness::Little, &short, 0, 0)
            .expect_err("a short body must not decode");
        assert!(
            matches!(err, CdrCodecError::Truncated { .. }),
            "expected Truncated, got {err:?}"
        );
    }

    /// The member-less edge, at the shape a REAL ROS 2 publisher emits.
    ///
    /// IDL forbids an empty struct, so rosidl gives `std_msgs/Empty` a
    /// `uint8 structure_needs_at_least_one_member` placeholder and the body is
    /// ONE octet — plus RTPS submessage padding, so the bridge typically sees
    /// 4 bytes. Cerulion models `Empty` as genuinely field-less, so without
    /// [`IDL_EMPTY_STRUCT_OCTETS`] handling the walk would account for zero of
    /// those bytes and the gate would refuse every single sample: a live
    /// `/probe` topic dead, flooding errors, on a frame that worked before.
    /// This arm is the anti-false-positive pin for that class.
    #[test]
    fn a_member_less_message_decodes_at_its_real_one_octet_wire_shape() {
        let mut empty = MessageSchema::new_in_package("Empty", "std_msgs");
        empty.description = None;
        let c = codec(vec![empty]);

        // The placeholder octet, then 0..=3 bytes of RTPS padding. Literal
        // bounds for the same reason as the sibling accept-loop.
        for pad in [0usize, 1, 2, 3] {
            let body = vec![0u8; IDL_EMPTY_STRUCT_OCTETS + pad];
            let frame = c
                .decode("std_msgs/Empty", CdrEndianness::Little, &body, 0, 0)
                .unwrap_or_else(|e| {
                    panic!("a real Empty body (1 octet + {pad} pad) must decode, got {e:?}")
                });
            assert_eq!(
                frame.len(),
                WireHeader::SIZE,
                "the placeholder is DROPPED — a Cerulion Empty frame has no payload"
            );
        }

        // Tolerated: a writer that omits the placeholder entirely (the octet
        // carries no information, and refusing it would be a fresh dead-route
        // class).
        assert!(c
            .decode("std_msgs/Empty", CdrEndianness::Little, &[], 0, 0)
            .is_ok());

        // But real content is still caught — the gate does not need a field to
        // fire, it just no longer counts the placeholder against the writer.
        assert_eq!(
            c.decode(
                "std_msgs/Empty",
                CdrEndianness::Little,
                &[0, 1, 2, 3, 4, 5, 6, 7],
                0,
                0
            ),
            Err(CdrCodecError::TrailingBytes {
                schema: "std_msgs/Empty".to_string(),
                consumed: 1,
                remaining: 7,
            })
        );

        // Egress mirror: encode re-emits the octet, so a ROS 2 reader sees the
        // body length it expects and `encode(decode(x)) == x` holds.
        assert_eq!(
            c.encode(
                "std_msgs/Empty",
                CdrEndianness::Little,
                &headerless_input(&[])
            ),
            Ok(vec![0u8])
        );
    }

    /// The reason the placeholder MUST be consumed rather than merely allowed
    /// for: a member-less message NESTED inside another shifts every following
    /// field by one byte, and the leftover is exactly 1 — inside the alignment
    /// allowance, where `TrailingBytes` structurally cannot see it. So a bare
    /// "let member-less schemas leave up to 4 bytes" fix would have converted
    /// this from a caught error into a SILENT mis-decode: the very class
    /// this gate exists to close.
    ///
    /// Oracle is the hand-computed field value, not another decode.
    ///
    /// The ENCODE half is pinned in the same body, and it is the only arm in
    /// the suite that covers the NESTED placeholder write: the sole
    /// member-less encode anywhere else is top-level `std_msgs/Empty`, which
    /// exercises a different call site. Without it, deleting the nested
    /// `write_idl_placeholder` goes undetected — an egress frame one byte
    /// short of what a ROS 2 reader expects, with every following field
    /// shifted, and a green suite.
    #[test]
    fn a_nested_member_less_message_does_not_shift_the_following_field() {
        let mut empty = MessageSchema::new_in_package("Empty", "std_msgs");
        empty.description = None;
        let mut outer = MessageSchema::new_in_package("Flagged", "test_msgs");
        outer.add_field(FieldDef::new("flag", nested("Empty", "std_msgs")));
        outer.add_field(FieldDef::new("value", FieldType::U32));
        let c = codec(vec![empty, outer]);

        // CDR: placeholder octet, pad to the u32's 4-alignment, then the u32.
        let mut body = vec![0u8; IDL_EMPTY_STRUCT_OCTETS];
        body.extend_from_slice(&[0, 0, 0]); // alignment pad before the u32
        body.extend_from_slice(&0xABCD_1234u32.to_le_bytes());
        assert_eq!(body.len(), 8);

        let frame = c
            .decode("test_msgs/Flagged", CdrEndianness::Little, &body, 0, 0)
            .expect("a nested member-less field must not break the walk");
        let payload = &frame[WireHeader::SIZE..];
        assert_eq!(
            &payload[payload.len() - 4..],
            &0xABCD_1234u32.to_le_bytes(),
            "`value` must read at its true offset — a skipped placeholder \
             would shift it by one byte and decode 0x341200AB-ish garbage"
        );

        // Egress mirror, against the SAME hand-built body: the nested
        // placeholder must be re-emitted, or `value` lands at CDR offset 0
        // instead of 4 and the body a ROS 2 reader receives is 4 bytes, not 8.
        assert_eq!(
            c.encode("test_msgs/Flagged", CdrEndianness::Little, &frame),
            Ok(body),
            "the nested member-less placeholder must be written back"
        );
    }

    /// THE CLASS. Two codecs carry the SAME qualified name with
    /// DIFFERENT definitions — the robot's (3 fields) and a local corpus entry
    /// that is a strict PREFIX of it (2 fields). This is the shape the wire
    /// `schema_hash` gate structurally cannot catch: the publisher stamps the
    /// hash from the corpus entry and the subscriber expects it from the same
    /// corpus entry, so both sides agree while both are wrong.
    ///
    /// Without the gate this decodes "successfully", silently dropping the third
    /// field's bytes. The test asserts the exact split so a failure is
    /// attributable, and separately asserts the WRITER's codec reads the same
    /// body cleanly — proving the body is well-formed and the schema, not the
    /// bytes, is what the gate rejected.
    #[test]
    fn a_corpus_schema_shorter_than_the_writers_is_caught_not_silently_decoded() {
        fn stamped(with_third_field: bool) -> MessageSchema {
            let mut s = MessageSchema::new_in_package("Reading", "sensor_msgs");
            s.add_field(FieldDef::new("seq", FieldType::U32));
            s.add_field(FieldDef::new("value", FieldType::F64));
            if with_third_field {
                s.add_field(FieldDef::new("extra", FieldType::U32));
            }
            s
        }
        let writer = codec(vec![stamped(true)]);
        let corpus = codec(vec![stamped(false)]);

        // The body the robot really put on the wire: u32 seq, pad to 8, f64
        // value, u32 extra → 4 + 4 + 8 + 4 = 20 bytes.
        let mut body = Vec::new();
        body.extend_from_slice(&7u32.to_le_bytes());
        body.extend_from_slice(&[0, 0, 0, 0]); // CDR pad before the f64
        body.extend_from_slice(&1.5f64.to_le_bytes());
        body.extend_from_slice(&0xDEAD_BEEFu32.to_le_bytes());
        assert_eq!(body.len(), 20);

        // The writer's own definition consumes all 20 — the body is sound.
        writer
            .decode("sensor_msgs/Reading", CdrEndianness::Little, &body, 0, 0)
            .expect("the writer's definition accounts for every byte");

        // The short corpus entry stops after `value` (16 bytes) and is now
        // REFUSED instead of publishing a frame missing `extra`.
        assert_eq!(
            corpus.decode("sensor_msgs/Reading", CdrEndianness::Little, &body, 0, 0),
            Err(CdrCodecError::TrailingBytes {
                schema: "sensor_msgs/Reading".to_string(),
                consumed: 16,
                remaining: 4,
            })
        );
    }

    /// The front-video shape, on the class the `HostileLength` guard MISSES.
    ///
    /// The Go2's `/frontvideostream` publishes `unitree_go/Go2FrontVideoData`
    /// as `[u64 time_frame][u32 video_height][u32 len][Annex-B H.264]` (the
    /// three-field definition the local store carries, derived from 60
    /// captured live samples). The community FOUR-field definition read that
    /// `video_height` as a byte-array length and blew up as
    /// `HostileLength` — loudly, but only by luck of that particular
    /// divergence. Had the local definition been SHORTER rather than
    /// mis-typed, every length prefix would have been sane and the bridge
    /// would have published a frame with the video silently missing.
    ///
    /// The body here is hand-built in that documented shape (Annex-B start
    /// code at body offset 16, `video_height` 360) because the real captures
    /// live in `examples/go2`, a SEPARATE cargo workspace — `cerulion_core` is
    /// the library those demos depend on, so reaching back into one would
    /// invert the dependency. The same contract IS pinned on the real robot
    /// bytes, in the workspace that owns them:
    /// `examples/go2/nodes/dds_bridge/tests/frontvideostream_wire_test.rs::`
    /// `real_bytes_pass_the_consumption_gate_and_a_short_definition_is_refused`.
    #[test]
    fn the_video_shape_is_caught_when_the_local_definition_is_short() {
        fn go2_video(with_payload_field: bool) -> MessageSchema {
            let mut s = MessageSchema::new_in_package("Go2FrontVideoData", "unitree_go");
            s.add_field(FieldDef::new("time_frame", FieldType::U64));
            s.add_field(FieldDef::new("video_height", FieldType::U32));
            if with_payload_field {
                s.add_field(FieldDef::new("video_data", FieldType::Bytes));
            }
            s
        }
        let robot = codec(vec![go2_video(true)]);
        let short_corpus = codec(vec![go2_video(false)]);

        const NAL: [u8; 8] = [0x00, 0x00, 0x00, 0x01, 0x41, 0x9A, 0x02, 0x0F];
        let mut body = Vec::new();
        body.extend_from_slice(&1_234_567_890u64.to_le_bytes());
        body.extend_from_slice(&360u32.to_le_bytes());
        body.extend_from_slice(&(NAL.len() as u32).to_le_bytes());
        body.extend_from_slice(&NAL);
        assert_eq!(
            &body[16..20],
            &[0x00, 0x00, 0x00, 0x01],
            "the Annex-B start code sits at body offset 16 in the real capture"
        );

        // The robot's definition accounts for the whole body, video included.
        let frame = robot
            .decode(
                "unitree_go/Go2FrontVideoData",
                CdrEndianness::Little,
                &body,
                0,
                0,
            )
            .expect("the robot's definition consumes the video payload");
        assert!(
            frame.ends_with(&NAL),
            "the decoded frame carries the H.264 bytes"
        );

        // The short local definition leaves the 4-byte length prefix + the
        // whole video payload unread — now a loud refusal, previously a
        // silently video-less frame.
        assert_eq!(
            short_corpus.decode(
                "unitree_go/Go2FrontVideoData",
                CdrEndianness::Little,
                &body,
                0,
                0,
            ),
            Err(CdrCodecError::TrailingBytes {
                schema: "unitree_go/Go2FrontVideoData".to_string(),
                consumed: 12,
                remaining: 4 + NAL.len(),
            })
        );
    }

    /// The gate is endian-independent (it reads cursor positions, never
    /// values), pinned on the REFUSAL side for big-endian too — the pass side
    /// is already covered by the pre-existing BE round-trip arms, but an
    /// endian-conditional regression in the refusal path would otherwise go
    /// unnoticed.
    #[test]
    fn the_gate_fires_identically_on_a_big_endian_body() {
        let c = codec(vec![vec3()]);
        let mut body = Vec::new();
        for v in [1.0f64, 2.0, 3.0] {
            body.extend_from_slice(&v.to_be_bytes());
        }
        body.extend_from_slice(&[0; 4]);
        assert_eq!(
            c.decode("geometry_msgs/Vector3", CdrEndianness::Big, &body, 0, 0),
            Err(CdrCodecError::TrailingBytes {
                schema: "geometry_msgs/Vector3".to_string(),
                consumed: 24,
                remaining: 4,
            })
        );
    }

    /// The gate lives at the TOP-LEVEL entry only, because `decode_payload`
    /// recurses for variable-nested sub-frames off the SAME reader, where
    /// stopping mid-body is correct — only the outermost cursor is
    /// meaningful. Named arm for that load-bearing placement: a message whose
    /// FIRST field is a variable-nested sub-frame decodes cleanly (the nested
    /// walk leaving the rest of the body unread must not fire the gate), while
    /// a genuinely short outer definition of the same body still does.
    #[test]
    fn a_variable_nested_subframe_does_not_trip_the_top_level_gate() {
        fn stamped(with_tail: bool) -> MessageSchema {
            let mut s = MessageSchema::new_in_package("Stamped2", "test_msgs");
            // std_msgs/Header is variable (its frame_id is a string), so this
            // field decodes through the recursive `decode_payload`.
            s.add_field(FieldDef::new("header", nested("Header", "std_msgs")));
            if with_tail {
                s.add_field(FieldDef::new("value", FieldType::U32));
            }
            s
        }
        let writer = codec(vec![time(), header(), stamped(true)]);
        let corpus = codec(vec![time(), header(), stamped(false)]);

        // CDR: Time{7,8}, frame_id "ab", then the u32 tail.
        let mut body = Vec::new();
        body.extend_from_slice(&7i32.to_le_bytes());
        body.extend_from_slice(&8u32.to_le_bytes());
        body.extend_from_slice(&3u32.to_le_bytes()); // "ab" + NUL
        body.extend_from_slice(b"ab\0");
        body.push(0); // pad 15 -> 16 for the u32
        body.extend_from_slice(&99u32.to_le_bytes());
        assert_eq!(body.len(), 20);

        // The nested walk consumes only the header; the OUTER field consumes
        // the rest — no gate, and the tail really is read.
        let frame = writer
            .decode("test_msgs/Stamped2", CdrEndianness::Little, &body, 0, 0)
            .expect("a variable-nested first field must not trip the gate");
        assert!(
            frame.windows(4).any(|w| w == 99u32.to_le_bytes()),
            "the u32 tail after the nested sub-frame must be decoded"
        );

        // Same body, outer definition missing the tail: the gate fires on the
        // bytes the nested walk deliberately left behind for it. Consumed is
        // 15, not 16 — CDR alignment is applied by the READ that needs it, and
        // the absent `value` field never asks, so the pad byte before it stays
        // unconsumed too.
        assert_eq!(
            corpus.decode("test_msgs/Stamped2", CdrEndianness::Little, &body, 0, 0),
            Err(CdrCodecError::TrailingBytes {
                schema: "test_msgs/Stamped2".to_string(),
                consumed: 15,
                remaining: 5,
            })
        );
    }

    /// The gate rides `decode_dds_payload` too (the encapsulation-stripping
    /// convenience entry), so a caller that hands over a full DDS payload is
    /// covered identically to one that hands over a bare body.
    #[test]
    fn the_gate_also_fires_through_decode_dds_payload() {
        let c = codec(vec![vec3()]);
        let mut dds = vec![0x00, 0x01, 0x00, 0x00]; // CDR_LE encapsulation
        for v in [1.0f64, 2.0, 3.0] {
            dds.extend_from_slice(&v.to_le_bytes());
        }
        dds.extend_from_slice(&[0xAA; 4]);
        let err = c
            .decode_dds_payload("geometry_msgs/Vector3", &dds, 0, 0)
            .expect_err("4 trailing body bytes are refused through the DDS entry too");
        assert!(
            matches!(
                err,
                CdrCodecError::TrailingBytes {
                    consumed: 24,
                    remaining: 4,
                    ..
                }
            ),
            "expected TrailingBytes(24, 4), got {err:?}"
        );
        // The message names the schema and states the diagnosis + the fix.
        //
        // BOTH remedy branches are pinned, each with the precondition that
        // makes it actually work.
        //
        // Branch 1's "delete the stale .msg and re-attach" is a NO-OP when a
        // COMPILED-IN definition of the same name exists: the attach chain
        // predicate is `store ∪ builtins` and runs BEFORE the acquisition
        // ladder, so the built-in resolves the type and the wire rung never
        // runs. The branch therefore carries that qualifier and points at
        // branch 2.
        //
        // Branch 2's SHADOW is likewise inert on its own. The bridge's codec
        // only reads the workspace store when its config carries `msg_dirs:`
        // (`BridgeConfig::effective_msg_dirs` → `bridge_schema_set_with_store`,
        // which early-returns on an empty slice), and `ros2 attach` emits that
        // key only when the store was non-empty or the run materialized files
        // — i.e. NOT in the scenario this branch addresses. So the branch names
        // the two ways to get the key emitted.
        let text = err.to_string();
        for expect in [
            "geometry_msgs/Vector3",
            "NOT fully consumed",
            "28 body bytes",
            // Branch 1 — workspace store: delete + re-attach, with the
            // compiled-in-definition qualifier that makes it accurate.
            "workspace store",
            "delete the stale schemas/<pkg>/msg/<Type>.msg",
            "cerulion ros2 attach",
            "UNLESS a COMPILED-IN definition of the same name exists",
            // Branch 2 — compiled corpus: no file to delete, so SHADOW it,
            // then make the bridge actually LOAD the store.
            "BUILTIN_MSGS",
            "WRITE a corrected schemas/<pkg>/msg/<Type>.msg",
            "SHADOWS",
            "ONLY when its config carries msg_dirs",
            "`msg_dirs: [../schemas]`",
        ] {
            assert!(
                text.contains(expect),
                "message must carry {expect:?}: {text}"
            );
        }
    }
}
