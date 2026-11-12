// SPDX-License-Identifier: AGPL-3.0-only
//! What a replay byte-mismatch line names: the field of the first differing
//! frame and its two values, spelled as `cerulion topic echo` prints them.
//!
//! The replay diff keeps the first differing frame of each stream with its
//! recorded twin (`ByteViolation` in `replay_engine`). After the pass, ONE
//! call of [`locate_difference`] turns that pair into a [`FrameDifference`],
//! deciding in this order:
//!
//! 1. A frame shorter than the 40-byte wire header: not decoded.
//! 2. Different `schema_hash` values: named, fields not compared.
//! 3. A frame the schema set cannot decode (no schema has the hash, or the
//!    walker refuses the bytes): not decoded, with the reason.
//! 4. A differing wire-header field other than `total_size` (which follows
//!    the payload length and is in the size clause already), in byte order.
//! 5. The first differing message value, in the schema's DECLARATION order
//!    (the order `topic echo` prints), which can differ from byte order.
//! 6. With every value equal: the tolerance decoder's refusal of a frame,
//!    when it refused one.
//! 7. Otherwise the first differing byte outside `total_size` and the part
//!    of the frame it sits in, or `total_size` itself when no other byte
//!    differs.
//!
//! Every other differing value of the frame is counted, never named. Values
//! use the YAML form's scalar spellings ([`cerulion_core::dynamic`]'s hidden
//! `yaml_format_f64` / `yaml_format_f32` / `yaml_quote_within`), the same
//! code `topic echo` prints through, so the two cannot spell a value
//! differently. Nothing here runs on a publish or receive path: it runs once
//! per diverging stream after a pass, and it allocates the strings it prints.

use std::fmt::Write as _;

use cerulion_core::codegen::layout::WireLayout;
use cerulion_core::dynamic::{
    yaml_format_f32, yaml_format_f64, yaml_quote_within, FieldType, FrameValue, FrameValueKind,
    FrameWalker, MessageSchema, PrimArray, PrimType, WireHeader,
};

use crate::replay_engine::{
    ByteDifference, DifferenceKind, DifferenceValue, HEADER_LENGTH_DERIVED_BYTES,
};

/// Strings of at most this many characters print whole; a longer pair
/// prints this many characters of each side around the first difference.
const STRING_WINDOW: usize = 64;

/// How many characters of a windowed string print before its first
/// difference.
const STRING_LEAD: usize = 16;

/// A path longer than this many bytes keeps its last this-many bytes,
/// behind a leading `...`.
const MAX_PATH_BYTES: usize = 256;

/// Which frame of the twin pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Side {
    Recorded,
    Replayed,
}

impl Side {
    fn word(self) -> &'static str {
        match self {
            Self::Recorded => "recorded",
            Self::Replayed => "replayed",
        }
    }
}

/// One side's value: the text the line prints and the value the report
/// carries.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Shown {
    pub(crate) text: String,
    pub(crate) json: DifferenceValue,
}

impl Shown {
    /// A value with no JSON number or bool form: the report carries the
    /// line's text.
    fn text(text: String) -> Self {
        Self {
            json: DifferenceValue::Text(text.clone()),
            text,
        }
    }
}

/// The first differing message value of a frame.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ValueDifference {
    /// The field path (`linear.x`, `ranges[12]`, `poses[2].position.x`).
    pub(crate) path: String,
    /// Where inside the field, for a field compared in parts: ` from char
    /// 120` (a long string), `, byte 12 of its opaque bytes`.
    pub(crate) at: String,
    pub(crate) recorded: Shown,
    pub(crate) replayed: Shown,
    /// A clause after the two values (` (bytes differ past the NUL at field
    /// byte 5)`), empty for most values.
    pub(crate) note: String,
}

/// What [`diff_frame_values`] found: the first differing value in
/// declaration order, and how many other values differ.
#[derive(Debug, Clone, PartialEq, Default)]
pub(crate) struct FrameValueDiff {
    pub(crate) first: Option<ValueDifference>,
    pub(crate) more: u64,
}

impl FrameValueDiff {
    /// Every differing value, the first included.
    fn count(&self) -> u64 {
        self.more + u64::from(self.first.is_some())
    }
}

/// A wire-header field the line can name.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum HeaderField {
    /// Named only when no other byte differs.
    TotalSize,
    OffsetTableOffset,
    OffsetTableCount,
    Sequence,
    TimestampNs,
    StepTag,
}

impl HeaderField {
    /// The fields named ahead of the payload, in byte order.
    const COMPARED: [Self; 5] = [
        Self::OffsetTableOffset,
        Self::OffsetTableCount,
        Self::Sequence,
        Self::TimestampNs,
        Self::StepTag,
    ];

    fn name(self) -> &'static str {
        match self {
            Self::TotalSize => "total_size",
            Self::OffsetTableOffset => "offset_table_offset",
            Self::OffsetTableCount => "offset_table_count",
            Self::Sequence => "sequence",
            Self::TimestampNs => "timestamp_ns",
            Self::StepTag => "step_tag",
        }
    }

    fn shown(self, header: &WireHeader) -> Shown {
        let number = |v: u64| Shown {
            text: v.to_string(),
            json: DifferenceValue::UInt(v),
        };
        match self {
            Self::TotalSize => number(u64::from(header.total_size)),
            Self::OffsetTableOffset => number(u64::from(header.offset_table_offset)),
            Self::OffsetTableCount => number(u64::from(header.offset_table_count)),
            Self::Sequence => number(u64::from(header.sequence)),
            Self::TimestampNs => Shown {
                text: format!("{} ns", header.timestamp_ns),
                json: DifferenceValue::UInt(header.timestamp_ns),
            },
            Self::StepTag => {
                let tag = header.step_tag;
                Shown::text(if tag.is_untagged() {
                    "untagged".to_string()
                } else {
                    match tag.step() {
                        Some(step) => format!("rank {} step {step}", tag.rank()),
                        None => format!("rank {}, no step", tag.rank()),
                    }
                })
            }
        }
    }

    fn differs(self, recorded: &WireHeader, replayed: &WireHeader) -> bool {
        match self {
            Self::TotalSize => recorded.total_size != replayed.total_size,
            Self::OffsetTableOffset => recorded.offset_table_offset != replayed.offset_table_offset,
            Self::OffsetTableCount => recorded.offset_table_count != replayed.offset_table_count,
            Self::Sequence => recorded.sequence != replayed.sequence,
            Self::TimestampNs => recorded.timestamp_ns != replayed.timestamp_ns,
            Self::StepTag => recorded.step_tag.raw() != replayed.step_tag.raw(),
        }
    }
}

/// The part of a frame a differing byte sits in when every value is equal.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum LayoutRegion {
    /// Padding between or after fixed-section fields.
    FixedSectionPadding,
    /// The offset table.
    OffsetTable,
    /// Inside a variable field's bytes, outside every value it holds.
    FieldFraming { path: String },
    /// Between two variable fields.
    UnusedVariableBytes,
    /// Past the end of the last field.
    TrailingBytes,
}

impl LayoutRegion {
    fn phrase(&self) -> String {
        match self {
            Self::FixedSectionPadding => "the fixed-section padding".to_string(),
            Self::OffsetTable => "the offset table".to_string(),
            Self::FieldFraming { path } => format!("the framing of {path}"),
            Self::UnusedVariableBytes => "unused bytes between variable fields".to_string(),
            Self::TrailingBytes => "bytes past the last field".to_string(),
        }
    }

    fn tag(&self) -> &'static str {
        match self {
            Self::FixedSectionPadding => "fixed_section_padding",
            Self::OffsetTable => "offset_table",
            Self::FieldFraming { .. } => "field_framing",
            Self::UnusedVariableBytes => "unused_variable_bytes",
            Self::TrailingBytes => "trailing_bytes",
        }
    }
}

/// Why the fields of a differing frame were not decoded.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum UndecodedReason {
    /// The frame is shorter than the wire header.
    FrameTooShort { side: Side, len: usize },
    /// No schema in the set has the frames' hash.
    UnknownSchemaHash { schema_hash: u64 },
    /// The frame does not decode as its schema.
    Undecodable {
        side: Side,
        schema: String,
        error: String,
    },
}

impl UndecodedReason {
    fn clause(&self) -> String {
        match self {
            Self::FrameTooShort { side, len } => format!(
                "the {} frame is {len} B, shorter than the 40-byte wire header",
                side.word()
            ),
            Self::UnknownSchemaHash { schema_hash } => format!(
                "no schema in this workspace or the built-in set has the frame's schema hash \
                 0x{schema_hash:016x}"
            ),
            Self::Undecodable {
                side,
                schema,
                error,
            } => format!(
                "the {} frame does not decode as {schema}: {error}",
                side.word()
            ),
        }
    }

    fn tag(&self) -> &'static str {
        match self {
            Self::FrameTooShort {
                side: Side::Recorded,
                ..
            } => "recorded_frame_too_short",
            Self::FrameTooShort {
                side: Side::Replayed,
                ..
            } => "replayed_frame_too_short",
            Self::UnknownSchemaHash { .. } => "unknown_schema_hash",
            Self::Undecodable {
                side: Side::Recorded,
                ..
            } => "recorded_frame_undecodable",
            Self::Undecodable {
                side: Side::Replayed,
                ..
            } => "replayed_frame_undecodable",
        }
    }
}

/// What differs in the first differing frame of a stream.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum FrameDifference {
    /// A message value.
    Field {
        value: ValueDifference,
        /// The other differing values.
        more: u64,
    },
    /// A wire-header field.
    Header {
        field: HeaderField,
        recorded: Shown,
        replayed: Shown,
        /// The other differing header fields and message values.
        more: u64,
    },
    /// The two frames carry different schemas.
    SchemaHash { recorded: u64, replayed: u64 },
    /// Every value is equal; a byte of the framing differs.
    Layout {
        byte_offset: usize,
        region: LayoutRegion,
    },
    /// The fields were not decoded.
    Undecoded {
        byte_offset: usize,
        reason: UndecodedReason,
    },
}

/// The tolerance plan's field decoder refused one frame of the pair, so its
/// byte-exact remainder was not compared.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct RemainderRefusal {
    pub(crate) side: Side,
    pub(crate) error: String,
}

/// One stream's byte verdict: the frame coordinates the report has always
/// carried plus what differs.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ByteVerdict {
    pub(crate) index: usize,
    pub(crate) first_diff_byte_offset: usize,
    pub(crate) recorded_len: usize,
    pub(crate) replayed_len: usize,
    pub(crate) difference: FrameDifference,
}

/// The detail of a byte-mismatch violation: `frame {i} on '{topic}'{scope}`,
/// what differs, then ` (recorded {n} B, replayed {m} B)`.
pub(crate) fn byte_mismatch_detail(topic: &str, scope: &str, verdict: &ByteVerdict) -> String {
    let mut out = format!("frame {} on '{topic}'{scope} differs ", verdict.index);
    let more_clause = |more: u64| {
        if more == 0 {
            String::new()
        } else {
            format!("; {more} more value(s) differ")
        }
    };
    let _ = match &verdict.difference {
        FrameDifference::Field { value, more } => write!(
            out,
            "at {}{}: recorded {}, replayed {}{}{}",
            value.path,
            value.at,
            value.recorded.text,
            value.replayed.text,
            value.note,
            more_clause(*more)
        ),
        FrameDifference::Header {
            field,
            recorded,
            replayed,
            more,
        } => write!(
            out,
            "in the wire header at {}: recorded {}, replayed {}{}",
            field.name(),
            recorded.text,
            replayed.text,
            more_clause(*more)
        ),
        FrameDifference::SchemaHash { recorded, replayed } => write!(
            out,
            "in the wire header at schema_hash: recorded 0x{recorded:016x}, replayed \
             0x{replayed:016x}; fields not compared, the frames carry different schemas"
        ),
        FrameDifference::Layout {
            byte_offset,
            region,
        } => write!(
            out,
            "at byte {byte_offset}, in {}; every field value is equal",
            region.phrase()
        ),
        FrameDifference::Undecoded {
            byte_offset,
            reason,
        } => write!(
            out,
            "at byte {byte_offset}, not decoded because {}",
            reason.clause()
        ),
    };
    let _ = write!(
        out,
        " (recorded {} B, replayed {} B)",
        verdict.recorded_len, verdict.replayed_len
    );
    out
}

/// The `--report` entry of a difference: one flat object whose `kind` says
/// which keys carry a value.
pub(crate) fn report_entry(difference: &FrameDifference) -> ByteDifference {
    let empty = |kind| ByteDifference {
        kind,
        path: None,
        recorded: None,
        replayed: None,
        byte_offset: None,
        region: None,
        reason: None,
        more: 0,
    };
    match difference {
        FrameDifference::Field { value, more } => ByteDifference {
            path: Some(value.path.clone()),
            recorded: Some(value.recorded.json.clone()),
            replayed: Some(value.replayed.json.clone()),
            more: *more,
            ..empty(DifferenceKind::Field)
        },
        FrameDifference::Header {
            field,
            recorded,
            replayed,
            more,
        } => ByteDifference {
            path: Some(field.name().to_string()),
            recorded: Some(recorded.json.clone()),
            replayed: Some(replayed.json.clone()),
            more: *more,
            ..empty(DifferenceKind::Header)
        },
        FrameDifference::SchemaHash { recorded, replayed } => ByteDifference {
            path: Some("schema_hash".to_string()),
            recorded: Some(DifferenceValue::Text(format!("0x{recorded:016x}"))),
            replayed: Some(DifferenceValue::Text(format!("0x{replayed:016x}"))),
            ..empty(DifferenceKind::SchemaHash)
        },
        FrameDifference::Layout {
            byte_offset,
            region,
        } => ByteDifference {
            path: match region {
                LayoutRegion::FieldFraming { path } => Some(path.clone()),
                _ => None,
            },
            byte_offset: Some(*byte_offset),
            region: Some(region.tag()),
            ..empty(DifferenceKind::Layout)
        },
        FrameDifference::Undecoded {
            byte_offset,
            reason,
        } => ByteDifference {
            byte_offset: Some(*byte_offset),
            reason: Some(reason.tag()),
            ..empty(DifferenceKind::Undecoded)
        },
    }
}

/// Decide what the first differing frame pair names (the order is in the
/// module docs).
///
/// `walker` is called at most once, and only when both frames hold a header
/// with the same `schema_hash`. `first_diff_byte_offset` is the diff's own
/// first differing byte (under a tolerance, outside the tolerated spans).
/// `skip` names the top-level fields a tolerance metric judges: they are
/// never compared here. `refusal` is the tolerance decoder's refusal of
/// either frame, reported when nothing decoded here differs.
pub(crate) fn locate_difference<'w>(
    walker: impl FnOnce() -> &'w FrameWalker,
    recorded: &[u8],
    replayed: &[u8],
    first_diff_byte_offset: usize,
    skip: &[&str],
    refusal: Option<&RemainderRefusal>,
) -> FrameDifference {
    let undecoded = |reason| FrameDifference::Undecoded {
        byte_offset: first_diff_byte_offset,
        reason,
    };
    let (Some(rec_header), Some(rep_header)) = (
        WireHeader::read_from_buf(recorded),
        WireHeader::read_from_buf(replayed),
    ) else {
        let (side, len) = if recorded.len() < WireHeader::SIZE {
            (Side::Recorded, recorded.len())
        } else {
            (Side::Replayed, replayed.len())
        };
        return undecoded(UndecodedReason::FrameTooShort { side, len });
    };
    if rec_header.schema_hash != rep_header.schema_hash {
        return FrameDifference::SchemaHash {
            recorded: rec_header.schema_hash,
            replayed: rep_header.schema_hash,
        };
    }
    let walker = walker();
    let schema_hash = rec_header.schema_hash;
    let (Some(schema), Some(layout)) = (
        walker.schema_for_hash(schema_hash),
        walker.layout_for_hash(schema_hash),
    ) else {
        return undecoded(UndecodedReason::UnknownSchemaHash { schema_hash });
    };
    let schema_name = layout.qualified_name.as_str();
    let refused = |side: Side, error: String| {
        undecoded(UndecodedReason::Undecodable {
            side,
            schema: schema_name.to_string(),
            error,
        })
    };
    let rec_value = match walker.walk_by_hash_keeping_raw_bools(recorded, usize::MAX) {
        Ok(v) => v,
        Err(e) => return refused(Side::Recorded, e.to_string()),
    };
    let rep_value = match walker.walk_by_hash_keeping_raw_bools(replayed, usize::MAX) {
        Ok(v) => v,
        Err(e) => return refused(Side::Replayed, e.to_string()),
    };
    let headers: Vec<HeaderField> = HeaderField::COMPARED
        .into_iter()
        .filter(|f| f.differs(&rec_header, &rep_header))
        .collect();
    let values = match diff_frame_values(walker, schema, &rec_value, &rep_value, skip) {
        Ok(values) => values,
        Err(error) => return refused(Side::Recorded, error),
    };
    if let Some(&field) = headers.first() {
        return FrameDifference::Header {
            field,
            recorded: field.shown(&rec_header),
            replayed: field.shown(&rep_header),
            more: (headers.len() as u64 - 1) + values.count(),
        };
    }
    if let Some(value) = values.first {
        return FrameDifference::Field {
            value,
            more: values.more,
        };
    }
    if let Some(r) = refusal {
        return refused(r.side, r.error.clone());
    }
    let offset = if HEADER_LENGTH_DERIVED_BYTES.contains(&first_diff_byte_offset) {
        first_diff_outside_total_size(recorded, replayed)
    } else {
        Some(first_diff_byte_offset)
    };
    match offset {
        Some(offset) if offset >= WireHeader::SIZE => {
            // The byte exists in the longer frame at least.
            let frame = if offset < recorded.len() {
                recorded
            } else {
                replayed
            };
            match classify_offset(walker, layout, frame, offset - WireHeader::SIZE) {
                Spot::Region(region) => FrameDifference::Layout {
                    byte_offset: offset,
                    region,
                },
                Spot::PastNul { path, text, k } => FrameDifference::Field {
                    value: ValueDifference {
                        path,
                        at: String::new(),
                        recorded: text.clone(),
                        replayed: text,
                        note: format!(" (bytes differ past the NUL at field byte {k})"),
                    },
                    more: 0,
                },
            }
        }
        // Only `total_size` differs (every other header byte was compared
        // above), so it is what the line names.
        _ => FrameDifference::Header {
            field: HeaderField::TotalSize,
            recorded: HeaderField::TotalSize.shown(&rec_header),
            replayed: HeaderField::TotalSize.shown(&rep_header),
            more: 0,
        },
    }
}

/// The first differing byte outside `total_size`, or the shorter length when
/// one frame is a prefix of the other there.
fn first_diff_outside_total_size(recorded: &[u8], replayed: &[u8]) -> Option<usize> {
    let common = recorded.len().min(replayed.len());
    (0..common)
        .filter(|i| !HEADER_LENGTH_DERIVED_BYTES.contains(i))
        .find(|&i| recorded[i] != replayed[i])
        .or_else(|| (recorded.len() != replayed.len()).then_some(common))
}

/// Where a differing byte of equal-valued frames sits.
enum Spot {
    Region(LayoutRegion),
    /// Inside a `string_fixed[n]` past its NUL: the text is equal.
    PastNul {
        path: String,
        text: Shown,
        k: usize,
    },
}

/// Classify payload byte `p` of `frame` against its layout.
fn classify_offset(walker: &FrameWalker, layout: &WireLayout, frame: &[u8], p: usize) -> Spot {
    let payload = &frame[WireHeader::SIZE..];
    if p < layout.fixed_size {
        let schema = walker.schema_for_hash(layout.schema_hash);
        return classify_fixed(
            walker,
            layout,
            schema.and_then(|s| s.package.as_deref()),
            payload,
            p,
            String::new(),
        );
    }
    let table_end = layout.fixed_size + layout.offset_table_bytes();
    if p < table_end {
        return Spot::Region(LayoutRegion::OffsetTable);
    }
    let mut last_end = table_end;
    for (i, field) in layout.variable_fields.iter().enumerate() {
        let at = layout.fixed_size + 8 * i;
        let Some(entry) = payload.get(at..at + 8) else {
            continue;
        };
        let offset = u32::from_le_bytes([entry[0], entry[1], entry[2], entry[3]]) as usize;
        let length = u32::from_le_bytes([entry[4], entry[5], entry[6], entry[7]]) as usize;
        let end = offset.saturating_add(length);
        if (offset..end).contains(&p) {
            return Spot::Region(LayoutRegion::FieldFraming {
                path: capped_path(&field.name),
            });
        }
        last_end = last_end.max(end);
    }
    Spot::Region(if p >= last_end {
        LayoutRegion::TrailingBytes
    } else {
        LayoutRegion::UnusedVariableBytes
    })
}

/// Classify fixed-section byte `p` (relative to the start of `layout`'s
/// fixed section in `bytes`), walking into inlined nested messages.
fn classify_fixed(
    walker: &FrameWalker,
    layout: &WireLayout,
    package: Option<&str>,
    bytes: &[u8],
    p: usize,
    path: String,
) -> Spot {
    let padding = Spot::Region(LayoutRegion::FixedSectionPadding);
    let Some(field) = layout
        .fixed_fields
        .iter()
        .find(|f| (f.offset..f.offset + f.size).contains(&p))
    else {
        return padding;
    };
    let mut path = path;
    push_field(&mut path, &field.name);
    let inner = p - field.offset;
    let base = bytes.get(field.offset..).unwrap_or(&[]);
    match &field.field_type {
        FieldType::StringFixed(n) => past_nul(base, *n, inner, path),
        FieldType::Nested {
            schema_name,
            package: target_package,
            ..
        } => match walker.nested_target(schema_name, target_package, package) {
            Some((nested, target)) => {
                classify_fixed(walker, nested, target.package.as_deref(), base, inner, path)
            }
            None => padding,
        },
        FieldType::FixedArray { element_type, .. } => match element_type.as_ref() {
            FieldType::StringFixed(n) if *n > 0 => {
                push_index(&mut path, inner / n);
                past_nul(
                    base.get((inner / n) * n..).unwrap_or(&[]),
                    *n,
                    inner % n,
                    path,
                )
            }
            FieldType::Nested {
                schema_name,
                package: target_package,
                ..
            } => match walker.nested_target(schema_name, target_package, package) {
                Some((nested, target)) if nested.fixed_size > 0 => {
                    let index = inner / nested.fixed_size;
                    push_index(&mut path, index);
                    classify_fixed(
                        walker,
                        nested,
                        target.package.as_deref(),
                        base.get(index * nested.fixed_size..).unwrap_or(&[]),
                        inner % nested.fixed_size,
                        path,
                    )
                }
                _ => padding,
            },
            _ => Spot::Region(LayoutRegion::FieldFraming {
                path: capped_path(&path),
            }),
        },
        _ => Spot::Region(LayoutRegion::FieldFraming {
            path: capped_path(&path),
        }),
    }
}

/// The `string_fixed[n]` at the start of `bytes`, whose byte `k` differs
/// although its text (up to the first NUL) is equal.
fn past_nul(bytes: &[u8], n: usize, k: usize, path: String) -> Spot {
    let field = &bytes[..n.min(bytes.len())];
    let end = field.iter().position(|&b| b == 0).unwrap_or(field.len());
    let text = match std::str::from_utf8(&field[..end]) {
        Ok(s) => whole_string(s),
        Err(_) => Shown::text(format!("not valid UTF-8 ({end} B)")),
    };
    Spot::PastNul {
        path: capped_path(&path),
        text,
        k,
    }
}

/// Compare two decoded frames of `schema` value by value, in declaration
/// order, leaving out the top-level fields in `skip`.
///
/// `Err` carries the walker's refusal of a nested reference it could not
/// resolve (the walk of the same frames would have refused first).
pub(crate) fn diff_frame_values(
    walker: &FrameWalker,
    schema: &MessageSchema,
    recorded: &FrameValue<'_>,
    replayed: &FrameValue<'_>,
    skip: &[&str],
) -> Result<FrameValueDiff, String> {
    let mut diff = Differ {
        walker,
        path: String::new(),
        found: FrameValueDiff::default(),
    };
    diff.message(schema, Some(recorded), Some(replayed), skip)?;
    Ok(diff.found)
}

/// One scalar value in its own type.
#[derive(Debug, Clone, Copy)]
enum Scalar<'a> {
    Bool(bool),
    /// A `bool` byte other than 0 or 1: true to every reader, and with no
    /// YAML form.
    BoolByte(u8),
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
    Str(&'a str),
    /// A string field whose bytes are not UTF-8.
    NotUtf8(&'a [u8]),
}

impl Scalar<'_> {
    /// Equal on the wire.
    fn same(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Bool(a), Self::Bool(b)) => a == b,
            (Self::BoolByte(a), Self::BoolByte(b)) => a == b,
            (Self::I8(a), Self::I8(b)) => a == b,
            (Self::U8(a), Self::U8(b)) => a == b,
            (Self::I16(a), Self::I16(b)) => a == b,
            (Self::U16(a), Self::U16(b)) => a == b,
            (Self::I32(a), Self::I32(b)) => a == b,
            (Self::U32(a), Self::U32(b)) => a == b,
            (Self::I64(a), Self::I64(b)) => a == b,
            (Self::U64(a), Self::U64(b)) => a == b,
            (Self::F32(a), Self::F32(b)) => a.to_bits() == b.to_bits(),
            (Self::F64(a), Self::F64(b)) => a.to_bits() == b.to_bits(),
            (Self::Str(a), Self::Str(b)) => a == b,
            (Self::NotUtf8(a), Self::NotUtf8(b)) => a == b,
            _ => false,
        }
    }

    fn shown(&self) -> Shown {
        let finite_f64 = |v: f64| {
            if v.is_finite() {
                DifferenceValue::Float64(v)
            } else {
                DifferenceValue::Text(yaml_format_f64(v))
            }
        };
        let (text, json) = match *self {
            Self::Bool(b) => (b.to_string(), DifferenceValue::Bool(b)),
            Self::BoolByte(_) => ("true".to_string(), DifferenceValue::Bool(true)),
            Self::I8(v) => (v.to_string(), DifferenceValue::Int(i64::from(v))),
            Self::U8(v) => (v.to_string(), DifferenceValue::UInt(u64::from(v))),
            Self::I16(v) => (v.to_string(), DifferenceValue::Int(i64::from(v))),
            Self::U16(v) => (v.to_string(), DifferenceValue::UInt(u64::from(v))),
            Self::I32(v) => (v.to_string(), DifferenceValue::Int(i64::from(v))),
            Self::U32(v) => (v.to_string(), DifferenceValue::UInt(u64::from(v))),
            Self::I64(v) => (v.to_string(), DifferenceValue::Int(v)),
            Self::U64(v) => (v.to_string(), DifferenceValue::UInt(v)),
            Self::F32(v) => (
                yaml_format_f32(v),
                if v.is_finite() {
                    DifferenceValue::Float32(v)
                } else {
                    DifferenceValue::Text(yaml_format_f32(v))
                },
            ),
            Self::F64(v) => (yaml_format_f64(v), finite_f64(v)),
            Self::Str(s) => return whole_string(s),
            Self::NotUtf8(b) => return Shown::text(format!("not valid UTF-8 ({} B)", b.len())),
        };
        Shown { text, json }
    }

    /// The wire bits, for two values whose text is the same.
    fn bits(&self) -> Option<String> {
        match *self {
            Self::Bool(b) => Some(format!("0x{:02x}", u8::from(b))),
            Self::BoolByte(b) => Some(format!("0x{b:02x}")),
            Self::F32(v) => Some(format!("0x{:08x}", v.to_bits())),
            Self::F64(v) => Some(format!("0x{:016x}", v.to_bits())),
            _ => None,
        }
    }
}

/// A string printed whole: double quoted with the YAML escapes.
fn whole_string(s: &str) -> Shown {
    Shown {
        text: quoted(s),
        json: DifferenceValue::Text(s.to_string()),
    }
}

fn quoted(s: &str) -> String {
    yaml_quote_within(s, usize::MAX).expect("no limit")
}

/// How one side of an array reads, element by element.
#[derive(Debug, Clone, Copy)]
enum ByteElem {
    U8,
    I8,
    Bool,
}

#[derive(Debug, Clone, Copy)]
enum Seq<'v, 'a> {
    /// One byte per element.
    Bytes(&'a [u8], ByteElem),
    Prim(PrimArray<'a>),
    /// `string[]`: each element `Str` or (not UTF-8) `Bytes`.
    Strings(&'v [FrameValueKind<'a>]),
    /// `string_fixed[n][m]`: back-to-back `n`-byte strings.
    FixedStrings(&'a [u8], usize),
    /// A zero-default array: `len` copies of one value.
    Zero(usize, Scalar<'static>),
}

impl<'a> Seq<'_, 'a> {
    fn len(&self) -> usize {
        match self {
            Self::Bytes(b, _) => b.len(),
            Self::Prim(a) => a.count,
            Self::Strings(e) => e.len(),
            Self::FixedStrings(raw, n) => raw.len().div_ceil(*n),
            Self::Zero(len, _) => *len,
        }
    }

    fn get(&self, i: usize) -> Scalar<'a> {
        match *self {
            Self::Bytes(b, ByteElem::U8) => Scalar::U8(b[i]),
            Self::Bytes(b, ByteElem::I8) => Scalar::I8(b[i] as i8),
            Self::Bytes(b, ByteElem::Bool) => match b[i] {
                0 => Scalar::Bool(false),
                1 => Scalar::Bool(true),
                other => Scalar::BoolByte(other),
            },
            Self::Prim(a) => prim_element(a, i),
            Self::Strings(e) => match &e[i] {
                FrameValueKind::Str(s) => Scalar::Str(s),
                FrameValueKind::Bytes(b) => Scalar::NotUtf8(b),
                _ => Scalar::NotUtf8(&[]),
            },
            Self::FixedStrings(raw, n) => {
                let chunk = &raw[i * n..((i + 1) * n).min(raw.len())];
                let end = chunk.iter().position(|&b| b == 0).unwrap_or(chunk.len());
                match std::str::from_utf8(&chunk[..end]) {
                    Ok(s) => Scalar::Str(s),
                    Err(_) => Scalar::NotUtf8(&chunk[..end]),
                }
            }
            Self::Zero(_, v) => v,
        }
    }
}

/// Element `i` of a little-endian primitive array, in its own type.
fn prim_element<'a>(a: PrimArray<'a>, i: usize) -> Scalar<'a> {
    let size = a.elem.size();
    let b = &a.bytes[i * size..(i + 1) * size];
    let a2 = || [b[0], b[1]];
    let a4 = || [b[0], b[1], b[2], b[3]];
    let a8 = || [b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]];
    match a.elem {
        PrimType::I16 => Scalar::I16(i16::from_le_bytes(a2())),
        PrimType::U16 => Scalar::U16(u16::from_le_bytes(a2())),
        PrimType::I32 => Scalar::I32(i32::from_le_bytes(a4())),
        PrimType::U32 => Scalar::U32(u32::from_le_bytes(a4())),
        PrimType::I64 => Scalar::I64(i64::from_le_bytes(a8())),
        PrimType::U64 => Scalar::U64(u64::from_le_bytes(a8())),
        PrimType::F32 => Scalar::F32(f32::from_le_bytes(a4())),
        PrimType::F64 => Scalar::F64(f64::from_le_bytes(a8())),
    }
}

/// The zero value of a scalar field type (a zero-default message), `None`
/// for a type that is not one scalar.
fn zero_scalar(ft: &FieldType) -> Option<Scalar<'static>> {
    Some(match ft {
        FieldType::Bool => Scalar::Bool(false),
        FieldType::I8 => Scalar::I8(0),
        FieldType::U8 => Scalar::U8(0),
        FieldType::I16 => Scalar::I16(0),
        FieldType::U16 => Scalar::U16(0),
        FieldType::I32 => Scalar::I32(0),
        FieldType::U32 => Scalar::U32(0),
        FieldType::I64 => Scalar::I64(0),
        FieldType::U64 => Scalar::U64(0),
        FieldType::F32 => Scalar::F32(0.0),
        FieldType::F64 => Scalar::F64(0.0),
        FieldType::String | FieldType::StringFixed(_) => Scalar::Str(""),
        _ => return None,
    })
}

fn prim_of(ft: &FieldType) -> Option<PrimType> {
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

/// A non-message field value, read for comparison.
enum Inline<'v, 'a> {
    Scalar(Scalar<'a>),
    Seq(Seq<'v, 'a>),
    /// Bytes with no element framing the comparison can read.
    Raw(&'a [u8]),
}

/// One side of an array of messages.
#[derive(Clone, Copy)]
enum Messages<'v, 'a> {
    /// `len` elements, `elements` decoded (empty for a zero-default array,
    /// whose elements are all the schema's zero value).
    Decoded {
        len: usize,
        elements: &'v [FrameValueKind<'a>],
        zero: bool,
    },
    /// Bytes not in the canonical element framing: compared as bytes.
    Raw,
}

/// A message with no bytes on the wire.
fn zero_size(layout: &WireLayout) -> bool {
    layout.is_fixed() && layout.fixed_size == 0
}

/// `path` + `.name` (no dot at the root): the path spelling of the YAML
/// codec's error text.
fn push_field(path: &mut String, name: &str) {
    if !path.is_empty() {
        path.push('.');
    }
    path.push_str(name);
}

/// `path` + `[index]`.
fn push_index(path: &mut String, index: usize) {
    let _ = write!(path, "[{index}]");
}

/// `path`, or its last [`MAX_PATH_BYTES`] bytes behind a leading `...`.
fn capped_path(path: &str) -> String {
    if path.len() <= MAX_PATH_BYTES {
        return path.to_string();
    }
    let mut cut = path.len() - MAX_PATH_BYTES;
    while !path.is_char_boundary(cut) {
        cut += 1;
    }
    format!("...{}", &path[cut..])
}

struct Differ<'w> {
    walker: &'w FrameWalker,
    /// The path of the value being compared.
    path: String,
    found: FrameValueDiff,
}

impl<'w> Differ<'w> {
    /// Count one differing value; `make` builds it only for the first.
    fn hit(&mut self, make: impl FnOnce() -> (String, Shown, Shown)) {
        if self.found.first.is_some() {
            self.found.more += 1;
            return;
        }
        let (at, recorded, replayed) = make();
        self.found.first = Some(ValueDifference {
            path: capped_path(&self.path),
            at,
            recorded,
            replayed,
            note: String::new(),
        });
    }

    /// Compare `schema`'s fields; `None` is a zero-default message (an empty
    /// variable nested blob, or one side of a zero-default array).
    fn message(
        &mut self,
        schema: &MessageSchema,
        recorded: Option<&FrameValue<'_>>,
        replayed: Option<&FrameValue<'_>>,
        skip: &[&str],
    ) -> Result<(), String> {
        let (recorded, replayed) = (present(schema, recorded), present(schema, replayed));
        if recorded.is_none() && replayed.is_none() {
            return Ok(());
        }
        let parent = schema.package.as_deref();
        for field in &schema.fields {
            if skip.contains(&field.name.as_str()) {
                continue;
            }
            let rec = recorded.and_then(|v| v.field(&field.name));
            let rep = replayed.and_then(|v| v.field(&field.name));
            let mark = self.path.len();
            push_field(&mut self.path, &field.name);
            self.value(&field.field_type, rec, rep, parent)?;
            self.path.truncate(mark);
        }
        Ok(())
    }

    fn value(
        &mut self,
        ft: &FieldType,
        recorded: Option<&FrameValueKind<'_>>,
        replayed: Option<&FrameValueKind<'_>>,
        parent: Option<&str>,
    ) -> Result<(), String> {
        match ft {
            FieldType::Nested {
                schema_name,
                package,
                ..
            } => {
                let (layout, target) = self.nested_target(schema_name, package, parent)?;
                if target.fields.is_empty() {
                    return Ok(());
                }
                match (
                    nested_value(layout, recorded),
                    nested_value(layout, replayed),
                ) {
                    (Ok(rec), Ok(rep)) => self.message(target, rec, rep, &[]),
                    _ => {
                        self.mismatched_kinds(recorded, replayed);
                        Ok(())
                    }
                }
            }
            FieldType::DynamicArray { element_type }
            | FieldType::FixedArray { element_type, .. }
                if matches!(element_type.as_ref(), FieldType::Nested { .. }) =>
            {
                self.message_array(ft, element_type, recorded, replayed, parent)
            }
            _ => {
                match (inline(ft, recorded), inline(ft, replayed)) {
                    (Some(Inline::Scalar(a)), Some(Inline::Scalar(b))) => {
                        if !a.same(&b) {
                            self.hit(|| scalar_pair(a, b));
                        }
                    }
                    (Some(Inline::Seq(a)), Some(Inline::Seq(b))) => self.seq(a, b),
                    (Some(Inline::Raw(a)), Some(Inline::Raw(b))) => self.raw(a, b),
                    // One side decoded to elements and the other did not,
                    // or the two decoded to different shapes: compared by
                    // their bytes where both have bytes.
                    _ => self.mismatched_kinds(recorded, replayed),
                }
                Ok(())
            }
        }
    }

    fn nested_target(
        &self,
        schema_name: &str,
        package: &Option<String>,
        parent: Option<&str>,
    ) -> Result<(&'w WireLayout, &'w MessageSchema), String> {
        self.walker
            .nested_target(schema_name, package, parent)
            .ok_or_else(|| {
                let target = match package {
                    Some(p) => format!("{p}/{schema_name}"),
                    None => schema_name.to_string(),
                };
                format!(
                    "nested field '{}' references unresolvable schema '{target}'",
                    self.path
                )
            })
    }

    /// Two values of different kinds: counted and named by their bytes
    /// where both have bytes, otherwise as unequal values.
    fn mismatched_kinds(
        &mut self,
        recorded: Option<&FrameValueKind<'_>>,
        replayed: Option<&FrameValueKind<'_>>,
    ) {
        if recorded == replayed {
            return;
        }
        match (raw_bytes(recorded), raw_bytes(replayed)) {
            (Some(a), Some(b)) => self.raw(a, b),
            _ => self.hit(|| {
                let describe = |k: Option<&FrameValueKind<'_>>| {
                    Shown::text(match k {
                        None => "the zero value".to_string(),
                        Some(_) => "a value of another shape".to_string(),
                    })
                };
                (String::new(), describe(recorded), describe(replayed))
            }),
        }
    }

    fn message_array(
        &mut self,
        ft: &FieldType,
        element_type: &FieldType,
        recorded: Option<&FrameValueKind<'_>>,
        replayed: Option<&FrameValueKind<'_>>,
        parent: Option<&str>,
    ) -> Result<(), String> {
        let FieldType::Nested {
            schema_name,
            package,
            ..
        } = element_type
        else {
            return Ok(());
        };
        let (layout, target) = self.nested_target(schema_name, package, parent)?;
        // A fixed array of zero-size messages carries no bytes.
        if matches!(ft, FieldType::FixedArray { .. }) && zero_size(layout) {
            return Ok(());
        }
        let (Some(rec), Some(rep)) = (messages(ft, recorded), messages(ft, replayed)) else {
            self.mismatched_kinds(recorded, replayed);
            return Ok(());
        };
        let (
            Messages::Decoded {
                len: rec_len,
                elements: rec_elems,
                zero: rec_zero,
            },
            Messages::Decoded {
                len: rep_len,
                elements: rep_elems,
                zero: rep_zero,
            },
        ) = (rec, rep)
        else {
            // An array not in the canonical element framing on either side
            // compares as its raw bytes.
            self.mismatched_kinds(recorded, replayed);
            return Ok(());
        };
        if rec_len != rep_len {
            self.hit(|| (String::new(), count_shown(rec_len), count_shown(rep_len)));
        }
        for i in 0..rec_len.min(rep_len) {
            let mark = self.path.len();
            push_index(&mut self.path, i);
            match (
                message_element(rec_elems, rec_zero, i),
                message_element(rep_elems, rep_zero, i),
            ) {
                (Ok(a), Ok(b)) => self.message(target, a, b, &[])?,
                _ => self.mismatched_kinds(rec_elems.get(i), rep_elems.get(i)),
            }
            self.path.truncate(mark);
        }
        Ok(())
    }

    fn seq(&mut self, recorded: Seq<'_, '_>, replayed: Seq<'_, '_>) {
        let (rec_len, rep_len) = (recorded.len(), replayed.len());
        if rec_len != rep_len {
            self.hit(|| (String::new(), count_shown(rec_len), count_shown(rep_len)));
        }
        for i in 0..rec_len.min(rep_len) {
            let (a, b) = (recorded.get(i), replayed.get(i));
            if !a.same(&b) {
                let mark = self.path.len();
                push_index(&mut self.path, i);
                self.hit(|| scalar_pair(a, b));
                self.path.truncate(mark);
            }
        }
    }

    /// Bytes compared as bytes: one differing value however many bytes
    /// differ.
    fn raw(&mut self, recorded: &[u8], replayed: &[u8]) {
        if recorded.len() != replayed.len() {
            let (a, b) = (recorded.len(), replayed.len());
            self.hit(|| {
                (
                    String::new(),
                    Shown::text(format!("{a} opaque byte(s)")),
                    Shown::text(format!("{b} opaque byte(s)")),
                )
            });
        } else if let Some(k) = recorded.iter().zip(replayed).position(|(a, b)| a != b) {
            let (a, b) = (recorded[k], replayed[k]);
            self.hit(|| {
                (
                    format!(", byte {k} of its opaque bytes"),
                    Shown::text(format!("0x{a:02x}")),
                    Shown::text(format!("0x{b:02x}")),
                )
            });
        }
    }
}

/// `{n} element(s)`.
fn count_shown(n: usize) -> Shown {
    Shown::text(format!("{n} element(s)"))
}

/// `value` unless it is an empty variable nested blob, which decodes to a
/// value with no fields: the zero default, as the YAML encoder reads it.
fn present<'v, 'a>(
    schema: &MessageSchema,
    value: Option<&'v FrameValue<'a>>,
) -> Option<&'v FrameValue<'a>> {
    value.filter(|v| !v.fields.is_empty() || schema.fields.is_empty())
}

/// A nested field's message, `None` for the zero default (no value, or a
/// message with no bytes). `Err` for a value that is not a message.
fn nested_value<'v, 'a>(
    layout: &WireLayout,
    kind: Option<&'v FrameValueKind<'a>>,
) -> Result<Option<&'v FrameValue<'a>>, ()> {
    match kind {
        Some(FrameValueKind::Nested(v)) if !zero_size(layout) => Ok(Some(v.as_ref())),
        Some(FrameValueKind::Nested(_)) | None => Ok(None),
        Some(_) => Err(()),
    }
}

/// One side of an array of messages, `None` for a value that is not one.
fn messages<'v, 'a>(
    ft: &FieldType,
    kind: Option<&'v FrameValueKind<'a>>,
) -> Option<Messages<'v, 'a>> {
    Some(match (kind, ft) {
        (None, FieldType::FixedArray { length, .. }) => Messages::Decoded {
            len: *length,
            elements: &[],
            zero: true,
        },
        (None, _) => Messages::Decoded {
            len: 0,
            elements: &[],
            zero: true,
        },
        (Some(FrameValueKind::Array(e)), FieldType::FixedArray { length, .. }) => {
            Messages::Decoded {
                len: *length,
                elements: e,
                zero: false,
            }
        }
        (Some(FrameValueKind::NestedArray { elements, .. }), _) => Messages::Decoded {
            len: elements.len(),
            elements,
            zero: false,
        },
        (Some(FrameValueKind::NestedArrayOpaque(_)), _) => Messages::Raw,
        (Some(_), _) => return None,
    })
}

/// Element `i` of one side of an array of messages; `None` for a
/// zero-default element. `Err` for an element that is not a message.
fn message_element<'v, 'a>(
    elements: &'v [FrameValueKind<'a>],
    zero: bool,
    i: usize,
) -> Result<Option<&'v FrameValue<'a>>, ()> {
    if zero {
        return Ok(None);
    }
    match elements.get(i) {
        Some(FrameValueKind::Nested(v)) => Ok(Some(v.as_ref())),
        _ => Err(()),
    }
}

/// The bytes behind a value, for comparing two values of different kinds.
fn raw_bytes<'a>(kind: Option<&FrameValueKind<'a>>) -> Option<&'a [u8]> {
    match kind? {
        FrameValueKind::Bytes(b) | FrameValueKind::NestedArrayOpaque(b) => Some(b),
        FrameValueKind::NestedArray { raw, .. } => Some(raw),
        FrameValueKind::PrimArray(a) => Some(a.bytes),
        FrameValueKind::Str(s) => Some(s.as_bytes()),
        _ => None,
    }
}

/// The two texts of a differing scalar pair, and where inside the value
/// they start. A pair whose text is the same gets each side's wire bits; a
/// long string pair prints a window around its first difference.
fn scalar_pair(recorded: Scalar<'_>, replayed: Scalar<'_>) -> (String, Shown, Shown) {
    match (recorded, replayed) {
        (Scalar::Str(a), Scalar::Str(b)) => return string_pair(a, b),
        (Scalar::NotUtf8(a), Scalar::NotUtf8(b)) if a.len() == b.len() => {
            let k = a.iter().zip(b).position(|(x, y)| x != y).unwrap_or(0);
            return (
                format!(", byte {k} of its bytes"),
                Shown::text(format!("0x{:02x}", a[k])),
                Shown::text(format!("0x{:02x}", b[k])),
            );
        }
        _ => {}
    }
    let (mut a, mut b) = (recorded.shown(), replayed.shown());
    if a.text == b.text {
        if let (Some(x), Some(y)) = (recorded.bits(), replayed.bits()) {
            a = Shown::text(format!("{} ({x})", a.text));
            b = Shown::text(format!("{} ({y})", b.text));
        }
    }
    (String::new(), a, b)
}

/// Two differing strings: whole when both are at most [`STRING_WINDOW`]
/// characters, otherwise a window of each starting [`STRING_LEAD`]
/// characters before the first differing character.
fn string_pair(recorded: &str, replayed: &str) -> (String, Shown, Shown) {
    let (rec_chars, rep_chars) = (recorded.chars().count(), replayed.chars().count());
    if rec_chars <= STRING_WINDOW && rep_chars <= STRING_WINDOW {
        return (
            String::new(),
            whole_string(recorded),
            whole_string(replayed),
        );
    }
    let first = recorded
        .chars()
        .zip(replayed.chars())
        .position(|(a, b)| a != b)
        .unwrap_or(rec_chars.min(rep_chars));
    let start = first.saturating_sub(STRING_LEAD);
    let window = |s: &str, len: usize| {
        let body: String = s.chars().skip(start).take(STRING_WINDOW).collect();
        Shown::text(format!(
            "{}{}{} ({len} chars)",
            if start > 0 { "..." } else { "" },
            quoted(&body),
            if start + STRING_WINDOW < len {
                "..."
            } else {
                ""
            }
        ))
    };
    (
        format!(" from char {first}"),
        window(recorded, rec_chars),
        window(replayed, rep_chars),
    )
}

/// A non-message field value as the comparison reads it, mirroring the
/// shapes the YAML encoder prints. `None` input is the zero default.
fn inline<'v, 'a>(ft: &FieldType, kind: Option<&'v FrameValueKind<'a>>) -> Option<Inline<'v, 'a>> {
    type K<'a> = FrameValueKind<'a>;
    let Some(kind) = kind else {
        return Some(match ft {
            FieldType::Bytes | FieldType::DynamicArray { .. } => {
                Inline::Seq(Seq::Bytes(&[], ByteElem::U8))
            }
            FieldType::FixedArray {
                element_type,
                length,
            } => Inline::Seq(Seq::Zero(*length, zero_scalar(element_type)?)),
            other => Inline::Scalar(zero_scalar(other)?),
        });
    };
    Some(match (ft, kind) {
        (FieldType::Bool, K::Bool(v)) => Inline::Scalar(Scalar::Bool(*v)),
        (FieldType::Bool, K::U8(b)) => Inline::Scalar(Scalar::BoolByte(*b)),
        (FieldType::I8, K::I8(v)) => Inline::Scalar(Scalar::I8(*v)),
        (FieldType::U8, K::U8(v)) => Inline::Scalar(Scalar::U8(*v)),
        (FieldType::I16, K::I16(v)) => Inline::Scalar(Scalar::I16(*v)),
        (FieldType::U16, K::U16(v)) => Inline::Scalar(Scalar::U16(*v)),
        (FieldType::I32, K::I32(v)) => Inline::Scalar(Scalar::I32(*v)),
        (FieldType::U32, K::U32(v)) => Inline::Scalar(Scalar::U32(*v)),
        (FieldType::I64, K::I64(v)) => Inline::Scalar(Scalar::I64(*v)),
        (FieldType::U64, K::U64(v)) => Inline::Scalar(Scalar::U64(*v)),
        (FieldType::F32, K::F32(v)) => Inline::Scalar(Scalar::F32(*v)),
        (FieldType::F64, K::F64(v)) => Inline::Scalar(Scalar::F64(*v)),
        (FieldType::String | FieldType::StringFixed(_), K::Str(s)) => {
            Inline::Scalar(Scalar::Str(s))
        }
        (FieldType::String | FieldType::StringFixed(_), K::Bytes(b)) => {
            Inline::Scalar(Scalar::NotUtf8(b))
        }
        (FieldType::Bytes, K::Bytes(b)) => Inline::Seq(Seq::Bytes(b, ByteElem::U8)),
        (
            FieldType::DynamicArray { element_type } | FieldType::FixedArray { element_type, .. },
            _,
        ) => scalar_array(ft, element_type, kind)?,
        _ => Inline::Raw(raw_bytes(Some(kind))?),
    })
}

fn scalar_array<'v, 'a>(
    ft: &FieldType,
    element_type: &FieldType,
    kind: &'v FrameValueKind<'a>,
) -> Option<Inline<'v, 'a>> {
    type K<'a> = FrameValueKind<'a>;
    Some(match (element_type, kind) {
        (FieldType::U8, K::Bytes(b)) => Inline::Seq(Seq::Bytes(b, ByteElem::U8)),
        (FieldType::I8, K::Bytes(b)) => Inline::Seq(Seq::Bytes(b, ByteElem::I8)),
        (FieldType::Bool, K::Bytes(b) | K::NestedArrayOpaque(b)) => {
            Inline::Seq(Seq::Bytes(b, ByteElem::Bool))
        }
        (FieldType::Bool, K::NestedArray { elements, raw })
            if elements.is_empty() && raw.is_empty() =>
        {
            Inline::Seq(Seq::Bytes(&[], ByteElem::Bool))
        }
        (_, K::PrimArray(a)) if prim_of(element_type) == Some(a.elem) => {
            if a.count.checked_mul(a.elem.size()) == Some(a.bytes.len()) {
                Inline::Seq(Seq::Prim(*a))
            } else {
                Inline::Raw(a.bytes)
            }
        }
        (FieldType::String, K::NestedArray { elements, .. }) => Inline::Seq(Seq::Strings(elements)),
        (FieldType::StringFixed(0), K::NestedArrayOpaque(_)) => match ft {
            FieldType::FixedArray { length, .. } => {
                Inline::Seq(Seq::Zero(*length, Scalar::Str("")))
            }
            _ => Inline::Raw(&[]),
        },
        (FieldType::StringFixed(n), K::NestedArrayOpaque(raw))
            if matches!(ft, FieldType::FixedArray { .. }) =>
        {
            Inline::Seq(Seq::FixedStrings(raw, *n))
        }
        _ => Inline::Raw(raw_bytes(Some(kind))?),
    })
}

#[cfg(test)]
mod tests;
