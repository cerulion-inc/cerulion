// SPDX-License-Identifier: AGPL-3.0-only
//! [`FrameView`]: validated, bounds-checked field access over one wire frame.

use super::DynamicError;
use crate::codegen::element_codec::variable_payload_align;
use crate::codegen::layout::WireLayout;
use crate::codegen::{FieldType, FrameValue, FrameWalker, PrimArray, PrimType};
use crate::shm_runtime::read_offset_entry;
use crate::wire::WireHeader;

/// A validated view over one wire frame, resolved by its `schema_hash`.
///
/// Construction ([`FrameView::new`]) validates the whole structure ONCE:
///
/// 1. `frame.len() >= 32` and the hash resolves to a layout;
/// 2. `total_size <= frame.len()` and `total_size >= 32 + data_floor`;
/// 3. the header's offset-table position/count match the layout;
/// 4. every NON-EMPTY offset-table entry lies in `[data_floor, payload_len]`,
///    no two non-empty entries overlap, and a primitive-array entry is
///    aligned to (and a whole multiple of) its element size. `(0,0)` and
///    zero-length entries are accepted as empty; typed-array entries must be
///    element-aligned both payload-relative and in memory, so `frame` must
///    itself be aligned to the widest element (8) - as a transport loan and
///    any heap allocation are.
///
/// Every accessor afterwards is a slice of the frame - no allocation, no
/// copy. Only the bytes inside `total_size` are ever read; the payload is
/// `frame[32..total_size]`.
///
/// A [`FrameWalker`] accepts anything in (4) except the floor/bounds rules
/// (out-of-order, overlapping and misaligned top-level entries are legal
/// wire for it); this view is stricter because a binding hands these slices
/// out as typed in-place arrays.
#[derive(Debug, Clone, Copy)]
pub struct FrameView<'l, 'a> {
    layout: &'l WireLayout,
    header: WireHeader,
    /// `frame[..total_size]`.
    frame: &'a [u8],
}

impl<'l, 'a> FrameView<'l, 'a> {
    /// Validate `frame` against the layout `walker` resolves its
    /// `schema_hash` to. `frame` is the whole wire frame (header first) as a
    /// subscriber's `payload()` returns it; trailing bytes past `total_size`
    /// are ignored.
    pub fn new(walker: &'l FrameWalker, frame: &'a [u8]) -> Result<Self, DynamicError> {
        let header = WireHeader::read_from_buf(frame).ok_or(DynamicError::FrameTooShort {
            have: frame.len(),
            need: WireHeader::SIZE,
        })?;
        let layout = walker
            .layout_for_hash(header.schema_hash)
            .ok_or(DynamicError::UnknownSchemaHash(header.schema_hash))?;
        Self::with_layout(layout, frame)
    }

    /// Like [`new`](Self::new) but against a layout the caller already
    /// resolved (e.g. cached per topic). The frame's `schema_hash` must
    /// equal `layout.schema_hash`.
    ///
    /// # Errors
    ///
    /// Returns [`DynamicError::InvalidLayout`] when the supplied layout cannot
    /// be safely used by the unchecked view accessors.
    pub fn with_layout(layout: &'l WireLayout, frame: &'a [u8]) -> Result<Self, DynamicError> {
        validate_layout(layout)?;
        let header = WireHeader::read_from_buf(frame).ok_or(DynamicError::FrameTooShort {
            have: frame.len(),
            need: WireHeader::SIZE,
        })?;
        if header.schema_hash != layout.schema_hash {
            return Err(DynamicError::SchemaHashMismatch {
                expected: layout.schema_hash,
                found: header.schema_hash,
            });
        }
        let total_size = header.total_size as usize;
        if total_size > frame.len() {
            return Err(DynamicError::TotalSizeExceedsBuffer {
                total_size: header.total_size,
                have: frame.len(),
            });
        }
        let data_floor = layout.data_floor();
        let need = WireHeader::SIZE + data_floor;
        if total_size < need {
            return Err(DynamicError::TotalSizeBelowPrefix {
                schema: layout.qualified_name.clone(),
                total_size: header.total_size,
                need,
            });
        }
        let n = layout.variable_fields.len();
        let expected_offset = (WireHeader::SIZE + layout.fixed_size) as u32;
        let expected_count = n as u32;
        if header.offset_table_offset != expected_offset
            || header.offset_table_count != expected_count
        {
            return Err(DynamicError::OffsetTableMismatch {
                expected_offset,
                offset: header.offset_table_offset,
                expected_count,
                count: header.offset_table_count,
            });
        }

        let frame = &frame[..total_size];
        let payload = &frame[WireHeader::SIZE..];
        let payload_len = payload.len();
        for (i, vf) in layout.variable_fields.iter().enumerate() {
            let (off, len) = read_offset_entry(payload, layout.fixed_size, i);
            if len == 0 {
                continue;
            }
            if (off as usize) < data_floor {
                return Err(DynamicError::OffsetBelowDataFloor {
                    field: vf.name.clone(),
                    offset: off,
                    data_floor,
                });
            }
            let Some(end) = (off as usize)
                .checked_add(len as usize)
                .filter(|end| *end <= payload_len)
            else {
                return Err(DynamicError::VariableFieldOutOfBounds {
                    field: vf.name.clone(),
                    offset: off,
                    length: len,
                    payload_len,
                });
            };
            let align = variable_payload_align(&vf.field_type);
            if !(off as usize).is_multiple_of(align) || !(len as usize).is_multiple_of(align) {
                return Err(DynamicError::MisalignedElements {
                    field: vf.name.clone(),
                    offset: off,
                    length: len,
                    elem_size: align,
                });
            }
            if align > 1
                && !(payload.as_ptr() as usize)
                    .wrapping_add(off as usize)
                    .is_multiple_of(align)
            {
                return Err(DynamicError::MisalignedBuffer {
                    field: vf.name.clone(),
                    offset: off,
                    elem_size: align,
                });
            }
            for (j, earlier) in layout.variable_fields.iter().enumerate().take(i) {
                let (eoff, elen) = read_offset_entry(payload, layout.fixed_size, j);
                if elen == 0 {
                    continue;
                }
                let eend = (eoff as usize).saturating_add(elen as usize);
                if (off as usize) < eend && (eoff as usize) < end {
                    return Err(DynamicError::OverlappingEntries {
                        first: earlier.name.clone(),
                        second: vf.name.clone(),
                    });
                }
            }
        }

        Ok(Self {
            layout,
            header,
            frame,
        })
    }

    /// The layout the frame was validated against.
    pub fn layout(&self) -> &'l WireLayout {
        self.layout
    }

    /// The parsed header.
    pub fn header(&self) -> &WireHeader {
        &self.header
    }

    /// `WireHeader::schema_hash`.
    pub fn schema_hash(&self) -> u64 {
        self.header.schema_hash
    }

    /// `WireHeader::timestamp_ns`.
    pub fn timestamp_ns(&self) -> u64 {
        self.header.timestamp_ns
    }

    /// `WireHeader::sequence`.
    pub fn sequence(&self) -> u32 {
        self.header.sequence
    }

    /// `WireHeader::total_size` - the frame's byte length.
    pub fn total_size(&self) -> u32 {
        self.header.total_size
    }

    /// The frame bytes, `total_size` long.
    pub fn frame(&self) -> &'a [u8] {
        self.frame
    }

    /// The payload (everything after the header): fixed section, offset
    /// table, variable payloads.
    pub fn payload(&self) -> &'a [u8] {
        &self.frame[WireHeader::SIZE..]
    }

    /// The fixed section (`fixed_size` bytes).
    pub fn fixed_section(&self) -> &'a [u8] {
        &self.payload()[..self.layout.fixed_size]
    }

    /// The bytes of fixed-section field `name`.
    pub fn fixed_field(&self, name: &str) -> Result<&'a [u8], DynamicError> {
        let idx = self
            .layout
            .fixed_fields
            .iter()
            .position(|f| f.name == name)
            .ok_or_else(|| DynamicError::UnknownFixedField(name.to_string()))?;
        self.fixed_field_at(idx)
    }

    /// [`fixed_field`](Self::fixed_field) by declaration index.
    pub fn fixed_field_at(&self, idx: usize) -> Result<&'a [u8], DynamicError> {
        let fl = self
            .layout
            .fixed_fields
            .get(idx)
            .ok_or_else(|| DynamicError::UnknownFixedField(format!("#{idx}")))?;
        Ok(&self.payload()[fl.offset..fl.offset + fl.size])
    }

    /// The payload bytes of variable field `name` (empty for an empty
    /// field).
    pub fn variable_field(&self, name: &str) -> Result<&'a [u8], DynamicError> {
        let idx = self.variable_index(name)?;
        self.variable_field_at(idx)
    }

    /// [`variable_field`](Self::variable_field) by declaration index
    /// (offset-table entry `idx`).
    pub fn variable_field_at(&self, idx: usize) -> Result<&'a [u8], DynamicError> {
        if idx >= self.layout.variable_fields.len() {
            return Err(DynamicError::UnknownVariableField(format!("#{idx}")));
        }
        let payload = self.payload();
        let (off, len) = read_offset_entry(payload, self.layout.fixed_size, idx);
        let (off, len) = (off as usize, len as usize);
        if len == 0 {
            return Ok(&[]);
        }
        Ok(&payload[off..off + len])
    }

    /// A `string` variable field as `&str`.
    ///
    /// Errors: [`DynamicError::NotAStringField`],
    /// [`DynamicError::InvalidUtf8`]. (The [`FrameWalker`] instead degrades
    /// invalid UTF-8 to bytes; a binding needs the loud arm.)
    pub fn str_field(&self, name: &str) -> Result<&'a str, DynamicError> {
        let idx = self.variable_index(name)?;
        if self.layout.variable_fields[idx].field_type != FieldType::String {
            return Err(DynamicError::NotAStringField(name.to_string()));
        }
        let bytes = self.variable_field_at(idx)?;
        std::str::from_utf8(bytes).map_err(|e| DynamicError::InvalidUtf8 {
            field: name.to_string(),
            valid_up_to: e.valid_up_to(),
        })
    }

    /// A primitive dynamic-array variable field (`int16[]` … `float64[]`)
    /// as a [`PrimArray`] - element-aligned in the frame (validated at
    /// construction), so a binding may view it in place as `&[T]`.
    ///
    /// Errors: [`DynamicError::NotAPrimitiveArrayField`] (including `uint8[]`
    /// / `int8[]` / `bool[]`, which are byte arrays - use
    /// [`variable_field`](Self::variable_field)).
    pub fn prim_array_field(&self, name: &str) -> Result<PrimArray<'a>, DynamicError> {
        let idx = self.variable_index(name)?;
        let elem = match &self.layout.variable_fields[idx].field_type {
            FieldType::DynamicArray { element_type } => match element_type.as_ref() {
                FieldType::I16 => PrimType::I16,
                FieldType::U16 => PrimType::U16,
                FieldType::I32 => PrimType::I32,
                FieldType::U32 => PrimType::U32,
                FieldType::I64 => PrimType::I64,
                FieldType::U64 => PrimType::U64,
                FieldType::F32 => PrimType::F32,
                FieldType::F64 => PrimType::F64,
                _ => return Err(DynamicError::NotAPrimitiveArrayField(name.to_string())),
            },
            _ => return Err(DynamicError::NotAPrimitiveArrayField(name.to_string())),
        };
        let bytes = self.variable_field_at(idx)?;
        Ok(PrimArray {
            elem,
            bytes,
            count: bytes.len() / elem.size(),
        })
    }

    /// Full typed decode through the [`FrameWalker`] (every
    /// [`FrameValueKind`](crate::codegen::FrameValueKind) helper: nested
    /// values, element arrays, ...). Allocates the value tree.
    pub fn decode(&self, walker: &FrameWalker) -> Result<FrameValue<'a>, DynamicError> {
        Ok(walker.walk_by_hash(self.frame)?)
    }

    fn variable_index(&self, name: &str) -> Result<usize, DynamicError> {
        self.layout
            .variable_fields
            .iter()
            .position(|f| f.name == name)
            .ok_or_else(|| DynamicError::UnknownVariableField(name.to_string()))
    }
}

/// Validate layout invariants required by unchecked field accessors.
///
/// Both [`FrameView::with_layout`] and [`FrameEncoder::new`] call this before
/// slicing or placing fields.
pub(super) fn validate_layout(layout: &WireLayout) -> Result<(), DynamicError> {
    if crate::wire::frame_prefix_exceeds_wire(layout.fixed_size, layout.variable_fields.len()) {
        return Err(DynamicError::InvalidLayout {
            schema: layout.qualified_name.clone(),
            detail: "frame prefix exceeds the u32 wire total_size",
        });
    }
    for field in &layout.fixed_fields {
        if field
            .offset
            .checked_add(field.size)
            .is_none_or(|end| end > layout.fixed_size)
        {
            return Err(DynamicError::InvalidLayout {
                schema: layout.qualified_name.clone(),
                detail: "a fixed field lies outside the fixed section",
            });
        }
    }
    Ok(())
}
