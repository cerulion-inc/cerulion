// SPDX-License-Identifier: AGPL-3.0-only
//! [`FrameEncoder`] / [`FrameCursor`]: schema-driven frame assembly into a
//! caller-provided buffer.

use super::view::validate_layout;
use super::DynamicError;
use crate::codegen::element_codec::variable_payload_align;
use crate::codegen::layout::WireLayout;
use crate::shm_runtime::{read_offset_entry, write_offset_entry};
use crate::wire::WireHeader;

/// Encodes wire frames for one [`WireLayout`] into caller-provided bytes.
///
/// The output is byte-identical to what the generated `<Name>Shm` writer +
/// `OutputProxy` path produces for the same field values (pinned by
/// `dynamic_encoder_identity_test`):
///
/// - `[0, 32)` - [`WireHeader`]: `schema_hash`, `total_size`,
///   `offset_table_offset = 32 + fixed_size`, `offset_table_count`,
///   `sequence` (0 unless [`FrameCursor::set_sequence`]), `timestamp_ns`.
/// - `[32, 32 + fixed_size)` - the fixed section; `#[repr(C)]` padding is
///   zero.
/// - the offset table - one little-endian `(u32 offset, u32 length)` pair per
///   variable field, in declaration order; `offset` is payload-relative
///   (from byte 32).
/// - variable payloads in declaration order, each placed at the running
///   cursor aligned up to [`variable_payload_align`] of its type (2/4/8 for
///   primitive arrays, 1 otherwise), alignment padding zero.
///
/// `new`, `required_len`, `begin` and every [`FrameCursor`] method perform
/// NO heap allocation on the success path, so a binding can build an encoder
/// per publish and write straight into a transport loan.
#[derive(Debug, Clone, Copy)]
pub struct FrameEncoder<'l> {
    layout: &'l WireLayout,
}

/// One frame under construction. Obtained from [`FrameEncoder::begin`];
/// the header, offset table and all padding are already written, so a
/// caller that writes no field still holds a well-formed frame with zeroed
/// values.
#[derive(Debug)]
pub struct FrameCursor<'l, 'b> {
    layout: &'l WireLayout,
    /// Exactly `total_size` bytes.
    frame: &'b mut [u8],
}

impl<'l> FrameEncoder<'l> {
    /// Prepare an encoder for `layout` (error values carry `String`s).
    ///
    /// # Errors
    ///
    /// Returns [`DynamicError::InvalidLayout`] when the supplied layout cannot
    /// be safely used by the unchecked cursor accessors.
    pub fn new(layout: &'l WireLayout) -> Result<Self, DynamicError> {
        validate_layout(layout)?;
        Ok(Self { layout })
    }

    /// The layout this encoder writes.
    pub fn layout(&self) -> &'l WireLayout {
        self.layout
    }

    /// Total frame length (header included) for variable-field byte lengths
    /// `var_lens` (one per `layout.variable_fields`, in order; empty for a
    /// fixed-only schema).
    ///
    /// Errors: [`DynamicError::VariableCountMismatch`],
    /// [`DynamicError::LengthNotElementMultiple`] (a typed array whose byte
    /// length is not whole elements), [`DynamicError::FrameTooLarge`]
    /// (`total_size` would not fit a `u32`).
    pub fn required_len(&self, var_lens: &[usize]) -> Result<usize, DynamicError> {
        self.place(var_lens, |_, _, _| {})
    }

    /// Start a frame in `buf`: zero `[0, total_size)`, write the header
    /// (`sequence = 0`) and the offset table, and return a cursor over the
    /// frame's fields. `buf` may be longer than the frame; bytes past
    /// `total_size` are untouched.
    ///
    /// Errors: those of [`required_len`](Self::required_len) plus
    /// [`DynamicError::BufferTooSmall`] and [`DynamicError::MisalignedBuffer`]
    /// (a nonempty primitive array would land at an address
    /// [`FrameView`](super::FrameView) refuses; checked before any write).
    pub fn begin<'b>(
        &self,
        buf: &'b mut [u8],
        var_lens: &[usize],
        timestamp_ns: u64,
    ) -> Result<FrameCursor<'l, 'b>, DynamicError> {
        let total = self.required_len(var_lens)?;
        let have = buf.len();
        let Some(frame) = buf.get_mut(..total) else {
            return Err(DynamicError::BufferTooSmall { need: total, have });
        };
        let base = frame.as_ptr() as usize + WireHeader::SIZE;
        let mut misaligned = None;
        self.place(var_lens, |idx, off, len| {
            let align = variable_payload_align(&self.layout.variable_fields[idx].field_type);
            if misaligned.is_none()
                && len > 0
                && align > 1
                && !base.wrapping_add(off as usize).is_multiple_of(align)
            {
                misaligned = Some((idx, off, align));
            }
        })?;
        if let Some((idx, offset, elem_size)) = misaligned {
            return Err(DynamicError::MisalignedBuffer {
                field: self.layout.variable_fields[idx].name.clone(),
                offset,
                elem_size,
            });
        }
        frame.fill(0);
        let fixed_size = self.layout.fixed_size;
        {
            let payload = &mut frame[WireHeader::SIZE..];
            // Placement was validated by `required_len` above; the closure
            // only records it.
            self.place(var_lens, |idx, off, len| {
                write_offset_entry(payload, fixed_size, idx, off, len);
            })?;
        }
        // `total` fits a u32 (checked in `place`); the prefix does too since
        // it is at most `total`.
        WireHeader {
            schema_hash: self.layout.schema_hash,
            total_size: total as u32,
            offset_table_offset: (WireHeader::SIZE + fixed_size) as u32,
            offset_table_count: self.layout.variable_fields.len() as u32,
            sequence: 0,
            timestamp_ns,
        }
        .write_to_buf(frame);
        Ok(FrameCursor {
            layout: self.layout,
            frame,
        })
    }

    /// Walk the variable-field placement, calling `emit(idx, offset, len)`
    /// per field (payload-relative `offset`), and return the total frame
    /// length. The single source of the placement rule so `required_len`
    /// and `begin` cannot drift.
    fn place(
        &self,
        var_lens: &[usize],
        mut emit: impl FnMut(usize, u32, u32),
    ) -> Result<usize, DynamicError> {
        let n = self.layout.variable_fields.len();
        if var_lens.len() != n {
            return Err(DynamicError::VariableCountMismatch {
                expected: n,
                got: var_lens.len(),
            });
        }
        const CEILING: usize = u32::MAX as usize;
        let mut cursor = self.layout.data_floor();
        for (idx, (&len, field)) in var_lens
            .iter()
            .zip(&self.layout.variable_fields)
            .enumerate()
        {
            // The payload alignment, which for a primitive array is also its
            // element size (1 for strings, bytes and nested/element arrays).
            let align = variable_payload_align(&field.field_type);
            if !len.is_multiple_of(align) {
                return Err(DynamicError::LengthNotElementMultiple {
                    field: self.layout.variable_fields[idx].name.clone(),
                    len,
                    elem_size: align,
                });
            }
            let aligned = cursor.checked_add(align - 1).map(|c| c & !(align - 1));
            let end = aligned.and_then(|a| a.checked_add(len));
            let needed = end.and_then(|e| e.checked_add(WireHeader::SIZE));
            match (aligned, end, needed) {
                (Some(aligned), Some(end), Some(needed)) if needed <= CEILING => {
                    emit(idx, aligned as u32, len as u32);
                    cursor = end;
                }
                _ => {
                    return Err(DynamicError::FrameTooLarge {
                        needed: needed.unwrap_or(usize::MAX),
                    });
                }
            }
        }
        let total = cursor
            .checked_add(WireHeader::SIZE)
            .filter(|t| *t <= CEILING)
            .ok_or(DynamicError::FrameTooLarge {
                needed: cursor.saturating_add(WireHeader::SIZE),
            })?;
        Ok(total)
    }
}

impl<'l, 'b> FrameCursor<'l, 'b> {
    /// The layout this frame follows.
    pub fn layout(&self) -> &'l WireLayout {
        self.layout
    }

    /// The whole fixed section (`fixed_size` bytes, zero-initialised).
    pub fn fixed_section_mut(&mut self) -> &mut [u8] {
        let fixed_size = self.layout.fixed_size;
        &mut self.frame[WireHeader::SIZE..WireHeader::SIZE + fixed_size]
    }

    /// The bytes of fixed-section field `name` (little-endian primitive,
    /// inline `string_fixed`, fixed array, or inlined fixed nested).
    pub fn fixed_field_mut(&mut self, name: &str) -> Result<&mut [u8], DynamicError> {
        let idx = self
            .layout
            .fixed_fields
            .iter()
            .position(|f| f.name == name)
            .ok_or_else(|| DynamicError::UnknownFixedField(name.to_string()))?;
        self.fixed_field_mut_at(idx)
    }

    /// [`fixed_field_mut`](Self::fixed_field_mut) by declaration index
    /// (`layout.fixed_fields[idx]`).
    pub fn fixed_field_mut_at(&mut self, idx: usize) -> Result<&mut [u8], DynamicError> {
        let fl = self
            .layout
            .fixed_fields
            .get(idx)
            .ok_or_else(|| DynamicError::UnknownFixedField(format!("#{idx}")))?;
        let start = WireHeader::SIZE + fl.offset;
        Ok(&mut self.frame[start..start + fl.size])
    }

    /// The payload bytes reserved for variable field `name` (exactly the
    /// length passed to [`FrameEncoder::begin`] for it).
    pub fn variable_field_mut(&mut self, name: &str) -> Result<&mut [u8], DynamicError> {
        let idx = self
            .layout
            .variable_fields
            .iter()
            .position(|f| f.name == name)
            .ok_or_else(|| DynamicError::UnknownVariableField(name.to_string()))?;
        self.variable_field_mut_at(idx)
    }

    /// [`variable_field_mut`](Self::variable_field_mut) by declaration index
    /// (`layout.variable_fields[idx]`, i.e. offset-table entry `idx`).
    pub fn variable_field_mut_at(&mut self, idx: usize) -> Result<&mut [u8], DynamicError> {
        if idx >= self.layout.variable_fields.len() {
            return Err(DynamicError::UnknownVariableField(format!("#{idx}")));
        }
        let payload = &mut self.frame[WireHeader::SIZE..];
        let (off, len) = read_offset_entry(payload, self.layout.fixed_size, idx);
        let (off, len) = (off as usize, len as usize);
        Ok(&mut payload[off..off + len])
    }

    /// Overwrite the header's `sequence` (a transport stamps it at commit;
    /// a binding replaying a recording may want the recorded value).
    pub fn set_sequence(&mut self, sequence: u32) {
        self.frame[20..24].copy_from_slice(&sequence.to_le_bytes());
    }

    /// The frame bytes written so far (`total_size` long).
    pub fn frame(&self) -> &[u8] {
        self.frame
    }

    /// Complete the frame; returns `total_size` (the number of bytes of the
    /// original buffer that hold the frame).
    pub fn finish(self) -> usize {
        self.frame.len()
    }
}
