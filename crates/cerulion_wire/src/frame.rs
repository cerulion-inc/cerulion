// SPDX-License-Identifier: MIT OR Apache-2.0
//! Structural, schema-free view over a Cerulion wire frame.
//!
//! A Cerulion frame lays out as:
//!
//! ```text
//! [WireHeader (32 B)][fixed section][offset table (8·N B)][variable payload]
//! ```
//!
//! Everything the STRUCTURAL layer needs is self-describing in the header:
//! the fixed section runs from the end of the header to the offset table,
//! whose position (`offset_table_offset`) and entry count
//! (`offset_table_count`) the producer stamps into the header, and each
//! `(offset, length)` table entry locates one variable field in the payload.
//! Interpreting those bytes AS typed fields (field names / element types from
//! a `.msg` schema) happens elsewhere — this reader hands back raw slices only.

use crate::error::WireError;
use crate::header::WireHeader;
use crate::offset::OffsetEntry;

/// A validated, borrowed view over a single Cerulion wire frame.
///
/// [`parse`](Self::parse) validates the header's self-consistency (total-size
/// bounds, offset-table bounds) up front, so every accessor below is
/// infallible EXCEPT [`variable_field`](Self::variable_field) (whose per-entry
/// offset/length are validated on access — a table entry can still point out
/// of bounds even when the table itself fits).
///
/// The view borrows the frame bytes; nothing is copied.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame<'a> {
    header: WireHeader,
    /// The frame bytes bounded to EXACTLY `header.total_size` (a caller-
    /// provided buffer may be larger — e.g. a shared-memory slot — so we
    /// bound to the declared frame at parse time).
    bytes: &'a [u8],
}

impl<'a> Frame<'a> {
    /// Parse and validate a frame from `buf`.
    ///
    /// `buf` must contain at least `total_size` bytes (it may be larger — the
    /// excess is ignored). Validation:
    /// - the buffer holds a full 32-byte header,
    /// - `total_size >= WireHeader::SIZE`,
    /// - the buffer is at least `total_size` bytes (not truncated),
    /// - when `offset_table_count > 0`, the offset table fits within the
    ///   frame and starts at or after the header.
    pub fn parse(buf: &'a [u8]) -> Result<Self, WireError> {
        let header = WireHeader::parse(buf)?;
        let total = header.total_size as usize;

        if total < WireHeader::SIZE {
            return Err(WireError::InvalidTotalSize {
                total_size: header.total_size,
                min: WireHeader::SIZE as u32,
            });
        }
        if buf.len() < total {
            return Err(WireError::TruncatedFrame {
                total_size: total,
                available: buf.len(),
            });
        }

        let count = header.offset_table_count as usize;
        if count > 0 {
            let table_off = header.offset_table_offset as usize;
            // `8 * count` cannot overflow usize on any 32/64-bit target
            // (count is a u32), but stay defensive against a hostile header.
            let table_bytes =
                count
                    .checked_mul(OffsetEntry::SIZE)
                    .ok_or(WireError::OffsetTableOutOfBounds {
                        offset: table_off,
                        count,
                        bytes: usize::MAX,
                        total_size: total,
                    })?;
            let table_end =
                table_off
                    .checked_add(table_bytes)
                    .ok_or(WireError::OffsetTableOutOfBounds {
                        offset: table_off,
                        count,
                        bytes: table_bytes,
                        total_size: total,
                    })?;
            // The table must sit after the header and within the frame.
            if table_off < WireHeader::SIZE || table_end > total {
                return Err(WireError::OffsetTableOutOfBounds {
                    offset: table_off,
                    count,
                    bytes: table_bytes,
                    total_size: total,
                });
            }
        }

        Ok(Self {
            header,
            bytes: &buf[..total],
        })
    }

    /// The parsed header.
    #[inline]
    pub fn header(&self) -> &WireHeader {
        &self.header
    }

    /// The whole frame, bounded to `total_size` (header included).
    #[inline]
    pub fn as_bytes(&self) -> &'a [u8] {
        self.bytes
    }

    /// The post-header payload: everything after the 32-byte header
    /// (`[fixed section][offset table][variable payload]`). Offsets carried
    /// in the offset table are relative to the FIRST byte of this slice.
    #[inline]
    pub fn payload(&self) -> &'a [u8] {
        &self.bytes[WireHeader::SIZE..]
    }

    /// The fixed section — the primitives / fixed-array bytes that precede the
    /// offset table.
    ///
    /// When the frame has variable fields (`offset_table_count > 0`) the fixed
    /// section runs `[WireHeader::SIZE, offset_table_offset)`. When there are
    /// no variable fields the whole payload is fixed — `offset_table_offset`
    /// carries no structural meaning (some producers stamp `0`), so the fixed
    /// section is the entire post-header payload.
    #[inline]
    pub fn fixed_section(&self) -> &'a [u8] {
        if self.header.offset_table_count == 0 {
            self.payload()
        } else {
            &self.bytes[WireHeader::SIZE..self.header.offset_table_offset as usize]
        }
    }

    /// Number of offset-table entries (== number of variable fields).
    #[inline]
    pub fn offset_entry_count(&self) -> usize {
        self.header.offset_table_count as usize
    }

    /// Read offset-table entry `idx`, or `None` if `idx` is out of range.
    ///
    /// The table bytes were validated to be in-bounds at [`parse`](Self::parse)
    /// time, so this read is infallible for `idx < offset_entry_count()`.
    #[inline]
    pub fn offset_entry(&self, idx: usize) -> Option<OffsetEntry> {
        if idx >= self.offset_entry_count() {
            return None;
        }
        let start = self.header.offset_table_offset as usize + idx * OffsetEntry::SIZE;
        // Bounds guaranteed by parse-time validation; `parse` proved
        // `table_off + 8*count <= total`.
        OffsetEntry::parse(&self.bytes[start..])
    }

    /// Iterate every offset-table entry in declaration order.
    pub fn offset_entries(&self) -> impl Iterator<Item = OffsetEntry> + '_ {
        (0..self.offset_entry_count()).map(move |i| {
            self.offset_entry(i)
                .expect("index < count is always a valid entry")
        })
    }

    /// Slice the bytes of variable field `idx` out of the payload.
    ///
    /// A zero-length entry (`(0, 0)` — an unwritten or genuinely empty field)
    /// yields an empty slice, never an error. A non-empty entry is validated
    /// against the frame: its region must lie at or after the variable-data
    /// floor (immediately after the offset table) and end within
    /// `total_size`; otherwise [`WireError::VariableFieldOutOfBounds`].
    ///
    /// Returns `None` if `idx >= offset_entry_count()`.
    pub fn variable_field(&self, idx: usize) -> Option<Result<&'a [u8], WireError>> {
        let entry = self.offset_entry(idx)?;
        Some(self.slice_variable(idx, entry))
    }

    fn slice_variable(&self, idx: usize, entry: OffsetEntry) -> Result<&'a [u8], WireError> {
        let len = entry.length as usize;
        if len == 0 {
            // An unwritten / genuinely empty field: the empty slice is the
            // correct value, not a corruption (mirrors the cerulion_core frame
            // walker's `read_offset_entry` (0, 0) semantics).
            return Ok(&[]);
        }
        let off = entry.offset as usize;
        let total = self.header.total_size as usize;
        // The variable-data floor: nothing valid points into the fixed section
        // or the offset table itself.
        let data_floor = self.header.offset_table_offset as usize
            + self.offset_entry_count() * OffsetEntry::SIZE;
        // The entry offset is payload-relative (from the first byte after the
        // header); convert to an absolute frame offset. `off` and `len` are
        // attacker-controlled `u32`s, so BOTH adds are checked — an offset
        // near `u32::MAX` (which overflows `usize` on a 32-bit target) is a
        // clean out-of-bounds error, never a panic (mirrors `cerulion_core`'s
        // `walk_payload`, and the sibling `end` add which was already checked).
        let start = WireHeader::SIZE.checked_add(off);
        let end = start.and_then(|s| s.checked_add(len));
        match (start, end) {
            (Some(start), Some(end)) if start >= data_floor && end <= total => {
                Ok(&self.bytes[start..end])
            }
            _ => Err(WireError::VariableFieldOutOfBounds {
                index: idx,
                offset: off,
                length: len,
                total_size: total,
            }),
        }
    }
}
