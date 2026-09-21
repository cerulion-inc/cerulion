// SPDX-License-Identifier: MIT OR Apache-2.0
//! The 32-byte Cerulion wire header.
//!
//! This is a BYTE-EXACT mirror of `cerulion_core::wire::WireHeader`. The
//! layout is pinned against drift by the lockstep test living inside
//! `cerulion_core` (`cerulion_core/tests/wire_lockstep_test.rs`), which
//! encodes headers with the real writer and re-parses them here — any change
//! to the source-of-truth layout fails that test loudly.

use crate::error::WireError;

/// Wire message header — a fixed 32 bytes, all fields little-endian.
///
/// Byte layout (offsets from the start of the frame):
///
/// | offset | field                | type |
/// |--------|----------------------|------|
/// | 0      | `schema_hash`        | u64  |
/// | 8      | `total_size`         | u32  |
/// | 12     | `offset_table_offset`| u32  |
/// | 16     | `offset_table_count` | u32  |
/// | 20     | `sequence`           | u32  |
/// | 24     | `timestamp_ns`       | u64  |
///
/// `offset_table_offset` is measured from the START of the frame (i.e. it
/// includes the 32-byte header). The production writer stamps it as
/// `WireHeader::SIZE + <fixed section size>` and `offset_table_count` as the
/// schema's variable-field count. When `offset_table_count == 0` the table is
/// empty and `offset_table_offset` carries no structural meaning (some
/// producers stamp `0`); see [`crate::Frame`].
#[repr(C, align(8))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WireHeader {
    /// Layout-sensitive schema hash for type validation. Opaque to
    /// this crate — it is the frame's type tag.
    pub schema_hash: u64,
    /// Total frame size in bytes, INCLUDING this 32-byte header.
    pub total_size: u32,
    /// Byte offset from the frame start to the offset table.
    pub offset_table_offset: u32,
    /// Number of offset-table entries (== number of variable-length fields).
    pub offset_table_count: u32,
    /// Message sequence number (publisher-stamped).
    pub sequence: u32,
    /// Timestamp in nanoseconds (publisher-stamped).
    pub timestamp_ns: u64,
}

impl WireHeader {
    /// Header size in bytes.
    pub const SIZE: usize = 32;

    /// Parse a header from the first [`SIZE`](Self::SIZE) bytes of `buf`,
    /// field-by-field little-endian (alignment-safe — `buf` need not be
    /// aligned).
    ///
    /// Returns [`WireError::FrameTooShort`] if `buf` is under 32 bytes.
    #[inline]
    pub fn parse(buf: &[u8]) -> Result<Self, WireError> {
        if buf.len() < Self::SIZE {
            return Err(WireError::FrameTooShort {
                have: buf.len(),
                need: Self::SIZE,
            });
        }
        // `try_into().unwrap()` on a slice of a statically-known length can
        // never fail — the slices below are all fixed-width sub-ranges of a
        // buffer already checked to be >= 32 bytes.
        Ok(Self {
            schema_hash: u64::from_le_bytes(buf[0..8].try_into().unwrap()),
            total_size: u32::from_le_bytes(buf[8..12].try_into().unwrap()),
            offset_table_offset: u32::from_le_bytes(buf[12..16].try_into().unwrap()),
            offset_table_count: u32::from_le_bytes(buf[16..20].try_into().unwrap()),
            sequence: u32::from_le_bytes(buf[20..24].try_into().unwrap()),
            timestamp_ns: u64::from_le_bytes(buf[24..32].try_into().unwrap()),
        })
    }

    /// Serialize this header into the first [`SIZE`](Self::SIZE) bytes of
    /// `buf`, field-by-field little-endian (the exact inverse of
    /// [`parse`](Self::parse)).
    ///
    /// Returns [`WireError::FrameTooShort`] if `buf` is under 32 bytes.
    #[inline]
    pub fn encode(&self, buf: &mut [u8]) -> Result<(), WireError> {
        if buf.len() < Self::SIZE {
            return Err(WireError::FrameTooShort {
                have: buf.len(),
                need: Self::SIZE,
            });
        }
        buf[0..8].copy_from_slice(&self.schema_hash.to_le_bytes());
        buf[8..12].copy_from_slice(&self.total_size.to_le_bytes());
        buf[12..16].copy_from_slice(&self.offset_table_offset.to_le_bytes());
        buf[16..20].copy_from_slice(&self.offset_table_count.to_le_bytes());
        buf[20..24].copy_from_slice(&self.sequence.to_le_bytes());
        buf[24..32].copy_from_slice(&self.timestamp_ns.to_le_bytes());
        Ok(())
    }

    /// Serialize this header to an owned 32-byte array.
    #[inline]
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        // Infallible: the buffer is exactly `SIZE` bytes.
        self.encode(&mut buf)
            .expect("a 32-byte buffer always fits a WireHeader");
        buf
    }
}
