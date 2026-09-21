// SPDX-License-Identifier: MIT OR Apache-2.0
//! Offset-table entry for variable-length fields.
//!
//! A byte-exact mirror of `cerulion_core::wire::OffsetEntry` (pinned against
//! drift by `cerulion_core`'s lockstep test).

use crate::error::WireError;

/// One offset-table entry: where a variable-length field's bytes live.
///
/// Both fields are little-endian `u32`. `offset` is measured from the start of
/// the POST-HEADER payload (the first byte after the 32-byte
/// [`crate::WireHeader`]), NOT from the frame start — the convention every
/// Cerulion producer stamps and every reader assumes.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OffsetEntry {
    /// Payload-relative byte offset of the field's data.
    pub offset: u32,
    /// Length of the field's data in bytes.
    pub length: u32,
}

impl OffsetEntry {
    /// Serialized size of one entry (two little-endian `u32`s).
    pub const SIZE: usize = 8;

    /// Construct an entry.
    #[inline]
    pub const fn new(offset: u32, length: u32) -> Self {
        Self { offset, length }
    }

    /// Parse an entry from the first [`SIZE`](Self::SIZE) bytes of `buf`.
    ///
    /// Returns `None` if `buf` is shorter than 8 bytes.
    #[inline]
    pub fn parse(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            offset: u32::from_le_bytes(buf[0..4].try_into().unwrap()),
            length: u32::from_le_bytes(buf[4..8].try_into().unwrap()),
        })
    }

    /// Serialize this entry into the first [`SIZE`](Self::SIZE) bytes of `buf`.
    ///
    /// Returns [`WireError::FrameTooShort`] if `buf` is shorter than 8 bytes.
    #[inline]
    pub fn encode(&self, buf: &mut [u8]) -> Result<(), WireError> {
        if buf.len() < Self::SIZE {
            return Err(WireError::FrameTooShort {
                have: buf.len(),
                need: Self::SIZE,
            });
        }
        buf[0..4].copy_from_slice(&self.offset.to_le_bytes());
        buf[4..8].copy_from_slice(&self.length.to_le_bytes());
        Ok(())
    }
}
