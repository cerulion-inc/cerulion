// SPDX-License-Identifier: MIT OR Apache-2.0
//! Structural decode failures for the Cerulion wire format.
//!
//! Every variant names the offending element with exact accounting. Like the
//! `cerulion_core` frame walker, this reader is an UNTRUSTED-INPUT parser: it
//! NEVER panics on adversarial bytes and NEVER fabricates a value — a
//! structurally impossible frame returns an [`Err`] instead.

/// A structural error decoding a Cerulion wire frame.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum WireError {
    /// The buffer is shorter than the 32-byte [`crate::WireHeader`].
    #[error("buffer too short: have {have} bytes, need at least {need} for the WireHeader")]
    FrameTooShort {
        /// Bytes actually present.
        have: usize,
        /// Bytes required (always [`crate::WireHeader::SIZE`]).
        need: usize,
    },

    /// The header's `total_size` is smaller than the header itself — a frame
    /// must be at least 32 bytes.
    #[error("invalid total_size {total_size}: must be >= {min} (the WireHeader size)")]
    InvalidTotalSize {
        /// The declared `total_size`.
        total_size: u32,
        /// The minimum legal value ([`crate::WireHeader::SIZE`]).
        min: u32,
    },

    /// The header declares a `total_size` larger than the buffer provided —
    /// the frame is truncated.
    #[error(
        "truncated frame: header declares total_size {total_size} but only {available} bytes are available"
    )]
    TruncatedFrame {
        /// The declared `total_size`.
        total_size: usize,
        /// Bytes actually available in the buffer.
        available: usize,
    },

    /// The offset table (`offset_table_offset` + `8 * offset_table_count`)
    /// does not fit inside the declared frame, or its start sits inside the
    /// header.
    #[error(
        "offset table out of bounds: table starts at {offset} for {count} entries \
         ({bytes} bytes) but the frame is only {total_size} bytes"
    )]
    OffsetTableOutOfBounds {
        /// Message-relative byte offset of the table start
        /// (`WireHeader.offset_table_offset`).
        offset: usize,
        /// Declared entry count (`WireHeader.offset_table_count`).
        count: usize,
        /// Table byte length (`8 * count`).
        bytes: usize,
        /// The declared frame `total_size`.
        total_size: usize,
    },

    /// An offset-table entry points to a region outside the variable payload —
    /// a corrupt or truncated frame. `offset` is payload-relative (measured
    /// from the first byte AFTER the WireHeader), matching
    /// [`crate::OffsetEntry`].
    #[error(
        "variable field {index} out of bounds: payload-relative offset {offset} len {length} \
         does not fit in the {total_size}-byte frame's variable region"
    )]
    VariableFieldOutOfBounds {
        /// The offset-table entry index.
        index: usize,
        /// The entry's payload-relative offset.
        offset: usize,
        /// The entry's declared length in bytes.
        length: usize,
        /// The declared frame `total_size`.
        total_size: usize,
    },
}
