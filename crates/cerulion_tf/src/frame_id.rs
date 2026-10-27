// SPDX-License-Identifier: AGPL-3.0-only
//! Frame handles, and the inline label an error carries for a frame id the
//! buffer has not registered.
//!
//! The buffer interns every frame id it registers: the name lives once in
//! the buffer's name table and everything else, errors included, carries a
//! [`FrameId`]. That keeps a [`crate::TfError`] small and `Copy` with no
//! limit on how long a frame id may be.

use std::fmt;

/// A frame the buffer has registered: a small handle into its frame table.
///
/// [`crate::TfBuffer::frame_id`] gives the handle for a name and
/// [`crate::TfBuffer::frame_name`] gives the name back. A node can resolve
/// its frame ids once and look up by handle on every tick with
/// [`crate::TfBuffer::lookup_ids`], which skips the string work. A handle
/// belongs to the buffer that issued it: it carries that buffer's identity,
/// so another buffer refuses it ([`crate::TfError::UnknownFrameId`]) even
/// when it holds a frame in the same slot. The identity is a process-wide
/// 64-bit counter value taken at construction that never wraps, so no two
/// buffers of one process share it; it never changes what a lookup
/// returns. Prints as `#n`, the slot; use
/// [`crate::TfBuffer::describe`] for the name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FrameId {
    buffer: u64,
    slot: u32,
}

impl FrameId {
    /// The handle of the frame table slot `index` of the buffer whose
    /// identity is `buffer`.
    pub(crate) fn new(buffer: u64, index: usize) -> Self {
        // The frame table never holds more than `u32::MAX` slots: the
        // constructor refuses a larger capacity.
        Self {
            buffer,
            slot: index as u32,
        }
    }

    /// The identity of the buffer that issued this handle.
    pub(crate) fn buffer(self) -> u64 {
        self.buffer
    }

    /// The frame table slot of this handle.
    pub(crate) fn index(self) -> usize {
        self.slot as usize
    }
}

impl fmt::Display for FrameId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "#{}", self.slot)
    }
}

/// The bytes of a frame id a [`FrameLabel`] keeps.
pub const FRAME_LABEL_BYTES: usize = 48;

/// A frame id the buffer has not registered, held inline so that an error
/// naming it allocates nothing.
///
/// It keeps the first [`FRAME_LABEL_BYTES`] bytes of the id, cut at a
/// character boundary, after the leading `/`s are removed; a longer id
/// prints with a trailing `...`. Only errors about a name the buffer could
/// not or did not register carry one; every other error carries a
/// [`FrameId`].
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct FrameLabel {
    len: u8,
    truncated: bool,
    bytes: [u8; FRAME_LABEL_BYTES],
}

impl FrameLabel {
    /// The label of the frame id `id`, with its leading `/`s removed.
    pub fn new(id: &str) -> Self {
        Self::of(normalize(id))
    }

    /// The label of `id`, which has its leading `/`s removed already.
    pub(crate) fn of(id: &str) -> Self {
        // Byte 0 is always a boundary, so the search always ends.
        let end = (0..=FRAME_LABEL_BYTES.min(id.len()))
            .rev()
            .find(|&end| id.is_char_boundary(end))
            .unwrap_or(0);
        let mut bytes = [0u8; FRAME_LABEL_BYTES];
        bytes[..end].copy_from_slice(&id.as_bytes()[..end]);
        Self {
            // At most FRAME_LABEL_BYTES, which fits a u8.
            len: end as u8,
            truncated: end < id.len(),
            bytes,
        }
    }

    /// The kept bytes of the id.
    pub fn as_str(&self) -> &str {
        // The bytes were copied from a `&str` up to a character boundary.
        std::str::from_utf8(&self.bytes[..usize::from(self.len)])
            .expect("a FrameLabel holds the bytes of a str")
    }

    /// Whether the id was longer than [`FRAME_LABEL_BYTES`] and was cut.
    pub fn is_truncated(&self) -> bool {
        self.truncated
    }
}

impl fmt::Debug for FrameLabel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "\"{self}\"")
    }
}

impl fmt::Display for FrameLabel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())?;
        if self.truncated {
            f.write_str("...")?;
        }
        Ok(())
    }
}

impl PartialEq<str> for FrameLabel {
    fn eq(&self, other: &str) -> bool {
        !self.truncated && self.as_str() == other
    }
}

impl PartialEq<&str> for FrameLabel {
    fn eq(&self, other: &&str) -> bool {
        *self == **other
    }
}

/// `id` without its leading `/`s: `/odom`, `//odom` and `odom` name one
/// frame. Removing all of them, not one, is what makes a stored name parse
/// back to itself.
pub(crate) fn normalize(id: &str) -> &str {
    id.trim_start_matches('/')
}
