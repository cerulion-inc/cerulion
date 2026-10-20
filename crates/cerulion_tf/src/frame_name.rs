// SPDX-License-Identifier: AGPL-3.0-only
//! Frame ids as fixed-capacity inline strings.
//!
//! The buffer and its errors carry frame ids by value. Holding them inline,
//! with a fixed capacity, is what lets a refused lookup name the frames
//! involved without touching the heap.

use std::fmt;
use std::str::FromStr;

use crate::TfError;

/// The longest frame id the buffer accepts, in bytes, counted after one
/// leading `/` is removed.
///
/// REP-105 names (`map`, `odom`, `base_link`), the links of a URDF and most
/// namespaced ids (`robot1/front_left_wheel`) fit. A longer id is refused at
/// insert with [`TfError::FrameIdTooLong`]; a namespaced camera frame can be
/// one (`robot_0/camera_aligned_depth_to_color_frame` is 43 bytes). The
/// limit keeps a [`TfError`], which can carry three names, under 128 bytes,
/// so a `Result` of one returns by value without a lint in the caller.
pub const MAX_FRAME_ID_LEN: usize = 36;

/// A frame id, held inline.
///
/// Built by [`str::parse`], which removes one leading `/` (the tf2 rule:
/// `/odom` and `odom` name one frame). An id that is empty after that, or
/// longer than [`MAX_FRAME_ID_LEN`] bytes, is a [`TfError`]. Compares equal
/// to its [`as_str`](Self::as_str) and prints as it.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct FrameName {
    len: u8,
    bytes: [u8; MAX_FRAME_ID_LEN],
}

impl FrameName {
    /// The placeholder an unused frame slot holds; never observable through
    /// the public surface, where every registered frame has a real name.
    pub(crate) const EMPTY: Self = Self {
        len: 0,
        bytes: [0; MAX_FRAME_ID_LEN],
    };

    /// The id as a string slice.
    pub fn as_str(&self) -> &str {
        // The bytes were copied whole from a `&str` by `from_short`, so they
        // are valid UTF-8 up to `len`.
        std::str::from_utf8(&self.bytes[..usize::from(self.len)])
            .expect("a FrameName holds the bytes of a str")
    }

    /// Holds `id`, which is at most [`MAX_FRAME_ID_LEN`] bytes.
    fn from_short(id: &str) -> Self {
        let mut bytes = [0u8; MAX_FRAME_ID_LEN];
        bytes[..id.len()].copy_from_slice(id.as_bytes());
        Self {
            // At most MAX_FRAME_ID_LEN, which fits a u8.
            len: id.len() as u8,
            bytes,
        }
    }

    /// The longest prefix of an over-long `id` that fits, cut at a character
    /// boundary so that it is still a `str`.
    fn prefix(id: &str) -> Self {
        let mut end = MAX_FRAME_ID_LEN;
        while !id.is_char_boundary(end) {
            end -= 1;
        }
        Self::from_short(&id[..end])
    }
}

impl FromStr for FrameName {
    type Err = TfError;

    /// Removes one leading `/`, then checks the length.
    ///
    /// # Errors
    ///
    /// [`TfError::EmptyFrameId`] for `""` and for `"/"`;
    /// [`TfError::FrameIdTooLong`] when the id is longer than
    /// [`MAX_FRAME_ID_LEN`] bytes.
    fn from_str(id: &str) -> Result<Self, TfError> {
        let id = strip_leading_slash(id);
        if id.is_empty() {
            return Err(TfError::EmptyFrameId);
        }
        if id.len() > MAX_FRAME_ID_LEN {
            return Err(TfError::FrameIdTooLong {
                prefix: Self::prefix(id),
                len: id.len(),
            });
        }
        Ok(Self::from_short(id))
    }
}

/// `id` without one leading `/`, the tf2 rule.
fn strip_leading_slash(id: &str) -> &str {
    id.strip_prefix('/').unwrap_or(id)
}

impl fmt::Debug for FrameName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(self.as_str(), f)
    }
}

impl fmt::Display for FrameName {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl AsRef<str> for FrameName {
    fn as_ref(&self) -> &str {
        self.as_str()
    }
}

impl PartialEq<str> for FrameName {
    fn eq(&self, other: &str) -> bool {
        self.as_str() == other
    }
}

impl PartialEq<&str> for FrameName {
    fn eq(&self, other: &&str) -> bool {
        self.as_str() == *other
    }
}
