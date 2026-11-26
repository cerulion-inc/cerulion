// SPDX-License-Identifier: AGPL-3.0-only
//! The cross-process rmw topic graph registry codec and gather fold.
//!
//! This module defines the host-wide wire format v1, fixed shared-memory
//! service sizing constants, snapshot frame encoder/decoder, per-writer
//! generation fold, and required-writer filter for `/__cerulion/rmw_graph`.
//!
//! # Wire contract (frame v1)
//!
//! There is one frame kind: a snapshot part carrying `(topic, type_name)` pairs
//! for one generation of a writer process. All multi-byte integers are
//! little-endian:
//!
//! ```text
//! offset  size  field
//!      0     2  magic       (0xCE, 0x52)
//!      2     1  version     (1)
//!      3     1  reserved    (0)
//!      4    16  writer_id   (u128, UniquePublisherId::value())
//!     20     4  generation  (u32)
//!     24     2  part_index  (u16, 0..part_count)
//!     26     2  part_count  (u16, >= 1)
//!     28     2  entry_count (u16)
//!     30   var  entries     (entry_count x [topic_len(u16), type_len(u16), topic, type])
//! ```
//!
//! An empty writer emits one part with `part_index = 0`, `part_count = 1`, and
//! `entry_count = 0`, so no separate presence frame kind exists.
//!
//! # Static configuration stability
//!
//! The sizing constants ([`RMW_GRAPH_MAX_WRITERS`], [`RMW_GRAPH_MAX_READERS`],
//! [`RMW_GRAPH_QUEUE_DEPTH`], [`RMW_GRAPH_FRAME_LEN`]) and frame v1 are fixed
//! for [`RMW_GRAPH_REGISTRY_SERVICE_NAME`]. Both writers and readers open the
//! service with these constants, so an incompatible change requires a new
//! service name.

use std::collections::BTreeSet;

/// Fixed iceoryx2 pub/sub service name for the rmw topic graph registry.
///
/// Carries no `/data` suffix so `TransportManager::list_topics` and
/// `cerulion_bagd` never surface it as a user topic.
pub const RMW_GRAPH_REGISTRY_SERVICE_NAME: &str = "/__cerulion/rmw_graph";

/// Fixed iceoryx2 event service name for the rmw topic graph doorbell.
///
/// Carries no `/data` suffix and uses a distinct service name from
/// [`RMW_GRAPH_REGISTRY_SERVICE_NAME`].
pub const RMW_GRAPH_DOORBELL_SERVICE_NAME: &str = "/__cerulion/rmw_graph/event";

/// Maximum concurrent rmw writer processes on one host (fixed for service v1).
pub const RMW_GRAPH_MAX_WRITERS: usize = 64;

/// Maximum concurrent rmw graph readers on one host (fixed for service v1).
pub const RMW_GRAPH_MAX_READERS: usize = 16;

/// Per-reader subscriber queue depth (fixed for service v1; at least
/// `2 * RMW_GRAPH_MAX_WRITERS`).
pub const RMW_GRAPH_QUEUE_DEPTH: usize = 128;

/// Maximum doorbell listeners (one per live writer process).
pub const RMW_GRAPH_DOORBELL_MAX_LISTENERS: usize = RMW_GRAPH_MAX_WRITERS;

/// Maximum doorbell notifiers (one per writer for teardown plus one per reader).
pub const RMW_GRAPH_DOORBELL_MAX_NOTIFIERS: usize = RMW_GRAPH_MAX_WRITERS + RMW_GRAPH_MAX_READERS;

/// Maximum serialized frame size in bytes (fixed for service v1).
pub const RMW_GRAPH_FRAME_LEN: usize = 2048;

/// Maximum byte length of a single ROS topic name in a frame.
pub const MAX_RMW_GRAPH_TOPIC_LEN: usize = 256;

/// Maximum byte length of a single ROS type name in a frame.
pub const MAX_RMW_GRAPH_TYPE_LEN: usize = 256;

/// Leading magic bytes (`0xCE, 0x52`), distinct from `reg_channel`,
/// `mirror_registry`, `run_registry`, and `ingress_build`.
pub const RMW_GRAPH_MAGIC: [u8; 2] = [0xCE, 0x52];

/// Wire format version 1 (fixed for [`RMW_GRAPH_REGISTRY_SERVICE_NAME`]).
pub const RMW_GRAPH_VERSION: u8 = 1;

/// Fixed frame header length in bytes.
pub const RMW_GRAPH_HEADER_LEN: usize = 2 + 1 + 1 + 16 + 4 + 2 + 2 + 2;

/// Per-entry length prefix size in bytes (`topic_len(2) + type_len(2)`).
pub const RMW_GRAPH_ENTRY_HEADER_LEN: usize = 4;

// Fixed header field offsets, defined once so encode and decode cannot drift.
const OFF_VERSION: usize = 2;
const OFF_RESERVED: usize = 3;
const OFF_WRITER_ID: usize = 4;
const OFF_GENERATION: usize = 20;
const OFF_PART_INDEX: usize = 24;
const OFF_PART_COUNT: usize = 26;
const OFF_ENTRY_COUNT: usize = 28;

const _: () = assert!(OFF_ENTRY_COUNT + 2 == RMW_GRAPH_HEADER_LEN);

/// Upper bound on per-writer shared-memory pool bytes (8 MiB).
const MAX_PER_WRITER_POOL_BYTES: usize = 8 * 1024 * 1024;

// Compile-time pins on the host-wide static configuration.
const _: () = assert!(
    RMW_GRAPH_HEADER_LEN
        + RMW_GRAPH_ENTRY_HEADER_LEN
        + MAX_RMW_GRAPH_TOPIC_LEN
        + MAX_RMW_GRAPH_TYPE_LEN
        <= RMW_GRAPH_FRAME_LEN,
    "one worst-case (topic, type) entry must fit inside a single frame"
);
const _: () = assert!(
    RMW_GRAPH_QUEUE_DEPTH >= 2 * RMW_GRAPH_MAX_WRITERS,
    "reader queue depth must hold at least two frames per writer"
);
const _: () = assert!(
    RMW_GRAPH_FRAME_LEN * (RMW_GRAPH_MAX_READERS * (RMW_GRAPH_QUEUE_DEPTH + 2) + 2)
        < MAX_PER_WRITER_POOL_BYTES,
    "per-writer shared-memory pool must stay under 8 MiB"
);
const _: () = assert!(
    MAX_RMW_GRAPH_TOPIC_LEN <= u16::MAX as usize && MAX_RMW_GRAPH_TYPE_LEN <= u16::MAX as usize,
    "entry length bounds must fit in u16"
);

/// One `(topic, type_name)` entry carried in a snapshot part frame.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct RmwGraphEntry {
    /// ROS topic name (for example, `"/joint_states"`).
    pub topic: String,
    /// ROS message type name (for example, `"sensor_msgs/msg/JointState"`).
    pub type_name: String,
}

/// One decoded snapshot part frame from the rmw graph registry wire.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RmwGraphFrame {
    /// Iceoryx2 `UniquePublisherId::value()` of the control publisher.
    pub writer_id: u128,
    /// Monotonic per-writer snapshot generation counter.
    pub generation: u32,
    /// Zero-based index of this part within the generation (`0..part_count`).
    pub part_index: u16,
    /// Total number of parts in this generation (`>= 1`).
    pub part_count: u16,
    /// Entries carried in this part (empty when a writer has zero endpoints).
    pub entries: Vec<RmwGraphEntry>,
}

/// Why an rmw graph snapshot frame failed to encode or decode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum RmwGraphFrameError {
    /// A topic name is empty.
    #[error("empty topic name")]
    EmptyTopic,
    /// A ROS type name is empty.
    #[error("empty type name")]
    EmptyType,
    /// A topic name exceeds [`MAX_RMW_GRAPH_TOPIC_LEN`] bytes.
    #[error("topic length {len} exceeds the {MAX_RMW_GRAPH_TOPIC_LEN}-byte limit")]
    TopicTooLong {
        /// Byte length of the rejected topic name.
        len: usize,
    },
    /// A ROS type name exceeds [`MAX_RMW_GRAPH_TYPE_LEN`] bytes.
    #[error("type length {len} exceeds the {MAX_RMW_GRAPH_TYPE_LEN}-byte limit")]
    TypeTooLong {
        /// Byte length of the rejected type name.
        len: usize,
    },
    /// A snapshot would require more than `u16::MAX` parts.
    #[error("snapshot requires {count} parts, exceeding the u16 limit")]
    TooManyParts {
        /// Computed number of parts.
        count: usize,
    },
    /// The raw frame is shorter than [`RMW_GRAPH_HEADER_LEN`].
    #[error("frame is {len} bytes, shorter than the {RMW_GRAPH_HEADER_LEN}-byte header")]
    TruncatedHeader {
        /// Observed frame length in bytes.
        len: usize,
    },
    /// The raw frame exceeds [`RMW_GRAPH_FRAME_LEN`].
    #[error("frame is {len} bytes, exceeding the {RMW_GRAPH_FRAME_LEN}-byte limit")]
    FrameTooLarge {
        /// Observed frame length in bytes.
        len: usize,
    },
    /// The leading magic bytes are not [`RMW_GRAPH_MAGIC`].
    #[error("bad frame magic (not an rmw graph frame)")]
    BadMagic,
    /// The version byte is not [`RMW_GRAPH_VERSION`].
    #[error("unsupported frame version {found} (expected {RMW_GRAPH_VERSION})")]
    BadVersion {
        /// Observed version byte.
        found: u8,
    },
    /// The reserved header byte is non-zero.
    #[error("non-zero reserved header byte {found} (expected 0)")]
    BadReserved {
        /// Observed reserved byte.
        found: u8,
    },
    /// `part_count` is zero or `part_index >= part_count`.
    #[error("invalid part bounds: part_index={part_index}, part_count={part_count}")]
    InvalidPartBounds {
        /// Observed part index.
        part_index: u16,
        /// Observed total part count.
        part_count: u16,
    },
    /// The frame ended before entry `entry_index` completed.
    #[error("frame truncated while reading entry {entry_index}")]
    TruncatedEntry {
        /// Zero-based index of the truncated entry.
        entry_index: usize,
    },
    /// Unconsumed bytes remain after the declared `entry_count` entries.
    #[error("frame carries {trailing} trailing bytes after the declared entries")]
    TrailingBytes {
        /// Number of unconsumed trailing bytes.
        trailing: usize,
    },
    /// Topic or type bytes are not valid UTF-8.
    #[error("topic or type bytes are not valid UTF-8")]
    InvalidUtf8,
}

fn read_u16_le(bytes: &[u8], offset: usize) -> Option<u16> {
    let slice = bytes.get(offset..offset + 2)?;
    Some(u16::from_le_bytes([slice[0], slice[1]]))
}

fn read_u32_le(bytes: &[u8], offset: usize) -> Option<u32> {
    let slice = bytes.get(offset..offset + 4)?;
    Some(u32::from_le_bytes([slice[0], slice[1], slice[2], slice[3]]))
}

fn read_u128_le(bytes: &[u8], offset: usize) -> Option<u128> {
    let slice = bytes.get(offset..offset + 16)?;
    let mut arr = [0u8; 16];
    arr.copy_from_slice(slice);
    Some(u128::from_le_bytes(arr))
}

fn write_part(
    writer_id: u128,
    generation: u32,
    part_index: u16,
    part_count: u16,
    entries: &[(&str, &str)],
) -> Vec<u8> {
    let payload_len: usize = entries
        .iter()
        .map(|(t, ty)| RMW_GRAPH_ENTRY_HEADER_LEN + t.len() + ty.len())
        .sum();
    let mut buf = Vec::with_capacity(RMW_GRAPH_HEADER_LEN + payload_len);
    buf.extend_from_slice(&RMW_GRAPH_MAGIC);
    buf.push(RMW_GRAPH_VERSION);
    buf.push(0); // reserved
    buf.extend_from_slice(&writer_id.to_le_bytes());
    buf.extend_from_slice(&generation.to_le_bytes());
    buf.extend_from_slice(&part_index.to_le_bytes());
    buf.extend_from_slice(&part_count.to_le_bytes());
    buf.extend_from_slice(&(entries.len() as u16).to_le_bytes());
    for &(topic, type_name) in entries {
        buf.extend_from_slice(&(topic.len() as u16).to_le_bytes());
        buf.extend_from_slice(&(type_name.len() as u16).to_le_bytes());
        buf.extend_from_slice(topic.as_bytes());
        buf.extend_from_slice(type_name.as_bytes());
    }
    buf
}

/// Encode a writer's `(topic, type_name)` set for one `generation` into one or
/// more bounded snapshot part frames (each at most [`RMW_GRAPH_FRAME_LEN`]
/// bytes).
///
/// Entries are validated before any frame is allocated, then deduplicated and
/// sorted so output frames are canonical regardless of input order. An empty
/// entry set produces a single 30-byte frame with `part_index = 0`,
/// `part_count = 1`, and `entry_count = 0`.
///
/// # Errors
///
/// Returns [`RmwGraphFrameError::EmptyTopic`], [`RmwGraphFrameError::EmptyType`],
/// [`RmwGraphFrameError::TopicTooLong`], [`RmwGraphFrameError::TypeTooLong`],
/// or [`RmwGraphFrameError::TooManyParts`].
pub fn encode_snapshot<'a, I>(
    writer_id: u128,
    generation: u32,
    entries: I,
) -> Result<Vec<Vec<u8>>, RmwGraphFrameError>
where
    I: IntoIterator<Item = (&'a str, &'a str)>,
{
    let mut sorted = BTreeSet::new();
    for (topic, type_name) in entries {
        if topic.is_empty() {
            return Err(RmwGraphFrameError::EmptyTopic);
        }
        if type_name.is_empty() {
            return Err(RmwGraphFrameError::EmptyType);
        }
        if topic.len() > MAX_RMW_GRAPH_TOPIC_LEN {
            return Err(RmwGraphFrameError::TopicTooLong { len: topic.len() });
        }
        if type_name.len() > MAX_RMW_GRAPH_TYPE_LEN {
            return Err(RmwGraphFrameError::TypeTooLong {
                len: type_name.len(),
            });
        }
        sorted.insert((topic, type_name));
    }

    if sorted.is_empty() {
        return Ok(vec![write_part(writer_id, generation, 0, 1, &[])]);
    }

    let mut parts: Vec<Vec<(&str, &str)>> = Vec::new();
    let mut current: Vec<(&str, &str)> = Vec::new();
    let mut current_bytes = RMW_GRAPH_HEADER_LEN;
    for (topic, type_name) in sorted {
        let entry_bytes = RMW_GRAPH_ENTRY_HEADER_LEN + topic.len() + type_name.len();
        if !current.is_empty() && current_bytes + entry_bytes > RMW_GRAPH_FRAME_LEN {
            parts.push(current);
            current = Vec::new();
            current_bytes = RMW_GRAPH_HEADER_LEN;
        }
        current.push((topic, type_name));
        current_bytes += entry_bytes;
    }
    if !current.is_empty() {
        parts.push(current);
    }

    let part_count = u16::try_from(parts.len())
        .map_err(|_| RmwGraphFrameError::TooManyParts { count: parts.len() })?;
    let mut frames = Vec::with_capacity(parts.len());
    for (idx, part) in parts.iter().enumerate() {
        frames.push(write_part(
            writer_id, generation, idx as u16, part_count, part,
        ));
    }
    Ok(frames)
}

/// Decode one snapshot part frame from raw wire bytes.
///
/// Every header field, length prefix, byte bound, and UTF-8 check runs before
/// any heap allocation so hostile or truncated frames never over-allocate.
///
/// # Errors
///
/// Returns a specific [`RmwGraphFrameError`] variant for any malformed frame.
pub fn decode_frame(bytes: &[u8]) -> Result<RmwGraphFrame, RmwGraphFrameError> {
    if bytes.len() < RMW_GRAPH_HEADER_LEN {
        return Err(RmwGraphFrameError::TruncatedHeader { len: bytes.len() });
    }
    if bytes.len() > RMW_GRAPH_FRAME_LEN {
        return Err(RmwGraphFrameError::FrameTooLarge { len: bytes.len() });
    }
    if bytes[0..2] != RMW_GRAPH_MAGIC {
        return Err(RmwGraphFrameError::BadMagic);
    }
    if bytes[OFF_VERSION] != RMW_GRAPH_VERSION {
        return Err(RmwGraphFrameError::BadVersion {
            found: bytes[OFF_VERSION],
        });
    }
    if bytes[OFF_RESERVED] != 0 {
        return Err(RmwGraphFrameError::BadReserved {
            found: bytes[OFF_RESERVED],
        });
    }

    let truncated_header = || RmwGraphFrameError::TruncatedHeader { len: bytes.len() };
    let writer_id = read_u128_le(bytes, OFF_WRITER_ID).ok_or_else(truncated_header)?;
    let generation = read_u32_le(bytes, OFF_GENERATION).ok_or_else(truncated_header)?;
    let part_index = read_u16_le(bytes, OFF_PART_INDEX).ok_or_else(truncated_header)?;
    let part_count = read_u16_le(bytes, OFF_PART_COUNT).ok_or_else(truncated_header)?;
    let entry_count = read_u16_le(bytes, OFF_ENTRY_COUNT).ok_or_else(truncated_header)?;

    if part_count == 0 || part_index >= part_count {
        return Err(RmwGraphFrameError::InvalidPartBounds {
            part_index,
            part_count,
        });
    }

    // Pass 1: validate every entry length, frame boundary, and UTF-8 slice
    // before allocating the result vector or any strings.
    let count = usize::from(entry_count);
    let mut cursor = RMW_GRAPH_HEADER_LEN;
    for entry_index in 0..count {
        let Some(topic_len_u16) = read_u16_le(bytes, cursor) else {
            return Err(RmwGraphFrameError::TruncatedEntry { entry_index });
        };
        let Some(type_len_u16) = read_u16_le(bytes, cursor + 2) else {
            return Err(RmwGraphFrameError::TruncatedEntry { entry_index });
        };
        let topic_len = usize::from(topic_len_u16);
        let type_len = usize::from(type_len_u16);
        if topic_len == 0 {
            return Err(RmwGraphFrameError::EmptyTopic);
        }
        if topic_len > MAX_RMW_GRAPH_TOPIC_LEN {
            return Err(RmwGraphFrameError::TopicTooLong { len: topic_len });
        }
        if type_len == 0 {
            return Err(RmwGraphFrameError::EmptyType);
        }
        if type_len > MAX_RMW_GRAPH_TYPE_LEN {
            return Err(RmwGraphFrameError::TypeTooLong { len: type_len });
        }
        let topic_start = cursor + RMW_GRAPH_ENTRY_HEADER_LEN;
        let topic_end = topic_start.saturating_add(topic_len);
        let type_end = topic_end.saturating_add(type_len);
        if type_end > bytes.len() {
            return Err(RmwGraphFrameError::TruncatedEntry { entry_index });
        }
        if std::str::from_utf8(&bytes[topic_start..topic_end]).is_err()
            || std::str::from_utf8(&bytes[topic_end..type_end]).is_err()
        {
            return Err(RmwGraphFrameError::InvalidUtf8);
        }
        cursor = type_end;
    }
    if cursor != bytes.len() {
        return Err(RmwGraphFrameError::TrailingBytes {
            trailing: bytes.len() - cursor,
        });
    }

    // Pass 2: every bound and UTF-8 invariant passed; materialize owned strings.
    let mut entries = Vec::with_capacity(count);
    let mut cursor = RMW_GRAPH_HEADER_LEN;
    for entry_index in 0..count {
        let topic_len = usize::from(
            read_u16_le(bytes, cursor).ok_or(RmwGraphFrameError::TruncatedEntry { entry_index })?,
        );
        let type_len = usize::from(
            read_u16_le(bytes, cursor + 2)
                .ok_or(RmwGraphFrameError::TruncatedEntry { entry_index })?,
        );
        let topic_start = cursor + RMW_GRAPH_ENTRY_HEADER_LEN;
        let topic_end = topic_start + topic_len;
        let type_end = topic_end + type_len;
        let topic = std::str::from_utf8(&bytes[topic_start..topic_end])
            .map_err(|_| RmwGraphFrameError::InvalidUtf8)?
            .to_string();
        let type_name = std::str::from_utf8(&bytes[topic_end..type_end])
            .map_err(|_| RmwGraphFrameError::InvalidUtf8)?
            .to_string();
        entries.push(RmwGraphEntry { topic, type_name });
        cursor = type_end;
    }

    Ok(RmwGraphFrame {
        writer_id,
        generation,
        part_index,
        part_count,
        entries,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestHeader {
        magic: [u8; 2],
        version: u8,
        reserved: u8,
        writer_id: u128,
        generation: u32,
        part_index: u16,
        part_count: u16,
        entry_count: u16,
    }

    impl TestHeader {
        fn valid() -> Self {
            Self {
                magic: RMW_GRAPH_MAGIC,
                version: RMW_GRAPH_VERSION,
                reserved: 0,
                writer_id: 1,
                generation: 1,
                part_index: 0,
                part_count: 1,
                entry_count: 0,
            }
        }

        fn to_bytes(&self) -> Vec<u8> {
            let mut bytes = Vec::with_capacity(RMW_GRAPH_HEADER_LEN);
            bytes.extend_from_slice(&self.magic);
            bytes.push(self.version);
            bytes.push(self.reserved);
            bytes.extend_from_slice(&self.writer_id.to_le_bytes());
            bytes.extend_from_slice(&self.generation.to_le_bytes());
            bytes.extend_from_slice(&self.part_index.to_le_bytes());
            bytes.extend_from_slice(&self.part_count.to_le_bytes());
            bytes.extend_from_slice(&self.entry_count.to_le_bytes());
            bytes
        }
    }

    fn append_entry(buf: &mut Vec<u8>, topic: &[u8], type_name: &[u8]) {
        buf.extend_from_slice(&(topic.len() as u16).to_le_bytes());
        buf.extend_from_slice(&(type_name.len() as u16).to_le_bytes());
        buf.extend_from_slice(topic);
        buf.extend_from_slice(type_name);
    }

    #[test]
    fn service_names_carry_no_data_suffix() {
        assert!(!RMW_GRAPH_REGISTRY_SERVICE_NAME.ends_with("/data"));
        assert!(!RMW_GRAPH_DOORBELL_SERVICE_NAME.ends_with("/data"));
        assert_ne!(
            RMW_GRAPH_REGISTRY_SERVICE_NAME,
            RMW_GRAPH_DOORBELL_SERVICE_NAME
        );
    }

    #[test]
    fn encode_matches_hand_written_wire_oracle_for_two_entries() {
        let writer_id: u128 = 0x0102_0304_0506_0708_090A_0B0C_0D0E_0F10;
        let generation: u32 = 0x1122_3344;
        let frames = encode_snapshot(
            writer_id,
            generation,
            [
                ("/joint_states", "sensor_msgs/msg/JointState"),
                ("/clock", "rosgraph_msgs/msg/Clock"),
            ],
        )
        .expect("valid snapshot encodes");

        // Sorted order places /clock before /joint_states.
        let mut expected = Vec::new();
        expected.extend_from_slice(&[0xCE, 0x52]); // magic
        expected.push(0x01); // version
        expected.push(0x00); // reserved
        expected.extend_from_slice(&[
            0x10, 0x0F, 0x0E, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, // writer_id LE low
            0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, // writer_id LE high
        ]);
        expected.extend_from_slice(&[0x44, 0x33, 0x22, 0x11]); // generation LE
        expected.extend_from_slice(&[0x00, 0x00]); // part_index = 0
        expected.extend_from_slice(&[0x01, 0x00]); // part_count = 1
        expected.extend_from_slice(&[0x02, 0x00]); // entry_count = 2
                                                   // Entry 0: "/clock" (6 bytes), "rosgraph_msgs/msg/Clock" (23 bytes)
        expected.extend_from_slice(&[0x06, 0x00, 0x17, 0x00]);
        expected.extend_from_slice(b"/clock");
        expected.extend_from_slice(b"rosgraph_msgs/msg/Clock");
        // Entry 1: "/joint_states" (13 bytes), "sensor_msgs/msg/JointState" (26 bytes)
        expected.extend_from_slice(&[0x0D, 0x00, 0x1A, 0x00]);
        expected.extend_from_slice(b"/joint_states");
        expected.extend_from_slice(b"sensor_msgs/msg/JointState");

        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0], expected);
    }

    #[test]
    fn encode_empty_snapshot_matches_hand_written_zero_entry_part() {
        let writer_id: u128 = 0xAA00_0000_0000_0000_0000_0000_0000_00BB;
        let frames =
            encode_snapshot(writer_id, 7, std::iter::empty()).expect("empty snapshot encodes");

        let expected: Vec<u8> = vec![
            0xCE, 0x52, // magic
            0x01, // version
            0x00, // reserved
            0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // writer_id LE low
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA, // writer_id LE high
            0x07, 0x00, 0x00, 0x00, // generation = 7
            0x00, 0x00, // part_index = 0
            0x01, 0x00, // part_count = 1
            0x00, 0x00, // entry_count = 0
        ];

        assert_eq!(frames, vec![expected]);
    }

    #[test]
    fn decode_hand_written_wire_oracle_returns_exact_frame() {
        let mut raw = vec![
            0xCE, 0x52, // magic
            0x01, // version
            0x00, // reserved
            0x2A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // writer_id = 42
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // writer_id high
            0x03, 0x00, 0x00, 0x00, // generation = 3
            0x01, 0x00, // part_index = 1
            0x02, 0x00, // part_count = 2
            0x01, 0x00, // entry_count = 1
            0x07, 0x00, 0x16, 0x00, // topic_len = 7, type_len = 22
        ];
        raw.extend_from_slice(b"/rosout");
        raw.extend_from_slice(b"rcl_interfaces/msg/Log");

        let frame = decode_frame(&raw).expect("valid hand-written wire decodes");
        assert_eq!(
            frame,
            RmwGraphFrame {
                writer_id: 42,
                generation: 3,
                part_index: 1,
                part_count: 2,
                entries: vec![RmwGraphEntry {
                    topic: "/rosout".to_string(),
                    type_name: "rcl_interfaces/msg/Log".to_string(),
                }],
            }
        );
    }

    #[test]
    fn decode_zero_entry_part_succeeds() {
        let raw = TestHeader {
            writer_id: 99,
            ..TestHeader::valid()
        }
        .to_bytes();
        let frame = decode_frame(&raw).expect("zero-entry part decodes");
        assert_eq!(
            frame,
            RmwGraphFrame {
                writer_id: 99,
                generation: 1,
                part_index: 0,
                part_count: 1,
                entries: Vec::new(),
            }
        );
    }

    #[test]
    fn encode_splits_large_snapshot_across_bounded_parts() {
        // Each entry is 4 + 20 + 26 = 50 bytes. 50 * 50 = 2500 > 2048 - 30 = 2018,
        // so 50 entries must split into 2 parts (40 in part 0, 10 in part 1).
        let pairs: Vec<(String, String)> = (0..50)
            .map(|i| {
                (
                    format!("/robot/topic_num_{i:03}"),
                    "sensor_msgs/msg/JointState".to_string(),
                )
            })
            .collect();
        let refs: Vec<(&str, &str)> = pairs
            .iter()
            .map(|(t, ty)| (t.as_str(), ty.as_str()))
            .collect();

        let frames = encode_snapshot(1234, 9, refs).expect("multi-part snapshot encodes");
        assert_eq!(frames.len(), 2);
        assert!(frames.iter().all(|f| f.len() <= RMW_GRAPH_FRAME_LEN));

        let part0 = decode_frame(&frames[0]).expect("part 0 decodes");
        let part1 = decode_frame(&frames[1]).expect("part 1 decodes");
        assert_eq!(part0.part_index, 0);
        assert_eq!(part0.part_count, 2);
        assert_eq!(part0.entries.len(), 40);
        assert_eq!(part1.part_index, 1);
        assert_eq!(part1.part_count, 2);
        assert_eq!(part1.entries.len(), 10);
        assert_eq!(part0.entries[0].topic, "/robot/topic_num_000");
        assert_eq!(part1.entries[9].topic, "/robot/topic_num_049");
    }

    #[test]
    fn encode_is_deterministic_across_opposite_input_orders_and_duplicates() {
        let a = encode_snapshot(
            55,
            2,
            [
                ("/z", "pkg/msg/Z"),
                ("/a", "pkg/msg/A"),
                ("/m", "pkg/msg/M"),
                ("/a", "pkg/msg/A"),
            ],
        )
        .expect("order A encodes");
        let b = encode_snapshot(
            55,
            2,
            [
                ("/a", "pkg/msg/A"),
                ("/m", "pkg/msg/M"),
                ("/z", "pkg/msg/Z"),
            ],
        )
        .expect("order B encodes");
        assert_eq!(a, b);
    }

    #[test]
    fn encode_refuses_empty_and_oversized_names() {
        assert_eq!(
            encode_snapshot(1, 1, [("", "std_msgs/msg/String")]),
            Err(RmwGraphFrameError::EmptyTopic)
        );
        assert_eq!(
            encode_snapshot(1, 1, [("/a", "")]),
            Err(RmwGraphFrameError::EmptyType)
        );
        let long_topic = format!("/{}", "a".repeat(MAX_RMW_GRAPH_TOPIC_LEN));
        assert_eq!(
            encode_snapshot(1, 1, [(long_topic.as_str(), "std_msgs/msg/String")]),
            Err(RmwGraphFrameError::TopicTooLong {
                len: MAX_RMW_GRAPH_TOPIC_LEN + 1
            })
        );
        let long_type = "t".repeat(MAX_RMW_GRAPH_TYPE_LEN + 1);
        assert_eq!(
            encode_snapshot(1, 1, [("/a", long_type.as_str())]),
            Err(RmwGraphFrameError::TypeTooLong {
                len: MAX_RMW_GRAPH_TYPE_LEN + 1
            })
        );
    }

    #[test]
    fn decode_refuses_every_malformed_wire_shape() {
        // Truncated header
        assert_eq!(
            decode_frame(&[0xCE, 0x52, 0x01]),
            Err(RmwGraphFrameError::TruncatedHeader { len: 3 })
        );

        // Oversized frame
        let oversized = vec![0u8; RMW_GRAPH_FRAME_LEN + 1];
        assert_eq!(
            decode_frame(&oversized),
            Err(RmwGraphFrameError::FrameTooLarge {
                len: RMW_GRAPH_FRAME_LEN + 1
            })
        );

        // Bad magic (mirror_registry's 0xCE37 and run_registry's 0xCE81)
        let mirror_magic = TestHeader {
            magic: [0xCE, 0x37],
            ..TestHeader::valid()
        }
        .to_bytes();
        assert_eq!(
            decode_frame(&mirror_magic),
            Err(RmwGraphFrameError::BadMagic)
        );
        let run_magic = TestHeader {
            magic: [0xCE, 0x81],
            ..TestHeader::valid()
        }
        .to_bytes();
        assert_eq!(decode_frame(&run_magic), Err(RmwGraphFrameError::BadMagic));

        // Bad version
        let bad_ver = TestHeader {
            version: 2,
            ..TestHeader::valid()
        }
        .to_bytes();
        assert_eq!(
            decode_frame(&bad_ver),
            Err(RmwGraphFrameError::BadVersion { found: 2 })
        );

        // Non-zero reserved byte
        let bad_res = TestHeader {
            reserved: 0xFF,
            ..TestHeader::valid()
        }
        .to_bytes();
        assert_eq!(
            decode_frame(&bad_res),
            Err(RmwGraphFrameError::BadReserved { found: 0xFF })
        );

        // Invalid part bounds: part_count == 0 and part_index >= part_count
        let zero_parts = TestHeader {
            part_count: 0,
            ..TestHeader::valid()
        }
        .to_bytes();
        assert_eq!(
            decode_frame(&zero_parts),
            Err(RmwGraphFrameError::InvalidPartBounds {
                part_index: 0,
                part_count: 0
            })
        );
        let oob_part = TestHeader {
            part_index: 2,
            part_count: 2,
            ..TestHeader::valid()
        }
        .to_bytes();
        assert_eq!(
            decode_frame(&oob_part),
            Err(RmwGraphFrameError::InvalidPartBounds {
                part_index: 2,
                part_count: 2
            })
        );

        // Hostile entry_count = u16::MAX with no entry bytes (must fail without allocating)
        let huge_count = TestHeader {
            entry_count: u16::MAX,
            ..TestHeader::valid()
        }
        .to_bytes();
        assert_eq!(
            decode_frame(&huge_count),
            Err(RmwGraphFrameError::TruncatedEntry { entry_index: 0 })
        );

        // Entry length prefix present, but payload truncated
        let mut trunc_body = TestHeader {
            entry_count: 1,
            ..TestHeader::valid()
        }
        .to_bytes();
        trunc_body.extend_from_slice(&4u16.to_le_bytes());
        trunc_body.extend_from_slice(&4u16.to_le_bytes());
        trunc_body.extend_from_slice(b"/abc"); // missing 4 type bytes
        assert_eq!(
            decode_frame(&trunc_body),
            Err(RmwGraphFrameError::TruncatedEntry { entry_index: 0 })
        );

        // Empty topic in wire entry
        let mut empty_topic = TestHeader {
            entry_count: 1,
            ..TestHeader::valid()
        }
        .to_bytes();
        append_entry(&mut empty_topic, b"", b"std_msgs/msg/String");
        assert_eq!(
            decode_frame(&empty_topic),
            Err(RmwGraphFrameError::EmptyTopic)
        );

        // Empty type in wire entry
        let mut empty_type = TestHeader {
            entry_count: 1,
            ..TestHeader::valid()
        }
        .to_bytes();
        append_entry(&mut empty_type, b"/a", b"");
        assert_eq!(
            decode_frame(&empty_type),
            Err(RmwGraphFrameError::EmptyType)
        );

        // Topic length over bound (even before payload bytes exist)
        let mut over_topic = TestHeader {
            entry_count: 1,
            ..TestHeader::valid()
        }
        .to_bytes();
        over_topic.extend_from_slice(&((MAX_RMW_GRAPH_TOPIC_LEN + 1) as u16).to_le_bytes());
        over_topic.extend_from_slice(&1u16.to_le_bytes());
        assert_eq!(
            decode_frame(&over_topic),
            Err(RmwGraphFrameError::TopicTooLong {
                len: MAX_RMW_GRAPH_TOPIC_LEN + 1
            })
        );

        // Type length over bound
        let mut over_type = TestHeader {
            entry_count: 1,
            ..TestHeader::valid()
        }
        .to_bytes();
        over_type.extend_from_slice(&2u16.to_le_bytes());
        over_type.extend_from_slice(&((MAX_RMW_GRAPH_TYPE_LEN + 1) as u16).to_le_bytes());
        assert_eq!(
            decode_frame(&over_type),
            Err(RmwGraphFrameError::TypeTooLong {
                len: MAX_RMW_GRAPH_TYPE_LEN + 1
            })
        );

        // Non-UTF-8 topic bytes
        let mut bad_utf8_topic = TestHeader {
            entry_count: 1,
            ..TestHeader::valid()
        }
        .to_bytes();
        append_entry(&mut bad_utf8_topic, &[0xFF, 0xFE], b"pkg/msg/T");
        assert_eq!(
            decode_frame(&bad_utf8_topic),
            Err(RmwGraphFrameError::InvalidUtf8)
        );

        // Non-UTF-8 type bytes
        let mut bad_utf8_type = TestHeader {
            entry_count: 1,
            ..TestHeader::valid()
        }
        .to_bytes();
        append_entry(&mut bad_utf8_type, b"/ok", &[0x80, 0x81]);
        assert_eq!(
            decode_frame(&bad_utf8_type),
            Err(RmwGraphFrameError::InvalidUtf8)
        );

        // Trailing bytes after declared entries
        let mut trailing = TestHeader::valid().to_bytes();
        trailing.extend_from_slice(&[0xDE, 0xAD]);
        assert_eq!(
            decode_frame(&trailing),
            Err(RmwGraphFrameError::TrailingBytes { trailing: 2 })
        );
    }

    #[test]
    fn encode_and_decode_accept_exact_length_boundaries() {
        let max_topic = format!("/{}", "a".repeat(MAX_RMW_GRAPH_TOPIC_LEN - 1));
        let max_type = "b".repeat(MAX_RMW_GRAPH_TYPE_LEN);
        assert_eq!(max_topic.len(), MAX_RMW_GRAPH_TOPIC_LEN);
        assert_eq!(max_type.len(), MAX_RMW_GRAPH_TYPE_LEN);

        let frames = encode_snapshot(77, 1, [(max_topic.as_str(), max_type.as_str())])
            .expect("exact max lengths encode");
        assert_eq!(frames.len(), 1);
        assert_eq!(
            frames[0].len(),
            RMW_GRAPH_HEADER_LEN
                + RMW_GRAPH_ENTRY_HEADER_LEN
                + MAX_RMW_GRAPH_TOPIC_LEN
                + MAX_RMW_GRAPH_TYPE_LEN
        );
        let decoded = decode_frame(&frames[0]).expect("exact max lengths decode");
        assert_eq!(
            decoded.entries,
            vec![RmwGraphEntry {
                topic: max_topic,
                type_name: max_type,
            }]
        );
    }
}
