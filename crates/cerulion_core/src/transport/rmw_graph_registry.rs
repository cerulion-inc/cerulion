// SPDX-License-Identifier: AGPL-3.0-only
//! The cross-process rmw topic graph registry codec and gather fold.
//!
//! Defines the host-wide wire format v1, fixed shared-memory service sizing
//! constants, snapshot frame encoder and decoder, per-writer generation fold,
//! and required-writer filter for `/__cerulion/rmw_graph`.
//!
//! # Wire contract (frame v1)
//!
//! One frame kind: a snapshot part carrying `(topic, type_name)` pairs for one
//! generation of a writer process. All multi-byte integers are little-endian:
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

use std::collections::{BTreeMap, BTreeSet};

pub use super::mirror_registry::GatherCompleteness;

/// Fixed iceoryx2 pub/sub service name for the rmw topic graph registry.
/// Carries no `/data` suffix so `TransportManager::list_topics` and
/// `cerulion_bagd` never surface it as a user topic.
pub const RMW_GRAPH_REGISTRY_SERVICE_NAME: &str = "/__cerulion/rmw_graph";

/// Fixed iceoryx2 event service name for the rmw topic graph doorbell.
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
const MAX_PER_WRITER_POOL_BYTES: usize = 8 * 1024 * 1024;

const _: () = assert!(OFF_ENTRY_COUNT + 2 == RMW_GRAPH_HEADER_LEN);
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
    pub topic: String,
    pub type_name: String,
}

/// One decoded snapshot part frame from the rmw graph registry wire.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RmwGraphFrame {
    pub writer_id: u128,
    pub generation: u32,
    pub part_index: u16,
    pub part_count: u16,
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
    TopicTooLong { len: usize },
    /// A ROS type name exceeds [`MAX_RMW_GRAPH_TYPE_LEN`] bytes.
    #[error("type length {len} exceeds the {MAX_RMW_GRAPH_TYPE_LEN}-byte limit")]
    TypeTooLong { len: usize },
    /// A snapshot would require more than `u16::MAX` parts.
    #[error("snapshot requires {count} parts, exceeding the u16 limit")]
    TooManyParts { count: usize },
    /// The raw frame is shorter than [`RMW_GRAPH_HEADER_LEN`].
    #[error("frame is {len} bytes, shorter than the {RMW_GRAPH_HEADER_LEN}-byte header")]
    TruncatedHeader { len: usize },
    /// The raw frame exceeds [`RMW_GRAPH_FRAME_LEN`].
    #[error("frame is {len} bytes, exceeding the {RMW_GRAPH_FRAME_LEN}-byte limit")]
    FrameTooLarge { len: usize },
    /// The leading magic bytes are not [`RMW_GRAPH_MAGIC`].
    #[error("bad frame magic (not an rmw graph frame)")]
    BadMagic,
    /// The version byte is not [`RMW_GRAPH_VERSION`].
    #[error("unsupported frame version {found} (expected {RMW_GRAPH_VERSION})")]
    BadVersion { found: u8 },
    /// The reserved header byte is non-zero.
    #[error("non-zero reserved header byte {found} (expected 0)")]
    BadReserved { found: u8 },
    /// `part_count` is zero or `part_index >= part_count`.
    #[error("invalid part bounds: part_index={part_index}, part_count={part_count}")]
    InvalidPartBounds { part_index: u16, part_count: u16 },
    /// The frame ended before entry `entry_index` completed.
    #[error("frame truncated while reading entry {entry_index}")]
    TruncatedEntry { entry_index: usize },
    /// Unconsumed bytes remain after the declared `entry_count` entries.
    #[error("frame carries {trailing} trailing bytes after the declared entries")]
    TrailingBytes { trailing: usize },
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
    buf.push(0);
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

fn parse_entry_slice(
    bytes: &[u8],
    cursor: usize,
    entry_index: usize,
) -> Result<(&str, &str, usize), RmwGraphFrameError> {
    let topic_len = usize::from(
        read_u16_le(bytes, cursor).ok_or(RmwGraphFrameError::TruncatedEntry { entry_index })?,
    );
    let type_len = usize::from(
        read_u16_le(bytes, cursor + 2).ok_or(RmwGraphFrameError::TruncatedEntry { entry_index })?,
    );
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
    let topic = std::str::from_utf8(&bytes[topic_start..topic_end])
        .map_err(|_| RmwGraphFrameError::InvalidUtf8)?;
    let type_name = std::str::from_utf8(&bytes[topic_end..type_end])
        .map_err(|_| RmwGraphFrameError::InvalidUtf8)?;
    Ok((topic, type_name, type_end))
}

/// Decode one snapshot part frame from raw wire bytes.
///
/// Every header field, length prefix, byte bound, and UTF-8 check runs before
/// any heap allocation so hostile or truncated frames never over-allocate.
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

    // Pass 1 validates every entry length, boundary, and UTF-8 slice before
    // allocating the output vector or any owned strings.
    let count = usize::from(entry_count);
    let mut cursor = RMW_GRAPH_HEADER_LEN;
    for entry_index in 0..count {
        let (_, _, next) = parse_entry_slice(bytes, cursor, entry_index)?;
        cursor = next;
    }
    if cursor != bytes.len() {
        return Err(RmwGraphFrameError::TrailingBytes {
            trailing: bytes.len() - cursor,
        });
    }

    let mut entries = Vec::with_capacity(count);
    let mut cursor = RMW_GRAPH_HEADER_LEN;
    for entry_index in 0..count {
        let (topic, type_name, next) = parse_entry_slice(bytes, cursor, entry_index)?;
        entries.push(RmwGraphEntry {
            topic: topic.to_string(),
            type_name: type_name.to_string(),
        });
        cursor = next;
    }

    Ok(RmwGraphFrame {
        writer_id,
        generation,
        part_index,
        part_count,
        entries,
    })
}

/// Result of folding snapshot frames for a gather pass: sorted deduplicated
/// `topic -> sorted unique type names`, paired with [`GatherCompleteness`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RmwGraphGather {
    pub topics: BTreeMap<String, BTreeSet<String>>,
    pub completeness: GatherCompleteness,
}

#[derive(Debug, Clone)]
struct GenerationBuffer {
    part_count: u16,
    inconsistent: bool,
    parts: BTreeMap<u16, Vec<RmwGraphEntry>>,
}

impl GenerationBuffer {
    fn is_complete(&self) -> bool {
        !self.inconsistent && self.parts.len() == usize::from(self.part_count)
    }
}

#[derive(Debug, Clone, Default)]
struct WriterFoldState {
    newest_complete: Option<(u32, Vec<RmwGraphEntry>)>,
    pending: BTreeMap<u32, GenerationBuffer>,
}

/// Pure per-writer generation fold for an rmw graph gather.
///
/// A writer counts as heard only once every part (`0..part_count`) of a single
/// generation has arrived with consistent `part_count`. Across generations from
/// one writer, the newest complete generation wins. When finalized against a
/// `required` writer set, any required writer with no complete generation
/// produces [`GatherCompleteness::Incomplete`] while still contributing entries
/// heard in its newest partial generation.
#[derive(Debug, Clone, Default)]
pub struct RmwGraphGatherFold {
    writers: BTreeMap<u128, WriterFoldState>,
}

impl RmwGraphGatherFold {
    /// Create an empty gather fold.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Record one decoded snapshot part frame.
    pub fn ingest(&mut self, frame: RmwGraphFrame) {
        if frame.part_count == 0 || frame.part_index >= frame.part_count {
            return;
        }
        let writer = self.writers.entry(frame.writer_id).or_default();
        if let Some((complete_gen, _)) = &writer.newest_complete {
            if frame.generation <= *complete_gen {
                return;
            }
        }
        let buf = writer
            .pending
            .entry(frame.generation)
            .or_insert_with(|| GenerationBuffer {
                part_count: frame.part_count,
                inconsistent: false,
                parts: BTreeMap::new(),
            });
        if buf.part_count != frame.part_count {
            buf.inconsistent = true;
            return;
        }
        if buf.inconsistent {
            return;
        }
        buf.parts.entry(frame.part_index).or_insert(frame.entries);
        if buf.is_complete() {
            if let Some(done) = writer.pending.remove(&frame.generation) {
                let entries = done.parts.into_values().flatten().collect();
                writer.newest_complete = Some((frame.generation, entries));
                writer.pending.retain(|&g, _| g > frame.generation);
            }
        }
    }

    /// Whether `writer_id` has delivered at least one complete generation.
    #[must_use]
    pub fn is_writer_complete(&self, writer_id: u128) -> bool {
        self.writers
            .get(&writer_id)
            .is_some_and(|w| w.newest_complete.is_some())
    }

    /// Whether every writer in `required` has delivered a complete generation.
    #[must_use]
    pub fn is_settled(&self, required: &BTreeSet<u128>) -> bool {
        required.iter().all(|id| self.is_writer_complete(*id))
    }

    /// Produce the union of entries and [`GatherCompleteness`] for `required`.
    #[must_use]
    pub fn finish(&self, required: &BTreeSet<u128>) -> RmwGraphGather {
        let mut topics: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
        let mut writers_heard = 0usize;
        for &writer_id in required {
            let Some(writer) = self.writers.get(&writer_id) else {
                continue;
            };
            if let Some((_, entries)) = &writer.newest_complete {
                writers_heard += 1;
                insert_entries(&mut topics, entries);
            } else if let Some((_, partial)) = writer.pending.iter().next_back() {
                for entries in partial.parts.values() {
                    insert_entries(&mut topics, entries);
                }
            }
        }
        let completeness = if writers_heard == required.len() {
            GatherCompleteness::Settled
        } else {
            GatherCompleteness::Incomplete {
                live_writers: required.len(),
                writers_heard,
            }
        };
        RmwGraphGather {
            topics,
            completeness,
        }
    }
}

fn insert_entries(topics: &mut BTreeMap<String, BTreeSet<String>>, entries: &[RmwGraphEntry]) {
    for entry in entries {
        topics
            .entry(entry.topic.clone())
            .or_default()
            .insert(entry.type_name.clone());
    }
}

/// Compute the exact set of remote writer publisher IDs a gather must hear
/// from before returning [`GatherCompleteness::Settled`].
///
/// Each element of `listed` is `(publisher_id, owner_node_id)` from iceoryx2's
/// `dynamic_config().list_publishers(...)`. Publishers whose `owner_node_id` is
/// in `dead_nodes` (observed in `NodeState::Dead`) or whose `publisher_id`
/// equals `own_writer` are excluded. Publishers whose owner node is alive,
/// absent from the node listing, or `Inaccessible` remain required.
#[must_use]
pub fn required_writers<N: Ord>(
    listed: &[(u128, N)],
    dead_nodes: &BTreeSet<N>,
    own_writer: Option<u128>,
) -> BTreeSet<u128> {
    listed
        .iter()
        .filter(|(publisher_id, node_id)| {
            own_writer != Some(*publisher_id) && !dead_nodes.contains(node_id)
        })
        .map(|(publisher_id, _)| *publisher_id)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_header_with(patch: impl FnOnce(&mut [u8])) -> Vec<u8> {
        let mut bytes = vec![0u8; RMW_GRAPH_HEADER_LEN];
        bytes[0..2].copy_from_slice(&RMW_GRAPH_MAGIC);
        bytes[OFF_VERSION] = RMW_GRAPH_VERSION;
        bytes[OFF_WRITER_ID..OFF_WRITER_ID + 16].copy_from_slice(&1u128.to_le_bytes());
        bytes[OFF_GENERATION..OFF_GENERATION + 4].copy_from_slice(&1u32.to_le_bytes());
        bytes[OFF_PART_COUNT..OFF_PART_COUNT + 2].copy_from_slice(&1u16.to_le_bytes());
        patch(&mut bytes);
        bytes
    }

    fn one_entry_wire(topic: &[u8], type_name: &[u8]) -> Vec<u8> {
        let mut buf = valid_header_with(|b| {
            b[OFF_ENTRY_COUNT..OFF_ENTRY_COUNT + 2].copy_from_slice(&1u16.to_le_bytes());
        });
        buf.extend_from_slice(&(topic.len() as u16).to_le_bytes());
        buf.extend_from_slice(&(type_name.len() as u16).to_le_bytes());
        buf.extend_from_slice(topic);
        buf.extend_from_slice(type_name);
        buf
    }

    fn make_frame(
        writer_id: u128,
        generation: u32,
        part_index: u16,
        part_count: u16,
        entries: &[(&str, &str)],
    ) -> RmwGraphFrame {
        RmwGraphFrame {
            writer_id,
            generation,
            part_index,
            part_count,
            entries: entries
                .iter()
                .map(|&(topic, type_name)| RmwGraphEntry {
                    topic: topic.to_string(),
                    type_name: type_name.to_string(),
                })
                .collect(),
        }
    }

    fn topic_map(rows: &[(&str, &[&str])]) -> BTreeMap<String, BTreeSet<String>> {
        rows.iter()
            .map(|&(t, tys)| (t.to_string(), tys.iter().map(|&s| s.to_string()).collect()))
            .collect()
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
        let frames = encode_snapshot(
            writer_id,
            0x1122_3344,
            [
                ("/joint_states", "sensor_msgs/msg/JointState"),
                ("/clock", "rosgraph_msgs/msg/Clock"),
            ],
        )
        .expect("valid snapshot encodes");

        // Sorted order places /clock before /joint_states.
        let mut expected = vec![
            0xCE, 0x52, 0x01, 0x00, // magic, version=1, reserved=0
            0x10, 0x0F, 0x0E, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, // writer_id LE low
            0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, // writer_id LE high
            0x44, 0x33, 0x22, 0x11, // generation LE
            0x00, 0x00, 0x01, 0x00, 0x02, 0x00, // part 0/1, 2 entries
            0x06, 0x00, 0x17, 0x00, // "/clock" (6), "rosgraph_msgs/msg/Clock" (23)
        ];
        expected.extend_from_slice(b"/clockrosgraph_msgs/msg/Clock");
        expected.extend_from_slice(&[0x0D, 0x00, 0x1A, 0x00]); // 13, 26
        expected.extend_from_slice(b"/joint_statessensor_msgs/msg/JointState");

        assert_eq!(frames, vec![expected]);
    }

    #[test]
    fn encode_empty_snapshot_matches_hand_written_zero_entry_part() {
        let writer_id: u128 = 0xAA00_0000_0000_0000_0000_0000_0000_00BB;
        let frames =
            encode_snapshot(writer_id, 7, std::iter::empty()).expect("empty snapshot encodes");

        let expected = vec![
            0xCE, 0x52, 0x01, 0x00, // magic, version, reserved
            0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // writer_id LE low
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAA, // writer_id LE high
            0x07, 0x00, 0x00, 0x00, // generation = 7
            0x00, 0x00, 0x01, 0x00, 0x00, 0x00, // part 0/1, 0 entries
        ];
        assert_eq!(frames, vec![expected]);
    }

    #[test]
    fn decode_hand_written_wire_oracle_and_zero_entry_part() {
        let mut raw = vec![
            0xCE, 0x52, 0x01, 0x00, // magic, version, reserved
            0x2A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // writer_id = 42
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // writer_id high
            0x03, 0x00, 0x00, 0x00, // generation = 3
            0x01, 0x00, 0x02, 0x00, 0x01, 0x00, // part 1/2, 1 entry
            0x07, 0x00, 0x16, 0x00, // topic_len = 7, type_len = 22
        ];
        raw.extend_from_slice(b"/rosoutrcl_interfaces/msg/Log");

        assert_eq!(
            decode_frame(&raw),
            Ok(make_frame(
                42,
                3,
                1,
                2,
                &[("/rosout", "rcl_interfaces/msg/Log")]
            ))
        );

        let empty = valid_header_with(|b| {
            b[OFF_WRITER_ID..OFF_WRITER_ID + 16].copy_from_slice(&99u128.to_le_bytes());
        });
        assert_eq!(decode_frame(&empty), Ok(make_frame(99, 1, 0, 1, &[])));
    }

    #[test]
    fn encode_splits_large_snapshot_across_bounded_parts() {
        // Each entry is 4 + 20 + 26 = 50 bytes. 50 entries = 2500 > 2048 - 30 = 2018,
        // so encoding splits into 40 entries in part 0 and 10 entries in part 1.
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
        assert_eq!(
            (part0.part_index, part0.part_count, part0.entries.len()),
            (0, 2, 40)
        );
        assert_eq!(
            (part1.part_index, part1.part_count, part1.entries.len()),
            (1, 2, 10)
        );
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
        let long_topic = format!("/{}", "a".repeat(MAX_RMW_GRAPH_TOPIC_LEN));
        let long_type = "t".repeat(MAX_RMW_GRAPH_TYPE_LEN + 1);

        assert_eq!(
            encode_snapshot(1, 1, [("/ok", "pkg/msg/Ok"), ("", "std_msgs/msg/String")]),
            Err(RmwGraphFrameError::EmptyTopic)
        );
        assert_eq!(
            encode_snapshot(1, 1, [("/a", "")]),
            Err(RmwGraphFrameError::EmptyType)
        );
        assert_eq!(
            encode_snapshot(1, 1, [(long_topic.as_str(), "std_msgs/msg/String")]),
            Err(RmwGraphFrameError::TopicTooLong {
                len: MAX_RMW_GRAPH_TOPIC_LEN + 1
            })
        );
        assert_eq!(
            encode_snapshot(1, 1, [("/a", long_type.as_str())]),
            Err(RmwGraphFrameError::TypeTooLong {
                len: MAX_RMW_GRAPH_TYPE_LEN + 1
            })
        );
    }

    #[test]
    fn decode_refuses_every_malformed_wire_shape() {
        let mut trunc_body = valid_header_with(|b| {
            b[OFF_ENTRY_COUNT..OFF_ENTRY_COUNT + 2].copy_from_slice(&1u16.to_le_bytes());
        });
        trunc_body.extend_from_slice(&[4, 0, 4, 0, b'/', b'a', b'b', b'c']);

        let mut over_topic = valid_header_with(|b| {
            b[OFF_ENTRY_COUNT..OFF_ENTRY_COUNT + 2].copy_from_slice(&1u16.to_le_bytes());
        });
        over_topic.extend_from_slice(&((MAX_RMW_GRAPH_TOPIC_LEN + 1) as u16).to_le_bytes());
        over_topic.extend_from_slice(&1u16.to_le_bytes());

        let mut over_type = valid_header_with(|b| {
            b[OFF_ENTRY_COUNT..OFF_ENTRY_COUNT + 2].copy_from_slice(&1u16.to_le_bytes());
        });
        over_type.extend_from_slice(&2u16.to_le_bytes());
        over_type.extend_from_slice(&((MAX_RMW_GRAPH_TYPE_LEN + 1) as u16).to_le_bytes());

        let mut trailing = valid_header_with(|_| {});
        trailing.extend_from_slice(&[0xDE, 0xAD]);

        let cases: Vec<(Vec<u8>, RmwGraphFrameError)> = vec![
            (
                vec![0xCE, 0x52, 0x01],
                RmwGraphFrameError::TruncatedHeader { len: 3 },
            ),
            (
                vec![0u8; RMW_GRAPH_FRAME_LEN + 1],
                RmwGraphFrameError::FrameTooLarge {
                    len: RMW_GRAPH_FRAME_LEN + 1,
                },
            ),
            (
                valid_header_with(|b| b[0..2].copy_from_slice(&[0xCE, 0x37])),
                RmwGraphFrameError::BadMagic,
            ),
            (
                valid_header_with(|b| b[0..2].copy_from_slice(&[0xCE, 0x81])),
                RmwGraphFrameError::BadMagic,
            ),
            (
                valid_header_with(|b| b[OFF_VERSION] = 2),
                RmwGraphFrameError::BadVersion { found: 2 },
            ),
            (
                valid_header_with(|b| b[OFF_RESERVED] = 0xFF),
                RmwGraphFrameError::BadReserved { found: 0xFF },
            ),
            (
                valid_header_with(|b| {
                    b[OFF_PART_COUNT..OFF_PART_COUNT + 2].copy_from_slice(&0u16.to_le_bytes());
                }),
                RmwGraphFrameError::InvalidPartBounds {
                    part_index: 0,
                    part_count: 0,
                },
            ),
            (
                valid_header_with(|b| {
                    b[OFF_PART_INDEX..OFF_PART_INDEX + 2].copy_from_slice(&2u16.to_le_bytes());
                    b[OFF_PART_COUNT..OFF_PART_COUNT + 2].copy_from_slice(&2u16.to_le_bytes());
                }),
                RmwGraphFrameError::InvalidPartBounds {
                    part_index: 2,
                    part_count: 2,
                },
            ),
            // Hostile entry_count = u16::MAX with no entry bytes must fail before allocating.
            (
                valid_header_with(|b| {
                    b[OFF_ENTRY_COUNT..OFF_ENTRY_COUNT + 2]
                        .copy_from_slice(&u16::MAX.to_le_bytes());
                }),
                RmwGraphFrameError::TruncatedEntry { entry_index: 0 },
            ),
            (
                trunc_body,
                RmwGraphFrameError::TruncatedEntry { entry_index: 0 },
            ),
            (
                one_entry_wire(b"", b"std_msgs/msg/String"),
                RmwGraphFrameError::EmptyTopic,
            ),
            (one_entry_wire(b"/a", b""), RmwGraphFrameError::EmptyType),
            (
                over_topic,
                RmwGraphFrameError::TopicTooLong {
                    len: MAX_RMW_GRAPH_TOPIC_LEN + 1,
                },
            ),
            (
                over_type,
                RmwGraphFrameError::TypeTooLong {
                    len: MAX_RMW_GRAPH_TYPE_LEN + 1,
                },
            ),
            (
                one_entry_wire(&[0xFF, 0xFE], b"pkg/msg/T"),
                RmwGraphFrameError::InvalidUtf8,
            ),
            (
                one_entry_wire(b"/ok", &[0x80, 0x81]),
                RmwGraphFrameError::InvalidUtf8,
            ),
            (trailing, RmwGraphFrameError::TrailingBytes { trailing: 2 }),
        ];

        for (raw, expected) in cases {
            assert_eq!(decode_frame(&raw), Err(expected));
        }
    }

    #[test]
    fn encode_and_decode_accept_exact_field_and_frame_boundaries() {
        let max_topic = format!("/{}", "a".repeat(MAX_RMW_GRAPH_TOPIC_LEN - 1));
        let max_type = "b".repeat(MAX_RMW_GRAPH_TYPE_LEN);
        let frames = encode_snapshot(77, 1, [(max_topic.as_str(), max_type.as_str())])
            .expect("exact max field lengths encode");
        assert_eq!(frames.len(), 1);
        assert_eq!(
            decode_frame(&frames[0])
                .expect("exact max lengths decode")
                .entries,
            vec![RmwGraphEntry {
                topic: max_topic,
                type_name: max_type,
            }]
        );

        // Exact RMW_GRAPH_FRAME_LEN (2048): 30 + 3*(4+250+250) + (4+251+251) = 2048.
        let t0 = format!("/0{}", "x".repeat(248));
        let t1 = format!("/1{}", "x".repeat(248));
        let t2 = format!("/2{}", "x".repeat(248));
        let t3 = format!("/3{}", "x".repeat(249));
        let ty250 = "y".repeat(250);
        let ty251 = "y".repeat(251);
        let exact_frames = encode_snapshot(
            88,
            2,
            [
                (t0.as_str(), ty250.as_str()),
                (t1.as_str(), ty250.as_str()),
                (t2.as_str(), ty250.as_str()),
                (t3.as_str(), ty251.as_str()),
            ],
        )
        .expect("exact 2048-byte frame encodes");
        assert_eq!(exact_frames.len(), 1);
        assert_eq!(exact_frames[0].len(), RMW_GRAPH_FRAME_LEN);
        assert_eq!(
            decode_frame(&exact_frames[0])
                .expect("2048B decodes")
                .entries
                .len(),
            4
        );
    }

    #[test]
    fn required_writers_filters_dead_and_own_and_keeps_absent_owners() {
        let listed = [
            (10u128, 100u64), // alive owner
            (20, 200),        // dead owner
            (30, 300),        // absent or inaccessible owner: stays required
            (40, 400),        // own writer
            (50, 200),        // second publisher on dead owner
        ];
        let dead = BTreeSet::from([200u64, 999]);

        assert_eq!(
            required_writers(&listed, &dead, Some(40)),
            BTreeSet::from([10, 30])
        );
        assert_eq!(
            required_writers(&listed, &dead, None),
            BTreeSet::from([10, 30, 40])
        );
        assert_eq!(
            required_writers(&[(40u128, 400u64)], &BTreeSet::new(), Some(40)),
            BTreeSet::new()
        );
    }

    #[test]
    fn fold_two_part_generation_settles_only_when_both_parts_arrive() {
        let required = BTreeSet::from([10u128]);
        let mut fold = RmwGraphGatherFold::new();

        fold.ingest(make_frame(
            10,
            1,
            1,
            2,
            &[("/joint_states", "sensor_msgs/msg/JointState")],
        ));
        assert!(!fold.is_writer_complete(10));
        assert!(!fold.is_settled(&required));
        assert_eq!(
            fold.finish(&required),
            RmwGraphGather {
                topics: topic_map(&[("/joint_states", &["sensor_msgs/msg/JointState"])]),
                completeness: GatherCompleteness::Incomplete {
                    live_writers: 1,
                    writers_heard: 0,
                },
            }
        );

        fold.ingest(make_frame(
            10,
            1,
            0,
            2,
            &[("/clock", "rosgraph_msgs/msg/Clock")],
        ));
        assert!(fold.is_writer_complete(10));
        assert!(fold.is_settled(&required));
        assert_eq!(
            fold.finish(&required),
            RmwGraphGather {
                topics: topic_map(&[
                    ("/clock", &["rosgraph_msgs/msg/Clock"]),
                    ("/joint_states", &["sensor_msgs/msg/JointState"]),
                ]),
                completeness: GatherCompleteness::Settled,
            }
        );
    }

    #[test]
    fn fold_lost_part_reports_incomplete_and_keeps_heard_entries() {
        let required = BTreeSet::from([10u128, 20]);
        let mut fold = RmwGraphGatherFold::new();

        fold.ingest(make_frame(10, 1, 0, 1, &[("/a", "pkg/msg/A")]));
        fold.ingest(make_frame(20, 1, 0, 2, &[("/stale", "pkg/msg/Stale")]));
        fold.ingest(make_frame(20, 2, 0, 2, &[("/b", "pkg/msg/B")]));

        assert_eq!(
            fold.finish(&required),
            RmwGraphGather {
                topics: topic_map(&[("/a", &["pkg/msg/A"]), ("/b", &["pkg/msg/B"])]),
                completeness: GatherCompleteness::Incomplete {
                    live_writers: 2,
                    writers_heard: 1,
                },
            }
        );
    }

    #[test]
    fn fold_uses_newest_complete_generation_and_handles_withdrawal() {
        let required = BTreeSet::from([10u128]);
        let mut fold = RmwGraphGatherFold::new();

        // Incomplete gen 1 is superseded once gen 2 completes.
        fold.ingest(make_frame(10, 1, 0, 2, &[("/partial1", "pkg/msg/P1")]));
        fold.ingest(make_frame(10, 2, 0, 1, &[("/old", "pkg/msg/Old")]));
        // Gen 3 starts (part 0 of 2) but does not complete: gen 2 stays authoritative.
        fold.ingest(make_frame(10, 3, 0, 2, &[("/mid", "pkg/msg/Mid")]));
        assert_eq!(
            fold.finish(&required),
            RmwGraphGather {
                topics: topic_map(&[("/old", &["pkg/msg/Old"])]),
                completeness: GatherCompleteness::Settled,
            }
        );

        // Gen 3 completes and supersedes gen 2; delayed stale gen 2 is ignored.
        fold.ingest(make_frame(10, 3, 1, 2, &[("/new", "pkg/msg/New")]));
        fold.ingest(make_frame(10, 2, 0, 1, &[("/old", "pkg/msg/Old")]));
        assert_eq!(
            fold.finish(&required),
            RmwGraphGather {
                topics: topic_map(&[("/mid", &["pkg/msg/Mid"]), ("/new", &["pkg/msg/New"])]),
                completeness: GatherCompleteness::Settled,
            }
        );

        // Gen 4 is an empty snapshot (writer removed its last endpoint).
        fold.ingest(make_frame(10, 4, 0, 1, &[]));
        assert_eq!(
            fold.finish(&required),
            RmwGraphGather {
                topics: BTreeMap::new(),
                completeness: GatherCompleteness::Settled,
            }
        );
    }

    #[test]
    fn fold_rejects_conflicting_part_counts_and_invalid_bounds() {
        let required = BTreeSet::from([10u128]);
        let mut fold = RmwGraphGatherFold::new();

        fold.ingest(make_frame(10, 1, 0, 0, &[("/bad", "pkg/msg/Bad")]));
        fold.ingest(make_frame(10, 1, 1, 1, &[("/bad", "pkg/msg/Bad")]));
        assert!(!fold.is_writer_complete(10));

        // Conflicting part_count for the same generation must not settle.
        fold.ingest(make_frame(10, 2, 0, 2, &[("/ok", "pkg/msg/Ok")]));
        fold.ingest(make_frame(10, 2, 0, 1, &[("/spoof", "pkg/msg/Spoof")]));
        fold.ingest(make_frame(10, 2, 1, 2, &[("/late", "pkg/msg/Late")]));
        assert!(!fold.is_writer_complete(10));
        assert_eq!(
            fold.finish(&required),
            RmwGraphGather {
                topics: topic_map(&[("/ok", &["pkg/msg/Ok"])]),
                completeness: GatherCompleteness::Incomplete {
                    live_writers: 1,
                    writers_heard: 0,
                },
            }
        );
    }

    #[test]
    fn fold_empty_required_set_settles_and_excludes_unrequired_writers() {
        let mut fold = RmwGraphGatherFold::new();
        fold.ingest(make_frame(99, 1, 0, 1, &[("/dead", "pkg/msg/Dead")]));

        assert!(fold.is_settled(&BTreeSet::new()));
        assert_eq!(
            fold.finish(&BTreeSet::new()),
            RmwGraphGather {
                topics: BTreeMap::new(),
                completeness: GatherCompleteness::Settled,
            }
        );

        let required = BTreeSet::from([10u128]);
        assert!(!fold.is_settled(&required));
        assert_eq!(
            fold.finish(&required),
            RmwGraphGather {
                topics: BTreeMap::new(),
                completeness: GatherCompleteness::Incomplete {
                    live_writers: 1,
                    writers_heard: 0,
                },
            }
        );
    }

    #[test]
    fn fold_multi_writer_multi_type_is_deterministic_across_arrival_orders() {
        let required = BTreeSet::from([10u128, 20]);
        let frames = vec![
            make_frame(10, 1, 0, 2, &[("/shared", "pkg/msg/ZType")]),
            make_frame(10, 1, 1, 2, &[("/only_one", "pkg/msg/One")]),
            make_frame(
                20,
                4,
                0,
                1,
                &[("/shared", "pkg/msg/AType"), ("/only_two", "pkg/msg/Two")],
            ),
            make_frame(10, 1, 0, 2, &[("/shared", "pkg/msg/ZType")]),
        ];

        let mut forward = RmwGraphGatherFold::new();
        for f in frames.iter().cloned() {
            forward.ingest(f);
        }
        let mut reverse = RmwGraphGatherFold::new();
        for f in frames.into_iter().rev() {
            reverse.ingest(f);
        }

        let expected = RmwGraphGather {
            topics: topic_map(&[
                ("/only_one", &["pkg/msg/One"]),
                ("/only_two", &["pkg/msg/Two"]),
                ("/shared", &["pkg/msg/AType", "pkg/msg/ZType"]),
            ]),
            completeness: GatherCompleteness::Settled,
        };
        assert_eq!(forward.finish(&required), expected);
        assert_eq!(reverse.finish(&required), expected);
    }
}
