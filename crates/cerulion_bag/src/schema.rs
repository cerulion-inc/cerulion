// SPDX-License-Identifier: AGPL-3.0-only
//! The `cerulion` MCAP schema encoding.
//!
//! Every Cerulion channel names an MCAP Schema record whose `encoding` field is
//! the fixed string [`SCHEMA_ENCODING`] (`"cerulion"`) and whose `data` field is
//! a small, versioned, little-endian binary descriptor:
//!
//! ```text
//! offset  size  field                type   value
//! 0       2     descriptor_version   u16    = 1  (DESCRIPTOR_VERSION)
//! 2       4     hash_recipe          u32    = cerulion_core::trace::bag::HASH_RECIPE (3)
//! 6       8     schema_hash          u64    the layout-sensitive schema hash
//! 14      4     wire_fixed_size      u32    the schema's fixed wire size in bytes
//! ```
//!
//! Total: [`DESCRIPTOR_LEN`] (18) bytes. The descriptor carries only what a
//! replayer needs to gate wire compatibility (recipe + hash + fixed size); the
//! qualified schema NAME lives in the Schema record's `name` field, not here.

use cerulion_core::trace::bag::HASH_RECIPE;

use crate::error::{BagError, BagResult};

/// The MCAP Schema `encoding` string for every Cerulion channel.
pub const SCHEMA_ENCODING: &str = "cerulion";

/// The current [`SchemaDescriptor`] wire version.
pub const DESCRIPTOR_VERSION: u16 = 1;

/// The exact byte length of an encoded [`SchemaDescriptor`].
pub const DESCRIPTOR_LEN: usize = 18;

/// The reserved topic-name prefix owned by the auto-registered channels. A user
/// topic starting with this is rejected (see [`BagError::ReservedTopicPrefix`]).
pub const RESERVED_PREFIX: &str = "__cerulion/";

/// The scheduler-trace reserved channel topic.
pub const SCHEDULER_TRACE_TOPIC: &str = "__cerulion/scheduler_trace";
/// The scheduler-trace reserved channel schema name.
pub const SCHEDULER_TRACE_SCHEMA: &str = "cerulion.SchedulerTrace";
/// The non-determinism reserved channel topic.
pub const NONDETERMINISM_TOPIC: &str = "__cerulion/nondeterminism";
/// The non-determinism reserved channel schema name.
pub const NONDETERMINISM_SCHEMA: &str = "cerulion.NonDeterminism";

/// The node-state CHECKPOINT reserved channel topic.
///
/// **The slash-less spelling is load-bearing.** `UserFrameWalk::next_user_frame`
/// tests [`RESERVED_PREFIX`] (`__cerulion/`, no leading slash), so a
/// `/__cerulion/state` spelling would NOT be skipped: it would reach replay's
/// `classify_topics` as a topic the graph neither produces nor consumes and make
/// every checkpointed bag fail `BagGraphMismatch` — "corrupt or hand-edited
/// recording", exit 2, on a bag that is neither.
pub const STATE_TOPIC: &str = "__cerulion/state";
/// The node-state checkpoint reserved channel schema name.
pub const STATE_SCHEMA: &str = "cerulion.State";

/// The record-time PRODUCER-LABEL reserved channel topic.
///
/// A `multi_publisher_topics` topic carries frames from several publishers on
/// ONE channel, and nothing in the wire frame says which one committed a given
/// frame — the header's `sequence` is a PER-PUBLISHER counter, so on replay the
/// interleaving is unrecoverable from the bag's user channels alone. This
/// channel carries the labels the recorder observed at record time (see
/// [`crate::producers`]), which is the only vantage that can still tell them
/// apart.
///
/// **The slash-less spelling is load-bearing**, exactly as it is for
/// [`STATE_TOPIC`]: `UserFrameWalk::next_user_frame` tests [`RESERVED_PREFIX`]
/// (`__cerulion/`, no leading slash), so a `/__cerulion/frame_producers`
/// spelling would NOT be skipped and would reach replay's `classify_topics` as
/// a topic the graph neither produces nor consumes.
///
/// Record-only for an old reader: replay skips every `__cerulion/*` topic, so a
/// bag carrying this channel replays on an older build exactly as it did
/// before — the labels are simply not consulted.
pub const FRAME_PRODUCERS_TOPIC: &str = "__cerulion/frame_producers";
/// The producer-label reserved channel schema name.
pub const FRAME_PRODUCERS_SCHEMA: &str = "cerulion.FrameProducers";

/// A decoded `cerulion` schema descriptor (the Schema record's `data` blob).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SchemaDescriptor {
    /// The descriptor wire version. Written as [`DESCRIPTOR_VERSION`].
    pub descriptor_version: u16,
    /// The schema-hash recipe id (see `cerulion_core::trace::bag::HASH_RECIPE`).
    /// Written as the current [`HASH_RECIPE`].
    pub hash_recipe: u32,
    /// The layout-sensitive schema hash (0 for the scheduler-trace stream, whose
    /// payload is the fixed 40-byte `TraceRingRecord`).
    pub schema_hash: u64,
    /// The schema's fixed wire size in bytes.
    ///
    /// **`0` is ambiguous on the wire and always has been**: it is what a
    /// purely-VARIABLE schema genuinely has, AND what a recorder writes
    /// when nothing could size the channel. The descriptor has no encoding
    /// for the difference, so a reader must not infer "variable" from a
    /// zero here. The recorder keeps the distinction on its own side (an
    /// `Option`, reported at the recording seam — see
    /// `graph_cmd::resolve_recorded_wire_size`) precisely
    /// because it cannot survive this field.
    pub wire_fixed_size: u32,
}

impl SchemaDescriptor {
    /// Build a descriptor for a user schema, stamping the current
    /// [`DESCRIPTOR_VERSION`] and [`HASH_RECIPE`].
    pub fn new(schema_hash: u64, wire_fixed_size: u32) -> Self {
        Self {
            descriptor_version: DESCRIPTOR_VERSION,
            hash_recipe: HASH_RECIPE,
            schema_hash,
            wire_fixed_size,
        }
    }

    /// Encode to the fixed [`DESCRIPTOR_LEN`]-byte little-endian blob.
    pub fn encode(&self) -> [u8; DESCRIPTOR_LEN] {
        let mut b = [0u8; DESCRIPTOR_LEN];
        b[0..2].copy_from_slice(&self.descriptor_version.to_le_bytes());
        b[2..6].copy_from_slice(&self.hash_recipe.to_le_bytes());
        b[6..14].copy_from_slice(&self.schema_hash.to_le_bytes());
        b[14..18].copy_from_slice(&self.wire_fixed_size.to_le_bytes());
        b
    }

    /// Decode from a Schema record `data` blob. Rejects a wrong length or an
    /// unknown [`descriptor_version`](Self::descriptor_version).
    pub fn decode(data: &[u8]) -> BagResult<Self> {
        if data.len() != DESCRIPTOR_LEN {
            return Err(BagError::SchemaDescriptor {
                reason: format!(
                    "descriptor is {} bytes, expected exactly {DESCRIPTOR_LEN}",
                    data.len()
                ),
            });
        }
        let descriptor_version = u16::from_le_bytes([data[0], data[1]]);
        if descriptor_version != DESCRIPTOR_VERSION {
            return Err(BagError::SchemaDescriptor {
                reason: format!(
                    "unknown descriptor_version {descriptor_version} (this build writes/reads v{DESCRIPTOR_VERSION})"
                ),
            });
        }
        let hash_recipe = u32::from_le_bytes(data[2..6].try_into().unwrap());
        let schema_hash = u64::from_le_bytes(data[6..14].try_into().unwrap());
        let wire_fixed_size = u32::from_le_bytes(data[14..18].try_into().unwrap());
        Ok(Self {
            descriptor_version,
            hash_recipe,
            schema_hash,
            wire_fixed_size,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cerulion_core::trace_ring::TRACE_RECORD_SIZE;

    #[test]
    fn descriptor_byte_oracle() {
        // Distinct, byte-recognisable field values.
        let d = SchemaDescriptor {
            descriptor_version: 1,
            hash_recipe: 3,
            schema_hash: 0x0102_0304_0506_0708,
            wire_fixed_size: 0x1112_1314,
        };
        let expected: [u8; DESCRIPTOR_LEN] = [
            0x01, 0x00, // descriptor_version = 1
            0x03, 0x00, 0x00, 0x00, // hash_recipe = 3
            0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, // schema_hash LE
            0x14, 0x13, 0x12, 0x11, // wire_fixed_size LE
        ];
        assert_eq!(d.encode(), expected, "descriptor byte layout is a contract");
    }

    #[test]
    fn descriptor_roundtrip_is_identity() {
        let d = SchemaDescriptor::new(0xDEAD_BEEF_CAFE_F00D, 1234);
        assert_eq!(SchemaDescriptor::decode(&d.encode()).unwrap(), d);
        assert_eq!(d.hash_recipe, HASH_RECIPE);
        assert_eq!(d.descriptor_version, DESCRIPTOR_VERSION);
    }

    #[test]
    fn scheduler_trace_descriptor_is_hash0_size40() {
        // The scheduler-trace stream: schema_hash 0, wire_fixed_size 40.
        let d = SchemaDescriptor::new(0, TRACE_RECORD_SIZE);
        assert_eq!(d.schema_hash, 0);
        assert_eq!(d.wire_fixed_size, 40);
        assert_eq!(TRACE_RECORD_SIZE, 40);
    }

    #[test]
    fn decode_rejects_bad_length_and_version() {
        assert!(matches!(
            SchemaDescriptor::decode(&[0u8; 17]),
            Err(BagError::SchemaDescriptor { .. })
        ));
        assert!(matches!(
            SchemaDescriptor::decode(&[0u8; 19]),
            Err(BagError::SchemaDescriptor { .. })
        ));
        // Right length, wrong version (99).
        let mut bad = SchemaDescriptor::new(1, 2).encode();
        bad[0] = 99;
        bad[1] = 0;
        assert!(matches!(
            SchemaDescriptor::decode(&bad),
            Err(BagError::SchemaDescriptor { .. })
        ));
    }
}
