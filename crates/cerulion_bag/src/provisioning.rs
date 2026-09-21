// SPDX-License-Identifier: AGPL-3.0-only
//! The per-channel PROVISIONING tuple, carried in the MCAP
//! Channel record's `metadata` map.
//!
//! # Why it exists
//!
//! Per-channel metadata in a Cerulion bag was, until now, exactly
//! `ChannelInfo { topic, schema_id }` — the topic name and a pointer to its
//! schema. Nothing recorded how the topic was PROVISIONED, so a reader
//! replaying or PLAYING the bag had to guess: `cerulion bag play` sizes a
//! played topic's `max_slice_len` from the largest payload it happens to
//! observe in the bag, which is a lower bound on what the original producer
//! could emit, not the value it was created with.
//!
//! Play fidelity needs the real numbers; the extension lands ONCE,
//! here, so the record side and the play side, which share it, cannot ship two
//! spellings of the same metadata.
//!
//! # WHO WRITES IT TODAY: nobody, and that is disclosed rather than implied
//!
//! This module is the FORMAT plus its reader and writer. No shipping producer
//! populates it yet: `cerulion_bagd` builds its `BagWriterConfig` through
//! `Default`, so every bag it writes carries an empty provisioning map — which
//! is exactly the byte-identical case above, not a silent loss. It could not do
//! better if it tried: a recorder taps a topic through a listener-less
//! `DataOnlySubscriber`, which exposes `max_borrowed_samples` and none of the
//! four fields here, so the numbers a recorder would need are not on the seam it
//! observes through.
//!
//! Populating it has to come from the side that
//! CREATED the service and therefore knows what it asked for.
//!
//! # Additive in both directions, by construction
//!
//! The MCAP Channel record already carries a `metadata` map (the writer emitted
//! an EMPTY one on every channel before this module existed), so:
//!
//! * An OLD reader parses the map and ignores keys it does not know — every
//!   MCAP reader must, since the map is open by specification.
//! * A NEW reader on an OLD bag reads an empty map and gets
//!   [`ChannelProvisioning::default()`], i.e. every field `None` — "not
//!   recorded", which is the accurate reading rather than a fabricated default.
//! * A channel with NOTHING to declare emits an empty map, so a bag whose
//!   producer records no provisioning is BYTE-IDENTICAL to one written before
//!   this module existed.
//!
//! That last property is what keeps `cerulion_bag`'s byte-determinism gate
//! meaningful across this change, and it is pinned by a test rather than
//! asserted here.
//!
//! # Key namespace and determinism
//!
//! Keys are namespaced [`PROVISIONING_KEY_PREFIX`] so they can never collide
//! with a foreign writer's metadata, and they are emitted in sorted key order
//! (the [`ChannelProvisioning::to_metadata`] contract) so one input produces one
//! byte sequence — `cerulion_bag` reads no clock and must not depend on map
//! iteration order either.

use std::collections::BTreeMap;

/// The namespace every provisioning metadata key carries.
///
/// A foreign MCAP writer's metadata is therefore never mistaken for ours, and
/// ours is obviously ours when a human reads the bag with a generic tool.
pub const PROVISIONING_KEY_PREFIX: &str = "cerulion.";

/// Metadata key: [`ChannelProvisioning::buffer_depth`].
pub const KEY_BUFFER_DEPTH: &str = "cerulion.buffer_depth";
/// Metadata key: [`ChannelProvisioning::max_slice_len`].
pub const KEY_MAX_SLICE_LEN: &str = "cerulion.max_slice_len";
/// Metadata key: [`ChannelProvisioning::history_depth`].
pub const KEY_HISTORY_DEPTH: &str = "cerulion.history_depth";
/// Metadata key: [`ChannelProvisioning::latched`].
pub const KEY_LATCHED: &str = "cerulion.latched";

/// How one recorded topic's transport was
/// PROVISIONED, as the recorder observed it.
///
/// Every field is `Option` and every `None` means exactly "not recorded" — this
/// type never fabricates a default on a producer's behalf. A reader that needs
/// to know whether a value is real must read the `Option`, not a sentinel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ChannelProvisioning {
    /// The subscriber buffer depth (queue slots) the topic's service was
    /// created with.
    pub buffer_depth: Option<u32>,
    /// The topic's `max_slice_len` — the largest payload its publisher may
    /// loan, in bytes. This is the value the play path must reproduce
    /// rather than infer from observed payloads.
    pub max_slice_len: Option<u32>,
    /// The publisher's retained-history depth (late-joiner delivery).
    pub history_depth: Option<u32>,
    /// Whether the topic is latched (a retained last value served to late
    /// joiners) — the `/tf_static` shape.
    pub latched: Option<bool>,
}

impl ChannelProvisioning {
    /// Whether this carries NOTHING — the state that emits an empty metadata
    /// map and so keeps a bag byte-identical to one written before this
    /// module existed.
    pub fn is_empty(&self) -> bool {
        *self == Self::default()
    }

    /// Render to sorted `(key, value)` metadata pairs.
    ///
    /// SORTED (and therefore deterministic) because the pairs come out of a
    /// `BTreeMap`; an absent field contributes NO key, so "not recorded" is
    /// represented by absence rather than by a magic value a reader could
    /// mistake for a measurement.
    pub fn to_metadata(&self) -> Vec<(String, String)> {
        let mut map: BTreeMap<&'static str, String> = BTreeMap::new();
        if let Some(v) = self.buffer_depth {
            map.insert(KEY_BUFFER_DEPTH, v.to_string());
        }
        if let Some(v) = self.max_slice_len {
            map.insert(KEY_MAX_SLICE_LEN, v.to_string());
        }
        if let Some(v) = self.history_depth {
            map.insert(KEY_HISTORY_DEPTH, v.to_string());
        }
        if let Some(v) = self.latched {
            map.insert(KEY_LATCHED, if v { "true" } else { "false" }.to_string());
        }
        map.into_iter().map(|(k, v)| (k.to_string(), v)).collect()
    }

    /// Read back from a Channel record's metadata map.
    ///
    /// TOLERANT of everything it does not own: unknown keys are ignored (the
    /// MCAP metadata map is open by specification, so refusing them would make
    /// a foreign writer's bag unreadable), and a key of ours whose value does
    /// not parse yields `None` for THAT field with a `warn!` — never a
    /// fabricated number, and never a refusal that would lose the rest of the
    /// bag over one bad string.
    pub fn from_metadata<'a, I, K, V>(pairs: I) -> Self
    where
        I: IntoIterator<Item = (K, V)>,
        K: AsRef<str> + 'a,
        V: AsRef<str> + 'a,
    {
        let mut out = Self::default();
        for (k, v) in pairs {
            let (key, value) = (k.as_ref(), v.as_ref());
            match key {
                KEY_BUFFER_DEPTH => out.buffer_depth = parse_u32(key, value),
                KEY_MAX_SLICE_LEN => out.max_slice_len = parse_u32(key, value),
                KEY_HISTORY_DEPTH => out.history_depth = parse_u32(key, value),
                KEY_LATCHED => out.latched = parse_bool(key, value),
                _ => {}
            }
        }
        out
    }
}

fn parse_u32(key: &str, value: &str) -> Option<u32> {
    match value.parse::<u32>() {
        Ok(v) => Some(v),
        Err(e) => {
            tracing::warn!(
                key,
                value,
                error = %e,
                "cerulion_bag: channel provisioning metadata value is not a whole number — \
                 reading this field as NOT RECORDED rather than inventing one"
            );
            None
        }
    }
}

fn parse_bool(key: &str, value: &str) -> Option<bool> {
    match value {
        "true" => Some(true),
        "false" => Some(false),
        other => {
            tracing::warn!(
                key,
                value = other,
                "cerulion_bag: channel provisioning metadata value is not `true`/`false` — \
                 reading this field as NOT RECORDED rather than inventing one"
            );
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_default_provisioning_emits_no_metadata_at_all() {
        // The byte-identity property: nothing to declare ⇒ nothing on the wire.
        let p = ChannelProvisioning::default();
        assert!(p.is_empty());
        assert_eq!(p.to_metadata(), Vec::<(String, String)>::new());
    }

    #[test]
    fn every_field_round_trips_through_the_metadata_map() {
        // A hand-written oracle: each field carries a DISTINCT value, so a
        // swapped pair of keys cannot pass.
        let p = ChannelProvisioning {
            buffer_depth: Some(4096),
            max_slice_len: Some(65_536),
            history_depth: Some(7),
            latched: Some(true),
        };
        let pairs = p.to_metadata();
        assert_eq!(
            pairs,
            vec![
                ("cerulion.buffer_depth".to_string(), "4096".to_string()),
                ("cerulion.history_depth".to_string(), "7".to_string()),
                ("cerulion.latched".to_string(), "true".to_string()),
                ("cerulion.max_slice_len".to_string(), "65536".to_string()),
            ],
            "keys must render SORTED — the byte-determinism contract"
        );
        assert_eq!(ChannelProvisioning::from_metadata(pairs), p);
    }

    #[test]
    fn a_partially_declared_provisioning_omits_the_fields_it_does_not_know() {
        let p = ChannelProvisioning {
            max_slice_len: Some(1024),
            latched: Some(false),
            ..Default::default()
        };
        assert_eq!(
            p.to_metadata(),
            vec![
                ("cerulion.latched".to_string(), "false".to_string()),
                ("cerulion.max_slice_len".to_string(), "1024".to_string()),
            ]
        );
        assert_eq!(ChannelProvisioning::from_metadata(p.to_metadata()), p);
        assert!(!p.is_empty());
    }

    #[test]
    fn an_old_bags_empty_metadata_map_reads_as_not_recorded() {
        // The shape before this module existed: every channel carried an EMPTY map.
        let read = ChannelProvisioning::from_metadata(Vec::<(String, String)>::new());
        assert_eq!(read, ChannelProvisioning::default());
        assert!(read.is_empty());
    }

    #[test]
    fn a_foreign_writers_metadata_is_ignored_not_refused() {
        let read = ChannelProvisioning::from_metadata(vec![
            ("ros2.qos".to_string(), "reliable".to_string()),
            ("cerulion.max_slice_len".to_string(), "77".to_string()),
            ("topic_owner".to_string(), "someone_else".to_string()),
        ]);
        assert_eq!(read.max_slice_len, Some(77));
        assert_eq!(read.buffer_depth, None);
    }

    #[test]
    fn an_unparseable_value_reads_as_not_recorded_never_as_a_number() {
        let read = ChannelProvisioning::from_metadata(vec![
            ("cerulion.buffer_depth".to_string(), "lots".to_string()),
            ("cerulion.latched".to_string(), "yes".to_string()),
            ("cerulion.max_slice_len".to_string(), "-1".to_string()),
        ]);
        assert_eq!(read, ChannelProvisioning::default());
    }
}
