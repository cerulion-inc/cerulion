// SPDX-License-Identifier: AGPL-3.0-only
//! The two sibling topic sets a split worker is told about, one type per
//! DIRECTION.
//!
//! A worker of a multi-process deployment is handed two sets of absolute topic
//! names, and they answer opposite questions: which topics a SIBLING group
//! produces for it ([`SiblingProducedTopics`], its cross-group inputs), and which
//! of its own outputs a sibling group CONSUMES ([`SiblingConsumedTopics`]).
//!
//! As two `BTreeSet<String>` side by side, one passed where the other belongs
//! type-checks and then answers the other question quietly: validation is told
//! the topics this worker WRITES for a sibling when what it needs is the ones a
//! sibling writes for it, and every correct cross-group edge then draws the "is
//! this a typo?" warning. It adds warnings rather than losing them, since each
//! outbound member has a producer in this worker's own view and so never reaches
//! the in-prefix miss branch at all.
//!
//! Nothing in the values says which direction they came from, so the direction is
//! carried by the type: a parameter or field of the other direction is a compile
//! error. One site is not covered by that, the pair read out of
//! `CrossProcessWiring` where both expose `contains`, and
//! `a_publisher_arms_a_doorbell_only_for_the_output_a_sibling_reads` is what holds
//! it.
//!
//! Both are `#[serde(transparent)]`: a worker plan file's bytes are the bare set
//! either way, so a plan written before these types parses unchanged.

use std::collections::BTreeSet;

/// One direction's set of absolute topic names: the two reads its callers make,
/// the `FromIterator` the derivations collect through, and borrowed iteration.
///
/// No `From<BTreeSet<String>>` and no by-value `IntoIterator`: nothing needs
/// them, and a `From` for a bare set would let an inferred `into()` build either
/// direction from the same value, which is the hazard these types exist to close.
macro_rules! topic_set_newtype {
    ($name:ident) => {
        impl $name {
            /// Does this set name `topic`?
            pub fn contains(&self, topic: &str) -> bool {
                self.0.contains(topic)
            }

            /// Does the set name nothing? An empty set is the ordinary
            /// single-process answer, not a missing one.
            ///
            /// No `len` beside it: no reader needs a count, and the two readers
            /// that exist ask membership or emptiness.
            pub fn is_empty(&self) -> bool {
                self.0.is_empty()
            }

            /// The topics in sorted order, which is the order the plan file
            /// carries them in.
            pub fn iter(&self) -> std::collections::btree_set::Iter<'_, String> {
                self.0.iter()
            }
        }

        impl FromIterator<String> for $name {
            fn from_iter<I: IntoIterator<Item = String>>(topics: I) -> Self {
                // hot-path-alloc-ok: cold: a direction set is collected once when a
                // deployment is planned and once when a worker's plan is read, never
                // per message and never inside a step
                Self(topics.into_iter().collect())
            }
        }

        impl<'a> IntoIterator for &'a $name {
            type Item = &'a String;
            type IntoIter = std::collections::btree_set::Iter<'a, String>;

            fn into_iter(self) -> Self::IntoIter {
                self.0.iter()
            }
        }
    };
}

/// The absolute topics a SIBLING group of the same graph produces for this
/// worker: its cross-group INPUT edges, by topic.
///
/// The subgraph split rewrites each of those inputs to the absolute topic and
/// drops the producer from this worker's view, so to the worker's own validation
/// the source is an in-prefix absolute name with no producer, which draws the
/// "is this a typo?" warning on a graph the supervisor had just validated. Only
/// the supervisor holds the whole graph, so it names the topics here and the
/// worker hands them to validation. A source NOT in this set still warns.
#[derive(Debug, Default, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(transparent)]
pub struct SiblingProducedTopics(BTreeSet<String>);

/// The absolute topics THIS worker produces that a SIBLING group consumes, the
/// mirror of [`SiblingProducedTopics`].
///
/// A publish reaches a consumer in another process only for a topic in this set,
/// so it is what the supervisor knows about this worker's outward edges. Empty on
/// a single-process run, which has no siblings.
#[derive(Debug, Default, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(transparent)]
pub struct SiblingConsumedTopics(BTreeSet<String>);

topic_set_newtype!(SiblingProducedTopics);
topic_set_newtype!(SiblingConsumedTopics);

#[cfg(test)]
mod tests {
    use super::*;

    /// The plan file's bytes are the bare set in both directions, so a plan
    /// written with these types is byte identical to one written without them and
    /// a plan written before them still parses.
    ///
    /// What this pins is the WIRE SHAPE, not the attribute: serde emits a newtype
    /// struct for a one-field tuple struct and the JSON writer serializes that as
    /// its inner value, so these bytes come out the same with or without
    /// `transparent`. The arm is here because the shape is what a plan file in
    /// flight depends on, whatever representation a future edit reaches for.
    #[test]
    fn both_directions_serialize_as_the_bare_set() {
        let produced: SiblingProducedTopics = ["/g/a/out".to_string(), "/g/b/out".to_string()]
            .into_iter()
            .collect();
        let json = serde_json::to_string(&produced).expect("serialize");
        assert_eq!(
            json, r#"["/g/a/out","/g/b/out"]"#,
            "the inbound set must serialize as the bare sorted array"
        );
        let back: SiblingProducedTopics = serde_json::from_str(&json).expect("round trip");
        assert_eq!(back, produced, "and parse back to the same set");

        let consumed: SiblingConsumedTopics = ["/g/c/out".to_string()].into_iter().collect();
        assert_eq!(
            serde_json::to_string(&consumed).expect("serialize"),
            r#"["/g/c/out"]"#,
            "the outbound set must serialize as the bare sorted array too"
        );
        // The two are interchangeable as BYTES and not as TYPES, which is the
        // point: a plan file is unaffected, and a swap at a call site is a
        // compile error.
        let as_consumed: SiblingConsumedTopics =
            serde_json::from_str(&json).expect("the bytes parse as either direction");
        assert_eq!(as_consumed.iter().count(), 2, "the bytes carry both topics");
    }

    /// The reads every gate makes: membership, size and emptiness.
    #[test]
    fn a_set_answers_membership_and_size() {
        let consumed: SiblingConsumedTopics = ["/g/a/out".to_string()].into_iter().collect();
        assert!(consumed.contains("/g/a/out"), "a named topic is a member");
        assert!(
            !consumed.contains("/g/b/out"),
            "a topic the set does not name is not a member, which is what every \
             reader of the set asks it"
        );
        assert!(!consumed.is_empty());
        assert!(
            SiblingConsumedTopics::default().is_empty(),
            "the default is the empty set, the single-process answer"
        );
        assert_eq!(
            consumed.iter().collect::<Vec<_>>(),
            vec!["/g/a/out"],
            "iteration yields the topics it was built from"
        );
    }
}
