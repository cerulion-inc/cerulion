// SPDX-License-Identifier: AGPL-3.0-only
//! Private per-input REPLAY FEEDS: the seam through which a replay hands each
//! consumer read exactly the frames its recorded read served.
//!
//! # What a feed is
//!
//! On a replay of a bag that records READ IDENTITY (every read record names the
//! writer and the sequence of the frame it served), every read stage of the
//! replayed runtime subscribes to its OWN service, named by
//! [`replay_feed_topic`], instead of the topic it reads live. Nothing else
//! publishes on that service. Before a gated read pops, the read gate
//! (`read_outcome::ReadPlanStage`) asks the installed [`ReadFeed`] to publish
//! the frames of THAT recorded read onto it, so the pop takes exactly those
//! frames and nothing else:
//!
//! * a frame the live consumer lost to a `drop_oldest` eviction is never fed,
//!   because the recorded read never served it;
//! * two consumers of one topic hold two feeds, so one consumer's reads never
//!   move a frame into the other's queue;
//! * frames of several writers on one topic are fed by `(writer, sequence)`, so
//!   the interleaving of the live queue is not re-created at all.
//!
//! # The call
//!
//! The gate calls [`ReadFeed::fill`] with the stage key, the step, and the
//! ORDINAL of the admission inside that step's installed plan (`0` is the first
//! entry of the step's `DueRead` list). The feed owns the mapping from that
//! triple to frames; the core holds no frame bytes and no bag.
//!
//! The call is made from the drain site that consulted the gate, which may be a
//! node's own tick on any thread, so a feed is `Send + Sync`.

use crate::read_outcome::{ReadStageRole, StageKey};
use crate::transport::TopicServiceConfig;

/// The service-name root every private replay feed lives under.
pub const REPLAY_FEED_TOPIC_ROOT: &str = "/__cerulion/replay_feed";

/// The private service a fed read stage subscribes to.
///
/// `/__cerulion/replay_feed/<node>/<input_idx>/<body|drain>`: one service per
/// stage, so the body and the drain subscriber of one input, which hold two
/// queues live, hold two feeds here.
#[must_use]
pub fn replay_feed_topic(key: &StageKey) -> String {
    format!(
        "{REPLAY_FEED_TOPIC_ROOT}/{}/{}/{}",
        key.node,
        key.input_idx,
        key.role.label()
    )
}

/// The service config a fed stage's subscriber creates its feed with.
///
/// Starts from the config of the topic the stage reads live, so the borrow
/// budget a latest-value input holds is the same, and changes three things:
/// the buffer is raised to `buffer` (the deepest single read the recording
/// holds for this stage, never below the input's own depth), history is off,
/// and the publisher side is producer-less (`External`), so the replay's one
/// feed publisher opens the service the subscriber created.
#[must_use]
pub fn replay_feed_topic_config(live: TopicServiceConfig, buffer: usize) -> TopicServiceConfig {
    let mut cfg = live;
    cfg.subscriber_max_buffer_size = cfg.subscriber_max_buffer_size.max(buffer);
    cfg.history_size = 0;
    cfg.max_publishers = None;
    cfg.max_subscribers = None;
    cfg.publisher_provisioning = crate::transport::PublisherProvisioning::External;
    cfg
}

/// Why a [`ReadFeed::fill`] could not place a read's frames.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReadFeedFault {
    /// What failed, in an operator's words.
    pub detail: String,
}

/// The replay side of a private feed: publish the frames of one recorded read.
pub trait ReadFeed: Send + Sync + std::fmt::Debug {
    /// Publish, onto `key`'s private service, exactly the frames the recorded
    /// read at position `ordinal` of `step`'s installed plan served.
    ///
    /// # Errors
    ///
    /// [`ReadFeedFault`] when the frames cannot be published. The gate records
    /// it as a replay read violation on `key`; the pop that follows then finds
    /// fewer frames than the admission names and the shortfall is minted too.
    fn fill(&self, key: &StageKey, step: u64, ordinal: u32) -> Result<(), ReadFeedFault>;
}

/// Which edges a recording stages a PRODUCER annotation on.
///
/// How often is the edge's own fact ([`annotates_every_read`]): before every
/// read on an edge whose wire sequence names no writer on its own, and once per
/// WRITER RUN on every other annotated edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EdgeIdentity {
    /// Every edge. A multi-writer edge before every read; a single-writer edge
    /// once per WRITER RUN at each read site (`read_outcome::writer_run_site`):
    /// the annotation is staged before a read whose writer differs from the one
    /// the site last annotated, and names the writer of every later read of the
    /// site up to the next annotation. The read log names the writer of every
    /// served frame. What every recording this build makes stages
    /// (`trace_format` 8).
    #[default]
    EveryEdge,
    /// Only edges whose wire sequence names no writer on its own (a
    /// `multi_publisher_topics` topic, or a producer-less external topic), and
    /// there before every read. What a recording made before read identity
    /// staged.
    MultiWriterEdges,
}

impl EdgeIdentity {
    /// Does an edge carry the annotation, given whether its topic can have more
    /// than one writer?
    #[must_use]
    pub const fn annotates(self, multi_writer: bool) -> bool {
        match self {
            Self::EveryEdge => true,
            Self::MultiWriterEdges => multi_writer,
        }
    }

    /// The annotation term an edge's read stage is sized with: a pair per read
    /// on a multi-writer edge under both rules, and on a single-writer edge
    /// under [`Self::EveryEdge`] the run opening of each read site.
    #[must_use]
    pub const fn sizing(self, multi_writer: bool) -> crate::read_outcome::ProducerAnnotation {
        use crate::read_outcome::ProducerAnnotation;
        match (self, multi_writer) {
            (_, true) => ProducerAnnotation::Annotated,
            (Self::EveryEdge, false) => ProducerAnnotation::WriterRuns,
            (Self::MultiWriterEdges, false) => ProducerAnnotation::Plain,
        }
    }
}

/// Does an annotated edge stage its annotation before EVERY read, rather than
/// once per writer run? `true` on a multi-writer edge under both
/// [`EdgeIdentity`] rules: such an edge carries its writer on every read, as
/// every recorder from `trace_format` 3 on staged it.
#[must_use]
pub fn annotates_every_read(multi_writer: bool) -> bool {
    #[cfg(feature = "oracle-mutants")]
    if mutants::writer_runs_on_multi_writer_edges_armed() {
        return false;
    }
    multi_writer
}

/// The core's annotation MUTANT the read-identity oracle tests must catch,
/// compiled only with the `oracle-mutants` feature.
#[cfg(feature = "oracle-mutants")]
pub mod mutants {
    use std::sync::atomic::{AtomicBool, Ordering};

    static WRITER_RUNS_ON_MULTI_WRITER_EDGES: AtomicBool = AtomicBool::new(false);
    static DROP_TRAILING_REFILLS: AtomicBool = AtomicBool::new(false);

    /// A scope that arms one mutant until it drops.
    #[derive(Debug)]
    pub struct MutantGuard(&'static AtomicBool);

    impl Drop for MutantGuard {
        fn drop(&mut self) {
            self.0.store(false, Ordering::SeqCst);
        }
    }

    /// Arm the mutant: a trace-driven burst drops the trailing refill its
    /// recording holds (see `Scheduler::set_replay_trailing_refills`).
    #[must_use]
    pub fn drop_trailing_refills() -> MutantGuard {
        DROP_TRAILING_REFILLS.store(true, Ordering::SeqCst);
        MutantGuard(&DROP_TRAILING_REFILLS)
    }

    pub(crate) fn trailing_refills_dropped() -> bool {
        DROP_TRAILING_REFILLS.load(Ordering::SeqCst)
    }

    /// Arm the mutant: a runtime built while armed stages writer runs on
    /// multi-writer edges too, instead of an annotation before every read.
    #[must_use]
    pub fn writer_runs_on_multi_writer_edges() -> MutantGuard {
        WRITER_RUNS_ON_MULTI_WRITER_EDGES.store(true, Ordering::SeqCst);
        MutantGuard(&WRITER_RUNS_ON_MULTI_WRITER_EDGES)
    }

    pub(crate) fn writer_runs_on_multi_writer_edges_armed() -> bool {
        WRITER_RUNS_ON_MULTI_WRITER_EDGES.load(Ordering::SeqCst)
    }
}

/// How a replay wires a runtime it builds: the identity rule its recording was
/// made under, and whether its read stages subscribe to private feeds.
#[derive(Debug, Clone, Default)]
pub struct ReplayWiring {
    /// The annotation rule, which must be the recording's so both sides stage
    /// the same records.
    pub identity: EdgeIdentity,
    /// `Some` = every read stage subscribes to its [`replay_feed_topic`]; the
    /// map names the buffer a stage's feed needs (the deepest single recorded
    /// read on it). A stage the map omits takes its input's own depth.
    pub feeds: Option<std::collections::HashMap<StageKey, usize>>,
}

impl ReplayWiring {
    /// The feed buffer for the stage `(node, input_idx, role)`, or `None` when
    /// this runtime is not fed.
    #[must_use]
    pub fn feed_buffer(&self, node: &str, input_idx: u16, role: ReadStageRole) -> Option<usize> {
        let feeds = self.feeds.as_ref()?;
        let key = StageKey {
            node: node.to_string(),
            input_idx,
            role,
        };
        Some(feeds.get(&key).copied().unwrap_or(0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(role: ReadStageRole) -> StageKey {
        StageKey {
            node: "sink".to_string(),
            input_idx: 2,
            role,
        }
    }

    #[test]
    fn a_feed_topic_names_node_input_and_role_so_two_stages_of_one_input_differ() {
        assert_eq!(
            replay_feed_topic(&key(ReadStageRole::Body)),
            "/__cerulion/replay_feed/sink/2/body"
        );
        assert_eq!(
            replay_feed_topic(&key(ReadStageRole::Drain)),
            "/__cerulion/replay_feed/sink/2/drain"
        );
    }

    #[test]
    fn every_edge_annotates_single_writer_edges_and_the_legacy_rule_does_not() {
        assert!(EdgeIdentity::EveryEdge.annotates(false));
        assert!(EdgeIdentity::EveryEdge.annotates(true));
        assert!(!EdgeIdentity::MultiWriterEdges.annotates(false));
        assert!(EdgeIdentity::MultiWriterEdges.annotates(true));
        assert_eq!(EdgeIdentity::default(), EdgeIdentity::EveryEdge);
    }

    #[test]
    fn writer_runs_size_single_writer_edges_for_one_run_and_multi_writer_edges_for_a_pair_per_read()
    {
        use crate::read_outcome::ProducerAnnotation;
        assert_eq!(
            EdgeIdentity::EveryEdge.sizing(false),
            ProducerAnnotation::WriterRuns
        );
        assert_eq!(
            EdgeIdentity::EveryEdge.sizing(true),
            ProducerAnnotation::Annotated
        );
        assert_eq!(
            EdgeIdentity::MultiWriterEdges.sizing(false),
            ProducerAnnotation::Plain
        );
        assert_eq!(
            EdgeIdentity::MultiWriterEdges.sizing(true),
            ProducerAnnotation::Annotated
        );
        assert!(annotates_every_read(true));
        assert!(!annotates_every_read(false));
        // `sizing` is `Plain` exactly where `annotates` is false.
        for identity in [EdgeIdentity::EveryEdge, EdgeIdentity::MultiWriterEdges] {
            for multi_writer in [false, true] {
                assert_eq!(
                    identity.sizing(multi_writer) == ProducerAnnotation::Plain,
                    !identity.annotates(multi_writer),
                    "{identity:?} multi_writer={multi_writer}"
                );
            }
        }
    }

    #[test]
    fn an_unfed_wiring_names_no_buffer_and_a_fed_one_defaults_an_omitted_stage_to_zero() {
        let unfed = ReplayWiring::default();
        assert_eq!(unfed.feed_buffer("sink", 2, ReadStageRole::Body), None);
        let mut feeds = std::collections::HashMap::new();
        feeds.insert(key(ReadStageRole::Body), 7);
        let fed = ReplayWiring {
            identity: EdgeIdentity::EveryEdge,
            feeds: Some(feeds),
        };
        assert_eq!(fed.feed_buffer("sink", 2, ReadStageRole::Body), Some(7));
        assert_eq!(fed.feed_buffer("sink", 2, ReadStageRole::Drain), Some(0));
    }
}
