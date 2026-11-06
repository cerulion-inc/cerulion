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
//!
//! # Whose frame a fed frame is
//!
//! Every frame of one stage's feed leaves the replay's one feed port, so the
//! frames' iceoryx2 origin names no writer. The drain that pops a fed read asks
//! [`ReadFeed::frames`] for the recorded writer token and wire sequence of each
//! frame, in pop order, and keys each frame on its recorded writer: the
//! `drop_oldest` probe keeps one eviction baseline per recorded writer, as it
//! keeps one per writer port live, and a callback read's
//! `ReceivedMessage::origin` is the recorded writer token, widened.
//!
//! The probe holds as many baselines as the feed service has publisher slots.
//! A feed for a topic the recorded graph writes is created with the slots a
//! build of the whole graph creates that topic with ([`live_publisher_slots`],
//! carried in [`ReplayWiring::publisher_slots`]), which a rank that writes
//! nothing on the topic does not see in its own subgraph. A feed for a topic
//! no node of the graph writes takes the building runtime's own view of it.
//!
//! # Feed names
//!
//! A feed's service name carries the replay's identifier, a fixed-width digest
//! of the node id, the input index and the stage role (see
//! [`replay_feed_topic`]). The digest keeps every feed name at most
//! [`REPLAY_FEED_TOPIC_MAX_LEN`] bytes whatever the node id's length, inside
//! the transport's topic name limit, and a compile-time assertion holds that
//! bound to the limit.

use crate::graph::GraphConfig;
use crate::read_outcome::{ReadStageRole, StageKey};
use crate::transport::{PublisherProvisioning, TopicServiceConfig};

/// The service-name root every private replay feed lives under.
pub const REPLAY_FEED_TOPIC_ROOT: &str = "/__cerulion/replay_feed";

/// Hex digits of the node digest a feed name carries in place of the node id.
const NODE_DIGEST_HEX_LEN: usize = 16;

/// Decimal digits of `n`.
const fn decimal_digits(mut n: u64) -> usize {
    let mut digits = 1;
    while n >= 10 {
        n /= 10;
        digits += 1;
    }
    digits
}

/// Hex digits of the 128-bit unique system id a replay identifier carries.
const REPLAY_ID_HEX_LEN: usize = 32;

/// The length of every identifier [`new_replay_id`] mints: `u` and the
/// [`REPLAY_ID_HEX_LEN`] hex digits of its unique system id.
const REPLAY_ID_MAX_LEN: usize = 1 + REPLAY_ID_HEX_LEN;

/// The longest stage role label a feed name carries.
const ROLE_LABEL_MAX_LEN: usize = {
    let (body, drain) = (
        ReadStageRole::Body.label().len(),
        ReadStageRole::Drain.label().len(),
    );
    if body > drain {
        body
    } else {
        drain
    }
};

/// The longest name [`replay_feed_topic`] gives a stage of a replay whose
/// identifier [`new_replay_id`] minted: the root, the replay identifier, the
/// node digest, a `u16` input index, the role label and four separators.
pub const REPLAY_FEED_TOPIC_MAX_LEN: usize = REPLAY_FEED_TOPIC_ROOT.len()
    + 1
    + REPLAY_ID_MAX_LEN
    + 1
    + NODE_DIGEST_HEX_LEN
    + 1
    + decimal_digits(u16::MAX as u64)
    + 1
    + ROLE_LABEL_MAX_LEN;

const _: () = assert!(REPLAY_FEED_TOPIC_MAX_LEN <= crate::transport::MAX_TOPIC_LEN);

/// The private service a fed read stage subscribes to.
///
/// `/__cerulion/replay_feed/<replay_id>/<node digest>/<input_idx>/<body|drain>`:
/// one service per stage, so the body and the drain subscriber of one input,
/// which hold two queues live, hold two feeds here. `<node digest>` is the 16
/// lowercase hex digits of [`crate::wire::fnv1a_hash`] over the node id's
/// bytes, so its width does not depend on the node id's length and, with a
/// `replay_id` from [`new_replay_id`], the name is at most
/// [`REPLAY_FEED_TOPIC_MAX_LEN`] bytes. The name is a function of its inputs
/// alone (FNV-1a is a fixed algorithm with no per-process seed), so the
/// replay's feed publisher and the runtime's fed subscriber, which each call
/// this, name one service.
#[must_use]
pub fn replay_feed_topic(replay_id: &str, key: &StageKey) -> String {
    #[cfg(feature = "oracle-mutants")]
    if mutants::full_node_feed_names_armed() {
        return format!(
            "{REPLAY_FEED_TOPIC_ROOT}/{replay_id}/{}/{}/{}",
            key.node,
            key.input_idx,
            key.role.label()
        );
    }
    format!(
        "{REPLAY_FEED_TOPIC_ROOT}/{replay_id}/{:016x}/{}/{}",
        crate::wire::fnv1a_hash(key.node.as_bytes()),
        key.input_idx,
        key.role.label()
    )
}

/// A fresh identifier for ONE replay: `u` and the 32 lowercase hex digits of
/// a fresh iceoryx2 `UniqueSystemId`. That id packs the process id, the
/// creation time to the nanosecond and a per-process counter, so two replays
/// in one process carry two counter values, and two processes that share an
/// iceoryx2 root, in separate PID namespaces included, carry the same
/// identifier only when they hold one process id and mint at the same
/// nanosecond as the same counter value.
///
/// # Errors
///
/// [`ReplayIdUnavailable`] when the clock the id reads cannot be read.
pub fn new_replay_id() -> Result<String, ReplayIdUnavailable> {
    let id = iceoryx2_bb_posix::unique_system_id::UniqueSystemId::new().map_err(|e| {
        ReplayIdUnavailable {
            detail: format!("{e:?}"),
        }
    })?;
    Ok(replay_id_of(id.value()))
}

/// The identifier of the replay whose unique system id is `value`.
fn replay_id_of(value: u128) -> String {
    format!("u{value:0width$x}", width = REPLAY_ID_HEX_LEN)
}

/// A replay identifier could not be minted.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReplayIdUnavailable {
    /// What failed, as iceoryx2 reports it.
    pub detail: String,
}

/// The service config a fed stage's subscriber creates its feed with.
///
/// Starts from the config of the topic the stage reads live, so the borrow
/// budget a latest-value input holds is the same, and changes three things:
/// the buffer is raised to `buffer` (the deepest single read the recording
/// holds for this stage, never below the input's own depth), history is off
/// (the create leg's history floor is cleared too, so the service the build
/// creates has no history and exactly that buffer), and the
/// publisher side is producer-less (`External`), so the replay's one
/// feed publisher opens the service the subscriber created. The publisher
/// slot count is `live_publishers` when it is `Some`
/// ([`ReplayWiring::live_publishers`]), else `live`'s own: the subscriber sizes
/// its `drop_oldest` probe's per-writer baselines from it, and a fed read keys
/// its frames on their recorded writers.
#[must_use]
pub fn replay_feed_topic_config(
    live: TopicServiceConfig,
    buffer: usize,
    live_publishers: Option<usize>,
) -> TopicServiceConfig {
    let mut cfg = live;
    cfg.subscriber_max_buffer_size = cfg.subscriber_max_buffer_size.max(buffer);
    cfg.history_size = 0;
    cfg.create_history_floor = None;
    cfg.max_publishers = live_publishers.or(cfg.max_publishers);
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

    /// The recorded `(writer token, wire sequence)` of each frame
    /// [`Self::fill`] publishes for the read at position `ordinal` of `step`'s
    /// installed plan, in the order it publishes them; `None` when the feed
    /// holds no such read.
    fn frames(&self, key: &StageKey, step: u64, ordinal: u32) -> Option<&[(u64, u32)]>;
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
    /// (`trace_format` 8 on).
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

    thread_local! {
        static FULL_NODE_FEED_NAMES: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    }

    /// A scope that arms the full-node feed names on its own thread until it
    /// drops. Thread-local, so a test arming it changes no other test's names.
    #[derive(Debug)]
    pub struct ThreadMutantGuard(());

    impl Drop for ThreadMutantGuard {
        fn drop(&mut self) {
            FULL_NODE_FEED_NAMES.with(|c| c.set(false));
        }
    }

    /// Arm the mutant on this thread: [`super::replay_feed_topic`] carries the
    /// full node id instead of its fixed-width digest.
    #[must_use]
    pub fn full_node_feed_names() -> ThreadMutantGuard {
        FULL_NODE_FEED_NAMES.with(|c| c.set(true));
        ThreadMutantGuard(())
    }

    pub(crate) fn full_node_feed_names_armed() -> bool {
        FULL_NODE_FEED_NAMES.with(std::cell::Cell::get)
    }

    thread_local! {
        static BATCH_FRAMES_AT_ONE_SLOT: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
        static BATCH_OVERFLOW_IGNORED: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    }

    /// A scope that arms one batch frame mutant on its own thread until it
    /// drops.
    #[derive(Debug)]
    pub struct BatchMutantGuard(&'static std::thread::LocalKey<std::cell::Cell<bool>>);

    impl Drop for BatchMutantGuard {
        fn drop(&mut self) {
            self.0.with(|c| c.set(false));
        }
    }

    /// Arm the mutant on this thread: the batch frame fit rule charges one
    /// slot per frame on an annotated edge too.
    #[must_use]
    pub fn batch_frames_at_one_slot() -> BatchMutantGuard {
        BATCH_FRAMES_AT_ONE_SLOT.with(|c| c.set(true));
        BatchMutantGuard(&BATCH_FRAMES_AT_ONE_SLOT)
    }

    pub(crate) fn batch_frames_at_one_slot_armed() -> bool {
        BATCH_FRAMES_AT_ONE_SLOT.with(std::cell::Cell::get)
    }

    /// Arm the mutant on this thread: a read whose drain overflowed its frame
    /// list stages the frames the list kept.
    #[must_use]
    pub fn batch_overflow_ignored() -> BatchMutantGuard {
        BATCH_OVERFLOW_IGNORED.with(|c| c.set(true));
        BatchMutantGuard(&BATCH_OVERFLOW_IGNORED)
    }

    pub(crate) fn batch_overflow_ignored_armed() -> bool {
        BATCH_OVERFLOW_IGNORED.with(std::cell::Cell::get)
    }
}

/// How a replay wires a runtime it builds: the identity rule its recording was
/// made under, and whether its read stages subscribe to private feeds.
#[derive(Debug, Clone, Default)]
pub struct ReplayWiring {
    /// The annotation rule, which must be the recording's so both sides stage
    /// the same records.
    pub identity: EdgeIdentity,
    /// The replay's identifier ([`new_replay_id`]): every private feed
    /// service name carries it, so two replays sharing an iceoryx2 config
    /// attach to each other's feeds only when their identifiers are equal.
    pub replay_id: String,
    /// `Some` = every read stage subscribes to its [`replay_feed_topic`]; the
    /// map names the buffer a stage's feed needs (the deepest single recorded
    /// read on it). A stage the map omits takes its input's own depth.
    pub feeds: Option<std::collections::HashMap<StageKey, usize>>,
    /// Per topic, the publisher slots a build of the whole recorded graph
    /// creates its service with ([`live_publisher_slots`]). A fed stage's feed
    /// service is created with its topic's entry; a topic the map omits takes
    /// the slots of the building runtime's own view of the topic.
    pub publisher_slots: std::collections::HashMap<String, usize>,
}

/// Per topic a native node of `config` writes, the publisher slots a build of
/// the whole of `config` creates its service with: the
/// [`MULTI_PUBLISHER_LOOSE_MAX`](crate::transport::MULTI_PUBLISHER_LOOSE_MAX)
/// of a `multi_publisher_topics` topic, and 1 for any other. A multi-process
/// deployment's supervisor creates every such service at that config before
/// any worker starts, so the map also holds the slots of a topic a worker reads
/// and another worker writes. A topic no native node writes is absent.
#[must_use]
pub fn live_publisher_slots(config: &GraphConfig) -> std::collections::HashMap<String, usize> {
    let mut slots = std::collections::HashMap::new();
    for node in config.native_nodes() {
        for output in &node.outputs {
            let topic = crate::graph::resolve_output_topic(&config.prefix, &node.id, output);
            let provisioning = if config.is_multi_publisher(&topic) {
                PublisherProvisioning::Multi
            } else {
                PublisherProvisioning::SingleWriter
            };
            if let Some(n) = provisioning.max_publishers() {
                slots.insert(topic, n);
            }
        }
    }
    slots
}

impl ReplayWiring {
    /// The publisher slots a fed stage reading `topic` creates its feed with,
    /// or `None` when [`Self::publisher_slots`] holds none for the topic.
    #[must_use]
    pub fn live_publishers(&self, topic: &str) -> Option<usize> {
        self.publisher_slots.get(topic).copied()
    }

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

    fn key_of(node: &str, input_idx: u16, role: ReadStageRole) -> StageKey {
        StageKey {
            node: node.to_string(),
            input_idx,
            role,
        }
    }

    /// The node segment is the FNV-1a digest of `sink`, pinned as a literal so
    /// a seeded or changed hash cannot pass.
    #[test]
    fn a_feed_topic_names_replay_node_input_and_role_so_two_stages_of_one_input_differ() {
        assert_eq!(
            replay_feed_topic("r1", &key(ReadStageRole::Body)),
            "/__cerulion/replay_feed/r1/4e2e8418e6525c72/2/body"
        );
        assert_eq!(
            replay_feed_topic("r1", &key(ReadStageRole::Drain)),
            "/__cerulion/replay_feed/r1/4e2e8418e6525c72/2/drain"
        );
    }

    /// A 300 character node id under the longest replay identifier, input
    /// index and role: the name passes the validation every topic service
    /// creation applies, at exactly the bound the compile-time assertion holds.
    #[test]
    fn a_feed_name_for_a_300_character_node_id_passes_topic_validation() {
        let node = "n".repeat(300);
        let longest = replay_feed_topic(
            &replay_id_of(u128::MAX),
            &key_of(&node, u16::MAX, ReadStageRole::Drain),
        );
        assert_eq!(longest.len(), REPLAY_FEED_TOPIC_MAX_LEN, "{longest}");
        assert_eq!(
            REPLAY_FEED_TOPIC_MAX_LEN, 86,
            "docs/bag.md states this bound"
        );
        assert_eq!(
            crate::transport::validate_topic_name(&longest),
            Ok(()),
            "{longest}"
        );
        let minted = replay_feed_topic(
            &new_replay_id().expect("the clock reads"),
            &key_of(&node, 0, ReadStageRole::Body),
        );
        assert_eq!(
            crate::transport::validate_topic_name(&minted),
            Ok(()),
            "{minted}"
        );
    }

    /// Two node ids equal for their first 218 characters name two feeds.
    #[test]
    fn two_node_ids_that_differ_only_after_character_218_name_two_feeds() {
        let stem = "s".repeat(218);
        let (a, b) = (format!("{stem}a"), format!("{stem}b"));
        for role in [ReadStageRole::Body, ReadStageRole::Drain] {
            assert_ne!(
                replay_feed_topic("r1", &key_of(&a, 0, role)),
                replay_feed_topic("r1", &key_of(&b, 0, role))
            );
        }
    }

    /// The same replay identifier and stage give the same name on every call.
    #[test]
    fn a_feed_name_is_a_function_of_its_inputs_alone() {
        let node = "a/long_node".repeat(40);
        let stage = key_of(&node, 7, ReadStageRole::Body);
        assert_eq!(
            replay_feed_topic("p1r2", &stage),
            replay_feed_topic("p1r2", &key_of(&node, 7, ReadStageRole::Body))
        );
        assert_eq!(
            replay_feed_topic("p1r2", &stage),
            format!(
                "/__cerulion/replay_feed/p1r2/{:016x}/7/body",
                crate::wire::fnv1a_hash(node.as_bytes())
            )
        );
    }

    /// MUTANT (`oracle-mutants`): a feed name carrying the full node id fails
    /// the 300 character test.
    #[cfg(feature = "oracle-mutants")]
    #[test]
    fn the_long_node_test_catches_full_node_feed_names() {
        let _armed = mutants::full_node_feed_names();
        let caught = std::panic::catch_unwind(
            a_feed_name_for_a_300_character_node_id_passes_topic_validation,
        );
        assert!(
            caught.is_err(),
            "the long node test passed under full node names"
        );
    }

    /// Two replays of graphs with the same node names: every feed name differs,
    /// so neither attaches to the other's services.
    #[test]
    fn two_replays_of_matching_node_names_never_share_a_feed_service() {
        for role in [ReadStageRole::Body, ReadStageRole::Drain] {
            assert_ne!(
                replay_feed_topic("replay-a", &key(role)),
                replay_feed_topic("replay-b", &key(role))
            );
        }
        assert_ne!(
            new_replay_id().expect("the clock reads"),
            new_replay_id().expect("the clock reads")
        );
    }

    /// A replay identifier is `u` and all 128 bits of its unique system id in
    /// hex, so two ids that differ in any one field, the creation time of two
    /// processes holding one process id in two PID namespaces included, name
    /// two replays.
    ///
    /// Hand oracle: `100 | 7 << 64` and the same value plus `1 << 32` render as
    /// the two 32 digit strings below, differing in one digit.
    #[test]
    fn a_replay_id_carries_every_bit_of_its_unique_system_id() {
        let at = |nanoseconds: u128| 100 | (nanoseconds << 32) | (7 << 64);
        assert_eq!(replay_id_of(at(0)), "u00000000000000070000000000000064");
        assert_eq!(replay_id_of(at(1)), "u00000000000000070000000100000064");
        assert_eq!(replay_id_of(u128::MAX).len(), REPLAY_ID_MAX_LEN);
        let minted = new_replay_id().expect("the clock reads");
        assert_eq!(minted.len(), REPLAY_ID_MAX_LEN, "{minted}");
        assert!(
            minted[1..]
                .chars()
                .all(|c| c.is_ascii_digit() || ('a'..='f').contains(&c)),
            "{minted}"
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

    /// A feed's service config keeps the live topic's publisher slots, so a
    /// fed `drop_oldest` probe holds as many per-writer baselines as it does
    /// live: a `multi_publisher_topics` bus's 16 stay 16, while the buffer is
    /// raised to the feed's, history is off, the subscriber slots are unset and
    /// the publisher side is producer-less.
    #[test]
    fn a_feed_config_keeps_the_live_topics_publisher_slots() {
        let live = TopicServiceConfig {
            subscriber_max_buffer_size: 4,
            subscriber_max_borrowed_samples: None,
            create_borrow_floor: None,
            create_publisher_floor: None,
            create_subscriber_floor: None,
            create_history_floor: None,
            max_subscribers: Some(17),
            max_publishers: Some(16),
            publisher_provisioning: PublisherProvisioning::Multi,
            history_size: 3,
            extra_event_listeners: 0,
            publisher_max_loaned_samples: None,
        };
        let fed = replay_feed_topic_config(live, 6, None);
        assert_eq!(fed.max_publishers, Some(16));
        assert_eq!(fed.subscriber_max_buffer_size, 6);
        assert_eq!(fed.history_size, 0);
        assert_eq!(fed.max_subscribers, None);
        assert_eq!(fed.publisher_provisioning, PublisherProvisioning::External);
        // A live config with a create-leg history floor yields a feed whose
        // created service has no history and the feed's own buffer.
        let floored = TopicServiceConfig {
            create_history_floor: Some(8),
            ..live
        };
        let fed = replay_feed_topic_config(floored, 6, None);
        assert_eq!(fed.create_history_floor, None);
        assert_eq!(fed.create_history_size(), 0);
        assert_eq!(fed.create_subscriber_max_buffer_size(), 6);
    }

    /// A rank that writes nothing on a bus holds an `External` view of it, with
    /// no publisher slots. Its feed takes the whole graph's 16, and a feed with
    /// no whole-graph entry keeps the rank's view.
    #[test]
    fn a_feed_config_takes_the_whole_graphs_publisher_slots_over_the_ranks_view() {
        let rank_view = TopicServiceConfig {
            subscriber_max_buffer_size: 16,
            subscriber_max_borrowed_samples: None,
            create_borrow_floor: None,
            create_publisher_floor: None,
            create_subscriber_floor: None,
            create_history_floor: None,
            max_subscribers: None,
            max_publishers: None,
            publisher_provisioning: PublisherProvisioning::External,
            history_size: 0,
            extra_event_listeners: 0,
            publisher_max_loaned_samples: None,
        };
        assert_eq!(
            replay_feed_topic_config(rank_view, 2, Some(16)).max_publishers,
            Some(16)
        );
        assert_eq!(
            replay_feed_topic_config(rank_view, 2, None).max_publishers,
            None
        );
    }

    /// Three writers on a `multi_publisher_topics` bus, a fourth writer on its
    /// own output, and a consumer of a topic no node writes: the bus holds 16
    /// slots, the fourth writer's topic 1, and the unwritten topic is absent.
    #[test]
    fn live_publisher_slots_name_every_written_topic_at_its_whole_graph_slots() {
        let yaml = "name: lps\nprefix: lps\nmulti_publisher_topics:\n  - /lps/bus\nnodes:\n\
            \x20 - id: w1\n    type: w\n    outputs:\n      - name: out\n        schema: geometry_msgs/Vector3\n        topic: /lps/bus\n\
            \x20 - id: w2\n    type: w\n    outputs:\n      - name: out\n        schema: geometry_msgs/Vector3\n        topic: /lps/bus\n\
            \x20 - id: w3\n    type: w\n    outputs:\n      - name: out\n        schema: geometry_msgs/Vector3\n        topic: /lps/bus\n\
            \x20 - id: src\n    type: w\n    outputs:\n      - name: out\n        schema: geometry_msgs/Vector3\n\
            \x20 - id: sink\n    type: s\n    inputs:\n      - name: a\n        source: /lps/bus\n      - name: b\n        source: /lps/ext\n";
        let config: GraphConfig = serde_yaml::from_str(yaml).unwrap();
        let slots = live_publisher_slots(&config);
        let expected: std::collections::HashMap<String, usize> = [
            (
                "/lps/bus".to_string(),
                crate::transport::MULTI_PUBLISHER_LOOSE_MAX,
            ),
            ("/lps/src/out".to_string(), 1),
        ]
        .into_iter()
        .collect();
        assert_eq!(slots, expected);
        assert_eq!(crate::transport::MULTI_PUBLISHER_LOOSE_MAX, 16);
        let wiring = ReplayWiring {
            publisher_slots: slots,
            ..ReplayWiring::default()
        };
        assert_eq!(wiring.live_publishers("/lps/bus"), Some(16));
        assert_eq!(wiring.live_publishers("/lps/ext"), None);
    }

    #[test]
    fn an_unfed_wiring_names_no_buffer_and_a_fed_one_defaults_an_omitted_stage_to_zero() {
        let unfed = ReplayWiring::default();
        assert_eq!(unfed.feed_buffer("sink", 2, ReadStageRole::Body), None);
        let mut feeds = std::collections::HashMap::new();
        feeds.insert(key(ReadStageRole::Body), 7);
        let fed = ReplayWiring {
            identity: EdgeIdentity::EveryEdge,
            replay_id: "r1".to_string(),
            feeds: Some(feeds),
            publisher_slots: std::collections::HashMap::new(),
        };
        assert_eq!(fed.feed_buffer("sink", 2, ReadStageRole::Body), Some(7));
        assert_eq!(fed.feed_buffer("sink", 2, ReadStageRole::Drain), Some(0));
    }
}
