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
/// made under, whether its read stages subscribe to private feeds, and the
/// `block` edges it feeds from its recording.
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
    /// The `block` edges this replay feeds from its recording, with no credit
    /// word, because their producer and consumer run in different passes. Empty
    /// on every build but a replay pass of a several-rank recording; see
    /// [`BagFedBlockEdges`] for what the set does and why it is sound.
    pub bag_fed_block_edges: BagFedBlockEdges,
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

/// One `block` edge a replay feeds from its recording, with no credit word:
/// `consumer_node.consumer_input` reads `topic`, which `producer_nodes` write,
/// and the replay runs the consumer and the producers in different passes.
///
/// A build holds at most one end of it, and which end is read off the build's
/// own graph ([`BagFedBlockEdges`]), never stated by the caller.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BagFedBlockEdge {
    /// The resolved absolute topic.
    pub topic: String,
    /// The consuming node.
    pub consumer_node: String,
    /// The consuming input.
    pub consumer_input: String,
    /// The topic's in-graph producers, the nodes a live credit word would
    /// defer.
    pub producer_nodes: Vec<String>,
}

/// The `block` edges a replay build feeds from its recording
/// ([`BagFedBlockEdge`]), held in their `Ord` order, so a build that refuses
/// two of them names the same one every time.
///
/// # What the set changes in a build
///
/// * The CONSUMER end: a build that holds `consumer_node`, and no in-graph
///   producer of `topic`, accepts the `block` input that
///   [`GraphTopology::validate`](crate::graph::topology::GraphTopology::validate)
///   refuses as having no producer to defer, and installs no backpressure
///   probe on it: no block probe, since no credit word counts it, and no
///   `drop_oldest` detector, whose eviction events and counts the live
///   `block` input never raised. The replay does not serve the block probe's
///   own at-depth event either: the recording does not hold when it fired.
/// * The PRODUCER end: a build that holds one of `producer_nodes`, and not the
///   consumer, writes `topic` with its consumer in another pass, and marks
///   each of `producer_nodes` it holds as held by the edge, so the scheduler
///   accepts a trailing refill the recording holds on that node below the
///   burst cap. Live, the consumer's credit word in another process could
///   defer the node between the fires of a burst, and this build has no
///   pre-fire check to show it.
///
/// The build verifies the end it holds, the way it verifies a credit
/// binding's role, and refuses, naming the entry: an entry a mapped credit
/// binding also names, an entry it holds both ends of or neither end of, and
/// a consumer end that is not a `block` input or whose topic has an in-graph
/// producer the entry does not name.
///
/// # Why the exemption is sound
///
/// [`crate::graph::topology::CreditedEdges`] is the other exemption from the
/// producer-less `block` refusal, and it can only name an edge whose mapped
/// credit word some process opened. This set is plain data, so its soundness
/// rests on where it reaches:
///
/// * only [`GraphRuntime::build_for_replay`](crate::graph::GraphRuntime::build_for_replay)
///   threads a caller's [`ReplayWiring`], and it builds on a
///   [`VirtualClock`](crate::clock::VirtualClock); every live,
///   deterministic-live and worker constructor builds with
///   `ReplayWiring::default()`, whose set is empty;
/// * a replay pass takes its fires from the recorded trace, so no fire is
///   decided against a count the build does not keep;
/// * [`GraphRuntime::run_live`](crate::graph::GraphRuntime::run_live) refuses
///   a runtime built with a non-empty set, before it sets anything up.
///
/// A library caller that drives such a runtime with `step()` and no recorded
/// fire plan runs the consumer end's `block` input with nothing holding its
/// producer. The polled seam does not refuse that; it is a documented limit
/// of the seam, as `run_live`'s node-death arming states for its own.
///
/// `block_timeout_ms` on a fed edge's input is accepted and has no effect on
/// either end. The key acts only through a producer's defer edge
/// (`BlockDeferEdge`), and neither build has one for the edge: the consumer
/// end has no producer of the topic, and the producer end has no `block`
/// consumer of it. A recording of an unchanged graph never holds such an
/// edge, because `cerulion graph run` refuses `block_timeout_ms` on an edge
/// its partition splits.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct BagFedBlockEdges {
    edges: std::collections::BTreeSet<BagFedBlockEdge>,
}

impl BagFedBlockEdges {
    /// The set of `edges` (a repeat is held once).
    #[must_use]
    pub fn new(edges: impl IntoIterator<Item = BagFedBlockEdge>) -> Self {
        Self {
            edges: edges.into_iter().collect(),
        }
    }

    /// No edge at all, the set of every build but a replay pass.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.edges.is_empty()
    }

    /// The edges, in `Ord` order.
    pub fn iter(&self) -> impl Iterator<Item = &BagFedBlockEdge> {
        self.edges.iter()
    }

    /// Which end of each edge the build over `topology` holds, verified
    /// against that graph (see the type's docs) and refused, at the first
    /// entry in `Ord` order that fails. `credited` is the build's own
    /// credit-bound edges and `graph` its identity, for the refusals' text.
    pub(crate) fn resolve(
        &self,
        topology: &crate::graph::topology::GraphTopology,
        credited: &crate::graph::topology::CreditedEdges<'_>,
        graph: &str,
    ) -> crate::error::TransportResult<BagFedEnds> {
        let mut ends = BagFedEnds::default();
        for edge in &self.edges {
            let refuse = |why: String| crate::error::TransportError::GraphError {
                reason: format!(
                    "graph '{graph}': the replay wiring feeds `block` edge '{}' -> '{}.{}' from a \
                     recording, {why}",
                    edge.topic, edge.consumer_node, edge.consumer_input
                ),
            };
            if credited.contains(&edge.topic, &edge.consumer_node, &edge.consumer_input) {
                return Err(refuse(
                    "and a cross-process credit binding names the same edge: an edge counts on \
                     its credit word or is fed from a recording, never both"
                        .to_string(),
                ));
            }
            let flow = topology.topic(&edge.topic);
            let consumer = flow.and_then(|f| {
                f.consumers
                    .iter()
                    .find(|c| c.node_id == edge.consumer_node && c.input == edge.consumer_input)
            });
            let local_producers: &[String] = flow.map_or(&[], |f| f.producers.as_slice());
            let (named, unnamed): (Vec<&str>, Vec<&str>) = local_producers
                .iter()
                .map(String::as_str)
                .partition(|p| edge.producer_nodes.iter().any(|n| n == p));
            match (consumer, named.is_empty()) {
                (Some(_), false) => {
                    return Err(refuse(format!(
                        "but this build holds both its consumer and its producer ({}); a fed \
                         edge's producer runs in another pass",
                        named.join(", ")
                    )));
                }
                (None, true) => {
                    return Err(refuse(format!(
                        "but this build holds neither that input nor any of its producers ({})",
                        edge.producer_nodes.join(", ")
                    )));
                }
                (Some(input), true) => {
                    if !matches!(input.policy, crate::graph::node::BackpressurePolicy::Block) {
                        return Err(refuse(format!(
                            "but this build declares that input {:?}, not `block`",
                            input.policy
                        )));
                    }
                    // A producer the entry does not name, here: the input
                    // counts on that producer's local word, which a fed edge
                    // must never silence.
                    if !unnamed.is_empty() {
                        return Err(refuse(format!(
                            "but this build also holds an in-graph producer of the topic the \
                             entry does not name ({}), and a `block` input with an in-graph \
                             producer counts on its local word",
                            unnamed.join(", ")
                        )));
                    }
                    ends.consumers.push((
                        edge.topic.clone(),
                        edge.consumer_node.clone(),
                        edge.consumer_input.clone(),
                    ));
                }
                // A producer end: this build writes `topic` and its consumer
                // runs in another pass.
                (None, false) => ends.producers.extend(named.into_iter().map(str::to_string)),
            }
        }
        Ok(ends)
    }
}

/// Which end of each [`BagFedBlockEdge`] one build holds, resolved by
/// [`BagFedBlockEdges::resolve`]. Empty on every build whose wiring feeds no
/// `block` edge.
#[derive(Debug, Default)]
pub(crate) struct BagFedEnds {
    /// `(topic, consumer node, consumer input)` of each consumer end, in the
    /// set's order.
    consumers: Vec<(String, String, String)>,
    /// Each producer node a fed edge holds here, in name order.
    producers: std::collections::BTreeSet<String>,
}

impl BagFedEnds {
    /// Is `node.input` on `topic` the consumer end of a fed edge?
    pub(crate) fn feeds_consumer(&self, topic: &str, node: &str, input: &str) -> bool {
        self.consumers
            .iter()
            .any(|(t, n, i)| t == topic && n == node && i == input)
    }

    /// The producer nodes a fed edge holds here, in name order.
    pub(crate) fn held_producers(&self) -> impl Iterator<Item = &str> {
        self.producers.iter().map(String::as_str)
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
            bag_fed_block_edges: BagFedBlockEdges::default(),
        };
        assert_eq!(fed.feed_buffer("sink", 2, ReadStageRole::Body), Some(7));
        assert_eq!(fed.feed_buffer("sink", 2, ReadStageRole::Drain), Some(0));
    }
}

#[cfg(test)]
mod bag_fed_block_edge_tests {
    //! Oracle vectors for [`BagFedBlockEdges::resolve`] and the validation
    //! exemption it feeds: a hand-written graph and declarations in, the
    //! hand-written refusal text or end out.

    use super::*;
    use crate::graph::node::{BackpressurePolicy, InputMeta, NodeInfo};
    use crate::graph::topology::{CreditedEdges, GraphTopology};
    use indexmap::IndexMap;

    const PRODUCER: &str = "  - id: prod\n    type: p\n    outputs:\n      - name: out\n        \
        schema: geometry_msgs/Vector3\n        topic: /fb/data\n";
    /// A second writer of `/fb/data` that no entry below names.
    const OTHER_PRODUCER: &str =
        "  - id: other\n    type: p\n    outputs:\n      - name: out\n        \
        schema: geometry_msgs/Vector3\n        topic: /fb/data\n";
    const CONSUMER: &str =
        "  - id: sink\n    type: s\n    inputs:\n      - name: inp\n        source: /fb/data\n";
    /// The refusals' common head for the entry `/fb/data -> sink.inp`.
    const HEAD: &str = "Graph validation error: graph 'fb': the replay wiring feeds `block` edge \
                        '/fb/data' -> 'sink.inp' from a recording, ";

    /// A graph of `nodes` (YAML node entries) under prefix `fb`, with `sink`'s
    /// `inp` declared `policy` at depth 2, and its topology.
    fn topology(nodes: &[&str], policy: BackpressurePolicy) -> GraphTopology {
        topology_listing("", nodes, policy)
    }

    /// [`topology`] with `listed` (YAML top-level keys) before the nodes.
    fn topology_listing(listed: &str, nodes: &[&str], policy: BackpressurePolicy) -> GraphTopology {
        let yaml = format!("name: fb\nprefix: fb\n{listed}nodes:\n{}", nodes.concat());
        let config: GraphConfig = serde_yaml::from_str(&yaml).expect("the test graph parses");
        let infos: IndexMap<String, NodeInfo> = config
            .nodes
            .iter()
            .map(|n| {
                let metas = if n.id == "sink" {
                    vec![InputMeta {
                        name: "inp".to_string(),
                        schema_hash: 0,
                        trigger: true,
                        depth: 2,
                        backpressure: policy,
                        expect_within_ms: None,
                        accumulate: false,
                        consume: Default::default(),
                    }]
                } else {
                    vec![]
                };
                (n.id.clone(), NodeInfo::with_meta(metas, vec![]))
            })
            .collect();
        GraphTopology::build(&config, &infos).expect("the topology builds")
    }

    /// The entry `topic -> sink.inp`, written by `prod`.
    fn edge(topic: &str) -> BagFedBlockEdge {
        BagFedBlockEdge {
            topic: topic.to_string(),
            consumer_node: "sink".to_string(),
            consumer_input: "inp".to_string(),
            producer_nodes: vec!["prod".to_string()],
        }
    }

    /// `set` resolved over `topology` with no credited edge, its refusal as
    /// text.
    fn resolve(set: &BagFedBlockEdges, topology: &GraphTopology) -> Result<(), String> {
        set.resolve(topology, &CreditedEdges::none(), "fb")
            .map(|_| ())
            .map_err(|e| e.to_string())
    }

    /// The empty set changes nothing: the consumer-only build is refused as
    /// a `block` input with no in-graph producer, as before the set existed.
    #[test]
    fn the_empty_set_resolves_no_end_and_leaves_the_producer_less_refusal() {
        let consumer_only = topology(&[CONSUMER], BackpressurePolicy::Block);
        let ends = BagFedBlockEdges::default()
            .resolve(&consumer_only, &CreditedEdges::none(), "fb")
            .expect("the empty set resolves");
        assert!(!ends.feeds_consumer("/fb/data", "sink", "inp"));
        let refused = consumer_only
            .validate_with_credit_edges(&CreditedEdges::none(), &ends)
            .expect_err("a producer-less `block` input is refused");
        assert!(
            refused.to_string().starts_with(
                "Graph validation error: block input 'sink.inp' reads topic '/fb/data', which has \
                 no in-graph producer; "
            ),
            "{refused}"
        );
    }

    /// The consumer end is exempt from the producer-less refusal, for that
    /// one input only.
    #[test]
    fn a_consumer_end_is_exempt_from_the_producer_less_refusal() {
        let consumer_only = topology(&[CONSUMER], BackpressurePolicy::Block);
        let set = BagFedBlockEdges::new([edge("/fb/data")]);
        let ends = set
            .resolve(&consumer_only, &CreditedEdges::none(), "fb")
            .expect("the consumer end resolves");
        assert!(ends.feeds_consumer("/fb/data", "sink", "inp"));
        assert!(!ends.feeds_consumer("/fb/data", "sink", "other"));
        assert!(!ends.feeds_consumer("/fb/other", "sink", "inp"));
        assert_eq!(
            ends.held_producers().count(),
            0,
            "a consumer end holds no producer"
        );
        consumer_only
            .validate_with_credit_edges(&CreditedEdges::none(), &ends)
            .expect("the fed input is exempt");
    }

    /// `block_timeout_ms` on a consumer end is accepted: the key is inert
    /// without a producer's defer edge.
    #[test]
    fn a_block_timeout_on_a_consumer_end_is_accepted() {
        let timed = format!("{CONSUMER}        block_timeout_ms: 100\n");
        let consumer_only = topology(&[timed.as_str()], BackpressurePolicy::Block);
        assert_eq!(
            consumer_only
                .topic("/fb/data")
                .map(|f| f.consumers[0].block_timeout_ms),
            Some(Some(100)),
            "PRECONDITION: the input carries the key"
        );
        let set = BagFedBlockEdges::new([edge("/fb/data")]);
        let ends = set
            .resolve(&consumer_only, &CreditedEdges::none(), "fb")
            .expect("the consumer end resolves");
        consumer_only
            .validate_with_credit_edges(&CreditedEdges::none(), &ends)
            .expect("the timeout on a fed input is accepted");
    }

    /// The producer end resolves, exempts no input and holds the producer the
    /// entry names. Beside a writer of the same topic the entry does not
    /// name, only the named one is held.
    #[test]
    fn a_producer_end_holds_the_producer_the_entry_names_and_exempts_no_input() {
        let producer_only = topology(&[PRODUCER], BackpressurePolicy::Block);
        let set = BagFedBlockEdges::new([edge("/fb/data")]);
        let ends = set
            .resolve(&producer_only, &CreditedEdges::none(), "fb")
            .expect("the producer end resolves");
        assert!(!ends.feeds_consumer("/fb/data", "sink", "inp"));
        assert_eq!(ends.held_producers().collect::<Vec<_>>(), vec!["prod"]);
        let two_writers = topology_listing(
            "multi_publisher_topics:\n  - /fb/data\n",
            &[PRODUCER, OTHER_PRODUCER],
            BackpressurePolicy::Block,
        );
        let ends = set
            .resolve(&two_writers, &CreditedEdges::none(), "fb")
            .expect("the producer end resolves");
        assert_eq!(ends.held_producers().collect::<Vec<_>>(), vec!["prod"]);
    }

    #[test]
    fn an_entry_whose_both_ends_this_build_holds_is_refused() {
        let both = topology(&[PRODUCER, CONSUMER], BackpressurePolicy::Block);
        assert_eq!(
            resolve(&BagFedBlockEdges::new([edge("/fb/data")]), &both),
            Err(format!(
                "{HEAD}but this build holds both its consumer and its producer (prod); a fed \
                 edge's producer runs in another pass"
            ))
        );
    }

    /// Neither end: a build of other nodes, an entry naming an input the
    /// build's `sink` does not declare, and an entry naming a topic `sink`
    /// does not read.
    #[test]
    fn an_entry_this_build_holds_neither_end_of_is_refused() {
        let other = "  - id: other\n    type: p\n    outputs:\n      - name: out\n        \
            schema: geometry_msgs/Vector3\n";
        let unrelated = topology(&[other], BackpressurePolicy::Block);
        assert_eq!(
            resolve(&BagFedBlockEdges::new([edge("/fb/data")]), &unrelated),
            Err(format!(
                "{HEAD}but this build holds neither that input nor any of its producers (prod)"
            ))
        );
        let consumer_only = topology(&[CONSUMER], BackpressurePolicy::Block);
        let mut unknown = edge("/fb/data");
        unknown.consumer_input = "ghost".to_string();
        assert_eq!(
            resolve(&BagFedBlockEdges::new([unknown]), &consumer_only),
            Err(
                "Graph validation error: graph 'fb': the replay wiring feeds `block` edge \
                 '/fb/data' -> 'sink.ghost' from a recording, but this build holds neither that \
                 input nor any of its producers (prod)"
                    .to_string()
            )
        );
        assert_eq!(
            resolve(
                &BagFedBlockEdges::new([edge("/fb/elsewhere")]),
                &consumer_only
            ),
            Err(
                "Graph validation error: graph 'fb': the replay wiring feeds `block` edge \
                 '/fb/elsewhere' -> 'sink.inp' from a recording, but this build holds neither \
                 that input nor any of its producers (prod)"
                    .to_string()
            )
        );
    }

    /// A consumer end must be a `block` input: a `drop_oldest` and a
    /// `sample(N)` input are refused, naming the declared policy.
    #[test]
    fn a_consumer_end_that_is_not_a_block_input_is_refused() {
        for (policy, shown) in [
            (BackpressurePolicy::DropOldest, "DropOldest"),
            (BackpressurePolicy::Sample(5), "Sample(5)"),
        ] {
            let consumer_only = topology(&[CONSUMER], policy);
            assert_eq!(
                resolve(&BagFedBlockEdges::new([edge("/fb/data")]), &consumer_only),
                Err(format!(
                    "{HEAD}but this build declares that input {shown}, not `block`"
                ))
            );
        }
    }

    /// A consumer end whose topic has an in-graph producer the entry does
    /// not name is refused: that producer's local word counts the input, and
    /// a fed edge must never silence it. With the named producer here too,
    /// the build holds both ends.
    #[test]
    fn a_consumer_end_with_a_local_producer_the_entry_does_not_name_is_refused() {
        let mixed = topology(&[OTHER_PRODUCER, CONSUMER], BackpressurePolicy::Block);
        assert_eq!(
            resolve(&BagFedBlockEdges::new([edge("/fb/data")]), &mixed),
            Err(format!(
                "{HEAD}but this build also holds an in-graph producer of the topic the entry \
                 does not name (other), and a `block` input with an in-graph producer counts on \
                 its local word"
            ))
        );
        let all_three = topology_listing(
            "multi_publisher_topics:\n  - /fb/data\n",
            &[PRODUCER, OTHER_PRODUCER, CONSUMER],
            BackpressurePolicy::Block,
        );
        assert_eq!(
            resolve(&BagFedBlockEdges::new([edge("/fb/data")]), &all_three),
            Err(format!(
                "{HEAD}but this build holds both its consumer and its producer (prod); a fed \
                 edge's producer runs in another pass"
            ))
        );
    }

    /// Two entries this build holds neither end of: the one first in `Ord`
    /// order is named, whatever order the set was built in, on every call.
    #[test]
    fn the_refusal_names_the_first_entry_in_order_on_every_build() {
        let consumer_only = topology(&[CONSUMER], BackpressurePolicy::Block);
        let set = BagFedBlockEdges::new([edge("/fb/z"), edge("/fb/a")]);
        let want = Err(
            "Graph validation error: graph 'fb': the replay wiring feeds `block` edge \
                        '/fb/a' -> 'sink.inp' from a recording, but this build holds neither \
                        that input nor any of its producers (prod)"
                .to_string(),
        );
        assert_eq!(resolve(&set, &consumer_only), want);
        assert_eq!(resolve(&set, &consumer_only), want);
        let reversed = BagFedBlockEdges::new([edge("/fb/a"), edge("/fb/z")]);
        assert_eq!(resolve(&reversed, &consumer_only), want);
        assert_eq!(set, reversed);
        assert_eq!(
            set.iter().map(|e| e.topic.as_str()).collect::<Vec<_>>(),
            vec!["/fb/a", "/fb/z"]
        );
        let repeated = BagFedBlockEdges::new([edge("/fb/a"), edge("/fb/a")]);
        assert_eq!(repeated.iter().count(), 1, "a repeat is held once");
        assert!(BagFedBlockEdges::default().is_empty());
        assert!(!repeated.is_empty());
    }
}
