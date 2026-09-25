// SPDX-License-Identifier: AGPL-3.0-only
//! The static `GraphTopology` — the build-time dataflow model.
//!
//! Built once at [`GraphRuntime::build`](crate::graph::runtime::GraphRuntime)
//! from the parsed [`GraphConfig`] plus each node's macro-declared
//! [`NodeInfo`], BEFORE any tick runs. It is the single source of truth for
//! "who publishes each topic, who consumes it, with what backpressure policy
//! and queue depth" — the cross-node facts the per-node `#[input]`/`#[output]`
//! attributes can't express on their own.
//!
//! # Why it exists (two consumers)
//!
//! 1. **Backpressure.** The zero-copy `block` policy defers a
//!    producer's tick the moment a downstream consumer's queue is full; that
//!    decision needs to know, per topic, *which* node is the producer and
//!    *which* consumers are `block` (and at what depth). `sample`/`throttle`
//!    wiring reads the same model. Build-time [`validate`](GraphTopology::validate)
//!    rejects misconfigured graphs LOUDLY before they run (graph-build is
//!    a compile-time check).
//! 2. **The multithreaded DAG scheduler.** This IS the DAG edge
//!    model that scheduler reuses: [`TopicFlow`] `{producer, consumers}` are
//!    the directed edges, and [`ConsumerEdge::depth`] is the per-edge "room
//!    downstream" capacity that — paired at runtime with the `block`
//!    outstanding counter — answers "can this node fire? (inputs ready) AND
//!    is there room downstream? (outputs not full)".
//!
//! # Determinism
//!
//! Topics and consumer edges are recorded in `config.nodes` / port order via
//! an insertion-ordered [`IndexMap`] / `Vec`, so the model is bit-identical
//! across runs (Principle #5: graph is the source of truth; Principle #7:
//! replay = live).

use std::collections::HashSet;

use indexmap::{IndexMap, IndexSet};

use crate::error::{TransportError, TransportResult};
use crate::graph::config::GraphConfig;
use crate::graph::node::{BackpressurePolicy, NodeInfo};

use super::{malformed_absolute_name, resolve_output_topic, resolve_source};

/// Fallback per-consumer queue depth when an input carries no macro
/// `InputMeta` (e.g. a closure-form `ClosureNodeEntry` consumer that never
/// went through `#[cerulion_node]`). Matches the macro's own default fifo
/// depth so topology-driven pool sizing is consistent with the declared
/// case. `block` consumers ALWAYS have macro metadata (the policy is a
/// macro attr), so this fallback only ever sizes non-`block`,
/// informational edges.
pub const DEFAULT_CONSUMER_DEPTH: usize = 10;

/// Hard ceiling on any consumer's declared queue depth
/// (`#[input(depth = N)]`) and any output's `history_size` (graph YAML).
///
/// Every unit of depth is one whole `max_slice_len`-sized SHM slot on
/// the topic's publisher pool: iceoryx2's default
/// `AllocationStrategy::Static` reserves full-size buckets, so a depth-64
/// queue on a 128 MiB (`TIER_HUGE`) topic reserves 8 GiB of
/// shared memory. That reservation is APPARENT, not resident — the pool
/// is lazy/demand-paged (resident tracks the working set), but
/// it still counts against the `/dev/shm` `size=` ceiling and the
/// process address space, so the cap keeps the worst case bounded and
/// documented. A consumer
/// that "needs" more than 64 in-flight messages is a design smell —
/// the consumer is permanently slower than the producer, which is a
/// backpressure-policy problem (`drop_oldest` / `sample(N)` /
/// `block`), not a buffering problem.
///
/// `history_size` shares the cap because history replays through the
/// same consumer queues: the per-topic iceoryx2
/// `subscriber_max_buffer_size` ceiling is sized from declared depths
/// (≤ 64), and iceoryx2-native history requires
/// `history ≤ subscriber buffer` for full delivery — a larger history
/// could never be delivered in full.
///
/// The macro mirrors this value at expand time (compile-time
/// rejection in `cerulion_macros`); this const is the runtime source
/// of truth enforced by [`GraphTopology::validate`], which also covers
/// non-macro consumers (closure nodes, cdylibs built against older
/// macro versions).
pub const MAX_CONSUMER_DEPTH: usize = 64;

/// One `(consumer-node, input)` edge consuming a topic.
///
/// **DAG edge + flow model:** `depth` is the per-edge "room
/// downstream"; the runtime `block` outstanding counter is the matching
/// numerator.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConsumerEdge {
    /// Consuming node's id.
    pub node_id: String,
    /// The consuming node's input field name.
    pub input: String,
    /// The input's declared backpressure policy.
    pub policy: BackpressurePolicy,
    /// The input's declared queue depth (`#[input(depth = N)]`), or
    /// [`DEFAULT_CONSUMER_DEPTH`] for non-macro consumers.
    pub depth: usize,
}

/// The set of `(topic, consumer node, consumer input)` edges a
/// build carries a CROSS-PROCESS CREDIT WORD for — the one exemption
/// [`GraphTopology::validate`]'s producer-less-`block` arm honours.
///
/// # Why a newtype rather than a bare `HashSet<(&str, &str, &str)>`
///
/// The exemption turns a REFUSAL into a build, so the question "may this edge
/// be exempted?" has exactly one valid answer: *because a credit word for it
/// was opened*. A bare set of three strings is forgeable from three string
/// literals, which would make the exemption assertable by anyone who could
/// spell the edge. The only non-empty constructor is
/// [`from_bindings`](Self::from_bindings), and it credits ONLY the bindings
/// carrying a MAPPED word.
///
/// The `is_mapped` filter is the half that makes the claim true rather than
/// merely narrow. `CreditBinding` is public with public fields and
/// [`CreditWord::local`](crate::credit::CreditWord::local) is public, so a
/// binding over a process-LOCAL heap word is constructible by anyone — and a
/// local word is exactly what a producer-less `block` topic must NOT be
/// exempted on, because nothing in another process is holding the other end of
/// it. Filtering here means the token can only ever name edges some process
/// really did `open_unowned` a shared page for.
#[derive(Debug, Default)]
pub struct CreditedEdges<'a> {
    edges: HashSet<(&'a str, &'a str, &'a str)>,
}

impl<'a> CreditedEdges<'a> {
    /// No credited edge at all — every `block` consumer is judged by the
    /// ordinary producer-less rule. The single-process default.
    pub fn none() -> Self {
        Self {
            edges: HashSet::new(),
        }
    }

    /// The credited set implied by a worker's opened credit bindings — the
    /// ONLY way to build a non-empty token.
    ///
    /// A binding whose word is a process-LOCAL one credits NOTHING: see the
    /// type's docs for why that filter is what makes the exemption sound.
    pub fn from_bindings(bindings: &'a [crate::graph::runtime::CreditBinding]) -> Self {
        Self {
            edges: bindings
                .iter()
                .filter(|b| b.word.is_mapped())
                .map(|b| {
                    (
                        b.topic.as_str(),
                        b.consumer_node.as_str(),
                        b.consumer_input.as_str(),
                    )
                })
                .collect(),
        }
    }

    /// Is this `(topic, node, input)` edge credited?
    pub(crate) fn contains(&self, topic: &str, node: &str, input: &str) -> bool {
        self.edges.contains(&(topic, node, input))
    }
}

/// One topic's in-graph producers plus all of its consumer edges. **The
/// DAG edge set for a topic.**
///
/// `#[non_exhaustive]`: blocks external literal
/// construction — the `producers` invariants (deduped, per-node,
/// multi-only-if-listed) are enforced by [`GraphTopology::build`] +
/// re-checked by [`GraphTopology::validate`], and an
/// external-crate literal would bypass both (the same hardening
/// `TopicServiceConfig` has).
#[derive(Debug, Clone)]
#[non_exhaustive]
pub struct TopicFlow {
    /// The resolved topic name (prefix-qualified).
    pub topic: String,
    /// The ids of the in-graph nodes that publish this topic, in graph
    /// order, DEDUPLICATED (a node publishing the topic through two
    /// outputs appears once — the unit here is the producer NODE, which
    /// is what block defer and scheduling act on). Empty for a topic with
    /// no in-graph producer (an external publisher); build-time
    /// validation rejects `block` consumers on producer-less topics. More
    /// than one entry is only legal for topics listed in
    /// `multi_publisher_topics` — `build` rejects the
    /// unlisted double-producer.
    pub producers: Vec<String>,
    /// Every `(node, input)` that consumes this topic, in graph order.
    pub consumers: Vec<ConsumerEdge>,
}

/// Why a topic's `block` edges cannot carry a cross-process
/// credit word. Produced by [`TopicFlow::credit_bar`]; rendered verbatim into
/// the plan-time refusal, which is why each variant carries what its sentence
/// has to name.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CreditBar {
    /// No in-graph producer — there is nothing this scheduler can defer.
    /// `GraphTopology::validate` refuses `block` here with its own message, so
    /// a partition remedy would be answering a question nobody asked.
    NoInGraphProducer,
    /// Two or more in-graph producers (`multi_publisher_topics`). The word
    /// counts ONE producer's outstanding frames, so two writers would each
    /// spend the other's credit. Always `>= 2`.
    MultipleProducers(usize),
    /// The topic carries BOTH `block` consumer(s) and non-`block` sibling(s),
    /// so the `block` ones are degraded to `drop_oldest` and there is no
    /// lossless defer left to credit. Carries the siblings — at least one, by
    /// construction — because they are the nodes the operator has to move.
    ///
    /// Both halves are load-bearing: a topic with siblings but NO `block`
    /// consumer is [`CreditBar::NoBlockConsumer`], not this.
    MixedTopic { first: String, rest: Vec<String> },
    /// One producer and NO `block` consumer — either the topic has no
    /// consumers at all, or only non-`block` ones. Either way there is no
    /// defer to credit.
    ///
    /// Unreachable from a `block_colocation_seeds` seed (which requires a
    /// `block` consumer), reachable on any other flow. Distinct from
    /// [`CreditBar::MixedTopic`], which means the topic has `block` consumers
    /// AND non-`block` siblings, so the `block` ones are degraded — a claim
    /// that is only true when such consumers exist.
    NoBlockConsumer,
}

impl CreditBar {
    /// The non-`block` consumers this bar blames, in graph order. Empty for
    /// every variant but [`CreditBar::MixedTopic`], which always names ≥ 1.
    pub fn siblings(&self) -> Vec<&str> {
        match self {
            Self::MixedTopic { first, rest } => std::iter::once(first.as_str())
                .chain(rest.iter().map(String::as_str))
                .collect(),
            _ => Vec::new(),
        }
    }
}

impl TopicFlow {
    /// True when the topic has ≥1 consumer and EVERY consumer is `block`.
    /// Only an all-`block` topic is eligible for producer pre-fire defer:
    /// deferring would otherwise starve a non-`block` sibling (decision K),
    /// so a mixed topic degrades its `block` consumers to `drop_oldest`.
    pub fn is_all_block(&self) -> bool {
        !self.consumers.is_empty()
            && self
                .consumers
                .iter()
                .all(|c| matches!(c.policy, BackpressurePolicy::Block))
    }

    /// Why this topic's `block` edges cannot carry a
    /// cross-process credit word — `None` means they CAN.
    ///
    /// # This is the ONE body of the creditability rule
    ///
    /// Two halves of the system have to agree about it, in different crates:
    /// `partition::validate_block_colocation` decides whether a partition that
    /// SPLITS such an edge is refused, and `cerulion_cli_engine`'s
    /// `credit_edges_for` decides whether the supervisor MINTS a word for it.
    /// They are a matched pair, and the two failure directions are not
    /// symmetric:
    ///
    /// - accept something the mint declines ⇒ the partition passes plan-time
    ///   validation and the consumer's worker then dies at
    ///   [`GraphTopology::validate`],
    ///   blaming an external publisher the operator does not have;
    /// - refuse something the mint accepts ⇒ a deployment that would have run
    ///   correctly is rejected and the credit word is unreachable.
    ///
    /// So the rule is written ONCE, here on the type both halves already hold,
    /// and each half calls it. Spelling it twice — once over a
    /// `BlockColocationSeed` and once over a `TopicFlow` — would let the two
    /// halves drift apart.
    ///
    /// # The variants are the operator's diagnosis
    ///
    /// The refusal has to say WHICH bar it hit, because "a split `block` edge
    /// cannot be honoured" is not true of every split: some run. `MixedTopic`
    /// carries the siblings it names so the message can point at the node the
    /// operator has to move, rather than a second field that must be kept in
    /// agreement with this decision.
    ///
    /// The rank half of the mint's condition (`producer_rank !=
    /// consumer_rank`) is NOT here: it is a property of the partition, not of
    /// the flow, and each caller already has it — the validator as its
    /// `pg == cg` arm, the mint as its rank lookup.
    ///
    /// Pure; allocates only when it is naming siblings.
    pub fn credit_bar(&self) -> Option<CreditBar> {
        match self.producers.len() {
            // Nothing to defer, and `validate` already refuses `block` here
            // with its own specific message.
            0 => Some(CreditBar::NoInGraphProducer),
            1 => {
                // ASKED FIRST, and the order is the correctness point: a topic
                // with one producer, NO `block` consumer and one or more
                // `drop_oldest` ones is not "mixed" in any sense the refusal
                // could act on — there is nothing to degrade and nothing to
                // defer. Reporting `MixedTopic` there renders a sentence about
                // "its `block` consumers" that names consumers which do not
                // exist.
                if !self.has_block_consumer() {
                    return Some(CreditBar::NoBlockConsumer);
                }
                let mut siblings = self
                    .consumers
                    .iter()
                    .filter(|c| !matches!(c.policy, BackpressurePolicy::Block))
                    .map(|c| format!("{}.{}", c.node_id, c.input));
                // `None` here means every consumer is `block`, and the guard
                // above proved there is at least one: THE creditable shape.
                siblings.next().map(|first| CreditBar::MixedTopic {
                    first,
                    rest: siblings.collect(),
                })
            }
            // `> 1` by construction; a credit word counts ONE producer's
            // outstanding frames, so two writers would each spend the other's.
            n => Some(CreditBar::MultipleProducers(n)),
        }
    }

    /// Can this topic's `block` edges carry a cross-process
    /// credit word? Defined AS `credit_bar().is_none()`, so the predicate and
    /// the reason cannot disagree — it is one function, not two that agree.
    ///
    /// Equivalent to `producers.len() == 1 && is_all_block()`, which is the
    /// condition the mint is written in terms of; the equivalence is pinned
    /// over the whole shape space rather than left to inspection.
    pub fn is_creditable(&self) -> bool {
        self.credit_bar().is_none()
    }

    /// The topic's ONE in-graph producer, or `None` when it has none (an
    /// outside publisher) or several (a `multi_publisher_topics` topic).
    ///
    /// The half of the single-writer question that [`producers_of`] cannot
    /// answer without the caller re-deciding what "one" means.
    ///
    /// [`producers_of`]: GraphTopology::producers_of
    pub fn sole_producer(&self) -> Option<&str> {
        match self.producers.as_slice() {
            [only] => Some(only.as_str()),
            _ => None,
        }
    }

    /// The topic's ONE consumer edge, or `None` when it has none or several.
    ///
    /// The reader-side twin of [`sole_producer`](Self::sole_producer). A topic
    /// with two consumer edges is a fan-out, and the pair is what lets a caller
    /// ask "is this topic one writer talking to one reader?" in the terms the
    /// model already uses, instead of reaching into `consumers` and deciding
    /// again. Two edges on the SAME node (one topic wired to two inputs) are
    /// two consumers here, which is what they are to the scheduler.
    pub fn sole_consumer(&self) -> Option<&ConsumerEdge> {
        match self.consumers.as_slice() {
            [only] => Some(only),
            _ => None,
        }
    }

    /// True when at least one consumer declared `block`.
    pub fn has_block_consumer(&self) -> bool {
        self.consumers
            .iter()
            .any(|c| matches!(c.policy, BackpressurePolicy::Block))
    }

    /// Sum of every consumer's declared depth — the lower bound on the
    /// iceoryx2 publisher pool's slot count for this topic (before history +
    /// loan headroom).
    pub fn sum_consumer_depths(&self) -> usize {
        self.consumers.iter().map(|c| c.depth).sum()
    }
}

/// Static, build-time model of the graph's dataflow (see module docs).
#[derive(Debug, Clone, Default)]
pub struct GraphTopology {
    /// Resolved-topic → [`TopicFlow`]. Insertion-ordered for deterministic
    /// iteration (producers recorded first in `config.nodes` order, then
    /// consumers).
    topics: IndexMap<String, TopicFlow>,
    /// The resolved
    /// `multi_publisher_topics` opt-in set, captured at build so the
    /// multi-producer LEGALITY invariant (`producers.len() > 1` only for
    /// listed topics) is a property of the VALUE re-checkable by
    /// [`Self::validate`], not a fact that lived only in `build`'s stack
    /// frame (the pub reuse surface can be fed literal-mutated
    /// models).
    multi_publisher_topics: HashSet<String>,
    /// Every node id in `config.nodes` order. The
    /// topology is otherwise topic-keyed, so a node with NO topic edges (a
    /// fully disconnected node, or a side-effect-only Period node) would be
    /// invisible to [`Self::derive_levels`]. Capturing the full node set here
    /// makes the levelization COMPLETE — every node is placed (a disconnected
    /// node carries in-degree 0 → level-0 root) so the level executor can
    /// never silently skip it.
    node_order: Vec<String>,
}

impl GraphTopology {
    /// Build the topology from the parsed graph + per-node macro metadata.
    ///
    /// `entry_infos` is the `info()` snapshot the graph runtime already
    /// caches (one FFI call per node); pass it by reference — no extra cost.
    /// Each consumer edge's `policy` + `depth` come from the consuming
    /// node's `InputMeta` (macro-declared); inputs without macro metadata
    /// get the default policy + [`DEFAULT_CONSUMER_DEPTH`].
    ///
    /// Errors on a topic published by two in-graph nodes (ambiguous
    /// producer).
    pub fn build(
        config: &GraphConfig,
        entry_infos: &IndexMap<String, NodeInfo>,
    ) -> TransportResult<Self> {
        let mut topics: IndexMap<String, TopicFlow> = IndexMap::new();

        // Pass 1: producers (every node output → exactly one topic).
        for node_def in config.native_nodes() {
            for output in &node_def.outputs {
                // Routes through the override-aware
                // resolver — a `topic: /tf` output records its producer
                // edge under the ABSOLUTE name. Defense in
                // depth: `build` is the pub reuse
                // surface and can be fed an UNVALIDATED config, so the
                // override-shape invariant is re-checked here (the
                // duplicate-edge checks below set the precedent).
                if let Some(t) = &output.topic {
                    if !t.starts_with('/') || malformed_absolute_name(t).is_some() {
                        return Err(TransportError::GraphError {
                            reason: format!(
                                "node '{}' output '{}': `topic: {}` must be a \
                                 well-formed absolute name (validate_graph \
                                 normally rejects this before topology build)",
                                node_def.id, output.name, t
                            ),
                        });
                    }
                }
                let topic = resolve_output_topic(&config.prefix, &node_def.id, output);
                let flow = topics.entry(topic.clone()).or_insert_with(|| TopicFlow {
                    topic: topic.clone(),
                    producers: Vec::new(),
                    consumers: Vec::new(),
                });
                // Topics listed in
                // `multi_publisher_topics` legally carry multiple in-graph
                // producers; unlisted topics keep the single-producer
                // rule — ANY second output to the same unlisted topic
                // rejects, including a second output on the SAME node
                // (parity with the earlier single-producer check; validate_graph's
                // duplicate-output-topic check rejects that shape earlier
                // on the normal path, this is the defense-in-depth for
                // unvalidated pub-surface configs). The producer list
                // is per NODE (deduped): a node publishing a listed topic
                // through two outputs is one deferrable producer, and the
                // runtime creates its publishers per output regardless.
                if let Some(existing) = flow.producers.first() {
                    if !config.is_multi_publisher(&topic) {
                        return Err(TransportError::GraphError {
                            reason: format!(
                                "topic '{topic}' is published by two in-graph nodes \
                                 ('{existing}' and '{}'); each topic must have a single \
                                 producer (if multiple publishers are intentional, list \
                                 the topic in `multi_publisher_topics:`)",
                                node_def.id
                            ),
                        });
                    }
                }
                if !flow.producers.contains(&node_def.id) {
                    flow.producers.push(node_def.id.clone());
                }
            }
        }

        // Pass 2: consumers (every node input → the topic it reads), with
        // policy + depth from the consuming node's macro `InputMeta`.
        for node_def in config.native_nodes() {
            let info = entry_infos.get(&node_def.id);
            // Duplicate input names
            // in a node's declared port METADATA are impossible for macro
            // structs (Rust rejects duplicate fields).
            // `NodeInfo::with_meta` asserts the same invariant at
            // construction, so this
            // build check is defense-in-depth: it guards in-crate literal
            // construction (fields are `pub(crate)`) and any future
            // constructor — the `.find()` below would otherwise silently
            // resolve every lookup to the FIRST meta, ignoring a later
            // duplicate's conflicting policy/depth. REJECT at build (this
            // is internal-Rust surface, not FFI/user surface — a warn +
            // first-meta-wins was the original shape, replaced by
            // rejection).
            if let Some(i) = info {
                let metas = i.input_meta();
                for (idx, m) in metas.iter().enumerate() {
                    if metas[..idx].iter().any(|p| p.name == m.name) {
                        return Err(TransportError::GraphError {
                            reason: format!(
                                "node '{}' declares duplicate input metadata for \
                                 '{}' — each port may carry exactly one InputMeta \
                                 (conflicting policy/depth declarations are ambiguous)",
                                node_def.id, m.name
                            ),
                        });
                    }
                }
            }
            for input in &node_def.inputs {
                let topic = resolve_source(&config.prefix, &input.source);
                let (policy, depth) = info
                    .and_then(|i| i.input_meta().iter().find(|m| m.name == input.name))
                    .map(|m| (m.backpressure, m.depth))
                    .unwrap_or((BackpressurePolicy::default(), DEFAULT_CONSUMER_DEPTH));
                let flow = topics.entry(topic.clone()).or_insert_with(|| TopicFlow {
                    topic: topic.clone(),
                    producers: Vec::new(),
                    consumers: Vec::new(),
                });
                flow.consumers.push(ConsumerEdge {
                    node_id: node_def.id.clone(),
                    input: input.name.clone(),
                    policy,
                    depth,
                });
            }
        }

        Ok(Self {
            topics,
            multi_publisher_topics: config.multi_publisher_topics.iter().cloned().collect(),
            node_order: config.native_nodes().map(|n| n.id.clone()).collect(),
        })
    }

    /// Validate the static contract, failing LOUDLY before any tick
    /// (graph-build = "compile-time"):
    ///
    /// * every consumer `depth` is ≥ 1 (a zero-depth queue can hold nothing);
    /// * every consumer `depth` is ≤ [`MAX_CONSUMER_DEPTH`] (each unit of
    ///   depth commits a full `max_slice_len`-sized SHM slot — the cap
    ///   bounds the worst-case shared-memory commitment);
    /// * every `block` consumer's topic has an in-graph producer (the
    ///   scheduler can only defer a producer it controls — an external
    ///   publisher can't be back-pressured).
    ///
    /// (Pool-slot-count provisioning + the `pool ≥ Σdepths` check land with
    /// the `block` enforcement that consumes them.)
    pub fn validate(&self) -> TransportResult<()> {
        self.validate_with_credit_edges(&CreditedEdges::none())
    }

    /// [`Self::validate`] with the set of `(topic, consumer node,
    /// consumer input)` edges this build carries a CROSS-PROCESS CREDIT WORD
    /// for — the one exemption to the producer-less-`block` refusal below.
    ///
    /// # Why an exemption is sound here and nowhere else
    ///
    /// The refusal exists because "this scheduler can only defer a producer in
    /// THIS graph". A credited edge has a producer the scheduler CAN defer — it
    /// is simply in another PROCESS, and the shared credit word
    /// ([`crate::credit::CreditShared`]) is exactly the channel that carries
    /// "your consumer is full" across the boundary. So the premise of the
    /// refusal is false for these edges and true for every other one; the
    /// lossless guarantee (Principle #6) is upheld by the word rather than by
    /// co-location.
    ///
    /// The token is PLAN-CARRIED and stamped by the supervisor, which is the
    /// only party that holds the whole graph. A worker cannot derive it — that
    /// is the entire reason the edge looks producer-less from inside the
    /// subgraph — so an exemption derived locally would be an exemption derived
    /// from the absence of the evidence. Same shape as the ingress
    /// exemption: a build-parameter fact, never an inference.
    ///
    /// It is NOT gated on the execution mode. `graph levels` has no mode, and a
    /// credit word is what makes the edge REPRESENTABLE, not how the ranks
    /// coordinate their steps.
    pub(crate) fn validate_with_credit_edges(
        &self,
        credited: &CreditedEdges<'_>,
    ) -> TransportResult<()> {
        // Defense in depth: a
        // (node, input) pair may appear at most once across ALL consumer
        // edges. `validate_graph` already rejects duplicate per-node input
        // names on every runtime path, but `GraphTopology::build` is `pub`
        // and is the advertised reuse surface for the scheduler — a
        // future caller building topology from an unvalidated config would
        // otherwise carry two same-keyed edges into any keyed fold-down
        // (the silent-overwrite class behind the orphaned-block-mirror
        // stall).
        let mut seen_edges: HashSet<(&str, &str)> = HashSet::new();
        for flow in self.topics.values() {
            for c in &flow.consumers {
                if !seen_edges.insert((c.node_id.as_str(), c.input.as_str())) {
                    return Err(TransportError::GraphError {
                        reason: format!(
                            "duplicate consumer edge: node '{}' declares input name '{}' \
                             more than once (duplicate input names in the graph config)",
                            c.node_id, c.input
                        ),
                    });
                }
            }
        }
        // The `producers` twins of the
        // duplicate-edge defense above — `build` enforces both invariants,
        // but the pub surface can carry literal-mutated models.
        // (1) per-NODE dedup: a duplicated producer entry would
        // double-register the node's block defer edges; (2) legality:
        // multiple producers are only legal for topics in the captured
        // opt-in set — without this re-check the invariant lived only in
        // `build`'s stack frame.
        for flow in self.topics.values() {
            let mut seen_producers: HashSet<&str> = HashSet::new();
            for p in &flow.producers {
                if !seen_producers.insert(p.as_str()) {
                    return Err(TransportError::GraphError {
                        reason: format!(
                            "topic '{}' lists producer node '{}' more than once — \
                             producers are per-node, deduplicated",
                            flow.topic, p
                        ),
                    });
                }
            }
            if flow.producers.len() > 1 && !self.multi_publisher_topics.contains(&flow.topic) {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "topic '{}' carries {} producers but is not listed in \
                         `multi_publisher_topics:` — multiple producers are only \
                         legal for opted-in topics",
                        flow.topic,
                        flow.producers.len()
                    ),
                });
            }
        }
        for flow in self.topics.values() {
            for c in &flow.consumers {
                if c.depth == 0 {
                    return Err(TransportError::GraphError {
                        reason: format!(
                            "input '{}.{}' on topic '{}' declares depth = 0; queue depth must \
                             be >= 1",
                            c.node_id, c.input, flow.topic
                        ),
                    });
                }
                if c.depth > MAX_CONSUMER_DEPTH {
                    return Err(TransportError::GraphError {
                        reason: format!(
                            "input '{}.{}' on topic '{}' declares depth = {}; the maximum \
                             queue depth is {MAX_CONSUMER_DEPTH} (every unit of depth commits \
                             a full max_slice_len-sized SHM slot). A consumer needing more \
                             than {MAX_CONSUMER_DEPTH} in-flight messages is permanently \
                             slower than its producer — use a backpressure policy \
                             (drop_oldest / sample(N) / block) instead of a deeper queue",
                            c.node_id, c.input, flow.topic, c.depth
                        ),
                    });
                }
                if matches!(c.policy, BackpressurePolicy::Block)
                    && flow.producers.is_empty()
                    && !credited.contains(&flow.topic, &c.node_id, &c.input)
                {
                    // This message reaches TWO different situations
                    // and must not prescribe one's remedy for the other.
                    //
                    // (a) the topic is genuinely published from outside this
                    //     graph — the original case, where `drop_oldest` /
                    //     `sample(N)` really is the fix; and
                    // (b) the graph DOES produce it, but a multi-process
                    //     partition put the producer in another worker, whose
                    //     nodes `subgraph_for` filtered out of this subgraph.
                    //     Here there is no external publisher at all, and
                    //     "use drop_oldest" prescribes silent data loss for a
                    //     partitioning mistake.
                    //
                    // The two are NOT distinguishable from here: a worker's
                    // subgraph has `process_groups` CLEARED (it is a
                    // single-process monolith by construction), so nothing in
                    // this config says which situation it is. Rather than
                    // thread a flag through `validate` for a message, both
                    // remedies are stated and LABELLED with the condition that
                    // selects them. Case (b) is normally pre-empted at plan
                    // time by `partition::validate_block_colocation`, which
                    // has the whole graph in view; this stays reachable for a
                    // worker reached some other way.
                    return Err(TransportError::GraphError {
                        reason: format!(
                            "block input '{}.{}' reads topic '{}', which has no in-graph \
                             producer; this scheduler can only defer producers in THIS \
                             graph — deferring an out-of-graph publisher needs the \
                             host-level multi-graph scheduler. If '{}' really is \
                             published from OUTSIDE this graph, use drop_oldest or \
                             sample(N) on this input. If this graph DOES produce '{}' and \
                             you are reading this from a multi-process WORKER, the \
                             partition split the producer away from this consumer and the \
                             plan carried no cross-process credit word for the edge — one \
                             is minted only for a topic with exactly ONE in-graph producer \
                             and NO non-`block` consumers. Put them in ONE \
                             `process_groups:` group, or run `--single-process` — never \
                             switch to drop_oldest for that, it silently loses data (the \
                             derived partition co-locates `block` edges automatically)",
                            c.node_id, c.input, flow.topic, flow.topic, flow.topic
                        ),
                    });
                }
            }
        }
        Ok(())
    }

    /// Look up a topic's flow (producers + consumers).
    pub fn topic(&self, topic: &str) -> Option<&TopicFlow> {
        self.topics.get(topic)
    }

    /// Iterate every topic's flow, in deterministic graph order.
    pub fn topics(&self) -> impl Iterator<Item = &TopicFlow> {
        self.topics.values()
    }

    /// The in-graph producer node ids for `topic` (empty when the topic is
    /// unknown or has no in-graph producer). This replaced the
    /// old single-`producer_of` accessor: a `multi_publisher_topics`-listed
    /// topic can carry several producers, and a "first producer" read
    /// would silently drop the rest.
    pub fn producers_of(&self, topic: &str) -> &[String] {
        self.topics
            .get(topic)
            .map(|f| f.producers.as_slice())
            .unwrap_or(&[])
    }

    /// Derive the trigger-aware DAG levels.
    ///
    /// `trigger_edges` is the runtime-supplied classification of which
    /// `(consumer_node_id, topic)` consumer edges are TRIGGERING — see
    /// [`TriggerEdges`]. Keeping the classification OUTSIDE
    /// `GraphTopology` is deliberate: the trigger-ness of an edge lives in
    /// `NodeInfo` (`#[input(trigger)]` + the node's `MacroPolicy`), not in
    /// the YAML-built topology. The topology stays policy-free; the runtime
    /// supplies the predicate.
    ///
    /// # Algorithm (Kahn / BFS in-degree peeling)
    ///
    /// Build the directed dependency graph over TRIGGERING edges only:
    /// `producer -> consumer` for every triggering consumer edge. A
    /// consumer of a multi-publisher topic depends on ALL N producers (its
    /// in-degree counts each), so its level is `max(producer levels) + 1`.
    /// Each Kahn peel wave is one level. Nodes surviving with in-degree > 0
    /// form an algebraic cycle and are reported via [`CycleError`].
    ///
    /// **Trigger-aware, NOT structural.** A non-triggering input is a
    /// LATEST-VALUE read (the node reads the last published sample, it is
    /// not woken by it) — it does NOT create a DAG edge, does NOT constrain
    /// level order, and CANNOT form a cycle. This is what makes a feedback
    /// loop through a non-trigger read (e.g. an estimator reading the last
    /// command) LEGAL: only a loop composed ENTIRELY of triggering edges is
    /// an algebraic loop, and only that is rejected.
    ///
    /// **Dependency note (this function is structure only):** excluding
    /// non-trigger edges from the DAG is correct ONLY because a
    /// non-trigger read is a step-boundary `Sample`-snapshot of the latest
    /// value. The deterministic-freshness semantics of that snapshot (read
    /// the value as of the START of the step, regardless of within-step
    /// producer order) live in the runtime. This function does NOT implement snapshotting —
    /// it only builds the level structure that the level executor consumes.
    ///
    /// # Determinism
    ///
    /// Within a level, nodes appear in graph (`config.nodes`) order —
    /// derived here from `topics()` producer/consumer insertion order, which
    /// is itself graph-ordered (Principle #5/#7). Roots (level 0) are nodes
    /// with no triggering in-edge, in graph order.
    pub fn derive_levels(&self, trigger_edges: &TriggerEdges) -> Result<Levels, CycleError> {
        // 1. Enumerate every node in deterministic graph (`config.nodes`)
        //    order from the build-captured `node_order`. This is the
        //    COMPLETE node set, so a fully disconnected node (no producer or
        //    consumer edge — e.g. a side-effect-only Period node) IS included:
        //    it carries in-degree 0 and lands as a level-0 root, keeping the
        //    levelization complete (the level executor never silently skips
        //    it). `IndexSet` preserves graph order, so within-level ordering
        //    is graph-order-stable (Principle #5/#7). Every edge endpoint
        //    (producer/consumer) is an in-graph node id, hence already in
        //    `node_order` — seeding from it loses nothing.
        let mut nodes: IndexSet<String> = IndexSet::new();
        for id in &self.node_order {
            nodes.insert(id.clone());
        }

        // 2. Build the triggering-edge adjacency: producer -> [consumers]
        //    and per-node in-degree, counting ONLY edges the runtime
        //    classified as triggering. A multi-publisher consumer depends
        //    on every producer of the topic, so its in-degree accrues one
        //    per (producer, this triggering edge) pair.
        let mut in_degree: IndexMap<&str, usize> = IndexMap::new();
        for n in &nodes {
            in_degree.insert(n.as_str(), 0);
        }
        // Adjacency keyed by producer node → the consumer nodes it triggers,
        // in deterministic (topic, consumer) insertion order.
        let mut adj: IndexMap<&str, Vec<&str>> = IndexMap::new();
        for n in &nodes {
            adj.insert(n.as_str(), Vec::new());
        }
        for flow in self.topics.values() {
            for c in &flow.consumers {
                if !trigger_edges.is_triggering(&c.node_id, &flow.topic) {
                    // Non-trigger (latest-value) read: NOT a DAG edge. See
                    // the dependency note above — its freshness is a
                    // step-boundary Sample-snapshot concern of the runtime.
                    continue;
                }
                // A triggering edge from EVERY producer of the topic to this
                // consumer. Producer-less (external) triggering topics add no
                // edge — the consumer stays a root, driven from outside.
                for producer in &flow.producers {
                    // Self-edge guard: a node triggering on its own output is
                    // an algebraic self-loop; record it so the in-degree
                    // survives Kahn and the cycle DFS names it.
                    adj.get_mut(producer.as_str())
                        .expect("producer is in the node set")
                        .push(c.node_id.as_str());
                    *in_degree
                        .get_mut(c.node_id.as_str())
                        .expect("consumer is in the node set") += 1;
                }
            }
        }

        // 3. Kahn peel: each wave of in-degree-0 nodes is one level, in graph
        //    order. `rank` records each node's final level for O(1) lookup.
        let mut rank: IndexMap<String, usize> = IndexMap::new();
        let mut levels: Vec<Level> = Vec::new();
        // The frontier is every node currently at in-degree 0, in graph order.
        let mut peeled = 0usize;
        let total = nodes.len();
        // Seed the first wave.
        let mut frontier: Vec<&str> = nodes
            .iter()
            .map(|s| s.as_str())
            .filter(|n| in_degree[*n] == 0)
            .collect();
        while !frontier.is_empty() {
            let level_idx = levels.len();
            // Sort the wave into graph order (the nodes IndexSet's first-seen
            // order). `frontier` is built either from the graph-ordered node
            // scan (the first wave) or by appending newly-zeroed consumers in
            // adjacency order; re-key against the node set's index to make the
            // within-level order graph-stable regardless of which producer
            // zeroed the consumer.
            let mut wave: Vec<&str> = frontier.clone();
            wave.sort_by_key(|n| nodes.get_index_of(*n).expect("node in set"));
            let mut next: Vec<&str> = Vec::new();
            let mut level_nodes: Vec<String> = Vec::with_capacity(wave.len());
            for n in &wave {
                level_nodes.push((*n).to_string());
                rank.insert((*n).to_string(), level_idx);
                peeled += 1;
                // Decrement every consumer this node triggers; newly-zeroed
                // consumers join the next wave.
                for &consumer in &adj[*n] {
                    let d = in_degree
                        .get_mut(consumer)
                        .expect("consumer tracked in in_degree");
                    *d -= 1;
                    if *d == 0 {
                        next.push(consumer);
                    }
                }
            }
            levels.push(Level { nodes: level_nodes });
            frontier = next;
        }

        // 4. Any node still carrying in-degree > 0 is in an algebraic cycle.
        if peeled < total {
            let survivors: IndexSet<&str> = nodes
                .iter()
                .map(|s| s.as_str())
                .filter(|n| in_degree[*n] > 0)
                .collect();
            return Err(self.extract_cycle(&survivors, &adj));
        }

        Ok(Levels { levels, rank })
    }

    /// COST-AWARE LEVEL REFINEMENT — relocate expensive, slow-rate
    /// nodes (and, where needed, whole dense chains) LATER so that a cheap,
    /// fast chain's gating levels stay cheap. Two phases: a within-slack
    /// latest-shift (fixed level count), then a CHAIN-CASCADE pass that may
    /// GROW the level count (see phase 2 below).
    ///
    /// # The problem this solves
    ///
    /// A level-`L+1` consumer cannot fire until EVERY node at level `L` has
    /// finished (the executor's per-level rayon join / the multi-process
    /// barrier crossing). So a fast chain's end-to-end latency is bounded below
    /// by the compute of its *gating* levels — even the parts of those levels
    /// it never reads. A 1 kHz proprioceptive chain whose sink sits at level 1
    /// waits for ALL of level 0, so a 512 µs camera node parked at level 0
    /// (because it is a source and Kahn puts every source at level 0) inflates
    /// the 1 kHz chain's latency by ~512 µs even though the two never exchange
    /// data.
    ///
    /// Kahn levelization ([`derive_levels`](Self::derive_levels)) assigns every
    /// node its EARLIEST legal level (ASAP). Many nodes have slack: an expensive
    /// camera root whose only trigger-consumer sits at level 2 (because that
    /// consumer also depends on a level-1 node) could legally sit at level 0 OR
    /// level 1 and still fire strictly before its consumer. This pass spends
    /// that slack to move expensive independent nodes off the cheap chains'
    /// gating levels.
    ///
    /// # Phase 1 — rate-guarded latest-slack shift (fixed level count)
    ///
    /// For each node we compute its slack window `[ASAP, ALAP]`, then place it
    /// at the LATEST level in that window that a rate guard permits:
    ///
    /// * **ASAP** = the Kahn level (from `base.rank`).
    /// * **ALAP** = `min(final level of each trigger-successor) − 1`. Computed
    ///   during a single REVERSE-topological pass (successors placed before
    ///   predecessors), so `ALAP` is always taken against successors' FINAL
    ///   levels — this is what makes the producer-below-consumer invariant hold
    ///   by construction (see "Correctness" below), never by a fix-up.
    /// * **Bias late.** Within `(ASAP, ALAP]` we scan from `ALAP` downward and
    ///   take the first (= latest) level the guard allows. Latest is right for
    ///   the objective: moving a node to level `d` VACATES every level in
    ///   `[ASAP, d−1]`, and cheap fast chains are typically shallow, so
    ///   vacating the earliest levels is exactly the relief they need.
    /// * **Rate guard (the rate weighting).** Define a node's *gating rate* as
    ///   the max `edge_rate_mhz` over its OUTGOING trigger edges — the fire
    ///   rate of the fastest chain it gates (a sink gates nothing ⇒ rate 0).
    ///   A level's gating rate is the max gating rate over its current
    ///   occupants. We REFUSE to move node `n` into a level `d` whose gating
    ///   rate EXCEEDS `n`'s own gating rate: doing so would drop `n`'s cost onto
    ///   the critical path of a chain FASTER than any chain `n` itself gates.
    ///   This is what keeps high-rate chains cheap while low-rate expensive
    ///   nodes drift late.
    ///
    /// **Why node-gating-rate, not origin-vs-destination.** The guard could be
    /// stated as "a level whose rate-criticality exceeds the origin's". We
    /// compare the destination's gating rate to the MOVED NODE's own gating
    /// rate instead, because that is the precise harm: relocating `n` to `d`
    /// keeps `n` on its own consumers' path either way (it stays strictly below
    /// them), so the only NEW burden is on the OTHER chains co-located at `d`.
    /// Guarding those chains by `n`'s rate protects exactly the faster
    /// co-located chains and no more.
    ///
    /// **Cost's role.** Node cost decides (a) whether a node is eligible at all
    /// (an uncosted node is PINNED at ASAP — conservative no-op) and (b) the
    /// processing priority (expensive nodes pick their destination first, so on
    /// a shared origin level the biggest cost claims relief first). The shift
    /// magnitude is NOT cost-scaled: every eligible slack-bearing node biases
    /// late, because moving even a cheap node off an early level only ever
    /// REDUCES that early level's cost, and the guard already blocks any move
    /// that would burden a faster chain. A high-rate node acquires slack only
    /// when its own consumer is gated by a slower PARALLEL path, in which case
    /// delaying it is latency-neutral for its own chain (the consumer waits on
    /// the slow path regardless) while still relieving earlier levels — so
    /// "high-rate chains stay at ASAP" holds where it matters: the fast chain's
    /// GATING levels stay minimal.
    ///
    /// # Phase 2 — the chain-cascade pass (level count may GROW)
    ///
    /// Phase 1 is structurally a no-op on a DENSE pipeline (every consumer
    /// exactly one level below its producer — the real humanoid perception
    /// chain has zero slack anywhere), which is exactly the flagship shape.
    /// Phase 2 shifts an eligible root +1 and CASCADES the shift to every
    /// downstream trigger-consumer that would stop being strictly below its
    /// producer (transitively), appending levels as needed — a whole dense
    /// chain slides later uniformly and vacates the fast chain's gating
    /// level.
    ///
    /// **The level-span model** (why the conditions are what they are):
    /// under global level-lockstep, a chain's e2e latency is the SUM of
    /// makespans of all levels STRICTLY BEFORE its sink level. Moving a node
    /// from level `a` to `b > a` leaves every chain with sink `> b`
    /// UNCHANGED (the cost sits inside its span either way — a 1 kHz spine
    /// spanning every level pays the camera wherever it sits), strictly
    /// HELPS every chain whose sink lies in `(a, b]` (the cost leaves its
    /// span), and hurts only the moved chain's own tail latency (µs-scale
    /// waits against a 30 Hz period). A destination-OCCUPANT rate guard is
    /// therefore provably unnecessary — and actively wrong (it refused
    /// every cascade on the real humanoid, whose 1 kHz spine occupies every
    /// level) — so phase 2 carries none.
    ///
    /// A root is eligible only when (1) it is costed, (2) RELIEF: a SINK of
    /// a strictly faster chain (CHAIN RATE = a NON-SINK's own gating rate, or
    /// a SINK's max direct incoming trigger-edge rate — a sink's gating rate
    /// is 0, so its incoming edge is the sole carrier of its chain's rate,
    /// which is what protects a fast chain's sink; a non-sink that merely
    /// INGESTS a fast stream but fires slow keeps its own slow gating rate,
    /// so it never masquerades as fast and never blocks a legitimate cascade
    /// through it) sits at a level ABOVE the root and OUTSIDE the
    /// root's downstream cone (a downstream sink is dragged by the root's
    /// own cascade and can never be relieved — chasing it would never
    /// terminate), and (3) MATERIALITY: the root's cost STRICTLY dominates
    /// its origin level's remaining cost (parallel-fire makespan: removing
    /// a non-max node relieves nothing). The cascade is refused atomically
    /// if it would RAISE any node of a strictly faster chain
    /// (NO-FASTER-DRAG — the model's "hurts only the moved chain" premise
    /// enforced; a refused march can strand a root mid-span, which is
    /// latency-neutral for every chain and accepted). An eligible root
    /// MARCHES +1 per pass and halts AT the deepest faster non-downstream
    /// sink's level (a sink never waits for its own level's mates, so
    /// co-location is full relief). Passes repeat until a moveless pass or
    /// the hard node-count bound; moves are monotone (only ever later), so
    /// termination follows by induction on the rate hierarchy (the fastest
    /// tier never moves; each tier's march is bounded by stabilized
    /// faster-tier sink levels). See the in-body comment for the full
    /// derivation.
    ///
    /// Growing the count is safe for CONSISTENCY across processes: the
    /// assignment is FROZEN into the graph yaml (`level_assignments:`) and
    /// every process derives its barrier generations from the same block, so
    /// cross-process consistency holds at ANY count by construction. But
    /// growth is NOT free for multi-process CADENCE: the level-lockstep
    /// barrier advances ONE generation per level, so +1 level = +1
    /// cross-process rendezvous per step (measured ~19% chain-cadence cost at
    /// neutral p50; growth still HELPS the monolith, ~−10% p50). Growth is
    /// therefore POLICY-GATED via [`RefineInputs::level_growth`]:
    /// [`LevelGrowth::Allow`] (the default, and the earlier behavior) keeps
    /// growth; [`LevelGrowth::Deny`] refuses any cascade that would append a
    /// level while still applying non-growing cascades. `Deny` is set by the
    /// shape-gated `cerulion graph partition` wiring for a
    /// multi-group-destined graph — the emitted-shape gate itself lives in the
    /// partition verb, NOT here (this fn only honors the policy it is handed).
    ///
    /// # Pins (nodes that never move)
    ///
    /// * **Uncosted** (no `node_cost_ns` entry) — PINNED. A completely
    ///   absent cost input (`node_cost_ns` empty) returns `base` UNCHANGED,
    ///   byte-identical (the no-profile-artifact contract: no cost ⇒ today's
    ///   Kahn output exactly).
    /// * **The max-rate chain's nodes.** A max-rate node has no strictly
    ///   faster sink anywhere, so it is never a cascade root; and
    ///   NO-FASTER-DRAG refuses any slower root's cascade that would raise
    ///   it — so the protected (fastest) chain never moves, sink included.
    ///   (Phase 1 additionally pins ALL sinks at ASAP within its
    ///   fixed-count world; phase 2 deliberately relaxes that — a cascaded
    ///   chain's sink moves WITH its chain, which is the point: the
    ///   perception chain's tail is not the protected chain.)
    /// * **Zero relief / immaterial** — no strictly-faster non-downstream
    ///   sink above the node (it already sits at/past every faster chain's
    ///   waiting span), or the node does not strictly dominate its level's
    ///   remaining cost.
    ///
    /// # Correctness (invariants, enforced by construction)
    ///
    /// * **Every trigger edge stays strictly level-increasing** (producer level
    ///   `<` consumer level). Phase 1: reverse-topological placement (`n` is
    ///   placed `≤ min(final(c)) − 1 < final(c)`). Phase 2: the cascade
    ///   definition — any consumer that would violate is raised with the
    ///   chain. `debug_assert`ed via the shared validator.
    /// * **Same node set, partitioned exactly once** — reconstruction places
    ///   every graph node at its assigned level, in graph order within the
    ///   level. `debug_assert`ed against the input node count.
    /// * **Contiguous, no empty level.** The level count may GROW (cascade)
    ///   under [`LevelGrowth::Allow`] but the final assignment is COMPRESSED
    ///   to a contiguous `0..K` range with every level occupied (the barrier
    ///   advances one generation per level, so empties are structurally
    ///   excluded), and `K` never exceeds the node count. With no applied
    ///   cascade the count is preserved exactly (phase 1 alone cannot move a
    ///   critical-path node: by induction from its sink each has zero slack).
    ///   Under [`LevelGrowth::Deny`] the count can NEVER grow (every
    ///   growing cascade is refused, so `K <= base.len()`) while non-growing
    ///   cascades still apply (a chain may still slide within the existing
    ///   count).
    /// * **Deterministic.** Phase-1 order `(ASAP desc, cost desc, graph asc)`
    ///   and phase-2 order `(cost desc, graph asc)` are total orders; the
    ///   cost/rate maps are only ever POINT-queried (never iterated for
    ///   ordering), so the output is byte-reproducible (Principle #7).
    ///
    /// # The cost input
    ///
    /// [`RefineInputs`] is a minimal projection: `node_cost_ns` +
    /// `edge_rate_mhz` are exactly what the objective reads. The profiler's
    /// core cost snapshot [`PartitionCosts`](super::partition::PartitionCosts)
    /// is a superset (`node_p50_ns` → `node_cost_ns`, `edge_rate_mhz` →
    /// `edge_rate_mhz`, dropping the partition-only `hop` field), so the
    /// CLI wiring adapts it with a two-field copy — no new parsing. Keeping the
    /// input MINIMAL keeps `topology` free of any dependency on `partition`
    /// (the dependency runs one way today; the refinement does not reverse it).
    ///
    /// Returns a fresh [`Levels`]; every downstream consumer (`runtime`,
    /// `partition`) reads `Levels` unchanged.
    pub fn refine_levels(
        &self,
        base: &Levels,
        trigger_edges: &TriggerEdges,
        inputs: &RefineInputs,
    ) -> Levels {
        // No-cost fallback contract: an absent cost input is a GUARANTEED
        // no-op — return the Kahn levelization byte-identical. A graph with no
        // profiler artifact keeps today's output exactly (Principle #7).
        let level_count = base.len();
        if inputs.node_cost_ns.is_empty() || level_count == 0 {
            return base.clone();
        }

        // ASAP = the Kahn level per node (authoritative from `base.rank`).
        // The node set + graph order come from `node_order` (the same source
        // `derive_levels` seeds from), so an id present here is present there.
        let asap = |n: &str| -> usize { base.level_of(n).unwrap_or(0) };

        // Build the triggering-edge SUCCESSOR adjacency (producer → consumers),
        // classifying edges EXACTLY as `derive_levels` does: only edges the
        // runtime marked triggering are DAG edges; a latest-value read is not.
        // Duplicates are tolerated (used only for min/max), so no dedup pass.
        let mut succ: IndexMap<&str, Vec<&str>> = IndexMap::new();
        for id in &self.node_order {
            succ.insert(id.as_str(), Vec::new());
        }
        for flow in self.topics.values() {
            for c in &flow.consumers {
                if !trigger_edges.is_triggering(&c.node_id, &flow.topic) {
                    continue;
                }
                for producer in &flow.producers {
                    if let Some(v) = succ.get_mut(producer.as_str()) {
                        v.push(c.node_id.as_str());
                    }
                }
            }
        }

        // Per-node gating rate = max `edge_rate_mhz` over the node's OUTGOING
        // trigger edges (the fastest chain it gates). A sink → 0. Precomputed
        // once (point lookups only ⇒ order-independent, deterministic).
        let mut node_gating_rate: IndexMap<&str, u64> = IndexMap::new();
        for id in &self.node_order {
            let n = id.as_str();
            let rate = succ[n]
                .iter()
                .map(|c| {
                    inputs
                        .edge_rate_mhz
                        .get(&(n.to_string(), (*c).to_string()))
                        .copied()
                        .unwrap_or(0)
                })
                .max()
                .unwrap_or(0);
            node_gating_rate.insert(n, rate);
        }

        // Processing order: REVERSE-topological (ASAP descending) so every
        // successor is placed before its predecessors — the invariant-by-
        // construction guarantee. Ties broken by (cost desc, graph-order asc):
        // on a shared origin level the most expensive node picks its
        // destination first. `graph-order asc` (the `node_order` index) is the
        // final deterministic tiebreak.
        let mut order: Vec<&str> = self.node_order.iter().map(|s| s.as_str()).collect();
        let index_of = |n: &str| -> usize {
            self.node_order
                .iter()
                .position(|x| x == n)
                .unwrap_or(usize::MAX)
        };
        order.sort_by(|a, b| {
            asap(b)
                .cmp(&asap(a))
                .then_with(|| {
                    let ca = inputs.node_cost_ns.get(*a).copied().unwrap_or(0);
                    let cb = inputs.node_cost_ns.get(*b).copied().unwrap_or(0);
                    cb.cmp(&ca)
                })
                .then_with(|| index_of(a).cmp(&index_of(b)))
        });

        // Assign final levels. `assigned[n]` is `n`'s final level; `level_gate`
        // tracks each level's running max gating rate (in processing order —
        // a deterministic occupancy approximation: the pinned critical node of
        // any level has ASAP == that level and is placed before any shallower
        // mover considers it, so a destination's protected fast chain is always
        // visible to the mover).
        let mut assigned: IndexMap<&str, usize> = IndexMap::new();
        let mut level_gate: Vec<u64> = vec![0; level_count];

        for &n in &order {
            let a = asap(n);
            let ng = node_gating_rate[n];

            // Pin: uncosted, or a sink (no successors). Both stay at ASAP.
            let costed = inputs.node_cost_ns.contains_key(n);
            let successors = &succ[n];
            let dest = if !costed || successors.is_empty() {
                a
            } else {
                // ALAP against successors' FINAL levels (all already placed).
                // `min(final(c)) ≥ ASAP(c) ≥ a + 1`, so `alap ≥ a` always.
                let alap = successors
                    .iter()
                    .map(|c| assigned[c])
                    .min()
                    .expect("non-empty successors")
                    - 1;
                if alap <= a {
                    a // zero slack
                } else {
                    // Bias late: latest level in `(a, alap]` the guard allows.
                    // Guard: do not enter a level gating a faster chain than n.
                    let mut chosen = a;
                    let mut d = alap;
                    while d > a {
                        if level_gate[d] <= ng {
                            chosen = d;
                            break;
                        }
                        d -= 1;
                    }
                    chosen
                }
            };

            assigned.insert(n, dest);
            level_gate[dest] = level_gate[dest].max(ng);
        }

        // ── PHASE 2: the CHAIN-CASCADE pass. ──
        //
        // The within-slack phase above is structurally a no-op on a DENSE
        // pipeline (every consumer exactly one level below its producer — the
        // real humanoid perception chain), which is exactly the flagship
        // shape. This pass may GROW the level count: an eligible root shifts
        // +1 and the shift CASCADES to every downstream trigger-consumer that
        // would violate strict increase (transitively), appending levels as
        // needed, so a whole dense chain slides later uniformly and vacates
        // the fast chain's gating level. Cross-process consistency at ANY
        // count holds by construction: the assignment is FROZEN in the yaml
        // and every process derives its barrier generations from the same
        // block.
        //
        // A node's CHAIN RATE is its observed fire cadence, which the
        // profiler measures at the OUTGOING edge:
        //   * a NON-SINK's chain rate is its own GATING rate (max rate over
        //     its outgoing trigger edges) — a Sync join / decimating consumer
        //     that ingests a fast stream but fires slow keeps its SLOW rate,
        //     so it never masquerades as fast;
        //   * a SINK's chain rate is its max DIRECT incoming trigger-edge
        //     rate — its gating rate is 0, so the incoming edge is the sole
        //     carrier of its chain's rate. This is what protects a fast
        //     chain's SINK (relative protection replacing phase 1's absolute
        //     sink pin: a cascaded chain's sink MOVES WITH its chain; the pin
        //     that survives is that the max-rate chain's nodes never move).
        // The sink/non-sink split is load-bearing: inheriting the fast
        // incoming rate onto a slow non-sink (a plain `max`) made
        // NO-FASTER-DRAG refuse a legitimate cascade THROUGH such a join —
        // the head_depth defect, where head_depth (10 Hz) could not
        // vacate L0 because its cascade dragged elevation_mapping, a 100 ms
        // Sync join fed by a 611 Hz fbe edge; dragging that join delays only
        // its own 10 Hz chain, never the 611 Hz producer's (whose fast sinks
        // branch off at the producer, not below the join).
        //
        // Eligibility of a cascade ROOT `n` (evaluated on the CURRENT state,
        // sequentially, in the fixed total order (cost desc, graph asc)):
        //   1. `n` is costed (uncosted stays pinned — conservative);
        //   2. RELIEF: a strictly faster chain still WAITS through `n`'s
        //      level — see the level-span model below;
        //   3. MATERIALITY: `cost(n) > max(cost(remaining origin mates))` —
        //      under the within-level parallel-fire model a level's makespan
        //      is its max node cost, so removing `n` shrinks the gating
        //      level's makespan only when `n` strictly dominates it. Exact
        //      ties deliberately do NOT move (zero relief would still grow a
        //      level = more barrier generations); the strict-max requirement
        //      also makes co-located expensive roots cascade one at a time,
        //      largest first (each move unlocks the next within the pass).
        //   4. NO-FASTER-DRAG: the cascade must never RAISE a node whose
        //      chain rate is strictly greater than the root's — see below.
        //
        // THE LEVEL-SPAN MODEL (why relief is sink-based, and why a
        // destination-occupant rate guard would be WRONG): under global
        // level-lockstep, a chain C's e2e latency = the SUM of makespans of
        // all levels STRICTLY BEFORE C's sink level (the levels C waits
        // through). Moving a node from level `a` to `b > a` therefore
        // (1) leaves every chain whose sink is > b UNCHANGED — both levels
        // lie inside its span, the cost is paid wherever it sits (a 1 kHz
        // spine spanning every level pays the camera's cost at ANY level, so
        // refusing the move because a spine node occupies the destination —
        // a destination-occupant guard — would block provably-neutral steps and
        // make the real humanoid a no-op); (2) strictly HELPS every chain
        // whose sink lies in (a, b] — the cost leaves its span; (3) hurts
        // only the moved chain's own tail latency (+1 level-makespan per
        // level of delay — µs-scale waits against a 30 Hz/33 ms period).
        //
        // RELIEF, precisely: root `n` at level `a` is eligible iff some SINK
        // `m` (no trigger successors) satisfies `chain_rate(m) >
        // chain_rate(n)` AND `level(m) > a` AND `m` is NOT in `n`'s
        // downstream cone. The per-step condition makes the node MARCH +1
        // per pass until it sits at the deepest faster sink's level (AT, not
        // above: a sink never waits for its own level's mates, so
        // co-location is already full relief and a further level would be
        // wasted). The DOWNSTREAM EXCLUSION is required for the model to be
        // true: a faster sink downstream of `n` is DRAGGED by `n`'s own
        // cascade, staying forever above it — it can never be relieved, and
        // chasing it would march to the pass cap. NO-FASTER-DRAG is the
        // model's own premise enforced: a cascade that raised a
        // strictly-faster chain's node would deepen THAT chain's span —
        // falsifying "hurts only the moved chain" — so such a cascade is
        // refused atomically (a slow node feeding a fast-rated join stops
        // marching just below the point where it would push the join). A
        // refused march can strand a root mid-span; that position is
        // provably latency-NEUTRAL for every chain (within-span placement
        // does not change any span sum), so it is accepted and documented
        // rather than pre-checked away.
        //
        // TERMINATION (provable, by induction on the rate hierarchy): every
        // applied move strictly increases ≥1 node's level and none ever
        // decreases (monotone, only-ever-later — no oscillation). Nodes of
        // the max rate tier have no strictly-faster sink ⇒ never roots;
        // they are never dragged either (NO-FASTER-DRAG refuses slower
        // roots) ⇒ their levels are FIXED. Inductively, a tier-k root's
        // march is bounded by the deepest faster (tier <k) NON-DOWNSTREAM
        // sink level, which stabilizes after finitely many faster-tier
        // moves; each step is +1, so every root's total movement is finite
        // WITHOUT the cap. The `total`-pass hard cap (early exit on a
        // moveless pass) remains as belt-and-suspenders. Deterministic
        // throughout: fixed candidate order, worklist in adjacency insertion
        // order, sequential state mutation, integer math.
        let total = self.node_order.len();
        // The node's chain rate:
        //   * NON-SINK (has trigger successors): its own GATING rate — the
        //     rate of the fastest chain it gates, which IS the node's own
        //     observed fire cadence. A decimating / Sync-join consumer that
        //     INGESTS a fast stream but FIRES slow must NOT inherit that fast
        //     incoming rate: delaying such a node delays only its own (slow)
        //     downstream chain, never the fast producer's chain (the fast
        //     chain's fast SINKS are not downstream of it — they branch off at
        //     the producer). Inheriting the incoming rate would make
        //     NO-FASTER-DRAG refuse a legitimate cascade THROUGH the join (the
        //     head_depth defect: head_depth (10 Hz) could not vacate
        //     L0 because its cascade dragged elevation_mapping, a 100 ms Sync
        //     join whose incoming fbe edge fires at 611 Hz — the join's *own*
        //     gating rate is 10 Hz, and dragging it delays nothing 611 Hz).
        //   * SINK (no trigger successors): max DIRECT incoming
        //     triggering-edge rate — a sink's gating rate is 0, so its
        //     incoming edge is the ONLY carrier of its chain's rate. This is
        //     what protects a fast chain's SINK (relative protection replacing
        //     phase 1's absolute sink pin); a Sync SINK is conservatively
        //     over-protected (treated as fast even if it fires slow), which is
        //     the safe direction. Transitive upstream rates are deliberately
        //     not propagated — the direct edge is what the profiler measured
        //     for this node's arrivals.
        // Edge case: a non-sink with unmeasured (rate-0) outgoing edges has
        // chain rate 0 — freely draggable AND never relieving. That is
        // acceptable: RELIEF is sink-only (below), so it relieves nothing
        // either way, and a node the profiler recorded no outflow for showed
        // no flow to delay.
        let mut chain_rate: IndexMap<&str, u64> = node_gating_rate.clone();
        for flow in self.topics.values() {
            for c in &flow.consumers {
                if !trigger_edges.is_triggering(&c.node_id, &flow.topic) {
                    continue;
                }
                // Incoming-edge inheritance is SINK-ONLY: a non-sink's chain
                // rate is its own gating rate (its measured fire cadence).
                if !succ[c.node_id.as_str()].is_empty() {
                    continue;
                }
                for producer in &flow.producers {
                    let r = inputs
                        .edge_rate_mhz
                        .get(&(producer.clone(), c.node_id.clone()))
                        .copied()
                        .unwrap_or(0);
                    if let Some(v) = chain_rate.get_mut(c.node_id.as_str()) {
                        *v = (*v).max(r);
                    }
                }
            }
        }
        // Fixed total candidate order: (cost desc, graph asc) — the phase-1
        // family without the reverse-topological key (cascades push
        // downstream themselves).
        let mut cascade_order: Vec<&str> = self.node_order.iter().map(|s| s.as_str()).collect();
        cascade_order.sort_by(|a, b| {
            let ca = inputs.node_cost_ns.get(*a).copied().unwrap_or(0);
            let cb = inputs.node_cost_ns.get(*b).copied().unwrap_or(0);
            cb.cmp(&ca).then_with(|| index_of(a).cmp(&index_of(b)))
        });
        for _pass in 0..total {
            let mut moved_any = false;
            for &root in &cascade_order {
                if !inputs.node_cost_ns.contains_key(root) {
                    continue;
                }
                let origin = assigned[root];
                let root_rate = chain_rate[root];
                // RELIEF (the level-span model): ∃ a strictly faster chain
                // still WAITING through this level — a SINK with chain rate
                // > the root's, at a level above the root, outside the
                // root's downstream cone (a downstream sink is dragged by
                // the root's own cascade and can never be relieved).
                let mut cone: HashSet<&str> = HashSet::new();
                cone.insert(root);
                let mut stack: Vec<&str> = vec![root];
                while let Some(m) = stack.pop() {
                    for &c in &succ[m] {
                        if cone.insert(c) {
                            stack.push(c);
                        }
                    }
                }
                let relieved = assigned.iter().any(|(m, &l)| {
                    l > origin
                        && succ[*m].is_empty()
                        && chain_rate[*m] > root_rate
                        && !cone.contains(*m)
                });
                if !relieved {
                    continue;
                }
                // MATERIALITY: root strictly dominates the origin level's
                // remaining cost (cascade members all sit strictly deeper, so
                // "remaining mates" = all origin mates).
                let max_mate_cost = assigned
                    .iter()
                    .filter(|(m, &l)| l == origin && **m != root)
                    .map(|(m, _)| inputs.node_cost_ns.get(*m).copied().unwrap_or(0))
                    .max()
                    .unwrap_or(0);
                if inputs.node_cost_ns[root] <= max_mate_cost {
                    continue;
                }
                // CASCADE SET: root +1, then transitively raise every
                // downstream trigger-consumer that would stop being strictly
                // below its producer. Monotone raises ⇒ finite worklist.
                let mut moved: IndexMap<&str, usize> = IndexMap::new();
                moved.insert(root, origin + 1);
                let mut queue: std::collections::VecDeque<&str> = std::collections::VecDeque::new();
                queue.push_back(root);
                while let Some(m) = queue.pop_front() {
                    let m_lvl = moved[m];
                    for &c in &succ[m] {
                        let cur = moved.get(c).copied().unwrap_or(assigned[c]);
                        if cur <= m_lvl {
                            moved.insert(c, m_lvl + 1);
                            queue.push_back(c);
                        }
                    }
                }
                // NO-FASTER-DRAG: the model's premise ("a move hurts only
                // the moved chain") enforced — refuse atomically if the
                // cascade would raise any node of a strictly faster chain
                // (e.g. a fast-rated join fed by this slow root).
                let refused = moved.keys().any(|m| chain_rate[*m] > root_rate);
                if refused {
                    continue;
                }
                // GROWTH GATE: under `Deny`, refuse ATOMICALLY any
                // cascade whose target-level set would reach BEYOND the Kahn
                // input's max level (`level_count - 1`, `level_count =
                // base.len()`) — i.e. would append a level. A cascade whose
                // targets all land within the original `0..level_count` range
                // is NON-growing and still applies (the gate kills GROWTH, not
                // cascading: a chain may still slide later, and the expensive
                // sinks still vacate a fast chain's span, as long as no level
                // is appended). Refusing growth keeps the multi-process
                // barrier at the same generation count per step (+1 level = +1
                // cross-process rendezvous, the measured ~19% chain-cadence
                // cost). Under `Allow` the gate is inert (pre-gate behavior).
                // Pure point-predicate on `moved` (no iteration-order
                // dependence): phase 1 never grows the count and every applied
                // cascade under `Deny` keeps `max(assigned) <= level_count-1`,
                // so the final count can never exceed `level_count` in `Deny`.
                if inputs.level_growth == LevelGrowth::Deny
                    && moved.values().any(|&lvl| lvl >= level_count)
                {
                    continue;
                }
                for (m, &new_lvl) in &moved {
                    assigned.insert(*m, new_lvl);
                }
                moved_any = true;
            }
            if !moved_any {
                break;
            }
        }

        // COMPRESS to a contiguous 0..K range (a cascade can vacate a level
        // when a whole chain segment occupied it alone; the barrier's
        // generation math requires no-empty-level contiguity).
        let mut used: Vec<usize> = assigned.values().copied().collect();
        used.sort_unstable();
        used.dedup();
        let remap: std::collections::HashMap<usize, usize> =
            used.iter().enumerate().map(|(i, l)| (*l, i)).collect();
        for v in assigned.values_mut() {
            *v = remap[v];
        }
        let final_level_count = used.len();

        // Reconstruct: place every node at its assigned level, in GRAPH order
        // within the level (the `node_order` scan is graph-ordered), mirroring
        // `derive_levels`' within-level ordering (Principle #5/#7).
        let mut out_levels: Vec<Level> = (0..final_level_count)
            .map(|_| Level { nodes: Vec::new() })
            .collect();
        let mut rank: IndexMap<String, usize> = IndexMap::new();
        for id in &self.node_order {
            if let Some(&lvl) = assigned.get(id.as_str()) {
                out_levels[lvl].nodes.push(id.clone());
                rank.insert(id.clone(), lvl);
            }
        }

        // Invariants (by construction — belt-and-suspenders in debug).
        // The partition-count check stays local (it is refine-specific: the
        // node set comes from `assigned`); the strictly-increasing-edge +
        // no-empty-level walks are the SHARED hard contract, extracted into
        // `level_invariant_violation` so this debug path
        // and the yaml `level_assignments` hard-Err path cannot diverge.
        debug_assert_eq!(
            out_levels.iter().map(|l| l.nodes.len()).sum::<usize>(),
            assigned.len(),
            "refine_levels must partition every node exactly once"
        );
        #[cfg(debug_assertions)]
        if let Some(violation) = self.level_invariant_violation(&out_levels, &rank, trigger_edges) {
            panic!("refine_levels broke a level invariant (internal bug): {violation}");
        }

        Levels {
            levels: out_levels,
            rank,
        }
    }

    /// The SHARED hard-invariant walk over a level
    /// assignment — the two properties every levelization consumed by the
    /// executor/barrier MUST satisfy, regardless of where it came from
    /// (Kahn guarantees them structurally; [`Self::refine_levels`] preserves
    /// them by construction and `debug_assert`s through this fn; the yaml
    /// `level_assignments:` path turns a violation into a hard
    /// [`TransportError`] — hand-edited files are untrusted input):
    ///
    /// 1. **No empty level** (contiguity): the levels vector has no empty
    ///    entry — the multi-process barrier advances ONE generation per
    ///    level, so an empty level would desync the per-step generation
    ///    math across processes.
    /// 2. **Every trigger edge strictly level-increasing**: for every
    ///    TRIGGERING consumer edge, `level(producer) < level(consumer)` —
    ///    else the consumer would fire in the same step-phase as (or before)
    ///    its producer, breaking the level executor's drain-between-levels
    ///    dataflow contract.
    ///
    /// Returns the FIRST violation as a factual, human-readable string (the
    /// caller wraps it with graph context + remedy), or `None` when the
    /// assignment is sound. Deterministic: levels are scanned in order and
    /// edges in topic/consumer/producer insertion (graph) order.
    pub(crate) fn level_invariant_violation(
        &self,
        levels: &[Level],
        rank: &IndexMap<String, usize>,
        trigger_edges: &TriggerEdges,
    ) -> Option<String> {
        // 1. No empty level.
        for (idx, level) in levels.iter().enumerate() {
            if level.nodes.is_empty() {
                return Some(format!(
                    "level {idx} is EMPTY — assigned levels must form a contiguous \
                     0..={} range with no gaps (the multi-process barrier advances \
                     one generation per level, so an empty level would desync the \
                     generation math); renumber the levels to close the gap",
                    levels.len().saturating_sub(1)
                ));
            }
        }
        // 2. Every trigger edge strictly level-increasing. Same edge
        //    classification as `derive_levels`: only TRIGGERING consumer
        //    edges are DAG edges; a latest-value read constrains nothing.
        for flow in self.topics.values() {
            for c in &flow.consumers {
                if !trigger_edges.is_triggering(&c.node_id, &flow.topic) {
                    continue;
                }
                for producer in &flow.producers {
                    let (Some(&lp), Some(&lc)) = (rank.get(producer), rank.get(&c.node_id)) else {
                        // Callers guarantee coverage (Kahn/refine place every
                        // node; the yaml path validates coverage first), so a
                        // missing rank here is itself a violation — report it
                        // rather than silently skipping the edge.
                        return Some(format!(
                            "trigger edge '{}' -> '{}' references a node with no \
                             assigned level",
                            producer, c.node_id
                        ));
                    };
                    if lp >= lc {
                        return Some(format!(
                            "trigger edge '{}' (level {lp}) -> '{}' (level {lc}) is not \
                             strictly level-increasing — the consumer would fire in the \
                             same step-phase as (or before) its producer",
                            producer, c.node_id
                        ));
                    }
                }
            }
        }
        None
    }

    /// Build [`Levels`] from the graph yaml's
    /// `level_assignments:` block (node id → level index) — the baked output
    /// of the cost-aware refinement, or a hand-written override.
    ///
    /// Hand-edited files are UNTRUSTED input, so everything
    /// [`Self::refine_levels`] guarantees by construction is re-checked here
    /// as a hard [`TransportError`]:
    ///
    /// * every key names an existing graph node (typos / stale blocks fail
    ///   loudly, naming the offenders);
    /// * EVERY graph node is covered — full coverage REQUIRED. A partial map
    ///   is ambiguous about intent (the emit always writes all nodes; a
    ///   hand-deleted line should fail loudly, not silently re-derive), so
    ///   missing nodes are named rather than Kahn-defaulted;
    /// * every trigger edge is strictly level-increasing and the assigned
    ///   levels form a contiguous `0..K` range with no empty level (shared
    ///   walk: `Self::level_invariant_violation`, the same `pub(crate)`
    ///   validator [`Self::refine_levels`] debug-asserts through).
    ///
    /// **Sink/pin semantics deliberately do NOT apply.** `refine_levels`'
    /// sinks-stay-ASAP / uncosted-stay-put rules are OBJECTIVE choices of the
    /// automatic refinement, not part of the level contract — a user may
    /// deliberately delay a sink. The invariants above are the only hard
    /// contract.
    ///
    /// Within-level order is graph (`nodes:`) declaration order — NEVER the
    /// map's key order (Principle #5/#7; identical to the Kahn/refine paths).
    ///
    /// `graph_name` contextualizes every error (the block lives in that
    /// graph's yaml file).
    pub fn levels_from_assignments(
        &self,
        assignments: &IndexMap<String, usize>,
        trigger_edges: &TriggerEdges,
        graph_name: &str,
    ) -> TransportResult<Levels> {
        // (a) + (b): unknown keys / full coverage — the CONFIG-ONLY half,
        // shared with `validate_graph`'s early load-time gate (one text, two
        // boundaries — defense-in-depth per the `validate_process_groups`
        // precedent).
        let ids: Vec<&str> = self.node_order.iter().map(|s| s.as_str()).collect();
        if let Some(violation) = level_assignment_coverage_violation(&ids, assignments) {
            return Err(TransportError::GraphError {
                reason: format!("graph '{graph_name}': {violation}"),
            });
        }

        // (c) Level-range guard BEFORE allocating the levels vector: N nodes
        //     can occupy at most levels 0..=N-1 under the contiguity contract,
        //     so any larger index is both provably invalid AND a potential
        //     huge allocation (`a: 999999999`). First offender in graph order.
        let node_count = self.node_order.len();
        for id in &self.node_order {
            let lvl = assignments[id.as_str()];
            if lvl >= node_count {
                return Err(TransportError::GraphError {
                    reason: format!(
                        "graph '{graph_name}': `level_assignments:` assigns node '{id}' \
                         level {lvl}, but {node_count} node(s) can occupy at most levels \
                         0..={} — levels must form a contiguous 0..K range",
                        node_count.saturating_sub(1)
                    ),
                });
            }
        }

        // Construct: place every node at its assigned level, in GRAPH order
        // within the level (the node_order scan IS graph order — never the
        // assignment map's key order).
        let level_count = self
            .node_order
            .iter()
            .map(|id| assignments[id.as_str()] + 1)
            .max()
            .unwrap_or(0);
        let mut levels: Vec<Level> = (0..level_count)
            .map(|_| Level { nodes: Vec::new() })
            .collect();
        let mut rank: IndexMap<String, usize> = IndexMap::new();
        for id in &self.node_order {
            let lvl = assignments[id.as_str()];
            levels[lvl].nodes.push(id.clone());
            rank.insert(id.clone(), lvl);
        }

        // (d) The shared hard contract: contiguous/no-empty-level +
        //     strictly-increasing trigger edges.
        if let Some(violation) = self.level_invariant_violation(&levels, &rank, trigger_edges) {
            return Err(TransportError::GraphError {
                reason: format!("graph '{graph_name}': invalid `level_assignments:` — {violation}"),
            });
        }

        Ok(Levels { levels, rank })
    }

    /// Recover a concrete algebraic-loop ring from the
    /// Kahn survivor set, for the [`CycleError`] message.
    ///
    /// Runs ONLY on the error path. A DFS over the survivors' OWN triggering
    /// edges (the `adj` restricted to survivor→survivor) finds a back-edge
    /// to a node already on the DFS stack; the stack slice from that node to
    /// the current node IS the ring.
    ///
    /// **Why multi-root (not a single anchor):** a survivor is a node Kahn
    /// could not peel (residual in-degree > 0), but that does NOT mean it
    /// lies ON a ring — a survivor can be a pure SINK downstream of a cycle
    /// (it is fed by a cycle node, so its in-degree never drops to 0, yet it
    /// has no outgoing survivor edge). Rooting the DFS only at the
    /// lowest-graph-order survivor would, for such a sink, exhaust with no
    /// successors and name a node not on any ring. Instead we try EVERY
    /// survivor as a DFS root in GRAPH ORDER, sharing one global `explored`
    /// (fully-processed) set across roots. The FIRST back-edge to a node on
    /// the current stack yields the ring. The survivor set is guaranteed to
    /// contain ≥ 1 cycle (Kahn left it), so a ring is always found; a node
    /// whose successors are all exhausted joins `explored` and is never
    /// re-rooted, so the walk is O(V + E) over survivors.
    ///
    /// # Determinism
    ///
    /// Roots are tried in graph order (`survivors` is graph-ordered), and
    /// each node's successors are visited in deterministic adjacency order.
    /// The returned `cycle` is the ring nodes in traversal order followed by
    /// the entry again, so a reader sees the loop close (e.g. `["a", "b",
    /// "a"]` or the self-loop `["a", "a"]`).
    fn extract_cycle<'a>(
        &self,
        survivors: &IndexSet<&'a str>,
        adj: &IndexMap<&'a str, Vec<&'a str>>,
    ) -> CycleError {
        // Pre-filter each node's successors to survivors only, in
        // deterministic adjacency order, deduped (a producer may push the
        // same consumer once per producer of a multi-pub topic).
        let succ = |n: &str| -> Vec<&'a str> {
            let mut seen: IndexSet<&'a str> = IndexSet::new();
            if let Some(list) = adj.get(n) {
                for &c in list {
                    if survivors.contains(&c) {
                        seen.insert(c);
                    }
                }
            }
            seen.into_iter().collect()
        };

        // `explored` = nodes whose entire survivor-reachable subgraph has
        // been DFS-exhausted with no ring found through them. Shared across
        // roots: once a node joins it, no later root re-walks it.
        let mut explored: IndexSet<&str> = IndexSet::new();

        // Try every survivor as a DFS root, in graph order.
        for &root in survivors.iter() {
            if explored.contains(&root) {
                continue;
            }
            // Iterative DFS from this root over survivor→survivor edges,
            // tracking the path stack. The first back-edge to a node on the
            // stack closes a ring.
            let mut stack: Vec<&str> = vec![root];
            // `on_stack` mirrors `stack` membership for O(1) cycle detection.
            let mut on_stack: IndexSet<&str> = IndexSet::new();
            on_stack.insert(root);
            // Per-frame iterator position into the node's survivor successors.
            let mut next_idx: Vec<usize> = vec![0];

            while let Some(&cur) = stack.last() {
                let depth = stack.len() - 1;
                let successors = succ(cur);
                if next_idx[depth] < successors.len() {
                    let target = successors[next_idx[depth]];
                    next_idx[depth] += 1;
                    if on_stack.contains(&target) {
                        // Found the ring: stack slice from `target` to `cur`.
                        let start = stack
                            .iter()
                            .position(|n| *n == target)
                            .expect("target is on the stack");
                        let mut ring: Vec<String> =
                            stack[start..].iter().map(|s| s.to_string()).collect();
                        // Close the loop so the reader sees it return to the
                        // ring entry (self-loop → ["a", "a"]).
                        ring.push(target.to_string());
                        return CycleError { cycle: ring };
                    }
                    // A node already fully explored (no ring through it) is
                    // not re-pushed; an unvisited, not-on-stack survivor is.
                    if !explored.contains(&target) {
                        stack.push(target);
                        on_stack.insert(target);
                        next_idx.push(0);
                    }
                } else {
                    // Exhausted this frame: its whole subgraph held no ring
                    // reachable through it → mark explored, backtrack.
                    explored.insert(cur);
                    stack.pop();
                    next_idx.pop();
                    on_stack.shift_remove(cur);
                }
            }
        }

        // Genuinely unreachable: Kahn left a non-empty survivor set, every
        // survivor has residual in-degree > 0 (a survivor predecessor), so
        // the survivor subgraph contains at least one ring — trying every
        // survivor as a root (above) is guaranteed to traverse it and return.
        // If control reaches here the survivor adjacency is internally
        // inconsistent; fail loudly with the survivor list rather than
        // silently naming a non-ring node.
        unreachable!(
            "extract_cycle exhausted all {} survivor roots without finding a \
             ring, but Kahn proved a cycle exists; survivors = {:?}",
            survivors.len(),
            survivors.iter().copied().collect::<Vec<_>>()
        )
    }
}

/// The CONFIG-ONLY structural half of `level_assignments:`
/// validation — unknown keys + full coverage. Needs no topology/trigger
/// metadata, so it runs at TWO boundaries with one text (defense-in-depth,
/// the `validate_process_groups` precedent): `validate_graph` (early
/// load-time gate) and [`GraphTopology::levels_from_assignments`] (the
/// build-time enforcement point, which additionally checks the trigger-edge
/// + contiguity invariants).
///
/// `node_ids_in_graph_order` is the graph's node ids in `nodes:` declaration
/// order; missing nodes are reported in that order, unknown keys in map
/// (yaml-authored) order — both deterministic. Returns the FIRST violation
/// class as a factual string (the caller prefixes graph context), or `None`.
pub(crate) fn level_assignment_coverage_violation(
    node_ids_in_graph_order: &[&str],
    assignments: &IndexMap<String, usize>,
) -> Option<String> {
    let node_set: IndexSet<&str> = node_ids_in_graph_order.iter().copied().collect();
    let unknown: Vec<&str> = assignments
        .keys()
        .map(|k| k.as_str())
        .filter(|k| !node_set.contains(k))
        .collect();
    if !unknown.is_empty() {
        return Some(format!(
            "the `level_assignments:` block in the graph's yaml names unknown \
             node(s) [{}] — every key must be a node id declared under `nodes:` \
             (check for typos or a stale hand-edited block)",
            unknown.join(", ")
        ));
    }
    let missing: Vec<&str> = node_ids_in_graph_order
        .iter()
        .copied()
        .filter(|n| !assignments.contains_key(*n))
        .collect();
    if !missing.is_empty() {
        return Some(format!(
            "the `level_assignments:` block must cover EVERY node — missing [{}]. \
             A partial map is ambiguous; assign every node a level, or delete the \
             whole `level_assignments:` block to fall back to the derived \
             levelization",
            missing.join(", ")
        ));
    }
    None
}

/// THE single level-resolution seam — every consumer of a
/// graph-wide levelization (the runtime executor build AND the partitioner's
/// plan-time global levelization) MUST route through this function so the
/// yaml `level_assignments:` override cannot diverge between paths.
///
/// * `config.level_assignments` **present** → build [`Levels`] from the
///   assignment via [`GraphTopology::levels_from_assignments`] (full
///   validation: unknown/missing nodes, strictly-increasing trigger edges,
///   contiguous no-empty-level range — each a loud [`TransportError`]).
/// * **absent** (`None`) → today's Kahn levelization
///   ([`GraphTopology::derive_levels`]), byte-identical — the no-block
///   contract. An algebraic trigger cycle maps to the runtime's rich
///   diagnostic (one voice for runtime + partitioner).
pub fn resolve_levels(
    config: &GraphConfig,
    topology: &GraphTopology,
    trigger_edges: &TriggerEdges,
) -> TransportResult<Levels> {
    match &config.level_assignments {
        Some(assignments) => {
            topology.levels_from_assignments(assignments, trigger_edges, config.identity())
        }
        None => topology
            .derive_levels(trigger_edges)
            .map_err(|cycle| TransportError::GraphError {
                reason: format!(
                    "graph '{}' cannot be scheduled — {}. Every edge in this ring \
                     feeds an `#[input(trigger)]` of a Data/Sync-triggered node, so it \
                     would fire forever with no level order. Break the loop by making at \
                     least one read NON-triggering (drop the `trigger` on one \
                     `#[input]`, so it becomes a latest-value read), or remove an edge.",
                    config.identity(),
                    cycle
                ),
            }),
    }
}

/// The runtime-supplied classification of which
/// consumer edges are TRIGGERING (a `(consumer_node_id, topic)` pair).
///
/// Lives OUTSIDE [`GraphTopology`] because trigger-ness is a node-policy
/// fact (`#[input(trigger)]` + the node's `MacroPolicy`), not a YAML
/// topology fact. The runtime builds this from each node's `NodeInfo`
/// (see `runtime.rs`), keeping the topology itself policy-free. A
/// `(node, topic)` key (not `(node, input)`) is the natural join key:
/// [`derive_levels`](GraphTopology::derive_levels) walks `ConsumerEdge`s,
/// which carry both the node id and the topic via the owning `TopicFlow`.
///
/// A pair NOT in the set is a non-triggering (latest-value) read.
#[derive(Debug, Clone, Default)]
pub struct TriggerEdges {
    set: HashSet<(String, String)>,
}

impl TriggerEdges {
    /// Construct an empty set (no triggering edges — every node is a root).
    pub fn new() -> Self {
        Self::default()
    }

    /// Mark the edge `(consumer_node_id, topic)` as triggering.
    pub fn insert(&mut self, consumer_node_id: impl Into<String>, topic: impl Into<String>) {
        self.set.insert((consumer_node_id.into(), topic.into()));
    }

    /// True when the consumer edge `(consumer_node_id, topic)` is a
    /// triggering (DAG) edge.
    pub fn is_triggering(&self, consumer_node_id: &str, topic: &str) -> bool {
        // O(1) hashed lookup. A borrowed `(&str, &str)` tuple does not
        // compose for `HashSet<(String, String)>` lookup, so we pay two
        // build-time allocations to form the owned key — fine, this runs at
        // graph-build (once per consumer edge), not on the hot path.
        self.set
            .contains(&(consumer_node_id.to_string(), topic.to_string()))
    }

    /// Number of triggering edges recorded.
    pub fn len(&self) -> usize {
        self.set.len()
    }

    /// True when no triggering edges are recorded.
    pub fn is_empty(&self) -> bool {
        self.set.is_empty()
    }
}

/// One DAG level — the set of nodes that may fire in
/// parallel because none triggers another within the level. Nodes are in
/// graph (`config.nodes`) order for determinism (Principle #5/#7).
///
/// No stability promise: the shape may change with the
/// multithreaded executor.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Level {
    /// Node ids in this level, in graph order.
    pub nodes: Vec<String>,
}

/// The trigger-aware DAG levelization of a graph —
/// the ordered levels plus a node→level rank index for O(1) lookup.
///
/// Produced by [`GraphTopology::derive_levels`]. Level 0 is the roots
/// (sources, Period/External nodes, consumers of only external/non-trigger
/// topics, disconnected nodes); level `k` is the longest chain of
/// triggering edges of length `k` ending at that node.
///
/// No stability promise.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Levels {
    levels: Vec<Level>,
    rank: IndexMap<String, usize>,
}

impl Levels {
    /// The level index of `node_id`, or `None` if the node is not in the
    /// levelization (not wired into any topic).
    pub fn level_of(&self, node_id: &str) -> Option<usize> {
        self.rank.get(node_id).copied()
    }

    /// The nodes at level `idx`, or `None` if `idx` is out of range.
    pub fn level(&self, idx: usize) -> Option<&Level> {
        self.levels.get(idx)
    }

    /// Iterate the levels in order (level 0 first).
    pub fn iter(&self) -> impl Iterator<Item = &Level> {
        self.levels.iter()
    }

    /// The number of levels (the longest triggering chain + 1; 0 for an
    /// empty graph).
    pub fn len(&self) -> usize {
        self.levels.len()
    }

    /// True when there are no levels (the graph has no nodes/edges).
    pub fn is_empty(&self) -> bool {
        self.levels.is_empty()
    }
}

/// Whether [`GraphTopology::refine_levels`]'s phase-2 chain-cascade
/// may GROW the level count.
///
/// A cascade slides a dense chain later, appending levels when the chain has
/// no slack below the current max (the flagship perception-pipeline shape).
/// Growth HELPS a single-process (monolith) graph — measured −10% p50 with a
/// tail collapse — but is NOT free for a multi-process split: the
/// level-lockstep barrier advances ONE generation per level, so +1 level =
/// +1 cross-process rendezvous per step (measured ~19% chain-cadence cost at
/// neutral p50). Growth is therefore gated on the EMITTED shape — a
/// graph destined for a multi-group `process_groups:` partition refines under
/// [`Deny`], a single-group (monolith) graph under [`Allow`]. The gate is a
/// pure point-predicate on each cascade's target-level set, so refinement
/// stays byte-reproducible (Principle #7) in BOTH modes.
///
/// [`Allow`]: LevelGrowth::Allow
/// [`Deny`]: LevelGrowth::Deny
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum LevelGrowth {
    /// Cascades may append levels beyond the Kahn input's max level. The
    /// earlier behavior, and the DEFAULT — so every construction that
    /// omits the field is byte-stable with the pre-gate output. Correct for a
    /// single-group (monolith) destination, where each extra level is a free
    /// scheduler phase, not a cross-process barrier generation.
    #[default]
    Allow,
    /// A cascade whose target levels would EXCEED the Kahn input's max level
    /// (`base.len() - 1`) is refused ATOMICALLY — the whole `moved` set is
    /// dropped, exactly as with the NO-FASTER-DRAG guard. NON-growing
    /// cascades (every target within the original `0..base.len()` range)
    /// still apply: the gate kills GROWTH, not cascading. Set by the
    /// shape-gated `cerulion graph partition` wiring when the graph
    /// bands into multiple process groups, so refinement never adds a
    /// cross-process barrier generation.
    Deny,
}

/// The MINIMAL cost input [`GraphTopology::refine_levels`] reads.
///
/// A deliberate projection of the profiler's core cost snapshot
/// [`PartitionCosts`](super::partition::PartitionCosts): refinement needs only
/// the per-node compute cost and the per-edge fire rate, NOT the partition-only
/// hop-cost / budget fields. Keeping the input minimal keeps `topology` free of
/// any dependency on `partition` (the dependency runs one way today —
/// `partition` reuses `topology`'s levelizer, not the reverse).
///
/// Integer-only (ns + millihertz) so the refinement is byte-reproducible
/// (Principle #7) — the maps are only ever POINT-queried, never iterated for
/// ordering, so the `IndexMap` iteration order does not affect the output.
///
/// **Cost adapter (CLI wiring, not this module):**
/// `PartitionCosts { node_p50_ns, edge_rate_mhz, hop }` maps to
/// `RefineInputs { node_cost_ns: node_p50_ns, edge_rate_mhz }` — a two-field
/// copy, no new parsing. The `.costs.yaml` artifact the profiler already writes
/// carries both maps. The third field, [`level_growth`](Self::level_growth), is
/// NOT a cost — it is the shape-gated growth policy, left at its
/// [`Default`] by the cost adapter and set by the partition verb.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct RefineInputs {
    /// Per-node compute cost in ns (the profiler's `node_p50_ns`), keyed by
    /// node id. A node with NO entry is uncosted → PINNED at ASAP (conservative
    /// no-op). An EMPTY map is the no-profile-artifact contract:
    /// [`GraphTopology::refine_levels`] returns the Kahn levelization unchanged.
    pub node_cost_ns: IndexMap<String, u64>,
    /// Per trigger-edge fire rate in MILLIHERTZ (1 Hz = 1000 mHz), keyed by the
    /// `(producer_node, consumer_node)` pair — mirroring
    /// [`PartitionCosts::edge_rate_mhz`](super::partition::PartitionCosts::edge_rate_mhz).
    /// A pair with NO entry is rate 0 (gates nothing time-critical), the
    /// conservative default. Feeds the rate guard (protect faster co-located
    /// chains).
    pub edge_rate_mhz: IndexMap<(String, String), u64>,
    /// Whether the phase-2 chain-cascade may GROW the level count.
    /// [`LevelGrowth::Allow`] (the [`Default`], so every pre-gate construction
    /// stays byte-stable) preserves the pre-gate behavior;
    /// [`LevelGrowth::Deny`] refuses any cascade that would append a level
    /// (see [`LevelGrowth`]). Set to [`Deny`](LevelGrowth::Deny) by the
    /// shape-gated partition verb when the graph bands into
    /// multiple process groups; every other construction leaves it
    /// [`Allow`](LevelGrowth::Allow).
    pub level_growth: LevelGrowth,
}

/// An algebraic cycle — a feedback loop composed
/// ENTIRELY of triggering edges — was found during level derivation.
///
/// The ring (read via [`Self::ring`]) is the concrete loop naming, in
/// traversal order, with the entry node repeated at the end so a reader sees
/// the loop close (e.g. `["a", "b", "a"]`; a self-loop is `["a", "a"]`). The
/// ring is deterministic: [`GraphTopology::derive_levels`] tries every
/// survivor as a DFS root in graph order and reports the first ring found.
///
/// A loop that passes through at least one NON-triggering (latest-value)
/// read is LEGAL and never produces this error — see
/// [`GraphTopology::derive_levels`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CycleError {
    /// The ring of node ids, closed (entry repeated at the end). PRIVATE so
    /// the closed-ring invariant (entry repeated at the end; ≥ 2 entries)
    /// can only be established by [`GraphTopology::derive_levels`] —
    /// external code reads it through [`Self::ring`].
    cycle: Vec<String>,
}

impl CycleError {
    /// The concrete ring naming the algebraic loop, in traversal order, with
    /// the entry node repeated at the end so a reader sees the loop close
    /// (e.g. `["a", "b", "a"]`; a self-loop is `["a", "a"]`).
    pub fn ring(&self) -> &[String] {
        &self.cycle
    }
}

impl std::fmt::Display for CycleError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "algebraic trigger cycle: {}", self.cycle.join(" -> "))
    }
}

impl std::error::Error for CycleError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
    use crate::graph::node::{InputMeta, NodeInfo};

    fn input_meta(name: &str, policy: BackpressurePolicy, depth: usize) -> InputMeta {
        InputMeta {
            name: name.to_string(),
            schema_hash: 0,
            trigger: false,
            depth,
            backpressure: policy,
            expect_within_ms: None,
        }
    }

    fn node(id: &str, inputs: &[(&str, &str)], outputs: &[&str]) -> NodeDef {
        NodeDef {
            ros2: None,
            id: id.to_string(),
            node_type: id.to_string(),
            inputs: inputs
                .iter()
                .map(|(name, source)| InputDef {
                    name: name.to_string(),
                    source: source.to_string(),
                })
                .collect(),
            outputs: outputs
                .iter()
                .map(|name| OutputDef {
                    name: name.to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                })
                .collect(),
        }
    }

    fn config(prefix: &str, nodes: Vec<NodeDef>) -> GraphConfig {
        GraphConfig {
            level_assignments: None,
            network: None,
            process_groups: Default::default(),
            process_group_order: Default::default(),
            multi_publisher_topics: Vec::new(),
            name: None,
            identity: "g".to_string(),
            prefix: prefix.to_string(),
            nodes,
        }
    }

    fn infos(entries: Vec<(&str, Vec<InputMeta>)>) -> IndexMap<String, NodeInfo> {
        entries
            .into_iter()
            .map(|(id, metas)| {
                let info = NodeInfo::with_meta(metas, Vec::new());
                (id.to_string(), info)
            })
            .collect()
    }

    #[test]
    fn build_maps_producer_and_consumers_with_policy_and_depth() {
        // src --topic "p/src/out"--> dst.in
        let cfg = config(
            "p",
            vec![
                node("src", &[], &["out"]),
                node("dst", &[("in", "src/out")], &[]),
            ],
        );
        let entry_infos = infos(vec![
            ("src", vec![]),
            ("dst", vec![input_meta("in", BackpressurePolicy::Block, 4)]),
        ]);
        let topo = GraphTopology::build(&cfg, &entry_infos).expect("build");
        let topic = "/p/src/out".to_string();
        let flow = topo.topic(&topic).expect("topic present");
        assert_eq!(flow.producers, vec!["src".to_string()]);
        assert_eq!(flow.consumers.len(), 1);
        assert_eq!(flow.consumers[0].node_id, "dst");
        assert_eq!(flow.consumers[0].policy, BackpressurePolicy::Block);
        assert_eq!(flow.consumers[0].depth, 4);
        assert!(flow.is_all_block());
        assert!(flow.has_block_consumer());
        assert_eq!(flow.sum_consumer_depths(), 4);
    }

    #[test]
    fn mixed_consumers_are_not_all_block() {
        let cfg = config(
            "p",
            vec![
                node("src", &[], &["out"]),
                node("a", &[("in", "src/out")], &[]),
                node("b", &[("in", "src/out")], &[]),
            ],
        );
        let entry_infos = infos(vec![
            ("src", vec![]),
            ("a", vec![input_meta("in", BackpressurePolicy::Block, 4)]),
            (
                "b",
                vec![input_meta("in", BackpressurePolicy::DropOldest, 8)],
            ),
        ]);
        let topo = GraphTopology::build(&cfg, &entry_infos).expect("build");
        let flow = topo.topic("/p/src/out").unwrap();
        assert!(flow.has_block_consumer());
        assert!(
            !flow.is_all_block(),
            "mixed block+drop_oldest is NOT all-block"
        );
        assert_eq!(flow.sum_consumer_depths(), 12);
    }

    #[test]
    fn validate_rejects_zero_depth() {
        let cfg = config(
            "p",
            vec![
                node("src", &[], &["out"]),
                node("dst", &[("in", "src/out")], &[]),
            ],
        );
        let entry_infos = infos(vec![
            ("src", vec![]),
            (
                "dst",
                vec![input_meta("in", BackpressurePolicy::DropOldest, 0)],
            ),
        ]);
        let topo = GraphTopology::build(&cfg, &entry_infos).expect("build");
        let err = topo.validate().expect_err("depth=0 must be rejected");
        assert!(format!("{err}").contains("depth"), "got: {err}");
    }

    /// The cap value is a documented contract
    /// (`docs/user-api.md`) AND is mirrored as a hard-coded literal in
    /// `cerulion_macros::validate` (proc-macro crates cannot depend on
    /// `cerulion_core`, so the macro cannot reference this const).
    /// This oracle pin forces whoever changes the const to also visit
    /// the macro mirror + the trybuild snapshot + the docs.
    #[test]
    fn max_consumer_depth_pinned_at_64() {
        assert_eq!(
            MAX_CONSUMER_DEPTH, 64,
            "MAX_CONSUMER_DEPTH changed — update the hard-coded mirror in \
             cerulion_macros/src/validate.rs, the depth_above_max trybuild \
             snapshot, and USER_API.md"
        );
    }

    #[test]
    fn validate_accepts_depth_at_max() {
        let cfg = config(
            "p",
            vec![
                node("src", &[], &["out"]),
                node("dst", &[("in", "src/out")], &[]),
            ],
        );
        let entry_infos = infos(vec![
            ("src", vec![]),
            (
                "dst",
                vec![input_meta(
                    "in",
                    BackpressurePolicy::DropOldest,
                    MAX_CONSUMER_DEPTH,
                )],
            ),
        ]);
        let topo = GraphTopology::build(&cfg, &entry_infos).expect("build");
        topo.validate()
            .expect("depth == MAX_CONSUMER_DEPTH (boundary) must pass");
    }

    #[test]
    fn validate_rejects_depth_above_max() {
        let cfg = config(
            "p",
            vec![
                node("src", &[], &["out"]),
                node("dst", &[("in", "src/out")], &[]),
            ],
        );
        let entry_infos = infos(vec![
            ("src", vec![]),
            (
                "dst",
                vec![input_meta(
                    "in",
                    BackpressurePolicy::DropOldest,
                    MAX_CONSUMER_DEPTH + 1,
                )],
            ),
        ]);
        let topo = GraphTopology::build(&cfg, &entry_infos).expect("build");
        let err = topo
            .validate()
            .expect_err("depth = MAX_CONSUMER_DEPTH + 1 must be rejected");
        let msg = format!("{err}");
        assert!(
            msg.contains("maximum") && msg.contains("64"),
            "diagnostic must name the cap; got: {msg}"
        );
        assert!(
            msg.contains("backpressure policy"),
            "diagnostic must point at the remediation; got: {msg}"
        );
    }

    #[test]
    fn validate_rejects_block_without_in_graph_producer() {
        // dst.in reads "external/topic" with no producing node in the graph.
        let cfg = config("p", vec![node("dst", &[("in", "/external/topic")], &[])]);
        let entry_infos = infos(vec![(
            "dst",
            vec![input_meta("in", BackpressurePolicy::Block, 4)],
        )]);
        let topo = GraphTopology::build(&cfg, &entry_infos).expect("build");
        let err = topo
            .validate()
            .expect_err("block on producer-less topic must be rejected");
        assert!(
            format!("{err}").contains("no in-graph") || format!("{err}").contains("external"),
            "got: {err}"
        );
        // The rejection points at the host-level multi-graph scheduler as
        // the structural limitation —
        // deferring an out-of-graph publisher is its scope, not a
        // user error to silently work around.
        assert!(
            format!("{err}").contains("host-level multi-graph scheduler"),
            "the rejection must name the structural limitation: {err}"
        );
    }

    /// The refusal above is a TWO-SITUATION message, and its whole
    /// point is that a single remedy ("use drop_oldest or sample(N)")
    /// PRESCRIBES SILENT DATA LOSS when the real cause is a partition split —
    /// the shape a multi-process WORKER reaches, where the producer exists but
    /// landed in another group. Every needle the arm above asserts
    /// (`no in-graph` / `external` / `host-level multi-graph scheduler`) also fits
    /// a single-remedy message, so this is what pins the two-situation content.
    #[test]
    fn the_producer_less_block_refusal_separates_the_two_situations_it_can_mean() {
        let cfg = config("p", vec![node("dst", &[("in", "/external/topic")], &[])]);
        let entry_infos = infos(vec![(
            "dst",
            vec![input_meta("in", BackpressurePolicy::Block, 4)],
        )]);
        let topo = GraphTopology::build(&cfg, &entry_infos).expect("build");
        let msg = topo
            .validate()
            .expect_err("block on producer-less topic must be rejected")
            .to_string();
        // Situation (a): genuinely published from outside this graph.
        assert!(
            msg.contains("published from OUTSIDE this graph"),
            "the refusal must scope its drop_oldest remedy to the EXTERNAL case; got: {msg}"
        );
        // Situation (b): a partition split the producer away — the case whose
        // remedy is co-location, and for which a drop_oldest remedy is actively wrong.
        for needle in [
            "multi-process WORKER",
            "ONE `process_groups:` group",
            "--single-process",
            "never switch to drop_oldest for that",
            "silently loses data",
        ] {
            assert!(
                msg.contains(needle),
                "the split-partition situation must name `{needle}`; got: {msg}"
            );
        }
    }

    #[test]
    fn validate_allows_drop_oldest_on_external_topic() {
        // Non-block consumers on a producer-less topic are fine.
        let cfg = config("p", vec![node("dst", &[("in", "/external/topic")], &[])]);
        let entry_infos = infos(vec![(
            "dst",
            vec![input_meta("in", BackpressurePolicy::DropOldest, 4)],
        )]);
        let topo = GraphTopology::build(&cfg, &entry_infos).expect("build");
        topo.validate()
            .expect("drop_oldest on an external topic is allowed");
        assert!(topo
            .producers_of(&resolve_source("p", "/external/topic"))
            .is_empty());
    }

    #[test]
    fn build_rejects_double_producer() {
        // two nodes publish the same resolved topic
        let cfg = config(
            "p",
            vec![node("a", &[], &["dup"]), node("a2", &[], &["dup"])],
        );
        // derived names include node_id, so "/p/a/dup" != "/p/a2/dup" — to force a
        // collision we point both at the same topic via identical ids is not
        // possible; instead a single node publishing the same output twice is
        // caught by graph validation, so double-producer here is via two
        // nodes whose resolved topics coincide. The derived form includes the
        // node id, so this config does NOT collide — assert distinct topics.
        let topo = GraphTopology::build(&cfg, &infos(vec![("a", vec![]), ("a2", vec![])]))
            .expect("distinct topics build fine");
        assert_eq!(topo.topics().count(), 2);
    }

    #[test]
    fn validate_rejects_duplicate_consumer_edges() {
        // Defense in depth: `GraphTopology::build`
        // is `pub` (the reuse surface) and does not require the
        // caller to have run `validate_graph` first — fed a duplicate-input
        // config it APPENDS two same-keyed consumer edges (never silently
        // overwrites). `validate()` must then reject them so a keyed
        // fold-down downstream can never hit the silent-overwrite class.
        let cfg = config(
            "p",
            vec![
                node("a", &[], &["out"]),
                node("b", &[], &["out"]),
                node("dst", &[("in", "a/out"), ("in", "b/out")], &[]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![("a", vec![]), ("b", vec![]), ("dst", vec![])]),
        )
        .expect("build appends duplicate edges rather than overwriting");
        // Direct append pin: both duplicate edges must survive
        // build — a dedup-at-build regression would otherwise misattribute
        // the failure to validate().
        assert_eq!(
            topo.topics().map(|f| f.consumers.len()).sum::<usize>(),
            2,
            "build must append both duplicate edges"
        );
        let reason = topo
            .validate()
            .expect_err("duplicate consumer edge must be rejected")
            .to_string();
        assert!(reason.contains("duplicate consumer edge"), "got: {reason}");
        assert!(
            reason.contains("node 'dst'") && reason.contains("input name 'in'"),
            "error must bind node and input to the right roles: {reason}"
        );
    }

    #[test]
    fn validate_accepts_distinct_inputs_on_one_node() {
        // False-positive guard for the duplicate-edge check: one
        // node with two DIFFERENT input names must validate fine — the
        // check keys on (node, input), not node alone. Co-located with the
        // check so the contract doesn't ride on unrelated e2e suites
        // (macro_sync_threading_test currently covers it incidentally).
        let cfg = config(
            "p",
            vec![
                node("src", &[], &["out1", "out2"]),
                node("dst", &[("a", "src/out1"), ("b", "src/out2")], &[]),
            ],
        );
        let topo = GraphTopology::build(&cfg, &infos(vec![("src", vec![]), ("dst", vec![])]))
            .expect("build");
        topo.validate()
            .expect("two distinct inputs on one node must pass validate");
    }

    #[test]
    fn build_records_producer_under_topic_override() {
        // A `topic: /tf` output records its producer
        // edge under the ABSOLUTE name — so a `block` consumer wired to
        // /tf is allowed (the producer is in-graph and the scheduler can
        // defer it), and the derived name ceases to exist.
        let mut cfg = config(
            "p",
            vec![
                node("bc", &[], &["tf"]),
                node("loc", &[("tf_in", "/tf")], &[]),
            ],
        );
        cfg.nodes[0].outputs[0].topic = Some("/tf".to_string());
        let entry_infos = infos(vec![
            ("bc", vec![]),
            (
                "loc",
                vec![input_meta("tf_in", BackpressurePolicy::Block, 4)],
            ),
        ]);
        let topo = GraphTopology::build(&cfg, &entry_infos).expect("build");
        topo.validate()
            .expect("block on /tf is allowed — the producer is in-graph");
        assert_eq!(topo.producers_of("/tf"), &["bc".to_string()]);
        assert!(
            topo.topic("/p/bc/tf").is_none(),
            "the derived name must NOT exist when overridden"
        );
    }

    #[test]
    fn build_rejects_unlisted_override_double_producer() {
        // Two nodes overriding to the same absolute
        // topic WITHOUT the opt-in keep the single-producer rejection,
        // and the error points at the remedy. (validate_graph's
        // duplicate-topic check also expresses this collision;
        // the topology twin is the pub-surface defense.)
        let mut cfg = config("p", vec![node("a", &[], &["tf"]), node("b", &[], &["tf"])]);
        cfg.nodes[0].outputs[0].topic = Some("/tf".to_string());
        cfg.nodes[1].outputs[0].topic = Some("/tf".to_string());
        let entry_infos = infos(vec![("a", vec![]), ("b", vec![])]);
        let err = GraphTopology::build(&cfg, &entry_infos)
            .expect_err("unlisted double-producer must be rejected");
        let msg = format!("{err}");
        assert!(
            msg.contains("single producer") && msg.contains("multi_publisher_topics"),
            "the rejection must state the rule + the opt-in remedy: {msg}"
        );
    }

    #[test]
    fn build_accepts_listed_multi_producer_and_block_defers_all() {
        // Listing the topic relaxes the
        // rule — BOTH producers are recorded in graph order, and a block
        // consumer is allowed (every producer is in-graph and deferrable;
        // the runtime fans the same gate out to each).
        let mut cfg = config(
            "p",
            vec![
                node("a", &[], &["tf"]),
                node("b", &[], &["tf"]),
                node("loc", &[("tf_in", "/tf")], &[]),
            ],
        );
        cfg.nodes[0].outputs[0].topic = Some("/tf".to_string());
        cfg.nodes[1].outputs[0].topic = Some("/tf".to_string());
        cfg.multi_publisher_topics = vec!["/tf".to_string()];
        let entry_infos = infos(vec![
            ("a", vec![]),
            ("b", vec![]),
            (
                "loc",
                vec![input_meta("tf_in", BackpressurePolicy::Block, 4)],
            ),
        ]);
        let topo = GraphTopology::build(&cfg, &entry_infos).expect("listed multi-pub builds");
        topo.validate()
            .expect("block on a listed multi-producer topic is allowed (all in-graph)");
        assert_eq!(
            topo.producers_of("/tf"),
            &["a".to_string(), "b".to_string()],
            "both producers recorded in graph order"
        );
    }

    #[test]
    fn validate_rejects_literal_duplicate_producers() {
        // The producers dedup twin of the
        // duplicate-consumer-edge defense — `build` enforces it, but the
        // pub surface can carry literal-mutated models (in-crate
        // literal here; `#[non_exhaustive]` blocks external crates).
        let mut topo = GraphTopology::default();
        topo.topics.insert(
            "/tf".to_string(),
            TopicFlow {
                topic: "/tf".to_string(),
                producers: vec!["a".to_string(), "a".to_string()],
                consumers: Vec::new(),
            },
        );
        topo.multi_publisher_topics.insert("/tf".to_string());
        let err = topo
            .validate()
            .expect_err("duplicate producers in a literal model must be rejected");
        assert!(format!("{err}").contains("more than once"), "got: {err}");
    }

    #[test]
    fn validate_rejects_literal_unlisted_multi_producer() {
        // The legality twin: >1 producers without the captured opt-in —
        // the invariant is a property of the VALUE now, not of build's
        // stack frame.
        let mut topo = GraphTopology::default();
        topo.topics.insert(
            "/tf".to_string(),
            TopicFlow {
                topic: "/tf".to_string(),
                producers: vec!["a".to_string(), "b".to_string()],
                consumers: Vec::new(),
            },
        );
        let err = topo
            .validate()
            .expect_err("unlisted multi-producer literal model must be rejected");
        assert!(format!("{err}").contains("not listed in"), "got: {err}");
    }

    #[test]
    fn build_dedups_same_node_producer_on_listed_topic() {
        // The producer list is per NODE: one node publishing a listed
        // topic through two outputs is ONE deferrable producer (the
        // scheduler's pre-fire slot is per node), not two entries.
        let mut cfg = config("p", vec![node("a", &[], &["tf1", "tf2"])]);
        cfg.nodes[0].outputs[0].topic = Some("/tf".to_string());
        cfg.nodes[0].outputs[1].topic = Some("/tf".to_string());
        cfg.multi_publisher_topics = vec!["/tf".to_string()];
        let entry_infos = infos(vec![("a", vec![])]);
        let topo = GraphTopology::build(&cfg, &entry_infos).expect("same-node listed builds");
        assert_eq!(
            topo.producers_of("/tf"),
            &["a".to_string()],
            "same node through two outputs dedups to one producer entry"
        );
    }

    #[test]
    fn build_rejects_duplicate_input_meta() {
        // The strictest shape (not warn+first-wins):
        // conflicting policy/depth declarations are ambiguous, so
        // build REJECTS them. `NodeInfo::with_meta` asserts the same
        // invariant at construction, so the duplicate is
        // built via in-crate LITERAL here — exactly the residual path
        // (fields are `pub(crate)`) this build check still guards.
        // (Mutation oracle: deleting the rejection makes build succeed and
        // the expect_err panic.)
        let cfg = config(
            "p",
            vec![
                node("src", &[], &["out"]),
                node("dst", &[("in", "src/out")], &[]),
            ],
        );
        let mut entry_infos = infos(vec![("src", vec![])]);
        entry_infos.insert(
            "dst".to_string(),
            NodeInfo {
                input_names: vec!["in".to_string(), "in".to_string()],
                input_meta: vec![
                    input_meta("in", BackpressurePolicy::DropOldest, 4),
                    input_meta("in", BackpressurePolicy::Block, 2), // conflicting duplicate
                ],
                ..Default::default()
            },
        );
        let reason = GraphTopology::build(&cfg, &entry_infos)
            .expect_err("duplicate input metadata must be rejected at build")
            .to_string();
        assert!(reason.contains("duplicate input metadata"), "got: {reason}");
        assert!(
            reason.contains("node 'dst'") && reason.contains("'in'"),
            "error must bind node and input to the right roles: {reason}"
        );
        // A node with UNIQUE meta names still builds — false-positive
        // guard in BOTH shapes. The two-DISTINCT-metas block matters:
        // with only a single meta, `metas[..0]`
        // never evaluates the scan predicate, so a variant that rejects
        // whenever idx > 0 survives co-located — two distinct
        // metas evaluate the predicate at idx 1 and kill it (and an
        // inverted-comparison variant along with it).
        let entry_infos = infos(vec![
            ("src", vec![]),
            (
                "dst",
                vec![input_meta("in", BackpressurePolicy::DropOldest, 4)],
            ),
        ]);
        let topo = GraphTopology::build(&cfg, &entry_infos).expect("unique metas build fine");
        let flow = topo.topic("/p/src/out").expect("topic present");
        assert_eq!(flow.consumers[0].policy, BackpressurePolicy::DropOldest);
        assert_eq!(flow.consumers[0].depth, 4);
        let entry_infos = infos(vec![
            ("src", vec![]),
            (
                "dst",
                vec![
                    input_meta("in", BackpressurePolicy::DropOldest, 4),
                    input_meta("in2", BackpressurePolicy::Block, 2),
                ],
            ),
        ]);
        GraphTopology::build(&cfg, &entry_infos)
            .expect("two DISTINCT metas on one node must build");
    }

    // ── Trigger-aware DAG levels ──────────────

    /// Build a `TriggerEdges` set from `(node_id, topic)` literal pairs.
    /// The owner's `runtime::build_trigger_edges` produces the same shape
    /// from `MacroPolicy`; these tests supply the classification directly so
    /// the level algorithm is exercised in isolation (policy-free by
    /// design).
    fn trig(pairs: &[(&str, &str)]) -> TriggerEdges {
        let mut t = TriggerEdges::new();
        for (n, topic) in pairs {
            t.insert(*n, *topic);
        }
        t
    }

    /// Collect the levels into a `Vec<Vec<String>>` for oracle comparison.
    fn level_vecs(levels: &Levels) -> Vec<Vec<String>> {
        levels.iter().map(|l| l.nodes.clone()).collect()
    }

    #[test]
    fn levels_linear_chain() {
        // a/out --(trigger)--> b/out --(trigger)--> c
        let cfg = config(
            "p",
            vec![
                node("a", &[], &["out"]),
                node("b", &[("in", "a/out")], &["out"]),
                node("c", &[("in", "b/out")], &[]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![("a", vec![]), ("b", vec![]), ("c", vec![])]),
        )
        .expect("build");
        let edges = trig(&[("b", "/p/a/out"), ("c", "/p/b/out")]);
        let levels = topo.derive_levels(&edges).expect("acyclic");
        assert_eq!(
            level_vecs(&levels),
            vec![
                vec!["a".to_string()],
                vec!["b".to_string()],
                vec!["c".to_string()],
            ]
        );
        assert_eq!(levels.level_of("a"), Some(0));
        assert_eq!(levels.level_of("b"), Some(1));
        assert_eq!(levels.level_of("c"), Some(2));
        assert_eq!(levels.len(), 3);
        // Non-empty-graph None case: a node not in the levelization has no
        // level (distinct from the empty-graph None case).
        assert_eq!(levels.level_of("ghost"), None);
    }

    #[test]
    fn levels_diamond_consumer_at_max_plus_one() {
        // a --> b, a --> c, then d triggers on BOTH b and c.
        //   level 0: a   level 1: b, c   level 2: d
        let cfg = config(
            "p",
            vec![
                node("a", &[], &["out"]),
                node("b", &[("in", "a/out")], &["out"]),
                node("c", &[("in", "a/out")], &["out"]),
                node("d", &[("in_b", "b/out"), ("in_c", "c/out")], &[]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![
                ("a", vec![]),
                ("b", vec![]),
                ("c", vec![]),
                ("d", vec![]),
            ]),
        )
        .expect("build");
        let edges = trig(&[
            ("b", "/p/a/out"),
            ("c", "/p/a/out"),
            ("d", "/p/b/out"),
            ("d", "/p/c/out"),
        ]);
        let levels = topo.derive_levels(&edges).expect("acyclic");
        assert_eq!(
            level_vecs(&levels),
            vec![
                vec!["a".to_string()],
                vec!["b".to_string(), "c".to_string()],
                vec!["d".to_string()],
            ]
        );
        // d depends on the deeper of its two producers → max(1,1)+1 = 2.
        assert_eq!(levels.level_of("d"), Some(2));
    }

    #[test]
    fn levels_two_independent_chains() {
        // a --> b   and   x --> y, no cross edges. Within-level order is
        // graph order: level 0 = [a, x], level 1 = [b, y].
        let cfg = config(
            "p",
            vec![
                node("a", &[], &["out"]),
                node("x", &[], &["out"]),
                node("b", &[("in", "a/out")], &[]),
                node("y", &[("in", "x/out")], &[]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![
                ("a", vec![]),
                ("x", vec![]),
                ("b", vec![]),
                ("y", vec![]),
            ]),
        )
        .expect("build");
        let edges = trig(&[("b", "/p/a/out"), ("y", "/p/x/out")]);
        let levels = topo.derive_levels(&edges).expect("acyclic");
        assert_eq!(
            level_vecs(&levels),
            vec![
                vec!["a".to_string(), "x".to_string()],
                vec!["b".to_string(), "y".to_string()],
            ]
        );
    }

    #[test]
    fn levels_disconnected_node_is_root() {
        // `lonely` publishes a topic nobody consumes — it has an edge in the
        // topology (a producer entry) but no triggering IN-edge, so it is a
        // level-0 root.
        let cfg = config(
            "p",
            vec![
                node("a", &[], &["out"]),
                node("b", &[("in", "a/out")], &[]),
                node("lonely", &[], &["out"]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![("a", vec![]), ("b", vec![]), ("lonely", vec![])]),
        )
        .expect("build");
        let edges = trig(&[("b", "/p/a/out")]);
        let levels = topo.derive_levels(&edges).expect("acyclic");
        assert_eq!(
            level_vecs(&levels),
            vec![
                vec!["a".to_string(), "lonely".to_string()],
                vec!["b".to_string()],
            ]
        );
        assert_eq!(levels.level_of("lonely"), Some(0));
    }

    #[test]
    fn levels_fully_disconnected_node_is_level0_root() {
        // `island` has NO inputs and NO outputs — zero topic edges, so it
        // never appears in any `TopicFlow`. The build-captured node set still
        // places it as a level-0 root (completeness — the executor must never
        // silently skip a side-effect-only node).
        let cfg = config(
            "p",
            vec![
                node("a", &[], &["out"]),
                node("b", &[("in", "a/out")], &[]),
                node("island", &[], &[]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![("a", vec![]), ("b", vec![]), ("island", vec![])]),
        )
        .expect("build");
        let edges = trig(&[("b", "/p/a/out")]);
        let levels = topo.derive_levels(&edges).expect("acyclic");
        assert_eq!(
            level_vecs(&levels),
            vec![
                vec!["a".to_string(), "island".to_string()],
                vec!["b".to_string()],
            ]
        );
        assert_eq!(levels.level_of("island"), Some(0));
    }

    #[test]
    fn levels_multi_publisher_consumer_at_max_over_n() {
        // Two producers a, b publish the SAME listed topic /tf; consumer c
        // triggers on it. c depends on BOTH a and b → max(0, 0) + 1 = 1.
        let mut cfg = config(
            "p",
            vec![
                node("a", &[], &["tf"]),
                node("b", &[], &["tf"]),
                node("c", &[("tf_in", "/tf")], &[]),
            ],
        );
        cfg.nodes[0].outputs[0].topic = Some("/tf".to_string());
        cfg.nodes[1].outputs[0].topic = Some("/tf".to_string());
        cfg.multi_publisher_topics = vec!["/tf".to_string()];
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![("a", vec![]), ("b", vec![]), ("c", vec![])]),
        )
        .expect("build");
        let edges = trig(&[("c", "/tf")]);
        let levels = topo.derive_levels(&edges).expect("acyclic");
        assert_eq!(
            level_vecs(&levels),
            vec![
                vec!["a".to_string(), "b".to_string()],
                vec!["c".to_string()],
            ]
        );
        assert_eq!(levels.level_of("c"), Some(1));
    }

    #[test]
    fn levels_multi_publisher_takes_max_of_producer_levels() {
        // Producer `deep` is itself at level 1 (it triggers on `a`); `b` is a
        // level-0 producer. `c` triggers on /tf which BOTH publish, so its
        // level is max(level(deep)=1, level(b)=0) + 1 = 2 — not 1.
        let mut cfg = config(
            "p",
            vec![
                node("a", &[], &["out"]),
                node("b", &[], &["tf"]),
                node("deep", &[("in", "a/out")], &["tf"]),
                node("c", &[("tf_in", "/tf")], &[]),
            ],
        );
        cfg.nodes[1].outputs[0].topic = Some("/tf".to_string());
        cfg.nodes[2].outputs[0].topic = Some("/tf".to_string());
        cfg.multi_publisher_topics = vec!["/tf".to_string()];
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![
                ("a", vec![]),
                ("b", vec![]),
                ("deep", vec![]),
                ("c", vec![]),
            ]),
        )
        .expect("build");
        let edges = trig(&[("deep", "/p/a/out"), ("c", "/tf")]);
        let levels = topo.derive_levels(&edges).expect("acyclic");
        assert_eq!(levels.level_of("a"), Some(0));
        assert_eq!(levels.level_of("b"), Some(0));
        assert_eq!(levels.level_of("deep"), Some(1));
        assert_eq!(
            levels.level_of("c"),
            Some(2),
            "consumer of a multi-pub topic is max over ALL producer levels + 1"
        );
    }

    #[test]
    fn levels_external_source_consumer_is_root() {
        // `consumer` triggers on an external topic with no in-graph
        // producer — no triggering IN-edge → level-0 root.
        let cfg = config(
            "p",
            vec![node("consumer", &[("h", "/external/health")], &[])],
        );
        let topo = GraphTopology::build(&cfg, &infos(vec![("consumer", vec![])])).expect("build");
        let edges = trig(&[("consumer", "/external/health")]);
        let levels = topo.derive_levels(&edges).expect("acyclic");
        assert_eq!(level_vecs(&levels), vec![vec!["consumer".to_string()]]);
        assert_eq!(levels.level_of("consumer"), Some(0));
    }

    #[test]
    fn levels_non_trigger_edge_excluded_from_dag() {
        // a --> b on /p/a/out, but b's read is NON-triggering. b therefore
        // has NO triggering in-edge → both a and b are level-0 roots even
        // though a structural (policy-blind) check would put b at level 1.
        let cfg = config(
            "p",
            vec![node("a", &[], &["out"]), node("b", &[("in", "a/out")], &[])],
        );
        let topo =
            GraphTopology::build(&cfg, &infos(vec![("a", vec![]), ("b", vec![])])).expect("build");
        // Empty trigger set — the a→b edge is latest-value.
        let levels = topo.derive_levels(&TriggerEdges::new()).expect("acyclic");
        assert_eq!(
            level_vecs(&levels),
            vec![vec!["a".to_string(), "b".to_string()]],
            "a non-triggering read is not a DAG edge; both nodes are roots"
        );
    }

    #[test]
    fn levels_feedback_loop_through_non_trigger_is_accepted() {
        // ROBOTICS-CRITICAL: estimator/controller feedback loop.
        //   estimator --state--> controller   (controller TRIGGERS on state)
        //   controller --cmd--> estimator     (estimator READS last cmd,
        //                                       NON-trigger — latest value)
        // Structurally this is a 2-cycle; trigger-aware it is a single
        // triggering edge estimator→controller, so it is ACCEPTED and
        // levelized (estimator level 0, controller level 1).
        let cfg = config(
            "p",
            vec![
                node("estimator", &[("cmd", "controller/cmd")], &["state"]),
                node("controller", &[("state", "estimator/state")], &["cmd"]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![("estimator", vec![]), ("controller", vec![])]),
        )
        .expect("build");
        // ONLY controller's read of estimator/state triggers. estimator's
        // read of controller/cmd is the latest-value back-edge — excluded.
        let edges = trig(&[("controller", "/p/estimator/state")]);
        let levels = topo
            .derive_levels(&edges)
            .expect("a loop through a non-trigger read is legal");
        assert_eq!(
            level_vecs(&levels),
            vec![
                vec!["estimator".to_string()],
                vec!["controller".to_string()],
            ]
        );
    }

    #[test]
    fn levels_algebraic_loop_is_rejected() {
        // The SAME two nodes, but now BOTH reads trigger → a genuine
        // algebraic loop. Rejected, ring named.
        let cfg = config(
            "p",
            vec![
                node("estimator", &[("cmd", "controller/cmd")], &["state"]),
                node("controller", &[("state", "estimator/state")], &["cmd"]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![("estimator", vec![]), ("controller", vec![])]),
        )
        .expect("build");
        let edges = trig(&[
            ("controller", "/p/estimator/state"),
            ("estimator", "/p/controller/cmd"),
        ]);
        let err = topo
            .derive_levels(&edges)
            .expect_err("an all-trigger loop must be rejected");
        // DFS root = lowest graph order survivor = estimator (index 0). Ring
        // closes back to it.
        assert_eq!(
            err.ring(),
            vec![
                "estimator".to_string(),
                "controller".to_string(),
                "estimator".to_string(),
            ]
            .as_slice(),
            "ring named, rooted at the lowest-graph-order survivor"
        );
        assert!(
            format!("{err}").contains("estimator -> controller -> estimator"),
            "Display renders the ring: {err}"
        );
    }

    #[test]
    fn cycle_recovery_self_loop() {
        // A node triggering on its own output: self-loop ["a", "a"].
        let cfg = config("p", vec![node("a", &[("loop_in", "a/out")], &["out"])]);
        let topo = GraphTopology::build(&cfg, &infos(vec![("a", vec![])])).expect("build");
        let edges = trig(&[("a", "/p/a/out")]);
        let err = topo
            .derive_levels(&edges)
            .expect_err("self trigger loop must be rejected");
        assert_eq!(
            err.ring(),
            vec!["a".to_string(), "a".to_string()].as_slice()
        );
    }

    #[test]
    fn cycle_recovery_two_cycle_anchored_at_lowest_order() {
        // Graph order forces the anchor: declare `zeta` BEFORE `alpha`, so
        // the lowest-graph-order survivor is `zeta` (index 0) — the ring
        // anchors there, NOT at the alphabetically-first node.
        let cfg = config(
            "p",
            vec![
                node("zeta", &[("in", "alpha/out")], &["out"]),
                node("alpha", &[("in", "zeta/out")], &["out"]),
            ],
        );
        let topo = GraphTopology::build(&cfg, &infos(vec![("zeta", vec![]), ("alpha", vec![])]))
            .expect("build");
        let edges = trig(&[("zeta", "/p/alpha/out"), ("alpha", "/p/zeta/out")]);
        let err = topo
            .derive_levels(&edges)
            .expect_err("2-cycle must be rejected");
        assert_eq!(
            err.ring(),
            vec!["zeta".to_string(), "alpha".to_string(), "zeta".to_string()].as_slice(),
            "rooted at lowest GRAPH order (zeta), not alphabetical"
        );
    }

    #[test]
    fn cycle_recovery_three_chain() {
        // a -> b -> c -> a, all triggering. Anchor a (index 0).
        let cfg = config(
            "p",
            vec![
                node("a", &[("in", "c/out")], &["out"]),
                node("b", &[("in", "a/out")], &["out"]),
                node("c", &[("in", "b/out")], &["out"]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![("a", vec![]), ("b", vec![]), ("c", vec![])]),
        )
        .expect("build");
        let edges = trig(&[("a", "/p/c/out"), ("b", "/p/a/out"), ("c", "/p/b/out")]);
        let err = topo
            .derive_levels(&edges)
            .expect_err("3-chain cycle must be rejected");
        assert_eq!(
            err.ring(),
            vec![
                "a".to_string(),
                "b".to_string(),
                "c".to_string(),
                "a".to_string(),
            ]
            .as_slice()
        );
    }

    #[test]
    fn cycle_recovery_names_only_the_ring_not_the_tail() {
        // A 2-cycle b<->c with an acyclic tail: a feeds b (a is NOT in the
        // ring), and d reads c non-triggering (d is NOT in the ring). Only
        // the b<->c ring is named.
        //   a --> b,  b <-> c (both trigger),  c --> d (d non-trigger)
        let cfg = config(
            "p",
            vec![
                node("a", &[], &["out"]),
                node("b", &[("from_a", "a/out"), ("from_c", "c/out")], &["out"]),
                node("c", &[("from_b", "b/out")], &["out"]),
                node("d", &[("from_c", "c/out")], &[]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![
                ("a", vec![]),
                ("b", vec![]),
                ("c", vec![]),
                ("d", vec![]),
            ]),
        )
        .expect("build");
        // a→b triggers, b↔c both trigger; d's read of c is NON-trigger.
        let edges = trig(&[("b", "/p/a/out"), ("b", "/p/c/out"), ("c", "/p/b/out")]);
        let err = topo
            .derive_levels(&edges)
            .expect_err("the b<->c ring must be rejected");
        // a is peeled (level 0); d is peeled (no triggering in-edge — its
        // read of c is non-trigger). Survivors are exactly {b, c}; the first
        // survivor root in graph order is b.
        assert_eq!(
            err.ring(),
            vec!["b".to_string(), "c".to_string(), "b".to_string()].as_slice(),
            "only the ring is named — the acyclic tail (a, d) is excluded"
        );
    }

    #[test]
    fn cycle_recovery_anchor_is_sink_downstream_of_ring() {
        // REGRESSION: the lowest-graph-order survivor is
        // NOT necessarily ON a ring. Here `b` is a pure SINK fed by the ring
        // {x, y}: it has residual in-degree > 0 (x triggers it) so Kahn never
        // peels it, but it has NO outgoing survivor edge. The OLD
        // single-anchor DFS rooted at the lowest-order survivor (`b`),
        // exhausted with no successors, and fell back to naming `["b"]` — a
        // node not on any ring. The multi-root DFS skips past `b` (its
        // subgraph holds no ring) and reports the real {x, y} ring.
        //
        // Triggering edges: x→b, x→y, y→x ({x,y} is the ring; b is the sink).
        // Declared order b, x, y so the lowest-order survivor IS the sink `b`.
        let cfg = config(
            "p",
            vec![
                node("b", &[("from_x", "x/out")], &[]),
                node("x", &[("from_y", "y/out")], &["out"]),
                node("y", &[("from_x", "x/out")], &["out"]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![("b", vec![]), ("x", vec![]), ("y", vec![])]),
        )
        .expect("build");
        let edges = trig(&[("b", "/p/x/out"), ("y", "/p/x/out"), ("x", "/p/y/out")]);
        let err = topo
            .derive_levels(&edges)
            .expect_err("the {x, y} ring must be rejected");
        assert_eq!(
            err.ring(),
            vec!["x".to_string(), "y".to_string(), "x".to_string()].as_slice(),
            "the ring is {{x, y}} — NOT the sink `b` the single-anchor DFS \
             would have named"
        );
    }

    #[test]
    fn levels_external_trigger_on_producerless_topic_is_root() {
        // A single node that TRIGGERS (non-empty TriggerEdges) on a
        // producer-less ABSOLUTE topic. The existing
        // `levels_external_source_consumer_is_root` test passes an empty
        // trigger set, masking the `is_triggering == true` AND
        // `flow.producers.is_empty()` branch in derive_levels: the edge is
        // classified triggering, but the topic has NO in-graph producer, so
        // NO DAG edge is added and the node stays a level-0 root.
        let cfg = config(
            "p",
            vec![node("watcher", &[("h", "/external/health")], &[])],
        );
        let topo = GraphTopology::build(&cfg, &infos(vec![("watcher", vec![])])).expect("build");
        // The read IS triggering — but the topic is producer-less.
        let edges = trig(&[("watcher", "/external/health")]);
        let levels = topo
            .derive_levels(&edges)
            .expect("a triggering read of a producer-less topic adds no edge");
        assert_eq!(level_vecs(&levels), vec![vec!["watcher".to_string()]]);
        assert_eq!(levels.level_of("watcher"), Some(0));
    }

    #[test]
    fn levels_deep_chain_longest_path() {
        // LONGEST-PATH (not BFS-depth) pin. Edges: a→b, b→d, a→c, c→d, b→c.
        //   a: root                                  → 0
        //   b: triggers on a                         → 1
        //   c: triggers on a AND b → max(0,1)+1      → 2
        //   d: triggers on b AND c → max(1,2)+1      → 3
        // A naive BFS-from-roots would rank d at 2 (shortest a→b→d); Kahn's
        // longest-path peeling ranks it 3.
        let cfg = config(
            "p",
            vec![
                node("a", &[], &["out"]),
                node("b", &[("from_a", "a/out")], &["out"]),
                node("c", &[("from_a", "a/out"), ("from_b", "b/out")], &["out"]),
                node("d", &[("from_b", "b/out"), ("from_c", "c/out")], &[]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![
                ("a", vec![]),
                ("b", vec![]),
                ("c", vec![]),
                ("d", vec![]),
            ]),
        )
        .expect("build");
        let edges = trig(&[
            ("b", "/p/a/out"),
            ("d", "/p/b/out"),
            ("c", "/p/a/out"),
            ("d", "/p/c/out"),
            ("c", "/p/b/out"),
        ]);
        let levels = topo.derive_levels(&edges).expect("acyclic");
        assert_eq!(levels.level_of("a"), Some(0));
        assert_eq!(levels.level_of("b"), Some(1));
        assert_eq!(
            levels.level_of("c"),
            Some(2),
            "c triggers on a (L0) AND b (L1) → max+1 = 2"
        );
        assert_eq!(
            levels.level_of("d"),
            Some(3),
            "d triggers on b (L1) AND c (L2) → max+1 = 3 (longest path, not BFS-depth)"
        );
        assert_eq!(levels.len(), 4, "four levels — the longest chain a→b→c→d");
        // Non-empty-graph None case: a node not in the graph has no level.
        assert_eq!(levels.level_of("ghost"), None);
    }

    #[test]
    fn levels_empty_graph_is_empty() {
        let cfg = config("p", vec![]);
        let topo = GraphTopology::build(&cfg, &infos(vec![])).expect("build");
        let levels = topo.derive_levels(&TriggerEdges::new()).expect("acyclic");
        assert!(levels.is_empty());
        assert_eq!(levels.len(), 0);
        assert_eq!(levels.level_of("nope"), None);
    }

    // ================= Cost-aware level refinement =================

    /// Build a [`RefineInputs`] from cost + rate literals. Rates are flat
    /// `(producer, consumer, rate_mhz)` triples.
    fn refine_in(costs: &[(&str, u64)], rates: &[(&str, &str, u64)]) -> RefineInputs {
        let mut node_cost_ns = IndexMap::new();
        for (n, c) in costs {
            node_cost_ns.insert((*n).to_string(), *c);
        }
        let mut edge_rate_mhz = IndexMap::new();
        for (p, c, r) in rates {
            edge_rate_mhz.insert(((*p).to_string(), (*c).to_string()), *r);
        }
        RefineInputs {
            node_cost_ns,
            edge_rate_mhz,
            ..Default::default()
        }
    }

    /// Assert every listed `(producer, consumer)` trigger edge is strictly
    /// level-increasing under `levels` (the correctness invariant).
    fn assert_edges_increasing(levels: &Levels, edges: &[(&str, &str)]) {
        for (p, c) in edges {
            let lp = levels.level_of(p).unwrap_or_else(|| panic!("{p} placed"));
            let lc = levels.level_of(c).unwrap_or_else(|| panic!("{c} placed"));
            assert!(lp < lc, "edge {p}(L{lp}) -> {c}(L{lc}) must be increasing");
        }
    }

    /// Σ of node costs at `lvl` (uncosted nodes contribute 0).
    fn level_cost(levels: &Levels, inputs: &RefineInputs, lvl: usize) -> u64 {
        levels
            .level(lvl)
            .map(|l| {
                l.nodes
                    .iter()
                    .map(|n| inputs.node_cost_ns.get(n).copied().unwrap_or(0))
                    .sum()
            })
            .unwrap_or(0)
    }

    /// The humanoid-shaped fixture (oracle (a)/(c)/(f)): a cheap 1 kHz
    /// proprioceptive chain (`imu` → `imu_filter`-sink) sharing level 0 with
    /// two expensive 30 Hz camera roots (`cam_head`/`cam_depth`) whose only
    /// trigger-consumer (`fuse`) sits at level 2, plus a cheap `mid` →
    /// `pre_fuse` bridge that puts `fuse` at level 2. Config node order is
    /// `[imu, imu_filter, cam_head, cam_depth, mid, pre_fuse, fuse]`.
    fn humanoid() -> (GraphTopology, TriggerEdges) {
        let cfg = config(
            "p",
            vec![
                node("imu", &[], &["out"]),
                node("imu_filter", &[("i", "imu/out")], &[]),
                node("cam_head", &[], &["out"]),
                node("cam_depth", &[], &["out"]),
                node("mid", &[], &["out"]),
                node("pre_fuse", &[("i", "mid/out")], &["out"]),
                node(
                    "fuse",
                    &[
                        ("a", "cam_head/out"),
                        ("b", "cam_depth/out"),
                        ("c", "pre_fuse/out"),
                    ],
                    &[],
                ),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![
                ("imu", vec![]),
                ("imu_filter", vec![]),
                ("cam_head", vec![]),
                ("cam_depth", vec![]),
                ("mid", vec![]),
                ("pre_fuse", vec![]),
                ("fuse", vec![]),
            ]),
        )
        .expect("build");
        let edges = trig(&[
            ("imu_filter", "/p/imu/out"),
            ("pre_fuse", "/p/mid/out"),
            ("fuse", "/p/cam_head/out"),
            ("fuse", "/p/cam_depth/out"),
            ("fuse", "/p/pre_fuse/out"),
        ]);
        (topo, edges)
    }

    /// The `(producer, consumer)` trigger edges of [`humanoid`], for the
    /// strictly-increasing invariant check.
    const HUMANOID_EDGES: &[(&str, &str)] = &[
        ("imu", "imu_filter"),
        ("mid", "pre_fuse"),
        ("cam_head", "fuse"),
        ("cam_depth", "fuse"),
        ("pre_fuse", "fuse"),
    ];

    #[test]
    fn refine_humanoid_moves_expensive_cams_off_the_cheap_chain() {
        let (topo, edges) = humanoid();
        let base = topo.derive_levels(&edges).expect("acyclic");
        // Kahn ASAP: L0 = [imu, cam_head, cam_depth, mid], L1 = [imu_filter,
        // pre_fuse], L2 = [fuse]. Cameras dominate L0's cost pre-refinement.
        assert_eq!(base.level_of("cam_head"), Some(0));
        assert_eq!(base.level_of("cam_depth"), Some(0));

        let inputs = refine_in(
            &[
                ("imu", 5),
                ("imu_filter", 5),
                ("cam_head", 500),
                ("cam_depth", 500),
                ("mid", 5),
                ("pre_fuse", 5),
                ("fuse", 50),
            ],
            &[
                ("imu", "imu_filter", 1_000_000), // 1 kHz proprioceptive
                ("cam_head", "fuse", 30_000),     // 30 Hz camera
                ("cam_depth", "fuse", 30_000),
                ("mid", "pre_fuse", 30_000),
                ("pre_fuse", "fuse", 30_000),
            ],
        );
        let refined = topo.refine_levels(&base, &edges, &inputs);

        // The cameras vacate level 0 (their slack destination L1 gates only the
        // 30 Hz fuse chain — no faster co-located chain to protect).
        assert_eq!(
            refined.level_of("cam_head"),
            Some(1),
            "cam_head moved to L1"
        );
        assert_eq!(refined.level_of("cam_depth"), Some(1), "cam_depth to L1");
        // The 1 kHz chain and the bridge are untouched.
        assert_eq!(refined.level_of("imu"), Some(0), "imu pinned (zero slack)");
        assert_eq!(refined.level_of("imu_filter"), Some(1), "sink pinned");
        assert_eq!(refined.level_of("mid"), Some(0));
        assert_eq!(refined.level_of("pre_fuse"), Some(1));
        assert_eq!(refined.level_of("fuse"), Some(2), "sink pinned at L-1");
        // Level 0 is now exactly the cheap survivors, in graph order.
        assert_eq!(
            refined.level(0).unwrap().nodes,
            vec!["imu".to_string(), "mid".to_string()]
        );
        // Every trigger edge is still strictly level-increasing.
        assert_edges_increasing(&refined, HUMANOID_EDGES);
        // Level count preserved; no level emptied.
        assert_eq!(refined.len(), base.len());
        assert!(refined.iter().all(|l| !l.nodes.is_empty()));

        // The headline: L0's compute collapsed to a small fraction of pre.
        let pre = level_cost(&base, &inputs, 0); // 5+500+500+5 = 1010
        let post = level_cost(&refined, &inputs, 0); // 5+5 = 10
        assert_eq!(pre, 1010);
        assert_eq!(post, 10);
        assert!(post * 10 < pre, "L0 post {post} not << pre {pre}");
    }

    #[test]
    fn refine_no_cost_input_is_byte_identical_noop() {
        let (topo, edges) = humanoid();
        let base = topo.derive_levels(&edges).expect("acyclic");
        // Completely absent costs ⇒ the Kahn levelization is returned unchanged.
        let refined = topo.refine_levels(&base, &edges, &RefineInputs::default());
        assert_eq!(
            refined, base,
            "empty cost input must be a byte-identical no-op"
        );
        // Rates present but node costs absent is still a no-op (nothing to
        // relocate without a cost).
        let rates_only = refine_in(&[], &[("cam_head", "fuse", 30_000)]);
        let refined2 = topo.refine_levels(&base, &edges, &rates_only);
        assert_eq!(refined2, base, "no node costs ⇒ still a no-op");
    }

    #[test]
    fn refine_partial_costs_pin_uncosted_nodes_at_asap() {
        let (topo, edges) = humanoid();
        let base = topo.derive_levels(&edges).expect("acyclic");
        // Only cam_head is costed (+ its outgoing rate). cam_depth is uncosted.
        let inputs = refine_in(&[("cam_head", 500)], &[("cam_head", "fuse", 30_000)]);
        let refined = topo.refine_levels(&base, &edges, &inputs);
        assert_eq!(
            refined.level_of("cam_head"),
            Some(1),
            "costed + slack ⇒ moves"
        );
        assert_eq!(
            refined.level_of("cam_depth"),
            Some(0),
            "uncosted ⇒ pinned at ASAP even with identical slack"
        );
        // Uncosted structural nodes stay put too.
        assert_eq!(refined.level_of("imu"), Some(0));
        assert_eq!(refined.level_of("imu_filter"), Some(1));
        assert_eq!(refined.level_of("fuse"), Some(2));
        assert_edges_increasing(&refined, HUMANOID_EDGES);
    }

    #[test]
    fn refine_zero_slack_pipeline_is_unchanged() {
        // a -> b -> c -> d, a straight chain: every node has zero slack.
        let cfg = config(
            "p",
            vec![
                node("a", &[], &["out"]),
                node("b", &[("i", "a/out")], &["out"]),
                node("c", &[("i", "b/out")], &["out"]),
                node("d", &[("i", "c/out")], &[]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![
                ("a", vec![]),
                ("b", vec![]),
                ("c", vec![]),
                ("d", vec![]),
            ]),
        )
        .expect("build");
        let edges = trig(&[("b", "/p/a/out"), ("c", "/p/b/out"), ("d", "/p/c/out")]);
        let base = topo.derive_levels(&edges).expect("acyclic");
        let inputs = refine_in(
            &[("a", 100), ("b", 100), ("c", 100), ("d", 100)],
            &[("a", "b", 1_000), ("b", "c", 1_000), ("c", "d", 1_000)],
        );
        let refined = topo.refine_levels(&base, &edges, &inputs);
        assert_eq!(refined, base, "a zero-slack pipeline must be unchanged");
    }

    #[test]
    fn refine_slow_chain_sink_cascades_but_protected_chain_never_moves() {
        // The sink pin under the chain-cascade pass: a cascaded
        // chain's sink moves WITH its chain; the pin that holds is that
        // the PROTECTED (max-rate) chain's nodes never move.
        //
        // Shape: a FAST 1 kHz chain r -> hub -> deep (deep = its sink) plus
        // a SLOW 10 Hz chain x -> bigsink whose huge sink is co-located with
        // hub at L1. Arm 1 (rate-skewed): bigsink cascades L1 -> L2 (the
        // fast chain's sink level, where deep gates nothing) — sinks are no
        // longer absolutely pinned. Arm 2 (all rates equal): NOTHING moves —
        // phase 2 fires only on a genuine rate skew.
        let mk = || {
            let cfg = config(
                "p",
                vec![
                    node("r", &[], &["out"]),
                    node("hub", &[("i", "r/out")], &["out"]),
                    node("deep", &[("i", "hub/out")], &[]),
                    node("x", &[], &["out"]),
                    node("bigsink", &[("i", "x/out")], &[]),
                ],
            );
            let topo = GraphTopology::build(
                &cfg,
                &infos(vec![
                    ("r", vec![]),
                    ("hub", vec![]),
                    ("deep", vec![]),
                    ("x", vec![]),
                    ("bigsink", vec![]),
                ]),
            )
            .expect("build");
            let edges = trig(&[
                ("hub", "/p/r/out"),
                ("deep", "/p/hub/out"),
                ("bigsink", "/p/x/out"),
            ]);
            (topo, edges)
        };

        // Arm 1: rate-skewed.
        let (topo, edges) = mk();
        let base = topo.derive_levels(&edges).expect("acyclic");
        assert_eq!(base.level_of("bigsink"), Some(1));
        let inputs = refine_in(
            &[
                ("r", 5),
                ("hub", 5),
                ("deep", 5),
                ("x", 5),
                ("bigsink", 9999),
            ],
            &[
                ("r", "hub", 1_000_000), // 1 kHz protected chain
                ("hub", "deep", 1_000_000),
                ("x", "bigsink", 10_000), // 10 Hz slow chain
            ],
        );
        let refined = topo.refine_levels(&base, &edges, &inputs);
        assert_eq!(
            refined.level_of("bigsink"),
            Some(2),
            "the slow chain's huge SINK cascades off the fast chain's gating \
             level (hub gates deep at 1 kHz through L1)"
        );
        // The protected (max-rate) chain never moves — the surviving pin.
        assert_eq!(refined.level_of("r"), Some(0));
        assert_eq!(refined.level_of("hub"), Some(1));
        assert_eq!(refined.level_of("deep"), Some(2), "the fast sink stays");
        // x (@L0, cost 5) does not strictly dominate its level (r also 5) —
        // immaterial, stays; only the dominating sink moved.
        assert_eq!(refined.level_of("x"), Some(0));
        assert_eq!(refined.len(), 3, "bigsink joined the existing L2");
        assert_edges_increasing(&refined, &[("r", "hub"), ("hub", "deep"), ("x", "bigsink")]);
        assert!(refined.iter().all(|l| !l.nodes.is_empty()));

        // Arm 2: ALL RATES EQUAL — no strictly-faster-gating mate anywhere.
        let (topo2, edges2) = mk();
        let base2 = topo2.derive_levels(&edges2).expect("acyclic");
        let equal = refine_in(
            &[
                ("r", 5),
                ("hub", 5),
                ("deep", 5),
                ("x", 5),
                ("bigsink", 9999),
            ],
            &[
                ("r", "hub", 1_000),
                ("hub", "deep", 1_000),
                ("x", "bigsink", 1_000),
            ],
        );
        let refined2 = topo2.refine_levels(&base2, &edges2, &equal);
        assert_eq!(
            refined2, base2,
            "equal rates => no relief => byte-identical"
        );
    }

    #[test]
    fn refine_is_deterministic_across_runs() {
        let (topo, edges) = humanoid();
        let base = topo.derive_levels(&edges).expect("acyclic");
        let inputs = refine_in(
            &[
                ("imu", 5),
                ("cam_head", 500),
                ("cam_depth", 500),
                ("mid", 5),
                ("pre_fuse", 5),
                ("fuse", 50),
            ],
            &[
                ("imu", "imu_filter", 1_000_000),
                ("cam_head", "fuse", 30_000),
                ("cam_depth", "fuse", 30_000),
                ("mid", "pre_fuse", 30_000),
                ("pre_fuse", "fuse", 30_000),
            ],
        );
        let a = topo.refine_levels(&base, &edges, &inputs);
        let b = topo.refine_levels(&base, &edges, &inputs);
        assert_eq!(a, b, "same inputs ⇒ byte-identical output");
        // A freshly rebuilt topology yields the same refinement.
        let (topo2, edges2) = humanoid();
        let base2 = topo2.derive_levels(&edges2).expect("acyclic");
        let c = topo2.refine_levels(&base2, &edges2, &inputs);
        assert_eq!(a, c, "rebuild ⇒ same refinement");
    }

    /// The rate-weighting fixture (oracle (g)): a fast chain `f0 -> f1 -> f2`
    /// (f2 sink) whose level 1 (`f1`) gates a 1 kHz edge, plus an expensive,
    /// slow node `x` whose ONLY slack destination is that level 1 (its
    /// consumer `xc` sits at level 2 because `xc` also reads `f1`). Config
    /// order `[f0, f1, f2, x, xc]`. `f1_rate` sets both of `f1`'s outgoing
    /// edge rates so the caller can toggle whether L1 gates a fast chain.
    fn rate_weight_fixture(f1_rate: u64) -> (GraphTopology, TriggerEdges, RefineInputs, Levels) {
        let cfg = config(
            "p",
            vec![
                node("f0", &[], &["out"]),
                node("f1", &[("i", "f0/out")], &["out"]),
                node("f2", &[("i", "f1/out")], &[]),
                node("x", &[], &["out"]),
                node("xc", &[("a", "f1/out"), ("b", "x/out")], &[]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![
                ("f0", vec![]),
                ("f1", vec![]),
                ("f2", vec![]),
                ("x", vec![]),
                ("xc", vec![]),
            ]),
        )
        .expect("build");
        let edges = trig(&[
            ("f1", "/p/f0/out"),
            ("f2", "/p/f1/out"),
            ("xc", "/p/f1/out"),
            ("xc", "/p/x/out"),
        ]);
        let base = topo.derive_levels(&edges).expect("acyclic");
        let inputs = refine_in(
            &[("f0", 5), ("f1", 5), ("f2", 5), ("x", 500), ("xc", 5)],
            &[
                ("f0", "f1", 1_000_000),
                ("f1", "f2", f1_rate),
                ("f1", "xc", f1_rate),
                ("x", "xc", 10_000), // 10 Hz — x's own (slow) chain
            ],
        );
        (topo, edges, inputs, base)
    }

    #[test]
    fn refine_no_faster_drag_stops_a_march_and_slow_dest_still_accepts() {
        // The level-span model has no destination-occupant guard: a
        // spine node occupying the destination does not block a
        // provably-neutral step. x (10 Hz, 500ns) at L0
        // shares the graph with the 1 kHz chain f0 -> f1 -> f2 and the
        // fast-rated join xc (consumes f1 @1 kHz AND x @10 Hz => chain rate
        // 1 kHz). Phase 1 refuses x's slack move (its within-slack
        // guard applies). Phase 2: x marches 0 -> 1 (relief: the
        // non-downstream 1 kHz sink f2@2 still waits through L0), but the
        // next step (x -> 2) would DRAG xc (chain 1 kHz > x's 10 Hz) — the
        // NO-FASTER-DRAG rule refuses it atomically, so x rests at 1 (a
        // latency-NEUTRAL mid-span position: every faster span contains L0
        // and L1 alike). Reverting the drag rule marches x + xc to the pass
        // cap — this pin kills that mutation.
        let (topo, edges, inputs, base) = rate_weight_fixture(1_000_000);
        assert_eq!(base.level_of("x"), Some(0));
        assert_eq!(base.level_of("xc"), Some(2), "x has slack (consumer at L2)");
        let refined = topo.refine_levels(&base, &edges, &inputs);
        assert_eq!(
            refined.level_of("x"),
            Some(1),
            "x marches one step, then the drag rule stops it"
        );
        assert_eq!(
            refined.level_of("xc"),
            Some(2),
            "the fast-rated join is NEVER dragged by the slower root"
        );
        assert_eq!(refined.level_of("f0"), Some(0));
        assert_eq!(refined.level_of("f1"), Some(1));
        assert_eq!(
            refined.level_of("f2"),
            Some(2),
            "the 1 kHz chain never moves"
        );
        assert_eq!(refined.len(), 3, "no level growth — the march was capped");
        assert_edges_increasing(
            &refined,
            &[("f0", "f1"), ("f1", "f2"), ("f1", "xc"), ("x", "xc")],
        );

        // Slow-f1 control (was the phase-1 anti-tautology toggle, still is):
        // with f1's edges at 1 Hz, phase 1's slack guard admits x -> 1, and
        // phase 2 finds NO faster sink above (xc's chain rate 10 Hz == x's,
        // f2's 1 Hz slower) => x stays exactly at the phase-1 placement.
        let (topo2, edges2, inputs2, base2) = rate_weight_fixture(1_000);
        let refined2 = topo2.refine_levels(&base2, &edges2, &inputs2);
        assert_eq!(
            refined2.level_of("x"),
            Some(1),
            "with a slow f1, phase 1 places x at 1 and phase 2 adds nothing"
        );
    }

    /// RELIEF-absence arm (the rate-guard pin): a slow expensive
    /// node whose ONLY faster sink is DOWNSTREAM of it (the fast-rated join
    /// it feeds) has no relief — a downstream sink is dragged by the node's
    /// own cascade and can never be escaped, so chasing it is excluded and
    /// the node NEVER moves (byte-identical). Kills a revert of the
    /// downstream-cone exclusion (x would pointlessly march to 1).
    #[test]
    fn refine_downstream_only_faster_sink_is_no_relief() {
        // f0 -> f1 -> xc and x -> xc: xc (chain 1 kHz via f1) is the ONLY
        // sink above x, and it is downstream of x. No f2.
        let cfg = config(
            "p",
            vec![
                node("f0", &[], &["out"]),
                node("f1", &[("i", "f0/out")], &["out"]),
                node("x", &[], &["out"]),
                node("xc", &[("a", "f1/out"), ("b", "x/out")], &[]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![
                ("f0", vec![]),
                ("f1", vec![]),
                ("x", vec![]),
                ("xc", vec![]),
            ]),
        )
        .expect("build");
        let edges = trig(&[("f1", "/p/f0/out"), ("xc", "/p/f1/out"), ("xc", "/p/x/out")]);
        let base = topo.derive_levels(&edges).expect("acyclic");
        assert_eq!(base.level_of("x"), Some(0));
        assert_eq!(base.level_of("xc"), Some(2));
        let inputs = refine_in(
            &[("f0", 5), ("f1", 5), ("x", 500), ("xc", 5)],
            &[
                ("f0", "f1", 1_000_000),
                ("f1", "xc", 1_000_000),
                ("x", "xc", 10_000),
            ],
        );
        // Phase 1: x's slack destination L1 hosts f1 (gating 1 kHz > 10 Hz)
        // => refused (phase 1's own guard). Phase 2: the only
        // faster sink (xc) is downstream => no relief => never a root.
        let refined = topo.refine_levels(&base, &edges, &inputs);
        assert_eq!(
            refined, base,
            "a downstream-only faster sink is no relief — byte-identical"
        );
    }

    #[test]
    fn refine_empty_graph() {
        let cfg = config("p", vec![]);
        let topo = GraphTopology::build(&cfg, &infos(vec![])).expect("build");
        let base = topo.derive_levels(&TriggerEdges::new()).expect("acyclic");
        let refined = topo.refine_levels(&base, &TriggerEdges::new(), &RefineInputs::default());
        assert!(refined.is_empty());
        assert_eq!(refined, base);
    }

    #[test]
    fn refine_single_node() {
        let cfg = config("p", vec![node("solo", &[], &["out"])]);
        let topo = GraphTopology::build(&cfg, &infos(vec![("solo", vec![])])).expect("build");
        let edges = TriggerEdges::new();
        let base = topo.derive_levels(&edges).expect("acyclic");
        // A lone root is a sink (no trigger consumer) ⇒ pinned whether costed
        // or not.
        let costed = refine_in(&[("solo", 9999)], &[]);
        let refined = topo.refine_levels(&base, &edges, &costed);
        assert_eq!(refined.level_of("solo"), Some(0));
        assert_eq!(refined.len(), 1);
        assert_eq!(refined, base);
        // Uncosted single node is identical.
        let refined_uncosted = topo.refine_levels(&base, &edges, &refine_in(&[], &[]));
        assert_eq!(refined_uncosted, base);
    }

    #[test]
    fn refine_all_equal_costs_uses_graph_order_within_level() {
        // The humanoid shape with ALL node costs equal: the cameras still move
        // (slack + a non-faster destination), and within-level ordering is
        // graph order — pinning a HashMap-random regression via the exact vecs.
        let (topo, edges) = humanoid();
        let base = topo.derive_levels(&edges).expect("acyclic");
        let inputs = refine_in(
            &[
                ("imu", 100),
                ("imu_filter", 100),
                ("cam_head", 100),
                ("cam_depth", 100),
                ("mid", 100),
                ("pre_fuse", 100),
                ("fuse", 100),
            ],
            &[
                ("imu", "imu_filter", 1_000_000),
                ("cam_head", "fuse", 30_000),
                ("cam_depth", "fuse", 30_000),
                ("mid", "pre_fuse", 30_000),
                ("pre_fuse", "fuse", 30_000),
            ],
        );
        let refined = topo.refine_levels(&base, &edges, &inputs);
        assert_eq!(
            level_vecs(&refined),
            vec![
                vec!["imu".to_string(), "mid".to_string()],
                vec![
                    "imu_filter".to_string(),
                    "cam_head".to_string(),
                    "cam_depth".to_string(),
                    "pre_fuse".to_string(),
                ],
                vec!["fuse".to_string()],
            ],
            "within-level order is graph order"
        );
        // Deterministic under all-equal costs.
        let again = topo.refine_levels(&base, &edges, &inputs);
        assert_eq!(refined, again);
    }

    #[test]
    fn refine_twenty_node_dag_preserves_all_invariants() {
        // A fixed 20-node funnel with deliberate slack:
        //   s0..s7  (L0 sources) -> m0..m3 (L1) -> a0,a1 (L2) -> t (L3 sink)
        //   e0..e4  (L0) -> t directly (slack 2 each — t is at L3)
        // Deterministic (seeded) costs/rates via index formulas.
        let mut node_ids: Vec<String> = Vec::new();
        for i in 0..8 {
            node_ids.push(format!("s{i}"));
        }
        for i in 0..4 {
            node_ids.push(format!("m{i}"));
        }
        node_ids.push("a0".to_string());
        node_ids.push("a1".to_string());
        node_ids.push("t".to_string());
        for i in 0..5 {
            node_ids.push(format!("e{i}"));
        }
        assert_eq!(node_ids.len(), 20);

        // (producer, consumer) trigger edges.
        let mut e: Vec<(String, String)> = Vec::new();
        for i in 0..4 {
            e.push((format!("s{}", 2 * i), format!("m{i}")));
            e.push((format!("s{}", 2 * i + 1), format!("m{i}")));
        }
        e.push(("m0".into(), "a0".into()));
        e.push(("m1".into(), "a0".into()));
        e.push(("m2".into(), "a1".into()));
        e.push(("m3".into(), "a1".into()));
        e.push(("a0".into(), "t".into()));
        e.push(("a1".into(), "t".into()));
        for i in 0..5 {
            e.push((format!("e{i}"), "t".into()));
        }

        // Every node publishes "out"; each consumer sources every producer's
        // "<producer>/out". Build NodeDefs from the edge list.
        let mut inputs_by_node: IndexMap<String, Vec<(String, String)>> = IndexMap::new();
        for id in &node_ids {
            inputs_by_node.insert(id.clone(), Vec::new());
        }
        for (p, c) in &e {
            inputs_by_node
                .get_mut(c)
                .unwrap()
                .push((format!("in_{p}"), format!("{p}/out")));
        }
        let nodes: Vec<NodeDef> = node_ids
            .iter()
            .map(|id| {
                let ins = &inputs_by_node[id];
                let in_refs: Vec<(&str, &str)> =
                    ins.iter().map(|(n, s)| (n.as_str(), s.as_str())).collect();
                node(id, &in_refs, &["out"])
            })
            .collect();
        let cfg = config("p", nodes);
        let topo = GraphTopology::build(
            &cfg,
            &infos(node_ids.iter().map(|id| (id.as_str(), vec![])).collect()),
        )
        .expect("build");

        let mut te = TriggerEdges::new();
        for (p, c) in &e {
            te.insert(c.as_str(), format!("/p/{p}/out"));
        }
        let base = topo.derive_levels(&te).expect("acyclic");

        // Seeded costs + rates (deterministic literals).
        let mut node_cost_ns = IndexMap::new();
        for (idx, id) in node_ids.iter().enumerate() {
            node_cost_ns.insert(id.clone(), ((idx * 37 + 11) % 100 + 1) as u64);
        }
        let mut edge_rate_mhz = IndexMap::new();
        for (idx, (p, c)) in e.iter().enumerate() {
            edge_rate_mhz.insert((p.clone(), c.clone()), ((idx * 13 + 1) * 1000) as u64);
        }
        let inputs = RefineInputs {
            node_cost_ns,
            edge_rate_mhz,
            ..Default::default()
        };

        let refined = topo.refine_levels(&base, &te, &inputs);

        // Invariants. (The chain-cascade pass may GROW
        // the level count — this seeded shape's rates DO trigger cascades —
        // so the pin is `>= base.len()` + contiguity, not `==`;
        // termination is implicitly pinned by the test completing at all.)
        let edge_refs: Vec<(&str, &str)> =
            e.iter().map(|(p, c)| (p.as_str(), c.as_str())).collect();
        assert_edges_increasing(&refined, &edge_refs);
        assert!(
            refined.len() >= base.len(),
            "cascades only ever add levels (got {} vs base {})",
            refined.len(),
            base.len()
        );
        assert!(
            refined.len() <= 20,
            "contiguous non-empty levels cannot exceed the node count"
        );
        assert!(
            refined.iter().all(|l| !l.nodes.is_empty()),
            "no level emptied"
        );
        let total: usize = refined.iter().map(|l| l.nodes.len()).sum();
        assert_eq!(total, 20, "every node partitioned exactly once");
        // Determinism.
        let again = topo.refine_levels(&base, &te, &inputs);
        assert_eq!(refined, again);
    }

    /// The DEEP-SPINE dense-pipeline fixture (oracle for the cascade
    /// model AND the growth gate): a DEEP 1 kHz spine `base_imu ->
    /// fbe -> wbc -> jtc` (sink @L3) spanning nearly every level PLUS the
    /// shallow 1 kHz `imu_filter` sink @L1, and the zero-slack camera pipeline
    /// (cams @L0, consumers at exactly L1, rectify -> stereo -> vision). Costs
    /// are the REAL artifact's (ns). Returns `(topo, te, base, inputs,
    /// edge_pairs)`; `inputs` carries the default [`LevelGrowth::Allow`].
    fn deep_spine_fixture() -> (
        GraphTopology,
        TriggerEdges,
        Levels,
        RefineInputs,
        &'static [(&'static str, &'static str)],
    ) {
        let cfg = config(
            "p",
            vec![
                node("base_imu", &[], &["out"]),
                node("imu_filter", &[("i", "base_imu/out")], &[]),
                node("fbe", &[("i", "base_imu/out")], &["out"]),
                node("wbc", &[("i", "fbe/out")], &["out"]),
                node("jtc", &[("i", "wbc/out")], &[]),
                node("cam_l", &[], &["out"]),
                node("cam_r", &[], &["out"]),
                node("cam_d", &[], &["out"]),
                node("rect_l", &[("i", "cam_l/out")], &["out"]),
                node("rect_r", &[("i", "cam_r/out")], &["out"]),
                node("pcf", &[("i", "cam_d/out")], &["out"]),
                node(
                    "stereo",
                    &[("a", "rect_l/out"), ("b", "rect_r/out")],
                    &["out"],
                ),
                node("vision", &[("i", "stereo/out")], &[]),
                node("elev", &[("i", "pcf/out")], &[]),
            ],
        );
        let ids = [
            "base_imu",
            "imu_filter",
            "fbe",
            "wbc",
            "jtc",
            "cam_l",
            "cam_r",
            "cam_d",
            "rect_l",
            "rect_r",
            "pcf",
            "stereo",
            "vision",
            "elev",
        ];
        let topo = GraphTopology::build(&cfg, &infos(ids.iter().map(|id| (*id, vec![])).collect()))
            .expect("build");
        let edge_pairs: &[(&str, &str)] = &[
            ("base_imu", "imu_filter"),
            ("base_imu", "fbe"),
            ("fbe", "wbc"),
            ("wbc", "jtc"),
            ("cam_l", "rect_l"),
            ("cam_r", "rect_r"),
            ("cam_d", "pcf"),
            ("rect_l", "stereo"),
            ("rect_r", "stereo"),
            ("stereo", "vision"),
            ("pcf", "elev"),
        ];
        let mut te = TriggerEdges::new();
        for (prod, cons) in edge_pairs {
            te.insert(*cons, format!("/p/{prod}/out"));
        }
        let base = topo.derive_levels(&te).expect("acyclic");
        let inputs = refine_in(
            &[
                ("base_imu", 8_400),
                ("imu_filter", 8_200),
                ("fbe", 9_000),
                ("wbc", 12_000),
                ("jtc", 7_000),
                ("cam_l", 19_500),
                ("cam_r", 20_900),
                ("cam_d", 23_600),
                ("rect_l", 5_000),
                ("rect_r", 5_000),
                ("pcf", 6_000),
                ("stereo", 15_000),
                ("vision", 30_000),
                ("elev", 80_400),
            ],
            &[
                ("base_imu", "imu_filter", 1_000_000), // 1 kHz
                ("base_imu", "fbe", 1_000_000),
                ("fbe", "wbc", 1_000_000),
                ("wbc", "jtc", 1_000_000),
                ("cam_l", "rect_l", 30_000), // 30 Hz camera chain
                ("cam_r", "rect_r", 30_000),
                ("cam_d", "pcf", 30_000),
                ("rect_l", "stereo", 30_000),
                ("rect_r", "stereo", 30_000),
                ("stereo", "vision", 30_000),
                ("pcf", "elev", 30_000),
            ],
        );
        (topo, te, base, inputs, edge_pairs)
    }

    /// The REAL-SHAPE oracle: under the level-span model
    /// the cams march to the DEEPEST faster sink's level (jtc @L3 — AT it, not
    /// above: a sink never waits for its own level's mates), fully leaving
    /// every 1 kHz waiting span, and the spine + imu_filter never move. This
    /// is the DEFAULT ([`LevelGrowth::Allow`]) behavior — growth 4 -> 7.
    #[test]
    fn refine_dense_pipeline_cascades_cam_chain_off_l0_real_humanoid_shape() {
        let (topo, te, base, inputs, edge_pairs) = deep_spine_fixture();
        // Kahn: the dense shape — cams AT L0, consumers at exactly L1; the
        // spine spans L0..L3 with its sink jtc at the last level.
        assert_eq!(base.level_of("cam_d"), Some(0));
        assert_eq!(base.level_of("pcf"), Some(1), "zero slack: consumer at L1");
        assert_eq!(base.level_of("jtc"), Some(3), "the deep 1 kHz sink");
        assert_eq!(base.len(), 4);

        let refined = topo.refine_levels(&base, &te, &inputs);

        // Hand-traced final (3 marching passes + a moveless 4th): the cams
        // marched to L3 — AT the deepest 1 kHz sink (jtc), past BOTH 1 kHz
        // waiting spans — dragging their chain below them; the spine +
        // imu_filter are byte-unmoved; the count GREW 4 -> 7.
        assert_eq!(
            level_vecs(&refined),
            vec![
                vec!["base_imu".to_string()],
                vec!["imu_filter".to_string(), "fbe".to_string()],
                vec!["wbc".to_string()],
                vec![
                    "jtc".to_string(),
                    "cam_l".to_string(),
                    "cam_r".to_string(),
                    "cam_d".to_string(),
                ],
                vec![
                    "rect_l".to_string(),
                    "rect_r".to_string(),
                    "pcf".to_string(),
                ],
                vec!["stereo".to_string(), "elev".to_string()],
                vec!["vision".to_string()],
            ],
            "cams AT the deepest 1 kHz sink's level; spine unmoved"
        );
        assert_eq!(refined.len(), 7, "level count grew (4 -> 7)");
        // The spine + shallow sink never moved (the protected chains).
        for (n, l) in [
            ("base_imu", 0),
            ("imu_filter", 1),
            ("fbe", 1),
            ("wbc", 2),
            ("jtc", 3),
        ] {
            assert_eq!(refined.level_of(n), Some(l), "1 kHz node {n} unmoved");
        }
        assert_edges_increasing(&refined, edge_pairs);
        assert!(refined.iter().all(|l| !l.nodes.is_empty()), "contiguous");
        // The headline: every 1 kHz waiting span is now camera-free — the
        // spans' levels (L0..L2) carry ONLY spine costs.
        let pre = level_cost(&base, &inputs, 0); // 8400+19500+20900+23600
        assert_eq!(pre, 72_400);
        assert_eq!(level_cost(&refined, &inputs, 0), 8_400);
        assert_eq!(level_cost(&refined, &inputs, 1), 8_200 + 9_000);
        assert_eq!(level_cost(&refined, &inputs, 2), 12_000);
        // Determinism across runs.
        let again = topo.refine_levels(&base, &te, &inputs);
        assert_eq!(refined, again);
    }

    /// Two FAST chains at the SAME rate: nothing moves regardless of cost
    /// skew — phase 2's relief requirement demands a STRICTLY faster gated
    /// chain at the origin level.
    #[test]
    fn refine_two_equal_rate_chains_move_nothing() {
        let cfg = config(
            "p",
            vec![
                node("a0", &[], &["out"]),
                node("b0", &[], &["out"]),
                node("a1", &[("i", "a0/out")], &[]),
                node("b1", &[("i", "b0/out")], &[]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![
                ("a0", vec![]),
                ("b0", vec![]),
                ("a1", vec![]),
                ("b1", vec![]),
            ]),
        )
        .expect("build");
        let edges = trig(&[("a1", "/p/a0/out"), ("b1", "/p/b0/out")]);
        let base = topo.derive_levels(&edges).expect("acyclic");
        let inputs = refine_in(
            &[("a0", 100), ("b0", 90_000), ("a1", 5), ("b1", 5)],
            &[("a0", "a1", 1_000_000), ("b0", "b1", 1_000_000)],
        );
        let refined = topo.refine_levels(&base, &edges, &inputs);
        assert_eq!(
            refined, base,
            "equal gating rates everywhere => zero relief => byte-identical"
        );
    }

    /// The REAL humanoid artifact FIXTURE (Principle #13 — real measured data,
    /// not fabricated): the FULL 36-node humanoid graph with its ACTUAL
    /// profiled costs + edge rates, transcribed VERBATIM from the profiler
    /// artifact `hum_test.costs.yaml` of a profiled humanoid graph (the
    /// `cerulion graph profile humanoid` output measured on a Jetson; the file
    /// is not in the repo, and every number is inlined here so this is fully self-contained and
    /// does not read the file). 30 costed nodes; 24 rated edges; 6
    /// uncosted/isolated nodes (ethercat_master, friction_compensation,
    /// joint_torque_controller, manipulation_planner, safety_monitor,
    /// swing_leg_planner — pinned). The trigger-edge structure is transcribed
    /// from the bench node sources (`#[input(trigger)]` fields); latest-value
    /// (non-`trigger`) reads are NOT DAG edges, so tf_broadcaster /
    /// robot_state_publisher / manipulation_planner are L0 sources. Returns
    /// `(topo, te, base, inputs, trig_edges)`; `inputs` carries the default
    /// [`LevelGrowth::Allow`].
    fn real_humanoid_fixture() -> (
        GraphTopology,
        TriggerEdges,
        Levels,
        RefineInputs,
        &'static [(&'static str, &'static str)],
    ) {
        // Node ids in humanoid.yaml declaration order (= within-level order).
        let ids: &[&str] = &[
            "joint_encoder_reader",
            "base_imu",
            "foot_ft_left",
            "foot_ft_right",
            "wrist_ft_left",
            "wrist_ft_right",
            "foot_contact",
            "head_cam_left",
            "head_cam_right",
            "head_depth",
            "manipulation_planner",
            "tf_broadcaster",
            "robot_state_publisher",
            "diagnostics",
            "floating_base_estimator",
            "contact_estimator",
            "imu_filter",
            "image_rectify_left",
            "image_rectify_right",
            "point_cloud_filter",
            "vision_detection",
            "stereo_disparity",
            "elevation_mapping",
            "centroidal_mpc",
            "whole_body_controller",
            "com_estimator",
            "gravity_compensation",
            "joint_limit_monitor",
            "object_tracker",
            "terrain_traversability",
            "joint_torque_controller",
            "footstep_planner",
            "friction_compensation",
            "swing_leg_planner",
            "safety_monitor",
            "ethercat_master",
        ];
        assert_eq!(ids.len(), 36, "the full humanoid graph");
        // The (producer, consumer) TRIGGER edges — every wired
        // `#[input(trigger)]` in the bench sources.
        let trig_edges: &[(&str, &str)] = &[
            ("joint_encoder_reader", "floating_base_estimator"),
            ("base_imu", "floating_base_estimator"),
            ("base_imu", "imu_filter"),
            ("foot_ft_left", "floating_base_estimator"),
            ("foot_ft_left", "contact_estimator"),
            ("foot_ft_right", "floating_base_estimator"),
            ("foot_ft_right", "contact_estimator"),
            ("foot_contact", "floating_base_estimator"),
            ("floating_base_estimator", "elevation_mapping"),
            ("floating_base_estimator", "centroidal_mpc"),
            ("floating_base_estimator", "whole_body_controller"),
            ("floating_base_estimator", "com_estimator"),
            ("floating_base_estimator", "gravity_compensation"),
            ("floating_base_estimator", "joint_limit_monitor"),
            ("floating_base_estimator", "joint_torque_controller"),
            ("floating_base_estimator", "safety_monitor"),
            ("head_cam_left", "image_rectify_left"),
            ("head_cam_right", "image_rectify_right"),
            ("head_depth", "point_cloud_filter"),
            ("image_rectify_left", "vision_detection"),
            ("image_rectify_left", "stereo_disparity"),
            ("image_rectify_right", "stereo_disparity"),
            ("point_cloud_filter", "elevation_mapping"),
            ("elevation_mapping", "terrain_traversability"),
            ("vision_detection", "object_tracker"),
            ("terrain_traversability", "footstep_planner"),
            ("whole_body_controller", "joint_torque_controller"),
            ("joint_torque_controller", "friction_compensation"),
            ("footstep_planner", "swing_leg_planner"),
            ("friction_compensation", "safety_monitor"),
            ("safety_monitor", "ethercat_master"),
        ];

        // Build NodeDefs: every node emits "out"; each consumer sources
        // "<producer>/out" for each incoming trigger edge (input name
        // "in_<producer>"). Nodes with no incoming trigger edge are L0 sources.
        let mut ins_by: IndexMap<&str, Vec<(String, String)>> = IndexMap::new();
        for id in ids {
            ins_by.insert(id, Vec::new());
        }
        for (p, c) in trig_edges {
            ins_by
                .get_mut(*c)
                .expect("consumer in ids")
                .push((format!("in_{p}"), format!("{p}/out")));
        }
        let nodes: Vec<NodeDef> = ids
            .iter()
            .map(|id| {
                let refs: Vec<(&str, &str)> = ins_by[*id]
                    .iter()
                    .map(|(n, s)| (n.as_str(), s.as_str()))
                    .collect();
                node(id, &refs, &["out"])
            })
            .collect();
        let cfg = config("hum", nodes);
        let topo = GraphTopology::build(&cfg, &infos(ids.iter().map(|id| (*id, vec![])).collect()))
            .expect("build");
        let mut te = TriggerEdges::new();
        for (p, c) in trig_edges {
            te.insert(*c, format!("/hum/{p}/out"));
        }
        let base = topo.derive_levels(&te).expect("acyclic");
        // Costs + rates transcribed VERBATIM from hum_test.costs.yaml. The 6
        // uncosted (isolated) nodes carry NO cost entry ⇒ pinned at ASAP. The
        // 7 rateless trigger edges (into/out of uncosted nodes — they never
        // fired during profiling) carry NO rate ⇒ 0.
        let inputs = refine_in(
            &[
                ("base_imu", 8384),
                ("centroidal_mpc", 7872),
                ("com_estimator", 8289),
                ("contact_estimator", 16897),
                ("diagnostics", 27873),
                ("elevation_mapping", 80387),
                ("floating_base_estimator", 32641),
                ("foot_contact", 8097),
                ("foot_ft_left", 8576),
                ("foot_ft_right", 9024),
                ("footstep_planner", 11968),
                ("gravity_compensation", 8800),
                ("head_cam_left", 19552),
                ("head_cam_right", 20864),
                ("head_depth", 23649),
                ("image_rectify_left", 14817),
                ("image_rectify_right", 14049),
                ("imu_filter", 8160),
                ("joint_encoder_reader", 8769),
                ("joint_limit_monitor", 8384),
                ("object_tracker", 10785),
                ("point_cloud_filter", 15168),
                ("robot_state_publisher", 9345),
                ("stereo_disparity", 22401),
                ("terrain_traversability", 12737),
                ("tf_broadcaster", 7328),
                ("vision_detection", 13601),
                ("whole_body_controller", 7264),
                ("wrist_ft_left", 6592),
                ("wrist_ft_right", 6785),
            ],
            &[
                ("base_imu", "floating_base_estimator", 1_032_171),
                ("base_imu", "imu_filter", 1_032_171),
                ("elevation_mapping", "terrain_traversability", 10_044),
                ("floating_base_estimator", "centroidal_mpc", 610_867),
                ("floating_base_estimator", "com_estimator", 610_867),
                ("floating_base_estimator", "elevation_mapping", 610_867),
                ("floating_base_estimator", "gravity_compensation", 610_867),
                ("floating_base_estimator", "joint_limit_monitor", 610_867),
                ("floating_base_estimator", "whole_body_controller", 610_867),
                ("foot_contact", "floating_base_estimator", 1_030_839),
                ("foot_ft_left", "contact_estimator", 1_031_915),
                ("foot_ft_left", "floating_base_estimator", 1_031_915),
                ("foot_ft_right", "contact_estimator", 1_031_659),
                ("foot_ft_right", "floating_base_estimator", 1_031_659),
                ("head_cam_left", "image_rectify_left", 31_209),
                ("head_cam_right", "image_rectify_right", 31_209),
                ("head_depth", "point_cloud_filter", 10_300),
                ("image_rectify_left", "stereo_disparity", 30_338),
                ("image_rectify_left", "vision_detection", 30_338),
                ("image_rectify_right", "stereo_disparity", 30_389),
                ("joint_encoder_reader", "floating_base_estimator", 1_032_479),
                ("point_cloud_filter", "elevation_mapping", 10_095),
                ("terrain_traversability", "footstep_planner", 10_044),
                ("vision_detection", "object_tracker", 30_338),
            ],
        );
        (topo, te, base, inputs, trig_edges)
    }

    /// The REAL humanoid artifact regression (the head_depth defect): the FULL
    /// 36-node humanoid graph with its ACTUAL profiled costs + edge rates —
    /// the DEFAULT ([`LevelGrowth::Allow`]) behavior (growth 7 -> 8).
    ///
    /// PROVENANCE (Principle #13 — real measured data, not fabricated):
    /// * node costs + edge rates transcribed VERBATIM from the profiler
    ///   artifact `hum_test.costs.yaml` of a profiled humanoid graph —
    ///   the `cerulion graph profile humanoid` output measured on a Jetson,
    ///   not kept in the repo (regenerate on any machine with the humanoid bench
    ///   workspace; every number is inlined below, so this test is fully
    ///   self-contained and does not read the file)
    ///   (30 costed nodes; 24 rated edges; 6 uncosted/isolated nodes:
    ///   ethercat_master, friction_compensation, joint_torque_controller,
    ///   manipulation_planner, safety_monitor, swing_leg_planner — pinned);
    /// * the trigger-edge structure transcribed from the humanoid bench's
    ///   node crates (`#[input(trigger)]`
    ///   fields), wired through its `humanoid.yaml`
    ///   (which topic feeds which input). Latest-value (non-`trigger`) reads
    ///   are NOT DAG edges, so they are omitted: tf_broadcaster /
    ///   robot_state_publisher (both read fbe latest-value only ⇒ L0 sources),
    ///   manipulation_planner (its only trigger `manip_goal` is an external
    ///   absolute topic with no in-graph producer ⇒ an L0 source), and the
    ///   `#[input]` reads on wbc / footstep_planner / centroidal_mpc.
    ///
    /// THE DEFECT this pins (a verb that moves ONLY `diagnostics`): a
    /// chain rate of `max(gating, incoming)` inflates a Sync JOIN's rate
    /// with its fast incoming edge. `elevation_mapping` is a 100 ms Sync join
    /// that INGESTS the 611 Hz `fbe` stream but fires at ~10 Hz (its outgoing
    /// `elev->terrain` edge = 10044 mHz); under that formula its chain rate reads the
    /// incoming `fbe->elev` 610867. `head_depth` (10 Hz) then cannot vacate L0
    /// because its cascade drags `point_cloud_filter -> elevation_mapping`,
    /// and NO-FASTER-DRAG sees the inflated 610867 > head_depth's 10300 and
    /// refuses the cascade atomically. `head_depth` stays the L0 max cost
    /// (23649), so the cams fail MATERIALITY forever and `imu_filter`'s
    /// (L1) L0-span keeps paying the camera costs.
    ///
    /// THE RULE (sink-only incoming inheritance): a non-sink's chain rate is
    /// its OWN gating rate, so `elevation_mapping`'s is 10044, not 610867;
    /// head_depth's cascade is accepted and the cams vacate L0.
    /// This fixture would be a NO-OP under the deep-spine oracle
    /// (`refine_dense_pipeline_..._real_humanoid_shape`) because that shape's
    /// uniform-rate chains have incoming == gating everywhere — this test's
    /// distinguishing feature is the fast-incoming-edge-on-a-slow-Sync-join.
    #[test]
    fn refine_real_humanoid_artifact_cams_and_depth_vacate_l0() {
        let (topo, te, base, inputs, trig_edges) = real_humanoid_fixture();

        // Kahn base (the pre-refinement dense pipeline): the cams + head_depth
        // sit AT L0; the deep 1 kHz spine ends at ethercat_master @L6 (7
        // levels). L0 is dominated by the camera costs.
        assert_eq!(base.level_of("head_cam_left"), Some(0));
        assert_eq!(base.level_of("head_cam_right"), Some(0));
        assert_eq!(base.level_of("head_depth"), Some(0));
        assert_eq!(base.level_of("imu_filter"), Some(1));
        assert_eq!(base.level_of("floating_base_estimator"), Some(1));
        assert_eq!(base.level_of("ethercat_master"), Some(6));
        assert_eq!(base.len(), 7, "Kahn: L0..L6");

        let refined = topo.refine_levels(&base, &te, &inputs);

        // ── THE DECISIVE PROPERTY (under the inflated rate all three stay at L0). ──
        // The cams + head_depth vacate L0 and land AT L1 — the correct
        // MATERIALITY halt. In cost-desc order diagnostics (27873) moves
        // first, then head_depth (23649, now the L0 max) then each cam
        // becomes material and cascades +1, but at L1 floating_base_estimator
        // (cost 32641) strictly dominates every one of them (23649 / 20864 /
        // 19552 all < 32641), so each halts at L1. Landing them at L1 is
        // latency-NEUTRAL for the deep 1 kHz spine (fbe already owns L1's
        // makespan) and strictly frees L0 for imu_filter's waiting span.
        assert_eq!(refined.level_of("head_depth"), Some(1), "head_depth off L0");
        assert_eq!(
            refined.level_of("head_cam_left"),
            Some(1),
            "cam_left off L0"
        );
        assert_eq!(
            refined.level_of("head_cam_right"),
            Some(1),
            "cam_right off L0"
        );

        // L0 now holds ONLY the cheap sources (graph order); its makespan
        // collapsed from head_depth's 23649 to foot_ft_right's 9024.
        assert_eq!(
            refined.level(0).unwrap().nodes,
            vec![
                "joint_encoder_reader".to_string(),
                "base_imu".to_string(),
                "foot_ft_left".to_string(),
                "foot_ft_right".to_string(),
                "wrist_ft_left".to_string(),
                "wrist_ft_right".to_string(),
                "foot_contact".to_string(),
                "manipulation_planner".to_string(),
                "tf_broadcaster".to_string(),
            ],
            "L0 = cheap sources only (cams + head_depth + diagnostics + rsp gone)"
        );
        let pre_l0 = level_cost(&base, &inputs, 0);
        let post_l0 = level_cost(&refined, &inputs, 0);
        // Pre: every L0 node's cost (cams dominate). Post: the cheap sources
        // (manipulation_planner is uncosted and contributes nothing).
        assert_eq!(
            post_l0,
            8769 + 8384 + 8576 + 9024 + 6592 + 6785 + 8097 + 7328
        );
        assert!(
            post_l0 < pre_l0,
            "L0 summed cost dropped: post {post_l0} < pre {pre_l0}"
        );
        // The max L0 node cost (the parallel-fire makespan) fell 23649 -> 9024.
        let max_l0 = |lv: &Levels| {
            lv.level(0)
                .unwrap()
                .nodes
                .iter()
                .map(|n| inputs.node_cost_ns.get(n).copied().unwrap_or(0))
                .max()
                .unwrap()
        };
        assert_eq!(
            max_l0(&base),
            27873,
            "pre: diagnostics owns L0 (head_depth 23649 is next)"
        );
        assert_eq!(max_l0(&refined), 9024, "post: foot_ft_right owns L0");

        // The PROTECTED 1 kHz chains never move (relative protection). The
        // deep spine (base_imu -> fbe -> wbc -> jtc -> friction -> safety ->
        // ethercat) stays byte-identical to Kahn; imu_filter's shallow sink
        // stays at L1. (jtc/friction/safety/ethercat are uncosted ⇒ pinned.)
        for (n, l) in [
            ("base_imu", 0),
            ("floating_base_estimator", 1),
            ("whole_body_controller", 2),
            ("joint_torque_controller", 3),
            ("friction_compensation", 4),
            ("safety_monitor", 5),
            ("ethercat_master", 6),
            ("imu_filter", 1),
        ] {
            assert_eq!(refined.level_of(n), Some(l), "1 kHz-adjacent {n} unmoved");
        }

        // The two co-located isolated costed sinks resolve by materiality:
        // diagnostics (27873) and robot_state_publisher (9345 > foot_ft_right
        // 9024) each dominate L0 in turn and move to L1 (halted by fbe);
        // tf_broadcaster (7328 < 9024) never dominates ⇒ stays L0.
        assert_eq!(refined.level_of("diagnostics"), Some(1));
        assert_eq!(refined.level_of("robot_state_publisher"), Some(1));
        assert_eq!(refined.level_of("tf_broadcaster"), Some(0));

        // The camera + depth chains cascade DOWN with their roots (the whole
        // dense chain slides), every hand-derived halt justified:
        //  - image_rectify_{l,r} @L2 (one below their L1 cams);
        //  - vision @L3, stereo @L3 (Sync join of both rectifies);
        //  - object_tracker @L4 (vision's sink, chain 30338);
        //  - point_cloud_filter @L3 (dragged to L2 by head_depth, then its
        //    own cascade to L3, halted by stereo 22401 > pcf 15168 at L3);
        //  - elevation_mapping @L4 (halts AT object_tracker @L4 — the deepest
        //    faster non-downstream sink; its own chain rate is 10044, NOT the
        //    inflated 610867 — the load-bearing assertion);
        //  - terrain @L5, footstep @L6, swing @L7 (the slow chain's tail,
        //    each with no faster sink above ⇒ dragged not marched).
        for (n, l) in [
            ("image_rectify_left", 2),
            ("image_rectify_right", 2),
            ("vision_detection", 3),
            ("stereo_disparity", 3),
            ("object_tracker", 4),
            ("point_cloud_filter", 3),
            ("elevation_mapping", 4),
            ("terrain_traversability", 5),
            ("footstep_planner", 6),
            ("swing_leg_planner", 7),
        ] {
            assert_eq!(refined.level_of(n), Some(l), "perception chain {n}");
        }
        // The fbe fan-out consumers stay at L2 (fbe @L1 + 1); the join
        // jtc rises to L3 (max(wbc@L2, fbe@L1) + 1).
        for (n, l) in [
            ("centroidal_mpc", 2),
            ("com_estimator", 2),
            ("gravity_compensation", 2),
            ("joint_limit_monitor", 2),
            ("contact_estimator", 1),
        ] {
            assert_eq!(refined.level_of(n), Some(l), "spine fan-out {n}");
        }

        // The count GREW 7 -> 8 (the slow depth/terrain chain slid down two
        // levels, appending L7); all invariants hold; deterministic.
        assert_eq!(refined.len(), 8, "level count grew 7 -> 8");
        assert!(refined.len() >= base.len(), "cascades only ever add levels");
        assert_edges_increasing(&refined, trig_edges);
        assert!(
            refined.iter().all(|l| !l.nodes.is_empty()),
            "no empty level"
        );
        let total: usize = refined.iter().map(|l| l.nodes.len()).sum();
        assert_eq!(total, 36, "every node partitioned exactly once");
        let again = topo.refine_levels(&base, &te, &inputs);
        assert_eq!(refined, again, "deterministic across runs");
    }

    // ================= Shape-gated level growth =================
    //
    // `LevelGrowth::Deny` refuses ANY phase-2 cascade whose target-level set
    // would reach beyond the Kahn input's max level (`base.len() - 1`), i.e.
    // would append a level. It kills GROWTH, not cascading: a cascade whose
    // targets all land within the original `0..base.len()` range still
    // applies. The oracles below hand-derive the Deny fixed point on the two
    // cost-refinement fixtures and a minimal positive control; `Allow` (the default)
    // is byte-stable with the earlier output.

    /// (a) DENY on the DEEP-SPINE fixture. The correct Deny output here is an
    /// INTERMEDIATE — NOT the Kahn base and NOT the grown Allow output — because
    /// the gate is NOT "refuse everything": this fixture mixes chains of
    /// DIFFERENT depths, so some cascades are NON-growing and STILL apply while
    /// the growing ones are refused. Count stays 4; the 1 kHz spine
    /// (base_imu/fbe/wbc/jtc) + imu_filter never move (max-rate protection).
    /// The move-by-move derivation:
    ///
    ///   * `elev` (the 80400 sink @L2) has a faster non-downstream sink above
    ///     it (`jtc` @L3, 1 kHz): it marches L2→L3 — a NON-growing cascade
    ///     ({elev:3}, max 3 = level_count-1) that leaves the 1 kHz spine's
    ///     waiting span, so it is APPLIED;
    ///   * `cam_d` (23600, the L0 max after `elev` moves) is material and
    ///     cascades L0→L1 dragging `pcf` L1→L2 ({cam_d:1, pcf:2}, max 2) — its
    ///     chain is only depth-2 (cam_d→pcf→elev, sink @L3) so it fits WITHIN
    ///     the count. APPLIED;
    ///   * the LONGER cam chains `cam_l`/`cam_r` (→rect→stereo→vision, sink
    ///     @L3) CANNOT slide without pushing `vision` past L3 to L4 ({cam_?:1,
    ///     rect_?:2, stereo:3, vision:4}, max 4 ≥ 4) — REFUSED, so they stay
    ///     at L0 (`stereo`'s own march is refused for the same reason).
    #[test]
    fn refine_deny_growth_on_deep_spine_applies_only_non_growing_cascades() {
        let (topo, te, base, default_inputs, edge_pairs) = deep_spine_fixture();
        assert_eq!(base.len(), 4, "Kahn: L0..L3");
        let mut inputs = default_inputs;
        inputs.level_growth = LevelGrowth::Deny;
        let refined = topo.refine_levels(&base, &te, &inputs);
        assert_eq!(
            level_vecs(&refined),
            vec![
                vec![
                    "base_imu".to_string(),
                    "cam_l".to_string(),
                    "cam_r".to_string()
                ],
                vec![
                    "imu_filter".to_string(),
                    "fbe".to_string(),
                    "cam_d".to_string(),
                    "rect_l".to_string(),
                    "rect_r".to_string(),
                ],
                vec!["wbc".to_string(), "pcf".to_string(), "stereo".to_string()],
                vec!["jtc".to_string(), "vision".to_string(), "elev".to_string()],
            ],
            "Deny intermediate: elev L2→L3, cam_d L0→L1 + pcf L1→L2; \
             cam_l/cam_r/stereo cascades refused (would append L4)"
        );
        assert_eq!(refined.len(), 4, "count NEVER grows under Deny");
        // The 1 kHz spine + shallow sink are byte-unmoved (max-rate protection).
        for (n, l) in [
            ("base_imu", 0),
            ("imu_filter", 1),
            ("fbe", 1),
            ("wbc", 2),
            ("jtc", 3),
        ] {
            assert_eq!(refined.level_of(n), Some(l), "1 kHz node {n} unmoved");
        }
        // The refused-growth chains stay at Kahn: cam_l/cam_r @L0, vision @L3.
        assert_eq!(refined.level_of("cam_l"), Some(0), "cam_l cascade refused");
        assert_eq!(refined.level_of("cam_r"), Some(0), "cam_r cascade refused");
        assert_eq!(
            refined.level_of("vision"),
            Some(3),
            "vision not dragged past L3"
        );
        assert_edges_increasing(&refined, edge_pairs);
        assert!(refined.iter().all(|l| !l.nodes.is_empty()), "contiguous");
        let again = topo.refine_levels(&base, &te, &inputs);
        assert_eq!(refined, again, "deterministic across runs");
    }

    /// (b) DENY on the REAL HUMANOID artifact. Contra a naive "every cascade
    /// grows so nothing moves" expectation, MANY of the moves are
    /// NON-growing and still apply under Deny — only the DEEPEST chain's final
    /// slide (which would append L7) is refused. Net: Deny == Allow EXCEPT the
    /// depth/terrain chain is one level shallower and the count stays 7 (not
    /// Allow's 8); the protected 1 kHz chains never move. The derivation:
    ///
    ///   * every off-L0 relocation is a +1 move to L1 (diagnostics, head_depth,
    ///     the two cams, robot_state_publisher) — NON-growing (target ≤ 6), so
    ///     L0 collapses to EXACTLY the Allow L0 (cheap sources only). The cams
    ///     do NOT stay at L0;
    ///   * the perception fan-out (image_rectify_* @L2, vision/stereo @L3,
    ///     object_tracker @L4) matches Allow — all within the count;
    ///   * the DEPTH/TERRAIN chain is where Deny bites. Under Allow
    ///     `elevation_mapping` marches +2 (L2→L4), dragging
    ///     terrain→footstep→swing until `swing` lands at L7 (the growth). Under
    ///     Deny the FIRST +1 (elev L2→L3, dragging terrain→footstep→swing to
    ///     L4/L5/L6, max 6) is NON-growing and APPLIED, but the SECOND +1
    ///     (which pushes swing to L7) is REFUSED. So pcf/elev/terrain/footstep/
    ///     swing each sit ONE level LOWER than Allow, and the count stays 7
    ///     (not Allow's 8). `pcf`'s own cascade is refused throughout (it would
    ///     also push swing to L7).
    #[test]
    fn refine_deny_growth_on_real_humanoid_keeps_count_but_shortens_depth_chain() {
        let (topo, te, base, default_inputs, trig_edges) = real_humanoid_fixture();
        assert_eq!(base.len(), 7, "Kahn: L0..L6");
        let mut inputs = default_inputs;
        inputs.level_growth = LevelGrowth::Deny;
        let refined = topo.refine_levels(&base, &te, &inputs);

        assert_eq!(
            level_vecs(&refined),
            vec![
                vec![
                    "joint_encoder_reader".to_string(),
                    "base_imu".to_string(),
                    "foot_ft_left".to_string(),
                    "foot_ft_right".to_string(),
                    "wrist_ft_left".to_string(),
                    "wrist_ft_right".to_string(),
                    "foot_contact".to_string(),
                    "manipulation_planner".to_string(),
                    "tf_broadcaster".to_string(),
                ],
                vec![
                    "head_cam_left".to_string(),
                    "head_cam_right".to_string(),
                    "head_depth".to_string(),
                    "robot_state_publisher".to_string(),
                    "diagnostics".to_string(),
                    "floating_base_estimator".to_string(),
                    "contact_estimator".to_string(),
                    "imu_filter".to_string(),
                ],
                vec![
                    "image_rectify_left".to_string(),
                    "image_rectify_right".to_string(),
                    "point_cloud_filter".to_string(),
                    "centroidal_mpc".to_string(),
                    "whole_body_controller".to_string(),
                    "com_estimator".to_string(),
                    "gravity_compensation".to_string(),
                    "joint_limit_monitor".to_string(),
                ],
                vec![
                    "vision_detection".to_string(),
                    "stereo_disparity".to_string(),
                    "elevation_mapping".to_string(),
                    "joint_torque_controller".to_string(),
                ],
                vec![
                    "object_tracker".to_string(),
                    "terrain_traversability".to_string(),
                    "friction_compensation".to_string(),
                ],
                vec!["footstep_planner".to_string(), "safety_monitor".to_string()],
                vec![
                    "swing_leg_planner".to_string(),
                    "ethercat_master".to_string()
                ],
            ],
            "Deny: cams/depth vacate L0 (non-growing) but the depth chain \
             stops one level short of Allow; count stays 7"
        );
        assert_eq!(
            refined.len(),
            7,
            "count NEVER grows under Deny (Allow grows 7→8)"
        );

        // The cams + head_depth STILL vacate L0 (these moves are non-growing).
        for n in ["head_cam_left", "head_cam_right", "head_depth"] {
            assert_eq!(refined.level_of(n), Some(1), "{n} still off L0 under Deny");
        }
        // The DEPTH/TERRAIN chain sits exactly ONE level below its Allow
        // position (Allow: pcf@3 elev@4 terrain@5 footstep@6 swing@7).
        for (n, l) in [
            ("point_cloud_filter", 2),
            ("elevation_mapping", 3),
            ("terrain_traversability", 4),
            ("footstep_planner", 5),
            ("swing_leg_planner", 6),
        ] {
            assert_eq!(
                refined.level_of(n),
                Some(l),
                "depth chain {n} one level below Allow"
            );
        }
        // The protected 1 kHz chain never moves.
        for (n, l) in [
            ("base_imu", 0),
            ("floating_base_estimator", 1),
            ("whole_body_controller", 2),
            ("joint_torque_controller", 3),
            ("ethercat_master", 6),
            ("imu_filter", 1),
        ] {
            assert_eq!(refined.level_of(n), Some(l), "1 kHz-adjacent {n} unmoved");
        }
        assert_edges_increasing(&refined, trig_edges);
        assert!(
            refined.iter().all(|l| !l.nodes.is_empty()),
            "no empty level"
        );
        let total: usize = refined.iter().map(|l| l.nodes.len()).sum();
        assert_eq!(total, 36, "every node partitioned exactly once");
        let again = topo.refine_levels(&base, &te, &inputs);
        assert_eq!(refined, again, "deterministic across runs");
    }

    /// (c) The ANTI-REGRESSION twin: `LevelGrowth::Allow` is the DEFAULT, so
    /// explicitly setting it must be byte-identical to the default construction
    /// on BOTH fixtures: adding the field did not perturb the pre-gate path.
    /// (The default outputs themselves are the full grown blocks pinned by the
    /// two Allow oracles above.)
    #[test]
    fn refine_explicit_allow_equals_default_on_both_fixtures() {
        let (topo, te, base, default_inputs, _) = deep_spine_fixture();
        let mut allow = default_inputs.clone();
        allow.level_growth = LevelGrowth::Allow;
        assert_eq!(
            topo.refine_levels(&base, &te, &default_inputs),
            topo.refine_levels(&base, &te, &allow),
            "deep spine: explicit Allow == default"
        );

        let (topo, te, base, default_inputs, _) = real_humanoid_fixture();
        let mut allow = default_inputs.clone();
        allow.level_growth = LevelGrowth::Allow;
        assert_eq!(
            topo.refine_levels(&base, &te, &default_inputs),
            topo.refine_levels(&base, &te, &allow),
            "humanoid: explicit Allow == default"
        );
    }

    /// (d) POSITIVE CONTROL: a minimal fixture where Deny APPLIES a real
    /// (multi-node) non-growing cascade — proving the gate is NOT "refuse all
    /// cascades". A fast 1 kHz chain `f0→f1→f2→f3` (sink @L3, count 4) gives a
    /// slow expensive root `s0` (chain `s0→s1`, 30 Hz) a faster non-downstream
    /// sink to march toward. Under Deny `s0` marches L0→L1→L2 (two non-growing
    /// steps, dragging `s1` to L2 then L3, all within `0..3`), then the third
    /// step (which would push `s1` to L4) is REFUSED — so `s0` halts at L2,
    /// count 4. Under Allow `s0` marches to the sink's level L3, `s1` appends
    /// L4, count grows 4→5. Hand oracle, not a self-compare.
    #[test]
    fn refine_deny_applies_a_non_growing_multi_node_cascade_within_the_count() {
        let cfg = config(
            "p",
            vec![
                node("f0", &[], &["out"]),
                node("f1", &[("i", "f0/out")], &["out"]),
                node("f2", &[("i", "f1/out")], &["out"]),
                node("f3", &[("i", "f2/out")], &[]),
                node("s0", &[], &["out"]),
                node("s1", &[("i", "s0/out")], &[]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![
                ("f0", vec![]),
                ("f1", vec![]),
                ("f2", vec![]),
                ("f3", vec![]),
                ("s0", vec![]),
                ("s1", vec![]),
            ]),
        )
        .expect("build");
        let edges = trig(&[
            ("f1", "/p/f0/out"),
            ("f2", "/p/f1/out"),
            ("f3", "/p/f2/out"),
            ("s1", "/p/s0/out"),
        ]);
        let base = topo.derive_levels(&edges).expect("acyclic");
        assert_eq!(base.len(), 4, "Kahn: f-chain L0..L3, s-chain L0..L1");
        let inputs = refine_in(
            &[
                ("f0", 10),
                ("f1", 10),
                ("f2", 10),
                ("f3", 10),
                ("s0", 100),
                ("s1", 1),
            ],
            &[
                ("f0", "f1", 1_000_000),
                ("f1", "f2", 1_000_000),
                ("f2", "f3", 1_000_000),
                ("s0", "s1", 30_000),
            ],
        );

        // Deny: the non-growing part of s0's march is APPLIED (s0 L0→L2, s1
        // dragged to L3, all within the original count 4); the growing step is
        // refused. f-chain never moves (max-rate protection).
        let mut deny = inputs.clone();
        deny.level_growth = LevelGrowth::Deny;
        let refined_deny = topo.refine_levels(&base, &edges, &deny);
        assert_eq!(
            level_vecs(&refined_deny),
            vec![
                vec!["f0".to_string()],
                vec!["f1".to_string()],
                vec!["f2".to_string(), "s0".to_string()],
                vec!["f3".to_string(), "s1".to_string()],
            ],
            "s0 cascaded L0→L2 (WITHIN the count); s1 dragged to L3; count 4"
        );
        assert_eq!(refined_deny.len(), 4, "no growth");
        assert_eq!(
            refined_deny.level_of("s0"),
            Some(2),
            "the applied non-growing cascade moved s0 two levels"
        );

        // Allow (contrast): s0 marches to the fast sink's level L3, s1 appends
        // L4, count grows 4→5 — proving the gate is load-bearing.
        let refined_allow = topo.refine_levels(&base, &edges, &inputs);
        assert_eq!(
            refined_allow.level_of("s0"),
            Some(3),
            "Allow reaches the sink's level"
        );
        assert_eq!(refined_allow.level_of("s1"), Some(4), "Allow appends L4");
        assert_eq!(refined_allow.len(), 5, "Allow grows 4→5");
        assert_ne!(
            refined_deny, refined_allow,
            "the growth gate is load-bearing"
        );
    }

    // ============ level_assignments yaml override ============

    /// Build an assignments map from `(node, level)` literals, preserving the
    /// given (yaml-authored) key order.
    fn assign(pairs: &[(&str, usize)]) -> IndexMap<String, usize> {
        pairs.iter().map(|(n, l)| ((*n).to_string(), *l)).collect()
    }

    /// A 4-node fixture with slack: `a -> b -> d` plus root `x` whose only
    /// consumer is `d` (so `x` can legally sit at level 0 OR 1). Kahn ASAP:
    /// L0 = [a, x], L1 = [b], L2 = [d]. Config order `[a, x, b, d]`.
    fn slack_fixture() -> (GraphConfig, GraphTopology, TriggerEdges) {
        let cfg = config(
            "p",
            vec![
                node("a", &[], &["out"]),
                node("x", &[], &["out"]),
                node("b", &[("i", "a/out")], &["out"]),
                node("d", &[("i", "b/out"), ("j", "x/out")], &[]),
            ],
        );
        let topo = GraphTopology::build(
            &cfg,
            &infos(vec![
                ("a", vec![]),
                ("x", vec![]),
                ("b", vec![]),
                ("d", vec![]),
            ]),
        )
        .expect("build");
        let edges = trig(&[("b", "/p/a/out"), ("d", "/p/b/out"), ("d", "/p/x/out")]);
        (cfg, topo, edges)
    }

    #[test]
    fn assignments_unknown_node_rejected() {
        let (_, topo, edges) = slack_fixture();
        let a = assign(&[("a", 0), ("x", 0), ("b", 1), ("d", 2), ("ghost", 1)]);
        let err = topo
            .levels_from_assignments(&a, &edges, "g")
            .expect_err("unknown node must be rejected");
        let msg = format!("{err}");
        assert!(msg.contains("graph 'g'"), "names the graph: {msg}");
        assert!(msg.contains("unknown node(s) [ghost]"), "names it: {msg}");
        assert!(msg.contains("level_assignments"), "names the block: {msg}");
    }

    #[test]
    fn assignments_partial_coverage_rejected() {
        let (_, topo, edges) = slack_fixture();
        // `x` and `d` hand-deleted — BOTH named, in graph order.
        let a = assign(&[("a", 0), ("b", 1)]);
        let err = topo
            .levels_from_assignments(&a, &edges, "g")
            .expect_err("partial coverage must be rejected");
        let msg = format!("{err}");
        assert!(msg.contains("must cover EVERY node"), "got: {msg}");
        assert!(
            msg.contains("missing [x, d]"),
            "graph-order offenders: {msg}"
        );
        assert!(
            msg.contains("delete the whole `level_assignments:` block"),
            "remedy names the Kahn fallback: {msg}"
        );
    }

    #[test]
    fn assignments_edge_decreasing_rejected() {
        let (_, topo, edges) = slack_fixture();
        // b (consumer of a) at the SAME level as a → not strictly increasing.
        let same = assign(&[("a", 0), ("x", 0), ("b", 0), ("d", 1)]);
        let err = topo
            .levels_from_assignments(&same, &edges, "g")
            .expect_err("equal-level trigger edge must be rejected");
        let msg = format!("{err}");
        assert!(
            msg.contains("'a' (level 0) -> 'b' (level 0)"),
            "names the edge + levels: {msg}"
        );
        assert!(msg.contains("not strictly level-increasing"), "got: {msg}");

        // INVERTED: producer above consumer.
        let inverted = assign(&[("a", 1), ("x", 0), ("b", 0), ("d", 2)]);
        let err2 = topo
            .levels_from_assignments(&inverted, &edges, "g")
            .expect_err("inverted trigger edge must be rejected");
        assert!(
            format!("{err2}").contains("'a' (level 1) -> 'b' (level 0)"),
            "got: {err2}"
        );
    }

    #[test]
    fn assignments_gapped_range_rejected() {
        let (_, topo, edges) = slack_fixture();
        // Level 1 left empty: a,x@0 then b@2, d@3.
        let a = assign(&[("a", 0), ("x", 0), ("b", 2), ("d", 3)]);
        let err = topo
            .levels_from_assignments(&a, &edges, "g")
            .expect_err("gapped level range must be rejected");
        let msg = format!("{err}");
        assert!(msg.contains("level 1 is EMPTY"), "names the gap: {msg}");
        assert!(
            msg.contains("barrier advances one generation per level"),
            "explains the WHY: {msg}"
        );
        assert!(msg.contains("renumber"), "remedy: {msg}");
    }

    #[test]
    fn assignments_level_out_of_range_rejected() {
        let (_, topo, edges) = slack_fixture();
        // 4 nodes ⇒ max legal level 3. Also guards the huge-alloc DoS shape.
        let a = assign(&[("a", 0), ("x", 0), ("b", 1), ("d", 999_999_999)]);
        let err = topo
            .levels_from_assignments(&a, &edges, "g")
            .expect_err("out-of-range level must be rejected before allocation");
        let msg = format!("{err}");
        assert!(
            msg.contains("node 'd'") && msg.contains("999999999"),
            "names node + level: {msg}"
        );
        assert!(
            msg.contains("at most levels 0..=3"),
            "names the bound: {msg}"
        );
    }

    #[test]
    fn assignments_valid_non_kahn_matches_hand_oracle() {
        let (_, topo, edges) = slack_fixture();
        // Non-Kahn: x (ASAP 0) hand-delayed to level 1 — the expensive-node-
        // delayed shape. Map written in REVERSE graph order to prove
        // within-level order comes from the graph, never the map.
        let a = assign(&[("d", 2), ("b", 1), ("x", 1), ("a", 0)]);
        let levels = topo
            .levels_from_assignments(&a, &edges, "g")
            .expect("valid assignment");
        assert_eq!(
            level_vecs(&levels),
            vec![
                vec!["a".to_string()],
                // graph order [a, x, b, d] ⇒ x before b at level 1, even
                // though the map lists b first.
                vec!["x".to_string(), "b".to_string()],
                vec!["d".to_string()],
            ],
            "levels match the assignment, within-level order is GRAPH order"
        );
        assert_eq!(levels.level_of("x"), Some(1));
        assert_eq!(levels.len(), 3);
        // And it genuinely differs from Kahn (non-Kahn assignment).
        let kahn = topo.derive_levels(&edges).expect("acyclic");
        assert_ne!(levels, kahn, "the override is a real non-Kahn shape");
    }

    #[test]
    fn assignments_sink_may_be_deliberately_delayed() {
        // Contrast with refine_levels' sink pin: the yaml path enforces ONLY
        // the hard invariants — a user may delay a sink. `x`'s consumer `d`
        // is a sink at Kahn L2; a 4-level assignment delaying sink `d` to L3
        // (with x at 2) is valid.
        let (_, topo, edges) = slack_fixture();
        let a = assign(&[("a", 0), ("x", 2), ("b", 1), ("d", 3)]);
        let levels = topo
            .levels_from_assignments(&a, &edges, "g")
            .expect("a delayed sink is legal in a hand-written assignment");
        assert_eq!(levels.level_of("d"), Some(3), "sink deliberately delayed");
        assert_eq!(levels.len(), 4);
    }

    #[test]
    fn assignments_deterministic_and_empty_graph_edge_cases() {
        let (_, topo, edges) = slack_fixture();
        let a = assign(&[("a", 0), ("x", 1), ("b", 1), ("d", 2)]);
        let one = topo.levels_from_assignments(&a, &edges, "g").expect("ok");
        let two = topo.levels_from_assignments(&a, &edges, "g").expect("ok");
        assert_eq!(one, two, "same inputs ⇒ byte-identical Levels");

        // Empty graph + empty map ⇒ empty Levels (vacuously valid).
        let empty_cfg = config("p", vec![]);
        let empty_topo = GraphTopology::build(&empty_cfg, &infos(vec![])).expect("build");
        let empty = empty_topo
            .levels_from_assignments(&assign(&[]), &TriggerEdges::new(), "g")
            .expect("empty graph + empty map is vacuously valid");
        assert!(empty.is_empty());

        // Empty map + NON-empty graph ⇒ the coverage error.
        let err = topo
            .levels_from_assignments(&assign(&[]), &edges, "g")
            .expect_err("empty block on a non-empty graph is partial coverage");
        assert!(format!("{err}").contains("must cover EVERY node"));
    }

    #[test]
    fn shared_invariant_validator_vectors() {
        let (_, topo, edges) = slack_fixture();
        let mk = |vecs: &[&[&str]]| -> (Vec<Level>, IndexMap<String, usize>) {
            let levels: Vec<Level> = vecs
                .iter()
                .map(|ns| Level {
                    nodes: ns.iter().map(|s| s.to_string()).collect(),
                })
                .collect();
            let mut rank = IndexMap::new();
            for (i, l) in levels.iter().enumerate() {
                for n in &l.nodes {
                    rank.insert(n.clone(), i);
                }
            }
            (levels, rank)
        };

        // PASS: the Kahn shape.
        let (lv, rk) = mk(&[&["a", "x"], &["b"], &["d"]]);
        assert_eq!(topo.level_invariant_violation(&lv, &rk, &edges), None);
        // PASS: the delayed-x shape.
        let (lv, rk) = mk(&[&["a"], &["x", "b"], &["d"]]);
        assert_eq!(topo.level_invariant_violation(&lv, &rk, &edges), None);
        // FAIL: empty level.
        let (lv, rk) = mk(&[&["a", "x"], &[], &["b"], &["d"]]);
        let v = topo
            .level_invariant_violation(&lv, &rk, &edges)
            .expect("empty level is a violation");
        assert!(v.contains("level 1 is EMPTY"), "got: {v}");
        // FAIL: equal-level trigger edge (a,b co-located).
        let (lv, rk) = mk(&[&["a", "x", "b"], &["d"]]);
        let v = topo
            .level_invariant_violation(&lv, &rk, &edges)
            .expect("co-located trigger edge is a violation");
        assert!(v.contains("not strictly level-increasing"), "got: {v}");
        // FAIL: inverted trigger edge.
        let (lv, rk) = mk(&[&["b", "x"], &["a"], &["d"]]);
        let v = topo
            .level_invariant_violation(&lv, &rk, &edges)
            .expect("inverted trigger edge is a violation");
        assert!(v.contains("'a' (level 1) -> 'b' (level 0)"), "got: {v}");
        // FAIL: a node missing from rank entirely.
        let (lv, mut rk) = mk(&[&["a", "x"], &["b"], &["d"]]);
        rk.shift_remove("b");
        let v = topo
            .level_invariant_violation(&lv, &rk, &edges)
            .expect("unranked edge endpoint is a violation");
        assert!(v.contains("no assigned level"), "got: {v}");
    }

    #[test]
    fn resolve_levels_absent_is_byte_identical_to_kahn() {
        let (cfg, topo, edges) = slack_fixture();
        assert!(cfg.level_assignments.is_none(), "fixture carries no block");
        let via_seam = resolve_levels(&cfg, &topo, &edges).expect("resolves");
        let kahn = topo.derive_levels(&edges).expect("acyclic");
        assert_eq!(via_seam, kahn, "absent block ⇒ Kahn, byte-identical");
    }

    #[test]
    fn resolve_levels_present_routes_to_the_assignment() {
        let (mut cfg, topo, edges) = slack_fixture();
        cfg.level_assignments = Some(assign(&[("a", 0), ("x", 1), ("b", 1), ("d", 2)]));
        let via_seam = resolve_levels(&cfg, &topo, &edges).expect("resolves");
        let direct = topo
            .levels_from_assignments(cfg.level_assignments.as_ref().unwrap(), &edges, "g")
            .expect("valid");
        assert_eq!(via_seam, direct, "the seam routes to the assignment path");
        assert_eq!(via_seam.level_of("x"), Some(1), "the override is live");
        // Determinism through the seam.
        let again = resolve_levels(&cfg, &topo, &edges).expect("resolves");
        assert_eq!(via_seam, again);
        // And an INVALID block through the seam is the loud Err.
        cfg.level_assignments = Some(assign(&[("a", 0), ("x", 0), ("b", 0), ("d", 1)]));
        let err = resolve_levels(&cfg, &topo, &edges).expect_err("invalid block rejects");
        assert!(format!("{err}").contains("not strictly level-increasing"));
    }

    #[test]
    fn resolve_levels_absent_cycle_keeps_the_rich_diagnostic() {
        // a triggers on b's output and b on a's — an algebraic loop. The
        // rich "cannot be scheduled" wording moved from runtime.rs into
        // resolve_levels; pin it survives.
        let cfg = config(
            "p",
            vec![
                node("a", &[("i", "b/out")], &["out"]),
                node("b", &[("i", "a/out")], &["out"]),
            ],
        );
        let topo =
            GraphTopology::build(&cfg, &infos(vec![("a", vec![]), ("b", vec![])])).expect("build");
        let edges = trig(&[("a", "/p/b/out"), ("b", "/p/a/out")]);
        let err = resolve_levels(&cfg, &topo, &edges).expect_err("cycle rejects");
        let msg = format!("{err}");
        assert!(msg.contains("cannot be scheduled"), "got: {msg}");
        assert!(msg.contains("algebraic trigger cycle"), "got: {msg}");
    }

    #[test]
    fn coverage_helper_reports_unknown_before_missing() {
        // The shared config-only helper: unknown keys win (a typo usually IS
        // the missing node), each class reported with ALL its offenders.
        let ids = ["a", "b", "c"];
        let both = assign(&[("a", 0), ("ghost", 1), ("phantom", 2)]);
        let v = level_assignment_coverage_violation(&ids, &both).expect("violation");
        assert!(v.contains("unknown node(s) [ghost, phantom]"), "got: {v}");
        let missing_only = assign(&[("a", 0)]);
        let v = level_assignment_coverage_violation(&ids, &missing_only).expect("violation");
        assert!(v.contains("missing [b, c]"), "got: {v}");
        let full = assign(&[("a", 0), ("b", 0), ("c", 0)]);
        assert_eq!(level_assignment_coverage_violation(&ids, &full), None);
    }
}
