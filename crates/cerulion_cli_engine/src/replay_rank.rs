// SPDX-License-Identifier: AGPL-3.0-only
//! The PURE per-rank replay PLANNER.
//!
//! Replay runs a free-run multi-rank bag **bag-fed and SEQUENTIALLY per rank**:
//! one
//! `GraphRuntime` per rank, one at a time — no k-clock multiplexing, no k
//! `TransportManager`s in one process, and therefore no Principle-8 exemption.
//! This module is the planner that says WHAT each of those runtimes is: which
//! nodes it owns, which subgraph it builds, which topics it produces, which of
//! its reads cross a rank boundary (bag-fed injection targets), which are
//! external (the re-keyed `inject_up_to` targets), and which slice of the
//! recorded step axis it covers.
//!
//! # Pure by construction
//!
//! Nothing here touches transport, SHM, the bag reader, the clock, or the
//! filesystem: the inputs are a [`GraphConfig`] plus two plain-data tables the
//! caller reads off the bag ([`RankManifest`], [`RankBoundarySummary`]) and,
//! for [`classify_split_block_edges`], the build's node declarations and the
//! node to rank map its plans give, and the output is data. That is what lets the planner unit-test on any platform
//! (the module is deliberately NOT `#[cfg(unix)]`-gated, unlike its
//! `replay_engine` consumer — the `replay_field_registry` precedent) and what
//! keeps every refusal below reachable from a hand-built oracle.
//!
//! # Rank membership comes from the MANIFESTS, never from `process_groups:`
//!
//! A recording's per-rank trace manifests are the truth of what actually ran.
//! The bag's embedded `graph.yaml` need not carry a `process_groups:` block at
//! all — the auto-partitioner derives a partition IN MEMORY for an unpartitioned graph and
//! the never-mutate floor keeps the file untouched, so a perfectly ordinary
//! multi-rank bag can embed a group-less graph (the crafted mp fixture in
//! `replay_engine_test.rs` is exactly that shape). Planning off
//! `config.process_groups` would therefore silently plan ONE rank for a
//! recording that ran on three. The manifests decide; the graph only supplies
//! topology.
//!
//! # Lockstep is ONE rank plan
//!
//! A lockstep bag gets a single-runtime whole-graph replay, and it is
//! expressible in this same type: hand [`plan_ranks`] ONE [`RankManifest`]
//! naming every graph node and it returns one [`RankPlan`] whose `subgraph` is
//! the parent config VERBATIM. So the engine keeps one code path
//! across both coordination modes instead of branching at the top. The
//! whole-graph short-circuit is not an optimization: restricting a rank that
//! already owns every node would rename the graph (`{name}_rank0`, which feeds
//! replay's iceoryx2 node name) and drop its `network:` block, changing a
//! byte-exact lockstep replay for no gain. On the node/input dimension it is
//! provably an identity — `subgraph_for` only rewrites a relative source whose
//! resolved topic no member produces, and `validate_graph` already refuses a
//! relative source that resolves to nothing, so with every node in the group
//! there is nothing to rewrite.
//!
//! # Error class
//!
//! Every [`RankPlanError`] is a structural disagreement between the recording
//! and its embedded graph, or an internally inconsistent recording — the
//! `ReplayError::BagGraphMismatch` / `RecordingInconsistent` class (exit 2, bag
//! fault), never a candidate divergence (exit 1/6) and never a harness bug
//! (exit 5). The engine maps them onto that vocabulary; the enum is
//! kept free-standing here so the planner stays platform-independent and
//! oracle-testable. [`SplitBlockEdgeError`] maps otherwise: `Uncreditable` and
//! `Timeout` are exit 2, `EngineBug` is exit 5, and the replay refuses nothing
//! on `Topology`.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use cerulion_core::graph::config::InputBackpressure;
use cerulion_core::graph::{
    block_colocation_seeds, resolve_output_topic, resolve_source, GraphConfig, GraphTopology,
    NodeInfo, SiblingProducedTopics,
};
use indexmap::IndexMap;

use crate::multiprocess::{
    credit_edges_for, sibling_topics_for, split_edge_timeout_producers, subgraph_for,
    CreditEdgePlan,
};

/// One rank's recorded node table — the `node_ids` of its
/// `__cerulion/trace_manifest_rank{N}.json` attachment, plus the rank the
/// attachment NAME carries (which is authoritative; the manifest body's own
/// `rank` key is ignored by every reader in the tree).
///
/// The supervisor departure-ring sentinel manifest (rank `u32::MAX`)
/// is NOT a worker and must be filtered out by the caller before planning —
/// `replay_engine::load_rank_tables` already does exactly that at the seam this
/// planner is fed from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RankManifest {
    /// The worker rank this manifest describes (from the attachment name).
    pub rank: u32,
    /// The node ids this rank recorded, in ring-manifest order. Order is NOT
    /// consumed here — [`RankPlan::members`] is emitted in GRAPH order
    /// (Principle #5: the graph file is the source of truth) so the plan cannot
    /// inherit a ring-order accident.
    pub node_ids: Vec<String>,
}

/// A summary of one rank's STEP_BOUNDARY (kind 3) stream — the recorded step
/// slice this rank's runtime must be driven across.
///
/// The caller derives it from the same single trace walk `validate_step_
/// boundaries` already makes; the planner never re-walks a bag. Phase-1
/// validation guarantees a rank's step indices are strictly consecutive, so the
/// step COUNT is `last_step - first_step + 1` and is deliberately not carried
/// as a second, driftable field.
///
/// The fields are PRIVATE and reached through the accessors below. The step
/// range carries an ORDER invariant (`first_step <= last_step`)
/// that only [`plan_ranks`] enforces — as
/// [`RankPlanError::BoundaryRangeInverted`] — so a caller that assembled the
/// struct by hand could hand any consumer an inverted range and a step COUNT
/// that underflows. `plan_ranks` is the one builder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RankBoundarySummary {
    rank: u32,
    first_step: u64,
    last_step: u64,
    first_target_ns: u64,
}

impl RankBoundarySummary {
    /// One rank's boundary summary.
    ///
    /// Deliberately INFALLIBLE, and deliberately not the place the order
    /// invariant is checked: the caller reads these numbers off a bag, and a
    /// range it disagrees with is a BAG FAULT that must reach the operator as
    /// [`RankPlanError::BoundaryRangeInverted`] (naming the rank and both
    /// steps, exit 2) rather than as a constructor's `Err` some caller maps to
    /// something else. What privacy buys is that the ONLY way to reach a
    /// consumer is through `plan_ranks`, which asks.
    #[must_use]
    pub fn new(rank: u32, first_step: u64, last_step: u64, first_target_ns: u64) -> Self {
        Self {
            rank,
            first_step,
            last_step,
            first_target_ns,
        }
    }

    /// The rank whose boundary stream this summarizes.
    #[must_use]
    pub fn rank(&self) -> u32 {
        self.rank
    }

    /// The stream's first (lowest) recorded step index.
    #[must_use]
    pub fn first_step(&self) -> u64 {
        self.first_step
    }

    /// The stream's last (highest) recorded step index.
    #[must_use]
    pub fn last_step(&self) -> u64 {
        self.last_step
    }

    /// The gating-clock target of the FIRST boundary — the per-rank clock
    /// re-advance's initial value.
    ///
    /// Under free-run every rank's controlled clock initializes from
    /// `real_ns()` at its OWN live-loop entry, so the ranks'
    /// epochs share one DOMAIN but not one VALUE, and they are nonzero. The
    /// planner therefore carries each rank's own first target and deliberately
    /// does NOT cross-check them for equality — cross-rank boundary equality is
    /// the LOCKSTEP contract, which is mode-gated off for free-run bags.
    #[must_use]
    pub fn first_target_ns(&self) -> u64 {
        self.first_target_ns
    }
}

/// One producer of a topic, on a rank OTHER than the one consuming it.
///
/// A declared `multi_publisher_topics:` topic legitimately has
/// SEVERAL, on several ranks, so an edge's blame anchor is a list rather than a
/// single name — see [`CrossRankEdge::producers`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RemoteProducer {
    /// The rank whose member produces the topic.
    pub rank: u32,
    /// The producing node's id, for blame text (a
    /// bag-fed divergence LOCALIZES to the producing rank).
    pub node: String,
}

/// One recorded read that crosses a rank boundary — a bag-fed injection target.
///
/// The consumer rank's runtime cannot receive these frames from a live
/// producer (the producing rank replays in a different pass, into a different
/// runtime), so the executor serves them from the RECORDED frames, steered per
/// edge by the consumer's own kind-6 read log. The edge identity here is the
/// read log's own key — `(topic, consumer_node, consumer_input)` — matching
/// `MappedCredit`'s live-side edge key and the in-process mirror maps.
///
/// # One entry per EDGE, never one per producer
///
/// A `multi_publisher_topics:` topic split across ranks makes SOME of an edge's
/// frames foreign and the rest local, but it is still ONE read stream on ONE
/// `(topic, consumer_node, consumer_input)` key — that key is the read log's
/// own, and emitting a second entry under it would make every consumer keyed on
/// it (the block-edge join, the injected-topic set) see a topic twice. So the
/// edge stays single and its blame anchor carries every foreign producer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CrossRankEdge {
    /// The resolved ABSOLUTE topic name.
    pub topic: String,
    /// The consuming node's id (a member of the owning [`RankPlan`]).
    pub consumer_node: String,
    /// The consuming node's input NAME (the kind-6 `input_idx` resolves to it).
    pub consumer_input: String,
    /// Every producer of `topic` that lives on ANOTHER rank, in graph order.
    ///
    /// NON-EMPTY by construction (an edge with no foreign producer is not a
    /// cross-rank edge at all). Exactly one entry on a single-writer topic,
    /// which is every edge the earlier planner could emit; more than one
    /// only on a declared multi-publisher topic whose writers are spread over
    /// several other ranks.
    ///
    /// This rank's OWN producers are deliberately absent: the edge exists to
    /// name what must be INJECTED, and a local producer publishes live.
    pub producers: Vec<RemoteProducer>,
}

impl CrossRankEdge {
    /// The edge's SINGLE foreign producer, or `None` when the topic has more
    /// than one.
    ///
    /// For the consumers that model a producer/consumer PAIR (the rederive
    /// plane's block-edge observation): a shared bus has no single producer to
    /// stand in the pair's producer slot, and picking the first in graph order
    /// would be a guess — so they decline the observation rather than model
    /// half of it.
    #[must_use]
    pub fn sole_producer(&self) -> Option<&RemoteProducer> {
        match self.producers.as_slice() {
            [only] => Some(only),
            _ => None,
        }
    }
}

/// The per-rank replay plan: everything the sequential executor needs to stand
/// up ONE rank's runtime and drive it across its recorded step slice.
///
/// Read-only from outside this module. Its nine fields are
/// mutually constrained — `members` must be exactly the graph nodes this rank
/// owns, `subgraph` the config restricted to THOSE members, `produced` the
/// topics those members write, `cross_rank_consumed` the reads whose producers
/// are elsewhere, `external_consumed` the ones nobody writes, and the step
/// slice the one the recording covers — and every one of those relations is
/// established by [`plan_ranks`] and by nothing else. A hand-assembled plan
/// disagreeing on any of them stands up a runtime that builds one graph and is
/// driven as another; there is no constructor because there is no second
/// correct way to make one.
#[derive(Debug, Clone)]
pub struct RankPlan {
    rank: u32,
    members: Vec<String>,
    subgraph: GraphConfig,
    produced: BTreeSet<String>,
    cross_rank_consumed: Vec<CrossRankEdge>,
    external_consumed: BTreeSet<String>,
    sibling_produced: SiblingProducedTopics,
    first_step: u64,
    last_step: u64,
    first_target_ns: u64,
}

impl RankPlan {
    /// The worker rank.
    #[must_use]
    pub fn rank(&self) -> u32 {
        self.rank
    }

    /// This rank's node ids in GRAPH order (Principle #5).
    #[must_use]
    pub fn members(&self) -> &[String] {
        &self.members
    }

    /// The graph this rank's runtime builds. A rank owning every graph node
    /// gets the parent config VERBATIM (see the module docs); any proper subset
    /// is restricted by `multiprocess::subgraph_for`, which is also what
    /// absolutizes a cross-rank relative source into the external absolute
    /// source that bag-fed injection serves.
    #[must_use]
    pub fn subgraph(&self) -> &GraphConfig {
        &self.subgraph
    }

    /// The absolute topics this rank's members PRODUCE (each output's `topic:`
    /// override honored — `resolve_output_topic`).
    ///
    /// A declared `multi_publisher_topics:` topic whose writers
    /// are SPLIT across ranks appears in the `produced` set of EVERY rank that
    /// writes it, and simultaneously in that rank's
    /// [`cross_rank_consumed`](Self::cross_rank_consumed) if it also reads it.
    /// The overlap is the whole point of the split shape: the pass PRODUCES its
    /// own writer's frames live and INJECTS the other ranks' from the bag.
    #[must_use]
    pub fn produced(&self) -> &BTreeSet<String> {
        &self.produced
    }

    /// The reads whose producer lives on ANOTHER rank, in graph-node order then
    /// input-declaration order (deterministic, Principle #5).
    #[must_use]
    pub fn cross_rank_consumed(&self) -> &[CrossRankEdge] {
        &self.cross_rank_consumed
    }

    /// The absolute topics this rank reads that NO rank produces — external
    /// sources, whose recorded frames replay injects per CONSUMING rank
    /// (`inject_up_to(target)` has no single "the target"
    /// once each rank has its own clock, so the injector is re-keyed per rank).
    ///
    /// Every graph node belongs to exactly one rank (enforced in
    /// [`plan_ranks`]), so "produced by no rank" is exactly "produced by no
    /// graph node".
    #[must_use]
    pub fn external_consumed(&self) -> &BTreeSet<String> {
        &self.external_consumed
    }

    /// The absolute topics another rank PRODUCES that this rank's subgraph
    /// reads, the set a live worker hands its validation
    /// (`multiprocess::sibling_topics_for`, the same function).
    ///
    /// The split absolutizes each such source and drops its producer from this
    /// rank's view, so without the set the pass build reads a correct cross-rank
    /// edge as an in-prefix source with no producer and warns that it may be a
    /// typo. Validation only: the set changes which warnings a build emits, never
    /// how it is wired. EMPTY for a rank that owns the whole graph (every
    /// lockstep plan).
    #[must_use]
    pub fn sibling_produced(&self) -> &SiblingProducedTopics {
        &self.sibling_produced
    }

    /// The first recorded step index this rank must be driven to.
    #[must_use]
    pub fn first_step(&self) -> u64 {
        self.first_step
    }

    /// The last recorded step index this rank must be driven to.
    #[must_use]
    pub fn last_step(&self) -> u64 {
        self.last_step
    }

    /// The rank's first STEP_BOUNDARY clock target — the initial value its
    /// clock re-advance starts from (nonzero under free-run; see
    /// [`RankBoundarySummary::first_target_ns`]).
    #[must_use]
    pub fn first_target_ns(&self) -> u64 {
        self.first_target_ns
    }
}

/// Why a recording cannot be planned per rank. Every variant names the
/// offender and the fix; the class is bag-fault (exit 2), see the module docs.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum RankPlanError {
    /// No worker manifests at all.
    #[error(
        "per-rank replay needs at least one worker trace manifest, but this bag carries none \
         (only the supervisor departure-ring sentinel, or nothing at all) — the recording is \
         corrupt or hand-edited; re-record with `cerulion graph run --record`"
    )]
    NoRankManifests,

    /// Two manifests claim the same rank.
    #[error(
        "two trace manifests claim rank {rank} — the recording is corrupt or hand-edited; \
         re-record with `cerulion graph run --record`"
    )]
    DuplicateRankManifest {
        /// The rank claimed twice.
        rank: u32,
    },

    /// The manifest ranks are not `0..k`.
    #[error(
        "the trace-manifest rank set is not contiguous from 0: expected rank {expected}, found \
         rank {found} — a worker's manifest is missing or misnamed; re-record with \
         `cerulion graph run --record`"
    )]
    RankNumberingNotContiguous {
        /// The rank the contiguous walk expected next.
        expected: u32,
        /// The rank actually present at that position.
        found: u32,
    },

    /// A manifest names a node the embedded graph does not declare.
    #[error(
        "rank {rank}'s trace manifest names node `{node}`, which the bag's embedded graph \
         `{graph}` does not declare — the recording and its graph.yaml disagree (corrupt or \
         hand-edited); re-record with `cerulion graph run --record`"
    )]
    ManifestNodeNotInGraph {
        /// The rank whose manifest names it.
        rank: u32,
        /// The unknown node id.
        node: String,
        /// The embedded graph's name.
        graph: String,
    },

    /// A graph node appears in no manifest — nothing would ever fire it.
    #[error(
        "the bag's embedded graph `{graph}` declares node `{node}`, but no rank's trace manifest \
         names it — the recording and its graph.yaml disagree (corrupt or hand-edited); \
         re-record with `cerulion graph run --record`"
    )]
    GraphNodeInNoManifest {
        /// The uncovered node id.
        node: String,
        /// The embedded graph's name.
        graph: String,
    },

    /// A node appears in two ranks' manifests — its owner is ambiguous.
    #[error(
        "node `{node}` appears in BOTH rank {first_rank}'s and rank {second_rank}'s trace \
         manifests — a node runs in exactly one worker, so the recording is corrupt or \
         hand-edited; re-record with `cerulion graph run --record`"
    )]
    NodeInTwoManifests {
        /// The doubly-claimed node id.
        node: String,
        /// The lower-numbered claiming rank.
        first_rank: u32,
        /// The higher-numbered claiming rank.
        second_rank: u32,
    },

    /// A rank has a manifest but no STEP_BOUNDARY stream.
    #[error(
        "rank {rank} has a trace manifest but no STEP_BOUNDARY (record_type 3) records — its \
         recorded clock trajectory cannot be re-advanced, so its nodes cannot be replayed. The \
         recording is corrupt or was truncated mid-write; re-record with \
         `cerulion graph run --record`"
    )]
    RankWithoutBoundaryStream {
        /// The boundary-less rank.
        rank: u32,
    },

    /// A boundary stream carries a rank no manifest claims.
    #[error(
        "the scheduler trace carries STEP_BOUNDARY records stamped rank {rank}, but no trace \
         manifest claims that rank — the recording is corrupt or hand-edited; re-record with \
         `cerulion graph run --record`"
    )]
    BoundaryStreamWithoutManifest {
        /// The unclaimed rank.
        rank: u32,
    },

    /// Two boundary summaries describe the same rank.
    #[error(
        "two STEP_BOUNDARY summaries describe rank {rank} — this is a bug in the replay engine's \
         boundary walk, please report"
    )]
    DuplicateBoundaryStream {
        /// The doubly-summarized rank.
        rank: u32,
    },

    /// A boundary summary's step range runs backwards.
    #[error(
        "rank {rank}'s STEP_BOUNDARY stream summarizes as steps {first_step}..={last_step}, which \
         runs backwards — the recording is corrupt or hand-edited; re-record with \
         `cerulion graph run --record`"
    )]
    BoundaryRangeInverted {
        /// The rank whose range is inverted.
        rank: u32,
        /// The summarized first step.
        first_step: u64,
        /// The summarized last step.
        last_step: u64,
    },

    /// One topic is produced from two ranks (and is not opted into multiple
    /// publishers) — the graph itself is invalid.
    ///
    /// The DECLARED multi-publisher counterpart is not an error: a
    /// shared bus is how robotics graphs are actually written, and refusing to
    /// replay one would make those recordings unreplayable. A listed topic's
    /// producers may sit on any ranks; the pass that owns one writes its own
    /// frames and injects the rest, steered by the record-time producer labels.
    #[error(
        "topic `{topic}` is produced by node `{first_node}` on rank {first_rank} AND by node \
         `{second_node}` on rank {second_rank}, but it is not listed in the graph's \
         `multi_publisher_topics:` — a single-writer topic has exactly one producer. The \
         recording and its graph.yaml disagree (corrupt or hand-edited); re-record with \
         `cerulion graph run --record`"
    )]
    TopicProducedByTwoRanks {
        /// The doubly-produced topic.
        topic: String,
        /// The first producing rank (in graph order).
        first_rank: u32,
        /// The first producing node.
        first_node: String,
        /// The second producing rank.
        second_rank: u32,
        /// The second producing node.
        second_node: String,
    },
}

/// Plan one [`RankPlan`] per recorded worker rank, ordered `0..k`.
///
/// `manifests` and `boundaries` may arrive in ANY order — the output is always
/// rank-ascending, and the rank set is required to be exactly `0..k` (a gap or
/// a duplicate is a loud refusal, never a silently re-indexed plan).
///
/// # Preconditions
///
/// `config` is pre-validated (`validate_graph` ran — the replay gate parses the
/// bag's embedded graph through the same path a run does), which is what makes
/// two facts below safe to rely on: a relative `source:` always resolves to a
/// topic some graph node produces, and a `topic:`-override reference is always
/// absolute.
///
/// # Errors
///
/// See [`RankPlanError`] — every arm is a structural bag/graph disagreement.
pub fn plan_ranks(
    config: &GraphConfig,
    manifests: &[RankManifest],
    boundaries: &[RankBoundarySummary],
) -> Result<Vec<RankPlan>, RankPlanError> {
    if manifests.is_empty() {
        return Err(RankPlanError::NoRankManifests);
    }

    // ── rank set: unique + contiguous 0..k ────────────────────────────────
    let mut ranks: Vec<u32> = Vec::with_capacity(manifests.len());
    for m in manifests {
        if ranks.contains(&m.rank) {
            return Err(RankPlanError::DuplicateRankManifest { rank: m.rank });
        }
        ranks.push(m.rank);
    }
    ranks.sort_unstable();
    for (expected, found) in ranks.iter().copied().enumerate() {
        let expected = expected as u32;
        if found != expected {
            return Err(RankPlanError::RankNumberingNotContiguous { expected, found });
        }
    }

    // ── node → rank, cross-checked against the graph both ways ────────────
    let graph_ids: HashSet<&str> = config.nodes.iter().map(|n| n.id.as_str()).collect();
    // Walk manifests rank-ascending so a double-claim names the LOWER rank
    // first, independent of input order.
    let mut by_rank: Vec<&RankManifest> = manifests.iter().collect();
    by_rank.sort_by_key(|m| m.rank);
    let mut rank_of: HashMap<&str, u32> = HashMap::with_capacity(config.nodes.len());
    for m in &by_rank {
        for node in &m.node_ids {
            if !graph_ids.contains(node.as_str()) {
                return Err(RankPlanError::ManifestNodeNotInGraph {
                    rank: m.rank,
                    node: node.clone(),
                    graph: config.identity().to_string(),
                });
            }
            if let Some(&first_rank) = rank_of.get(node.as_str()) {
                return Err(RankPlanError::NodeInTwoManifests {
                    node: node.clone(),
                    first_rank,
                    second_rank: m.rank,
                });
            }
            rank_of.insert(node.as_str(), m.rank);
        }
    }
    // Coverage, reported in GRAPH order so the first offender is stable.
    for n in &config.nodes {
        if !rank_of.contains_key(n.id.as_str()) {
            return Err(RankPlanError::GraphNodeInNoManifest {
                node: n.id.clone(),
                graph: config.identity().to_string(),
            });
        }
    }

    // ── boundary summaries: one per rank, exactly ─────────────────────────
    let mut summary_of: HashMap<u32, RankBoundarySummary> =
        HashMap::with_capacity(boundaries.len());
    for b in boundaries {
        if !ranks.contains(&b.rank) {
            return Err(RankPlanError::BoundaryStreamWithoutManifest { rank: b.rank });
        }
        if b.last_step < b.first_step {
            return Err(RankPlanError::BoundaryRangeInverted {
                rank: b.rank,
                first_step: b.first_step,
                last_step: b.last_step,
            });
        }
        if summary_of.insert(b.rank, *b).is_some() {
            return Err(RankPlanError::DuplicateBoundaryStream { rank: b.rank });
        }
    }
    for rank in ranks.iter().copied() {
        if !summary_of.contains_key(&rank) {
            return Err(RankPlanError::RankWithoutBoundaryStream { rank });
        }
    }

    // ── producer map: absolute topic → (rank, node), graph order ──────────
    let multi_publisher: BTreeSet<&str> = config
        .multi_publisher_topics
        .iter()
        .map(String::as_str)
        .collect();
    // A topic may have SEVERAL producers, on several ranks —
    // every one of them, in graph order. A same-rank repeat is one
    // worker's own business (a listed multi-publisher topic, or two outputs of
    // one node; the live graph's own double-producer rule already judged it
    // at build), and a CROSS-rank repeat is equally legal ON A DECLARED
    // multi-publisher topic. What stays refused is the undeclared one: a
    // single-writer topic has exactly one producer, so two ranks claiming it is
    // a recording that disagrees with its own graph.
    //
    // The list is per NODE, deduped — the same rule
    // `GraphTopology::build` applies to its own producer list, and for the same
    // reason. A node may write ONE declared multi-publisher topic through TWO
    // outputs (`validate_graph`'s duplicate-output-topic check exempts a listed
    // topic, and the topology's double-producer check exempts it too), so an
    // output-keyed list would name that node twice. Downstream that is not
    // cosmetic: `CrossRankEdge::sole_producer` answers "has this edge ONE
    // producer to model a pair with", and a doubled name makes a
    // single-writer-to-this-rank edge look like a shared bus — silently
    // declining the block-credit observation the rederive plane would otherwise
    // make.
    let mut produced_by: BTreeMap<String, Vec<(u32, &str)>> = BTreeMap::new();
    for n in &config.nodes {
        let rank = rank_of[n.id.as_str()];
        for o in &n.outputs {
            let topic = resolve_output_topic(&config.prefix, &n.id, o);
            let seen = produced_by.entry(topic.clone()).or_default();
            if !multi_publisher.contains(topic.as_str()) {
                // The FIRST producer on another rank, in graph order — the
                // blame anchor this refusal has always named.
                if let Some((first_rank, first_node)) = seen.iter().find(|(r, _)| *r != rank) {
                    return Err(RankPlanError::TopicProducedByTwoRanks {
                        topic,
                        first_rank: *first_rank,
                        first_node: (*first_node).to_string(),
                        second_rank: rank,
                        second_node: n.id.clone(),
                    });
                }
            }
            if !seen.contains(&(rank, n.id.as_str())) {
                seen.push((rank, n.id.as_str()));
            }
        }
    }

    // ── per-rank plans, rank-ascending ────────────────────────────────────
    let mut plans = Vec::with_capacity(ranks.len());
    for rank in ranks.iter().copied() {
        let members: Vec<String> = config
            .nodes
            .iter()
            .filter(|n| rank_of[n.id.as_str()] == rank)
            .map(|n| n.id.clone())
            .collect();
        let produced: BTreeSet<String> = produced_by
            .iter()
            .filter(|(_, producers)| producers.iter().any(|(r, _)| *r == rank))
            .map(|(topic, _)| topic.clone())
            .collect();

        let mut cross_rank_consumed = Vec::new();
        let mut external_consumed = BTreeSet::new();
        for n in config
            .nodes
            .iter()
            .filter(|n| rank_of[n.id.as_str()] == rank)
        {
            for input in &n.inputs {
                // Classify by the RESOLVED topic, never by the leading slash: an
                // ABSOLUTE source can name another rank's `topic:`-overridden
                // output, which is a cross-rank edge just as much as a relative
                // one is.
                let topic = resolve_source(&config.prefix, &input.source);
                match produced_by.get(&topic) {
                    // EVERY producer on another rank. A topic this
                    // rank also produces still yields an edge when some OTHER
                    // rank writes it too (the split multi-publisher shape) —
                    // the local half is published live, the foreign half is
                    // what this edge asks the bag for.
                    Some(producers) => {
                        let foreign: Vec<RemoteProducer> = producers
                            .iter()
                            .filter(|(r, _)| *r != rank)
                            .map(|(r, node)| RemoteProducer {
                                rank: *r,
                                node: (*node).to_string(),
                            })
                            .collect();
                        if !foreign.is_empty() {
                            cross_rank_consumed.push(CrossRankEdge {
                                topic,
                                consumer_node: n.id.clone(),
                                consumer_input: input.name.clone(),
                                producers: foreign,
                            });
                        }
                    }
                    None => {
                        external_consumed.insert(topic);
                    }
                }
            }
        }

        // The whole-graph short-circuit (module docs): restricting a rank that
        // already owns every node is an identity on nodes + inputs, and would
        // only rename the graph and drop its `network:` block.
        let subgraph = if members.len() == config.nodes.len() {
            config.clone()
        } else {
            subgraph_for(config, &format!("rank{rank}"), &members)
        };
        let sibling_produced = sibling_topics_for(config, &members);
        let summary = summary_of[&rank];
        plans.push(RankPlan {
            rank,
            members,
            subgraph,
            produced,
            cross_rank_consumed,
            external_consumed,
            sibling_produced,
            first_step: summary.first_step,
            last_step: summary.last_step,
            first_target_ns: summary.first_target_ns,
        });
    }
    Ok(plans)
}

// ===========================================================================
// Split `block` edges a several-rank replay can and cannot re-execute
// ===========================================================================

/// Where the `block` on a split input comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockDeclaredBy {
    /// The graph the bag embeds sets `backpressure: block` on the input.
    Graph,
    /// The consumer node's declaration in the build being replayed, with no
    /// graph key over it.
    Node,
}

/// A `block` input the bag's rank manifests split from a topic that two or
/// more in-graph producers write, one or more of them in another rank. A
/// `block` edge crosses ranks only on a credit word, and a credit word counts
/// one producer's outstanding frames, so a several-rank replay refuses the
/// bag before its first pass.
///
/// The fields are private: [`Self::new`] is the one builder, and it holds the
/// invariant the sentence states, two or more producers with one or more in
/// another rank than the consumer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UncreditableSplitEdge {
    topic: String,
    consumer_node: String,
    consumer_input: String,
    consumer_rank: usize,
    /// Every in-graph producer of the topic with its rank, in topic order.
    producers: Vec<(String, usize)>,
    block_from: BlockDeclaredBy,
}

impl UncreditableSplitEdge {
    /// The refusal of the `block` input `consumer_node.consumer_input` on
    /// `topic`, in `consumer_rank`, whose topic `producers` write (each with
    /// its rank, in topic order), or `None` when `producers` names no
    /// producer or one, or names none in another rank than `consumer_rank`:
    /// such an edge has no producer to count, is creditable, or is not split.
    #[must_use]
    pub fn new(
        topic: String,
        consumer_node: String,
        consumer_input: String,
        consumer_rank: usize,
        producers: Vec<(String, usize)>,
        block_from: BlockDeclaredBy,
    ) -> Option<Self> {
        if producers.len() < 2 || producers.iter().all(|(_, rank)| *rank == consumer_rank) {
            return None;
        }
        Some(Self {
            topic,
            consumer_node,
            consumer_input,
            consumer_rank,
            producers,
            block_from,
        })
    }
}

impl std::fmt::Display for UncreditableSplitEdge {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let (node, input, topic) = (&self.consumer_node, &self.consumer_input, &self.topic);
        let placed: Vec<String> = self
            .producers
            .iter()
            .map(|(producer, rank)| format!("'{producer}' in rank {rank}"))
            .collect();
        // `new` holds two or more producers, so the last one always follows
        // "and".
        let placed = match placed.split_last() {
            Some((last, rest)) => format!("{} and {last}", rest.join(", ")),
            None => String::new(),
        };
        write!(
            f,
            "block input '{node}.{input}' reads topic '{topic}', which has {n} in-graph \
             producers, and this bag's rank manifests put '{node}' in rank {rank}, {placed}. A \
             `block` edge can cross ranks only on a topic with exactly one in-graph producer \
             (its credit word counts one producer's outstanding frames), so this bag cannot be \
             re-executed with `block` on that input. ",
            n = self.producers.len(),
            rank = self.consumer_rank,
        )?;
        match self.block_from {
            BlockDeclaredBy::Graph => write!(
                f,
                "`block` comes from the graph this bag embeds (`backpressure: block` on \
                 '{node}.{input}'), so the bag was changed after it was recorded. Re-execute an \
                 unchanged copy of the bag, or record the run again."
            ),
            BlockDeclaredBy::Node => write!(
                f,
                "`block` comes from the declaration of input '{input}' on node '{node}' in the \
                 build you replay. If '{node}' did not declare `block` on '{input}' when the run \
                 was recorded, check out the recording-era node sources, build them and replay \
                 with that build; otherwise record the run again with every producer of \
                 '{topic}' and '{node}' in one process group."
            ),
        }
    }
}

impl std::error::Error for UncreditableSplitEdge {}

/// A one-producer `block` edge the bag's rank manifests split across ranks
/// whose input sets `block_timeout_ms`. Across ranks the edge counts on a
/// shared credit word, which a timeout release keeps exact only when the
/// producer and the consumer fire one at a time on one thread, so a
/// several-rank replay refuses the bag before its first pass.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SplitEdgeTimeout {
    /// The resolved absolute topic.
    pub topic: String,
    /// The consuming node.
    pub consumer_node: String,
    /// The consuming input.
    pub consumer_input: String,
    /// The consumer's rank.
    pub consumer_rank: usize,
    /// The topic's one in-graph producer.
    pub producer: String,
    /// The producer's rank.
    pub producer_rank: usize,
}

impl std::fmt::Display for SplitEdgeTimeout {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let Self {
            topic,
            consumer_node: node,
            consumer_input: input,
            consumer_rank: rank,
            producer,
            producer_rank,
        } = self;
        write!(
            f,
            "this bag's rank manifests split the `block` edge '{topic}' -> '{node}.{input}' \
             across ranks ('{node}' in rank {rank}, its producer '{producer}' in rank \
             {producer_rank}), and the input sets `block_timeout_ms`. Across ranks the edge \
             counts on a shared credit word, and a timeout release keeps that count exact only \
             when the producer and the consumer fire one at a time on one thread, so `cerulion \
             graph run` refuses `block_timeout_ms` on a split edge before any worker starts. \
             `block_timeout_ms` comes from the graph this bag embeds, so the bag was changed \
             after it was recorded. Re-execute an unchanged copy of the bag, or record the run \
             again."
        )
    }
}

impl std::error::Error for SplitEdgeTimeout {}

/// Why [`classify_split_block_edges`] refuses a bag.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum SplitBlockEdgeError {
    /// The whole graph the bag embeds, with the build's node declarations,
    /// does not build into a valid topology ([`GraphTopology::build`] or
    /// [`GraphTopology::validate`]); `reason` is that check's own text.
    #[error("{reason}")]
    Topology {
        /// The topology check's refusal.
        reason: String,
    },
    /// A split `block` edge whose topic two or more in-graph producers write.
    #[error(transparent)]
    Uncreditable(#[from] UncreditableSplitEdge),
    /// A split one-producer `block` edge whose input sets `block_timeout_ms`.
    #[error(transparent)]
    Timeout(#[from] SplitEdgeTimeout),
    /// A fact the replay engine guarantees did not hold; `reason` names it.
    /// Reached through [`classify_split_block_edges`] only by a map that
    /// places no rank for a native node, which a map [`plan_node_ranks`]
    /// builds from [`plan_ranks`]' plans never does. Its two other sources
    /// cannot fire past validation: [`credit_edges_for`]'s depth refusals (a
    /// depth of 0, or one past `u32`), and a minted timeout edge with more
    /// than one producer (the mint credits a topic with one producer only).
    #[error("{reason}; this is a bug in the replay engine, please report")]
    EngineBug {
        /// What did not hold.
        reason: String,
    },
}

/// Classify every `block` edge the manifests' `node_rank` map splits across
/// ranks: the creditable ones (`Ok`, the edges a live run mints a cross-process
/// credit word for, in [`credit_edges_for`]'s order), or why not (`Err`): the
/// first edge a several-rank replay refuses, a whole-graph topology refusal,
/// or a fact the engine guarantees that did not hold.
///
/// PURE: the whole graph the bag embeds, the build's own node declarations
/// (`infos`, read before any pass) and the map in, a verdict out. `node_rank`
/// must place every native (non-`ros2:`) node of `config`, as a map
/// [`plan_node_ranks`] builds from [`plan_ranks`]' plans does; one that leaves
/// one out is [`SplitBlockEdgeError::EngineBug`], naming the first in graph
/// order. Over ranks in place of process groups, the order is:
///
/// 1. the whole-graph topology is built; a build refusal is
///    [`SplitBlockEdgeError::Topology`];
/// 2. every flow [`block_colocation_seeds`] seeds whose
///    [`BlockColocationSeed::credit_bar`](cerulion_core::graph::BlockColocationSeed::credit_bar)
///    is set (two or more in-graph producers: a seed always has a producer and
///    a `block` consumer) is refused at its first `block` consumer with a
///    producer in another rank, as [`UncreditableSplitEdge`]. A producer in
///    the consumer's rank is not split from it, as the live partition check
///    skips a producer in the consumer's group (which covers a node that
///    produces the topic it consumes). Asked before validation, so a refusal
///    that validation makes on the same flow (a `block_timeout_ms` on a shared
///    topic, a `block` ring) does not stand in front of this one;
/// 3. the topology is validated ([`GraphTopology::validate`]); a refusal is
///    [`SplitBlockEdgeError::Topology`]. Past it every depth is in
///    `1..=MAX_CONSUMER_DEPTH`, so [`credit_edges_for`]'s own depth refusals
///    (a depth of 0, or one past `u32`) cannot fire; one that did would be
///    [`SplitBlockEdgeError::EngineBug`];
/// 4. the creditable split edges are [`credit_edges_for`]'s, the live mint,
///    and the first whose input sets `block_timeout_ms`
///    (`multiprocess::split_edge_timeout_producers`, the live check) is
///    refused as [`SplitEdgeTimeout`]. A minted edge has one producer on one
///    rank; one that names more is [`SplitBlockEdgeError::EngineBug`], never a
///    passed timeout.
///
/// Deterministic: topology order, then graph order within a flow; two calls
/// over one input return equal values.
pub fn classify_split_block_edges(
    config: &GraphConfig,
    infos: &IndexMap<String, NodeInfo>,
    node_rank: &HashMap<&str, usize>,
) -> Result<Vec<CreditEdgePlan>, SplitBlockEdgeError> {
    if let Some(node) = config
        .native_nodes()
        .find(|n| !node_rank.contains_key(n.id.as_str()))
    {
        return Err(SplitBlockEdgeError::EngineBug {
            reason: format!(
                "node '{}' is in no rank plan, and the split `block` edge check needs every \
                 native node's rank",
                node.id
            ),
        });
    }
    let topology_err = |reason: String| SplitBlockEdgeError::Topology { reason };
    let topology = GraphTopology::build(config, infos).map_err(|e| topology_err(e.to_string()))?;

    // Every node the topology names is a native node of `config`, which the
    // check above found placed, so the indexing below cannot miss.
    for seed in block_colocation_seeds(&topology) {
        if seed.credit_bar().is_none() {
            continue;
        }
        let producers: Vec<(String, usize)> = seed
            .producers
            .iter()
            .map(|p| (p.clone(), node_rank[p.as_str()]))
            .collect();
        for (consumer, input) in &seed.block_consumers {
            if let Some(edge) = UncreditableSplitEdge::new(
                seed.topic.clone(),
                consumer.clone(),
                input.clone(),
                node_rank[consumer.as_str()],
                producers.clone(),
                block_declared_by(config, consumer, input),
            ) {
                return Err(edge.into());
            }
        }
    }

    topology
        .validate()
        .map_err(|e| topology_err(e.to_string()))?;
    let credited =
        credit_edges_for(&topology, node_rank).map_err(|e| SplitBlockEdgeError::EngineBug {
            reason: format!("past validation, the credit mint refused the topology: {e}"),
        })?;
    for edge in &credited {
        let Some(producers) = split_edge_timeout_producers(
            &topology,
            &edge.topic,
            &edge.consumer_node,
            &edge.consumer_input,
        ) else {
            continue;
        };
        let ([producer], [producer_rank]) = (producers, edge.producer_ranks.as_slice()) else {
            return Err(SplitBlockEdgeError::EngineBug {
                reason: format!(
                    "the split `block` edge '{}' -> '{}.{}' sets `block_timeout_ms` and the \
                     mint credits it with {} producer(s) on {} rank(s), where a credit word \
                     counts one producer",
                    edge.topic,
                    edge.consumer_node,
                    edge.consumer_input,
                    producers.len(),
                    edge.producer_ranks.len()
                ),
            });
        };
        return Err(SplitEdgeTimeout {
            topic: edge.topic.clone(),
            consumer_node: edge.consumer_node.clone(),
            consumer_input: edge.consumer_input.clone(),
            consumer_rank: edge.consumer_rank,
            producer: producer.clone(),
            producer_rank: *producer_rank,
        }
        .into());
    }
    Ok(credited)
}

/// Whether `node.input`'s `block` is the graph's `backpressure: block` key or
/// the node's own declaration (the key is absent, so the declaration stands).
fn block_declared_by(config: &GraphConfig, node: &str, input: &str) -> BlockDeclaredBy {
    let graph_key = config
        .nodes
        .iter()
        .find(|n| n.id == node)
        .and_then(|n| n.inputs.iter().find(|i| i.name == input))
        .and_then(|i| i.backpressure);
    match graph_key {
        Some(InputBackpressure::Block) => BlockDeclaredBy::Graph,
        Some(InputBackpressure::DropOldest) | None => BlockDeclaredBy::Node,
    }
}

/// Each plan's members mapped to its rank: the manifests' node to rank map,
/// in the shape [`classify_split_block_edges`] and [`credit_edges_for`] take.
#[must_use]
pub fn plan_node_ranks(plans: &[RankPlan]) -> HashMap<&str, usize> {
    plans
        .iter()
        .flat_map(|plan| {
            plan.members()
                .iter()
                .map(move |m| (m.as_str(), plan.rank() as usize))
        })
        .collect()
}

// ===========================================================================
// The BAG-TIME window, in ONE place
// ===========================================================================

/// The recorded instant `--duration` is measured FROM, given every rank's own
/// first boundary target.
///
/// **What min-across-ranks buys, stated precisely.** Under free-run each rank's
/// controlled clock initializes from `real_ns()` at its OWN live-loop entry,
/// so the epochs share one DOMAIN but not one VALUE —
/// [`RankBoundarySummary::first_target_ns`] says so, and this function is the
/// reason it matters. Taking the MINIMUM gives every rank ONE SHARED ORIGIN, so
/// `--duration D` names the same wall interval for all of them; it does NOT
/// give each rank its own first D seconds. A rank whose live loop entered later
/// therefore loses more of its own tail than an earlier one — which is the
/// point (they ran concurrently, and a bound over concurrent streams must be
/// one interval, not k interleaved ones), but it means a per-rank
/// `ticks_replayed` under a bound is NOT expected to be uniform.
///
/// Under lockstep there is one stream and this is simply its first target.
///
/// `0` for an empty summary list — no boundaries at all is refused by the
/// engine's own boundary gate a moment later, and returning a sentinel here
/// would be a second, weaker copy of that refusal.
#[must_use]
pub fn run_epoch_ns(summaries: &[RankBoundarySummary]) -> u64 {
    summaries
        .iter()
        .map(|s| s.first_target_ns)
        .min()
        .unwrap_or(0)
}

/// Is a recorded instant OUTSIDE the `--duration` bag-time window?
///
/// **HALF-OPEN, `[epoch, epoch + D)`** — the one rule, in the one place, for
/// every surface that applies a bag-time bound: the resim pass loop, the resume
/// path's `last_replayed_step`, and `bag play`'s playback walk. They disagreed
/// once (`> d` on the playback half against `>= d` on the resim half) while
/// `docs/bag.md` documented half-open for both, so `bag play --duration 0`
/// republished the whole first frame of every channel while
/// `bag play --resim all --duration 0` correctly executed nothing.
///
/// Half-open is what makes `--duration 0` cover NOTHING, which is the one
/// degenerate ask an operator can express and the meaning the deleted
/// `--max-ticks 0` carried. The cost, stated: a bound EXACTLY equal to the
/// recorded span excludes the final step / frame. The way to say "the whole
/// bag" is to omit the flag, which is also its default.
///
/// `elapsed` is the caller's own bag-time delta — resim measures it from the
/// shared [`run_epoch_ns`], playback from each CHANNEL's own first stamp (wire
/// stamps in different channels are different producers' clocks, which
/// `bag_cmd` refuses to compare) — so this function judges the BOUND, never the
/// origin.
#[must_use]
pub fn beyond_duration_bound(elapsed_ns: u64, bound_ns: u64) -> bool {
    elapsed_ns >= bound_ns
}

#[cfg(test)]
mod bag_time_window_tests {
    use super::*;

    fn summary(rank: u32, first_target_ns: u64) -> RankBoundarySummary {
        RankBoundarySummary::new(rank, 0, 9, first_target_ns)
    }

    #[test]
    fn the_run_epoch_is_the_lowest_first_target_across_the_ranks() {
        // Hand oracle: three ranks whose live loops entered 5 ms apart. The
        // shared origin is the EARLIEST, so a `--duration` names one interval.
        let s = [
            summary(0, 1_000_000_000),
            summary(1, 1_005_000_000),
            summary(2, 995_000_000),
        ];
        assert_eq!(run_epoch_ns(&s), 995_000_000);
        // Order-independent (a `min`, not a "first").
        let reordered = [s[1], s[2], s[0]];
        assert_eq!(run_epoch_ns(&reordered), 995_000_000);
        // One rank (the lockstep shape) is its own epoch.
        assert_eq!(run_epoch_ns(&s[..1]), 1_000_000_000);
        // No boundaries at all: 0, and the engine's own gate refuses the bag.
        assert_eq!(run_epoch_ns(&[]), 0);
    }

    #[test]
    fn the_duration_window_is_half_open_on_both_sides_of_the_boundary() {
        // `--duration 0` covers NOTHING — the degenerate ask, and the one a
        // `>` comparison silently turned into "the first frame".
        assert!(beyond_duration_bound(0, 0), "elapsed 0 under a 0 bound");
        // Strictly inside.
        assert!(!beyond_duration_bound(0, 100));
        assert!(!beyond_duration_bound(99, 100));
        // AT the bound is OUT (half-open).
        assert!(beyond_duration_bound(100, 100));
        // Past it.
        assert!(beyond_duration_bound(101, 100));
        // Saturating callers hand this an already-clamped delta; the boundary
        // rule is the same at the extremes.
        assert!(!beyond_duration_bound(u64::MAX - 1, u64::MAX));
        assert!(beyond_duration_bound(u64::MAX, u64::MAX));
    }
}

#[cfg(test)]
mod split_block_edge_tests {
    //! Oracle vectors for [`classify_split_block_edges`]: a hand-written graph,
    //! hand-built node declarations and a hand-built node to rank map in, a
    //! hand-written verdict out.

    use super::*;
    use cerulion_core::graph::node::{BackpressurePolicy, InputMeta};

    /// One input of a test graph: `(name, source, extra YAML keys, one per
    /// line)`.
    type In<'a> = (&'a str, &'a str, &'a str);

    /// A test graph under prefix `g`. Each node is `(id, its one output's
    /// topic override, its inputs)`: `None` declares no output, `Some("")`
    /// one `out` on the default topic `/g/<id>/out`, `Some(t)` one `out` on
    /// `t`.
    fn graph(listed: &[&str], nodes: &[(&str, Option<&str>, &[In<'_>])]) -> GraphConfig {
        let mut y = String::from("name: split\nprefix: g\n");
        if !listed.is_empty() {
            y.push_str("multi_publisher_topics:\n");
            for t in listed {
                y.push_str(&format!("  - {t}\n"));
            }
        }
        y.push_str("nodes:\n");
        for (id, out, inputs) in nodes {
            y.push_str(&format!("  - id: {id}\n    type: t\n"));
            if !inputs.is_empty() {
                y.push_str("    inputs:\n");
                for (name, source, extra) in *inputs {
                    y.push_str(&format!("      - name: {name}\n        source: {source}\n"));
                    for key in extra.lines() {
                        y.push_str(&format!("        {key}\n"));
                    }
                }
            }
            if let Some(topic) = out {
                y.push_str(
                    "    outputs:\n      - name: out\n        schema: geometry_msgs/Vector3\n",
                );
                if !topic.is_empty() {
                    y.push_str(&format!("        topic: {topic}\n"));
                }
            }
        }
        cerulion_core::graph::parse_graph(&y).expect("the test graph parses")
    }

    /// One declared input at depth 4.
    fn meta(name: &str, backpressure: BackpressurePolicy, trigger: bool) -> InputMeta {
        InputMeta {
            name: name.to_string(),
            schema_hash: 0,
            trigger,
            depth: 4,
            backpressure,
            expect_within_ms: None,
            accumulate: false,
            consume: Default::default(),
        }
    }

    /// Every node of `config` with no input metadata, then `declared` over
    /// it: `(node, its declared inputs)`.
    fn infos(
        config: &GraphConfig,
        declared: &[(&str, Vec<InputMeta>)],
    ) -> IndexMap<String, NodeInfo> {
        let mut out: IndexMap<String, NodeInfo> = config
            .nodes
            .iter()
            .map(|n| (n.id.clone(), NodeInfo::with_meta(vec![], vec![])))
            .collect();
        for (node, metas) in declared {
            out.insert(
                (*node).to_string(),
                NodeInfo::with_meta(metas.clone(), vec![]),
            );
        }
        out
    }

    fn ranks<'a>(pairs: &[(&'a str, usize)]) -> HashMap<&'a str, usize> {
        pairs.iter().copied().collect()
    }

    const BLOCK: BackpressurePolicy = BackpressurePolicy::Block;
    const DROP: BackpressurePolicy = BackpressurePolicy::DropOldest;

    /// `prod -> sink.gate`, with `key` on `sink.gate`.
    fn one_writer(key: &str) -> GraphConfig {
        graph(
            &[],
            &[
                ("prod", Some(""), &[]),
                ("sink", None, &[("gate", "prod/out", key)]),
            ],
        )
    }

    /// `p1` and `p2` both write the listed `/g/bus`, and `sink.inp` reads it
    /// with `key`.
    fn shared_bus(key: &str) -> GraphConfig {
        graph(
            &["/g/bus"],
            &[
                ("p1", Some("/g/bus"), &[]),
                ("p2", Some("/g/bus"), &[]),
                ("sink", None, &[("inp", "/g/bus", key)]),
            ],
        )
    }

    fn sink_gate_plan(consumer: &str, consumer_rank: usize) -> CreditEdgePlan {
        CreditEdgePlan {
            topic: "/g/prod/out".to_string(),
            consumer_node: consumer.to_string(),
            consumer_input: "gate".to_string(),
            depth: 4,
            producer_ranks: vec![0],
            consumer_rank,
            producer_nodes: vec!["prod".to_string()],
        }
    }

    /// The refusal of `consumer.inp` on `/g/bus`, `p1` and `p2` at `p1` and
    /// `p2`.
    fn bus_refusal(
        consumer: &str,
        consumer_rank: usize,
        (p1, p2): (usize, usize),
        block_from: BlockDeclaredBy,
    ) -> SplitBlockEdgeError {
        SplitBlockEdgeError::Uncreditable(
            UncreditableSplitEdge::new(
                "/g/bus".to_string(),
                consumer.to_string(),
                "inp".to_string(),
                consumer_rank,
                vec![("p1".to_string(), p1), ("p2".to_string(), p2)],
                block_from,
            )
            .expect("two producers, one in another rank"),
        )
    }

    #[test]
    fn a_creditable_split_edge_is_credited_and_nothing_is_refused() {
        let config = one_writer("");
        let infos = infos(&config, &[("sink", vec![meta("gate", BLOCK, true)])]);
        assert_eq!(
            classify_split_block_edges(&config, &infos, &ranks(&[("prod", 0), ("sink", 1)])),
            Ok(vec![sink_gate_plan("sink", 1)])
        );
    }

    /// A split `block` input that is not a trigger is credited exactly as a
    /// trigger is: the live mint does not ask whether the input fires its node.
    #[test]
    fn a_split_block_input_that_is_not_a_trigger_is_credited_like_a_trigger() {
        let config = one_writer("");
        let infos = infos(&config, &[("sink", vec![meta("gate", BLOCK, false)])]);
        assert_eq!(
            classify_split_block_edges(&config, &infos, &ranks(&[("prod", 0), ("sink", 1)])),
            Ok(vec![sink_gate_plan("sink", 1)])
        );
    }

    #[test]
    fn a_co_located_block_edge_is_neither_credited_nor_refused() {
        let config = one_writer("");
        let infos_one = infos(&config, &[("sink", vec![meta("gate", BLOCK, true)])]);
        assert_eq!(
            classify_split_block_edges(&config, &infos_one, &ranks(&[("prod", 1), ("sink", 1)])),
            Ok(vec![])
        );
        // Two writers and their `block` reader in one rank: nothing is split.
        let bus = shared_bus("");
        let infos_bus = infos(&bus, &[("sink", vec![meta("inp", BLOCK, true)])]);
        assert_eq!(
            classify_split_block_edges(
                &bus,
                &infos_bus,
                &ranks(&[("p1", 0), ("p2", 0), ("sink", 0)])
            ),
            Ok(vec![])
        );
    }

    #[test]
    fn a_split_edge_that_is_not_block_is_neither_credited_nor_refused() {
        let config = one_writer("");
        let infos_one = infos(&config, &[("sink", vec![meta("gate", DROP, true)])]);
        assert_eq!(
            classify_split_block_edges(&config, &infos_one, &ranks(&[("prod", 0), ("sink", 1)])),
            Ok(vec![])
        );
        let bus = shared_bus("");
        let infos_bus = infos(&bus, &[("sink", vec![meta("inp", DROP, true)])]);
        assert_eq!(
            classify_split_block_edges(
                &bus,
                &infos_bus,
                &ranks(&[("p1", 0), ("p2", 0), ("sink", 1)])
            ),
            Ok(vec![])
        );
    }

    /// A topic `multi_publisher_topics:` lists but one in-graph node writes is
    /// creditable, as the live mint credits it.
    #[test]
    fn a_listed_topic_with_one_writer_is_credited() {
        let config = graph(
            &["/g/bus"],
            &[
                ("p1", Some("/g/bus"), &[]),
                ("sink", None, &[("inp", "/g/bus", "")]),
            ],
        );
        let infos = infos(&config, &[("sink", vec![meta("inp", BLOCK, true)])]);
        assert_eq!(
            classify_split_block_edges(&config, &infos, &ranks(&[("p1", 0), ("sink", 1)])),
            Ok(vec![CreditEdgePlan {
                topic: "/g/bus".to_string(),
                consumer_node: "sink".to_string(),
                consumer_input: "inp".to_string(),
                depth: 4,
                producer_ranks: vec![0],
                consumer_rank: 1,
                producer_nodes: vec!["p1".to_string()],
            }])
        );
    }

    #[test]
    fn two_foreign_writers_on_a_block_edge_the_graph_sets_are_refused_naming_both() {
        let config = shared_bus("backpressure: block");
        // The node declares `drop_oldest`; the graph's key is what makes it
        // `block`.
        let infos = infos(&config, &[("sink", vec![meta("inp", DROP, true)])]);
        let verdict = classify_split_block_edges(
            &config,
            &infos,
            &ranks(&[("p1", 0), ("p2", 2), ("sink", 1)]),
        );
        assert_eq!(
            verdict,
            Err(bus_refusal("sink", 1, (0, 2), BlockDeclaredBy::Graph))
        );
        assert_eq!(
            verdict.unwrap_err().to_string(),
            "block input 'sink.inp' reads topic '/g/bus', which has 2 in-graph producers, and \
             this bag's rank manifests put 'sink' in rank 1, 'p1' in rank 0 and 'p2' in rank 2. \
             A `block` edge can cross ranks only on a topic with exactly one in-graph producer \
             (its credit word counts one producer's outstanding frames), so this bag cannot be \
             re-executed with `block` on that input. `block` comes from the graph this bag embeds \
             (`backpressure: block` on 'sink.inp'), so the bag was changed after it was recorded. \
             Re-execute an unchanged copy of the bag, or record the run again."
        );
    }

    /// The mixed shape: one writer in the consumer's rank, one in another.
    /// The build replayed declares `block`, where the recording's sources may
    /// have declared `drop_oldest` (`cerulion graph run` accepts this split
    /// when the node declares `drop_oldest`), so
    /// the refusal names what replay knows: the declaration in this build.
    #[test]
    fn a_split_with_one_local_writer_whose_block_the_build_declares_is_refused() {
        let config = shared_bus("");
        let infos = infos(&config, &[("sink", vec![meta("inp", BLOCK, true)])]);
        let verdict = classify_split_block_edges(
            &config,
            &infos,
            &ranks(&[("p1", 0), ("p2", 1), ("sink", 1)]),
        );
        assert_eq!(
            verdict,
            Err(bus_refusal("sink", 1, (0, 1), BlockDeclaredBy::Node))
        );
        assert_eq!(
            verdict.unwrap_err().to_string(),
            "block input 'sink.inp' reads topic '/g/bus', which has 2 in-graph producers, and \
             this bag's rank manifests put 'sink' in rank 1, 'p1' in rank 0 and 'p2' in rank 1. \
             A `block` edge can cross ranks only on a topic with exactly one in-graph producer \
             (its credit word counts one producer's outstanding frames), so this bag cannot be \
             re-executed with `block` on that input. `block` comes from the declaration of input \
             'inp' on node 'sink' in the build you replay. If 'sink' did not declare `block` on \
             'inp' when the run was recorded, check out the recording-era node sources, build \
             them and replay with that build; otherwise record the run again with every \
             producer of '/g/bus' and 'sink' in one process group."
        );
    }

    /// Three writers are named in topic order, which is the graph's
    /// declaration order and not their names' order, the last after "and".
    #[test]
    fn three_writers_are_named_in_topic_order() {
        let config = graph(
            &["/g/bus"],
            &[
                ("zeta", Some("/g/bus"), &[]),
                ("alpha", Some("/g/bus"), &[]),
                ("mid", Some("/g/bus"), &[]),
                ("sink", None, &[("inp", "/g/bus", "")]),
            ],
        );
        let infos = infos(&config, &[("sink", vec![meta("inp", BLOCK, true)])]);
        let verdict = classify_split_block_edges(
            &config,
            &infos,
            &ranks(&[("zeta", 2), ("alpha", 0), ("mid", 0), ("sink", 0)]),
        );
        let text = verdict.unwrap_err().to_string();
        assert!(
            text.starts_with(
                "block input 'sink.inp' reads topic '/g/bus', which has 3 in-graph producers, \
                 and this bag's rank manifests put 'sink' in rank 0, 'zeta' in rank 2, 'alpha' \
                 in rank 0 and 'mid' in rank 0. "
            ),
            "{text}"
        );
    }

    /// The same graph and ranks classify by the build's declaration, and a
    /// graph key over it wins.
    #[test]
    fn the_candidate_build_decides_a_split_edge_the_graph_leaves_to_it() {
        let split = ranks(&[("p1", 0), ("p2", 1), ("sink", 1)]);
        let config = shared_bus("");
        let dropping = infos(&config, &[("sink", vec![meta("inp", DROP, true)])]);
        assert_eq!(
            classify_split_block_edges(&config, &dropping, &split),
            Ok(vec![])
        );
        let keyed = shared_bus("backpressure: drop_oldest");
        let blocking = infos(&keyed, &[("sink", vec![meta("inp", BLOCK, true)])]);
        assert_eq!(
            classify_split_block_edges(&keyed, &blocking, &split),
            Ok(vec![])
        );
    }

    #[test]
    fn of_two_block_readers_of_a_topic_only_the_split_one_is_refused() {
        let bus = graph(
            &["/g/bus"],
            &[
                ("p1", Some("/g/bus"), &[]),
                ("p2", Some("/g/bus"), &[]),
                ("near", None, &[("inp", "/g/bus", "")]),
                ("far", None, &[("inp", "/g/bus", "")]),
            ],
        );
        let declared = infos(
            &bus,
            &[
                ("near", vec![meta("inp", BLOCK, true)]),
                ("far", vec![meta("inp", BLOCK, true)]),
            ],
        );
        assert_eq!(
            classify_split_block_edges(
                &bus,
                &declared,
                &ranks(&[("p1", 0), ("p2", 0), ("near", 0), ("far", 1)])
            ),
            Err(bus_refusal("far", 1, (0, 0), BlockDeclaredBy::Node))
        );
        // One writer: only the split reader is credited.
        let one = graph(
            &[],
            &[
                ("prod", Some(""), &[]),
                ("near", None, &[("gate", "prod/out", "")]),
                ("far", None, &[("gate", "prod/out", "")]),
            ],
        );
        let declared_one = infos(
            &one,
            &[
                ("near", vec![meta("gate", BLOCK, true)]),
                ("far", vec![meta("gate", BLOCK, true)]),
            ],
        );
        assert_eq!(
            classify_split_block_edges(
                &one,
                &declared_one,
                &ranks(&[("prod", 0), ("near", 0), ("far", 1)])
            ),
            Ok(vec![sink_gate_plan("far", 1)])
        );
    }

    /// A node that writes the topic it reads with `block` is refused as a
    /// split edge when the topic's other writer runs in another rank, before
    /// the validation that refuses the `block` ring the self edge makes; with
    /// both writers in its rank the ring is the validation's refusal.
    #[test]
    fn a_block_reader_that_writes_its_own_topic_is_judged_split_first() {
        let config = graph(
            &["/g/bus"],
            &[
                ("p1", Some("/g/bus"), &[("inp", "/g/bus", "")]),
                ("p2", Some("/g/bus"), &[]),
            ],
        );
        let declared = infos(&config, &[("p1", vec![meta("inp", BLOCK, true)])]);
        assert_eq!(
            classify_split_block_edges(&config, &declared, &ranks(&[("p1", 0), ("p2", 1)])),
            Err(bus_refusal("p1", 0, (0, 1), BlockDeclaredBy::Node))
        );
        let verdict =
            classify_split_block_edges(&config, &declared, &ranks(&[("p1", 0), ("p2", 0)]));
        let Err(SplitBlockEdgeError::Topology { reason }) = &verdict else {
            panic!("the validation refuses the ring: {verdict:?}");
        };
        assert!(
            reason.starts_with(
                "Graph validation error: a ring of `block` edges: p1 -> p1.inp (topic \
                 '/g/bus'). "
            ),
            "{reason}"
        );
    }

    #[test]
    fn a_split_edge_with_a_block_timeout_is_refused_naming_both_ranks() {
        let config = one_writer("block_timeout_ms: 100");
        let declared = infos(&config, &[("sink", vec![meta("gate", BLOCK, true)])]);
        let verdict =
            classify_split_block_edges(&config, &declared, &ranks(&[("prod", 0), ("sink", 1)]));
        assert_eq!(
            verdict,
            Err(SplitBlockEdgeError::Timeout(SplitEdgeTimeout {
                topic: "/g/prod/out".to_string(),
                consumer_node: "sink".to_string(),
                consumer_input: "gate".to_string(),
                consumer_rank: 1,
                producer: "prod".to_string(),
                producer_rank: 0,
            }))
        );
        assert_eq!(
            verdict.unwrap_err().to_string(),
            "this bag's rank manifests split the `block` edge '/g/prod/out' -> 'sink.gate' \
             across ranks ('sink' in rank 1, its producer 'prod' in rank 0), and the input sets \
             `block_timeout_ms`. Across ranks the edge counts on a shared credit word, and a \
             timeout release keeps that count exact only when the producer and the consumer \
             fire one at a time on one thread, so `cerulion graph run` refuses \
             `block_timeout_ms` on a split edge before any worker starts. `block_timeout_ms` \
             comes from the graph this bag embeds, so the bag was changed after it was \
             recorded. Re-execute an unchanged copy of the bag, or record the run again."
        );
        // The same key on the edge kept in one rank is legal there.
        assert_eq!(
            classify_split_block_edges(&config, &declared, &ranks(&[("prod", 1), ("sink", 1)])),
            Ok(vec![])
        );
    }

    /// `block_timeout_ms` on a topic two nodes write: split, it is the split
    /// refusal, asked before the validation; kept in one rank, it is the
    /// validation's shared-topic refusal.
    #[test]
    fn a_block_timeout_on_a_shared_topic_is_refused_as_split_before_validation() {
        let config = shared_bus("block_timeout_ms: 100");
        let declared = infos(&config, &[("sink", vec![meta("inp", BLOCK, true)])]);
        assert_eq!(
            classify_split_block_edges(
                &config,
                &declared,
                &ranks(&[("p1", 0), ("p2", 0), ("sink", 1)])
            ),
            Err(bus_refusal("sink", 1, (0, 0), BlockDeclaredBy::Node))
        );
        let verdict = classify_split_block_edges(
            &config,
            &declared,
            &ranks(&[("p1", 1), ("p2", 1), ("sink", 1)]),
        );
        let Err(SplitBlockEdgeError::Topology { reason }) = &verdict else {
            panic!("the shared-topic timeout is the validation's refusal: {verdict:?}");
        };
        assert!(
            reason.starts_with(
                "Graph validation error: input 'sink.inp' on topic '/g/bus' sets \
                 `block_timeout_ms`, but the topic has 2 in-graph producers ('p1', 'p2') and is \
                 listed in `multi_publisher_topics:`"
            ),
            "{reason}"
        );
    }

    /// A topic two nodes write that `multi_publisher_topics:` does not list is
    /// refused by the topology build, in its own words, before any edge is
    /// classified.
    #[test]
    fn an_unlisted_topic_two_nodes_write_is_the_topology_build_s_refusal() {
        let config = graph(
            &[],
            &[
                ("p1", Some("/g/bus"), &[]),
                ("p2", Some("/g/bus"), &[]),
                ("sink", None, &[("inp", "/g/bus", "")]),
            ],
        );
        let declared = infos(&config, &[("sink", vec![meta("inp", BLOCK, true)])]);
        assert_eq!(
            classify_split_block_edges(
                &config,
                &declared,
                &ranks(&[("p1", 0), ("p2", 0), ("sink", 1)])
            ),
            Err(SplitBlockEdgeError::Topology {
                reason: "Graph validation error: topic '/g/bus' is published by two in-graph \
                         nodes ('p1' and 'p2'); each topic must have a single producer (if \
                         multiple publishers are intentional, list the topic in \
                         `multi_publisher_topics:`)"
                    .to_string()
            })
        );
    }

    /// A depth out of range on a creditable SPLIT edge is the validation's
    /// refusal in its own words. The mint's depth refusals, which name a
    /// planning bug, are never reached.
    #[test]
    fn a_depth_out_of_range_is_the_topology_validation_s_refusal() {
        let split = ranks(&[("prod", 0), ("sink", 1)]);
        let zero = one_writer("depth: 0");
        let declared = infos(&zero, &[("sink", vec![meta("gate", BLOCK, true)])]);
        assert_eq!(
            classify_split_block_edges(&zero, &declared, &split),
            Err(SplitBlockEdgeError::Topology {
                reason: "Graph validation error: input 'sink.gate' on topic '/g/prod/out' \
                         declares depth = 0; queue depth must be >= 1"
                    .to_string()
            })
        );
        let deep = one_writer("depth: 65");
        let verdict = classify_split_block_edges(&deep, &declared, &split);
        let Err(SplitBlockEdgeError::Topology { reason }) = &verdict else {
            panic!("depth 65 is the validation's refusal: {verdict:?}");
        };
        assert!(
            reason.starts_with(
                "Graph validation error: input 'sink.gate' on topic '/g/prod/out' declares \
                 depth = 65; the maximum queue depth is 64 "
            ),
            "{reason}"
        );
    }

    /// A map that places no rank for a native node of the graph is the engine
    /// bug arm, naming the first such node in graph order: every rank lookup
    /// the check makes then has an answer, so no count or list can leave a
    /// producer out.
    #[test]
    fn a_map_that_places_no_rank_for_a_node_is_an_engine_bug_naming_it() {
        let unplaced = |node: &str| SplitBlockEdgeError::EngineBug {
            reason: format!(
                "node '{node}' is in no rank plan, and the split `block` edge check needs every \
                 native node's rank"
            ),
        };
        let config = one_writer("");
        let declared = infos(&config, &[("sink", vec![meta("gate", BLOCK, true)])]);
        let verdict = classify_split_block_edges(&config, &declared, &ranks(&[("prod", 0)]));
        assert_eq!(verdict, Err(unplaced("sink")));
        assert_eq!(
            verdict.unwrap_err().to_string(),
            "node 'sink' is in no rank plan, and the split `block` edge check needs every native \
             node's rank; this is a bug in the replay engine, please report"
        );
        let bus = shared_bus("");
        let declared_bus = infos(&bus, &[("sink", vec![meta("inp", BLOCK, true)])]);
        assert_eq!(
            classify_split_block_edges(&bus, &declared_bus, &ranks(&[("p2", 1), ("sink", 1)])),
            Err(unplaced("p1"))
        );
        // Two nodes unplaced: the first in graph order is named.
        assert_eq!(
            classify_split_block_edges(&bus, &declared_bus, &ranks(&[("p2", 1)])),
            Err(unplaced("p1"))
        );
    }

    /// Two one-writer split `block` edges, the first in topology order with no
    /// `block_timeout_ms` and the second with one: the timeout refusal names
    /// the second, so an edge with no key is passed over, not the end of the
    /// check.
    #[test]
    fn a_split_edge_without_a_timeout_is_passed_over_for_the_next_one() {
        let config = graph(
            &[],
            &[
                ("first", Some(""), &[]),
                ("second", Some(""), &[]),
                (
                    "sink",
                    None,
                    &[
                        ("a", "first/out", ""),
                        ("b", "second/out", "block_timeout_ms: 100"),
                    ],
                ),
            ],
        );
        let declared = infos(
            &config,
            &[(
                "sink",
                vec![meta("a", BLOCK, false), meta("b", BLOCK, false)],
            )],
        );
        assert_eq!(
            classify_split_block_edges(
                &config,
                &declared,
                &ranks(&[("first", 0), ("second", 0), ("sink", 1)])
            ),
            Err(SplitBlockEdgeError::Timeout(SplitEdgeTimeout {
                topic: "/g/second/out".to_string(),
                consumer_node: "sink".to_string(),
                consumer_input: "b".to_string(),
                consumer_rank: 1,
                producer: "second".to_string(),
                producer_rank: 0,
            }))
        );
    }

    /// Both split edges set `block_timeout_ms`: the first in topology order is
    /// named.
    #[test]
    fn of_two_split_edges_with_a_timeout_the_first_is_named() {
        let config = graph(
            &[],
            &[
                ("first", Some(""), &[]),
                ("second", Some(""), &[]),
                (
                    "sink",
                    None,
                    &[
                        ("a", "first/out", "block_timeout_ms: 100"),
                        ("b", "second/out", "block_timeout_ms: 100"),
                    ],
                ),
            ],
        );
        let declared = infos(
            &config,
            &[(
                "sink",
                vec![meta("a", BLOCK, false), meta("b", BLOCK, false)],
            )],
        );
        assert_eq!(
            classify_split_block_edges(
                &config,
                &declared,
                &ranks(&[("first", 0), ("second", 0), ("sink", 1)])
            ),
            Err(SplitBlockEdgeError::Timeout(SplitEdgeTimeout {
                topic: "/g/first/out".to_string(),
                consumer_node: "sink".to_string(),
                consumer_input: "a".to_string(),
                consumer_rank: 1,
                producer: "first".to_string(),
                producer_rank: 0,
            }))
        );
    }

    /// Two refusable edges: the one on the topic the topology lists first, by
    /// its first writer's declaration, is named, though its reader is declared
    /// after the other's and its topic's name sorts after the other's. Two
    /// calls, each over a map built afresh, agree.
    #[test]
    fn the_first_refusable_edge_in_topology_order_is_named_on_every_call() {
        let config = graph(
            &["/g/a", "/g/b"],
            &[
                ("p3", Some("/g/b"), &[]),
                ("p4", Some("/g/b"), &[]),
                ("p1", Some("/g/a"), &[]),
                ("p2", Some("/g/a"), &[]),
                ("sink_a", None, &[("inp", "/g/a", "")]),
                ("sink_b", None, &[("inp", "/g/b", "")]),
            ],
        );
        let declared = infos(
            &config,
            &[
                ("sink_a", vec![meta("inp", BLOCK, true)]),
                ("sink_b", vec![meta("inp", BLOCK, true)]),
            ],
        );
        let map = || {
            ranks(&[
                ("sink_b", 1),
                ("sink_a", 1),
                ("p4", 0),
                ("p3", 0),
                ("p2", 0),
                ("p1", 0),
            ])
        };
        let first = classify_split_block_edges(&config, &declared, &map());
        let second = classify_split_block_edges(&config, &declared, &map());
        let want = SplitBlockEdgeError::Uncreditable(
            UncreditableSplitEdge::new(
                "/g/b".to_string(),
                "sink_b".to_string(),
                "inp".to_string(),
                1,
                vec![("p3".to_string(), 0), ("p4".to_string(), 0)],
                BlockDeclaredBy::Node,
            )
            .expect("two producers, one in another rank"),
        );
        assert_eq!(first, Err(want.clone()));
        assert_eq!(second, Err(want));
        assert_eq!(
            first.unwrap_err().to_string(),
            second.unwrap_err().to_string()
        );
    }

    /// Two split `block` readers of one topic: the first in graph order is
    /// named, though its name sorts after the other's.
    #[test]
    fn of_two_split_block_readers_of_one_topic_the_first_declared_is_named() {
        let config = graph(
            &["/g/bus"],
            &[
                ("p1", Some("/g/bus"), &[]),
                ("p2", Some("/g/bus"), &[]),
                ("zz", None, &[("inp", "/g/bus", "")]),
                ("aa", None, &[("inp", "/g/bus", "")]),
            ],
        );
        let declared = infos(
            &config,
            &[
                ("zz", vec![meta("inp", BLOCK, true)]),
                ("aa", vec![meta("inp", BLOCK, true)]),
            ],
        );
        assert_eq!(
            classify_split_block_edges(
                &config,
                &declared,
                &ranks(&[("p1", 0), ("p2", 0), ("zz", 1), ("aa", 2)])
            ),
            Err(bus_refusal("zz", 1, (0, 0), BlockDeclaredBy::Node))
        );
    }

    /// A `block` input both the graph's key and the node's declaration set
    /// takes the graph cause: the key is what the bag embeds.
    #[test]
    fn a_block_input_the_graph_and_the_node_both_set_takes_the_graph_cause() {
        let config = shared_bus("backpressure: block");
        let declared = infos(&config, &[("sink", vec![meta("inp", BLOCK, true)])]);
        assert_eq!(
            classify_split_block_edges(
                &config,
                &declared,
                &ranks(&[("p1", 0), ("p2", 0), ("sink", 1)])
            ),
            Err(bus_refusal("sink", 1, (0, 0), BlockDeclaredBy::Graph))
        );
    }

    /// `block_timeout_ms` on a split edge of a topic `multi_publisher_topics:`
    /// lists and one node writes: creditable, so not the two-writer refusal,
    /// and the validation refuses the key on a listed topic before the mint.
    #[test]
    fn a_block_timeout_on_a_listed_one_writer_topic_is_the_validation_s_refusal() {
        let config = graph(
            &["/g/bus"],
            &[
                ("p1", Some("/g/bus"), &[]),
                ("sink", None, &[("inp", "/g/bus", "block_timeout_ms: 100")]),
            ],
        );
        let declared = infos(&config, &[("sink", vec![meta("inp", BLOCK, true)])]);
        let verdict =
            classify_split_block_edges(&config, &declared, &ranks(&[("p1", 0), ("sink", 1)]));
        let Err(SplitBlockEdgeError::Topology { reason }) = &verdict else {
            panic!("the listed-topic timeout is the validation's refusal: {verdict:?}");
        };
        assert!(
            reason.starts_with(
                "Graph validation error: input 'sink.inp' on topic '/g/bus' sets \
                 `block_timeout_ms`, but the topic has 1 in-graph producer ('p1') and is listed \
                 in `multi_publisher_topics:`"
            ),
            "{reason}"
        );
    }

    /// The refusal exists only for two or more producers with one or more in
    /// another rank than the consumer.
    #[test]
    fn the_refusal_is_built_only_for_two_producers_with_one_in_another_rank() {
        let build = |consumer_rank: usize, producers: &[(&str, usize)]| {
            UncreditableSplitEdge::new(
                "/g/bus".to_string(),
                "sink".to_string(),
                "inp".to_string(),
                consumer_rank,
                producers
                    .iter()
                    .map(|(p, r)| ((*p).to_string(), *r))
                    .collect(),
                BlockDeclaredBy::Node,
            )
        };
        assert_eq!(build(1, &[]), None, "no producer");
        assert_eq!(
            build(1, &[("p1", 0)]),
            None,
            "one producer, even a foreign one"
        );
        assert_eq!(
            build(1, &[("p1", 1), ("p2", 1)]),
            None,
            "two producers in the consumer's rank"
        );
        assert!(
            build(1, &[("p1", 0), ("p2", 1)]).is_some(),
            "two producers, one in another rank"
        );
        assert!(
            build(1, &[("p1", 0), ("p2", 2)]).is_some(),
            "two producers, both in other ranks"
        );
    }

    /// A split edge two writers write is named before a split edge whose
    /// input sets `block_timeout_ms`, even when the timeout's topic comes first:
    /// the live run refuses the partition before it mints a word.
    #[test]
    fn a_split_edge_two_writers_write_is_named_before_a_timeout() {
        let config = graph(
            &["/g/bus"],
            &[
                ("prod", Some(""), &[]),
                (
                    "sink",
                    None,
                    &[("gate", "prod/out", "block_timeout_ms: 100")],
                ),
                ("p1", Some("/g/bus"), &[]),
                ("p2", Some("/g/bus"), &[]),
                ("reader", None, &[("inp", "/g/bus", "")]),
            ],
        );
        let declared = infos(
            &config,
            &[
                ("sink", vec![meta("gate", BLOCK, true)]),
                ("reader", vec![meta("inp", BLOCK, true)]),
            ],
        );
        assert_eq!(
            classify_split_block_edges(
                &config,
                &declared,
                &ranks(&[
                    ("prod", 0),
                    ("sink", 1),
                    ("p1", 0),
                    ("p2", 0),
                    ("reader", 1)
                ])
            ),
            Err(bus_refusal("reader", 1, (0, 0), BlockDeclaredBy::Node))
        );
    }

    #[test]
    fn plan_node_ranks_maps_each_member_to_its_plan_s_rank() {
        let config = one_writer("");
        let plans = plan_ranks(
            &config,
            &[
                RankManifest {
                    rank: 1,
                    node_ids: vec!["sink".to_string()],
                },
                RankManifest {
                    rank: 0,
                    node_ids: vec!["prod".to_string()],
                },
            ],
            &[
                RankBoundarySummary::new(0, 0, 3, 10),
                RankBoundarySummary::new(1, 0, 3, 20),
            ],
        )
        .expect("two ranks plan");
        assert_eq!(plan_node_ranks(&plans), ranks(&[("prod", 0), ("sink", 1)]));
    }
}
