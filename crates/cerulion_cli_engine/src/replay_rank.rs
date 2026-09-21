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
//! caller reads off the bag ([`RankManifest`], [`RankBoundarySummary`]), and
//! the output is data. That is what lets the planner unit-test on any platform
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
//! oracle-testable.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use cerulion_core::graph::{resolve_output_topic, resolve_source, GraphConfig};

use crate::multiprocess::subgraph_for;

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
        let summary = summary_of[&rank];
        plans.push(RankPlan {
            rank,
            members,
            subgraph,
            produced,
            cross_rank_consumed,
            external_consumed,
            first_step: summary.first_step,
            last_step: summary.last_step,
            first_target_ns: summary.first_target_ns,
        });
    }
    Ok(plans)
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
