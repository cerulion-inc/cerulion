// SPDX-License-Identifier: AGPL-3.0-only
//! Which trigger edges could run as one fused synchronous chain, and why every
//! other edge cannot.
//!
//! Pure build-time analysis over [`GraphTopology`], [`TriggerEdges`],
//! [`Levels`] and the run's colocation. It decides nothing at run time, changes
//! no execution path, and is read by `cerulion graph chains` and by the run
//! directory's manifest. Nothing in the executor consumes the result yet.
//!
//! # What a fused chain is
//!
//! Inside one process, a linear single-consumer trigger chain
//! `n_0 -> n_1 -> ... -> n_k` can execute as one synchronous call sequence: the
//! executor fires `n_0`, its tick returns with the frame already committed to
//! shared memory exactly as it is committed today, and the executor then calls
//! `n_1` directly with a read-only view of those bytes instead of ending the
//! level, crossing a level boundary and receiving the frame out of `n_1`'s own
//! queue. The publish is untouched, so every observer sees the same frames in
//! the same bytes with gap-free sequences. What the chain removes is the step
//! machinery between the hops.
//!
//! This module answers only the first question: which edges qualify. Fusing
//! them is separate work.
//!
//! # The rules, and the reason each one exists
//!
//! The unit is one consumer edge: a `(topic, consumer node, consumer input)`
//! triple, exactly as [`GraphTopology`] records it. Every edge gets a verdict,
//! and an edge that does not qualify carries the [`ChainBar`] that refused it,
//! so the census is total: the bars partition the graph's consumer edges.
//!
//! | rule | bar | why |
//! |---|---|---|
//! | the edge is a trigger edge | [`ChainBar::LatestValueRead`] | a non-trigger input is a latest-value read, not a hop: the consumer is not woken by it |
//! | the topic has exactly one in-graph producer | [`ChainBar::NoInGraphProducer`], [`ChainBar::MultipleProducers`] | an outside publisher hands over no slot, and two writers give the consumer no single producer to be called from |
//! | the topic has exactly one consumer edge | [`ChainBar::FanOut`] | the second consumer would wait for the first consumer's whole chain, which is a scheduling decision the level structure already makes |
//! | the producer feeds exactly one fusable edge | [`ChainBar::ProducerBranches`] | the same wait, one level up: a node publishing two single-consumer topics is a fan-out across topics, and a chain ends at a fan-out |
//! | producer and consumer run in one process | [`ChainBar::SeparateProcesses`] | another address space has no slot to hand over |
//! | the edge is neither `block` nor `sample(N)` | [`ChainBar::BlockEdge`], [`ChainBar::SampleEdge`] | `block` means "defer the producer while the consumer is at threshold", and a consumer that runs inside the producer's own fire can never reach the threshold, so the declared contract could not be honoured; `sample(N)` states that the consumer reads less often than the producer publishes, and a synchronous call reads in lockstep with it |
//! | the consumer fires on this frame | [`ChainBar::ConsumerPolicy`] | a period is a clock decision, a sync window is an alignment decision across several edges, and an external node fires from outside the graph |
//! | this edge is the consumer's only trigger | [`ChainBar::ConsumerJoin`] | a join fires on a set of frames, not on one; the chain ends at it and its inbound edges stay queued |
//! | the consumer declares no `throttle_ms` | [`ChainBar::ConsumerThrottled`] | a rate cap is a decision to defer a fire, and a synchronous call has nowhere to defer to |
//! | neither end is block-involved | [`ChainBar::BlockInvolved`] | the executor already fires those nodes serially against a shared outstanding count, and a chain would reorder that pairing |
//! | the consumer reads no context written at or after the head's level | [`ChainBar::LateContextInput`] | the determinism rule, derived below |
//! | the chain is within [`MAX_FUSED_CHAIN_NODES`] | [`ChainBar::LengthCeiling`] | the synchronous call depth is the chain length, so the depth is bounded at build time |
//!
//! Every one of these is decidable from the graph's declarations, so the set is
//! frozen at build and never re-derived from what the run happens to do. A
//! fusion set that varied with load would make a replay a different program.
//!
//! One exclusion a fused executor would also need is NOT a rule here: a
//! consumer that keeps a borrow of its trigger input alive past its own tick
//! could not be served by a view that ends with the call. No declaration says
//! that, and the accessors a node reads a trigger input through hand out a
//! view scoped to the tick, so there is nothing for this analysis to read.
//! Should such a declaration exist, it belongs in the table above.
//!
//! # The context rule, derived
//!
//! The level executor snapshots a firing node's non-trigger inputs at the start
//! of its OWN level phase, which is after every node at every earlier level has
//! ticked. A fused consumer at level `L+1` is called during level `L`'s tick
//! phase, when only part of level `L` has ticked. If that consumer reads a
//! latest-value input written by another node at level `L`, the queued run and
//! the fused run can observe different values, and the two stop being the same
//! program.
//!
//! The rule closes it by refusal rather than by cleverness: a fused consumer
//! may read no non-trigger input whose in-graph producer sits at a level at or
//! after the CHAIN HEAD's level. Everything below the head has ticked and
//! committed before the head fires, so its values are final; everything at or
//! after the head has not. The head is the bound, not the consumer's own level,
//! because the whole chain runs inside the head's level phase.
//!
//! The rule is scoped to in-graph producers, which is what level order decides.
//! A non-trigger input fed from outside the graph is read at a slightly earlier
//! instant of the same step; that is a change of when inside one step, not of
//! which step, and it is measured by the executor's own firewall test when the
//! chain is actually fused.
//!
//! An edge refused here CUTS its chain rather than killing it: the refused
//! consumer becomes the head of a new chain, which is judged against its own
//! level. The same applies to the length ceiling.
//!
//! # Why the verdicts add up
//!
//! Every surviving edge is either a hop of exactly one chain or carries a bar,
//! with nothing left over. A consumer has at most one trigger edge, so at most
//! one inbound surviving edge; a producer is left at most one outbound
//! surviving edge; and every surviving edge steps strictly one level up. So
//! the surviving edges are disjoint PATHS, each walked once from its start.
//!
//! # Determinism
//!
//! Edges are walked in [`GraphTopology`] order, which is graph order, and every
//! collection here preserves insertion order, so two analyses of one graph
//! produce byte-identical output.

use indexmap::{IndexMap, IndexSet};

use crate::graph::config::{GraphConfig, NodeDef};
use crate::graph::node::{BackpressurePolicy, MacroPolicy, NodeInfo};
use crate::graph::topology::{ConsumerEdge, GraphTopology, Levels, TopicFlow, TriggerEdges};

use super::resolve_source;

/// The longest chain the executor may fuse, in nodes.
///
/// A fused hop is a direct call, so the call depth of a chain is its length and
/// a graph could otherwise declare an unbounded one. A chain longer than this
/// is SPLIT at the ceiling: the boundary edge keeps the queued path and the
/// remainder starts a new chain, so nothing is silently dropped and the depth
/// of any single chain stays bounded.
///
/// The value is far above any shape a real graph reaches: a linear chain of 32
/// nodes is longer than the whole node count of every graph shipped with this
/// repository, so the ceiling costs nothing today and exists so that a
/// generated graph cannot hand the executor an unbounded recursion.
pub const MAX_FUSED_CHAIN_NODES: usize = 32;

/// Where the run puts each node, for the one-process rule.
#[derive(Debug, Clone, Copy)]
pub enum Colocation<'a> {
    /// One process holds every node, so every pair is colocated. This is what
    /// a `--single-process` run and a graph with no `process_groups:` block
    /// executed as a monolith look like.
    SingleProcess,
    /// The run executes this `process_groups:` partition: two nodes are
    /// colocated when one group lists both.
    ProcessGroups(&'a IndexMap<String, Vec<String>>),
}

impl<'a> Colocation<'a> {
    /// Where `node_id` runs.
    fn placement(self, node_id: &str) -> Placement<'a> {
        match self {
            Self::SingleProcess => Placement::Monolith,
            Self::ProcessGroups(groups) => groups
                .iter()
                .find(|(_, members)| members.iter().any(|m| m == node_id))
                .map(|(name, _)| Placement::Group(name.as_str()))
                .unwrap_or(Placement::Unplaced),
        }
    }

    /// The group name to print for `node_id`, or `None` when the run holds
    /// every node in one process or the node is in no group.
    fn group_name(self, node_id: &str) -> Option<String> {
        match self.placement(node_id) {
            Placement::Group(g) => Some(g.to_string()),
            Placement::Monolith | Placement::Unplaced => None,
        }
    }
}

/// Where one node runs. Private: callers state the deployment through
/// [`Colocation`] and read the verdict, never this.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Placement<'a> {
    /// One process holds every node.
    Monolith,
    /// A member of this process group.
    Group(&'a str),
    /// A partition is in force and this node is in none of its groups. Never
    /// colocated with anything, including another unplaced node: a node with no
    /// declared group cannot be PROVEN to share a process, and a partition that
    /// leaves a node out is refused at graph load anyway.
    Unplaced,
}

/// How a consumer decides to fire, for the rule that a fused consumer fires on
/// the frame it is handed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConsumerFireClass {
    /// Fires on a clock.
    Period,
    /// Fires on an aligned set within a window.
    Sync,
    /// Fires on a complete set, with no timing bound.
    UnboundedSync,
    /// Fires from outside the graph.
    External,
}

impl ConsumerFireClass {
    /// The word this class prints as.
    pub fn label(self) -> &'static str {
        match self {
            Self::Period => "period",
            Self::Sync => "sync",
            Self::UnboundedSync => "unbounded sync",
            Self::External => "external",
        }
    }
}

/// Why one consumer edge cannot be a fused hop.
///
/// Each variant carries what its sentence has to name, so a reader never has to
/// join the reason back to the graph by hand.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ChainBar {
    /// The edge is not a trigger edge: the consumer reads the last published
    /// value and is not woken by it, so there is no hop to fuse.
    LatestValueRead,
    /// No in-graph node publishes the topic.
    NoInGraphProducer,
    /// Several in-graph nodes publish the topic. Always two or more.
    MultipleProducers { producers: usize },
    /// The topic carries several consumer edges. Always two or more.
    FanOut { consumers: usize },
    /// The producer feeds several otherwise-fusable edges on different topics,
    /// so a chain through it would make one consumer wait for the other's whole
    /// chain. Always two or more.
    ProducerBranches { branches: usize },
    /// The two ends run in different processes. A `None` group name means the
    /// node is in no declared group.
    SeparateProcesses {
        producer_group: Option<String>,
        consumer_group: Option<String>,
    },
    /// The consumer declared `block` on this input.
    BlockEdge,
    /// The consumer declared `sample(N)` on this input.
    SampleEdge { window_ms: u64 },
    /// The consumer does not fire on this frame.
    ConsumerPolicy { policy: ConsumerFireClass },
    /// The consumer has several trigger edges, so it fires on a set of frames.
    /// Always two or more.
    ConsumerJoin { trigger_edges: usize },
    /// The consumer declares a producer-side rate cap.
    ConsumerThrottled { throttle_ms: u64 },
    /// One end is involved in a `block` topic and is already fired serially
    /// against the shared outstanding count.
    BlockInvolved { producer: bool, consumer: bool },
    /// The consumer reads a latest-value input written by a node at or after
    /// the chain head's level, so a fused call would read it before it is
    /// final.
    LateContextInput {
        input: String,
        written_by: String,
        written_at_level: usize,
        head_level: usize,
    },
    /// Extending the chain would pass [`MAX_FUSED_CHAIN_NODES`]. The consumer
    /// starts a new chain instead.
    LengthCeiling { ceiling: usize },
    /// The supplied levelization places no level on this node, so neither the
    /// head's level nor the context rule can be decided for it.
    ///
    /// Unreachable from the graph build, whose levelization covers every node
    /// in the graph. It exists because this analysis is callable with a
    /// levelization and a topology derived from different graphs, and a partial
    /// levelization must refuse the edge rather than silently switch the
    /// context rule off for it.
    LevelUnknown { node: String },
    /// The supplied levelization does not place the consumer after the
    /// producer, so the edge is not a hop that levelization agrees with.
    ///
    /// A trigger edge always steps exactly one level in a levelization derived
    /// from the same graph, so this is the other half of the mismatch
    /// [`ChainBar::LevelUnknown`] covers. Refusing it is also what keeps the
    /// surviving edges a set of disjoint PATHS: a cycle needs one edge that
    /// does not increase the level, and an unwalkable cycle would leave edges
    /// that are neither a hop nor refused.
    LevelNotIncreasing {
        producer_level: usize,
        consumer_level: usize,
    },
}

impl ChainBar {
    /// The census key: a stable, greppable word for this bar.
    ///
    /// Counting is keyed on this rather than on the rendered sentence, so a
    /// count stays comparable across graphs whose sentences name different
    /// nodes.
    pub fn label(&self) -> &'static str {
        match self {
            Self::LatestValueRead => "latest-value-read",
            Self::NoInGraphProducer => "no-in-graph-producer",
            Self::MultipleProducers { .. } => "multiple-producers",
            Self::FanOut { .. } => "fan-out",
            Self::ProducerBranches { .. } => "producer-branches",
            Self::SeparateProcesses { .. } => "separate-processes",
            Self::BlockEdge => "block",
            Self::SampleEdge { .. } => "sample",
            Self::ConsumerPolicy { .. } => "consumer-policy",
            Self::ConsumerJoin { .. } => "join",
            Self::ConsumerThrottled { .. } => "throttle",
            Self::BlockInvolved { .. } => "block-involved",
            Self::LateContextInput { .. } => "late-context-input",
            Self::LengthCeiling { .. } => "length-ceiling",
            Self::LevelUnknown { .. } => "level-unknown",
            Self::LevelNotIncreasing { .. } => "level-not-increasing",
        }
    }
}

impl std::fmt::Display for ChainBar {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::LatestValueRead => write!(
                f,
                "the consumer reads this topic as a latest value and is not woken by it"
            ),
            Self::NoInGraphProducer => write!(
                f,
                "no node in this graph publishes the topic, so there is no producer to be called from"
            ),
            Self::MultipleProducers { producers } => write!(
                f,
                "{producers} nodes in this graph publish the topic, so the consumer has no single producer to be called from"
            ),
            Self::FanOut { consumers } => write!(
                f,
                "the topic has {consumers} consumers, and a chain ends at a fan-out"
            ),
            Self::ProducerBranches { branches } => write!(
                f,
                "the producer feeds {branches} single-consumer topics, and a chain ends at a fan-out"
            ),
            Self::SeparateProcesses {
                producer_group,
                consumer_group,
            } => {
                let side = |g: &Option<String>| match g {
                    Some(name) => format!("group '{name}'"),
                    None => "no declared group".to_string(),
                };
                write!(
                    f,
                    "the producer runs in {} and the consumer in {}",
                    side(producer_group),
                    side(consumer_group)
                )
            }
            Self::BlockEdge => write!(
                f,
                "the input declares `block`, whose deferral a consumer running inside the producer's own fire could never reach"
            ),
            Self::SampleEdge { window_ms } => write!(
                f,
                "the input declares `sample({window_ms})`, which reads less often than the producer publishes"
            ),
            Self::ConsumerPolicy { policy } => write!(
                f,
                "the consumer fires on {}, not on this frame",
                policy.label()
            ),
            Self::ConsumerJoin { trigger_edges } => write!(
                f,
                "the consumer has {trigger_edges} trigger inputs, so it fires on a set of frames"
            ),
            Self::ConsumerThrottled { throttle_ms } => write!(
                f,
                "the consumer declares `throttle_ms = {throttle_ms}`, and a direct call has nowhere to defer to"
            ),
            Self::BlockInvolved { producer, consumer } => {
                let who = match (*producer, *consumer) {
                    (true, true) => "both ends are",
                    (true, false) => "the producer is",
                    _ => "the consumer is",
                };
                write!(
                    f,
                    "{who} on a `block` topic and already fired serially against the shared outstanding count"
                )
            }
            Self::LateContextInput {
                input,
                written_by,
                written_at_level,
                head_level,
            } => write!(
                f,
                "the consumer reads '{input}' as a latest value and '{written_by}' writes it at level {written_at_level}, at or after the chain head's level {head_level}"
            ),
            Self::LengthCeiling { ceiling } => write!(
                f,
                "the chain already holds {ceiling} nodes, the most one chain may hold; the consumer starts a new chain"
            ),
            Self::LevelUnknown { node } => write!(
                f,
                "the levelization places no level on '{node}'"
            ),
            Self::LevelNotIncreasing {
                producer_level,
                consumer_level,
            } => write!(
                f,
                "the levelization places the consumer at level {consumer_level}, not after the producer's level {producer_level}"
            ),
        }
    }
}

/// One consumer edge and what the analysis decided about it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConsumerEdgeVerdict {
    /// The resolved topic name.
    pub topic: String,
    /// The topic's single in-graph producer, when it has exactly one.
    pub producer: Option<String>,
    /// The consuming node's id.
    pub consumer: String,
    /// The consuming node's input field name.
    pub input: String,
    /// `None` when the edge is a fused hop; otherwise why it is not.
    pub bar: Option<ChainBar>,
}

impl ConsumerEdgeVerdict {
    /// True when this edge is a fused hop.
    pub fn is_fused(&self) -> bool {
        self.bar.is_none()
    }
}

/// One chain that qualifies for fused execution, head first.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FusedChain {
    /// The chain's nodes, `n_0` (the head) first. Always two or more.
    pub nodes: Vec<String>,
    /// The hop topics, `topics[i]` carrying `nodes[i] -> nodes[i + 1]`. Always
    /// one shorter than `nodes`.
    pub topics: Vec<String>,
    /// The head's DAG level. Chain order and level order coincide for a linear
    /// chain, so `nodes[i]` sits at `head_level + i`.
    pub head_level: usize,
}

impl FusedChain {
    /// The number of hops, one fewer than the node count.
    pub fn hops(&self) -> usize {
        self.topics.len()
    }
}

/// The result of the analysis: the chains that qualify, and a verdict for every
/// consumer edge in the graph.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ChainCensus {
    chains: Vec<FusedChain>,
    edges: Vec<ConsumerEdgeVerdict>,
}

impl ChainCensus {
    /// The chains that qualify, in graph order of their heads.
    pub fn chains(&self) -> &[FusedChain] {
        &self.chains
    }

    /// Every consumer edge in the graph, in graph order, each with its verdict.
    pub fn edges(&self) -> &[ConsumerEdgeVerdict] {
        &self.edges
    }

    /// How many consumer edges the graph declares. The denominator of the
    /// census.
    pub fn consumer_edge_count(&self) -> usize {
        self.edges.len()
    }

    /// How many consumer edges are fused hops.
    pub fn fused_hop_count(&self) -> usize {
        self.chains.iter().map(FusedChain::hops).sum()
    }

    /// How many consumer edges keep the queued path.
    ///
    /// Counted from the verdicts rather than subtracted from the total, so the
    /// two halves of the census are read off the same values and cannot
    /// disagree.
    pub fn queued_edge_count(&self) -> usize {
        self.edges.iter().filter(|e| e.bar.is_some()).count()
    }

    /// The node count of the longest chain, or 0 when nothing qualifies.
    pub fn longest_chain_nodes(&self) -> usize {
        self.chains.iter().map(|c| c.nodes.len()).max().unwrap_or(0)
    }

    /// The bars that refused edges, most frequent first and then by label, so
    /// two runs of one graph render identically.
    pub fn bars(&self) -> Vec<(&'static str, usize)> {
        let mut counts: IndexMap<&'static str, usize> = IndexMap::new();
        for edge in &self.edges {
            if let Some(bar) = &edge.bar {
                *counts.entry(bar.label()).or_insert(0) += 1;
            }
        }
        let mut out: Vec<(&'static str, usize)> = counts.into_iter().collect();
        out.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.cmp(b.0)));
        out
    }
}

/// Decide which of the graph's consumer edges could run as fused chains.
///
/// Pure: it reads the graph's declarations and returns a verdict for every
/// consumer edge plus the chains those verdicts assemble into. It opens no
/// transport, reads no clock and changes no execution path.
///
/// `entry_infos` carries each node's macro-declared policy, per-input trigger
/// marks, backpressure and rate cap, and is the same map
/// [`GraphTopology::build`] and
/// [`build_trigger_edges`](crate::graph::build_trigger_edges) are given.
/// `levels` must be the levelization the run will execute.
pub fn census_chains(
    config: &GraphConfig,
    entry_infos: &IndexMap<String, NodeInfo>,
    topology: &GraphTopology,
    trigger_edges: &TriggerEdges,
    levels: &Levels,
    colocation: Colocation<'_>,
) -> ChainCensus {
    let node_defs: IndexMap<&str, &NodeDef> =
        config.native_nodes().map(|n| (n.id.as_str(), n)).collect();

    // A node is block-involved when it consumes a `block` input or publishes a
    // topic some node consumes as `block`. The executor fires exactly that set
    // serially against the shared outstanding count, and both halves are
    // readable off the topology, which was built from the same declarations the
    // executor reads.
    let mut block_involved: IndexSet<String> = IndexSet::new();
    for flow in topology.topics() {
        if !flow.has_block_consumer() {
            continue;
        }
        for producer in &flow.producers {
            block_involved.insert(producer.clone());
        }
        for consumer in &flow.consumers {
            if matches!(consumer.policy, BackpressurePolicy::Block) {
                block_involved.insert(consumer.node_id.clone());
            }
        }
    }

    // How many trigger edges each node has. A consumer with more than one fires
    // on a set of frames, which is the join rule.
    let mut trigger_edge_count: IndexMap<String, usize> = IndexMap::new();
    for flow in topology.topics() {
        for consumer in &flow.consumers {
            if trigger_edges.is_triggering(&consumer.node_id, &flow.topic) {
                *trigger_edge_count
                    .entry(consumer.node_id.clone())
                    .or_insert(0) += 1;
            }
        }
    }

    let cx = EdgeContext {
        entry_infos,
        trigger_edges,
        levels,
        colocation,
        block_involved: &block_involved,
        trigger_edge_count: &trigger_edge_count,
    };

    let mut edges: Vec<ConsumerEdgeVerdict> = Vec::new();
    for flow in topology.topics() {
        let sole_producer = flow.sole_producer().map(str::to_string);
        for consumer in &flow.consumers {
            let bar = per_edge_bar(flow, consumer, sole_producer.as_deref(), &cx);
            edges.push(ConsumerEdgeVerdict {
                topic: flow.topic.clone(),
                producer: sole_producer.clone(),
                consumer: consumer.node_id.clone(),
                input: consumer.input.clone(),
                bar,
            });
        }
    }

    refuse_branching_producers(&mut edges);
    let chains = assemble_chains(
        &mut edges,
        config,
        &node_defs,
        trigger_edges,
        levels,
        topology,
    );

    ChainCensus { chains, edges }
}

/// What every per-edge rule reads, gathered once so the rules take one
/// argument instead of nine.
#[derive(Clone, Copy)]
struct EdgeContext<'a, 'g> {
    entry_infos: &'a IndexMap<String, NodeInfo>,
    trigger_edges: &'a TriggerEdges,
    levels: &'a Levels,
    colocation: Colocation<'g>,
    /// Nodes the executor already fires serially against a shared outstanding
    /// count.
    block_involved: &'a IndexSet<String>,
    /// How many trigger edges each node has.
    trigger_edge_count: &'a IndexMap<String, usize>,
}

/// The rules decidable from one edge alone, asked in the order an operator
/// should act on them: what the topic is (writers, then readers), then where
/// its ends run, then what the edge and the consumer declare.
///
/// The first bar found is the one reported. Reporting every bar an edge trips
/// would make the census a multiset that cannot be added up, and the order is
/// chosen so the reported bar is the one closest to the cause: a topic with two
/// writers is not usefully described as a policy problem.
fn per_edge_bar(
    flow: &TopicFlow,
    consumer: &ConsumerEdge,
    sole_producer: Option<&str>,
    cx: &EdgeContext<'_, '_>,
) -> Option<ChainBar> {
    let EdgeContext {
        entry_infos,
        trigger_edges,
        levels,
        colocation,
        block_involved,
        trigger_edge_count,
    } = *cx;
    if !trigger_edges.is_triggering(&consumer.node_id, &flow.topic) {
        return Some(ChainBar::LatestValueRead);
    }
    let producer = match (flow.producers.len(), sole_producer) {
        (0, _) => return Some(ChainBar::NoInGraphProducer),
        (1, Some(p)) => p,
        (n, _) => return Some(ChainBar::MultipleProducers { producers: n }),
    };
    if flow.sole_consumer().is_none() {
        return Some(ChainBar::FanOut {
            consumers: flow.consumers.len(),
        });
    }
    let Some(producer_level) = levels.level_of(producer) else {
        return Some(ChainBar::LevelUnknown {
            node: producer.to_string(),
        });
    };
    let Some(consumer_level) = levels.level_of(&consumer.node_id) else {
        return Some(ChainBar::LevelUnknown {
            node: consumer.node_id.clone(),
        });
    };
    if consumer_level <= producer_level {
        return Some(ChainBar::LevelNotIncreasing {
            producer_level,
            consumer_level,
        });
    }
    let producer_place = colocation.placement(producer);
    let consumer_place = colocation.placement(&consumer.node_id);
    let colocated = match (producer_place, consumer_place) {
        (Placement::Monolith, Placement::Monolith) => true,
        (Placement::Group(a), Placement::Group(b)) => a == b,
        _ => false,
    };
    if !colocated {
        return Some(ChainBar::SeparateProcesses {
            producer_group: colocation.group_name(producer),
            consumer_group: colocation.group_name(&consumer.node_id),
        });
    }
    match consumer.policy {
        BackpressurePolicy::Block => return Some(ChainBar::BlockEdge),
        BackpressurePolicy::Sample(window_ms) => return Some(ChainBar::SampleEdge { window_ms }),
        BackpressurePolicy::DropOldest => {}
    }
    let info = entry_infos.get(&consumer.node_id);
    // An absent macro policy is the runtime's Data fallback: the node fires on
    // any input arrival, which IS "the producer committed a frame" when the
    // node has exactly one trigger edge. The join rule below is what judges it,
    // so the fallback is not a separate bar.
    if let Some(policy) = info.and_then(NodeInfo::policy) {
        let class = match policy {
            MacroPolicy::Period { .. } => Some(ConsumerFireClass::Period),
            MacroPolicy::Sync { .. } => Some(ConsumerFireClass::Sync),
            MacroPolicy::UnboundedSync => Some(ConsumerFireClass::UnboundedSync),
            MacroPolicy::External => Some(ConsumerFireClass::External),
            MacroPolicy::DataTrigger { .. } => None,
        };
        if let Some(policy) = class {
            return Some(ChainBar::ConsumerPolicy { policy });
        }
    }
    let triggers = trigger_edge_count
        .get(consumer.node_id.as_str())
        .copied()
        .unwrap_or(0);
    if triggers > 1 {
        return Some(ChainBar::ConsumerJoin {
            trigger_edges: triggers,
        });
    }
    if let Some(throttle_ms) = info.and_then(NodeInfo::throttle_ms) {
        return Some(ChainBar::ConsumerThrottled { throttle_ms });
    }
    let producer_blocked = block_involved.contains(producer);
    let consumer_blocked = block_involved.contains(consumer.node_id.as_str());
    if producer_blocked || consumer_blocked {
        return Some(ChainBar::BlockInvolved {
            producer: producer_blocked,
            consumer: consumer_blocked,
        });
    }
    None
}

/// Refuse every edge of a producer that feeds two or more otherwise-fusable
/// edges.
///
/// A topic-level fan-out is refused by [`ChainBar::FanOut`] because the second
/// consumer would wait for the first consumer's whole chain. A node publishing
/// two single-consumer topics is the same wait one level up, and it is also
/// what makes "a node belongs to at most one chain" true: a consumer has at
/// most one trigger edge and therefore at most one inbound fused edge, so once
/// a producer has at most one outbound fused edge, the fused edges form
/// disjoint paths.
///
/// Both branches are refused rather than one being picked, because picking one
/// would need a rule for which, and a chain ending at a fan-out is the rule
/// already applied one level down.
fn refuse_branching_producers(edges: &mut [ConsumerEdgeVerdict]) {
    // Counted over the SURVIVING edges only: a second output whose consumer
    // reads it as a latest value, or across a process boundary, is no branch,
    // and counting it would cost a chain for nothing.
    let mut outbound: IndexMap<&str, usize> = IndexMap::new();
    for edge in edges.iter() {
        if !edge.is_fused() {
            continue;
        }
        if let Some(producer) = edge.producer.as_deref() {
            *outbound.entry(producer).or_insert(0) += 1;
        }
    }
    let branching: Vec<(String, usize)> = outbound
        .into_iter()
        .filter(|(_, n)| *n > 1)
        .map(|(p, n)| (p.to_string(), n))
        .collect();
    for edge in edges.iter_mut() {
        if !edge.is_fused() {
            continue;
        }
        let branches = edge
            .producer
            .as_deref()
            .and_then(|producer| branching.iter().find(|(p, _)| p.as_str() == producer))
            .map(|(_, n)| *n);
        if let Some(branches) = branches {
            edge.bar = Some(ChainBar::ProducerBranches { branches });
        }
    }
}

/// Link the surviving edges into chains, applying the context rule and the
/// length ceiling as the walk goes.
///
/// After [`refuse_branching_producers`] every surviving edge has one producer
/// and one consumer, every consumer is the consumer of at most one surviving
/// edge, and every producer is the producer of at most one. Every surviving
/// edge also steps strictly one level up, so no cycle of them exists: the
/// surviving edges are disjoint PATHS, and every one of them is reached. A
/// head is a producer that consumes no surviving edge; the walk follows each
/// head to its end.
fn assemble_chains(
    edges: &mut [ConsumerEdgeVerdict],
    config: &GraphConfig,
    node_defs: &IndexMap<&str, &NodeDef>,
    trigger_edges: &TriggerEdges,
    levels: &Levels,
    topology: &GraphTopology,
) -> Vec<FusedChain> {
    // producer -> the index of its one surviving edge, in graph order.
    let mut next_hop: IndexMap<String, usize> = IndexMap::new();
    let mut is_consumer: IndexSet<String> = IndexSet::new();
    for (idx, edge) in edges.iter().enumerate() {
        if !edge.is_fused() {
            continue;
        }
        if let Some(producer) = edge.producer.as_ref() {
            next_hop.insert(producer.clone(), idx);
            is_consumer.insert(edge.consumer.clone());
        }
    }

    let heads: Vec<String> = next_hop
        .keys()
        .filter(|p| !is_consumer.contains(p.as_str()))
        .cloned()
        .collect();

    let mut chains: Vec<FusedChain> = Vec::new();
    // Every node is visited at most once: the walk only ever steps onto a
    // consumer, and a consumer has at most one inbound surviving edge. The set
    // is kept anyway so a levelization inconsistent with the topology cannot
    // turn a cycle into an unbounded walk.
    let mut walked: IndexSet<String> = IndexSet::new();

    for head in heads {
        let mut open: Option<FusedChain> = None;
        let mut current = head;
        if !walked.insert(current.clone()) {
            continue;
        }
        while let Some(&idx) = next_hop.get(&current) {
            let consumer = edges[idx].consumer.clone();
            if !walked.insert(consumer.clone()) {
                break;
            }
            let chain_head = open
                .as_ref()
                .map(|c| c.nodes[0].clone())
                .unwrap_or_else(|| current.clone());
            // Defence in depth: every surviving edge had both of its ends
            // placed by `per_edge_bar`, and a chain head is always one of those
            // ends, so this lookup answers. It refuses rather than assumes
            // anyway, because assuming a level here would switch the context
            // rule off for the whole chain.
            let Some(head_level) = levels.level_of(&chain_head) else {
                edges[idx].bar = Some(ChainBar::LevelUnknown { node: chain_head });
                break;
            };
            let at_ceiling = open
                .as_ref()
                .map(|c| c.nodes.len() >= MAX_FUSED_CHAIN_NODES)
                .unwrap_or(false);
            let cut = if at_ceiling {
                Some(ChainBar::LengthCeiling {
                    ceiling: MAX_FUSED_CHAIN_NODES,
                })
            } else {
                late_context_bar(
                    &consumer,
                    head_level,
                    config,
                    node_defs,
                    trigger_edges,
                    levels,
                    topology,
                )
            };
            if let Some(bar) = cut {
                edges[idx].bar = Some(bar);
                if let Some(chain) = open.take() {
                    chains.push(chain);
                }
                // The refused consumer is not fused INTO this chain, but it may
                // head one of its own: it is judged against its own level, and
                // everything below that level has ticked before it fires.
                current = consumer;
                continue;
            }
            let chain = open.get_or_insert_with(|| FusedChain {
                nodes: vec![current.clone()],
                topics: Vec::new(),
                head_level,
            });
            chain.nodes.push(consumer.clone());
            chain.topics.push(edges[idx].topic.clone());
            current = consumer;
        }
        if let Some(chain) = open.take() {
            chains.push(chain);
        }
    }
    chains
}

/// The context rule for one candidate consumer: does it read a latest-value
/// input written by a node at or after `head_level`?
///
/// Reports the FIRST offending input in graph order, so the message names one
/// concrete thing to move rather than a list.
fn late_context_bar(
    consumer: &str,
    head_level: usize,
    config: &GraphConfig,
    node_defs: &IndexMap<&str, &NodeDef>,
    trigger_edges: &TriggerEdges,
    levels: &Levels,
    topology: &GraphTopology,
) -> Option<ChainBar> {
    // Every consumer edge in the topology was recorded from a native node, so
    // this lookup answers; a node with no declaration reads no context.
    let node_def = node_defs.get(consumer)?;
    for input in &node_def.inputs {
        let topic = resolve_source(&config.prefix, &input.source);
        if trigger_edges.is_triggering(consumer, &topic) {
            continue;
        }
        for writer in topology.producers_of(&topic) {
            let Some(level) = levels.level_of(writer) else {
                return Some(ChainBar::LevelUnknown {
                    node: writer.clone(),
                });
            };
            if level >= head_level {
                return Some(ChainBar::LateContextInput {
                    input: input.name.clone(),
                    written_by: writer.clone(),
                    written_at_level: level,
                    head_level,
                });
            }
        }
    }
    None
}
