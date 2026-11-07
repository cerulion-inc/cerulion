// SPDX-License-Identifier: AGPL-3.0-only
//! Executor units: which nodes of one process share a thread.
//!
//! A free-run deployment runs every unit on its own executor. An executor is
//! one thread driving one [`GraphRuntime::run_live`](crate::GraphRuntime::run_live)
//! over the nodes of its unit, with its own step counter, its own gating clock
//! and its own trace ring, exactly as one worker process does. Inside a unit the
//! level loop runs as it always has: a producer's frame is read by its consumer
//! at the next level of the same step, on the same thread, with no wake. Between
//! units the frame crosses the same bounded drop-oldest queue an edge between
//! processes crosses, and each unit waits only on its own inputs, its own timer,
//! its own watchdog deadlines, its own external sources and its own credit
//! words. A slow tick delays its own unit and, by data, the units that read from
//! it; it never holds another unit's fire.
//!
//! # Units are opt in
//!
//! Without a profile the nodes of a process form ONE unit, the level loop a
//! process runs its nodes on when it hosts no units, except each node that
//! declares `isolate: true`, which forms a unit of its own. A [`Coupling`]
//! still joins its nodes, and `isolate: false` changes nothing. The unit
//! carries [`Boundary::NoProfile`] on its first member and each isolated unit
//! [`Boundary::Isolated`]. A unit runs a contiguous band of the graph's
//! levels, and the planner keeps a node above its band's first level in the
//! unit only where a node of the unit feeds it at the level just below. An
//! isolated node can be a node's only feeder at that level, and an isolated
//! node that runs in a unit of its own takes no other node, so a node past an
//! isolated node's level joins the earliest unit of the other nodes that
//! starts at its level or holds a node feeding it at the level below, and
//! otherwise starts a later unit, carrying [`Boundary::AfterIsolatedLevel`];
//! the first unit then carries [`Boundary::NoProfileCut`] in place of
//! [`Boundary::NoProfile`]. With a profile the rules below split and merge.
//!
//! A unit is woken by the kernel on ONE input, its first declared trigger or
//! sync input another unit or process writes; a frame on any other input is
//! seen when its current wait ends (`GraphRuntime::doorbell_wake_word_block`).
//! A join with two or more such inputs, as one whose unit holds none of its
//! producers (it is a unit of its own, or its producers in its process all
//! declare `isolate: true`), therefore waits up to one recheck for a set that
//! completes on any of them but the first
//! (<https://github.com/cerulion-inc/cerulion/issues/755>).
//!
//! # With a profile, every node starts alone and the profile merges links
//!
//! A node may join its producer's unit only across a LINK: an edge the shared
//! chain predicates ([`crate::graph::chain::linear_links`]) find to be a linear
//! hand-off, one producer handing each frame to one consumer that fires on it.
//! The fusion census and the partitioner start from the same link predicates;
//! the census then adds its own fusion rules, so every fused chain is a run of
//! links.
//!
//! A link merges only when the graph's profile ([`PartitionCosts`], the cost
//! snapshot `graph profile` writes) shows the merge is safe: the unit's summed
//! p50 tick, the new node included, stays under [`BUDGET_DIVISOR`]⁻¹ of the
//! shortest period the merged unit is fed at, and under the same share of the
//! tightest `tick_within_ms` of its members. A link whose nodes the profile
//! holds no tick for does not merge.
//!
//! A unit is fed at each member's own period, else through each trigger edge
//! into the member: at the edge's profiled rate, or, when the profile does not
//! rate it, from another node of the process at the period that node's group
//! is fed at before any merge (the node and the nodes `isolate: false` and
//! couplings joined to it), from another process at an unknown rate. A member
//! no graph node triggers, or one with a trigger input no graph node
//! publishes, is also fed at its fire rate: the fastest profiled rate of its
//! trigger edges out, unknown when the profile rates none. Every member's
//! feeds are gathered before the first merge, so a merge is judged against
//! every feed of the merged unit whichever chain is walked first.
//!
//! Each node that starts a unit carries the reason ([`Boundary`]): a source, an
//! edge that is no link (latest-value, fan-out, join, a period or sync
//! consumer...), `isolate: true`, an isolated producer, an unprofiled merge, a
//! merge into a unit fed at a rate the profile does not hold, a merge over
//! budget, or a cycle of links that no chain head reaches. A node in one unit
//! with a node that triggers it starts no unit and carries no reason.
//!
//! # Overrides
//!
//! `isolate: true` puts the node in a unit of its own, with or without a
//! profile. With a profile, `isolate: false` joins the node to the unit of
//! every same-process node that triggers it, whatever the link rules say: a
//! fan-in node merges its producers' units, a fan-out consumer shares the
//! producer's unit, and the tick budget is not applied. When a producer
//! declares `isolate: true` and its consumer `isolate: false`, the producer's
//! declaration wins.
//!
//! A [`Coupling`] (every producer and consumer of a topic with a `block`
//! consumer) merges its nodes into one unit whatever the other rules say: the
//! producer's deferral reads an outstanding count kept by the consumers' drains
//! inside one level loop. The coupling joins them before any link is judged,
//! as an `isolate: false` join does, so a link's tick budget sums the ticks of
//! every node a coupling puts in the unit and reads their feeds and deadlines.
//!
//! # Fixed at load
//!
//! Units are derived once, before the deployment is planned, from the graph
//! and the profile. Nothing re-derives them during a run, so a node's records
//! come from one executor for the whole run.
//!
//! # Names
//!
//! A host whose members form one unit keeps its own name, so a deployment with
//! nothing to split plans byte-identically to one planned before units
//! existed. A host that splits names each unit `<host>.<first node>`.

use indexmap::{IndexMap, IndexSet};

use crate::graph::chain::{linear_links, ChainBar, ConsumerEdgeVerdict};
use crate::graph::config::GraphConfig;
use crate::graph::node::{MacroPolicy, NodeInfo};
use crate::graph::partition::{block_colocation_seeds, PartitionCosts};
use crate::graph::topology::{resolve_levels, GraphTopology, TriggerEdges};

/// A link merges when the unit's summed p50 tick times this divisor stays
/// below the period it is fed at (and below a member's `tick_within_ms`): a
/// quarter of the period.
pub const BUDGET_DIVISOR: u64 = 4;

/// Nodes that must share one executor whatever the chain rules say.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Coupling {
    /// The topic whose `block` consumer couples these nodes.
    pub topic: String,
    /// The coupled node ids, in graph order. Nodes outside a host are ignored
    /// when that host's units are derived.
    pub nodes: Vec<String>,
}

/// Why a node does not share its producer's unit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Boundary {
    /// No trigger input is fed by a node of the same process.
    Source,
    /// The node's trigger edge from this process is no link; the shared chain
    /// predicate's verdict says why (latest-value, fan-out, join, ...).
    NotALink {
        /// The producer of the edge.
        producer: String,
        /// The chain predicate that refused the edge.
        bar: ChainBar,
    },
    /// The node declares `isolate: true`.
    Isolated,
    /// Fed by a node that declares `isolate: true`.
    AfterIsolated {
        /// The isolated producer.
        producer: String,
    },
    /// A link whose merge the profile does not show to be safe, as when it
    /// holds no tick for a node the merge involves.
    Unprofiled {
        /// The producer across the link.
        producer: String,
    },
    /// A link whose merged unit is fed at a period the profile does not
    /// hold: no rate it holds says how often `node` is fed.
    UnknownFeed {
        /// The node whose feed is unknown: a member of the merged unit, or a
        /// member of a unit (as it stands before any link merge) that feeds
        /// it through trigger edges the profile does not rate.
        node: String,
    },
    /// A link whose merge would put the unit over its tick budget.
    OverBudget(BudgetVerdict),
    /// A node on a cycle made only of links, which no chain head reaches.
    LinkCycle {
        /// The producer across the node's link.
        producer: String,
    },
    /// The graph has no profile: every node of the process that does not
    /// declare `isolate: true` runs in this unit. Carried by the unit's first
    /// such node.
    NoProfile,
    /// The graph has no profile, and the nodes of the process that do not
    /// declare `isolate: true` run in more than one unit, because an isolated
    /// node cuts them: this is the first of those units, and each later one
    /// starts with [`Boundary::AfterIsolatedLevel`]. Carried by the unit's
    /// first node that does not declare `isolate: true`.
    NoProfileCut,
    /// The graph has no profile, and this unit starts past an isolated node's
    /// level with nodes that no node of an earlier unit that does not declare
    /// `isolate: true` feeds at the level just below (an isolated node in a
    /// unit of its own takes no other node): a unit runs a contiguous band of
    /// the graph's levels, and a node above its band's first level must be
    /// fed from inside the unit at the level just below.
    AfterIsolatedLevel {
        /// The isolated node at the highest level below this unit's first
        /// level.
        node: String,
        /// That node's level.
        level: usize,
    },
}

impl Boundary {
    /// One line naming the boundary, for `graph plan` and log lines.
    pub fn describe(&self) -> String {
        match self {
            Self::Source => "source (no trigger input from this process)".to_string(),
            Self::NotALink { producer, bar } => {
                format!("not a chain link from {producer}: {} ({bar})", bar.label())
            }
            Self::Isolated => "isolate: true".to_string(),
            Self::AfterIsolated { producer } => {
                format!("after an isolated node ({producer} declares isolate: true)")
            }
            Self::Unprofiled { producer } => format!(
                "the profile does not show that joining {producer}'s unit is safe (re-run \
                 `cerulion graph profile` so it holds a tick for every node the merge involves)"
            ),
            Self::UnknownFeed { node } => format!(
                "the profile holds no rate for how often {node} is fed, so the merged unit's \
                 period is unknown (run `cerulion graph profile` while {node} is fed)"
            ),
            Self::OverBudget(v) => v.describe(),
            Self::LinkCycle { producer } => {
                format!(
                    "on a cycle of chain links (fed by {producer}), which no chain head reaches"
                )
            }
            Self::NoProfile => "no profile: every node of this process that does not declare \
                                `isolate: true` runs in this unit"
                .to_string(),
            Self::NoProfileCut => "no profile: the nodes of this process that do not declare \
                                   `isolate: true` run in this unit, apart from those in a unit \
                                   that starts past an isolated node's level (a unit runs a \
                                   contiguous band of levels)"
                .to_string(),
            Self::AfterIsolatedLevel { node, level } => format!(
                "no profile: past level {level} of {node}, which declares isolate: true, and no \
                 node of an earlier unit that does not declare isolate: true feeds it at the \
                 level below (a unit runs a contiguous band of levels)"
            ),
        }
    }
}

/// What the tick budget measured for a merge it refused.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BudgetVerdict {
    /// The node that would have joined.
    pub node: String,
    /// The unit's summed p50 tick with the node included.
    pub unit_ns: u64,
    /// The budget it reached: the limit divided by [`BUDGET_DIVISOR`].
    pub budget_ns: u64,
    /// What the budget was derived from.
    pub against: BudgetAgainst,
    /// The profile's label (its file).
    pub profile: String,
}

impl BudgetVerdict {
    fn describe(&self) -> String {
        let against = match &self.against {
            BudgetAgainst::Period { period_ns } => format!("the unit's {period_ns} ns period"),
            BudgetAgainst::Deadline { node, deadline_ns } => {
                format!("{node}'s {deadline_ns} ns tick_within")
            }
        };
        format!(
            "tick budget: the unit's p50 ticks would sum to {} ns >= {} ns, 1/{BUDGET_DIVISOR} \
             of {against} (profile {})",
            self.unit_ns, self.budget_ns, self.profile
        )
    }
}

/// The limit a [`BudgetVerdict`] measured a merge against.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BudgetAgainst {
    /// The period the unit is fed at.
    Period {
        /// That period.
        period_ns: u64,
    },
    /// The tightest `tick_within_ms` of a member of the merged unit.
    Deadline {
        /// The member that declares it.
        node: String,
        /// The deadline.
        deadline_ns: u64,
    },
}

/// One executor: the nodes that run on one thread.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExecutorUnit {
    /// The unit's name, also its process-group name in the expanded plan.
    pub name: String,
    /// The member node ids, in the host's member order.
    pub nodes: Vec<String>,
    /// Every member that does not share its producer's unit, with why.
    pub boundaries: Vec<(String, Boundary)>,
    /// The `block` topics that merged nodes into this unit.
    pub block_topics: Vec<String>,
}

/// The per-node facts unit derivation reads, gathered from a graph by
/// [`UnitGraph::from_graph`] or written by hand in a test.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct UnitGraph {
    /// Every trigger edge whose producer is a graph node: `(producer, consumer)`,
    /// once per consumer input.
    pub trigger_edges: Vec<(String, String)>,
    /// The shared chain predicate's verdict on every consumer edge
    /// ([`linear_links`]); an edge with no bar is a link, and an edge barred
    /// [`ChainBar::NoInGraphProducer`] is a trigger input whose topic no graph
    /// node publishes.
    pub links: Vec<ConsumerEdgeVerdict>,
    /// Each node's `isolate:` declaration.
    pub isolate: IndexMap<String, bool>,
    /// Each period node's period.
    pub period_ns: IndexMap<String, u64>,
    /// Each node's `tick_within_ms`, in ns.
    pub tick_within_ns: IndexMap<String, u64>,
    /// The `block` couplings.
    pub couplings: Vec<Coupling>,
    /// Each node's global DAG level (the levels the partition validator
    /// checks every unit's band against). Empty when the graph's levels cannot
    /// be resolved (a trigger cycle, which the graph's build refuses).
    pub level: IndexMap<String, usize>,
}

impl UnitGraph {
    /// Gather the facts from a built topology and the nodes' declarations.
    pub fn from_graph(
        config: &GraphConfig,
        entry_infos: &IndexMap<String, NodeInfo>,
        topology: &GraphTopology,
        trigger_edges: &TriggerEdges,
    ) -> Self {
        let mut out = Self {
            links: linear_links(entry_infos, topology, trigger_edges),
            ..Self::default()
        };
        for flow in topology.topics() {
            for consumer in &flow.consumers {
                if !trigger_edges.is_triggering(&consumer.node_id, &flow.topic) {
                    continue;
                }
                for producer in &flow.producers {
                    out.trigger_edges
                        .push((producer.clone(), consumer.node_id.clone()));
                }
            }
        }
        for node in config.native_nodes() {
            if let Some(isolate) = node.isolate {
                out.isolate.insert(node.id.clone(), isolate);
            }
            if let Some(info) = entry_infos.get(&node.id) {
                if let Some(MacroPolicy::Period { period_ms }) = info.policy() {
                    out.period_ns
                        .insert(node.id.clone(), period_ms.saturating_mul(1_000_000));
                }
                if let Some(ms) = info.tick_within_ms() {
                    out.tick_within_ns
                        .insert(node.id.clone(), ms.saturating_mul(1_000_000));
                }
            }
        }
        if let Ok(levels) = resolve_levels(config, topology, trigger_edges) {
            for node in config.native_nodes() {
                if let Some(l) = levels.level_of(&node.id) {
                    out.level.insert(node.id.clone(), l);
                }
            }
        }
        for seed in block_colocation_seeds(topology) {
            let mut nodes: IndexSet<String> = IndexSet::new();
            nodes.extend(seed.producers.iter().cloned());
            nodes.extend(seed.block_consumers.iter().map(|(n, _)| n.clone()));
            nodes.extend(seed.other_consumers.iter().map(|(n, _)| n.clone()));
            out.couplings.push(Coupling {
                topic: seed.topic.clone(),
                nodes: nodes.into_iter().collect(),
            });
        }
        out
    }
}

/// A profile for the tick budget: the cost snapshot and a label for it.
#[derive(Debug, Clone, Copy)]
pub struct UnitProfile<'a> {
    /// Per-node p50 tick and per-edge rate.
    pub costs: &'a PartitionCosts,
    /// Where it came from, for the boundary text.
    pub label: &'a str,
}

/// Under the `oracle-mutants` feature, setting this variable makes
/// [`derive_units`] merge a link that the profile does not show to be safe.
#[cfg(feature = "oracle-mutants")]
pub const MERGE_MUTANT_ENV: &str = "CERULION_UNIT_MERGE_MUTANT";

/// Under the `oracle-mutants` feature, setting this variable makes
/// [`derive_units`] ignore the tick budget of a profiled merge.
#[cfg(feature = "oracle-mutants")]
pub const BUDGET_MUTANT_ENV: &str = "CERULION_UNIT_BUDGET_MUTANT";

/// Whether the merge mutant is armed (never, outside the `oracle-mutants`
/// feature).
fn merge_mutant() -> bool {
    #[cfg(feature = "oracle-mutants")]
    {
        std::env::var_os(MERGE_MUTANT_ENV).is_some()
    }
    #[cfg(not(feature = "oracle-mutants"))]
    {
        false
    }
}

/// Whether the budget mutant is armed (never, outside the `oracle-mutants`
/// feature).
fn budget_mutant() -> bool {
    #[cfg(feature = "oracle-mutants")]
    {
        std::env::var_os(BUDGET_MUTANT_ENV).is_some()
    }
    #[cfg(not(feature = "oracle-mutants"))]
    {
        false
    }
}

fn root(parent: &mut [usize], mut i: usize) -> usize {
    while parent[i] != i {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    i
}

fn union(parent: &mut [usize], a: usize, b: usize) {
    let (ra, rb) = (root(parent, a), root(parent, b));
    if ra != rb {
        let (keep, drop) = if ra < rb { (ra, rb) } else { (rb, ra) };
        parent[drop] = keep;
    }
}

/// The period of a trigger edge's profiled rate; none when the profile does
/// not rate the edge or rates it 0.
fn rated_period_ns(edge: &(String, String), costs: Option<&PartitionCosts>) -> Option<u64> {
    costs?
        .edge_rate_mhz
        .get(edge)
        .copied()
        .filter(|r| *r > 0)
        .map(|mhz| 1_000_000_000_000 / mhz)
}

/// How often `node` fires as the profile shows it: the shortest period of its
/// profiled trigger edges out.
fn fire_period_ns(node: &str, graph: &UnitGraph, costs: Option<&PartitionCosts>) -> Option<u64> {
    graph
        .trigger_edges
        .iter()
        .filter(|(p, _)| p == node)
        .filter_map(|edge| rated_period_ns(edge, costs))
        .min()
}

/// What a group of members is fed at.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Feed<'a> {
    /// At this period, at the shortest.
    Every(u64),
    /// Through `node`, at a rate the profile does not hold: a member of the
    /// group, or of a group that feeds it through trigger edges the profile
    /// does not rate.
    Unknown(&'a str),
}

/// The feed of a unit fed by both `a` and `b`: the shorter period, or unknown
/// when either is.
fn faster<'a>(a: Feed<'a>, b: Feed<'a>) -> Feed<'a> {
    match (a, b) {
        (Feed::Every(x), Feed::Every(y)) => Feed::Every(x.min(y)),
        (Feed::Unknown(n), _) | (_, Feed::Unknown(n)) => Feed::Unknown(n),
    }
}

/// [`faster`] over feeds that may be absent; an absent feed adds nothing.
fn joined<'a>(a: Option<Feed<'a>>, b: Option<Feed<'a>>) -> Option<Feed<'a>> {
    match (a, b) {
        (Some(x), Some(y)) => Some(faster(x, y)),
        (x, y) => x.or(y),
    }
}

/// Record that the group rooted at `r` is also fed by `f`. True when that
/// changes the group's feed.
fn feed<'a>(fed: &mut IndexMap<usize, Feed<'a>>, r: usize, f: Feed<'a>) -> bool {
    let now = fed.get(&r).map_or(f, |&known| faster(known, f));
    fed.insert(r, now) != Some(now)
}

/// A member's `tick_within_ms`, in ns, and the member.
#[derive(Debug, Clone, Copy)]
struct Deadline<'a> {
    ns: u64,
    node: &'a str,
}

/// The tighter of two deadlines; `a` on a tie.
fn tighter<'a>(a: Option<Deadline<'a>>, b: Option<Deadline<'a>>) -> Option<Deadline<'a>> {
    match (a, b) {
        (Some(x), Some(y)) => Some(if y.ns < x.ns { y } else { x }),
        (x, y) => x.or(y),
    }
}

/// A unit being grown along one chain.
#[derive(Clone, Copy)]
struct Growing<'a> {
    /// The summed p50 tick of its members, when every one is profiled.
    sum_ns: Option<u64>,
    /// What it is fed at, when any of its feeds is known. `merge_verdict`
    /// reads it from the unit joined, never from what joining brings.
    feed: Option<Feed<'a>>,
    /// The tightest `tick_within_ms` among its members.
    deadline: Option<Deadline<'a>>,
}

/// Decide whether `node` may join the unit `unit` across the link from
/// `producer`. `Ok(sum)` = merge, with the unit's new sum.
/// `unit.feed` is what the merged unit is fed at. `add` is what joining
/// brings: the summed p50 tick of `node` and every node `isolate: false`
/// already joined to it, and their tightest `tick_within`.
fn merge_verdict(
    node: &str,
    producer: &str,
    unit: &Growing<'_>,
    add: &Growing<'_>,
    profile: Option<UnitProfile<'_>>,
) -> Result<Option<u64>, Boundary> {
    let refuse = |boundary: Boundary| {
        if merge_mutant() {
            Ok(None)
        } else {
            Err(boundary)
        }
    };
    let unprofiled = || Boundary::Unprofiled {
        producer: producer.to_string(),
    };
    let Some(profile) = profile else {
        return refuse(unprofiled());
    };
    let (Some(sum), Some(p50)) = (unit.sum_ns, add.sum_ns) else {
        return refuse(unprofiled());
    };
    let period_ns = match unit.feed {
        Some(Feed::Every(p)) => p,
        Some(Feed::Unknown(n)) => {
            return refuse(Boundary::UnknownFeed {
                node: n.to_string(),
            })
        }
        None => return refuse(unprofiled()),
    };
    let unit_ns = sum.saturating_add(p50);
    if budget_mutant() {
        return Ok(Some(unit_ns));
    }
    let over = |limit: u64| unit_ns.saturating_mul(BUDGET_DIVISOR) >= limit;
    let verdict = |against: BudgetAgainst, limit: u64| {
        Boundary::OverBudget(BudgetVerdict {
            node: node.to_string(),
            unit_ns,
            budget_ns: limit / BUDGET_DIVISOR,
            against,
            profile: profile.label.to_string(),
        })
    };
    if over(period_ns) {
        return Err(verdict(BudgetAgainst::Period { period_ns }, period_ns));
    }
    if let Some(d) = tighter(unit.deadline, add.deadline) {
        if over(d.ns) {
            return Err(verdict(
                BudgetAgainst::Deadline {
                    node: d.node.to_string(),
                    deadline_ns: d.ns,
                },
                d.ns,
            ));
        }
    }
    Ok(Some(unit_ns))
}

/// Split one host's members into executor units.
///
/// Without a profile the members form one unit, apart from each member that
/// declares `isolate: true`, which forms one of its own, and the members past
/// one that the first unit cannot hold in a contiguous band of levels; a
/// `block` coupling joins its nodes' units. With a profile the link rules of
/// this module's documentation split and merge them.
///
/// Units are ordered by their first member and list their members in
/// `members` order, so the result depends only on the inputs.
pub fn derive_units(
    host: &str,
    members: &[String],
    graph: &UnitGraph,
    profile: Option<UnitProfile<'_>>,
) -> Vec<ExecutorUnit> {
    let Some(profile) = profile else {
        return units_without_profile(host, members, graph);
    };
    units_from_profile(host, members, graph, profile)
}

/// [`derive_units`] without a profile.
///
/// Each member that declares `isolate: true` forms a unit of its own
/// ([`Boundary::Isolated`]), unless a `block` coupling keeps it with a member
/// that does not; every other member is a PARTICIPANT. The participants form
/// units under the partition validator's rule: a unit owns a contiguous band
/// of the graph's levels, and each of its nodes re-levelizes, inside the unit,
/// to its rank in that band, so on derived levels a node above the band's
/// first level needs a node of the unit that feeds it at the level just below
/// (on assigned levels the validator asks only for the contiguous band, which
/// that feeder also gives). Only an isolated member can take that feeder away,
/// so the participants are placed level by level, lowest first, member order
/// breaking ties:
///
/// - a participant that no cutting isolated member precedes in level joins the
///   first unit;
/// - any other joins the earliest unit that starts at its level or holds a
///   node feeding it at the level just below;
/// - one that no unit takes starts a unit at its level.
///
/// Each unit therefore holds every level from its first to its last. On
/// derived levels, in a process the partition validator accepts, every node
/// above the process's lowest level is fed from inside the process at the
/// level below, so each node of a unit above its first level is fed from
/// inside the unit at the level below. When a member's level is unknown, or
/// no isolated member cuts, the participants form one unit.
///
/// When the participants form one unit its first member that does not declare
/// `isolate: true` carries [`Boundary::NoProfile`]. Otherwise that member of
/// the first unit carries [`Boundary::NoProfileCut`], and that member of each
/// later unit [`Boundary::AfterIsolatedLevel`], naming the cutting isolated
/// member at the highest level below the unit's first level (member order
/// breaking ties). Each `block` coupling then joins the units of its nodes; a
/// member a coupling keeps with another member carries no `isolate: true`
/// reason, and its unit lists the block topic instead.
fn units_without_profile(host: &str, members: &[String], graph: &UnitGraph) -> Vec<ExecutorUnit> {
    let in_host: IndexSet<&str> = members.iter().map(String::as_str).collect();
    let isolated = |n: &str| graph.isolate.get(n) == Some(&true);

    // An isolated member a coupling (or a chain of couplings) puts with a
    // non-isolated member runs in that member's unit, so it cuts nothing.
    let mut coupled: Vec<usize> = (0..members.len()).collect();
    let _ = join_couplings(graph, &in_host, &mut coupled);
    let mut kept: IndexSet<usize> = IndexSet::new();
    for (i, member) in members.iter().enumerate() {
        if !isolated(member) {
            kept.insert(root(&mut coupled, i));
        }
    }
    let mut cut = vec![false; members.len()];
    for (i, member) in members.iter().enumerate() {
        cut[i] = isolated(member) && !kept.contains(&root(&mut coupled, i));
    }
    let mut feeders: Vec<Vec<usize>> = vec![Vec::new(); members.len()];
    for (producer, consumer) in &graph.trigger_edges {
        if let (Some(p), Some(c)) = (
            in_host.get_index_of(producer.as_str()),
            in_host.get_index_of(consumer.as_str()),
        ) {
            feeders[c].push(p);
        }
    }

    // The unit of each participant (a cutting member's entry is unused): 0 is
    // the first unit, `u >= 1` the later unit `later[u - 1]`, which records its
    // first level and the cutting member it is named past, with that member's
    // level.
    let mut unit_of: Vec<usize> = vec![0; members.len()];
    let mut later: Vec<(usize, usize, usize)> = Vec::new();
    // Without a known level for every member, or without a cutting member,
    // every participant stays in the first unit.
    let levels: Option<Vec<usize>> = members
        .iter()
        .map(|m| graph.level.get(m).copied())
        .collect();
    if let Some(level) = levels.filter(|_| cut.contains(&true)) {
        let mut participants: Vec<usize> = (0..members.len()).filter(|&i| !cut[i]).collect();
        participants.sort_by_key(|&i| (level[i], i));
        let mut first_level: Option<usize> = None;
        for &i in &participants {
            let l = level[i];
            let past = (0..members.len())
                .filter(|&j| cut[j] && level[j] < l)
                .max_by_key(|&j| (level[j], std::cmp::Reverse(j)));
            let u = match (first_level, past) {
                (None, _) => {
                    first_level = Some(l);
                    0
                }
                (Some(_), None) => 0,
                (Some(first), Some(j)) => {
                    let starts_at = |u: usize| if u == 0 { first } else { later[u - 1].0 };
                    // Participants are placed lowest level first, so a feeder
                    // at the level below is already placed.
                    let fed_from = |u: usize| {
                        feeders[i]
                            .iter()
                            .any(|&p| !cut[p] && level[p] + 1 == l && unit_of[p] == u)
                    };
                    match (0..=later.len()).find(|&u| starts_at(u) == l || fed_from(u)) {
                        Some(u) => u,
                        None => {
                            later.push((l, j, level[j]));
                            later.len()
                        }
                    }
                }
            };
            unit_of[i] = u;
        }
    }

    let mut parent: Vec<usize> = (0..members.len()).collect();
    let mut starts: IndexMap<&str, Boundary> = IndexMap::new();
    let mut head: Vec<Option<usize>> = vec![None; later.len() + 1];
    let mut carrier: Vec<Option<usize>> = vec![None; later.len() + 1];
    for (i, member) in members.iter().enumerate() {
        if cut[i] {
            starts.insert(member.as_str(), Boundary::Isolated);
            continue;
        }
        let u = unit_of[i];
        match head[u] {
            Some(h) => union(&mut parent, h, i),
            None => head[u] = Some(i),
        }
        if carrier[u].is_none() && !isolated(member) {
            carrier[u] = Some(i);
        }
    }
    for (u, c) in carrier.iter().enumerate() {
        let Some(c) = *c else { continue };
        let reason = match u.checked_sub(1).map(|k| later[k]) {
            None if later.is_empty() => Boundary::NoProfile,
            None => Boundary::NoProfileCut,
            Some((_, j, level)) => Boundary::AfterIsolatedLevel {
                node: members[j].clone(),
                level,
            },
        };
        starts.insert(members[c].as_str(), reason);
    }
    let block_topics = join_couplings(graph, &in_host, &mut parent);
    let topics_by_root = topics_by_final_root(block_topics, &mut parent);
    for (i, member) in members.iter().enumerate() {
        let n = member.as_str();
        let r = root(&mut parent, i);
        if topics_by_root.contains_key(&r) && starts.get(n) == Some(&Boundary::Isolated) {
            starts.shift_remove(n);
        }
    }
    assemble_units(host, members, &mut parent, &starts, &topics_by_root)
}

/// Join the nodes of every `block` coupling that has two or more members in
/// `in_host` into one group of `parent`, and return each coupling's topic
/// keyed by the root its group had when it was joined.
fn join_couplings(
    graph: &UnitGraph,
    in_host: &IndexSet<&str>,
    parent: &mut [usize],
) -> IndexMap<usize, Vec<String>> {
    let mut block_topics: IndexMap<usize, Vec<String>> = IndexMap::new();
    for coupling in &graph.couplings {
        let local: Vec<usize> = coupling
            .nodes
            .iter()
            .filter_map(|n| in_host.get_index_of(n.as_str()))
            .collect();
        if local.len() < 2 {
            continue;
        }
        for pair in local.windows(2) {
            union(parent, pair[0], pair[1]);
        }
        let r = root(parent, local[0]);
        block_topics
            .entry(r)
            .or_default()
            .push(coupling.topic.clone());
    }
    block_topics
}

/// `block_topics` rekeyed by the current root of each group, for a `parent`
/// that later unions may have changed.
fn topics_by_final_root(
    block_topics: IndexMap<usize, Vec<String>>,
    parent: &mut [usize],
) -> IndexMap<usize, Vec<String>> {
    let mut topics_by_root: IndexMap<usize, Vec<String>> = IndexMap::new();
    for (r, topics) in block_topics {
        let fr = root(parent, r);
        topics_by_root.entry(fr).or_default().extend(topics);
    }
    topics_by_root
}

/// One [`ExecutorUnit`] per group of `parent`, ordered by its first member,
/// with the reasons in `starts` and the block topics of its root. A host whose
/// members form one unit keeps its own name; otherwise each unit is named
/// `<host>.<first node>`.
fn assemble_units(
    host: &str,
    members: &[String],
    parent: &mut [usize],
    starts: &IndexMap<&str, Boundary>,
    topics_by_root: &IndexMap<usize, Vec<String>>,
) -> Vec<ExecutorUnit> {
    let mut groups: IndexMap<usize, Vec<String>> = IndexMap::new();
    for (i, member) in members.iter().enumerate() {
        let r = root(parent, i);
        groups.entry(r).or_default().push(member.clone());
    }
    let single = groups.len() == 1;
    groups
        .into_iter()
        .map(|(r, nodes)| ExecutorUnit {
            name: if single {
                host.to_string()
            } else {
                format!("{host}.{}", nodes[0])
            },
            boundaries: nodes
                .iter()
                .filter_map(|n| starts.get(n.as_str()).map(|b| (n.clone(), b.clone())))
                .collect(),
            block_topics: topics_by_root.get(&r).cloned().unwrap_or_default(),
            nodes,
        })
        .collect()
}

/// [`derive_units`] with a profile: every node starts alone and the profile
/// merges links (this module's documentation).
fn units_from_profile(
    host: &str,
    members: &[String],
    graph: &UnitGraph,
    profile: UnitProfile<'_>,
) -> Vec<ExecutorUnit> {
    let profile = Some(profile);
    let in_host: IndexSet<&str> = members.iter().map(String::as_str).collect();
    let index = |n: &str| in_host.get_index_of(n);
    let declared = |n: &str| graph.isolate.get(n).copied();
    let costs = profile.map(|p| p.costs);

    // In-host trigger producers of each member.
    let mut producers: IndexMap<&str, IndexSet<&str>> = IndexMap::new();
    for (p, c) in &graph.trigger_edges {
        if in_host.contains(p.as_str()) && in_host.contains(c.as_str()) {
            producers.entry(c.as_str()).or_default().insert(p.as_str());
        }
    }
    // The in-host link into each member, and the in-host links out of it.
    let mut link_in: IndexMap<&str, &str> = IndexMap::new();
    let mut link_out: IndexMap<&str, &str> = IndexMap::new();
    for edge in graph.links.iter().filter(|e| e.is_fused()) {
        let Some(p) = edge.producer.as_deref() else {
            continue;
        };
        if in_host.contains(p) && in_host.contains(edge.consumer.as_str()) {
            link_in.insert(edge.consumer.as_str(), p);
            link_out.insert(p, edge.consumer.as_str());
        }
    }

    let mut parent: Vec<usize> = (0..members.len()).collect();
    let mut starts: IndexMap<&str, Boundary> = IndexMap::new();
    // Members whose merge is decided by walking their chain.
    let mut candidates: IndexSet<&str> = IndexSet::new();
    for node in members {
        let n = node.as_str();
        if declared(n) == Some(true) {
            starts.insert(n, Boundary::Isolated);
            continue;
        }
        let own = producers.get(n).filter(|p| !p.is_empty());
        let Some(own) = own else {
            starts.insert(n, Boundary::Source);
            continue;
        };
        if declared(n) == Some(false) {
            let joinable: Vec<&str> = own
                .iter()
                .copied()
                .filter(|p| declared(p) != Some(true))
                .collect();
            if joinable.is_empty() {
                let producer = own.first().map(|p| p.to_string()).unwrap_or_default();
                starts.insert(n, Boundary::AfterIsolated { producer });
            }
            for p in joinable {
                if let (Some(a), Some(b)) = (index(n), index(p)) {
                    union(&mut parent, a, b);
                }
            }
            continue;
        }
        let Some(&p) = link_in.get(n) else {
            // An in-host trigger edge that is no link: name the first one.
            let fallback = own.first().map(|p| p.to_string()).unwrap_or_default();
            let (producer, bar) = graph
                .links
                .iter()
                .filter(|e| e.consumer == n)
                .find_map(|e| {
                    let bar = e.bar.clone()?;
                    match e.producer.as_deref() {
                        Some(p) if in_host.contains(p) => Some((p.to_string(), bar)),
                        Some(_) => None,
                        None => Some((fallback.clone(), bar)),
                    }
                })
                .unwrap_or((fallback, ChainBar::NoInGraphProducer));
            starts.insert(n, Boundary::NotALink { producer, bar });
            continue;
        };
        if declared(p) == Some(true) {
            starts.insert(
                n,
                Boundary::AfterIsolated {
                    producer: p.to_string(),
                },
            );
            continue;
        }
        candidates.insert(n);
    }

    // A coupling joins its nodes before any link is judged, as an
    // `isolate: false` join does: the feeds, ticks and deadlines below are
    // gathered per group, so a verdict counts every node a coupling puts in
    // the unit.
    let block_topics = join_couplings(graph, &in_host, &mut parent);

    // The summed p50 tick and tightest `tick_within` of the unit rooted at
    // `r` so far (a node and every node `isolate: false` or a coupling joined
    // to it).
    let unit_totals = |r: usize, parent: &mut Vec<usize>| {
        let mut sum_ns = costs.map(|_| 0u64);
        let mut deadline: Option<Deadline<'_>> = None;
        for (i, member) in members.iter().enumerate() {
            if root(parent, i) != r {
                continue;
            }
            sum_ns = match (sum_ns, costs.and_then(|c| c.node_p50_ns.get(member))) {
                (Some(acc), Some(p50)) => Some(acc.saturating_add(*p50)),
                _ => None,
            };
            if let Some(&ns) = graph.tick_within_ns.get(member) {
                let d = Deadline {
                    ns,
                    node: member.as_str(),
                };
                deadline = tighter(deadline, Some(d));
            }
        }
        Growing {
            sum_ns,
            feed: None,
            deadline,
        }
    };
    // A member that is no candidate and links to one heads a chain.
    let heads = |n: &str| {
        !candidates.contains(n) && link_out.get(n).is_some_and(|f| candidates.contains(*f))
    };
    // What each group is fed at, keyed by its root, from every member before
    // the first merge: the member's own period; else each trigger edge into
    // it at the edge's profiled rate, or, for an edge the profile does not
    // rate, at the feed of the producer's group when that is another group of
    // this host and at an unknown rate when the producer is in another
    // process. A member no graph node triggers, or one with a trigger input no
    // graph node publishes, is also fed at its fire rate (`fire_period_ns`),
    // unknown when the profile rates none of its edges out.
    let mut fed: IndexMap<usize, Feed<'_>> = IndexMap::new();
    let mut carried: Vec<(usize, usize)> = Vec::new();
    for (i, member) in members.iter().enumerate() {
        let n = member.as_str();
        let r = root(&mut parent, i);
        if let Some(&period_ns) = graph.period_ns.get(n) {
            feed(&mut fed, r, Feed::Every(period_ns));
            continue;
        }
        let mut triggered = false;
        for edge in graph.trigger_edges.iter().filter(|(_, c)| c == n) {
            triggered = true;
            if let Some(period_ns) = rated_period_ns(edge, costs) {
                feed(&mut fed, r, Feed::Every(period_ns));
                continue;
            }
            match index(&edge.0) {
                Some(j) => {
                    let from = root(&mut parent, j);
                    if from != r {
                        carried.push((from, r));
                    }
                }
                None => {
                    feed(&mut fed, r, Feed::Unknown(n));
                }
            }
        }
        let from_outside = graph
            .links
            .iter()
            .any(|e| e.consumer == n && e.bar == Some(ChainBar::NoInGraphProducer));
        if !triggered || from_outside {
            let fires = fire_period_ns(n, graph, costs).map_or(Feed::Unknown(n), Feed::Every);
            feed(&mut fed, r, fires);
        }
    }
    // Carry each group's feed across those edges until no feed changes. A
    // change gives a group its first feed, shortens its period to one already
    // in the map or makes it unknown, so this ends.
    loop {
        let mut changed = false;
        for &(from, into) in &carried {
            if let Some(&f) = fed.get(&from) {
                changed |= feed(&mut fed, into, f);
            }
        }
        if !changed {
            break;
        }
    }
    let mut walked: IndexSet<&str> = IndexSet::new();
    // Walk each chain of candidate links from the node that heads it, growing
    // one unit at a time.
    for node in members {
        let head = node.as_str();
        if !heads(head) {
            continue;
        }
        let head_root = index(head).map(|i| root(&mut parent, i));
        let totals = head_root.map(|r| unit_totals(r, &mut parent));
        let mut unit = Growing {
            sum_ns: totals.as_ref().and_then(|t| t.sum_ns),
            feed: head_root.and_then(|r| fed.get(&r).copied()),
            deadline: totals.as_ref().and_then(|t| t.deadline),
        };
        let mut at = head;
        while let Some(&next) = link_out.get(at) {
            if !candidates.contains(next) || !walked.insert(next) {
                break;
            }
            // `unit` holds the ticks of every member of `at`'s group. A `next`
            // in that group (an `isolate: false` join or a coupling put both
            // there) adds nothing, so the walk moves on without a verdict.
            if let (Some(a), Some(b)) = (index(next), index(at)) {
                if root(&mut parent, a) == root(&mut parent, b) {
                    at = next;
                    continue;
                }
            }
            let next_root = index(next).map(|i| root(&mut parent, i));
            let next_feed = next_root.and_then(|r| fed.get(&r).copied());
            let add = match next_root {
                Some(r) => unit_totals(r, &mut parent),
                None => Growing {
                    sum_ns: None,
                    feed: None,
                    deadline: None,
                },
            };
            // Merged, the unit is fed by every feed of both groups (the link
            // between them among `next`'s), so the verdict reads the shorter
            // period.
            let judged = Growing {
                feed: joined(unit.feed, next_feed),
                ..unit
            };
            match merge_verdict(next, at, &judged, &add, profile) {
                Ok(sum) => {
                    if let (Some(a), Some(b)) = (index(next), index(at)) {
                        let old = [root(&mut parent, a), root(&mut parent, b)];
                        union(&mut parent, a, b);
                        for r in old {
                            fed.shift_remove(&r);
                        }
                        if let Some(f) = judged.feed {
                            let r = root(&mut parent, a);
                            fed.insert(r, f);
                        }
                    }
                    unit = Growing {
                        sum_ns: sum,
                        feed: judged.feed,
                        deadline: tighter(unit.deadline, add.deadline),
                    };
                }
                Err(boundary) => {
                    starts.insert(next, boundary);
                    unit = Growing {
                        feed: next_feed,
                        ..add
                    };
                }
            }
            at = next;
        }
    }

    // A candidate no walk reached sits on a cycle made only of links: it
    // starts a unit of its own and says so.
    for node in members {
        let n = node.as_str();
        if candidates.contains(n) && !walked.contains(n) {
            let producer = link_in.get(n).map(|p| p.to_string()).unwrap_or_default();
            starts.insert(n, Boundary::LinkCycle { producer });
        }
    }

    let topics_by_root = topics_by_final_root(block_topics, &mut parent);
    // A node in one unit with a node of this host that triggers it starts no
    // unit, so it carries no reason. A `block` flow keeps its nodes together
    // over `isolate: true`; such a node does not start a unit either, and the
    // unit lists the block topic instead.
    for (i, member) in members.iter().enumerate() {
        let n = member.as_str();
        let r = root(&mut parent, i);
        let with_producer = producers.get(n).is_some_and(|own| {
            own.iter()
                .filter_map(|p| index(p))
                .any(|j| root(&mut parent, j) == r)
        });
        let kept_over_isolate =
            topics_by_root.contains_key(&r) && starts.get(n) == Some(&Boundary::Isolated);
        if with_producer || kept_over_isolate {
            starts.shift_remove(n);
        }
    }
    assemble_units(host, members, &mut parent, &starts, &topics_by_root)
}

/// A graph's `process_groups:` rewritten to one group per executor unit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnitPlan {
    /// One entry per unit, in the declared group order with each group
    /// replaced by its units.
    pub groups: IndexMap<String, Vec<String>>,
    /// The declared `process_group_order`, each name replaced by its units;
    /// empty when the graph declares none.
    pub order: Vec<String>,
    /// The hosting process of every unit of a group that split, keyed by unit
    /// name. A unit absent from the map is its own process.
    pub hosts: IndexMap<String, String>,
    /// The units of every declared group, keyed by the declared group name.
    pub units: IndexMap<String, Vec<ExecutorUnit>>,
    /// Declared groups kept as one unit because a unit name would have keyed
    /// the same as another group's or unit's.
    pub name_collisions: Vec<String>,
}

impl UnitPlan {
    /// The declared groups that host more than one unit.
    pub fn split_hosts(&self) -> impl Iterator<Item = (&str, &[ExecutorUnit])> {
        self.units
            .iter()
            .filter(|(_, units)| units.len() > 1)
            .map(|(host, units)| (host.as_str(), units.as_slice()))
    }

    /// Whether any declared group hosts more than one unit.
    pub fn splits_any(&self) -> bool {
        self.split_hosts().next().is_some()
    }

    /// The same config with `process_groups:` (and `process_group_order:`)
    /// replaced by one group per unit.
    pub fn apply(&self, config: &GraphConfig) -> GraphConfig {
        let mut out = config.clone();
        out.process_groups = self.groups.clone();
        out.process_group_order = self.order.clone();
        out
    }
}

/// Split every declared group of `config` into executor units.
///
/// `keep_whole` names declared groups that stay one unit whatever the rules
/// say (a caller uses it for a group whose split partition it refused). A
/// graph without `process_groups:` yields an empty plan. Unit names are
/// compared exactly; [`expand_process_groups_keyed`] compares them under a
/// caller's normalization.
pub fn expand_process_groups(
    config: &GraphConfig,
    graph: &UnitGraph,
    profile: Option<UnitProfile<'_>>,
    keep_whole: &IndexSet<String>,
) -> UnitPlan {
    expand_process_groups_keyed(config, graph, profile, keep_whole, &|name| name.to_string())
}

/// [`expand_process_groups`] with unit names compared under `key` (for
/// example the sanitizing that turns a group name into a file name). A group
/// whose split would give a unit a name that keys the same as another group
/// or unit stays one unit, and is listed in [`UnitPlan::name_collisions`].
pub fn expand_process_groups_keyed(
    config: &GraphConfig,
    graph: &UnitGraph,
    profile: Option<UnitProfile<'_>>,
    keep_whole: &IndexSet<String>,
    key: &dyn Fn(&str) -> String,
) -> UnitPlan {
    let whole = |host: &str, members: &[String]| {
        vec![ExecutorUnit {
            name: host.to_string(),
            nodes: members.to_vec(),
            boundaries: Vec::new(),
            block_topics: Vec::new(),
        }]
    };
    let mut units_by_host: IndexMap<String, Vec<ExecutorUnit>> = config
        .process_groups
        .iter()
        .map(|(host, members)| {
            let units = if keep_whole.contains(host) {
                whole(host, members)
            } else {
                derive_units(host, members, graph, profile)
            };
            (host.clone(), units)
        })
        .collect();
    let mut name_collisions: Vec<String> = Vec::new();
    loop {
        let mut owners: IndexMap<String, Vec<&str>> = IndexMap::new();
        for (host, units) in &units_by_host {
            for unit in units {
                owners
                    .entry(key(&unit.name))
                    .or_default()
                    .push(host.as_str());
            }
        }
        let colliding: Option<String> = units_by_host
            .iter()
            .filter(|(_, units)| units.len() > 1)
            .find(|(_, units)| units.iter().any(|u| owners[&key(&u.name)].len() > 1))
            .map(|(host, _)| host.clone());
        let Some(host) = colliding else {
            break;
        };
        let members = config.process_groups[&host].clone();
        units_by_host.insert(host.clone(), whole(&host, &members));
        name_collisions.push(host);
    }
    let mut groups = IndexMap::new();
    let mut hosts = IndexMap::new();
    for (host, units) in &units_by_host {
        for unit in units {
            groups.insert(unit.name.clone(), unit.nodes.clone());
            if units.len() > 1 {
                hosts.insert(unit.name.clone(), host.clone());
            }
        }
    }
    let order = config
        .process_group_order
        .iter()
        .flat_map(|host| {
            units_by_host
                .get(host)
                .map(|units: &Vec<ExecutorUnit>| {
                    units.iter().map(|u| u.name.clone()).collect::<Vec<_>>()
                })
                .unwrap_or_else(|| vec![host.clone()])
        })
        .collect();
    UnitPlan {
        groups,
        order,
        hosts,
        units: units_by_host,
        name_collisions,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::partition::HopCosts;
    use std::collections::BTreeMap;

    fn s(v: &[&str]) -> Vec<String> {
        v.iter().map(|x| x.to_string()).collect()
    }

    fn verdict(p: &str, c: &str, bar: Option<ChainBar>) -> ConsumerEdgeVerdict {
        ConsumerEdgeVerdict {
            topic: format!("/t/{p}/out"),
            producer: Some(p.to_string()),
            consumer: c.to_string(),
            input: "inp".to_string(),
            bar,
        }
    }

    /// A hand-built graph: `links` are trigger edges the chain predicates
    /// accept, `barred` trigger edges they refuse (with the bar), `periods`
    /// the period sources in ms.
    fn graph(
        links: &[(&str, &str)],
        barred: &[(&str, &str, ChainBar)],
        periods: &[(&str, u64)],
    ) -> UnitGraph {
        let mut g = UnitGraph::default();
        for (p, c) in links {
            g.trigger_edges.push((p.to_string(), c.to_string()));
            g.links.push(verdict(p, c, None));
        }
        for (p, c, bar) in barred {
            g.trigger_edges.push((p.to_string(), c.to_string()));
            g.links.push(verdict(p, c, Some(bar.clone())));
        }
        for (n, ms) in periods {
            g.period_ns.insert(n.to_string(), ms * 1_000_000);
        }
        g.level = levels(&g);
        g
    }

    /// Longest-path levels over `g`'s trigger edges, as the graph's
    /// levelization gives them: a node nothing triggers is at level 0. A cycle
    /// stops after one relaxation per node.
    fn levels(g: &UnitGraph) -> IndexMap<String, usize> {
        let mut level: IndexMap<String, usize> = IndexMap::new();
        for (p, c) in &g.trigger_edges {
            level.entry(p.clone()).or_insert(0);
            level.entry(c.clone()).or_insert(0);
        }
        for n in g.period_ns.keys() {
            level.entry(n.clone()).or_insert(0);
        }
        for _ in 0..level.len() {
            for (p, c) in &g.trigger_edges {
                let next = level[p] + 1;
                if level[c] < next {
                    level.insert(c.clone(), next);
                }
            }
        }
        level
    }

    fn costs(p50: &[(&str, u64)], rates: &[(&str, &str, u64)]) -> PartitionCosts {
        PartitionCosts {
            node_p50_ns: p50.iter().map(|(n, v)| (n.to_string(), *v)).collect(),
            edge_rate_mhz: rates
                .iter()
                .map(|(p, c, r)| ((p.to_string(), c.to_string()), *r))
                .collect::<BTreeMap<_, _>>(),
            hop: HopCosts {
                cross_ns: 0,
                intra_ns: 0,
            },
        }
    }

    fn profile(c: &PartitionCosts) -> Option<UnitProfile<'_>> {
        Some(UnitProfile {
            costs: c,
            label: "costs.yaml",
        })
    }

    fn nodes(units: &[ExecutorUnit]) -> Vec<Vec<String>> {
        units.iter().map(|u| u.nodes.clone()).collect()
    }

    fn start_of<'a>(units: &'a [ExecutorUnit], node: &str) -> Option<&'a Boundary> {
        units
            .iter()
            .flat_map(|u| u.boundaries.iter())
            .find(|(n, _)| n == node)
            .map(|(_, b)| b)
    }

    fn chain3() -> UnitGraph {
        graph(&[("cam", "det"), ("det", "track")], &[], &[("cam", 1)])
    }

    /// `producer` (10 ms) publishes one topic that `relay` and `join` read,
    /// and `join` also reads `relay`'s output: no edge is a chain link.
    fn producer_relay_join() -> UnitGraph {
        let join = ChainBar::ConsumerJoin { trigger_edges: 2 };
        graph(
            &[],
            &[
                ("producer", "relay", ChainBar::FanOut { consumers: 2 }),
                ("producer", "join", join.clone()),
                ("relay", "join", join),
            ],
            &[("producer", 10)],
        )
    }

    fn prj() -> Vec<String> {
        s(&["producer", "relay", "join"])
    }

    // ---- Without a profile, a process is one unit.

    #[test]
    fn without_a_profile_a_producer_a_relay_and_a_join_are_one_unit() {
        let units = derive_units("g", &prj(), &producer_relay_join(), None);
        assert_eq!(nodes(&units), [prj()]);
        assert_eq!(units[0].name, "g", "one unit keeps its host's name");
        assert_eq!(
            units[0].boundaries,
            [("producer".to_string(), Boundary::NoProfile)],
            "the unit says why it is one unit"
        );
        assert!(units[0].block_topics.is_empty());
    }

    #[test]
    fn without_a_profile_isolate_true_gives_that_node_alone_a_unit() {
        // `join`, at the last level, declares isolate: true: it leaves the one
        // unit and nothing else does.
        let mut g = producer_relay_join();
        g.isolate.insert("join".into(), true);
        let units = derive_units("g", &prj(), &g, None);
        assert_eq!(nodes(&units), [s(&["producer", "relay"]), s(&["join"])]);
        assert_eq!(
            units.iter().map(|u| u.name.as_str()).collect::<Vec<_>>(),
            ["g.producer", "g.join"]
        );
        assert_eq!(
            units[0].boundaries,
            [("producer".to_string(), Boundary::NoProfile)]
        );
        assert_eq!(
            units[1].boundaries,
            [("join".to_string(), Boundary::Isolated)]
        );
    }

    #[test]
    fn without_a_profile_isolate_true_mid_graph_cuts_the_one_unit_past_its_level() {
        // `relay` (level 1) declares isolate: true. `producer` (0) and `join`
        // (2) in one unit would own levels 0 and 2 with level 1 outside it,
        // which no unit may, so `join` starts a unit past relay's level and
        // `producer` heads the first of two units.
        let mut g = producer_relay_join();
        g.isolate.insert("relay".into(), true);
        let units = derive_units("g", &prj(), &g, None);
        assert_eq!(
            nodes(&units),
            [s(&["producer"]), s(&["relay"]), s(&["join"])]
        );
        assert_eq!(start_of(&units, "producer"), Some(&Boundary::NoProfileCut));
        assert_eq!(start_of(&units, "relay"), Some(&Boundary::Isolated));
        assert_eq!(
            start_of(&units, "join"),
            Some(&Boundary::AfterIsolatedLevel {
                node: "relay".into(),
                level: 1
            })
        );
    }

    #[test]
    fn without_a_profile_a_node_at_the_isolated_level_stays_below_the_cut() {
        // `x` (level 1) declares isolate: true; `c`, also at level 1, stays in
        // `producer`'s unit. `join` (2) is fed at level 1 only by `x`, so it
        // starts a unit past level 1.
        let join = ChainBar::ConsumerJoin { trigger_edges: 2 };
        let fan = ChainBar::FanOut { consumers: 3 };
        let mut g = graph(
            &[],
            &[
                ("producer", "x", fan.clone()),
                ("producer", "c", fan.clone()),
                ("producer", "join", fan),
                ("x", "join", join),
            ],
            &[("producer", 10)],
        );
        g.isolate.insert("x".into(), true);
        let units = derive_units("g", &s(&["producer", "x", "c", "join"]), &g, None);
        assert_eq!(
            nodes(&units),
            [s(&["producer", "c"]), s(&["x"]), s(&["join"])]
        );
        assert_eq!(
            start_of(&units, "join"),
            Some(&Boundary::AfterIsolatedLevel {
                node: "x".into(),
                level: 1
            })
        );
    }

    #[test]
    fn without_a_profile_unknown_levels_cut_nothing() {
        // A hand-built graph with no levels: the non-isolated members stay one
        // unit whatever the isolated member's place.
        let mut g = producer_relay_join();
        g.level.clear();
        g.isolate.insert("relay".into(), true);
        let units = derive_units("g", &prj(), &g, None);
        assert_eq!(nodes(&units), [s(&["producer", "join"]), s(&["relay"])]);
    }

    #[test]
    fn without_a_profile_isolate_false_changes_nothing() {
        let mut g = producer_relay_join();
        g.isolate.insert("join".into(), false);
        let units = derive_units("g", &prj(), &g, None);
        assert_eq!(nodes(&units), [prj()]);
        assert_eq!(
            units[0].boundaries,
            [("producer".to_string(), Boundary::NoProfile)]
        );
    }

    #[test]
    fn without_a_profile_every_node_declaring_isolate_true_is_a_unit_of_its_own() {
        let mut g = producer_relay_join();
        for n in prj() {
            g.isolate.insert(n, true);
        }
        let units = derive_units("g", &prj(), &g, None);
        assert_eq!(
            nodes(&units),
            [s(&["producer"]), s(&["relay"]), s(&["join"])]
        );
        assert!(units
            .iter()
            .all(|u| u.boundaries == [(u.nodes[0].clone(), Boundary::Isolated)]));
    }

    #[test]
    fn with_a_profile_the_same_process_splits_at_every_edge_that_is_no_link() {
        let c = costs(&[("producer", 1), ("relay", 1), ("join", 1)], &[]);
        let units = derive_units("g", &prj(), &producer_relay_join(), profile(&c));
        assert_eq!(
            nodes(&units),
            [s(&["producer"]), s(&["relay"]), s(&["join"])]
        );
        assert_eq!(start_of(&units, "producer"), Some(&Boundary::Source));
        assert_eq!(
            start_of(&units, "join"),
            Some(&Boundary::NotALink {
                producer: "producer".into(),
                bar: ChainBar::ConsumerJoin { trigger_edges: 2 }
            })
        );
    }

    #[test]
    fn without_a_profile_a_linear_chain_is_one_unit() {
        let units = derive_units("g", &s(&["cam", "det", "track"]), &chain3(), None);
        assert_eq!(nodes(&units), [s(&["cam", "det", "track"])]);
        assert_eq!(units[0].name, "g");
        assert_eq!(
            units[0].boundaries,
            [("cam".to_string(), Boundary::NoProfile)]
        );
    }

    #[test]
    fn without_a_profile_isolate_true_mid_chain_cuts_the_chain_past_its_level() {
        let mut g = chain3();
        g.isolate.insert("det".into(), true);
        let units = derive_units("g", &s(&["cam", "det", "track"]), &g, None);
        assert_eq!(nodes(&units), [s(&["cam"]), s(&["det"]), s(&["track"])]);
        assert_eq!(start_of(&units, "det"), Some(&Boundary::Isolated));
        assert_eq!(
            start_of(&units, "track"),
            Some(&Boundary::AfterIsolatedLevel {
                node: "det".into(),
                level: 1
            })
        );
    }

    #[test]
    fn without_a_profile_isolating_one_branch_of_a_fan_out_keeps_the_other_branch_whole() {
        // `producer` (level 0) feeds `x` and `c` (level 1), and `c` feeds `d`
        // (level 2). `x` declares isolate: true. `d` is past x's level but is
        // fed at the level below by `c`, so `producer`, `c` and `d` stay one
        // unit and only `x` leaves it.
        let mut g = graph(
            &[("c", "d")],
            &[
                ("producer", "x", ChainBar::FanOut { consumers: 2 }),
                ("producer", "c", ChainBar::FanOut { consumers: 2 }),
            ],
            &[("producer", 10)],
        );
        g.isolate.insert("x".into(), true);
        let units = derive_units("g", &s(&["producer", "x", "c", "d"]), &g, None);
        assert_eq!(nodes(&units), [s(&["producer", "c", "d"]), s(&["x"])]);
        assert_eq!(
            units[0].boundaries,
            [("producer".to_string(), Boundary::NoProfile)],
            "the one unit of the other nodes says so: {units:?}"
        );
        assert_eq!(units[1].boundaries, [("x".to_string(), Boundary::Isolated)]);
    }

    #[test]
    fn without_a_profile_an_isolated_source_leaves_the_join_with_the_other_source() {
        // `cam` and `lidar` (level 0) both feed `fusion` (level 1), and `lidar`
        // declares isolate: true. `cam` feeds `fusion` at the level below, so
        // the two share a unit and the join takes only lidar's frames from
        // another unit.
        let join = ChainBar::ConsumerJoin { trigger_edges: 2 };
        let mut g = graph(
            &[],
            &[("cam", "fusion", join.clone()), ("lidar", "fusion", join)],
            &[("cam", 10), ("lidar", 10)],
        );
        g.isolate.insert("lidar".into(), true);
        let units = derive_units("g", &s(&["cam", "lidar", "fusion"]), &g, None);
        assert_eq!(nodes(&units), [s(&["cam", "fusion"]), s(&["lidar"])]);
        assert_eq!(start_of(&units, "cam"), Some(&Boundary::NoProfile));
        assert_eq!(start_of(&units, "fusion"), None, "{units:?}");
    }

    #[test]
    fn without_a_profile_the_first_unit_of_a_cut_process_does_not_claim_every_node() {
        // `relay` (level 1) declares isolate: true, so `join` (level 2) starts
        // a unit past its level. `producer` heads the first unit, which does
        // not hold every node that does not declare isolate: true, so its
        // reason must not say so.
        let mut g = producer_relay_join();
        g.isolate.insert("relay".into(), true);
        let units = derive_units("g", &prj(), &g, None);
        assert_eq!(
            nodes(&units),
            [s(&["producer"]), s(&["relay"]), s(&["join"])]
        );
        let first = start_of(&units, "producer").expect("producer heads a unit");
        assert_ne!(first, &Boundary::NoProfile, "{units:?}");
        assert!(
            !first.describe().contains("every node"),
            "`{}` claims a unit it does not hold",
            first.describe()
        );
        assert_eq!(first, &Boundary::NoProfileCut);
        assert!(
            first
                .describe()
                .contains("apart from those in a unit that starts past"),
            "{}",
            first.describe()
        );
    }

    #[test]
    fn without_a_profile_each_later_unit_names_the_isolated_node_just_below_it() {
        // src -> i1 -> a -> i2 -> b, with i1 (level 1) and i2 (level 3)
        // isolated: `a` starts a unit past level 1 of i1, `b` one past level 3
        // of i2.
        let mut g = graph(
            &[("src", "i1"), ("i1", "a"), ("a", "i2"), ("i2", "b")],
            &[],
            &[("src", 10)],
        );
        g.isolate.insert("i1".into(), true);
        g.isolate.insert("i2".into(), true);
        let members = s(&["src", "i1", "a", "i2", "b"]);
        let units = derive_units("g", &members, &g, None);
        assert_eq!(
            nodes(&units),
            [s(&["src"]), s(&["i1"]), s(&["a"]), s(&["i2"]), s(&["b"])]
        );
        assert_eq!(start_of(&units, "src"), Some(&Boundary::NoProfileCut));
        assert_eq!(
            start_of(&units, "a"),
            Some(&Boundary::AfterIsolatedLevel {
                node: "i1".into(),
                level: 1
            })
        );
        assert_eq!(
            start_of(&units, "b"),
            Some(&Boundary::AfterIsolatedLevel {
                node: "i2".into(),
                level: 3
            })
        );
    }

    #[test]
    fn without_a_profile_a_later_unit_keeps_the_chain_past_the_isolated_node() {
        // src -> iso -> a -> b -> c: `a`, `b` and `c` share the unit past
        // iso's level, each fed at the level below from inside it, and the
        // unit's first member in member order carries the reason.
        let mut g = graph(
            &[("src", "iso"), ("iso", "a"), ("a", "b"), ("b", "c")],
            &[],
            &[("src", 10)],
        );
        g.isolate.insert("iso".into(), true);
        let units = derive_units("g", &s(&["c", "b", "a", "iso", "src"]), &g, None);
        assert_eq!(
            nodes(&units),
            [s(&["c", "b", "a"]), s(&["iso"]), s(&["src"])]
        );
        assert_eq!(
            units[0].boundaries,
            [(
                "c".to_string(),
                Boundary::AfterIsolatedLevel {
                    node: "iso".into(),
                    level: 1
                }
            )]
        );
        assert_eq!(start_of(&units, "src"), Some(&Boundary::NoProfileCut));
    }

    #[test]
    fn without_a_profile_a_node_no_isolated_node_precedes_in_level_stays_in_the_first_unit() {
        // Assigned levels (a `level_assignments:` graph may leave a level
        // between a producer and its consumer): `producer` 0, `b` 1, `a` 2
        // fed only by `producer`, `x` 3 isolated and fed by `a`. No isolated
        // node sits below `a`'s level, so `a` stays in the first unit though
        // no node feeds it at level 1.
        let mut g = graph(
            &[("producer", "a"), ("producer", "b"), ("a", "x")],
            &[],
            &[("producer", 10)],
        );
        g.level = [("producer", 0), ("b", 1), ("a", 2), ("x", 3)]
            .into_iter()
            .map(|(n, l)| (n.to_string(), l))
            .collect();
        g.isolate.insert("x".into(), true);
        let units = derive_units("g", &s(&["producer", "a", "b", "x"]), &g, None);
        assert_eq!(nodes(&units), [s(&["producer", "a", "b"]), s(&["x"])]);
        assert_eq!(start_of(&units, "producer"), Some(&Boundary::NoProfile));
    }

    // ---- With a profile, every node starts alone and the profile merges links.

    #[test]
    fn a_profiled_chain_under_budget_merges_into_one_unit_named_for_its_host() {
        let c = costs(&[("cam", 2_000), ("det", 20_000), ("track", 5_000)], &[]);
        let units = derive_units("g", &s(&["cam", "det", "track"]), &chain3(), profile(&c));
        assert_eq!(nodes(&units), [s(&["cam", "det", "track"])]);
        assert_eq!(units[0].name, "g");
        assert_eq!(units[0].boundaries, [("cam".to_string(), Boundary::Source)]);
    }

    #[test]
    fn a_node_the_profile_does_not_cover_does_not_merge() {
        let c = costs(&[("cam", 2_000), ("track", 5_000)], &[]);
        let units = derive_units("g", &s(&["cam", "det", "track"]), &chain3(), profile(&c));
        assert_eq!(nodes(&units), [s(&["cam"]), s(&["det"]), s(&["track"])]);
        assert_eq!(
            start_of(&units, "det"),
            Some(&Boundary::Unprofiled {
                producer: "cam".into()
            })
        );
    }

    /// The budget oracle: a 1 kHz chain `pace -> filt -> slow -> sink` whose
    /// `slow` ticks 50 ms. The expected split is fixed by hand.
    #[test]
    fn a_merge_over_the_tick_budget_starts_a_unit() {
        let g = graph(
            &[("pace", "filt"), ("filt", "slow"), ("slow", "sink")],
            &[],
            &[("pace", 1)],
        );
        let c = costs(
            &[
                ("pace", 2_000),
                ("filt", 20_000),
                ("slow", 50_000_000),
                ("sink", 5_000),
            ],
            &[
                ("pace", "filt", 1_000_000),
                ("filt", "slow", 1_000_000),
                ("slow", "sink", 20_000),
            ],
        );
        let units = derive_units("g", &s(&["pace", "filt", "slow", "sink"]), &g, profile(&c));
        assert_eq!(
            nodes(&units),
            [s(&["pace", "filt"]), s(&["slow"]), s(&["sink"])]
        );
        let Some(Boundary::OverBudget(v)) = start_of(&units, "slow") else {
            panic!("slow must start a unit over budget: {units:?}");
        };
        assert_eq!(v.unit_ns, 50_022_000);
        assert_eq!(
            v.against,
            BudgetAgainst::Period {
                period_ns: 1_000_000
            }
        );
        assert_eq!(v.budget_ns, 250_000);
        assert_eq!(v.profile, "costs.yaml");
        assert!(matches!(
            start_of(&units, "sink"),
            Some(Boundary::OverBudget(_))
        ));
    }

    #[test]
    fn the_budget_is_a_quarter_of_the_period_exactly() {
        let g = graph(&[("src", "mid")], &[], &[("src", 10)]);
        let under = costs(&[("src", 0), ("mid", 2_499_999)], &[]);
        let units = derive_units("g", &s(&["src", "mid"]), &g, profile(&under));
        assert_eq!(nodes(&units), [s(&["src", "mid"])]);
        let at = costs(&[("src", 0), ("mid", 2_500_000)], &[]);
        let units = derive_units("g", &s(&["src", "mid"]), &g, profile(&at));
        assert_eq!(nodes(&units), [s(&["src"]), s(&["mid"])]);
    }

    #[test]
    fn a_tick_within_deadline_refuses_a_merge_the_period_allows() {
        // 10 Hz; src + mid + out tick 3.002 ms, under a quarter of 100 ms, but
        // `out` declares tick_within 10 ms and 3.002 ms reaches a quarter of it.
        let mut g = graph(&[("src", "mid"), ("mid", "out")], &[], &[("src", 100)]);
        g.tick_within_ns.insert("out".into(), 10_000_000);
        let c = costs(&[("src", 1_000), ("mid", 3_000_000), ("out", 1_000)], &[]);
        let units = derive_units("g", &s(&["src", "mid", "out"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["src", "mid"]), s(&["out"])]);
        let Some(Boundary::OverBudget(v)) = start_of(&units, "out") else {
            panic!("out must be over budget: {units:?}");
        };
        assert_eq!(
            v.against,
            BudgetAgainst::Deadline {
                node: "out".into(),
                deadline_ns: 10_000_000
            }
        );
    }

    #[test]
    fn an_earlier_members_deadline_bounds_every_later_merge() {
        // `mid` declares tick_within 10 ms; `out` ticks 2 ms more, which
        // pushes the unit past a quarter of `mid`'s deadline. The refusal
        // names `mid`, whose deadline it is.
        let mut g = graph(&[("src", "mid"), ("mid", "out")], &[], &[("src", 1000)]);
        g.tick_within_ns.insert("mid".into(), 10_000_000);
        let c = costs(&[("src", 0), ("mid", 1_000_000), ("out", 2_000_000)], &[]);
        let units = derive_units("g", &s(&["src", "mid", "out"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["src", "mid"]), s(&["out"])]);
        let Some(b @ Boundary::OverBudget(v)) = start_of(&units, "out") else {
            panic!("out must be over budget: {units:?}");
        };
        assert_eq!(
            v.against,
            BudgetAgainst::Deadline {
                node: "mid".into(),
                deadline_ns: 10_000_000
            }
        );
        assert_eq!(
            b.describe(),
            "tick budget: the unit's p50 ticks would sum to 3000000 ns >= 2500000 ns, 1/4 of \
             mid's 10000000 ns tick_within (profile costs.yaml)"
        );
    }

    #[test]
    fn a_join_partners_deadline_is_named_for_the_partner() {
        // `y` declares isolate: false on its join edge from `c` and
        // tick_within 10 ms, so it rides with `c`. Joining `src`'s unit brings
        // `c` and `y`: 3 ms reaches a quarter of `y`'s deadline, under a
        // quarter of the 100 ms period.
        let mut g = graph(
            &[("src", "c")],
            &[("c", "y", ChainBar::ConsumerJoin { trigger_edges: 2 })],
            &[("src", 100)],
        );
        g.isolate.insert("y".into(), false);
        g.tick_within_ns.insert("y".into(), 10_000_000);
        let c = costs(
            &[("src", 1_000_000), ("c", 1_000_000), ("y", 1_000_000)],
            &[],
        );
        let units = derive_units("g", &s(&["src", "c", "y"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["src"]), s(&["c", "y"])]);
        let Some(Boundary::OverBudget(v)) = start_of(&units, "c") else {
            panic!("c must be over budget: {units:?}");
        };
        assert_eq!(v.node, "c");
        assert_eq!(
            v.against,
            BudgetAgainst::Deadline {
                node: "y".into(),
                deadline_ns: 10_000_000
            }
        );
    }

    #[test]
    fn a_joining_node_brings_the_ticks_of_the_nodes_isolate_false_joined_to_it() {
        // `y` declares isolate: false on its join edge from `c`, so it rides
        // with `c`. Joining `src`'s unit therefore brings 1 + 2 ms against a
        // 10 ms period, which reaches a quarter of it.
        let mut g = graph(
            &[("src", "c")],
            &[("c", "y", ChainBar::ConsumerJoin { trigger_edges: 2 })],
            &[("src", 10)],
        );
        g.isolate.insert("y".into(), false);
        let c = costs(&[("src", 0), ("c", 1_000_000), ("y", 2_000_000)], &[]);
        let units = derive_units("g", &s(&["src", "c", "y"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["src"]), s(&["c", "y"])]);
        let Some(Boundary::OverBudget(v)) = start_of(&units, "c") else {
            panic!("c must start a unit over budget: {units:?}");
        };
        assert_eq!(v.unit_ns, 3_000_000);
    }

    #[test]
    fn a_link_inside_a_unit_an_isolate_false_join_made_adds_no_ticks() {
        // `s -> b -> y` is a chain; the join `x`, triggered by `s` and `b`,
        // declares isolate: false, so `s`, `b` and `x` share one unit before
        // the walk (1.2 ms of ticks). The link `s -> b` lies inside that unit
        // and adds nothing; `y` brings 0.2 ms, so the unit ticks 1.4 ms, under
        // a quarter of the 10 ms period, and `y` joins.
        let join = ChainBar::ConsumerJoin { trigger_edges: 2 };
        let mut g = graph(
            &[("s", "b"), ("b", "y")],
            &[("s", "x", join.clone()), ("b", "x", join)],
            &[("s", 10)],
        );
        g.isolate.insert("x".into(), false);
        let c = costs(
            &[
                ("s", 100_000),
                ("b", 100_000),
                ("x", 1_000_000),
                ("y", 200_000),
            ],
            &[],
        );
        let units = derive_units("g", &s(&["s", "b", "x", "y"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["s", "b", "x", "y"])], "{units:?}");
        assert_eq!(units[0].boundaries, [("s".to_string(), Boundary::Source)]);
    }

    fn over_period(node: &str, unit_ns: u64, period_ns: u64) -> Boundary {
        Boundary::OverBudget(BudgetVerdict {
            node: node.to_string(),
            unit_ns,
            budget_ns: period_ns / BUDGET_DIVISOR,
            against: BudgetAgainst::Period { period_ns },
            profile: "costs.yaml".to_string(),
        })
    }

    #[test]
    fn a_node_refused_over_budget_does_not_rejoin_through_a_later_member_of_that_unit() {
        // `s -> b -> y` is a chain at 100 Hz; the join `x`, triggered by `s`
        // and `y`, declares isolate: false, so `s`, `x` and `y` share one unit
        // (1.6 ms) before the walk. `b` brings 1 ms: 2.6 ms reaches a quarter
        // of the 10 ms period, so `b` starts a unit. The profile measured
        // `s -> b` at 90 Hz (an 11.1 ms period). The link `b -> y` would put
        // the same 2.6 ms back into the unit `s` feeds every 10 ms, so `y`
        // stays with `s` and says why.
        let join = ChainBar::ConsumerJoin { trigger_edges: 2 };
        let mut g = graph(
            &[("s", "b"), ("b", "y")],
            &[("s", "x", join.clone()), ("y", "x", join.clone())],
            &[("s", 10)],
        );
        g.isolate.insert("x".into(), false);
        let c = costs(
            &[
                ("s", 1_000_000),
                ("b", 1_000_000),
                ("x", 300_000),
                ("y", 300_000),
            ],
            &[("s", "b", 90_000)],
        );
        let units = derive_units("g", &s(&["s", "b", "x", "y"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["s", "x", "y"]), s(&["b"])], "{units:?}");
        assert_eq!(
            units[0].boundaries,
            [
                ("s".to_string(), Boundary::Source),
                ("y".to_string(), over_period("y", 2_600_000, 10_000_000)),
            ]
        );
        assert_eq!(
            units[1].boundaries,
            [("b".to_string(), over_period("b", 2_600_000, 10_000_000))]
        );

        // One hop later: `s -> b -> c -> y` with `x` joining `b` and `y`.
        // `b` (with `x` and `y`) joins `s` at 1.6 ms; `c` brings 1 ms and
        // starts a unit; `y` would put `c` back into the 10 ms unit.
        let mut g = graph(
            &[("s", "b"), ("b", "c"), ("c", "y")],
            &[("b", "x", join.clone()), ("y", "x", join)],
            &[("s", 10)],
        );
        g.isolate.insert("x".into(), false);
        let c = costs(
            &[
                ("s", 500_000),
                ("b", 500_000),
                ("c", 1_000_000),
                ("x", 300_000),
                ("y", 300_000),
            ],
            &[("b", "c", 90_000)],
        );
        let units = derive_units("g", &s(&["s", "b", "c", "x", "y"]), &g, profile(&c));
        assert_eq!(
            nodes(&units),
            [s(&["s", "b", "x", "y"]), s(&["c"])],
            "{units:?}"
        );
        assert_eq!(
            start_of(&units, "c"),
            Some(&over_period("c", 2_600_000, 10_000_000))
        );
        assert_eq!(
            start_of(&units, "y"),
            Some(&over_period("y", 2_600_000, 10_000_000))
        );
    }

    #[test]
    fn a_merge_into_a_unit_another_node_feeds_is_judged_at_the_faster_period() {
        // `h2` (50 Hz) links to `y`; the join `x`, triggered by `y` and the
        // 100 Hz source `h1`, declares isolate: false, so `y`, `h1` and `x`
        // share one unit (2.1 ms) fed every 10 ms. With `h2` the unit would
        // tick 4.1 ms: under a quarter of `h2`'s 20 ms period, over a quarter
        // of `h1`'s 10 ms one.
        let join = ChainBar::ConsumerJoin { trigger_edges: 2 };
        let mut g = graph(
            &[("h2", "y")],
            &[("h1", "x", join.clone()), ("y", "x", join)],
            &[("h1", 10), ("h2", 20)],
        );
        g.isolate.insert("x".into(), false);
        let c = costs(
            &[
                ("h2", 2_000_000),
                ("y", 300_000),
                ("h1", 1_500_000),
                ("x", 300_000),
            ],
            &[],
        );
        let units = derive_units("g", &s(&["h2", "y", "h1", "x"]), &g, profile(&c));
        assert_eq!(
            nodes(&units),
            [s(&["h2"]), s(&["y", "h1", "x"])],
            "{units:?}"
        );
        assert_eq!(
            units[1].boundaries,
            [
                ("y".to_string(), over_period("y", 4_100_000, 10_000_000)),
                ("h1".to_string(), Boundary::Source),
            ]
        );
    }

    /// Each unit's members, sorted, and the units sorted: a partition
    /// compared whatever the member order.
    fn unit_sets(units: &[ExecutorUnit]) -> Vec<Vec<String>> {
        let mut sets: Vec<Vec<String>> = units
            .iter()
            .map(|u| {
                let mut nodes = u.nodes.clone();
                nodes.sort();
                nodes
            })
            .collect();
        sets.sort();
        sets
    }

    fn judged_at_period(units: &[ExecutorUnit], node: &str, period_ns: u64) -> bool {
        matches!(
            start_of(units, node),
            Some(Boundary::OverBudget(BudgetVerdict {
                against: BudgetAgainst::Period { period_ns: p },
                ..
            })) if *p == period_ns
        )
    }

    #[test]
    fn a_group_two_chains_feed_is_judged_at_the_faster_chain_in_either_member_order() {
        // `hA` (10 Hz) links to `y` and `hB` (100 Hz) to `z`; the join `x`,
        // triggered by `y` and `z`, declares isolate: false, so `y`, `z` and
        // `x` share one unit (0.9 ms) fed every 10 ms through `z`. With `hA`
        // it would tick 2.9 ms, over a quarter of 10 ms; with `hB` 1.9 ms,
        // under it. `hB` joins and `hA` does not, whichever head's walk runs
        // first and whether or not the profile rates the links.
        let join = ChainBar::ConsumerJoin { trigger_edges: 2 };
        let mut g = graph(
            &[("hA", "y"), ("hB", "z")],
            &[("y", "x", join.clone()), ("z", "x", join.clone())],
            &[("hA", 100), ("hB", 10)],
        );
        g.isolate.insert("x".into(), false);
        let p50 = [
            ("hA", 2_000_000),
            ("hB", 1_000_000),
            ("y", 300_000),
            ("z", 300_000),
            ("x", 300_000),
        ];
        let unrated = costs(&p50, &[]);
        let rated = costs(
            &p50,
            &[
                ("hA", "y", 10_000),
                ("hB", "z", 100_000),
                ("y", "x", 10_000),
                ("z", "x", 100_000),
            ],
        );
        for c in [&unrated, &rated] {
            for order in [["hA", "hB", "y", "z", "x"], ["hB", "hA", "y", "z", "x"]] {
                let units = derive_units("g", &s(&order), &g, profile(c));
                assert_eq!(
                    unit_sets(&units),
                    [s(&["hA"]), s(&["hB", "x", "y", "z"])],
                    "{order:?}: {units:?}"
                );
                assert!(judged_at_period(&units, "y", 10_000_000), "{units:?}");
            }
        }

        // The same shape with the heads' members interleaved: `h1` (100 Hz)
        // links to `y`, `h2` (50 Hz) to `w`, and `x` joins `w` and `y`. With
        // `h1` the unit ticks 2.4 ms, under a quarter of 10 ms; with `h2`
        // 2.9 ms, over it.
        let mut g = graph(
            &[("h2", "w"), ("h1", "y")],
            &[("w", "x", join.clone()), ("y", "x", join)],
            &[("h1", 10), ("h2", 20)],
        );
        g.isolate.insert("x".into(), false);
        let c = costs(
            &[
                ("h2", 2_000_000),
                ("w", 300_000),
                ("h1", 1_500_000),
                ("y", 300_000),
                ("x", 300_000),
            ],
            &[],
        );
        for order in [["h2", "w", "h1", "y", "x"], ["h1", "y", "h2", "w", "x"]] {
            let units = derive_units("g", &s(&order), &g, profile(&c));
            assert_eq!(
                unit_sets(&units),
                [s(&["h1", "w", "x", "y"]), s(&["h2"])],
                "{order:?}: {units:?}"
            );
            assert!(judged_at_period(&units, "w", 10_000_000), "{units:?}");
        }
    }

    #[test]
    fn an_isolate_false_members_feed_from_another_process_bounds_a_merge_into_its_unit() {
        // `s` (10 Hz) links to `b`; the join `x`, triggered by `b` and by `o`
        // of another process at 1 kHz, declares isolate: false, so `b` and
        // `x` share one unit fed every 1 ms. With `s` it would tick 0.4 ms,
        // over a quarter of 1 ms.
        let join = ChainBar::ConsumerJoin { trigger_edges: 2 };
        let mut g = graph(
            &[("s", "b")],
            &[("b", "x", join.clone()), ("o", "x", join)],
            &[("s", 100)],
        );
        g.isolate.insert("x".into(), false);
        let c = costs(
            &[("s", 100_000), ("b", 100_000), ("x", 200_000)],
            &[("o", "x", 1_000_000)],
        );
        let units = derive_units("g", &s(&["s", "b", "x"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["s"]), s(&["b", "x"])], "{units:?}");
        assert_eq!(
            start_of(&units, "b"),
            Some(&over_period("b", 400_000, 1_000_000))
        );
    }

    #[test]
    fn a_node_no_graph_node_triggers_is_fed_at_its_profiled_fire_rate() {
        // `e` has no period and no trigger input from a graph node (an
        // external source). It and the 100 Hz link `h -> y` trigger the join
        // `x`, which declares isolate: false, so `e`, `y` and `x` share one
        // unit. The profile rates `e`'s edge out at 50 Hz: the unit is fed
        // every 10 ms and `y` joins `h` at 1.8 ms. At 1 kHz the unit is fed
        // every 1 ms and `y` stays out.
        let join = ChainBar::ConsumerJoin { trigger_edges: 2 };
        let mut g = graph(
            &[("h", "y")],
            &[("e", "x", join.clone()), ("y", "x", join)],
            &[("h", 10)],
        );
        g.isolate.insert("x".into(), false);
        let p50 = [
            ("e", 200_000),
            ("h", 1_000_000),
            ("y", 300_000),
            ("x", 300_000),
        ];
        let members = s(&["e", "h", "y", "x"]);
        let slow = costs(
            &p50,
            &[("h", "y", 100_000), ("e", "x", 50_000), ("y", "x", 100_000)],
        );
        let units = derive_units("g", &members, &g, profile(&slow));
        assert_eq!(nodes(&units), [s(&["e", "h", "y", "x"])], "{units:?}");
        let fast = costs(
            &p50,
            &[
                ("h", "y", 100_000),
                ("e", "x", 1_000_000),
                ("y", "x", 100_000),
            ],
        );
        let units = derive_units("g", &members, &g, profile(&fast));
        assert_eq!(nodes(&units), [s(&["e", "y", "x"]), s(&["h"])], "{units:?}");
        assert_eq!(
            start_of(&units, "y"),
            Some(&over_period("y", 1_800_000, 1_000_000))
        );

        // A chain such a node heads merges at that rate. Without the rate the
        // refusal names the node whose feed is unknown.
        let g = graph(&[("e", "z")], &[], &[]);
        let c = costs(&[("e", 100_000), ("z", 100_000)], &[("e", "z", 100_000)]);
        let units = derive_units("g", &s(&["e", "z"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["e", "z"])], "{units:?}");
        let c = costs(&[("e", 100_000), ("z", 100_000)], &[]);
        let units = derive_units("g", &s(&["e", "z"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["e"]), s(&["z"])], "{units:?}");
        let unknown = Boundary::UnknownFeed { node: "e".into() };
        assert_eq!(start_of(&units, "z"), Some(&unknown));
        assert_eq!(
            unknown.describe(),
            "the profile holds no rate for how often e is fed, so the merged unit's period is \
             unknown (run `cerulion graph profile` while e is fed)"
        );
    }

    /// A trigger input of `consumer` whose topic no graph node publishes.
    fn outside_input(consumer: &str, topic: &str) -> ConsumerEdgeVerdict {
        ConsumerEdgeVerdict {
            topic: topic.to_string(),
            producer: None,
            consumer: consumer.to_string(),
            input: "outside".to_string(),
            bar: Some(ChainBar::NoInGraphProducer),
        }
    }

    /// `h` (10 Hz) links to `a`; `x` has no policy and fires on `a`'s topic
    /// and on `/cam`, which no graph node publishes, and declares isolate:
    /// false, so `a` and `x` share one unit. `x` triggers `z` of another
    /// process. Every node ticks 1 ms.
    fn fed_from_outside() -> UnitGraph {
        let join = ChainBar::ConsumerJoin { trigger_edges: 2 };
        let mut g = graph(&[("h", "a")], &[("a", "x", join)], &[("h", 100)]);
        g.links.push(outside_input("x", "/cam"));
        g.trigger_edges.push(("x".into(), "z".into()));
        g.isolate.insert("x".into(), false);
        g
    }

    const EVERY_TICK_1_MS: [(&str, u64); 3] =
        [("h", 1_000_000), ("a", 1_000_000), ("x", 1_000_000)];

    #[test]
    fn a_trigger_input_no_graph_node_publishes_feeds_the_unit_at_the_members_fire_rate() {
        // The profile rates `h -> a` and `a -> x` at 10 Hz and `x -> z` at
        // 1 kHz: `x` fires every 1 ms, so the unit is fed every 1 ms and `a`
        // stays out of `h`'s unit at 3 ms.
        let c = costs(
            &EVERY_TICK_1_MS,
            &[
                ("h", "a", 10_000),
                ("a", "x", 10_000),
                ("x", "z", 1_000_000),
            ],
        );
        let units = derive_units("g", &s(&["h", "a", "x"]), &fed_from_outside(), profile(&c));
        assert_eq!(nodes(&units), [s(&["h"]), s(&["a", "x"])], "{units:?}");
        assert_eq!(
            start_of(&units, "a"),
            Some(&over_period("a", 3_000_000, 1_000_000))
        );
    }

    #[test]
    fn a_member_fed_from_outside_the_graph_without_a_fire_rate_has_an_unknown_feed() {
        // The profile rates no edge out of `x`, so how often `x` fires is
        // unknown.
        let c = costs(&EVERY_TICK_1_MS, &[("h", "a", 10_000), ("a", "x", 10_000)]);
        let units = derive_units("g", &s(&["h", "a", "x"]), &fed_from_outside(), profile(&c));
        assert_eq!(nodes(&units), [s(&["h"]), s(&["a", "x"])], "{units:?}");
        assert_eq!(
            start_of(&units, "a"),
            Some(&Boundary::UnknownFeed { node: "x".into() })
        );
    }

    /// `h` (10 Hz) links to `a`; the join `x`, triggered by `a` and by
    /// `other`, declares isolate: false, so `a` and `x` share one unit. The
    /// profile rates `h -> a` and `a -> x` at 10 Hz and not `other -> x`, and
    /// every node ticks 1 ms.
    fn rated_beside_unrated(other: &str) -> (UnitGraph, PartitionCosts) {
        let join = ChainBar::ConsumerJoin { trigger_edges: 2 };
        let mut g = graph(
            &[("h", "a")],
            &[("a", "x", join.clone()), (other, "x", join)],
            &[("h", 100)],
        );
        g.isolate.insert("x".into(), false);
        let c = costs(
            &[
                ("h", 1_000_000),
                ("a", 1_000_000),
                ("x", 1_000_000),
                (other, 1_000_000),
            ],
            &[("h", "a", 10_000), ("a", "x", 10_000)],
        );
        (g, c)
    }

    #[test]
    fn an_unrated_trigger_edge_from_another_process_beside_a_rated_one_makes_the_feed_unknown() {
        // `o` is in another process: its edge feeds the unit at an unknown
        // rate.
        let (g, c) = rated_beside_unrated("o");
        let units = derive_units("g", &s(&["h", "a", "x"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["h"]), s(&["a", "x"])], "{units:?}");
        assert_eq!(
            start_of(&units, "a"),
            Some(&Boundary::UnknownFeed { node: "x".into() })
        );
    }

    #[test]
    fn an_unrated_trigger_edge_from_another_group_beside_a_rated_one_carries_its_feed() {
        // `p` is in this process, runs at 1 kHz and declares isolate: true,
        // so `x` does not join it: its edge feeds the unit every 1 ms, and
        // 3 ms reaches a quarter of that.
        let (mut g, c) = rated_beside_unrated("p");
        g.period_ns.insert("p".into(), 1_000_000);
        g.isolate.insert("p".into(), true);
        let units = derive_units("g", &s(&["h", "a", "p", "x"]), &g, profile(&c));
        assert_eq!(
            nodes(&units),
            [s(&["h"]), s(&["a", "x"]), s(&["p"])],
            "{units:?}"
        );
        assert_eq!(
            start_of(&units, "a"),
            Some(&over_period("a", 3_000_000, 1_000_000))
        );
    }

    #[test]
    fn a_feed_carried_through_two_groups_reaches_a_group_listed_before_them() {
        // `f` (1 kHz) links to `g` and `g` to `m`, unrated; the join `x`,
        // triggered by `y` and `m`, declares isolate: false, so `y`, `x` and
        // `m` share one unit, fed through `m` from `f`'s group. `hA` (10 Hz)
        // links to `y`: with `hA` the unit ticks 1.6 ms, over a quarter of
        // 1 ms. The members list `f` after the group `f`'s feed reaches.
        let join = ChainBar::ConsumerJoin { trigger_edges: 2 };
        let mut g = graph(
            &[("hA", "y"), ("f", "g"), ("g", "m")],
            &[("y", "x", join.clone()), ("m", "x", join)],
            &[("hA", 100), ("f", 1)],
        );
        g.isolate.insert("x".into(), false);
        let c = costs(
            &[
                ("hA", 1_000_000),
                ("y", 200_000),
                ("x", 200_000),
                ("m", 200_000),
                ("g", 10_000),
                ("f", 10_000),
            ],
            &[],
        );
        let units = derive_units("g", &s(&["hA", "y", "x", "m", "g", "f"]), &g, profile(&c));
        assert_eq!(
            nodes(&units),
            [s(&["hA"]), s(&["y", "x", "m"]), s(&["g", "f"])],
            "{units:?}"
        );
        assert_eq!(
            units[1].boundaries,
            [
                ("y".to_string(), over_period("y", 1_600_000, 1_000_000)),
                ("m".to_string(), over_period("m", 620_000, 1_000_000)),
            ]
        );
    }

    #[test]
    fn a_later_walk_reads_the_feed_a_merge_gave_the_merged_group() {
        // The join `j`, triggered by `a`, `b` and by `o` of another process at
        // 1 kHz, declares isolate: false, so `a`, `b` and `j` share one unit
        // fed every 1 ms. `h1` (10 Hz) links to `a` and joins at 0.11 ms; the
        // merged unit keeps `h1`'s place and is fed every 1 ms. `h2` (10 Hz)
        // links to `b`: 0.31 ms reaches a quarter of 1 ms.
        let join = ChainBar::ConsumerJoin { trigger_edges: 3 };
        let mut g = graph(
            &[("h1", "a"), ("h2", "b")],
            &[
                ("a", "j", join.clone()),
                ("b", "j", join.clone()),
                ("o", "j", join),
            ],
            &[("h1", 100), ("h2", 100)],
        );
        g.isolate.insert("j".into(), false);
        let c = costs(
            &[
                ("h1", 50_000),
                ("h2", 200_000),
                ("a", 20_000),
                ("b", 20_000),
                ("j", 20_000),
            ],
            &[("o", "j", 1_000_000)],
        );
        let units = derive_units("g", &s(&["h1", "h2", "a", "b", "j"]), &g, profile(&c));
        assert_eq!(
            nodes(&units),
            [s(&["h1", "a", "b", "j"]), s(&["h2"])],
            "{units:?}"
        );
        assert_eq!(
            units[0].boundaries,
            [
                ("h1".to_string(), Boundary::Source),
                ("b".to_string(), over_period("b", 310_000, 1_000_000)),
            ]
        );
    }

    /// `p` (10 Hz) triggers `m` and `c`. The join `m`, also triggered by `o`
    /// of another process at `o_mhz`, declares isolate: false, so `p` and `m`
    /// share one unit. `k` (100 Hz) and `p` trigger the join `c`, and `c`
    /// links to `q`. The profile rates `p -> m`, `k -> c` and `c -> q` and
    /// never `p -> c`, so `c` is fed through that edge as often as the unit
    /// of `p` and `m` is.
    fn fed_through_a_producers_partner(o_mhz: Option<u64>) -> Vec<ExecutorUnit> {
        let join = ChainBar::ConsumerJoin { trigger_edges: 2 };
        let mut g = graph(
            &[("c", "q")],
            &[
                ("p", "m", join.clone()),
                ("o", "m", join.clone()),
                ("p", "c", join.clone()),
                ("k", "c", join),
            ],
            &[("p", 100), ("k", 10)],
        );
        g.isolate.insert("m".into(), false);
        let mut rates = vec![("p", "m", 10_000), ("k", "c", 100_000), ("c", "q", 100_000)];
        if let Some(mhz) = o_mhz {
            rates.push(("o", "m", mhz));
        }
        let c = costs(
            &[
                ("p", 10_000),
                ("m", 10_000),
                ("k", 10_000),
                ("c", 200_000),
                ("q", 200_000),
            ],
            &rates,
        );
        derive_units("g", &s(&["p", "m", "k", "c", "q"]), &g, profile(&c))
    }

    #[test]
    fn an_unknown_feed_names_a_partner_of_the_producer_of_an_unrated_edge() {
        // No rate says how often `o` feeds `m`, so the unit of `p` and `m` is
        // fed at an unknown rate, and so is `c` through the unrated `p -> c`:
        // the merge of `q` is refused naming `m`, which is in neither `c` nor
        // `q` and triggers neither.
        let units = fed_through_a_producers_partner(None);
        assert_eq!(
            nodes(&units),
            [s(&["p", "m"]), s(&["k"]), s(&["c"]), s(&["q"])],
            "{units:?}"
        );
        assert_eq!(
            start_of(&units, "q"),
            Some(&Boundary::UnknownFeed { node: "m".into() })
        );
    }

    #[test]
    fn an_unrated_edge_feeds_its_consumer_at_the_feed_of_the_producers_whole_unit() {
        // `o` feeds `m` at 1 kHz, so the unit of `p` and `m` is fed every
        // 1 ms, and so is `c` through the unrated `p -> c`, though `p` fires
        // every 100 ms: `c` and `q` would tick 0.4 ms, over a quarter of 1 ms.
        let units = fed_through_a_producers_partner(Some(1_000_000));
        assert_eq!(
            nodes(&units),
            [s(&["p", "m"]), s(&["k"]), s(&["c"]), s(&["q"])],
            "{units:?}"
        );
        assert_eq!(
            start_of(&units, "q"),
            Some(&over_period("q", 400_000, 1_000_000))
        );
    }

    /// `h` (1 kHz) links to `a`, and `b` reads `a`'s topic as `block`, so `a`
    /// and `b` share one unit (the link `a -> b` lies inside it).
    fn block_partner() -> UnitGraph {
        let mut g = graph(&[("h", "a"), ("a", "b")], &[], &[("h", 1)]);
        g.couplings.push(Coupling {
            topic: "/t/a/out".into(),
            nodes: s(&["a", "b"]),
        });
        g
    }

    #[test]
    fn a_block_partner_counts_in_the_budget_of_a_link_into_its_unit() {
        // Joining `h` brings both `a` and `b`: 50.02 ms reaches a quarter of
        // 1 ms.
        let c = costs(&[("h", 10_000), ("a", 10_000), ("b", 50_000_000)], &[]);
        let units = derive_units("g", &s(&["h", "a", "b"]), &block_partner(), profile(&c));
        assert_eq!(nodes(&units), [s(&["h"]), s(&["a", "b"])], "{units:?}");
        assert_eq!(units[0].boundaries, [("h".to_string(), Boundary::Source)]);
        assert_eq!(
            units[1].boundaries,
            [("a".to_string(), over_period("a", 50_020_000, 1_000_000))]
        );
        assert_eq!(units[1].block_topics, s(&["/t/a/out"]));
    }

    #[test]
    fn a_profile_that_holds_no_tick_for_a_block_partner_names_only_the_unit_it_starts() {
        // `b` shares `a`'s unit, so only `a` says why it starts one.
        let c = costs(&[("h", 10_000), ("a", 10_000)], &[]);
        let units = derive_units("g", &s(&["h", "a", "b"]), &block_partner(), profile(&c));
        assert_eq!(nodes(&units), [s(&["h"]), s(&["a", "b"])], "{units:?}");
        assert_eq!(
            units[1].boundaries,
            [(
                "a".to_string(),
                Boundary::Unprofiled {
                    producer: "h".into()
                }
            )]
        );
    }

    #[test]
    fn without_a_profile_a_block_partner_shares_the_one_unit() {
        let units = derive_units("g", &s(&["h", "a", "b"]), &block_partner(), None);
        assert_eq!(nodes(&units), [s(&["h", "a", "b"])], "{units:?}");
        assert_eq!(
            units[0].boundaries,
            [("h".to_string(), Boundary::NoProfile)]
        );
        assert_eq!(units[0].block_topics, s(&["/t/a/out"]));
    }

    #[test]
    fn a_fan_out_consumer_a_block_coupling_keeps_with_its_producer_names_no_reason() {
        // `slow` reads `pace`'s fan-out topic and a `block` coupling keeps
        // the two together: `slow` starts no unit. `fast` does.
        let mut g = graph(
            &[],
            &[
                ("pace", "fast", ChainBar::FanOut { consumers: 2 }),
                ("pace", "slow", ChainBar::FanOut { consumers: 2 }),
            ],
            &[("pace", 1)],
        );
        g.couplings.push(Coupling {
            topic: "t".into(),
            nodes: s(&["pace", "slow"]),
        });
        let c = costs(&[("pace", 1), ("fast", 1), ("slow", 1)], &[]);
        let units = derive_units("g", &s(&["pace", "fast", "slow"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["pace", "slow"]), s(&["fast"])]);
        assert_eq!(
            units[0].boundaries,
            [("pace".to_string(), Boundary::Source)]
        );
        assert_eq!(
            start_of(&units, "fast"),
            Some(&Boundary::NotALink {
                producer: "pace".into(),
                bar: ChainBar::FanOut { consumers: 2 }
            })
        );
    }

    #[test]
    fn a_consumer_a_block_coupling_keeps_with_its_isolated_producer_names_no_reason() {
        // `a` declares isolate: true and a `block` coupling keeps it with its
        // consumer `b`: with a profile neither starts a unit; without one the
        // unit is the process's one unit and `b`, its first node that does not
        // declare isolate: true, says so.
        let mut g = graph(&[("a", "b")], &[], &[("a", 10)]);
        g.isolate.insert("a".into(), true);
        g.couplings.push(Coupling {
            topic: "t".into(),
            nodes: s(&["a", "b"]),
        });
        let c = costs(&[("a", 1), ("b", 1)], &[]);
        let units = derive_units("g", &s(&["a", "b"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["a", "b"])]);
        assert!(units[0].boundaries.is_empty(), "{units:?}");

        let units = derive_units("g", &s(&["a", "b"]), &g, None);
        assert_eq!(nodes(&units), [s(&["a", "b"])]);
        assert_eq!(
            units[0].boundaries,
            [("b".to_string(), Boundary::NoProfile)]
        );
        assert_eq!(units[0].block_topics, s(&["t"]));
    }

    #[test]
    fn without_a_profile_a_block_coupling_of_two_isolated_nodes_is_a_unit_of_its_own() {
        // `b` and `c` declare isolate: true and a `block` coupling keeps them
        // together, apart from the process's one unit, which `a` heads.
        let mut g = graph(&[("a", "b"), ("b", "c")], &[], &[("a", 10)]);
        g.isolate.insert("b".into(), true);
        g.isolate.insert("c".into(), true);
        g.couplings.push(Coupling {
            topic: "t".into(),
            nodes: s(&["b", "c"]),
        });
        let units = derive_units("g", &s(&["a", "b", "c"]), &g, None);
        assert_eq!(nodes(&units), [s(&["a"]), s(&["b", "c"])]);
        assert_eq!(
            units[0].boundaries,
            [("a".to_string(), Boundary::NoProfile)]
        );
        assert!(units[1].boundaries.is_empty(), "{units:?}");
        assert_eq!(units[1].block_topics, s(&["t"]));
    }

    #[test]
    fn a_block_flow_keeps_an_isolated_node_and_says_so() {
        let mut g = graph(&[("a", "b")], &[], &[("a", 10)]);
        g.isolate.insert("b".into(), true);
        g.couplings.push(Coupling {
            topic: "t".into(),
            nodes: s(&["a", "b"]),
        });
        let units = derive_units("g", &s(&["a", "b"]), &g, None);
        assert_eq!(nodes(&units), [s(&["a", "b"])]);
        assert_eq!(start_of(&units, "b"), None, "b does not start a unit");
        assert_eq!(units[0].block_topics, s(&["t"]));
    }

    // ---- Structure: the shared chain predicate's bars.

    #[test]
    fn an_edge_the_chain_predicate_refuses_starts_a_unit_with_its_bar() {
        let g = graph(
            &[],
            &[
                ("pace", "fast", ChainBar::FanOut { consumers: 2 }),
                ("pace", "slow", ChainBar::FanOut { consumers: 2 }),
            ],
            &[("pace", 1)],
        );
        let c = costs(&[("pace", 1), ("fast", 1), ("slow", 1)], &[]);
        let units = derive_units("g", &s(&["pace", "fast", "slow"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["pace"]), s(&["fast"]), s(&["slow"])]);
        assert_eq!(
            units.iter().map(|u| u.name.as_str()).collect::<Vec<_>>(),
            ["g.pace", "g.fast", "g.slow"]
        );
        assert_eq!(
            start_of(&units, "fast"),
            Some(&Boundary::NotALink {
                producer: "pace".into(),
                bar: ChainBar::FanOut { consumers: 2 }
            })
        );
    }

    #[test]
    fn a_consumer_of_another_process_is_a_source() {
        let g = graph(&[("x", "b"), ("b", "c")], &[], &[]);
        let c = costs(&[("b", 1), ("c", 1)], &[("x", "b", 1_000_000)]);
        let units = derive_units("g", &s(&["b", "c"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["b", "c"])]);
        assert_eq!(start_of(&units, "b"), Some(&Boundary::Source));
    }

    #[test]
    fn a_unit_fed_at_an_unknown_rate_does_not_merge() {
        // `b` is fed by another process at a rate the profile does not hold.
        let g = graph(&[("x", "b"), ("b", "c")], &[], &[]);
        let c = costs(&[("b", 1), ("c", 1)], &[]);
        let units = derive_units("g", &s(&["b", "c"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["b"]), s(&["c"])]);
        assert_eq!(
            start_of(&units, "c"),
            Some(&Boundary::UnknownFeed { node: "b".into() })
        );
    }

    // ---- Overrides.

    #[test]
    fn isolate_true_gives_a_node_its_own_unit_and_ends_the_chain_after_it() {
        let mut g = chain3();
        g.isolate.insert("det".into(), true);
        let c = costs(&[("cam", 1), ("det", 1), ("track", 1)], &[]);
        let units = derive_units("g", &s(&["cam", "det", "track"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["cam"]), s(&["det"]), s(&["track"])]);
        assert_eq!(start_of(&units, "det"), Some(&Boundary::Isolated));
        assert_eq!(
            start_of(&units, "track"),
            Some(&Boundary::AfterIsolated {
                producer: "det".into()
            })
        );
    }

    #[test]
    fn isolate_false_merges_across_a_fan_out_and_a_fan_in_whatever_the_link_rules_say() {
        let mut g = graph(
            &[],
            &[
                ("pace", "fast", ChainBar::FanOut { consumers: 2 }),
                ("pace", "slow", ChainBar::FanOut { consumers: 2 }),
            ],
            &[("pace", 1)],
        );
        g.isolate.insert("fast".into(), false);
        let c = costs(&[("pace", 1), ("fast", 1), ("slow", 1)], &[]);
        let units = derive_units("g", &s(&["pace", "fast", "slow"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["pace", "fast"]), s(&["slow"])]);

        let join = ChainBar::ConsumerJoin { trigger_edges: 2 };
        let mut j = graph(
            &[],
            &[("a", "join", join.clone()), ("c", "join", join)],
            &[("a", 10), ("c", 10)],
        );
        j.isolate.insert("join".into(), false);
        let c = costs(&[("a", 1), ("c", 1), ("join", 1)], &[]);
        let units = derive_units("g", &s(&["a", "c", "join"]), &j, profile(&c));
        assert_eq!(nodes(&units), [s(&["a", "c", "join"])]);
    }

    #[test]
    fn a_producer_isolate_true_wins_over_its_consumer_isolate_false() {
        let mut g = graph(&[("a", "b")], &[], &[("a", 10)]);
        g.isolate.insert("a".into(), true);
        g.isolate.insert("b".into(), false);
        let c = costs(&[("a", 1), ("b", 1)], &[]);
        let units = derive_units("g", &s(&["a", "b"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["a"]), s(&["b"])]);
        assert_eq!(
            start_of(&units, "b"),
            Some(&Boundary::AfterIsolated {
                producer: "a".into()
            })
        );
    }

    #[test]
    fn a_block_flow_stays_one_unit() {
        let mut g = graph(
            &[],
            &[
                ("pace", "fast", ChainBar::FanOut { consumers: 2 }),
                ("pace", "slow", ChainBar::FanOut { consumers: 2 }),
            ],
            &[("pace", 1)],
        );
        g.couplings.push(Coupling {
            topic: "t".into(),
            nodes: s(&["pace", "slow"]),
        });
        let c = costs(&[("pace", 1), ("fast", 1), ("slow", 1)], &[]);
        let units = derive_units("g", &s(&["pace", "fast", "slow"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["pace", "slow"]), s(&["fast"])]);
        assert_eq!(units[0].block_topics, s(&["t"]));
    }

    #[test]
    fn a_cycle_of_links_starts_a_unit_with_a_reason() {
        let g = graph(&[("a", "b"), ("b", "a")], &[], &[]);
        let c = costs(&[("a", 1), ("b", 1)], &[]);
        let units = derive_units("g", &s(&["a", "b"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["a"]), s(&["b"])]);
        assert!(units.iter().all(|u| !u.boundaries.is_empty()));
        assert_eq!(
            start_of(&units, "a"),
            Some(&Boundary::LinkCycle {
                producer: "b".into()
            })
        );
    }

    #[test]
    fn units_keep_member_order() {
        let g = graph(&[("c", "a")], &[], &[("c", 10)]);
        let c = costs(&[("a", 1), ("c", 1)], &[]);
        let units = derive_units("g", &s(&["a", "b", "c"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["a", "c"]), s(&["b"])]);
    }

    fn config_with_groups(groups: &[(&str, &[&str])], order: &[&str]) -> GraphConfig {
        let mut config = GraphConfig {
            level_assignments: None,
            network: None,
            name: None,
            identity: "units".to_string(),
            prefix: "p".to_string(),
            nodes: Vec::new(),
            multi_publisher_topics: Vec::new(),
            process_groups: IndexMap::new(),
            process_group_order: Vec::new(),
        };
        for (g, m) in groups {
            config.process_groups.insert(g.to_string(), s(m));
        }
        config.process_group_order = s(order);
        config
    }

    #[test]
    fn expansion_replaces_each_group_by_its_units_in_place() {
        let config = config_with_groups(&[("cam", &["cam"]), ("work", &["cam2", "det"])], &[]);
        let mut g = graph(&[("cam2", "det")], &[], &[("cam2", 1)]);
        g.isolate.insert("det".into(), true);
        let plan = expand_process_groups(&config, &g, None, &IndexSet::new());
        let keys: Vec<&str> = plan.groups.keys().map(String::as_str).collect();
        assert_eq!(keys, ["cam", "work.cam2", "work.det"]);
        assert_eq!(plan.hosts.get("work.det").map(String::as_str), Some("work"));
        assert!(!plan.hosts.contains_key("cam"));
        assert!(plan.splits_any());
        assert_eq!(plan.apply(&config).process_groups, plan.groups);
    }

    #[test]
    fn expansion_rewrites_the_declared_order() {
        let config = config_with_groups(&[("a", &["x", "y"]), ("b", &["z"])], &["b", "a"]);
        let mut g = UnitGraph::default();
        g.isolate.insert("y".into(), true);
        let plan = expand_process_groups(&config, &g, None, &IndexSet::new());
        assert_eq!(plan.order, s(&["b", "a.x", "a.y"]));
    }

    #[test]
    fn a_kept_whole_group_is_one_unit_under_its_own_name() {
        let config = config_with_groups(&[("work", &["pace", "fast"])], &[]);
        let keep: IndexSet<String> = ["work".to_string()].into_iter().collect();
        let plan = expand_process_groups(&config, &UnitGraph::default(), None, &keep);
        assert_eq!(plan.groups.len(), 1);
        assert_eq!(plan.groups["work"], s(&["pace", "fast"]));
        assert!(plan.hosts.is_empty());
        assert!(!plan.splits_any());
    }

    #[test]
    fn a_group_whose_unit_name_collides_stays_one_unit() {
        // `x` and `a_b` declare isolate: true. `work`'s unit for `x` would be
        // named `work.x`, which a declared group already is; under a key
        // folding `.` to `_`, `a-b` and `a_b` collide too.
        let mut isolated = UnitGraph::default();
        isolated.isolate.insert("x".into(), true);
        isolated.isolate.insert("a_b".into(), true);
        let config = config_with_groups(&[("work", &["p", "x"]), ("work.x", &["y"])], &[]);
        let plan = expand_process_groups(&config, &isolated, None, &IndexSet::new());
        assert_eq!(plan.name_collisions, s(&["work"]));
        assert_eq!(plan.groups["work"], s(&["p", "x"]));
        assert_eq!(plan.groups["work.x"], s(&["y"]));

        let config = config_with_groups(&[("g", &["a-b", "a_b"])], &[]);
        let fold = |n: &str| n.replace(['-', '.'], "_");
        let plan = expand_process_groups_keyed(&config, &isolated, None, &IndexSet::new(), &fold);
        assert_eq!(plan.name_collisions, s(&["g"]));
        assert_eq!(plan.groups.len(), 1);
    }

    #[test]
    fn a_process_per_node_graph_plans_unchanged() {
        let config = config_with_groups(&[("grp_a", &["a"]), ("grp_b", &["b"])], &[]);
        let g = graph(&[("a", "b")], &[], &[("a", 10)]);
        let plan = expand_process_groups(&config, &g, None, &IndexSet::new());
        assert_eq!(plan.groups, config.process_groups);
        assert!(plan.hosts.is_empty());
    }
}
