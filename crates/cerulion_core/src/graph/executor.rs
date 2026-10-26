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
//! # Every node starts alone; a profile merges links
//!
//! A node may join its producer's unit only across a LINK: an edge the shared
//! chain predicates ([`crate::graph::chain::linear_links`]) find to be a linear
//! hand-off, one producer handing each frame to one consumer that fires on it.
//! The same predicates serve the fusion census and the partitioner, so the three
//! agree on what a chain is.
//!
//! A link merges only when the graph's profile ([`PartitionCosts`], the cost
//! snapshot `graph profile` writes) shows the merge is safe: the unit's summed
//! p50 tick, the new node included, stays under [`BUDGET_DIVISOR`]⁻¹ of the
//! period the unit is fed at, and under the same share of the node's own
//! `tick_within_ms`. Without a profile, or without a profiled tick for every node
//! involved, nothing merges: every data-triggered node is a unit of its own, so a
//! slow node never holds a fast one from the first run.
//!
//! Each node that starts a unit carries the reason ([`Boundary`]): a source, an
//! edge that is no link (latest-value, fan-out, join, a period or sync
//! consumer...), `isolate: true`, an isolated producer, an unprofiled merge, or
//! a merge over budget.
//!
//! # Overrides
//!
//! `isolate: true` puts the node in a unit of its own. `isolate: false` joins
//! the node to the unit of every same-process node that triggers it, whatever
//! the automatic rules say: a fan-in node merges its producers' units, a
//! fan-out consumer shares the producer's unit, and no profile is needed. When a
//! producer declares `isolate: true` and its consumer `isolate: false`, the
//! producer's declaration wins.
//!
//! A [`Coupling`] (every producer and consumer of a topic with a `block`
//! consumer) merges its nodes into one unit after every other rule: the
//! producer's deferral reads an outstanding count kept by the consumers' drains
//! inside one level loop.
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
use crate::graph::topology::{GraphTopology, TriggerEdges};

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
    /// A link whose merge no profile shows to be safe.
    Unprofiled {
        /// The producer across the link.
        producer: String,
    },
    /// A link whose merge would put the unit over its tick budget.
    OverBudget(BudgetVerdict),
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
                "no profile shows that joining {producer}'s unit is safe (run `cerulion graph \
                 profile` to let a chain merge)"
            ),
            Self::OverBudget(v) => v.describe(),
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
            BudgetAgainst::Deadline { deadline_ns } => {
                format!("{}'s {deadline_ns} ns tick_within", self.node)
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
    /// The joining node's own `tick_within_ms`.
    Deadline {
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
    /// ([`linear_links`]); an edge with no bar is a link.
    pub links: Vec<ConsumerEdgeVerdict>,
    /// Each node's `isolate:` declaration.
    pub isolate: IndexMap<String, bool>,
    /// Each period node's period.
    pub period_ns: IndexMap<String, u64>,
    /// Each node's `tick_within_ms`, in ns.
    pub tick_within_ns: IndexMap<String, u64>,
    /// The `block` couplings.
    pub couplings: Vec<Coupling>,
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
/// [`derive_units`] merge a link that no profile shows to be safe.
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

/// The period a unit headed by `head` is fed at: the head's own period, else
/// the fastest profiled rate of its trigger edges.
fn head_period_ns(head: &str, graph: &UnitGraph, costs: Option<&PartitionCosts>) -> Option<u64> {
    if let Some(p) = graph.period_ns.get(head) {
        return Some(*p);
    }
    let costs = costs?;
    graph
        .trigger_edges
        .iter()
        .filter(|(_, c)| c == head)
        .filter_map(|(p, c)| costs.edge_rate_mhz.get(&(p.clone(), c.clone())))
        .copied()
        .filter(|r| *r > 0)
        .max()
        .map(|mhz| 1_000_000_000_000 / mhz)
}

/// A unit being grown along one chain.
struct Growing {
    /// The summed p50 tick of its members, when every one is profiled.
    sum_ns: Option<u64>,
    /// The period it is fed at, when known.
    period_ns: Option<u64>,
    /// The tightest `tick_within_ms` among its members.
    deadline_ns: Option<u64>,
}

/// Decide whether `node` may join the unit `unit` across the link from
/// `producer`. `Ok(sum)` = merge, with the unit's new sum.
fn merge_verdict(
    node: &str,
    producer: &str,
    unit: &Growing,
    graph: &UnitGraph,
    profile: Option<UnitProfile<'_>>,
) -> Result<Option<u64>, Boundary> {
    let unprofiled = || Boundary::Unprofiled {
        producer: producer.to_string(),
    };
    let Some(profile) = profile else {
        return if merge_mutant() {
            Ok(None)
        } else {
            Err(unprofiled())
        };
    };
    let (Some(sum), Some(p50), Some(period_ns)) = (
        unit.sum_ns,
        profile.costs.node_p50_ns.get(node).copied(),
        unit.period_ns,
    ) else {
        return if merge_mutant() {
            Ok(None)
        } else {
            Err(unprofiled())
        };
    };
    let unit_ns = sum.saturating_add(p50);
    if budget_mutant() {
        return Ok(Some(unit_ns));
    }
    let deadline = match (unit.deadline_ns, graph.tick_within_ns.get(node).copied()) {
        (Some(a), Some(b)) => Some(a.min(b)),
        (a, b) => a.or(b),
    };
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
    if let Some(deadline_ns) = deadline {
        if over(deadline_ns) {
            return Err(verdict(
                BudgetAgainst::Deadline { deadline_ns },
                deadline_ns,
            ));
        }
    }
    Ok(Some(unit_ns))
}

/// Split one host's members into executor units.
///
/// Units are ordered by their first member and list their members in
/// `members` order, so the result depends only on the inputs.
pub fn derive_units(
    host: &str,
    members: &[String],
    graph: &UnitGraph,
    profile: Option<UnitProfile<'_>>,
) -> Vec<ExecutorUnit> {
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

    // Walk each chain of candidate links from the node that heads it, growing
    // one unit at a time.
    for node in members {
        let head = node.as_str();
        if candidates.contains(head) {
            continue;
        }
        let Some(&first) = link_out.get(head) else {
            continue;
        };
        if !candidates.contains(first) {
            continue;
        }
        // The head's unit so far: the head and every node `isolate: false`
        // already joined to it.
        let head_root = index(head).map(|i| root(&mut parent, i));
        let mut sum_ns = Some(0u64);
        let mut deadline_ns: Option<u64> = None;
        for (i, member) in members.iter().enumerate() {
            if Some(root(&mut parent, i)) != head_root {
                continue;
            }
            sum_ns = match (sum_ns, costs.and_then(|c| c.node_p50_ns.get(member))) {
                (Some(acc), Some(p50)) => Some(acc.saturating_add(*p50)),
                _ => None,
            };
            if let Some(&d) = graph.tick_within_ns.get(member) {
                deadline_ns = Some(deadline_ns.map_or(d, |x: u64| x.min(d)));
            }
        }
        let mut unit = Growing {
            sum_ns: costs.and(sum_ns),
            period_ns: head_period_ns(head, graph, costs),
            deadline_ns,
        };
        let mut at = head;
        while let Some(&next) = link_out.get(at) {
            if !candidates.contains(next) {
                break;
            }
            match merge_verdict(next, at, &unit, graph, profile) {
                Ok(sum) => {
                    if let (Some(a), Some(b)) = (index(next), index(at)) {
                        union(&mut parent, a, b);
                    }
                    unit.sum_ns = sum;
                    if let Some(&d) = graph.tick_within_ns.get(next) {
                        unit.deadline_ns = Some(unit.deadline_ns.map_or(d, |x| x.min(d)));
                    }
                }
                Err(boundary) => {
                    starts.insert(next, boundary);
                    unit = Growing {
                        deadline_ns: graph.tick_within_ns.get(next).copied(),
                        sum_ns: costs.and_then(|c| c.node_p50_ns.get(next).copied()),
                        period_ns: costs
                            .and_then(|c| c.edge_rate_mhz.get(&(at.to_string(), next.to_string())))
                            .copied()
                            .filter(|r| *r > 0)
                            .map(|mhz| 1_000_000_000_000 / mhz)
                            .or(unit.period_ns),
                    };
                }
            }
            at = next;
        }
    }

    let mut block_topics: IndexMap<usize, Vec<String>> = IndexMap::new();
    for coupling in &graph.couplings {
        let local: Vec<usize> = coupling.nodes.iter().filter_map(|n| index(n)).collect();
        if local.len() < 2 {
            continue;
        }
        for pair in local.windows(2) {
            union(&mut parent, pair[0], pair[1]);
        }
        let r = root(&mut parent, local[0]);
        block_topics
            .entry(r)
            .or_default()
            .push(coupling.topic.clone());
    }
    let mut topics_by_root: IndexMap<usize, Vec<String>> = IndexMap::new();
    for (r, topics) in block_topics {
        let fr = root(&mut parent, r);
        topics_by_root.entry(fr).or_default().extend(topics);
    }
    // A `block` flow keeps its nodes together over `isolate: true`; such a
    // node does not start a unit, and the unit lists the block topic instead.
    for (i, member) in members.iter().enumerate() {
        let r = root(&mut parent, i);
        if topics_by_root.contains_key(&r)
            && starts.get(member.as_str()) == Some(&Boundary::Isolated)
        {
            starts.shift_remove(member.as_str());
        }
    }

    let mut groups: IndexMap<usize, Vec<String>> = IndexMap::new();
    for (i, member) in members.iter().enumerate() {
        let r = root(&mut parent, i);
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
    expand_process_groups_keyed(config, graph, profile, keep_whole, &|name| {
        name.to_string()
    })
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
                owners.entry(key(&unit.name)).or_default().push(host.as_str());
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
        g
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

    // ---- Without a profile, every data-triggered node is its own unit.

    #[test]
    fn without_a_profile_every_data_triggered_node_is_its_own_unit() {
        let units = derive_units("g", &s(&["cam", "det", "track"]), &chain3(), None);
        assert_eq!(nodes(&units), [s(&["cam"]), s(&["det"]), s(&["track"])]);
        assert_eq!(start_of(&units, "cam"), Some(&Boundary::Source));
        assert_eq!(
            start_of(&units, "det"),
            Some(&Boundary::Unprofiled {
                producer: "cam".into()
            })
        );
        assert_eq!(
            start_of(&units, "track"),
            Some(&Boundary::Unprofiled {
                producer: "det".into()
            })
        );
    }

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
                deadline_ns: 10_000_000
            }
        );
    }

    #[test]
    fn an_earlier_members_deadline_bounds_every_later_merge() {
        // `mid` declares tick_within 10 ms; `out` ticks 2 ms more, which
        // pushes the unit past a quarter of `mid`'s deadline.
        let mut g = graph(&[("src", "mid"), ("mid", "out")], &[], &[("src", 1000)]);
        g.tick_within_ns.insert("mid".into(), 10_000_000);
        let c = costs(&[("src", 0), ("mid", 1_000_000), ("out", 2_000_000)], &[]);
        let units = derive_units("g", &s(&["src", "mid", "out"]), &g, profile(&c));
        assert_eq!(nodes(&units), [s(&["src", "mid"]), s(&["out"])]);
        assert!(matches!(
            start_of(&units, "out"),
            Some(Boundary::OverBudget(BudgetVerdict {
                against: BudgetAgainst::Deadline {
                    deadline_ns: 10_000_000
                },
                ..
            }))
        ));
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
    fn isolate_false_merges_without_a_profile_across_a_fan_out_and_a_fan_in() {
        let mut g = graph(
            &[],
            &[
                ("pace", "fast", ChainBar::FanOut { consumers: 2 }),
                ("pace", "slow", ChainBar::FanOut { consumers: 2 }),
            ],
            &[("pace", 1)],
        );
        g.isolate.insert("fast".into(), false);
        let units = derive_units("g", &s(&["pace", "fast", "slow"]), &g, None);
        assert_eq!(nodes(&units), [s(&["pace", "fast"]), s(&["slow"])]);

        let join = ChainBar::ConsumerJoin { trigger_edges: 2 };
        let mut j = graph(
            &[],
            &[("a", "join", join.clone()), ("c", "join", join)],
            &[("a", 10), ("c", 10)],
        );
        j.isolate.insert("join".into(), false);
        let units = derive_units("g", &s(&["a", "c", "join"]), &j, None);
        assert_eq!(nodes(&units), [s(&["a", "c", "join"])]);
    }

    #[test]
    fn a_producer_isolate_true_wins_over_its_consumer_isolate_false() {
        let mut g = graph(&[("a", "b")], &[], &[("a", 10)]);
        g.isolate.insert("a".into(), true);
        g.isolate.insert("b".into(), false);
        let units = derive_units("g", &s(&["a", "b"]), &g, None);
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
        let units = derive_units("g", &s(&["pace", "fast", "slow"]), &g, None);
        assert_eq!(nodes(&units), [s(&["pace", "slow"]), s(&["fast"])]);
        assert_eq!(units[0].block_topics, s(&["t"]));
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
        let g = graph(&[("cam2", "det")], &[], &[("cam2", 1)]);
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
        let plan = expand_process_groups(&config, &UnitGraph::default(), None, &IndexSet::new());
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
        // `work`'s unit for `x` would be named `work.x`, which a declared
        // group already is; under a key folding `.` to `_`, `a-b` and `a_b`
        // collide too.
        let config = config_with_groups(&[("work", &["p", "x"]), ("work.x", &["y"])], &[]);
        let plan = expand_process_groups(&config, &UnitGraph::default(), None, &IndexSet::new());
        assert_eq!(plan.name_collisions, s(&["work"]));
        assert_eq!(plan.groups["work"], s(&["p", "x"]));
        assert_eq!(plan.groups["work.x"], s(&["y"]));

        let config = config_with_groups(&[("g", &["a-b", "a_b"])], &[]);
        let fold = |n: &str| n.replace(['-', '.'], "_");
        let plan = expand_process_groups_keyed(
            &config,
            &UnitGraph::default(),
            None,
            &IndexSet::new(),
            &fold,
        );
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
