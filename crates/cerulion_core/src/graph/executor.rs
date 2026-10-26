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
//! # The unit is a linear chain
//!
//! A node joins its producer's unit when the edge between them is a linear
//! hand-off. [`derive_units`] starts a new unit at a node for each of these
//! reasons ([`Boundary`]):
//!
//! | boundary | the node |
//! |---|---|
//! | [`Boundary::Source`] | has no trigger input fed by a node of the same process (a period, external or sync-free source, or a consumer of another process's or an outside topic) |
//! | [`Boundary::FanIn`] | has two or more trigger inputs (a join or a sync set) |
//! | [`Boundary::FanOut`] | is fed by a producer that triggers more than one node of the process |
//! | [`Boundary::Isolated`] | declares `isolate: true` |
//! | [`Boundary::AfterIsolated`] | is fed by a node that declares `isolate: true` |
//! | [`Boundary::OverBudget`] | ticks long against its period or a deadline in its chain, by the profile |
//! | [`Boundary::AfterOverBudget`] | is fed by a node over budget |
//!
//! Latest-value (non-trigger) edges never join units: the consumer is not woken
//! by them.
//!
//! # Overrides
//!
//! `isolate: true` puts the node in a unit of its own. `isolate: false` joins
//! the node to the unit of every same-process node that triggers it, whatever
//! the automatic rules say: a fan-in node merges its producers' units, a
//! fan-out consumer shares the producer's unit, and the tick budget is not
//! applied to it. When a producer declares `isolate: true` and its consumer
//! `isolate: false`, the producer's declaration wins.
//!
//! A [`Coupling`] (every producer and consumer of a topic with a `block`
//! consumer) merges its nodes into one unit after every other rule: the
//! producer's deferral reads an outstanding count kept by the consumers' drains
//! inside one level loop.
//!
//! # The tick budget
//!
//! With a profile ([`PartitionCosts`], the cost snapshot `graph profile`
//! writes), a node is over budget when its p50 tick is at least
//! [`BUDGET_DIVISOR`]⁻¹ of the period it is fed at, or of the tightest
//! `tick_within_ms` of a node downstream of it in the same process. The period
//! a node is fed at is the profiled rate of its trigger edge, else its own
//! period, else its producer's. Without a profile no node is over budget and
//! the split is structural.
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

use std::collections::BTreeSet;

use indexmap::{IndexMap, IndexSet};

use crate::graph::config::GraphConfig;
use crate::graph::node::{MacroPolicy, NodeInfo};
use crate::graph::partition::{block_colocation_seeds, PartitionCosts};
use crate::graph::topology::{GraphTopology, TriggerEdges};

/// A node is over budget when its p50 tick times this divisor reaches the
/// period it is fed at or a downstream deadline: a quarter of the period.
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
    /// Two or more trigger inputs.
    FanIn {
        /// The number of trigger inputs.
        inputs: usize,
    },
    /// Fed by a producer that triggers more than one node of the process.
    FanOut {
        /// The fanning producer.
        producer: String,
        /// How many nodes of the process it triggers.
        consumers: usize,
    },
    /// The node declares `isolate: true`.
    Isolated,
    /// Fed by a node that declares `isolate: true`.
    AfterIsolated {
        /// The isolated producer.
        producer: String,
    },
    /// The node ticks long against its budget.
    OverBudget(BudgetVerdict),
    /// Fed by a node over budget.
    AfterOverBudget {
        /// The producer over budget.
        producer: String,
    },
}

impl Boundary {
    /// One line naming the boundary, for `graph plan` and log lines.
    pub fn describe(&self) -> String {
        match self {
            Self::Source => "source (no trigger input from this process)".to_string(),
            Self::FanIn { inputs } => format!("fan-in ({inputs} trigger inputs)"),
            Self::FanOut {
                producer,
                consumers,
            } => format!("fan-out ({producer} triggers {consumers} nodes)"),
            Self::Isolated => "isolate: true".to_string(),
            Self::AfterIsolated { producer } => {
                format!("after an isolated node ({producer} declares isolate: true)")
            }
            Self::OverBudget(v) => v.describe(),
            Self::AfterOverBudget { producer } => {
                format!("after a node over its tick budget ({producer})")
            }
        }
    }
}

/// What the tick budget measured for a node over it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BudgetVerdict {
    /// The node's profiled p50 tick.
    pub p50_ns: u64,
    /// The budget it reached: the limit divided by [`BUDGET_DIVISOR`].
    pub budget_ns: u64,
    /// What the budget was derived from.
    pub against: BudgetAgainst,
    /// The profile's label (its file, or "costs").
    pub profile: String,
}

impl BudgetVerdict {
    fn describe(&self) -> String {
        let against = match &self.against {
            BudgetAgainst::Period { period_ns } => format!("its {period_ns} ns period"),
            BudgetAgainst::Deadline { node, deadline_ns } => {
                format!("the {deadline_ns} ns tick_within of {node}")
            }
        };
        format!(
            "tick budget: p50 {} ns >= {} ns, 1/{BUDGET_DIVISOR} of {against} (profile {})",
            self.p50_ns, self.budget_ns, self.profile
        )
    }
}

/// The limit a [`BudgetVerdict`] measured a node against.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BudgetAgainst {
    /// The period the node is fed at.
    Period {
        /// That period.
        period_ns: u64,
    },
    /// The tightest `tick_within_ms` of a node downstream of it.
    Deadline {
        /// The node declaring it.
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
    /// The first entry is the unit's own start.
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
    /// Each node's trigger-input count, outside producers included.
    pub trigger_inputs: IndexMap<String, usize>,
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
        let mut out = Self::default();
        for flow in topology.topics() {
            for consumer in &flow.consumers {
                if !trigger_edges.is_triggering(&consumer.node_id, &flow.topic) {
                    continue;
                }
                *out.trigger_inputs
                    .entry(consumer.node_id.clone())
                    .or_insert(0) += 1;
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

/// Whether the tick-budget rule is applied (the oracle's mutant seam ignores
/// it under the `oracle-mutants` feature).
fn budget_rule_on() -> bool {
    #[cfg(feature = "oracle-mutants")]
    {
        if std::env::var_os(BUDGET_MUTANT_ENV).is_some() {
            return false;
        }
    }
    true
}

/// Setting this variable in a build with the `oracle-mutants` feature makes
/// [`derive_units`] ignore the tick budget.
#[cfg(feature = "oracle-mutants")]
pub const BUDGET_MUTANT_ENV: &str = "CERULION_UNIT_BUDGET_MUTANT";

/// The period `node` is fed at: its trigger edge's profiled rate, else its own
/// period, else its first in-host trigger producer's, without revisiting a node.
fn fed_period_ns(
    node: &str,
    graph: &UnitGraph,
    host: &IndexSet<&str>,
    costs: Option<&PartitionCosts>,
    seen: &mut BTreeSet<String>,
) -> Option<u64> {
    if !seen.insert(node.to_string()) {
        return None;
    }
    if let Some(costs) = costs {
        let rate = graph
            .trigger_edges
            .iter()
            .filter(|(_, c)| c == node)
            .filter_map(|(p, c)| costs.edge_rate_mhz.get(&(p.clone(), c.clone())))
            .copied()
            .max();
        if let Some(mhz) = rate.filter(|r| *r > 0) {
            return Some(1_000_000_000_000 / mhz);
        }
    }
    if let Some(p) = graph.period_ns.get(node) {
        return Some(*p);
    }
    let producer = graph
        .trigger_edges
        .iter()
        .find(|(p, c)| c == node && host.contains(p.as_str()))
        .map(|(p, _)| p.clone())?;
    fed_period_ns(&producer, graph, host, costs, seen)
}

/// The tightest `tick_within` among the in-host nodes downstream of `node` by
/// trigger edges, with the node declaring it.
fn tightest_downstream_deadline(
    node: &str,
    graph: &UnitGraph,
    host: &IndexSet<&str>,
) -> Option<(String, u64)> {
    let mut stack = vec![node.to_string()];
    let mut seen: BTreeSet<String> = BTreeSet::new();
    let mut best: Option<(String, u64)> = None;
    while let Some(n) = stack.pop() {
        for (_, c) in graph.trigger_edges.iter().filter(|(p, _)| *p == n) {
            if !host.contains(c.as_str()) || !seen.insert(c.clone()) {
                continue;
            }
            if let Some(d) = graph.tick_within_ns.get(c) {
                if best.as_ref().is_none_or(|(_, b)| d < b) {
                    best = Some((c.clone(), *d));
                }
            }
            stack.push(c.clone());
        }
    }
    best
}

fn budget_verdict(
    node: &str,
    graph: &UnitGraph,
    host: &IndexSet<&str>,
    profile: Option<UnitProfile<'_>>,
) -> Option<BudgetVerdict> {
    let profile = profile?;
    let p50 = *profile.costs.node_p50_ns.get(node)?;
    let reached = |limit: u64| p50.saturating_mul(BUDGET_DIVISOR) >= limit;
    let mut seen = BTreeSet::new();
    if let Some(period_ns) = fed_period_ns(node, graph, host, Some(profile.costs), &mut seen) {
        if reached(period_ns) {
            return Some(BudgetVerdict {
                p50_ns: p50,
                budget_ns: period_ns / BUDGET_DIVISOR,
                against: BudgetAgainst::Period { period_ns },
                profile: profile.label.to_string(),
            });
        }
    }
    if let Some((by, deadline_ns)) = tightest_downstream_deadline(node, graph, host) {
        if reached(deadline_ns) {
            return Some(BudgetVerdict {
                p50_ns: p50,
                budget_ns: deadline_ns / BUDGET_DIVISOR,
                against: BudgetAgainst::Deadline {
                    node: by,
                    deadline_ns,
                },
                profile: profile.label.to_string(),
            });
        }
    }
    None
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

    // In-host trigger producers of each member, and in-host trigger consumer
    // nodes of each member.
    let mut producers: IndexMap<&str, IndexSet<&str>> = IndexMap::new();
    let mut consumers: IndexMap<&str, IndexSet<&str>> = IndexMap::new();
    for (p, c) in &graph.trigger_edges {
        if in_host.contains(p.as_str()) && in_host.contains(c.as_str()) {
            producers.entry(c.as_str()).or_default().insert(p.as_str());
            consumers.entry(p.as_str()).or_default().insert(c.as_str());
        }
    }

    let budget_on = budget_rule_on();
    let over: IndexMap<&str, BudgetVerdict> = members
        .iter()
        .filter(|n| declared(n) != Some(false) && budget_on)
        .filter_map(|n| budget_verdict(n, graph, &in_host, profile).map(|v| (n.as_str(), v)))
        .collect();

    let mut parent: Vec<usize> = (0..members.len()).collect();
    let mut starts: IndexMap<&str, Boundary> = IndexMap::new();
    for node in members {
        let n = node.as_str();
        let own = producers.get(n);
        let inputs = graph.trigger_inputs.get(n).copied().unwrap_or(0);
        if declared(n) == Some(true) {
            starts.insert(n, Boundary::Isolated);
            continue;
        }
        let Some(own) = own.filter(|p| !p.is_empty()) else {
            starts.insert(n, Boundary::Source);
            continue;
        };
        if declared(n) == Some(false) {
            // Join every in-host producer that does not itself declare
            // isolate: true.
            let joinable: Vec<&str> = own
                .iter()
                .copied()
                .filter(|p| declared(p) != Some(true))
                .collect();
            match joinable.first() {
                Some(_) => {
                    for p in joinable {
                        if let (Some(a), Some(b)) = (index(n), index(p)) {
                            union(&mut parent, a, b);
                        }
                    }
                }
                None => {
                    let producer = own.first().map(|p| p.to_string()).unwrap_or_default();
                    starts.insert(n, Boundary::AfterIsolated { producer });
                }
            }
            continue;
        }
        if let Some(v) = over.get(n) {
            starts.insert(n, Boundary::OverBudget(v.clone()));
            continue;
        }
        if inputs >= 2 || own.len() >= 2 {
            starts.insert(
                n,
                Boundary::FanIn {
                    inputs: inputs.max(own.len()),
                },
            );
            continue;
        }
        if inputs == 0 {
            starts.insert(n, Boundary::Source);
            continue;
        }
        let Some(&p) = own.first() else {
            starts.insert(n, Boundary::Source);
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
        if over.contains_key(p) {
            starts.insert(
                n,
                Boundary::AfterOverBudget {
                    producer: p.to_string(),
                },
            );
            continue;
        }
        let fan = consumers.get(p).map_or(0, IndexSet::len);
        if fan >= 2 {
            starts.insert(
                n,
                Boundary::FanOut {
                    producer: p.to_string(),
                    consumers: fan,
                },
            );
            continue;
        }
        if let (Some(a), Some(b)) = (index(n), index(p)) {
            union(&mut parent, a, b);
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
    // Re-key the topics by final roots (a later coupling may have merged them).
    let mut topics_by_root: IndexMap<usize, Vec<String>> = IndexMap::new();
    for (r, topics) in block_topics {
        let fr = root(&mut parent, r);
        topics_by_root.entry(fr).or_default().extend(topics);
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
/// graph without `process_groups:` yields an empty plan.
pub fn expand_process_groups(
    config: &GraphConfig,
    graph: &UnitGraph,
    profile: Option<UnitProfile<'_>>,
    keep_whole: &IndexSet<String>,
) -> UnitPlan {
    let mut groups = IndexMap::new();
    let mut hosts = IndexMap::new();
    let mut units_by_host = IndexMap::new();
    for (host, members) in &config.process_groups {
        let units = if keep_whole.contains(host) {
            vec![ExecutorUnit {
                name: host.clone(),
                nodes: members.clone(),
                boundaries: Vec::new(),
                block_topics: Vec::new(),
            }]
        } else {
            derive_units(host, members, graph, profile)
        };
        for unit in &units {
            groups.insert(unit.name.clone(), unit.nodes.clone());
            if units.len() > 1 {
                hosts.insert(unit.name.clone(), host.clone());
            }
        }
        units_by_host.insert(host.clone(), units);
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

    /// A hand-built graph: `edges` are trigger edges `producer -> consumer`,
    /// each consumer's trigger-input count is its in-edge count plus
    /// `outside` inputs, and `periods` are period sources.
    fn graph(edges: &[(&str, &str)], periods: &[(&str, u64)], outside: &[&str]) -> UnitGraph {
        let mut g = UnitGraph::default();
        for (p, c) in edges {
            g.trigger_edges.push((p.to_string(), c.to_string()));
            *g.trigger_inputs.entry(c.to_string()).or_insert(0) += 1;
        }
        for n in outside {
            *g.trigger_inputs.entry(n.to_string()).or_insert(0) += 1;
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

    fn names(units: &[ExecutorUnit]) -> Vec<Vec<String>> {
        units.iter().map(|u| u.nodes.clone()).collect()
    }

    fn start_of<'a>(units: &'a [ExecutorUnit], node: &str) -> Option<&'a Boundary> {
        units
            .iter()
            .flat_map(|u| u.boundaries.iter())
            .find(|(n, _)| n == node)
            .map(|(_, b)| b)
    }

    #[test]
    fn a_linear_chain_is_one_unit_and_keeps_the_host_name() {
        let g = graph(&[("cam", "det"), ("det", "track")], &[("cam", 10)], &[]);
        let units = derive_units("g", &s(&["cam", "det", "track"]), &g, None);
        assert_eq!(names(&units), [s(&["cam", "det", "track"])]);
        assert_eq!(units[0].name, "g");
        assert_eq!(units[0].boundaries, [("cam".to_string(), Boundary::Source)]);
    }

    #[test]
    fn a_fan_out_starts_a_unit_at_every_consumer() {
        let g = graph(&[("pace", "fast"), ("pace", "slow")], &[("pace", 1)], &[]);
        let units = derive_units("g", &s(&["pace", "fast", "slow"]), &g, None);
        assert_eq!(names(&units), [s(&["pace"]), s(&["fast"]), s(&["slow"])]);
        assert_eq!(
            units.iter().map(|u| u.name.as_str()).collect::<Vec<_>>(),
            ["g.pace", "g.fast", "g.slow"]
        );
        assert_eq!(
            start_of(&units, "fast"),
            Some(&Boundary::FanOut {
                producer: "pace".into(),
                consumers: 2
            })
        );
    }

    #[test]
    fn a_fan_in_starts_a_unit() {
        let g = graph(
            &[("a", "b"), ("c", "join"), ("b", "join")],
            &[("a", 10), ("c", 10)],
            &[],
        );
        let units = derive_units("g", &s(&["a", "b", "c", "join"]), &g, None);
        assert_eq!(names(&units), [s(&["a", "b"]), s(&["c"]), s(&["join"])]);
        assert_eq!(
            start_of(&units, "join"),
            Some(&Boundary::FanIn { inputs: 2 })
        );
    }

    #[test]
    fn a_consumer_of_another_process_is_a_source() {
        let g = graph(&[("x", "b"), ("b", "c")], &[], &[]);
        let units = derive_units("g", &s(&["b", "c"]), &g, None);
        assert_eq!(names(&units), [s(&["b", "c"])]);
        assert_eq!(start_of(&units, "b"), Some(&Boundary::Source));
    }

    #[test]
    fn an_outside_trigger_beside_an_in_host_one_is_a_fan_in() {
        let g = graph(&[("a", "b")], &[("a", 10)], &["b"]);
        let units = derive_units("g", &s(&["a", "b"]), &g, None);
        assert_eq!(names(&units), [s(&["a"]), s(&["b"])]);
        assert_eq!(start_of(&units, "b"), Some(&Boundary::FanIn { inputs: 2 }));
    }

    #[test]
    fn latest_value_edges_never_join_units() {
        // `b` reads `a` only as a latest-value input: no trigger edge.
        let g = graph(&[], &[("a", 10), ("b", 10)], &[]);
        let units = derive_units("g", &s(&["a", "b"]), &g, None);
        assert_eq!(names(&units), [s(&["a"]), s(&["b"])]);
    }

    #[test]
    fn isolate_true_gives_a_node_its_own_unit_and_ends_the_chain_after_it() {
        let mut g = graph(&[("a", "b"), ("b", "c")], &[("a", 10)], &[]);
        g.isolate.insert("b".into(), true);
        let units = derive_units("g", &s(&["a", "b", "c"]), &g, None);
        assert_eq!(names(&units), [s(&["a"]), s(&["b"]), s(&["c"])]);
        assert_eq!(start_of(&units, "b"), Some(&Boundary::Isolated));
        assert_eq!(
            start_of(&units, "c"),
            Some(&Boundary::AfterIsolated {
                producer: "b".into()
            })
        );
    }

    #[test]
    fn isolate_false_joins_a_fan_out_consumer_and_merges_a_fan_in() {
        let mut g = graph(&[("pace", "fast"), ("pace", "slow")], &[("pace", 1)], &[]);
        g.isolate.insert("fast".into(), false);
        let units = derive_units("g", &s(&["pace", "fast", "slow"]), &g, None);
        assert_eq!(names(&units), [s(&["pace", "fast"]), s(&["slow"])]);

        let mut j = graph(
            &[("a", "join"), ("c", "join")],
            &[("a", 10), ("c", 10)],
            &[],
        );
        j.isolate.insert("join".into(), false);
        let units = derive_units("g", &s(&["a", "c", "join"]), &j, None);
        assert_eq!(names(&units), [s(&["a", "c", "join"])]);
    }

    #[test]
    fn a_producer_isolate_true_wins_over_its_consumer_isolate_false() {
        let mut g = graph(&[("a", "b")], &[("a", 10)], &[]);
        g.isolate.insert("a".into(), true);
        g.isolate.insert("b".into(), false);
        let units = derive_units("g", &s(&["a", "b"]), &g, None);
        assert_eq!(names(&units), [s(&["a"]), s(&["b"])]);
        assert_eq!(
            start_of(&units, "b"),
            Some(&Boundary::AfterIsolated {
                producer: "a".into()
            })
        );
    }

    #[test]
    fn a_block_flow_stays_one_unit_across_a_fan_out() {
        let mut g = graph(&[("pace", "fast"), ("pace", "slow")], &[("pace", 1)], &[]);
        g.couplings.push(Coupling {
            topic: "t".into(),
            nodes: s(&["pace", "slow"]),
        });
        let units = derive_units("g", &s(&["pace", "fast", "slow"]), &g, None);
        assert_eq!(names(&units), [s(&["pace", "slow"]), s(&["fast"])]);
        assert_eq!(units[0].block_topics, s(&["t"]));
    }

    /// The budget oracle: a 1 kHz chain `pace -> filt -> slow -> sink` whose
    /// `slow` ticks 50 ms. The expected split is fixed by hand.
    fn budget_case() -> (UnitGraph, PartitionCosts) {
        let g = graph(
            &[("pace", "filt"), ("filt", "slow"), ("slow", "sink")],
            &[("pace", 1)],
            &[],
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
        (g, c)
    }

    #[test]
    fn a_node_over_its_tick_budget_is_split_out_with_the_chain_after_it() {
        let (g, c) = budget_case();
        let profile = UnitProfile {
            costs: &c,
            label: "costs.yaml",
        };
        let units = derive_units(
            "g",
            &s(&["pace", "filt", "slow", "sink"]),
            &g,
            Some(profile),
        );
        assert_eq!(
            names(&units),
            [s(&["pace", "filt"]), s(&["slow"]), s(&["sink"])]
        );
        let Some(Boundary::OverBudget(v)) = start_of(&units, "slow") else {
            panic!("slow must start a unit over budget: {units:?}");
        };
        assert_eq!(v.p50_ns, 50_000_000);
        assert_eq!(
            v.against,
            BudgetAgainst::Period {
                period_ns: 1_000_000
            }
        );
        assert_eq!(v.budget_ns, 250_000);
        assert_eq!(v.profile, "costs.yaml");
        assert_eq!(
            start_of(&units, "sink"),
            Some(&Boundary::AfterOverBudget {
                producer: "slow".into()
            })
        );
    }

    #[test]
    fn without_a_profile_the_same_chain_stays_one_unit() {
        let (g, _) = budget_case();
        let units = derive_units("g", &s(&["pace", "filt", "slow", "sink"]), &g, None);
        assert_eq!(names(&units), [s(&["pace", "filt", "slow", "sink"])]);
    }

    #[test]
    fn isolate_false_keeps_a_node_over_budget_in_its_chain() {
        let (mut g, c) = budget_case();
        g.isolate.insert("slow".into(), false);
        let profile = UnitProfile {
            costs: &c,
            label: "costs.yaml",
        };
        let units = derive_units(
            "g",
            &s(&["pace", "filt", "slow", "sink"]),
            &g,
            Some(profile),
        );
        // `slow` is not over budget once it declares isolate: false, so the
        // chain after it stays whole too.
        assert_eq!(names(&units), [s(&["pace", "filt", "slow", "sink"])]);
    }

    #[test]
    fn a_tight_downstream_deadline_splits_an_upstream_node() {
        // 10 Hz chain; `mid` ticks 3 ms, under a quarter of 100 ms, but `out`
        // declares tick_within 10 ms and 3 ms reaches a quarter of that.
        let mut g = graph(&[("src", "mid"), ("mid", "out")], &[("src", 100)], &[]);
        g.tick_within_ns.insert("out".into(), 10_000_000);
        let c = costs(&[("src", 1_000), ("mid", 3_000_000), ("out", 1_000)], &[]);
        let profile = UnitProfile {
            costs: &c,
            label: "p",
        };
        let units = derive_units("g", &s(&["src", "mid", "out"]), &g, Some(profile));
        assert_eq!(names(&units), [s(&["src"]), s(&["mid"]), s(&["out"])]);
        let Some(Boundary::OverBudget(v)) = start_of(&units, "mid") else {
            panic!("mid must be over budget: {units:?}");
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
    fn a_node_under_budget_stays_in_its_chain() {
        let g = graph(&[("src", "mid")], &[("src", 10)], &[]);
        // 2.4 ms against a 10 ms period: under a quarter.
        let c = costs(&[("src", 1_000), ("mid", 2_400_000)], &[]);
        let profile = UnitProfile {
            costs: &c,
            label: "p",
        };
        let units = derive_units("g", &s(&["src", "mid"]), &g, Some(profile));
        assert_eq!(names(&units), [s(&["src", "mid"])]);
        // 2.5 ms reaches exactly a quarter: split.
        let c = costs(&[("src", 1_000), ("mid", 2_500_000)], &[]);
        let profile = UnitProfile {
            costs: &c,
            label: "p",
        };
        let units = derive_units("g", &s(&["src", "mid"]), &g, Some(profile));
        assert_eq!(names(&units), [s(&["src"]), s(&["mid"])]);
    }

    #[test]
    fn units_keep_member_order() {
        let g = graph(&[("c", "a")], &[("c", 10)], &[]);
        let units = derive_units("g", &s(&["a", "b", "c"]), &g, None);
        assert_eq!(names(&units), [s(&["a", "c"]), s(&["b"])]);
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
        let config = config_with_groups(
            &[("cam", &["cam"]), ("work", &["pace", "fast", "slow"])],
            &[],
        );
        let g = graph(&[("pace", "fast"), ("pace", "slow")], &[("pace", 1)], &[]);
        let plan = expand_process_groups(&config, &g, None, &IndexSet::new());
        let keys: Vec<&str> = plan.groups.keys().map(String::as_str).collect();
        assert_eq!(keys, ["cam", "work.pace", "work.fast", "work.slow"]);
        assert_eq!(
            plan.hosts.get("work.fast").map(String::as_str),
            Some("work")
        );
        assert!(!plan.hosts.contains_key("cam"));
        assert!(plan.splits_any());
        let applied = plan.apply(&config);
        assert_eq!(applied.process_groups, plan.groups);
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
    fn a_process_per_node_graph_plans_unchanged() {
        let config = config_with_groups(&[("grp_a", &["a"]), ("grp_b", &["b"])], &[]);
        let g = graph(&[("a", "b")], &[("a", 10)], &[]);
        let plan = expand_process_groups(&config, &g, None, &IndexSet::new());
        assert_eq!(plan.groups, config.process_groups);
        assert!(plan.hosts.is_empty());
    }
}
