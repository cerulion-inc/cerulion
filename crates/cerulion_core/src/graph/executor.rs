// SPDX-License-Identifier: AGPL-3.0-only
//! Executor units: which nodes of one process share a thread.
//!
//! A free-run deployment runs every node on its own executor. An executor is
//! one thread driving one [`GraphRuntime::run_live`](crate::GraphRuntime::run_live)
//! over the nodes of its unit, with its own step counter, its own gating clock
//! and its own trace ring, exactly as one worker process does. Two units that
//! share a process share nothing in the live loop: each one waits on its own
//! inputs, its own timer, its own watchdog deadlines, its own external sources
//! and its own credit words, so a slow tick delays only its own unit and, by
//! data, the units that read from it.
//!
//! This module decides the units. It is pure: a process's members and the
//! couplings that must stay on one thread go in, the units come out, and
//! [`expand_process_groups`] rewrites a graph's `process_groups:` so that the
//! deployment planner gives every unit its own worker plan and every edge
//! between units the same wiring an edge between processes gets.
//!
//! # What keeps nodes on one unit
//!
//! A unit holds more than one node only for a [`Coupling`]:
//!
//! - [`CouplingKind::BlockFlow`]: every producer and every consumer of a topic
//!   with a `block` consumer. The producer's deferral reads a process-local
//!   outstanding count kept by the consumers' drains, so the flow runs on one
//!   level loop.
//! - [`CouplingKind::FusedChain`]: a chain from the chain census, and only when
//!   the run's fusion decision is on. A fused chain is a same-thread hand-off
//!   by definition: the head fires and the next node runs on the same thread
//!   with the head's frame. Splitting it would put a cross-thread wake back on
//!   every hop.
//!
//! Inside a multi-node unit the level loop runs as it does today, so those
//! nodes still wait on each other.
//!
//! # Names
//!
//! A host whose members form one unit keeps its own name, so a deployment with
//! nothing to split plans byte-identically to one planned before units
//! existed. A host that splits names each unit `<host>.<first node>`.

use indexmap::{IndexMap, IndexSet};

use crate::graph::chain::ChainCensus;
use crate::graph::config::GraphConfig;
use crate::graph::partition::block_colocation_seeds;
use crate::graph::topology::GraphTopology;

/// Why a set of nodes must share one executor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CouplingKind {
    /// The producers and consumers of a topic with a `block` consumer.
    BlockFlow,
    /// A fused chain, present only when the run's fusion decision is on.
    FusedChain,
}

impl CouplingKind {
    /// The label used in log lines.
    pub fn label(self) -> &'static str {
        match self {
            Self::BlockFlow => "block_flow",
            Self::FusedChain => "fused_chain",
        }
    }
}

/// A set of nodes that must run on one executor, and why.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Coupling {
    /// Why the nodes share an executor.
    pub kind: CouplingKind,
    /// The coupled node ids, in graph order. Nodes outside a host are ignored
    /// when that host's units are derived.
    pub nodes: Vec<String>,
}

/// One executor: the nodes that run on one thread.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExecutorUnit {
    /// The unit's name, also its process-group name in the expanded plan.
    pub name: String,
    /// The member node ids, in the host's member order.
    pub nodes: Vec<String>,
    /// The couplings that merged nodes into this unit (empty for one node).
    pub kinds: Vec<CouplingKind>,
}

/// The couplings a graph's declarations impose.
///
/// `census` is the chain census of the run, passed only when the run's fusion
/// decision is on; `None` adds no chain couplings.
pub fn executor_couplings(
    topology: &GraphTopology,
    census: Option<&ChainCensus>,
) -> Vec<Coupling> {
    let mut out = Vec::new();
    for seed in block_colocation_seeds(topology) {
        let mut nodes: IndexSet<String> = IndexSet::new();
        nodes.extend(seed.producers.iter().cloned());
        nodes.extend(seed.block_consumers.iter().map(|(n, _)| n.clone()));
        nodes.extend(seed.other_consumers.iter().map(|(n, _)| n.clone()));
        out.push(Coupling {
            kind: CouplingKind::BlockFlow,
            nodes: nodes.into_iter().collect(),
        });
    }
    if let Some(census) = census {
        for chain in census.chains() {
            out.push(Coupling {
                kind: CouplingKind::FusedChain,
                nodes: chain.nodes.clone(),
            });
        }
    }
    out
}

/// Split one host's members into executor units.
///
/// Every member starts as its own unit; each coupling merges the members it
/// names. Units are ordered by their first member, and each unit lists its
/// members in `members` order, so the result depends only on the inputs.
pub fn derive_units(host: &str, members: &[String], couplings: &[Coupling]) -> Vec<ExecutorUnit> {
    let index: IndexMap<&str, usize> = members
        .iter()
        .enumerate()
        .map(|(i, m)| (m.as_str(), i))
        .collect();
    let mut parent: Vec<usize> = (0..members.len()).collect();
    let mut kinds: Vec<IndexSet<CouplingKind>> = vec![IndexSet::new(); members.len()];

    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }

    for coupling in couplings {
        let local: Vec<usize> = coupling
            .nodes
            .iter()
            .filter_map(|n| index.get(n.as_str()).copied())
            .collect();
        let Some((&first, rest)) = local.split_first() else {
            continue;
        };
        for &other in rest {
            let a = root(&mut parent, first);
            let b = root(&mut parent, other);
            if a != b {
                let (keep, drop) = if a < b { (a, b) } else { (b, a) };
                parent[drop] = keep;
                let moved = std::mem::take(&mut kinds[drop]);
                kinds[keep].extend(moved);
            }
        }
        if !rest.is_empty() {
            let r = root(&mut parent, first);
            kinds[r].insert(coupling.kind);
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
            nodes,
            kinds: kinds[r].iter().copied().collect(),
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
}

/// Split every declared group of `config` into executor units.
///
/// `keep_whole` names declared groups that stay one unit whatever their
/// couplings say (a caller uses it for a group whose split partition it
/// refused). A graph without `process_groups:` yields an empty plan.
pub fn expand_process_groups(
    config: &GraphConfig,
    couplings: &[Coupling],
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
                kinds: Vec::new(),
            }]
        } else {
            derive_units(host, members, couplings)
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

    fn s(v: &[&str]) -> Vec<String> {
        v.iter().map(|x| x.to_string()).collect()
    }

    fn block(nodes: &[&str]) -> Coupling {
        Coupling {
            kind: CouplingKind::BlockFlow,
            nodes: s(nodes),
        }
    }

    fn chain(nodes: &[&str]) -> Coupling {
        Coupling {
            kind: CouplingKind::FusedChain,
            nodes: s(nodes),
        }
    }

    #[test]
    fn every_member_is_its_own_unit_without_couplings() {
        let units = derive_units("g", &s(&["pace", "fast", "slow"]), &[]);
        let names: Vec<&str> = units.iter().map(|u| u.name.as_str()).collect();
        assert_eq!(names, ["g.pace", "g.fast", "g.slow"]);
        assert!(units.iter().all(|u| u.nodes.len() == 1 && u.kinds.is_empty()));
    }

    #[test]
    fn a_single_member_host_keeps_its_name() {
        let units = derive_units("grp_a", &s(&["a"]), &[]);
        assert_eq!(units.len(), 1);
        assert_eq!(units[0].name, "grp_a");
        assert_eq!(units[0].nodes, s(&["a"]));
    }

    #[test]
    fn a_block_flow_stays_one_unit_and_its_siblings_split() {
        let units = derive_units(
            "g",
            &s(&["pace", "fast", "slow"]),
            &[block(&["pace", "slow"])],
        );
        assert_eq!(units.len(), 2);
        assert_eq!(units[0].name, "g.pace");
        assert_eq!(units[0].nodes, s(&["pace", "slow"]));
        assert_eq!(units[0].kinds, vec![CouplingKind::BlockFlow]);
        assert_eq!(units[1].nodes, s(&["fast"]));
    }

    #[test]
    fn couplings_that_share_a_node_merge_transitively() {
        let units = derive_units(
            "g",
            &s(&["a", "b", "c", "d"]),
            &[block(&["a", "b"]), chain(&["b", "c"])],
        );
        assert_eq!(units.len(), 2);
        assert_eq!(units[0].nodes, s(&["a", "b", "c"]));
        assert_eq!(
            units[0].kinds,
            vec![CouplingKind::BlockFlow, CouplingKind::FusedChain]
        );
        assert_eq!(units[1].nodes, s(&["d"]));
    }

    #[test]
    fn a_coupling_reaching_outside_the_host_merges_only_local_members() {
        let units = derive_units("g", &s(&["a", "b"]), &[block(&["a", "x", "b"])]);
        assert_eq!(units.len(), 1);
        assert_eq!(units[0].name, "g");
        assert_eq!(units[0].nodes, s(&["a", "b"]));
        let alone = derive_units("h", &s(&["c", "d"]), &[block(&["a", "c"])]);
        assert_eq!(alone.len(), 2);
        assert!(alone.iter().all(|u| u.kinds.is_empty()));
    }

    #[test]
    fn units_keep_member_order_whatever_the_coupling_order() {
        let units = derive_units("g", &s(&["a", "b", "c"]), &[chain(&["c", "a"])]);
        assert_eq!(units[0].nodes, s(&["a", "c"]));
        assert_eq!(units[1].nodes, s(&["b"]));
    }

    fn graph_with_groups(groups: &[(&str, &[&str])], order: &[&str]) -> GraphConfig {
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
        let config = graph_with_groups(&[("cam", &["cam"]), ("work", &["pace", "fast"])], &[]);
        let plan = expand_process_groups(&config, &[], &IndexSet::new());
        let names: Vec<&str> = plan.groups.keys().map(String::as_str).collect();
        assert_eq!(names, ["cam", "work.pace", "work.fast"]);
        assert_eq!(plan.hosts.get("work.pace").map(String::as_str), Some("work"));
        assert_eq!(plan.hosts.get("work.fast").map(String::as_str), Some("work"));
        assert!(!plan.hosts.contains_key("cam"));
        assert!(plan.splits_any());
        assert!(plan.order.is_empty());
    }

    #[test]
    fn expansion_rewrites_the_declared_order() {
        let config = graph_with_groups(&[("a", &["x", "y"]), ("b", &["z"])], &["b", "a"]);
        let plan = expand_process_groups(&config, &[], &IndexSet::new());
        assert_eq!(plan.order, s(&["b", "a.x", "a.y"]));
    }

    #[test]
    fn a_kept_whole_group_is_one_unit_under_its_own_name() {
        let config = graph_with_groups(&[("work", &["pace", "fast"])], &[]);
        let keep: IndexSet<String> = ["work".to_string()].into_iter().collect();
        let plan = expand_process_groups(&config, &[], &keep);
        assert_eq!(plan.groups.len(), 1);
        assert_eq!(plan.groups["work"], s(&["pace", "fast"]));
        assert!(plan.hosts.is_empty());
        assert!(!plan.splits_any());
    }

    #[test]
    fn a_graph_with_nothing_to_split_plans_unchanged() {
        let config = graph_with_groups(&[("grp_a", &["a"]), ("grp_b", &["b"])], &[]);
        let plan = expand_process_groups(&config, &[], &IndexSet::new());
        assert_eq!(plan.groups, config.process_groups);
        assert!(plan.hosts.is_empty());
    }
}
