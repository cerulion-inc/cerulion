// SPDX-License-Identifier: AGPL-3.0-only
//! The pure cost-aware auto-partitioner.
//!
//! Process-per-node BASELINE + validated GREEDY FUSION. Every test ties the
//! grouping to a HAND-COMPUTED oracle (never a self-compare); the generative
//! (property) tests assert that EVERY output is spawner-consumable — it passes
//! the same [`validate_partition`] the partitioner uses internally AND the
//! literal `install_barrier_participant` precondition (each group's participant
//! map's non-`None` entries == `0..local_subgraph_level_count`), where the
//! local level count is computed by the REAL [`GraphTopology::derive_levels`]
//! on the group's induced sub-topology.
//!
//! Pure — NO iceoryx2 — so parallel-safe (no `#[serial]`).

use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::partition::{
    auto_partition, baseline_process_per_node, derive_default_budget_ns, derive_fire_targets,
    derive_fire_targets_from_observations, derive_process_groups, harvest_costs,
    validate_partition, AutoPartition, FireTargetPolicy, FusionRejection, FusionRejectionReason,
    HopCosts, PartitionCosts, ProfileResult, WarmupObservation,
};
use cerulion_core::graph::topology::{GraphTopology, Levels, TriggerEdges};
use cerulion_core::graph::NodeInfo;
use cerulion_core::scheduler::TraceEntry;
use indexmap::IndexMap;
use std::collections::{BTreeMap, BTreeSet};
use std::num::NonZeroUsize;
use std::sync::Arc;
use tracing_test::traced_test;

/// The empty isolated set. Every partitioner call site passes this
/// (no online-profiler isolation), exercising the additive migration: an empty
/// `isolated` reproduces the pre-profiler behavior exactly.
fn iso_none() -> BTreeSet<String> {
    BTreeSet::new()
}

/// Per-second latency reclaimed per fused crossing = `cross_ns − intra_ns`; the
/// coupling of an edge is `rate_mhz × SAVING`. Hand-oracle helper for the
/// non-uniform-coupling / rejection-record tests below.
const SAVING: u128 = (CROSS_NS - INTRA_NS) as u128;

const PREFIX: &str = "p";

// Measured-ish hop constants: cross > intra, so fusion IS profitable.
const INTRA_NS: u64 = 1_000;
const CROSS_NS: u64 = 6_800;

// --------------------------------------------------------------------------
// Fixture builders (mirror partition_test.rs patterns).
// --------------------------------------------------------------------------

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

/// Empty macro metadata for every node (topology reads YAML inputs, not meta).
fn infos(config: &GraphConfig) -> IndexMap<String, NodeInfo> {
    config
        .nodes
        .iter()
        .map(|n| (n.id.clone(), NodeInfo::with_meta(Vec::new(), Vec::new())))
        .collect()
}

fn config_of(nodes: Vec<NodeDef>) -> GraphConfig {
    GraphConfig {
        level_assignments: None,
        network: None,
        name: None,
        identity: "g".to_string(),
        prefix: PREFIX.to_string(),
        nodes,
        multi_publisher_topics: Vec::new(),
        process_groups: IndexMap::new(),
        process_group_order: Vec::new(),
    }
}

/// Trigger edges from a list of `(consumer, "prod_node/out")` YAML sources —
/// resolves each source to its absolute topic and marks it triggering.
fn triggers(edges: &[(&str, &str)]) -> TriggerEdges {
    let mut t = TriggerEdges::new();
    for (consumer, source) in edges {
        // "nj/out" -> "/p/nj/out"
        t.insert(*consumer, format!("/{PREFIX}/{source}"));
    }
    t
}

/// Uniform costs: every node `node_ns`, every listed edge `rate_mhz`.
fn costs(node_ids: &[&str], edges: &[(&str, &str)], node_ns: u64, rate_mhz: u64) -> PartitionCosts {
    let node_p50_ns: BTreeMap<String, u64> = node_ids
        .iter()
        .map(|id| (id.to_string(), node_ns))
        .collect();
    let edge_rate_mhz: BTreeMap<(String, String), u64> = edges
        .iter()
        .map(|(p, c)| ((p.to_string(), c.to_string()), rate_mhz))
        .collect();
    PartitionCosts {
        node_p50_ns,
        edge_rate_mhz,
        hop: HopCosts {
            intra_ns: INTRA_NS,
            cross_ns: CROSS_NS,
        },
    }
}

/// Costs with per-EDGE rates (NON-uniform) — every node `node_ns`, each listed
/// `(producer, consumer, rate_mhz)` carries its own rate; unlisted edges default
/// to rate 0. Drives the descending-coupling-order + rejection-record oracles.
fn costs_edges(node_ids: &[&str], node_ns: u64, edges: &[(&str, &str, u64)]) -> PartitionCosts {
    PartitionCosts {
        node_p50_ns: node_ids
            .iter()
            .map(|id| (id.to_string(), node_ns))
            .collect(),
        edge_rate_mhz: edges
            .iter()
            .map(|(p, c, r)| ((p.to_string(), c.to_string()), *r))
            .collect(),
        hop: HopCosts {
            intra_ns: INTRA_NS,
            cross_ns: CROSS_NS,
        },
    }
}

/// Reduce an `AutoPartition` to `Vec<(name, Vec<member>)>` for oracle asserts.
fn shape(p: &AutoPartition) -> Vec<(String, Vec<String>)> {
    p.groups
        .iter()
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect()
}

fn expect(pairs: &[(&str, &[&str])]) -> Vec<(String, Vec<String>)> {
    pairs
        .iter()
        .map(|(n, m)| (n.to_string(), m.iter().map(|s| s.to_string()).collect()))
        .collect()
}

/// The REAL per-group induced-subgraph level count — reuses
/// `GraphTopology::derive_levels` (NOT a reimplementation). This is the
/// `local_count` that `install_barrier_participant` compares the participant
/// map against.
fn local_level_count(
    members: &[String],
    config: &GraphConfig,
    trigger_edges: &TriggerEdges,
) -> usize {
    let member_set: std::collections::HashSet<&str> = members.iter().map(|s| s.as_str()).collect();
    let sub_nodes: Vec<NodeDef> = config
        .nodes
        .iter()
        .filter(|nd| member_set.contains(nd.id.as_str()))
        .cloned()
        .collect();
    let sub = config_of(sub_nodes);
    let topo = GraphTopology::build(&sub, &infos(&sub)).expect("sub topology");
    topo.derive_levels(trigger_edges)
        .expect("sub levelization")
        .len()
}

fn global_levels(config: &GraphConfig, trigger_edges: &TriggerEdges) -> Levels {
    GraphTopology::build(config, &infos(config))
        .expect("topology")
        .derive_levels(trigger_edges)
        .expect("levels")
}

// --------------------------------------------------------------------------
// A linear chain n0 -> n1 -> ... -> n{k-1}.
// --------------------------------------------------------------------------

fn chain(
    k: usize,
) -> (
    GraphConfig,
    TriggerEdges,
    Vec<String>,
    Vec<(String, String)>,
) {
    let mut nodes = Vec::new();
    let mut edges = Vec::new();
    for i in 0..k {
        let id = format!("n{i}");
        if i == 0 {
            nodes.push(node(&id, &[], &["out"]));
        } else {
            let src = format!("n{}/out", i - 1);
            // leak-free owned strings via a small local vec
            let ndef = NodeDef {
                ros2: None,
                id: id.clone(),
                node_type: id.clone(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: src,
                }],
                outputs: if i + 1 < k {
                    vec![OutputDef {
                        name: "out".to_string(),
                        schema: "Vector3".to_string(),
                        max_slice_len: None,
                        history_size: 0,
                        topic: None,
                    }]
                } else {
                    Vec::new()
                },
            };
            nodes.push(ndef);
            edges.push((format!("n{}", i - 1), format!("n{i}")));
        }
    }
    let config = config_of(nodes);
    let mut t = TriggerEdges::new();
    for i in 1..k {
        t.insert(format!("n{i}"), format!("/{PREFIX}/n{}/out", i - 1));
    }
    let ids: Vec<String> = (0..k).map(|i| format!("n{i}")).collect();
    (config, t, ids, edges)
}

fn chain_costs(
    edges: &[(String, String)],
    ids: &[String],
    node_ns: u64,
    rate_mhz: u64,
) -> PartitionCosts {
    PartitionCosts {
        node_p50_ns: ids.iter().map(|id| (id.clone(), node_ns)).collect(),
        edge_rate_mhz: edges
            .iter()
            .map(|(p, c)| ((p.clone(), c.clone()), rate_mhz))
            .collect(),
        hop: HopCosts {
            intra_ns: INTRA_NS,
            cross_ns: CROSS_NS,
        },
    }
}

// ==========================================================================
// ORACLE VECTORS
// ==========================================================================

#[test]
fn chain_fuses_whole_when_budget_permits() {
    // 5-node chain, uniform coupling, budget covers all 5 nodes.
    let (config, t, ids, edges) = chain(5);
    let costs = chain_costs(&edges, &ids, 100, 10_000);
    let part = auto_partition(&config, &infos(&config), &t, &costs, &iso_none(), 100_000)
        .expect("partition");
    assert_eq!(
        shape(&part),
        expect(&[("grp_n0", &["n0", "n1", "n2", "n3", "n4"])]),
        "the whole chain fuses into one group named after the lead (n0)"
    );
    assert_eq!(part.fusions.len(), 4, "4 edges fused");
    // All ties broken in graph order (n0-n1, n1-n2, n2-n3, n3-n4).
    let order: Vec<(String, String)> = part
        .fusions
        .iter()
        .map(|f| (f.producer.clone(), f.consumer.clone()))
        .collect();
    assert_eq!(
        order,
        vec![
            ("n0".into(), "n1".into()),
            ("n1".into(), "n2".into()),
            ("n2".into(), "n3".into()),
            ("n3".into(), "n4".into()),
        ],
        "equal-coupling ties fuse in graph order"
    );
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

#[test]
fn chain_budget_forces_pairs() {
    // cost 100/node, budget 250: pairs fit (200), triples don't (300).
    // Greedy graph-order: fuse (n0,n1); (n1,n2) would make {n0,n1,n2}=300>250 skip;
    // fuse (n2,n3); (n3,n4) would make {n2,n3,n4}=300>250 skip. => {n0,n1}{n2,n3}{n4}.
    let (config, t, ids, edges) = chain(5);
    let costs = chain_costs(&edges, &ids, 100, 10_000);
    let part =
        auto_partition(&config, &infos(&config), &t, &costs, &iso_none(), 250).expect("partition");
    assert_eq!(
        shape(&part),
        expect(&[
            ("grp_n0", &["n0", "n1"]),
            ("grp_n2", &["n2", "n3"]),
            ("grp_n4", &["n4"]),
        ]),
        "budget 250 fuses adjacent pairs, leaving the tail solo"
    );
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

#[test]
fn diamond_fuses_all_when_budget_permits() {
    // n0 -> {n1, n2} -> n3 (3 global levels, a WIDE middle level).
    let config = config_of(vec![
        node("n0", &[], &["out"]),
        node("n1", &[("inp", "n0/out")], &["out"]),
        node("n2", &[("inp", "n0/out")], &["out"]),
        node("n3", &[("a", "n1/out"), ("b", "n2/out")], &[]),
    ]);
    let t = triggers(&[
        ("n1", "n0/out"),
        ("n2", "n0/out"),
        ("n3", "n1/out"),
        ("n3", "n2/out"),
    ]);
    let cst = costs(
        &["n0", "n1", "n2", "n3"],
        &[("n0", "n1"), ("n0", "n2"), ("n1", "n3"), ("n2", "n3")],
        100,
        10_000,
    );
    let part = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 100_000)
        .expect("partition");
    assert_eq!(
        shape(&part),
        expect(&[("grp_n0", &["n0", "n1", "n2", "n3"])]),
        "the whole diamond (incl. the wide level) fuses into one group"
    );
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

#[test]
fn fan_out_budget_forces_split() {
    // n0 -> {n1, n2, n3}. cost 100, budget 250 => only (n0,n1) fits.
    let config = config_of(vec![
        node("n0", &[], &["out"]),
        node("n1", &[("inp", "n0/out")], &[]),
        node("n2", &[("inp", "n0/out")], &[]),
        node("n3", &[("inp", "n0/out")], &[]),
    ]);
    let t = triggers(&[("n1", "n0/out"), ("n2", "n0/out"), ("n3", "n0/out")]);
    let cst = costs(
        &["n0", "n1", "n2", "n3"],
        &[("n0", "n1"), ("n0", "n2"), ("n0", "n3")],
        100,
        10_000,
    );
    let part =
        auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 250).expect("partition");
    assert_eq!(
        shape(&part),
        expect(&[
            ("grp_n0", &["n0", "n1"]),
            ("grp_n2", &["n2"]),
            ("grp_n3", &["n3"]),
        ]),
        "budget 250 fuses only n0+n1; the other consumers stay solo"
    );
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

#[test]
fn two_independent_chains_stay_separate() {
    // a0->a1 and b0->b1, no cross edges. Each chain fuses; no cross-group fusion.
    let config = config_of(vec![
        node("a0", &[], &["out"]),
        node("a1", &[("inp", "a0/out")], &[]),
        node("b0", &[], &["out"]),
        node("b1", &[("inp", "b0/out")], &[]),
    ]);
    let t = triggers(&[("a1", "a0/out"), ("b1", "b0/out")]);
    let cst = costs(
        &["a0", "a1", "b0", "b1"],
        &[("a0", "a1"), ("b0", "b1")],
        100,
        10_000,
    );
    let part = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 100_000)
        .expect("partition");
    assert_eq!(
        shape(&part),
        expect(&[("grp_a0", &["a0", "a1"]), ("grp_b0", &["b0", "b1"])]),
        "each independent chain becomes its own fused group"
    );
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

#[test]
fn output_feeds_derive_process_groups_matching_oracle() {
    // The partitioner output is consumable by derive_process_groups.
    // 5-chain, budget forces {n0,n1}{n2,n3}{n4}; the participant maps must match
    // the ascending-owned-global oracle.
    let (config, t, ids, edges) = chain(5);
    let cst = chain_costs(&edges, &ids, 100, 10_000);
    let part =
        auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 250).expect("partition");

    let mut cfg2 = config.clone();
    cfg2.process_groups = part.groups.clone();
    let levels = global_levels(&config, &t);
    let derived = derive_process_groups(&cfg2, &levels).expect("derive");
    // grp_n0 owns globals 0,1; grp_n2 owns 2,3; grp_n4 owns 4.
    assert_eq!(
        derived[0].global_level_map,
        vec![Some(0), Some(1), None, None, None]
    );
    assert_eq!(
        derived[1].global_level_map,
        vec![None, None, Some(0), Some(1), None]
    );
    assert_eq!(
        derived[2].global_level_map,
        vec![None, None, None, None, Some(0)]
    );
}

// ==========================================================================
// REJECTION ARMS
// ==========================================================================

#[test]
fn bridge_split_rejected_naming_bridge() {
    // 4-node chain n0->n1->n2->n3. Hand-built partition {X:[n0,n2,n3], Y:[n1]}:
    // n1 (global level 1) is FOREIGN, so X owns non-contiguous globals {0,2,3}.
    // X's subgraph re-levelizes n2 to a local ROOT (its producer n1 is foreign)
    // => local count 2 != 3 owned. validate_partition must reject NAMING n1.
    let (config, t, _ids, _edges) = chain(4);
    let bad: IndexMap<String, Vec<String>> = [
        ("X".to_string(), vec!["n0".into(), "n2".into(), "n3".into()]),
        ("Y".to_string(), vec!["n1".into()]),
    ]
    .into_iter()
    .collect();
    let err = validate_partition(&bad, &config, &infos(&config), &t)
        .expect_err("a foreign-bridged group must be rejected")
        .to_string();
    assert!(
        err.contains("n1") && err.contains("bridge"),
        "rejection must NAME the bridge node n1; got: {err}"
    );
    assert!(
        err.contains("n2"),
        "rejection should name the collapsed member n2; got: {err}"
    );
}

#[test]
fn valid_contiguous_split_accepted() {
    // Anti-tautology control for the bridge test: {X:[n0,n1], Y:[n2,n3]} on the
    // same 4-chain IS a contiguous split and must be accepted.
    let (config, t, _ids, _edges) = chain(4);
    let good: IndexMap<String, Vec<String>> = [
        ("X".to_string(), vec!["n0".into(), "n1".into()]),
        ("Y".to_string(), vec!["n2".into(), "n3".into()]),
    ]
    .into_iter()
    .collect();
    validate_partition(&good, &config, &infos(&config), &t)
        .expect("a contiguous pipeline split must validate");
}

#[test]
fn diamond_valid_wide_and_middle_split_accepted() {
    // {n0},{n1,n2},{n3} over the diamond: the middle group owns the WIDE global
    // level 1 (n1 AND n2) and re-levelizes to a single local level. n3's inputs
    // both cross the boundary but n3 is its own group's root — valid.
    let config = config_of(vec![
        node("n0", &[], &["out"]),
        node("n1", &[("inp", "n0/out")], &["out"]),
        node("n2", &[("inp", "n0/out")], &["out"]),
        node("n3", &[("a", "n1/out"), ("b", "n2/out")], &[]),
    ]);
    let t = triggers(&[
        ("n1", "n0/out"),
        ("n2", "n0/out"),
        ("n3", "n1/out"),
        ("n3", "n2/out"),
    ]);
    let good: IndexMap<String, Vec<String>> = [
        ("A".to_string(), vec!["n0".into()]),
        ("B".to_string(), vec!["n1".into(), "n2".into()]),
        ("C".to_string(), vec!["n3".into()]),
    ]
    .into_iter()
    .collect();
    validate_partition(&good, &config, &infos(&config), &t).expect("valid diamond split");
}

#[test]
fn budget_below_any_pair_leaves_process_per_node() {
    // cost 100/node, budget 150 < 200: no pair ever fits => process-per-node.
    let (config, t, ids, edges) = chain(4);
    let cst = chain_costs(&edges, &ids, 100, 10_000);
    let part =
        auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 150).expect("partition");
    assert_eq!(part.groups.len(), 4, "no fusion under a sub-pair budget");
    assert!(part.fusions.is_empty(), "no fusions recorded");
    assert_eq!(
        shape(&part),
        expect(&[
            ("grp_n0", &["n0"]),
            ("grp_n1", &["n1"]),
            ("grp_n2", &["n2"]),
            ("grp_n3", &["n3"]),
        ])
    );
}

#[test]
fn missing_node_cost_is_loud_error() {
    let (config, t, ids, edges) = chain(3);
    let mut cst = chain_costs(&edges, &ids, 100, 10_000);
    cst.node_p50_ns.remove("n1"); // drop one node's cost
    let err = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 100_000)
        .expect_err("a node with no cost must be a loud error")
        .to_string();
    assert!(
        err.contains("n1") && err.contains("no p50 duration"),
        "error must name the cost-less node; got: {err}"
    );
}

// ==========================================================================
// EDGE CASES
// ==========================================================================

#[test]
fn single_node_is_its_own_group() {
    let config = config_of(vec![node("solo", &[], &[])]);
    let t = TriggerEdges::new();
    let cst = costs(&["solo"], &[], 100, 10_000);
    let part = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 100_000)
        .expect("partition");
    assert_eq!(shape(&part), expect(&[("grp_solo", &["solo"])]));
    assert!(part.fusions.is_empty());
}

#[test]
fn empty_graph_yields_empty_partition() {
    let config = config_of(Vec::new());
    let t = TriggerEdges::new();
    let cst = PartitionCosts {
        node_p50_ns: BTreeMap::new(),
        edge_rate_mhz: BTreeMap::new(),
        hop: HopCosts {
            intra_ns: INTRA_NS,
            cross_ns: CROSS_NS,
        },
    };
    let part = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 100_000)
        .expect("partition");
    assert!(part.groups.is_empty(), "no nodes => no groups");
    assert!(part.fusions.is_empty());
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("empty is valid");
}

#[test]
fn zero_rate_edge_is_not_fused() {
    // n0->n1->n2, rate(n0,n1)=high, rate(n1,n2)=0. Only n0+n1 fuse; n2 stays solo.
    let (config, t, ids, _edges) = chain(3);
    let mut cst = chain_costs(&[("n0".into(), "n1".into())], &ids, 100, 10_000);
    // ensure (n1,n2) has NO rate entry => defaults to 0 => coupling 0 => no fuse.
    cst.edge_rate_mhz
        .remove(&("n1".to_string(), "n2".to_string()));
    let part = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 100_000)
        .expect("partition");
    assert_eq!(
        shape(&part),
        expect(&[("grp_n0", &["n0", "n1"]), ("grp_n2", &["n2"])]),
        "the zero-rate edge is not fused"
    );
}

#[test]
fn cross_le_intra_leaves_process_per_node() {
    // cross <= intra => saving 0 => every coupling 0 => no fusion, even with
    // huge rates and infinite budget.
    let (config, t, ids, edges) = chain(5);
    let mut cst = chain_costs(&edges, &ids, 1, 1_000_000);
    cst.hop = HopCosts {
        intra_ns: 5_000,
        cross_ns: 5_000, // equal => no profit
    };
    let part = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), u64::MAX)
        .expect("partition");
    assert_eq!(
        part.groups.len(),
        5,
        "no profitable fusion => process-per-node"
    );
    assert!(part.fusions.is_empty());
}

#[test]
fn zero_rates_everywhere_leaves_process_per_node() {
    let (config, t, ids, edges) = chain(4);
    let cst = chain_costs(&edges, &ids, 100, 0); // all rates 0
    let part = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), u64::MAX)
        .expect("partition");
    assert_eq!(part.groups.len(), 4, "all-zero rates => no fusion");
    assert!(part.fusions.is_empty());
}

#[test]
fn sanitized_group_name_from_lead() {
    // A node id with non-ident chars => sanitized in the group name.
    let config = config_of(vec![node("cam.left/raw", &[], &[])]);
    let t = TriggerEdges::new();
    let cst = costs(&["cam.left/raw"], &[], 100, 10_000);
    let part = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 100_000)
        .expect("partition");
    let names: Vec<&str> = part.groups.keys().map(|s| s.as_str()).collect();
    assert_eq!(names, vec!["grp_cam_left_raw"], "non-ident chars => '_'");
}

// ==========================================================================
// PROPERTY / GENERATIVE TESTS
// ==========================================================================

/// Deterministic PCG-style LCG (no external rand dep; reproducible).
struct Rng(u64);
impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.0 >> 17
    }
    fn below(&mut self, n: u64) -> u64 {
        if n == 0 {
            0
        } else {
            self.next_u64() % n
        }
    }
}

/// Generate a random acyclic layered DAG. Node `i` may only consume outputs of
/// EARLIER nodes (strictly smaller index) — guarantees no cycle. Returns
/// (config, trigger_edges, node_ids, edges as (producer, consumer)).
fn gen_dag(
    rng: &mut Rng,
) -> (
    GraphConfig,
    TriggerEdges,
    Vec<String>,
    Vec<(String, String)>,
) {
    let n = 2 + rng.below(9) as usize; // 2..=10 nodes
    let ids: Vec<String> = (0..n).map(|i| format!("v{i}")).collect();
    // For each node, choose a set of earlier producers.
    let mut producers_of: Vec<Vec<usize>> = vec![Vec::new(); n];
    for (i, producers) in producers_of.iter_mut().enumerate().skip(1) {
        let max_preds = std::cmp::min(i, 3);
        let k = rng.below(max_preds as u64 + 1) as usize; // 0..=max_preds
        let mut chosen: Vec<usize> = Vec::new();
        for _ in 0..k {
            let p = rng.below(i as u64) as usize;
            if !chosen.contains(&p) {
                chosen.push(p);
            }
        }
        chosen.sort_unstable();
        *producers = chosen;
    }
    // Which nodes have downstream consumers (need an output port).
    let mut has_consumer = vec![false; n];
    for prods in &producers_of {
        for &p in prods {
            has_consumer[p] = true;
        }
    }
    let mut nodes: Vec<NodeDef> = Vec::with_capacity(n);
    let mut edges: Vec<(String, String)> = Vec::new();
    let mut t = TriggerEdges::new();
    for i in 0..n {
        let inputs: Vec<InputDef> = producers_of[i]
            .iter()
            .enumerate()
            .map(|(j, &p)| InputDef {
                name: format!("in{j}"),
                source: format!("v{p}/out"),
            })
            .collect();
        let outputs = if has_consumer[i] {
            vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }]
        } else {
            Vec::new()
        };
        nodes.push(NodeDef {
            ros2: None,
            id: ids[i].clone(),
            node_type: ids[i].clone(),
            inputs,
            outputs,
        });
        for &p in &producers_of[i] {
            edges.push((ids[p].clone(), ids[i].clone()));
            t.insert(ids[i].clone(), format!("/{PREFIX}/v{p}/out"));
        }
    }
    (config_of(nodes), t, ids, edges)
}

fn gen_costs(
    rng: &mut Rng,
    ids: &[String],
    edges: &[(String, String)],
    profitable: bool,
    zero_rates: bool,
) -> PartitionCosts {
    let node_p50_ns: BTreeMap<String, u64> = ids
        .iter()
        .map(|id| (id.clone(), 1 + rng.below(500)))
        .collect();
    let edge_rate_mhz: BTreeMap<(String, String), u64> = edges
        .iter()
        .map(|(p, c)| {
            let r = if zero_rates { 0 } else { rng.below(200_000) };
            ((p.clone(), c.clone()), r)
        })
        .collect();
    let hop = if profitable {
        HopCosts {
            intra_ns: 1_000,
            cross_ns: 6_800,
        }
    } else {
        HopCosts {
            intra_ns: 5_000,
            cross_ns: 5_000,
        }
    };
    PartitionCosts {
        node_p50_ns,
        edge_rate_mhz,
        hop,
    }
}

/// Assert an AutoPartition covers every node exactly once + is spawner-consumable.
fn assert_spawner_consumable(part: &AutoPartition, config: &GraphConfig, t: &TriggerEdges) {
    let infos = infos(config);
    // (1) Complete partition: every node exactly once.
    let mut seen: std::collections::BTreeSet<String> = std::collections::BTreeSet::new();
    for members in part.groups.values() {
        for m in members {
            assert!(seen.insert(m.clone()), "node {m} appears twice");
        }
    }
    let all: std::collections::BTreeSet<String> =
        config.nodes.iter().map(|n| n.id.clone()).collect();
    assert_eq!(seen, all, "every node must be assigned exactly once");

    // (2) The REAL validator passes (same one the partitioner uses internally).
    validate_partition(&part.groups, config, &infos, t).expect("output must validate");

    // (3) The literal install_barrier_participant precondition on each group:
    //     the participant map's non-None entries == 0..local_subgraph_level_count.
    let mut cfg2 = config.clone();
    cfg2.process_groups = part.groups.clone();
    let levels = global_levels(config, t);
    let derived = derive_process_groups(&cfg2, &levels).expect("derive");
    for g in &derived {
        let non_none: Vec<usize> = g.global_level_map.iter().filter_map(|o| *o).collect();
        let members = &part.groups[&g.name];
        let local = local_level_count(members, config, t);
        assert_eq!(
            non_none,
            (0..local).collect::<Vec<_>>(),
            "group '{}' map non-None entries must be a contiguous bijection onto \
             0..{local} (install_barrier_participant precondition)",
            g.name
        );
    }
}

#[test]
fn prop_every_output_is_spawner_consumable() {
    let mut rng = Rng(0x5EED_1234);
    for _ in 0..400 {
        let (config, t, ids, edges) = gen_dag(&mut rng);
        let budget = 1 + rng.below(2_000); // varied budget forces varied splits
        let costs = gen_costs(&mut rng, &ids, &edges, true, false);
        let part = auto_partition(&config, &infos(&config), &t, &costs, &iso_none(), budget)
            .expect("partition");
        assert_spawner_consumable(&part, &config, &t);
    }
}

#[test]
fn prop_determinism_byte_identical() {
    let mut rng = Rng(0xD00D_F00D);
    for _ in 0..300 {
        let (config, t, ids, edges) = gen_dag(&mut rng);
        let budget = 1 + rng.below(2_000);
        let costs = gen_costs(&mut rng, &ids, &edges, true, false);
        let a =
            auto_partition(&config, &infos(&config), &t, &costs, &iso_none(), budget).expect("a");
        let b =
            auto_partition(&config, &infos(&config), &t, &costs, &iso_none(), budget).expect("b");
        assert_eq!(a, b, "two runs on identical inputs must be byte-identical");
    }
}

#[test]
fn prop_unprofitable_hops_are_process_per_node() {
    let mut rng = Rng(0xBEEF_0001);
    for _ in 0..200 {
        let (config, t, ids, edges) = gen_dag(&mut rng);
        let costs = gen_costs(&mut rng, &ids, &edges, false, false); // cross == intra
        let part = auto_partition(&config, &infos(&config), &t, &costs, &iso_none(), u64::MAX)
            .expect("partition");
        assert_eq!(
            part.groups.len(),
            config.nodes.len(),
            "cross <= intra => process-per-node"
        );
        assert!(part.fusions.is_empty(), "no fusions when unprofitable");
        assert_spawner_consumable(&part, &config, &t);
    }
}

#[test]
fn prop_zero_rates_are_process_per_node() {
    let mut rng = Rng(0xACE0_1111);
    for _ in 0..200 {
        let (config, t, ids, edges) = gen_dag(&mut rng);
        let costs = gen_costs(&mut rng, &ids, &edges, true, true); // profitable hops, 0 rates
        let part = auto_partition(&config, &infos(&config), &t, &costs, &iso_none(), u64::MAX)
            .expect("partition");
        assert_eq!(
            part.groups.len(),
            config.nodes.len(),
            "all-zero rates => process-per-node"
        );
        assert!(part.fusions.is_empty());
        assert_spawner_consumable(&part, &config, &t);
    }
}

// ==========================================================================
// DESCENDING-COUPLING ranking is load-bearing.
//
// Every oracle above uses uniform couplings, so only the graph-order TIEBREAK
// is exercised. These two tests flip ONLY the per-edge rates on the SAME
// 3-chain under a pair-only budget, yielding DIFFERENT partitions — proving the
// greedy order is driven by coupling MAGNITUDE, not graph order. (They also pin
// the OverBudget rejection record.)
// ==========================================================================

#[test]
fn descending_coupling_higher_edge_wins_tail_pair() {
    // 3-chain n0->n1->n2, cost 100/node, budget 250 (a pair=200 fits, a
    // triple=300 does not). rate(n1,n2) > rate(n0,n1) => coupling(n1,n2) is
    // ranked first, so the TAIL pair {n1,n2} fuses and n0 is starved solo.
    let (config, t, ids, _edges) = chain(3);
    let ids_ref: Vec<&str> = ids.iter().map(|s| s.as_str()).collect();
    let cst = costs_edges(&ids_ref, 100, &[("n0", "n1", 10_000), ("n1", "n2", 20_000)]);
    let part =
        auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 250).expect("partition");
    assert_eq!(
        shape(&part),
        expect(&[("grp_n0", &["n0"]), ("grp_n1", &["n1", "n2"])]),
        "the HIGHER-coupling tail edge fuses first; n0 is budget-starved solo"
    );
    // Exactly one fusion — the tail edge — proving order picked it over (n0,n1).
    let fused: Vec<(String, String)> = part
        .fusions
        .iter()
        .map(|f| (f.producer.clone(), f.consumer.clone()))
        .collect();
    assert_eq!(fused, vec![("n1".into(), "n2".into())]);
    // The starved head edge is recorded OverBudget with the exact merged load.
    assert_eq!(
        part.rejections,
        vec![FusionRejection {
            producer: "n0".into(),
            consumer: "n1".into(),
            coupling: 10_000u128 * SAVING,
            reason: FusionRejectionReason::OverBudget {
                load: 300,
                budget: 250,
            },
        }],
        "the head edge is starved AFTER the tail fuses — OverBudget{{300,250}}"
    );
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

#[test]
fn descending_coupling_flip_reverses_outcome() {
    // Anti-symmetry control: SAME graph + budget, only the rates flipped so
    // rate(n0,n1) > rate(n1,n2). Now the HEAD pair {n0,n1} fuses and the TAIL
    // node n2 is starved solo — the mirror image of the test above, driven
    // purely by which coupling ranks higher.
    let (config, t, ids, _edges) = chain(3);
    let ids_ref: Vec<&str> = ids.iter().map(|s| s.as_str()).collect();
    let cst = costs_edges(&ids_ref, 100, &[("n0", "n1", 20_000), ("n1", "n2", 10_000)]);
    let part =
        auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 250).expect("partition");
    assert_eq!(
        shape(&part),
        expect(&[("grp_n0", &["n0", "n1"]), ("grp_n2", &["n2"])]),
        "flipping the rates fuses the HEAD pair instead — outcome tracks coupling order"
    );
    let fused: Vec<(String, String)> = part
        .fusions
        .iter()
        .map(|f| (f.producer.clone(), f.consumer.clone()))
        .collect();
    assert_eq!(fused, vec![("n0".into(), "n1".into())]);
    assert_eq!(
        part.rejections,
        vec![FusionRejection {
            producer: "n1".into(),
            consumer: "n2".into(),
            coupling: 10_000u128 * SAVING,
            reason: FusionRejectionReason::OverBudget {
                load: 300,
                budget: 250,
            },
        }]
    );
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

// ==========================================================================
// The in-fusion `GroupCheck::Bridged` rejection path.
//
// The standalone `validate_partition` bridge arm is covered above; this pins
// the bridge rejection HAPPENING DURING auto_partition. A "skip" trigger edge
// (n0->n3) lets greedy fusion ATTEMPT {n0,n2,n3} after {n2,n3} exists — that
// group owns non-contiguous globals {0,2,3} with n2's only trigger predecessor
// (n1) foreign, so it re-levelizes short and is rejected as Bridged.
// ==========================================================================

/// n0->n1->n2->n3 chain PLUS a "skip" trigger edge n0->n3. Global levels:
/// n0=0, n1=1, n2=2, n3=3 (the skip edge does not lower n3, which still
/// depends on n2). Candidate pairs: (n0,n1),(n1,n2),(n2,n3),(n0,n3).
fn skip_edge_graph() -> (GraphConfig, TriggerEdges) {
    let config = config_of(vec![
        node("n0", &[], &["out"]),
        node("n1", &[("inp", "n0/out")], &["out"]),
        node("n2", &[("inp", "n1/out")], &["out"]),
        // n3 triggers on BOTH n2 (the chain) and n0 (the skip edge).
        node("n3", &[("a", "n2/out"), ("b", "n0/out")], &[]),
    ]);
    let t = triggers(&[
        ("n1", "n0/out"),
        ("n2", "n1/out"),
        ("n3", "n2/out"),
        ("n3", "n0/out"),
    ]);
    (config, t)
}

#[test]
fn in_fusion_bridge_rejection_records_named_bridge_and_partition() {
    let (config, t) = skip_edge_graph();
    // rate(n2,n3) HIGHEST => {n2,n3} fuses first; rate(n0,n3) SECOND => the skip
    // edge is attempted next, merging {n0} with {n2,n3} => bridged (n1 foreign).
    // (n0,n1) and (n1,n2) carry no rate => coupling 0 => Unprofitable, never
    // fuse. Budget 100_000 is generous so the bridge — NOT the budget — is the
    // gate that rejects the skip fusion.
    let cst = costs_edges(
        &["n0", "n1", "n2", "n3"],
        100,
        &[("n2", "n3", 20_000), ("n0", "n3", 10_000)],
    );
    let part = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 100_000)
        .expect("partition");

    // Only the tail pair fused; n0 and n1 stay solo.
    assert_eq!(
        shape(&part),
        expect(&[
            ("grp_n0", &["n0"]),
            ("grp_n1", &["n1"]),
            ("grp_n2", &["n2", "n3"]),
        ]),
        "the bridged skip fusion is refused, leaving n0/n1 solo + the tail pair fused"
    );
    assert_eq!(
        part.fusions
            .iter()
            .map(|f| (f.producer.clone(), f.consumer.clone()))
            .collect::<Vec<_>>(),
        vec![("n2".into(), "n3".into())],
        "only the tail edge fused"
    );

    // The rejection audit trail, in consideration (descending-coupling) order:
    // the skip edge Bridged(n1), then the two zero-coupling edges Unprofitable.
    assert_eq!(
        part.rejections,
        vec![
            FusionRejection {
                producer: "n0".into(),
                consumer: "n3".into(),
                coupling: 10_000u128 * SAVING,
                reason: FusionRejectionReason::Bridged {
                    bridge: "n1".into(),
                },
            },
            FusionRejection {
                producer: "n0".into(),
                consumer: "n1".into(),
                coupling: 0,
                reason: FusionRejectionReason::Unprofitable,
            },
            FusionRejection {
                producer: "n1".into(),
                consumer: "n2".into(),
                coupling: 0,
                reason: FusionRejectionReason::Unprofitable,
            },
        ],
        "the in-fusion bridge names n1; the zero-rate edges are Unprofitable"
    );

    // The emitted partition is still spawner-consumable by construction.
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

// ==========================================================================
// NON-CONTIGUOUS owned-band rejection (the direct-edge case the
// re-levelization bijection alone does NOT catch).
//
// The Bridged arm above catches a gapped band whose gap is caused by a FOREIGN
// bridge (the bijection FAILS). This section covers the OTHER way a group owns
// a gapped band: a DIRECT in-group edge that spans the gap keeps the bijection
// INTACT (owned {0,4} with a direct 0→4 edge re-levelizes to locals {0,1}), so
// only the explicit contiguity test rejects it.
// ==========================================================================

/// The autoware-shaped graph. A source `a`@L0 has a DIRECT trigger
/// edge to consumer `c`; `c` ALSO consumes the tail of a 4-deep sibling chain
/// `s0→s1→s2→s3` (`s3`@L3), so `c` re-levelizes to global level 4. The direct
/// `a→c` edge spans the NON-CONTIGUOUS band {0,4} yet keeps a fused {a,c}
/// group's re-levelization bijection intact (locals collapse to {0,1}).
fn direct_edge_to_deep_consumer_graph() -> (GraphConfig, TriggerEdges) {
    let config = config_of(vec![
        node("a", &[], &["out"]),                           // global level 0
        node("s0", &[], &["out"]),                          // global level 0
        node("s1", &[("inp", "s0/out")], &["out"]),         // global level 1
        node("s2", &[("inp", "s1/out")], &["out"]),         // global level 2
        node("s3", &[("inp", "s2/out")], &["out"]),         // global level 3
        node("c", &[("x", "a/out"), ("y", "s3/out")], &[]), // global level 4
    ]);
    let t = triggers(&[
        ("s1", "s0/out"),
        ("s2", "s1/out"),
        ("s3", "s2/out"),
        ("c", "a/out"),
        ("c", "s3/out"),
    ]);
    (config, t)
}

#[test]
fn in_fusion_non_contiguous_direct_edge_rejected() {
    // (a) The autoware-shaped oracle: ONLY the direct a→c edge carries a rate,
    // so it is the single positive-coupling candidate (attempted first). Every
    // chain edge is rate 0 => coupling 0 => Unprofitable, never fused. Budget
    // 100_000 is generous so CONTIGUITY — not budget — is the gate that rejects
    // the a→c fusion.
    let (config, t) = direct_edge_to_deep_consumer_graph();
    let cst = costs_edges(
        &["a", "s0", "s1", "s2", "s3", "c"],
        100,
        &[("a", "c", 20_000)],
    );
    let part = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 100_000)
        .expect("partition");

    // a stays a SINGLETON — the a→c fusion is rejected NonContiguous.
    assert!(
        part.fusions.is_empty(),
        "no fusion is taken; got {:?}",
        part.fusions
    );
    // The final partition is process-per-node (each group owns exactly ONE
    // global level => fully contiguous).
    assert_eq!(
        shape(&part),
        expect(&[
            ("grp_a", &["a"]),
            ("grp_s0", &["s0"]),
            ("grp_s1", &["s1"]),
            ("grp_s2", &["s2"]),
            ("grp_s3", &["s3"]),
            ("grp_c", &["c"]),
        ]),
        "the non-contiguous a→c fusion is refused, leaving a solo"
    );
    // The rejection audit trail NAMES the candidate edge (a→c) AND the gapped
    // owned band {0,4}.
    assert!(
        part.rejections.iter().any(|r| r.producer == "a"
            && r.consumer == "c"
            && r.reason == FusionRejectionReason::NonContiguous { owned: vec![0, 4] }),
        "a→c must be rejected NonContiguous owning {{0,4}}; got: {:?}",
        part.rejections
    );
    // The emitted partition is spawner-consumable by construction.
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

#[test]
fn validate_partition_rejects_hand_built_non_contiguous_group() {
    // (b) A hand-built process_groups block putting the direct-edge endpoints
    // {a, c} in ONE group (owning gapped globals {0,4}) is rejected with the
    // contiguity message — even though {a,c}'s subgraph re-levelizes to a
    // contiguous local band (so the bijection alone would PASS it). The sibling
    // chain forms the second group (contiguous {0,1,2,3} — the anti-tautology).
    let (config, t) = direct_edge_to_deep_consumer_graph();
    let bad: IndexMap<String, Vec<String>> = [
        ("G_ac".to_string(), vec!["a".into(), "c".into()]),
        (
            "G_chain".to_string(),
            vec!["s0".into(), "s1".into(), "s2".into(), "s3".into()],
        ),
    ]
    .into_iter()
    .collect();
    let err = validate_partition(&bad, &config, &infos(&config), &t)
        .expect_err("a non-contiguous group must be rejected")
        .to_string();
    // One-voice wording shared with the runtime + plan-time guard, PLUS the set.
    assert!(
        err.contains("non-adjacent global DAG levels")
            && err.contains("CONTIGUOUS-split partition")
            && err.contains("[0, 4]"),
        "message must reuse the runtime wording + name the gapped band {{0,4}}; got: {err}"
    );
    assert!(
        err.contains("G_ac"),
        "message must name the offending group; got: {err}"
    );
}

#[test]
fn contiguity_boundary_controls_stay_valid() {
    // (c) Boundary controls: a {2,3}-BAND group and single-node groups all own
    // CONTIGUOUS bands, so `validate_partition` accepts them — the contiguity
    // test must not over-reject. Uses the same direct-edge graph but splits the
    // chain at the {2,3} band (a mid-pipeline 2-level band) with a/c/s0/s1 as
    // singletons.
    let (config, t) = direct_edge_to_deep_consumer_graph();
    let good: IndexMap<String, Vec<String>> = [
        ("g_a".to_string(), vec!["a".into()]),   // owns {0}
        ("g_s0".to_string(), vec!["s0".into()]), // owns {0}
        ("g_s1".to_string(), vec!["s1".into()]), // owns {1}
        ("g_band".to_string(), vec!["s2".into(), "s3".into()]), // owns {2,3}
        ("g_c".to_string(), vec!["c".into()]),   // owns {4}
    ]
    .into_iter()
    .collect();
    validate_partition(&good, &config, &infos(&config), &t)
        .expect("a {2,3} band + singletons are all contiguous");
}

// ==========================================================================
// Regression pins.
//   (1) transient-gap-heals — the gate rejects only the PREMATURE gap-spanning
//       candidate; later ADJACENT merges reassemble the SAME contiguous group.
//   (2) a robotics-CLASS multi-level fuse is byte-identical pre/post the contiguity fix
//       (a shape with NO gap-spanner is untouched by the new contiguity test).
// ==========================================================================

#[test]
fn transient_gap_edge_rejected_but_group_reassembles_contiguously() {
    // n0→n1→n2→n3 chain PLUS a skip edge n0→n3 (n3 still @L3 — it depends on
    // n2). Cost the SKIP edge HIGHEST so it is the FIRST candidate: fusing {n0,n3}
    // owns the gapped band {0,3} ⇒ rejected NonContiguous (the direct skip edge
    // keeps the bijection intact, so this is NOT a Bridged rejection). The three
    // ADJACENT edges are all profitable and fuse in order, so the final group is
    // the WHOLE chain {n0,n1,n2,n3} — CONTIGUOUS {0,1,2,3}, containing BOTH n0
    // and n3. The premature gap-spanner is refused; the adjacent merges reassemble
    // the exact same group a gap-blind partitioner would produce.
    let (config, t) = skip_edge_graph();
    let cst = costs_edges(
        &["n0", "n1", "n2", "n3"],
        100,
        &[
            ("n0", "n3", 30_000), // the gap-spanner — offered FIRST
            ("n0", "n1", 20_000),
            ("n1", "n2", 20_000),
            ("n2", "n3", 20_000),
        ],
    );
    let part = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 100_000)
        .expect("partition");

    // The final group is the WHOLE contiguous chain (n0..n3 reassembled).
    assert_eq!(
        shape(&part),
        expect(&[("grp_n0", &["n0", "n1", "n2", "n3"])]),
        "the adjacent merges reassemble the full contiguous chain"
    );
    // The adjacent edges fused in consideration order; the skip edge did NOT.
    assert_eq!(
        part.fusions
            .iter()
            .map(|f| (f.producer.clone(), f.consumer.clone()))
            .collect::<Vec<_>>(),
        vec![
            ("n0".into(), "n1".into()),
            ("n1".into(), "n2".into()),
            ("n2".into(), "n3".into()),
        ],
        "only the adjacent chain edges fused"
    );
    // The premature gap-spanner was rejected NonContiguous owning {0,3}.
    assert!(
        part.rejections.iter().any(|r| r.producer == "n0"
            && r.consumer == "n3"
            && r.reason == FusionRejectionReason::NonContiguous { owned: vec![0, 3] }),
        "n0→n3 must be rejected NonContiguous owning {{0,3}}; got: {:?}",
        part.rejections
    );
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

#[test]
fn robotics_class_fan_in_pipeline_fuse_is_byte_identical() {
    // A nav2/perception-CLASS shape with NO gap-spanning edge: the
    // contiguity test must leave a normal multi-level fuse UNCHANGED. Two sensor
    // sources (laser@0, odom@0) feed a costmap@1; a planner@2 and controller@3
    // form the tail. Every trigger edge is rated; the budget fits the whole
    // pipeline, so the correct result is ONE contiguous group
    // owning {0,1,2,3}. This is the byte-identical regression guard: the
    // NonContiguous test must not perturb a defect-free fuse.
    let config = config_of(vec![
        node("laser", &[], &["out"]), // L0
        node("odom", &[], &["out"]),  // L0
        node(
            "costmap",
            &[("l", "laser/out"), ("o", "odom/out")],
            &["out"],
        ), // L1
        node("planner", &[("cm", "costmap/out")], &["out"]), // L2
        node("controller", &[("pl", "planner/out")], &[]), // L3
    ]);
    let t = triggers(&[
        ("costmap", "laser/out"),
        ("costmap", "odom/out"),
        ("planner", "costmap/out"),
        ("controller", "planner/out"),
    ]);
    let cst = costs_edges(
        &["laser", "odom", "costmap", "planner", "controller"],
        100,
        &[
            ("laser", "costmap", 20_000),
            ("odom", "costmap", 20_000),
            ("costmap", "planner", 20_000),
            ("planner", "controller", 20_000),
        ],
    );
    let part = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 100_000)
        .expect("partition");
    assert_eq!(
        shape(&part),
        expect(&[(
            "grp_laser",
            &["laser", "odom", "costmap", "planner", "controller"],
        )]),
        "a defect-free multi-level pipeline fuses into one contiguous group (unchanged by the contiguity fix)"
    );
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

// ==========================================================================
// Budget boundary (load == budget) — the gate is `load >
// budget_ns`, so equality FUSES. A variant that uses `>=` instead of `>`
// would reject at equality (leaving process-per-node), which these pin
// from both sides.
// ==========================================================================

#[test]
fn budget_exactly_equal_to_load_is_fused() {
    // 2-chain, cost 100/node => merged load 200; budget EXACTLY 200 => fuse.
    let (config, t, ids, edges) = chain(2);
    let cst = chain_costs(&edges, &ids, 100, 10_000);
    let part =
        auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 200).expect("partition");
    assert_eq!(
        shape(&part),
        expect(&[("grp_n0", &["n0", "n1"])]),
        "load 200 == budget 200 must FUSE (gate is > not >=)"
    );
    assert_eq!(part.fusions.len(), 1);
    assert!(
        part.rejections.is_empty(),
        "the sole candidate fused — no rejection recorded"
    );
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

#[test]
fn budget_one_below_load_rejects_over_budget() {
    // The just-over twin: budget 199 < load 200 => rejected OverBudget with the
    // exact load/budget fields (the rejection's audit trail).
    let (config, t, ids, edges) = chain(2);
    let cst = chain_costs(&edges, &ids, 100, 10_000);
    let part =
        auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 199).expect("partition");
    assert_eq!(part.groups.len(), 2, "load 200 > budget 199 => not fused");
    assert!(part.fusions.is_empty());
    assert_eq!(
        part.rejections,
        vec![FusionRejection {
            producer: "n0".into(),
            consumer: "n1".into(),
            coupling: 10_000u128 * SAVING,
            reason: FusionRejectionReason::OverBudget {
                load: 200,
                budget: 199,
            },
        }],
        "the over-budget candidate records the exact load and budget"
    );
}

#[test]
fn unprofitable_edge_records_unprofitable_rejection() {
    // A zero-coupling (zero-rate) edge is recorded Unprofitable.
    // n0->n1->n2: only (n0,n1) carries a rate; (n1,n2) has none => coupling 0.
    let (config, t, ids, _edges) = chain(3);
    let cst = chain_costs(&[("n0".into(), "n1".into())], &ids, 100, 10_000);
    let part = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 100_000)
        .expect("partition");
    assert_eq!(part.fusions.len(), 1, "(n0,n1) fuses");
    assert_eq!(
        part.rejections,
        vec![FusionRejection {
            producer: "n1".into(),
            consumer: "n2".into(),
            coupling: 0,
            reason: FusionRejectionReason::Unprofitable,
        }],
        "the zero-rate edge is the sole rejection: Unprofitable, coupling 0"
    );
}

// ==========================================================================
// Fix interaction: the already-same-group skip must
// PRECEDE the coupling==0 gate. A zero-coupling candidate whose endpoints were
// already TRANSITIVELY fused (via OTHER rated edges) asked for nothing — it is
// neither a fusion NOR a rejection (AutoPartition.rejections' own doc). Recording
// it Unprofitable would pollute the audit trail the partition emitter consumes.
// Reverting the reorder records an Unprofitable (n0,n2) => the triangle test FAILS.
// ==========================================================================

/// Triangle n0->n1, n1->n2, PLUS the skip edge n0->n2. Global levels n0=0,
/// n1=1, n2=2 (n2 triggers on BOTH n1 and n0, so max(1,0)+1). A generous budget
/// fuses the whole triangle via the two rated edges — by the time the
/// zero-coupling skip edge (n0,n2) is considered, n0 and n2 are ALREADY in one
/// group, so it must be SKIPPED (not recorded).
fn triangle_graph() -> (GraphConfig, TriggerEdges) {
    let config = config_of(vec![
        node("n0", &[], &["out"]),
        node("n1", &[("inp", "n0/out")], &["out"]),
        node("n2", &[("a", "n1/out"), ("b", "n0/out")], &[]),
    ]);
    let t = triggers(&[("n1", "n0/out"), ("n2", "n1/out"), ("n2", "n0/out")]);
    (config, t)
}

#[test]
fn transitively_fused_zero_coupling_edge_is_not_recorded_rejection() {
    // (n0,n1)=20000 ranks first, (n1,n2)=10000 second, (n0,n2)=0 last. The two
    // rated edges fuse the WHOLE triangle; the zero-coupling skip edge (n0,n2) is
    // then already-same-group => SKIPPED before the coupling==0 gate. Reverting
    // the reorder records it Unprofitable, so `rejections` becomes non-empty here.
    let (config, t) = triangle_graph();
    let cst = costs_edges(
        &["n0", "n1", "n2"],
        100,
        &[("n0", "n1", 20_000), ("n1", "n2", 10_000), ("n0", "n2", 0)],
    );
    let part = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 100_000)
        .expect("partition");
    assert_eq!(
        shape(&part),
        expect(&[("grp_n0", &["n0", "n1", "n2"])]),
        "the two rated edges fuse the whole triangle into one group"
    );
    assert_eq!(part.fusions.len(), 2, "exactly the two rated edges fused");
    // THE PIN: the already-same-group skip edge (n0,n2) is recorded NOWHERE.
    assert!(
        part.rejections
            .iter()
            .all(|r| !(r.producer == "n0" && r.consumer == "n2")),
        "an already-same-group zero-coupling edge must NOT be a rejection; got {:?}",
        part.rejections
    );
    assert!(
        part.rejections.is_empty(),
        "the whole triangle fused — no rejection of any kind; got {:?}",
        part.rejections
    );
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

#[test]
fn disconnected_zero_coupling_edge_still_records_unprofitable() {
    // Anti-over-fix control: the reorder skips only ALREADY-SAME-GROUP candidates;
    // it must NOT drop all zero-coupling records. A 2-node graph n0->n1 at rate 0
    // — endpoints NOT otherwise connected — never fuses, so its sole candidate is
    // still recorded Unprofitable. (Dropping every zero-coupling record fails here.)
    let (config, t, ids, edges) = chain(2);
    let cst = chain_costs(&edges, &ids, 100, 0); // rate 0 => coupling 0
    let part = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 100_000)
        .expect("partition");
    assert_eq!(part.groups.len(), 2, "a zero-rate edge never fuses");
    assert!(part.fusions.is_empty());
    assert_eq!(
        part.rejections,
        vec![FusionRejection {
            producer: "n0".into(),
            consumer: "n1".into(),
            coupling: 0,
            reason: FusionRejectionReason::Unprofitable,
        }],
        "a disconnected zero-coupling edge is still recorded Unprofitable"
    );
}

// ==========================================================================
// The merged-load budget fold SATURATES on
// overflow (same discipline as the coupling arithmetic). A pathological node-cost
// sum must CLAMP to u64::MAX (rejecting the fusion), never WRAP.
// ==========================================================================

#[test]
fn budget_fold_saturates_on_overflow_and_rejects() {
    // 2-chain n0->n1 with a NONZERO edge rate (so the candidate is profitable and
    // reaches the budget gate). Costs u64::MAX + 100 => the merged load SATURATES
    // to u64::MAX, which exceeds budget 200 => OverBudget, process-per-node. A
    // WRAPPING `.sum()` would compute u64::MAX.wrapping_add(100) == 99 <= 200 and
    // FUSE — so this test fails on a `.sum()` revert.
    let (config, t, _ids, _edges) = chain(2);
    let cst = PartitionCosts {
        node_p50_ns: [("n0".to_string(), u64::MAX), ("n1".to_string(), 100)]
            .into_iter()
            .collect(),
        edge_rate_mhz: [(("n0".to_string(), "n1".to_string()), 10_000u64)]
            .into_iter()
            .collect(),
        hop: HopCosts {
            intra_ns: INTRA_NS,
            cross_ns: CROSS_NS,
        },
    };
    let part =
        auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 200).expect("partition");
    assert_eq!(
        part.groups.len(),
        2,
        "the saturated load u64::MAX > budget 200 => process-per-node"
    );
    assert!(
        part.fusions.is_empty(),
        "no fusion under the saturated load"
    );
    assert_eq!(
        part.rejections,
        vec![FusionRejection {
            producer: "n0".into(),
            consumer: "n1".into(),
            coupling: 10_000u128 * SAVING,
            reason: FusionRejectionReason::OverBudget {
                load: u64::MAX,
                budget: 200,
            },
        }],
        "the fold clamps to u64::MAX (not a wrapped 99) => exact OverBudget payload"
    );
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

// ==========================================================================
// Stray cost-snapshot entries WARN (never error) and do
// NOT change the partition. A typo'd key (naming a nonexistent node / edge)
// would otherwise silently fall back to the 0 default and change the outcome.
// ==========================================================================

/// The clean-snapshot oracle partition for the 3-chain used below: uniform
/// coupling + a generous budget fuse the whole chain into one group.
fn clean_3chain_oracle() -> Vec<(String, Vec<String>)> {
    expect(&[("grp_n0", &["n0", "n1", "n2"])])
}

#[traced_test]
#[test]
fn stray_node_cost_key_warns_and_does_not_change_partition() {
    let (config, t, ids, edges) = chain(3);
    let clean = chain_costs(&edges, &ids, 100, 10_000);
    let clean_part =
        auto_partition(&config, &infos(&config), &t, &clean, &iso_none(), 100_000).expect("clean");
    assert_eq!(
        shape(&clean_part),
        clean_3chain_oracle(),
        "clean 3-chain fuses whole (hand oracle)"
    );

    // A stray node_p50_ns key naming a node not in the graph.
    let mut stray = clean.clone();
    stray.node_p50_ns.insert("ghost".to_string(), 999);
    let stray_part =
        auto_partition(&config, &infos(&config), &t, &stray, &iso_none(), 100_000).expect("stray");

    assert!(
        logs_contain("stray cost-snapshot entry"),
        "a stray node_p50_ns key must emit the stray-entry warn"
    );
    assert_eq!(
        shape(&stray_part),
        clean_3chain_oracle(),
        "a stray node cost key must NOT change the partition"
    );
}

#[traced_test]
#[test]
fn stray_edge_rate_key_warns_and_does_not_change_partition() {
    let (config, t, ids, edges) = chain(3);
    let clean = chain_costs(&edges, &ids, 100, 10_000);
    let clean_part =
        auto_partition(&config, &infos(&config), &t, &clean, &iso_none(), 100_000).expect("clean");

    // A TYPO'd edge key: the intended (n1,n2) is mistyped with a nonexistent
    // consumer. The REAL (n1,n2) still carries its rate (chain_costs sets it),
    // so the partition is unchanged; the typo is surfaced loudly.
    let mut stray = clean.clone();
    stray
        .edge_rate_mhz
        .insert(("n1".to_string(), "n2x".to_string()), 50_000);
    let stray_part =
        auto_partition(&config, &infos(&config), &t, &stray, &iso_none(), 100_000).expect("stray");

    assert!(
        logs_contain("stray cost-snapshot entry"),
        "a stray edge_rate_mhz key must emit the stray-entry warn"
    );
    assert_eq!(
        shape(&stray_part),
        clean_3chain_oracle(),
        "a stray edge key must NOT change the partition"
    );
    assert_eq!(
        shape(&clean_part),
        clean_3chain_oracle(),
        "control: the clean snapshot produces the same partition"
    );
}

#[traced_test]
#[test]
fn clean_snapshot_emits_no_stray_warn() {
    // Anti-tautology control: an EXACT snapshot (every key matches a node/edge)
    // must emit NO stray-entry warn — proving the warn is genuinely conditional.
    let (config, t, ids, edges) = chain(3);
    let clean = chain_costs(&edges, &ids, 100, 10_000);
    let _ =
        auto_partition(&config, &infos(&config), &t, &clean, &iso_none(), 100_000).expect("clean");
    assert!(
        !logs_contain("stray cost-snapshot entry"),
        "a clean (exact) snapshot must emit no stray-key warn"
    );
}

// ==========================================================================
// The pure online-profiler harness (`harvest_costs`) + the
// `auto_partition` `isolated` param. Every assertion is against a HAND oracle
// (never a self-compare); the maths (p50, mHz rate) are hand-computed.
// ==========================================================================

/// A trace entry carrying only the two fields `harvest_costs` reads (node id +
/// duration); step/level/fire_time are irrelevant to profiling.
fn te(node_id: &str, duration_ns: u64) -> TraceEntry {
    TraceEntry {
        node_id: Arc::from(node_id),
        step: 0,
        fire_time_ns: 0,
        global_level: 0,
        duration_ns,
        discarded: false,
    }
}

/// Build a `fire_counts` map from `(node, count)` pairs.
fn fires(pairs: &[(&str, u64)]) -> BTreeMap<String, u64> {
    pairs.iter().map(|(id, c)| (id.to_string(), *c)).collect()
}

/// Build a UNIFORM per-node target map (every `node` → `n`) — the
/// map form of a single scalar fire target at the
/// `harvest_costs` call sites.
fn uniform_targets(nodes: &[&str], n: u64) -> BTreeMap<String, u64> {
    nodes.iter().map(|id| (id.to_string(), n)).collect()
}

/// The hop constants used by the harness tests (the same spread as above).
fn hop() -> HopCosts {
    HopCosts {
        intra_ns: INTRA_NS,
        cross_ns: CROSS_NS,
    }
}

const ONE_SEC_NS: u64 = 1_000_000_000;

#[test]
fn harvest_happy_p50_and_rate_oracle() {
    // 3-chain n0->n1->n2. All well-sampled (fires_target=1, counts >= 1).
    // p50 samples chosen so the ODD (n0) and EVEN (n1) medians are distinct and
    // the even case proves the LOWER-median (20, not the 25 average).
    let (config, t, _ids, _edges) = chain(3);
    let trace = vec![
        te("n0", 5),
        te("n0", 25),
        te("n0", 15), // sorted [5,15,25] => median 15 (odd)
        te("n1", 40),
        te("n1", 10),
        te("n1", 30),
        te("n1", 20), // sorted [10,20,30,40] => lower median 20 (NOT 25)
        te("n2", 7),  // single sample => 7
    ];
    // Distinct producer fire counts => distinct edge rates over a 1s window.
    let fc = fires(&[("n0", 10), ("n1", 25), ("n2", 30)]);
    let profile = harvest_costs(
        &config,
        &infos(&config),
        &t,
        &trace,
        &fc,
        ONE_SEC_NS,
        hop(),
        &uniform_targets(&["n0", "n1", "n2"], 1),
    )
    .expect("harvest");

    let oracle = ProfileResult {
        costs: PartitionCosts {
            node_p50_ns: [
                ("n0".to_string(), 15),
                ("n1".to_string(), 20),
                ("n2".to_string(), 7),
            ]
            .into_iter()
            .collect(),
            // rate_mHz = fires * 1e12 / 1e9 = fires * 1000.
            edge_rate_mhz: [
                (("n0".to_string(), "n1".to_string()), 10_000), // n0 fired 10 => 10Hz
                (("n1".to_string(), "n2".to_string()), 25_000), // n1 fired 25 => 25Hz
            ]
            .into_iter()
            .collect(),
            hop: hop(),
        },
        isolated: BTreeSet::new(),
    };
    assert_eq!(profile, oracle, "harvest output must equal the hand oracle");

    // And the output feeds auto_partition (whole chain fuses under a big budget).
    let part = auto_partition(
        &config,
        &infos(&config),
        &t,
        &profile.costs,
        &profile.isolated,
        100_000,
    )
    .expect("partition");
    assert_eq!(shape(&part), expect(&[("grp_n0", &["n0", "n1", "n2"])]));
}

#[test]
fn harvest_rate_integer_math_oracle() {
    // 2-chain n0->n1. n0 fired 30 times over a 2s window => 15 Hz => 15_000 mHz.
    // Hand check: 30 * 1e12 / 2e9 = 30 * 500 = 15_000.
    let (config, t, _ids, _edges) = chain(2);
    let trace = vec![te("n0", 100), te("n1", 200)];
    let fc = fires(&[("n0", 30), ("n1", 30)]);
    let two_sec = 2 * ONE_SEC_NS;
    let profile = harvest_costs(
        &config,
        &infos(&config),
        &t,
        &trace,
        &fc,
        two_sec,
        hop(),
        &uniform_targets(&["n0", "n1"], 1),
    )
    .expect("harvest");
    assert_eq!(
        profile.costs.edge_rate_mhz,
        [(("n0".to_string(), "n1".to_string()), 15_000)]
            .into_iter()
            .collect(),
        "30 fires / 2s = 15_000 mHz (hand-computed)"
    );
    assert!(profile.isolated.is_empty());
    assert_eq!(profile.costs.node_p50_ns.get("n0"), Some(&100));
}

#[test]
fn harvest_undersample_boundary_at_target_kept_one_below_isolated() {
    // fires_target=5. n0 fired EXACTLY 5 (>= target => KEPT); n1 fired 4
    // (< target => ISOLATED). Kills a `<`->`<=` off-by-one: `<=` would isolate
    // n0 too.
    let (config, t, _ids, _edges) = chain(2);
    let trace = vec![te("n0", 42), te("n1", 99)];
    let fc = fires(&[("n0", 5), ("n1", 4)]);
    let profile = harvest_costs(
        &config,
        &infos(&config),
        &t,
        &trace,
        &fc,
        ONE_SEC_NS,
        hop(),
        &uniform_targets(&["n0", "n1"], 5),
    )
    .expect("harvest");
    assert_eq!(
        profile.isolated,
        ["n1".to_string()].into_iter().collect::<BTreeSet<_>>(),
        "only n1 (one below target) is isolated; n0 at exactly target is kept"
    );
    assert_eq!(
        profile.costs.node_p50_ns,
        [("n0".to_string(), 42)].into_iter().collect(),
        "the kept node n0 carries a cost; the isolated n1 does NOT"
    );
    // The (n0,n1) edge touches isolated n1 => omitted (never a fusion candidate).
    assert!(
        profile.costs.edge_rate_mhz.is_empty(),
        "an edge into an isolated node is not emitted; got {:?}",
        profile.costs.edge_rate_mhz
    );
}

#[test]
fn harvest_wellsampled_but_no_duration_samples_is_demoted_to_isolated() {
    // fires_target=1, ALL counts high (well-sampled by fire count), but the
    // trace carries NO sample for n2 (ring-evicted). n2 has no measured p50, so it
    // is DEMOTED to isolated — Principle #13 (never a fabricated 0 cost).
    let (config, t, _ids, _edges) = chain(3);
    let trace = vec![te("n0", 10), te("n1", 20)]; // no n2 entry
    let fc = fires(&[("n0", 100), ("n1", 100), ("n2", 100)]);
    let profile = harvest_costs(
        &config,
        &infos(&config),
        &t,
        &trace,
        &fc,
        ONE_SEC_NS,
        hop(),
        &uniform_targets(&["n0", "n1", "n2"], 1),
    )
    .expect("harvest");
    assert_eq!(
        profile.isolated,
        ["n2".to_string()].into_iter().collect::<BTreeSet<_>>(),
        "n2 is well-sampled by fire count but has no duration sample => isolated"
    );
    assert_eq!(
        profile
            .costs
            .node_p50_ns
            .keys()
            .cloned()
            .collect::<Vec<_>>(),
        vec!["n0".to_string(), "n1".to_string()],
        "only n0/n1 carry a cost; n2 is not fabricated a 0"
    );
    // Still feeds auto_partition with no missing-cost error (n2 isolated).
    auto_partition(
        &config,
        &infos(&config),
        &t,
        &profile.costs,
        &profile.isolated,
        100_000,
    )
    .expect("isolated n2 must not trip the missing-cost error");
}

#[test]
fn harvest_zero_window_yields_zero_rates_no_div_by_zero() {
    // window_ns=0 (degenerate) must NOT panic; every edge rate defaults to 0
    // (unknown rate = do not fuse). p50 is unaffected (window-independent).
    let (config, t, _ids, _edges) = chain(2);
    let trace = vec![te("n0", 55), te("n1", 66)];
    let fc = fires(&[("n0", 1_000_000), ("n1", 1_000_000)]);
    let profile = harvest_costs(
        &config,
        &infos(&config),
        &t,
        &trace,
        &fc,
        0,
        hop(),
        &uniform_targets(&["n0", "n1"], 1),
    )
    .expect("harvest");
    assert_eq!(
        profile.costs.edge_rate_mhz,
        [(("n0".to_string(), "n1".to_string()), 0)]
            .into_iter()
            .collect(),
        "a zero-length window yields rate 0 for every edge (no div-by-zero)"
    );
    assert_eq!(profile.costs.node_p50_ns.get("n0"), Some(&55));
    assert_eq!(profile.costs.node_p50_ns.get("n1"), Some(&66));
}

#[test]
fn harvest_all_undersampled_then_auto_partition_is_process_per_node() {
    // Adversarial: every node under-sampled => all isolated, empty costs. Feeding
    // that to auto_partition must NOT error and must yield process-per-node.
    let (config, t, _ids, _edges) = chain(3);
    let trace = vec![te("n0", 10), te("n1", 20), te("n2", 30)];
    // fires_target=100, every count below => all isolated.
    let fc = fires(&[("n0", 1), ("n1", 2), ("n2", 3)]);
    let profile = harvest_costs(
        &config,
        &infos(&config),
        &t,
        &trace,
        &fc,
        ONE_SEC_NS,
        hop(),
        &uniform_targets(&["n0", "n1", "n2"], 100),
    )
    .expect("harvest");
    assert_eq!(
        profile.isolated,
        ["n0".to_string(), "n1".to_string(), "n2".to_string()]
            .into_iter()
            .collect::<BTreeSet<_>>(),
        "every node is isolated"
    );
    assert!(
        profile.costs.node_p50_ns.is_empty(),
        "no costs when all isolated"
    );
    assert!(
        profile.costs.edge_rate_mhz.is_empty(),
        "no edges when all isolated"
    );

    let part = auto_partition(
        &config,
        &infos(&config),
        &t,
        &profile.costs,
        &profile.isolated,
        u64::MAX,
    )
    .expect("all-isolated must not error");
    assert_eq!(part.groups.len(), 3, "all-isolated => process-per-node");
    assert!(part.fusions.is_empty(), "no fusion among isolated nodes");
    assert_eq!(
        shape(&part),
        expect(&[
            ("grp_n0", &["n0"]),
            ("grp_n1", &["n1"]),
            ("grp_n2", &["n2"]),
        ])
    );
}

#[test]
fn harvest_determinism_byte_identical_and_matches_oracle() {
    // Two runs on identical inputs are byte-identical AND equal a hand oracle
    // (so this is NOT a self-compare). Mixed: n1 isolated, n0/n2 well-sampled.
    let (config, t, _ids, _edges) = chain(3);
    let trace = vec![
        te("n0", 3),
        te("n0", 9),   // sorted [3,9] => lower median 3
        te("n1", 500), // n1 isolated below => this sample is ignored
        te("n2", 8),
    ];
    // fires_target=5: n0=10 kept, n1=2 isolated, n2=5 kept (boundary).
    let fc = fires(&[("n0", 10), ("n1", 2), ("n2", 5)]);
    let a = harvest_costs(
        &config,
        &infos(&config),
        &t,
        &trace,
        &fc,
        ONE_SEC_NS,
        hop(),
        &uniform_targets(&["n0", "n1", "n2"], 5),
    )
    .expect("a");
    let b = harvest_costs(
        &config,
        &infos(&config),
        &t,
        &trace,
        &fc,
        ONE_SEC_NS,
        hop(),
        &uniform_targets(&["n0", "n1", "n2"], 5),
    )
    .expect("b");
    assert_eq!(a, b, "two runs on identical inputs must be byte-identical");

    let oracle = ProfileResult {
        costs: PartitionCosts {
            node_p50_ns: [("n0".to_string(), 3), ("n2".to_string(), 8)]
                .into_iter()
                .collect(),
            // Both edges touch isolated n1 => NEITHER edge is emitted.
            edge_rate_mhz: BTreeMap::new(),
            hop: hop(),
        },
        isolated: ["n1".to_string()].into_iter().collect(),
    };
    assert_eq!(
        a, oracle,
        "harvest output equals the independent hand oracle"
    );
}

// ---- auto_partition `isolated` param: hand-built cost snapshots ----

#[test]
fn auto_partition_isolated_node_stays_singleton_under_hot_edge() {
    // 3-chain n0->n1->n2. n2 is isolated. A HUGE rate on the (n1,n2) edge must
    // NOT pull n2 into a group, while the normal (n0,n1) edge DOES fuse. Proves
    // isolation — not the absence of a rate — is what keeps n2 solo.
    let (config, t, _ids, _edges) = chain(3);
    let cst = PartitionCosts {
        node_p50_ns: [("n0".to_string(), 100), ("n1".to_string(), 100)]
            .into_iter()
            .collect(), // n2 has NO cost (isolated)
        edge_rate_mhz: [
            (("n0".to_string(), "n1".to_string()), 10_000),
            (("n1".to_string(), "n2".to_string()), u64::MAX), // hot edge into n2
        ]
        .into_iter()
        .collect(),
        hop: hop(),
    };
    let iso: BTreeSet<String> = ["n2".to_string()].into_iter().collect();
    let part =
        auto_partition(&config, &infos(&config), &t, &cst, &iso, 100_000).expect("partition");
    assert_eq!(
        shape(&part),
        expect(&[("grp_n0", &["n0", "n1"]), ("grp_n2", &["n2"])]),
        "the hot edge into isolated n2 never fuses; n0/n1 fuse normally"
    );
    assert_eq!(
        part.fusions
            .iter()
            .map(|f| (f.producer.clone(), f.consumer.clone()))
            .collect::<Vec<_>>(),
        vec![("n0".into(), "n1".into())],
        "only the non-isolated edge fused"
    );
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

#[test]
fn auto_partition_isolated_node_needs_no_cost() {
    // The core contract: an isolated node absent from node_p50_ns must NOT trip
    // the missing-cost error. Anti-tautology control: WITHOUT the isolated set
    // the same snapshot IS a loud error.
    let (config, t, _ids, _edges) = chain(2);
    let cst = PartitionCosts {
        node_p50_ns: [("n0".to_string(), 100)].into_iter().collect(), // n1 MISSING
        edge_rate_mhz: BTreeMap::new(),
        hop: hop(),
    };
    // Control: empty isolated => n1's missing cost is a loud error.
    let err = auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), 100_000)
        .expect_err("a missing cost with no isolation must error")
        .to_string();
    assert!(err.contains("n1") && err.contains("no p50 duration"));

    // With n1 isolated => success, n1 is its own singleton group.
    let iso: BTreeSet<String> = ["n1".to_string()].into_iter().collect();
    let part = auto_partition(&config, &infos(&config), &t, &cst, &iso, 100_000)
        .expect("isolated n1 needs no cost");
    assert_eq!(
        shape(&part),
        expect(&[("grp_n0", &["n0"]), ("grp_n1", &["n1"])]),
        "the isolated node stays a singleton with no fabricated cost"
    );
}

#[test]
fn auto_partition_isolated_typo_is_loud_error() {
    // An isolated name that is not a real graph node is a typo => loud error
    // naming the bad name (kills a silent wrong-node exemption).
    let (config, t, ids, edges) = chain(2);
    let cst = chain_costs(&edges, &ids, 100, 10_000);
    let iso: BTreeSet<String> = ["n1x".to_string()].into_iter().collect();
    let err = auto_partition(&config, &infos(&config), &t, &cst, &iso, 100_000)
        .expect_err("a typo'd isolated name must be a loud error")
        .to_string();
    assert!(
        err.contains("n1x") && err.contains("not a node in this graph"),
        "the error must name the bogus isolated node; got: {err}"
    );
}

// ==========================================================================
// A RECORDING-OFF-shaped trace (fires push
// TraceEntries but duration_ns is hard-0) must ISOLATE, never cost 0. A p50
// of 0 is a fabricated "free" node that always fuses (Principle #13). Kills
// Catches a revert to a `!samples.is_empty()`-only guard (which costed 0).
// ==========================================================================

#[test]
fn harvest_recording_off_zero_durations_isolate_not_cost_zero() {
    // Both nodes well-sampled by FIRE COUNT, and the trace carries samples for
    // both — but every duration is 0 (exactly what a recording-off run
    // produces: fires always push entries; duration_ns stays 0). Neither node
    // may be costed 0; both are demoted to isolated.
    let (config, t, _ids, _edges) = chain(2);
    let trace = vec![te("n0", 0), te("n0", 0), te("n1", 0)];
    let fc = fires(&[("n0", 100), ("n1", 100)]);
    let profile = harvest_costs(
        &config,
        &infos(&config),
        &t,
        &trace,
        &fc,
        ONE_SEC_NS,
        hop(),
        &uniform_targets(&["n0", "n1"], 1),
    )
    .expect("harvest");
    assert!(
        profile.costs.node_p50_ns.is_empty(),
        "a zero lower-median must NEVER be costed (fabricated free node); got {:?}",
        profile.costs.node_p50_ns
    );
    assert_eq!(
        profile.isolated,
        ["n0".to_string(), "n1".to_string()]
            .into_iter()
            .collect::<BTreeSet<_>>(),
        "recording-off-shaped nodes are ISOLATED"
    );
    // And the demoted output still feeds auto_partition (totality holds).
    auto_partition(
        &config,
        &infos(&config),
        &t,
        &profile.costs,
        &profile.isolated,
        100_000,
    )
    .expect("all-isolated output must not trip the missing-cost error");
}

#[test]
fn harvest_zero_samples_with_nonzero_median_still_costed() {
    // Anti-over-fix control: SOME zero samples are fine as long as the LOWER
    // MEDIAN is nonzero — only a zero median (recording-off-indistinguishable)
    // demotes. sorted [0, 5, 9] => median 5 => costed 5, NOT isolated.
    let (config, t, _ids, _edges) = chain(2);
    let trace = vec![
        te("n0", 0),
        te("n0", 9),
        te("n0", 5),
        te("n1", 7), // plain nonzero single sample
    ];
    let fc = fires(&[("n0", 100), ("n1", 100)]);
    let profile = harvest_costs(
        &config,
        &infos(&config),
        &t,
        &trace,
        &fc,
        ONE_SEC_NS,
        hop(),
        &uniform_targets(&["n0", "n1"], 1),
    )
    .expect("harvest");
    assert_eq!(
        profile.costs.node_p50_ns,
        [("n0".to_string(), 5), ("n1".to_string(), 7)]
            .into_iter()
            .collect(),
        "a nonzero lower-median with some zero samples is still a measured cost"
    );
    assert!(profile.isolated.is_empty(), "nothing demoted");
}

#[test]
fn harvest_zero_median_from_even_count_isolates_conservatively() {
    // Boundary: sorted [0, 10] => LOWER median 0 (index (2-1)/2 = 0) => the
    // conservative direction is isolation, even though a real nonzero sample
    // exists — a zero median cannot be distinguished from recording-off.
    let (config, t, _ids, _edges) = chain(2);
    let trace = vec![te("n0", 10), te("n0", 0), te("n1", 3)];
    let fc = fires(&[("n0", 100), ("n1", 100)]);
    let profile = harvest_costs(
        &config,
        &infos(&config),
        &t,
        &trace,
        &fc,
        ONE_SEC_NS,
        hop(),
        &uniform_targets(&["n0", "n1"], 1),
    )
    .expect("harvest");
    assert_eq!(
        profile.isolated,
        ["n0".to_string()].into_iter().collect::<BTreeSet<_>>(),
        "a zero LOWER median (even-count tiebreak) isolates conservatively"
    );
    assert_eq!(
        profile.costs.node_p50_ns,
        [("n1".to_string(), 3)].into_iter().collect(),
        "the sibling with a measured nonzero median stays costed"
    );
}

// ==========================================================================
// Per-node fire-target derivation (`derive_fire_targets` +
// `FireTargetPolicy`) and `harvest_costs`'s per-node isolation gate. Every
// assertion is a HAND-COMPUTED integer oracle (never a self-compare).
// ==========================================================================

#[test]
fn fire_target_policy_default_constants_pinned() {
    // The default policy — pinned explicitly so a constant change is a
    // loud test failure, not a silent behavior shift.
    let p = FireTargetPolicy::default();
    assert_eq!(p.min_samples, 20, "default min_samples");
    assert_eq!(p.max_samples, 1000, "default max_samples");
    assert_eq!(p.fraction_num, 1, "default fraction numerator");
    assert_eq!(p.fraction_den, 2, "default fraction denominator (÷2 droop)");
}

#[test]
fn derive_fire_targets_clamps_and_scales_per_node_oracle() {
    // Four rate regimes over a 2s warm-up / 30s cap under the default 1/2
    // policy. Hand-computed per node (total_expected = fires * cap / warmup,
    // then ÷2, then clamp into [20, 1000]):
    //   hz30 : 60 fires  -> 60*30/2  = 900   -> 450  (in range)
    //   hz1  :  2 fires  ->  2*30/2  =  30   ->  15  -> clamps UP to min 20
    //   khz1 : 2000 fires-> 2000*30/2= 30000 -> 15000-> clamps DOWN to max 1000
    //   silent: 0 fires  -> NO entry (no rate to project from)
    let warmup = fires(&[("hz30", 60), ("hz1", 2), ("silent", 0), ("khz1", 2000)]);
    let warmup_ns = 2 * ONE_SEC_NS;
    let cap_ns = 30 * ONE_SEC_NS;
    let targets = derive_fire_targets(&warmup, warmup_ns, cap_ns, &FireTargetPolicy::default());

    // Full-map equality against a hand-built oracle also pins BTree determinism.
    let oracle: BTreeMap<String, u64> = [
        ("hz30".to_string(), 450),
        ("hz1".to_string(), 20),
        ("khz1".to_string(), 1000),
        // "silent" is deliberately ABSENT.
    ]
    .into_iter()
    .collect();
    assert_eq!(
        targets, oracle,
        "per-node derived targets must equal the hand oracle (silent node absent)"
    );
    assert!(
        !targets.contains_key("silent"),
        "a node with zero warm-up fires gets NO target entry (silent contract)"
    );
}

#[test]
fn derive_fire_targets_zero_warmup_window_yields_empty_no_div_by_zero() {
    // window_ns == 0 is degenerate (no rate information) — must return an empty
    // map, never divide by zero. p50-style totality guard.
    let warmup = fires(&[("n0", 100), ("n1", 200)]);
    let targets = derive_fire_targets(&warmup, 0, 30 * ONE_SEC_NS, &FireTargetPolicy::default());
    assert!(
        targets.is_empty(),
        "a zero-length warm-up window derives NO targets; got {targets:?}"
    );
}

#[test]
fn derive_fire_targets_zero_denominator_yields_empty_no_div_by_zero() {
    // A degenerate policy (fraction_den == 0, reachable only via a struct
    // literal that bypasses `FireTargetPolicy::new`) must not divide by zero —
    // it is treated as no-derivation (empty map).
    let warmup = fires(&[("n0", 100)]);
    let bad = FireTargetPolicy {
        min_samples: 20,
        max_samples: 1000,
        fraction_num: 1,
        fraction_den: 0,
    };
    let targets = derive_fire_targets(&warmup, 2 * ONE_SEC_NS, 30 * ONE_SEC_NS, &bad);
    assert!(
        targets.is_empty(),
        "a zero denominator derives NO targets (no div-by-zero); got {targets:?}"
    );
}

#[test]
fn derive_fire_targets_two_horizon_oracle() {
    // The SAME warm-up observation projected onto TWO horizons
    // yields DIFFERENT hand-computed maps — the cap horizon feeds the
    // profiler's STOP GATE, the actual-window horizon feeds its HARVEST gate.
    // Catches the two call sites being re-unified back onto one horizon.
    // Warm-up = 2 s; horizons = 30 s (cap) vs 10 s (window).
    //   hz30 (60 fires): cap 60*30/2=900 -> 450 | window 60*10/2=300 -> 150
    //   hz1  ( 2 fires): cap  2*30/2= 30 ->  15 -> clamp 20
    //                    window 2*10/2= 10 ->  5 -> clamp 20   (floor equalizes)
    //   khz1 (2000):     cap 30000 -> 15000 -> clamp 1000
    //                    window 10000 -> 5000 -> clamp 1000    (ceiling equalizes)
    let warmup = fires(&[("hz30", 60), ("hz1", 2), ("khz1", 2000)]);
    let warmup_ns = 2 * ONE_SEC_NS;
    let policy = FireTargetPolicy::default();

    let cap_horizon = derive_fire_targets(&warmup, warmup_ns, 30 * ONE_SEC_NS, &policy);
    let window_horizon = derive_fire_targets(&warmup, warmup_ns, 10 * ONE_SEC_NS, &policy);

    let cap_oracle: BTreeMap<String, u64> = [
        ("hz30".to_string(), 450),
        ("hz1".to_string(), 20),
        ("khz1".to_string(), 1000),
    ]
    .into_iter()
    .collect();
    let window_oracle: BTreeMap<String, u64> = [
        ("hz30".to_string(), 150),
        ("hz1".to_string(), 20),
        ("khz1".to_string(), 1000),
    ]
    .into_iter()
    .collect();
    assert_eq!(cap_horizon, cap_oracle, "cap-horizon map == hand oracle");
    assert_eq!(
        window_horizon, window_oracle,
        "window-horizon map == hand oracle"
    );
    assert_ne!(
        cap_horizon, window_horizon,
        "the two horizons MUST differ on the mid-rate node (a re-unified \
         single-horizon implementation cannot produce both maps)"
    );
    // The a-fortiori direction the harvest gate relies on: window <= cap
    // => every window-projected target <= its cap-projected twin.
    for (node, &window_target) in &window_horizon {
        assert!(
            window_target <= cap_horizon[node],
            "window-horizon target must be <= cap-horizon for '{node}'"
        );
    }
}

#[test]
fn derive_fire_targets_degenerate_horizon_equals_clamped_half_fires() {
    // The NO-WARM-UP fallback shape: horizon == warm-up window
    // (the whole observed window IS the observation) degenerates to
    // `target = clamp(fires/2, 20, 1000)`. Exact-integer vector:
    //   100 fires -> 50 | 10 -> 5 -> clamp 20 | 39 -> 19 -> clamp 20
    //   4000 -> 2000 -> clamp 1000 | 0 -> ABSENT
    let w = 5 * ONE_SEC_NS;
    let observed = fires(&[
        ("mid", 100),
        ("tiny", 10),
        ("edge", 39),
        ("flood", 4000),
        ("silent", 0),
    ]);
    let targets = derive_fire_targets(&observed, w, w, &FireTargetPolicy::default());
    let oracle: BTreeMap<String, u64> = [
        ("mid".to_string(), 50),
        ("tiny".to_string(), 20),
        ("edge".to_string(), 20),
        ("flood".to_string(), 1000),
        // "silent" deliberately ABSENT.
    ]
    .into_iter()
    .collect();
    assert_eq!(
        targets, oracle,
        "horizon == window degenerates to clamp(fires/2, 20, 1000), exactly"
    );
}

// ==========================================================================
// PER-NODE warm-up windows, anchored at each
// node's first fire (`derive_fire_targets_from_observations` +
// `WarmupObservation`). Hand-computed integer oracles throughout; the shared
// -window `derive_fire_targets` arms above stay the pin for the delegation.
// ==========================================================================

/// Build a per-node observation map from `(node, fires, window_ns)` triples.
fn obs(triples: &[(&str, u64, u64)]) -> BTreeMap<String, WarmupObservation> {
    triples
        .iter()
        .map(|&(node, fires, window_ns)| (node.to_string(), WarmupObservation { fires, window_ns }))
        .collect()
}

#[test]
fn per_node_windows_project_per_node_targets_a_shared_window_cannot() {
    // THE motivating shape, as arithmetic: three nodes with the SAME active fire
    // count over THREE DIFFERENT active windows must derive THREE different
    // targets. A shared-window implementation has one window to divide by, so
    // it cannot produce this map at all — which is what makes the assertion a
    // discriminator rather than a restatement.
    //
    // Horizon 30 s, default policy (÷2, clamp [20, 1000]):
    //   early : 20 fires over 1.0 s -> 20*30/1   = 600 -> 300
    //   mid   : 20 fires over 0.5 s -> 20*30/0.5 = 1200 -> 600
    //   late  : 20 fires over 0.1 s -> 20*30/0.1 = 6000 -> 3000 -> clamp 1000
    let observations = obs(&[
        ("early", 20, ONE_SEC_NS),
        ("mid", 20, ONE_SEC_NS / 2),
        ("late", 20, ONE_SEC_NS / 10),
    ]);
    let targets = derive_fire_targets_from_observations(
        &observations,
        30 * ONE_SEC_NS,
        &FireTargetPolicy::default(),
    );
    let oracle: BTreeMap<String, u64> = [
        ("early".to_string(), 300),
        ("mid".to_string(), 600),
        ("late".to_string(), 1000),
    ]
    .into_iter()
    .collect();
    assert_eq!(
        targets, oracle,
        "each node's target must be projected from ITS OWN window"
    );
}

#[test]
fn a_node_that_fired_but_produced_no_sustained_sample_is_floored_never_dropped() {
    // The regression this rule exists to prevent: anchoring at the first fire
    // leaves a node slower than the warm-up with ZERO fires inside its own
    // active window (a 1 Hz node's first fire lands 200 ms into a 1 s warm-up,
    // so 800 ms follow with nothing further). It DID fire — membership is that
    // evidence — so it must be FLOORED to `min_samples`, never dropped into the
    // "silent through warm-up" isolation the never-firing contract reserves.
    //
    // `boundary` additionally pins the zero-WINDOW arm (first fire seen in the
    // warm-up-end poll itself): no rate, no div-by-zero, same floor.
    let observations = obs(&[
        ("slow", 0, 800_000_000),
        ("boundary", 0, 0),
        ("healthy", 20, ONE_SEC_NS),
    ]);
    let policy = FireTargetPolicy::default();
    let targets = derive_fire_targets_from_observations(&observations, 30 * ONE_SEC_NS, &policy);
    let oracle: BTreeMap<String, u64> = [
        ("slow".to_string(), policy.min_samples),
        ("boundary".to_string(), policy.min_samples),
        // 20 fires over 1 s projected to 30 s = 600, ÷2 = 300.
        ("healthy".to_string(), 300),
    ]
    .into_iter()
    .collect();
    assert_eq!(
        targets, oracle,
        "a node that fired is FLOORED, never absent; got {targets:?}"
    );
    // The silent contract is the OTHER side of the same rule: a node absent
    // from the observations stays absent from the targets (no fabricated
    // number), which is what `harvest_costs` reads as "silent through warm-up".
    assert!(
        !targets.contains_key("never_fired"),
        "a node with NO observation derives NO target (the silent contract)"
    );
}

#[test]
fn excluding_the_bring_up_burst_makes_the_target_reachable() {
    // The defect and its remedy, from measured numbers, in
    // ONE body — so the remedy is asserted against the measurement rather than
    // against itself. Measured on a CI run: a `period_ms = 50` (20 Hz) node
    // over a 6.022 s window, warm-up 1 s, bring-up debt ~38 fires.
    //
    // SHARED window (burst counted as rate): 58 fires over 1 s.
    // ANCHORED at the first fire (burst excluded): 20 fires over 1 s.
    const WINDOW_NS: u64 = 6_022_000_000;
    const DELIVERED: u64 = 160; // fires the node actually managed (burst included)
    let policy = FireTargetPolicy::default();

    let shared = derive_fire_targets(&fires(&[("ticker", 58)]), ONE_SEC_NS, WINDOW_NS, &policy);
    let anchored = derive_fire_targets_from_observations(
        &obs(&[("ticker", 20, ONE_SEC_NS)]),
        WINDOW_NS,
        &policy,
    );

    // 58 * 6.022 / 1 = 349 -> 174 : ABOVE the 160 the node delivered.
    assert_eq!(
        shared["ticker"], 174,
        "the burst-inflated target (hand oracle)"
    );
    assert!(
        DELIVERED < shared["ticker"],
        "the defect: a node firing at exactly its declared period falls \
         short of a target derived from its bring-up burst"
    );
    // 20 * 6.022 / 1 = 120 -> 60 : reachable, with room to spare.
    assert_eq!(
        anchored["ticker"], 60,
        "the sustained-rate target (hand oracle)"
    );
    assert!(
        DELIVERED >= anchored["ticker"],
        "with the burst excluded the node meets its target and is COSTED"
    );
}

#[test]
fn the_shared_window_form_delegates_to_the_per_node_form() {
    // `derive_fire_targets` is the no-warm-up fallback's entry point and must
    // stay byte-identical to a per-node call with ONE uniform window — that is
    // what keeps a single implementation of the clamp. The zero-fire node is
    // the discriminator: the shared form filters it out (never fired), so the
    // equality only holds if the wrapper does that filtering rather than
    // handing a zero-fire observation down (which would FLOOR it to 20).
    let policy = FireTargetPolicy::default();
    let warmup_ns = 2 * ONE_SEC_NS;
    let horizon_ns = 30 * ONE_SEC_NS;
    let shared = derive_fire_targets(
        &fires(&[("hz30", 60), ("hz1", 2), ("silent", 0)]),
        warmup_ns,
        horizon_ns,
        &policy,
    );
    let per_node = derive_fire_targets_from_observations(
        &obs(&[("hz30", 60, warmup_ns), ("hz1", 2, warmup_ns)]),
        horizon_ns,
        &policy,
    );
    assert_eq!(
        shared, per_node,
        "the shared-window form must equal the per-node form over one window"
    );
    assert!(
        !shared.contains_key("silent"),
        "the wrapper filters a never-fired node BEFORE delegating; got {shared:?}"
    );
}

#[test]
fn per_node_derivation_zero_denominator_yields_empty_no_div_by_zero() {
    // The degenerate-policy guard moved down with the arithmetic, so it is
    // pinned where it now lives (its shared-window twin above pins the wrapper).
    let bad = FireTargetPolicy {
        min_samples: 20,
        max_samples: 1000,
        fraction_num: 1,
        fraction_den: 0,
    };
    let targets = derive_fire_targets_from_observations(
        &obs(&[("n0", 100, ONE_SEC_NS)]),
        30 * ONE_SEC_NS,
        &bad,
    );
    assert!(
        targets.is_empty(),
        "a zero denominator derives NO targets (no div-by-zero); got {targets:?}"
    );
}

// ==========================================================================
// The core-count-derived DEFAULT budget (`derive_default_budget_ns`)
// — ceil-div oracle vectors + two end-to-end auto_partition pins showing the
// derived budget actually SHAPES the grouping. Hand-computed integers only.
// ==========================================================================

#[test]
fn derive_default_budget_ceil_div_oracle() {
    let cores = |n: usize| NonZeroUsize::new(n).expect("nonzero");
    assert_eq!(
        derive_default_budget_ns(1000, cores(4)),
        250,
        "even split: 1000 / 4 = 250"
    );
    assert_eq!(
        derive_default_budget_ns(1001, cores(4)),
        251,
        "CEIL division: 1001 / 4 rounds UP to 251 (a floor-div mutant gives 250)"
    );
    assert_eq!(
        derive_default_budget_ns(1000, cores(1)),
        1000,
        "one core: the budget is the whole total (maximal fusion allowed)"
    );
    assert_eq!(
        derive_default_budget_ns(0, cores(4)),
        1,
        "zero total floors at 1 (a 0 budget is rejected downstream; callers \
         avoid freezing this case at all)"
    );
}

#[test]
fn derived_budget_cores_eq_node_count_yields_process_per_node() {
    // 3-chain, 100 ns per node (total 300), hot profitable edges. With
    // cores == node count the derived budget is exactly ONE node's cost
    // (ceil(300/3) = 100), so ANY two-node fusion (load 200) blows the
    // budget => the grouping stays process-per-node. Pins that the derived
    // default is a REAL constraint, not a pass-through.
    let (config, t, ids, edges) = chain(3);
    let cst = chain_costs(&edges, &ids, 100, 10_000);
    let budget = derive_default_budget_ns(300, NonZeroUsize::new(3).expect("nonzero"));
    assert_eq!(budget, 100, "hand check: ceil(300/3) = 100");

    let part =
        auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), budget).expect("partition");
    assert_eq!(
        shape(&part),
        expect(&[
            ("grp_n0", &["n0"]),
            ("grp_n1", &["n1"]),
            ("grp_n2", &["n2"]),
        ]),
        "a one-node-sized budget forbids every fusion => process-per-node"
    );
    assert!(part.fusions.is_empty(), "no fusion fits the budget");
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

#[test]
fn derived_budget_single_core_yields_maximal_fusion() {
    // Same chain, cores == 1: the derived budget equals the TOTAL compute
    // (ceil(300/1) = 300), so the whole profitable chain fuses into one
    // group — the single-core box has nothing to balance across.
    let (config, t, ids, edges) = chain(3);
    let cst = chain_costs(&edges, &ids, 100, 10_000);
    let budget = derive_default_budget_ns(300, NonZeroUsize::new(1).expect("nonzero"));
    assert_eq!(budget, 300, "hand check: ceil(300/1) = 300 = the total");

    let part =
        auto_partition(&config, &infos(&config), &t, &cst, &iso_none(), budget).expect("partition");
    assert_eq!(
        shape(&part),
        expect(&[("grp_n0", &["n0", "n1", "n2"])]),
        "a total-sized budget admits the maximal profitable fusion"
    );
    validate_partition(&part.groups, &config, &infos(&config), &t).expect("valid");
}

#[test]
fn harvest_per_node_targets_discriminate_same_fire_count() {
    // Two nodes with the SAME fire count (10) but DIFFERENT per-node targets:
    // n0's target 5 is met (10 >= 5, KEPT) while n1's target 20 is not
    // (10 < 20, ISOLATED). A single SCALAR gate cannot produce this split —
    // this catches a revert back to a scalar gate.
    let (config, t, _ids, _edges) = chain(2);
    let trace = vec![te("n0", 42), te("n1", 99)];
    let fc = fires(&[("n0", 10), ("n1", 10)]);
    let targets: BTreeMap<String, u64> = [("n0".to_string(), 5), ("n1".to_string(), 20)]
        .into_iter()
        .collect();
    let profile = harvest_costs(
        &config,
        &infos(&config),
        &t,
        &trace,
        &fc,
        ONE_SEC_NS,
        hop(),
        &targets,
    )
    .expect("harvest");
    assert_eq!(
        profile.costs.node_p50_ns,
        [("n0".to_string(), 42)].into_iter().collect(),
        "n0 (10 >= its target 5) is costed"
    );
    assert_eq!(
        profile.isolated,
        ["n1".to_string()].into_iter().collect::<BTreeSet<_>>(),
        "n1 (10 < its target 20) is ISOLATED despite the SAME fire count as n0"
    );
}

#[test]
fn harvest_per_node_boundary_at_target_kept_one_below_isolated() {
    // The `<` vs `<=` off-by-one pin, per-node: n0 count == its target (KEPT);
    // n1 count == its target - 1 (ISOLATED). Using `<=` would isolate n0 too.
    let (config, t, _ids, _edges) = chain(2);
    let trace = vec![te("n0", 7), te("n1", 8)];
    let fc = fires(&[("n0", 10), ("n1", 9)]);
    let targets: BTreeMap<String, u64> = [("n0".to_string(), 10), ("n1".to_string(), 10)]
        .into_iter()
        .collect();
    let profile = harvest_costs(
        &config,
        &infos(&config),
        &t,
        &trace,
        &fc,
        ONE_SEC_NS,
        hop(),
        &targets,
    )
    .expect("harvest");
    assert_eq!(
        profile.costs.node_p50_ns,
        [("n0".to_string(), 7)].into_iter().collect(),
        "n0 at EXACTLY its target is kept (count == target, boundary stays `<`)"
    );
    assert_eq!(
        profile.isolated,
        ["n1".to_string()].into_iter().collect::<BTreeSet<_>>(),
        "n1 one BELOW its target is isolated"
    );
}

#[test]
fn harvest_node_absent_from_targets_is_isolated() {
    // A node with fires recorded but NO entry in the target map is under-sampled
    // (unknown target ⇒ treat as unmet, the conservative choice) ⇒ ISOLATED.
    // Anti-tautology: the sibling n0 (present, target met) IS costed.
    let (config, t, _ids, _edges) = chain(2);
    let trace = vec![te("n0", 11), te("n1", 22)];
    let fc = fires(&[("n0", 10), ("n1", 10)]);
    // n1 is deliberately ABSENT from the target map.
    let targets: BTreeMap<String, u64> = [("n0".to_string(), 5)].into_iter().collect();
    let profile = harvest_costs(
        &config,
        &infos(&config),
        &t,
        &trace,
        &fc,
        ONE_SEC_NS,
        hop(),
        &targets,
    )
    .expect("harvest");
    assert_eq!(
        profile.costs.node_p50_ns,
        [("n0".to_string(), 11)].into_iter().collect(),
        "n0 (present, 10 >= target 5) is costed"
    );
    assert_eq!(
        profile.isolated,
        ["n1".to_string()].into_iter().collect::<BTreeSet<_>>(),
        "n1 absent from the target map is ISOLATED"
    );
}

// --------------------------------------------------------------------------
// `baseline_process_per_node`: the no-costs emit fallback.
// --------------------------------------------------------------------------

#[test]
fn baseline_process_per_node_matches_partitioner_baseline_and_oracle() {
    // A 3-node chain n0->n1->n2 plus a disconnected `solo`.
    let config = config_of(vec![
        node("n0", &[], &["out"]),
        node("n1", &[("in", "n0/out")], &["out"]),
        node("n2", &[("in", "n1/out")], &[]),
        node("solo", &[], &[]),
    ]);
    let t = triggers(&[("n1", "n0/out"), ("n2", "n1/out")]);

    let baseline = baseline_process_per_node(&config, &infos(&config), &t).expect("baseline");
    let baseline_shape: Vec<(String, Vec<String>)> = baseline
        .iter()
        .map(|(k, v)| (k.clone(), v.clone()))
        .collect();

    // Hand oracle: every node in its own grp_<node>, in pipeline order
    // (n0 @L0, solo @L0 graph-order-after, n1 @L1, n2 @L2).
    assert_eq!(
        baseline_shape,
        expect(&[
            ("grp_n0", &["n0"]),
            ("grp_solo", &["solo"]),
            ("grp_n1", &["n1"]),
            ("grp_n2", &["n2"]),
        ]),
        "process-per-node, named + ordered by the partitioner"
    );

    // "Reuse, don't duplicate" contract: with an UNPROFITABLE cost snapshot
    // (rate 0 ⇒ coupling 0), auto_partition fuses nothing and must land on the
    // IDENTICAL shape — baseline IS the partitioner's own starting point (this
    // cross-check is an independent path + a hand oracle, not a self-compare).
    let unprofitable = costs(
        &["n0", "n1", "n2", "solo"],
        &[("n0", "n1"), ("n1", "n2")],
        100,
        0,
    );
    let via_auto = auto_partition(
        &config,
        &infos(&config),
        &t,
        &unprofitable,
        &iso_none(),
        100_000,
    )
    .expect("auto");
    assert_eq!(
        shape(&via_auto),
        baseline_shape,
        "baseline_process_per_node must equal auto_partition's no-fusion result"
    );
}
