// SPDX-License-Identifier: AGPL-3.0-only
//! Spine dep (c): cross-process partition derivation + validation.
//!
//! Pure derivation/validation — NO iceoryx2, so parallel-safe (no `#[serial]`).
//! The headline oracle (`derive_matches_barrier_level_gate_oracle`) reproduces
//! the EXACT `MAP_A`/`MAP_B` participant-maps hand-pasted in
//! `barrier_level_gate_iox2_test.rs` for the 5-node single chain partitioned
//! `{P0:[n0,n1], P1:[n2,n3,n4]}`, so the derivation is tied to a hand-computed
//! oracle, not a self-compare.

use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::partition::{
    derive_process_groups, validate_process_groups, ProcessGroup,
};
use cerulion_core::graph::topology::{GraphTopology, Levels, TriggerEdges};
use indexmap::IndexMap;

const PREFIX: &str = "p";

/// One node: id, (input_name, source) pairs, output names.
fn node(id: &str, inputs: &[(&str, &str)], outputs: &[&str]) -> NodeDef {
    NodeDef {
        fuse: None,
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

/// A 5-node single chain n0 -> n1 -> n2 -> n3 -> n4, with the given
/// `process_groups`. NodeInfo is empty (topology consumer edges come from the
/// YAML `inputs`, not from macro metadata).
fn chain_config(process_groups: IndexMap<String, Vec<String>>) -> GraphConfig {
    GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        name: None,
        identity: "chain".to_string(),
        prefix: PREFIX.to_string(),
        nodes: vec![
            node("n0", &[], &["out"]),
            node("n1", &[("inp", "n0/out")], &["out"]),
            node("n2", &[("inp", "n1/out")], &["out"]),
            node("n3", &[("inp", "n2/out")], &["out"]),
            node("n4", &[("inp", "n3/out")], &[]),
        ],
        multi_publisher_topics: Vec::new(),
        process_groups,
        process_group_order: Vec::new(),
    }
}

/// Reproduce the runtime's GLOBAL Kahn levelization for the chain: each
/// consumer edge is a TRIGGERING edge (mirrors a DataTrigger chain), so the
/// levels are n0=0, n1=1, n2=2, n3=3, n4=4.
fn chain_levels(config: &GraphConfig) -> Levels {
    let entry_infos: IndexMap<String, cerulion_core::graph::NodeInfo> = config
        .nodes
        .iter()
        .map(|n| {
            (
                n.id.clone(),
                cerulion_core::graph::NodeInfo::with_meta(Vec::new(), Vec::new()),
            )
        })
        .collect();
    let topo = GraphTopology::build(config, &entry_infos).expect("topology build");
    let mut edges = TriggerEdges::new();
    // (consumer, resolved-producer-topic) for each chain hop.
    edges.insert("n1", "/p/n0/out");
    edges.insert("n2", "/p/n1/out");
    edges.insert("n3", "/p/n2/out");
    edges.insert("n4", "/p/n3/out");
    topo.derive_levels(&edges).expect("derive levels")
}

fn groups(pairs: &[(&str, &[&str])]) -> IndexMap<String, Vec<String>> {
    pairs
        .iter()
        .map(|(name, members)| {
            (
                name.to_string(),
                members.iter().map(|s| s.to_string()).collect(),
            )
        })
        .collect()
}

/// Build the GLOBAL Kahn levelization for `config`, treating each listed
/// `(consumer, resolved-topic)` pair as a TRIGGERING edge (mirrors a
/// DataTrigger chain). Generalizes `chain_levels` to arbitrary topologies
/// (the diamond + the smaller-config mismatch test reuse it).
fn levels_for(config: &GraphConfig, trigger_edges: &[(&str, &str)]) -> Levels {
    let entry_infos: IndexMap<String, cerulion_core::graph::NodeInfo> = config
        .nodes
        .iter()
        .map(|n| {
            (
                n.id.clone(),
                cerulion_core::graph::NodeInfo::with_meta(Vec::new(), Vec::new()),
            )
        })
        .collect();
    let topo = GraphTopology::build(config, &entry_infos).expect("topology build");
    let mut edges = TriggerEdges::new();
    for (consumer, topic) in trigger_edges {
        edges.insert(*consumer, *topic);
    }
    topo.derive_levels(&edges).expect("derive levels")
}

/// A 4-node DIAMOND `n0 -> {n1, n2} -> n3`: n1 AND n2 both consume n0 (both land
/// at global level 1 — a WIDE level the linear `chain_config` cannot produce),
/// and n3 consumes BOTH n1 and n2 (level 2). 3 global levels total.
fn diamond_config(process_groups: IndexMap<String, Vec<String>>) -> GraphConfig {
    GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        name: None,
        identity: "diamond".to_string(),
        prefix: PREFIX.to_string(),
        nodes: vec![
            node("n0", &[], &["out"]),
            node("n1", &[("inp", "n0/out")], &["out"]),
            node("n2", &[("inp", "n0/out")], &["out"]),
            node("n3", &[("a", "n1/out"), ("b", "n2/out")], &[]),
        ],
        multi_publisher_topics: Vec::new(),
        process_groups,
        process_group_order: Vec::new(),
    }
}

/// Levelize the diamond: n0=0, n1=1, n2=1, n3=2 (3 levels). Both fan-out edges
/// from n0 trigger; n3's two fan-in edges both trigger.
fn diamond_levels(config: &GraphConfig) -> Levels {
    levels_for(
        config,
        &[
            ("n1", "/p/n0/out"),
            ("n2", "/p/n0/out"),
            ("n3", "/p/n1/out"),
            ("n3", "/p/n2/out"),
        ],
    )
}

#[test]
fn derive_matches_barrier_level_gate_oracle() {
    // The EXACT oracle hand-pasted in barrier_level_gate_iox2_test.rs:
    //   MAP_A = [Some(0), Some(1), None, None, None]  (P0 owns globals 0,1)
    //   MAP_B = [None, None, Some(0), Some(1), Some(2)] (P1 owns globals 2,3,4)
    let config = chain_config(groups(&[
        ("P0", &["n0", "n1"]),
        ("P1", &["n2", "n3", "n4"]),
    ]));
    // Sanity: the levelization is the 5-level single chain the oracle assumes.
    let levels = chain_levels(&config);
    assert_eq!(levels.len(), 5, "chain must levelize to 5 global levels");
    assert_eq!(levels.level_of("n0"), Some(0));
    assert_eq!(levels.level_of("n4"), Some(4));

    let derived = derive_process_groups(&config, &levels).expect("derive");
    let expected = vec![
        ProcessGroup {
            name: "P0".to_string(),
            rank: 0,
            global_level_map: vec![Some(0), Some(1), None, None, None],
        },
        ProcessGroup {
            name: "P1".to_string(),
            rank: 1,
            global_level_map: vec![None, None, Some(0), Some(1), Some(2)],
        },
    ];
    assert_eq!(
        derived, expected,
        "derivation must match the MAP_A/MAP_B oracle"
    );
}

#[test]
fn rank_is_declaration_order_by_default() {
    // Semantic group names where LISTING order != ALPHABETICAL order: list
    // `zeta` BEFORE `alpha`. With declaration-order ranking, zeta=0/alpha=1.
    // A (deleted) name-sort would have flipped this to alpha=0/zeta=1, so the
    // assertion PROVES rank follows the listing, not the name.
    let config = chain_config(groups(&[
        ("zeta", &["n0", "n1"]),
        ("alpha", &["n2", "n3", "n4"]),
    ]));
    let levels = chain_levels(&config);
    let derived = derive_process_groups(&config, &levels).expect("derive");

    let names: Vec<&str> = derived.iter().map(|p| p.name.as_str()).collect();
    let ranks: Vec<usize> = derived.iter().map(|p| p.rank).collect();
    assert_eq!(
        names,
        vec!["zeta", "alpha"],
        "rank order is the LISTING order, not alphabetical (alpha<zeta)"
    );
    assert_eq!(ranks, vec![0, 1], "rank is the position in the listing");

    // zeta (rank 0) owns globals 0,1; alpha (rank 1) owns globals 2,3,4.
    assert_eq!(
        derived[0].global_level_map,
        vec![Some(0), Some(1), None, None, None]
    );
    assert_eq!(
        derived[1].global_level_map,
        vec![None, None, Some(0), Some(1), Some(2)]
    );
}

#[test]
fn process_group_order_overrides_rank() {
    // Same groups listed zeta-then-alpha, but an explicit `process_group_order`
    // REVERSES the rank to [alpha, zeta]. The derived ranks + maps must follow
    // the override, not the listing order.
    let mut config = chain_config(groups(&[
        ("zeta", &["n0", "n1"]),
        ("alpha", &["n2", "n3", "n4"]),
    ]));
    config.process_group_order = vec!["alpha".to_string(), "zeta".to_string()];
    let levels = chain_levels(&config);
    let derived = derive_process_groups(&config, &levels).expect("derive");

    let names: Vec<&str> = derived.iter().map(|p| p.name.as_str()).collect();
    let ranks: Vec<usize> = derived.iter().map(|p| p.rank).collect();
    assert_eq!(
        names,
        vec!["alpha", "zeta"],
        "explicit process_group_order overrides the listing order"
    );
    assert_eq!(
        ranks,
        vec![0, 1],
        "rank is the position in the override list"
    );

    // alpha (now rank 0) owns globals 2,3,4; zeta (now rank 1) owns globals 0,1.
    assert_eq!(
        derived[0].global_level_map,
        vec![None, None, Some(0), Some(1), Some(2)],
        "alpha (rank 0) owns globals 2,3,4"
    );
    assert_eq!(
        derived[1].global_level_map,
        vec![Some(0), Some(1), None, None, None],
        "zeta (rank 1) owns globals 0,1"
    );
}

#[test]
fn process_group_order_rejects_missing_extra_duplicate() {
    // MISSING: a declared group never named in the order list.
    let mut config = chain_config(groups(&[
        ("zeta", &["n0", "n1"]),
        ("alpha", &["n2", "n3", "n4"]),
    ]));
    config.process_group_order = vec!["zeta".to_string()];
    let err = validate_process_groups(&config)
        .expect_err("an order list missing a declared group must be rejected")
        .to_string();
    assert!(
        err.contains("alpha") && err.contains("missing from process_group_order"),
        "missing-group error must name the absent group; got: {err}"
    );

    // UNKNOWN/EXTRA: the order list names a group that is not declared.
    let mut config = chain_config(groups(&[
        ("zeta", &["n0", "n1"]),
        ("alpha", &["n2", "n3", "n4"]),
    ]));
    config.process_group_order = vec!["zeta".to_string(), "alpha".to_string(), "gamma".to_string()];
    let err = validate_process_groups(&config)
        .expect_err("an order list naming an unknown group must be rejected")
        .to_string();
    assert!(
        err.contains("gamma") && err.contains("not a declared group"),
        "unknown-group error must name the bad entry; got: {err}"
    );

    // DUPLICATE: a name appears twice in the order list.
    let mut config = chain_config(groups(&[
        ("zeta", &["n0", "n1"]),
        ("alpha", &["n2", "n3", "n4"]),
    ]));
    config.process_group_order = vec!["zeta".to_string(), "zeta".to_string(), "alpha".to_string()];
    let err = validate_process_groups(&config)
        .expect_err("an order list with a duplicate name must be rejected")
        .to_string();
    assert!(
        err.contains("zeta") && err.contains("more than once"),
        "duplicate error must name the repeated group; got: {err}"
    );
}

#[test]
fn mutation_swapping_groups_changes_maps() {
    // Move n2 from P1 to P0 → P0 gains global level 2, P1 loses it. Proves the
    // derivation is LOAD-BEARING (reads the actual level placement), not a
    // constant.
    let config = chain_config(groups(&[
        ("P0", &["n0", "n1", "n2"]),
        ("P1", &["n3", "n4"]),
    ]));
    let levels = chain_levels(&config);
    let derived = derive_process_groups(&config, &levels).expect("derive");

    assert_eq!(
        derived[0].global_level_map,
        vec![Some(0), Some(1), Some(2), None, None],
        "P0 now owns globals 0,1,2 (n2 moved in)"
    );
    assert_eq!(
        derived[1].global_level_map,
        vec![None, None, None, Some(0), Some(1)],
        "P1 now owns only globals 3,4 (n2 moved out)"
    );
}

#[test]
fn empty_process_groups_is_ok() {
    // Absent ⇒ single-process monolith ⇒ validation is a no-op Ok.
    let config = chain_config(IndexMap::new());
    assert!(!config.has_process_groups());
    validate_process_groups(&config).expect("empty process_groups must validate Ok");
}

#[test]
fn orphan_node_rejected() {
    // n4 is in no group → orphan.
    let config = chain_config(groups(&[("P0", &["n0", "n1"]), ("P1", &["n2", "n3"])]));
    let err = validate_process_groups(&config)
        .expect_err("an unlisted node must be rejected")
        .to_string();
    assert!(
        err.contains("n4") && err.contains("not assigned to any process group"),
        "orphan error must name the node + the rule; got: {err}"
    );
}

#[test]
fn node_in_two_groups_rejected() {
    // n1 listed in BOTH P0 and P1.
    let config = chain_config(groups(&[
        ("P0", &["n0", "n1"]),
        ("P1", &["n1", "n2", "n3", "n4"]),
    ]));
    let err = validate_process_groups(&config)
        .expect_err("a node in two groups must be rejected")
        .to_string();
    assert!(
        err.contains("n1") && err.contains("more than one process group"),
        "double-assignment error must name the node + both groups; got: {err}"
    );
    assert!(
        err.contains("P0") && err.contains("P1"),
        "error must name both colliding groups; got: {err}"
    );
}

#[test]
fn dangling_node_ref_rejected() {
    // P1 lists "ghost", which is not a declared node.
    let config = chain_config(groups(&[
        ("P0", &["n0", "n1"]),
        ("P1", &["n2", "n3", "n4", "ghost"]),
    ]));
    let err = validate_process_groups(&config)
        .expect_err("a dangling node reference must be rejected")
        .to_string();
    assert!(
        err.contains("ghost") && err.contains("dangling reference"),
        "dangling error must name the bad ref; got: {err}"
    );
}

#[test]
fn derive_wide_level_one_local_index() {
    // The HIGH gap: a group owning TWO nodes at the SAME global level must get
    // EXACTLY ONE local index there. Over the diamond, P1 owns n1 AND n2 (both
    // global level 1); its map must be [None, Some(0), None] — proving the
    // `owned_levels` HashSet dedups same-level nodes into one local index. A
    // per-node Vec/counter regression would emit two Somes (and break the
    // bijection onto 0..local_count).
    let config = diamond_config(groups(&[
        ("P0", &["n0"]),
        ("P1", &["n1", "n2"]),
        ("P2", &["n3"]),
    ]));
    let levels = diamond_levels(&config);
    assert_eq!(levels.len(), 3, "diamond levelizes to 3 global levels");
    assert_eq!(levels.level_of("n0"), Some(0));
    assert_eq!(levels.level_of("n1"), Some(1));
    assert_eq!(levels.level_of("n2"), Some(1));
    assert_eq!(levels.level_of("n3"), Some(2));

    let derived = derive_process_groups(&config, &levels).expect("derive");
    assert_eq!(
        derived[0].global_level_map,
        vec![Some(0), None, None],
        "P0 owns only global level 0"
    );
    assert_eq!(
        derived[1].global_level_map,
        vec![None, Some(0), None],
        "P1 owns n1 AND n2 at global level 1 → EXACTLY one local index there"
    );
    assert_eq!(
        derived[2].global_level_map,
        vec![None, None, Some(0)],
        "P2 owns only global level 2"
    );
}

#[test]
fn derive_non_contiguous_owned_levels() {
    // A group owning NON-contiguous global levels: on the 5-node chain,
    // P0 owns n0,n2,n4 (globals 0,2,4) and P1 owns n1,n3 (globals 1,3). The
    // `next_local` gap-skip must assign local indices 0,1,2 ONLY at the owned
    // globals, `None` between them.
    let config = chain_config(groups(&[
        ("P0", &["n0", "n2", "n4"]),
        ("P1", &["n1", "n3"]),
    ]));
    let levels = chain_levels(&config);
    let derived = derive_process_groups(&config, &levels).expect("derive");

    assert_eq!(
        derived[0].global_level_map,
        vec![Some(0), None, Some(1), None, Some(2)],
        "P0 owns globals 0,2,4 → locals 0,1,2 with None gaps"
    );
    assert_eq!(
        derived[1].global_level_map,
        vec![None, Some(0), None, Some(1), None],
        "P1 owns globals 1,3 → locals 0,1 with None gaps"
    );
}

#[test]
fn validate_graph_rejects_bad_partition() {
    // Integration pin: an otherwise-valid graph (valid prefix/topology) whose
    // ONLY defect is an orphan-node partition (n4 in no group) must be rejected
    // by `validate_graph` itself — proving `validate_graph` calls
    // `validate_process_groups`. Deleting that call would leave this green only
    // if the call is absent.
    let config = chain_config(groups(&[("P0", &["n0", "n1"]), ("P1", &["n2", "n3"])]));
    let err = cerulion_core::graph::validate_graph(&config)
        .expect_err("validate_graph must reject an orphan-node partition")
        .to_string();
    assert!(
        err.contains("n4") && err.contains("not assigned to any process group"),
        "validate_graph must surface the partition error; got: {err}"
    );
}

#[test]
fn empty_group_rejected() {
    // A group with no members is rejected (it owns nothing yet shifts later
    // groups' ranks). P0 is empty; P1 holds the whole chain.
    let config = chain_config(groups(&[
        ("P0", &[]),
        ("P1", &["n0", "n1", "n2", "n3", "n4"]),
    ]));
    let err = validate_process_groups(&config)
        .expect_err("an empty group must be rejected")
        .to_string();
    assert!(
        err.contains("P0") && err.contains("empty"),
        "empty-group error must name the group + 'empty'; got: {err}"
    );
}

#[test]
fn empty_group_name_rejected() {
    // An empty/whitespace-only group NAME sorts first into a rank-0 ghost group
    // that later feeds barrier name derivation — reject it.
    let config = chain_config(groups(&[("", &["n0", "n1", "n2", "n3", "n4"])]));
    let err = validate_process_groups(&config)
        .expect_err("an empty group name must be rejected")
        .to_string();
    assert!(
        err.contains("empty or whitespace"),
        "empty-name error must say 'empty or whitespace'; got: {err}"
    );
}

#[test]
fn same_group_duplicate_node_rejected() {
    // n0 listed TWICE in the SAME group P0. The message must say "same process
    // group 'P0'" — NOT the self-contradictory cross-group "'P0' and 'P0'".
    let config = chain_config(groups(&[
        ("P0", &["n0", "n0"]),
        ("P1", &["n1", "n2", "n3", "n4"]),
    ]));
    let err = validate_process_groups(&config)
        .expect_err("a node listed twice in one group must be rejected")
        .to_string();
    assert!(
        err.contains("n0") && err.contains("same process group 'P0'"),
        "same-group dup error must name the node + the single group; got: {err}"
    );
    assert!(
        !err.contains("more than one process group"),
        "same-group dup must NOT use the cross-group message; got: {err}"
    );
}

#[test]
fn derive_rejects_orphan_via_internal_validation() {
    // `derive_process_groups` validates FIRST, so an orphan-partition config is
    // rejected at derive time (not silently tolerated).
    let config = chain_config(groups(&[("P0", &["n0", "n1"]), ("P1", &["n2", "n3"])]));
    let levels = chain_levels(&config);
    let err = derive_process_groups(&config, &levels)
        .expect_err("derive must reject an orphan-node partition via internal validation")
        .to_string();
    assert!(
        err.contains("n4") && err.contains("not assigned to any process group"),
        "derive must surface the validation error; got: {err}"
    );
}

#[test]
fn derive_errs_on_config_levels_mismatch() {
    // The config/levels MISMATCH path validation cannot catch: a structurally
    // VALID partition (all 5 chain nodes assigned) derived against a SMALLER
    // levelization (4-node chain, missing n4) → the member-no-level error.
    // Validation passes (config is internally consistent), so the only failure
    // is the level lookup for n4.
    let full_config = chain_config(groups(&[("P0", &["n0", "n1", "n2", "n3", "n4"])]));

    // A smaller config (n0..n3 only) — n4 has NO level here.
    let small_config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        name: None,
        identity: "small".to_string(),
        prefix: PREFIX.to_string(),
        nodes: vec![
            node("n0", &[], &["out"]),
            node("n1", &[("inp", "n0/out")], &["out"]),
            node("n2", &[("inp", "n1/out")], &["out"]),
            node("n3", &[("inp", "n2/out")], &[]),
        ],
        multi_publisher_topics: Vec::new(),
        process_groups: IndexMap::new(),
        process_group_order: Vec::new(),
    };
    let small_levels = levels_for(
        &small_config,
        &[
            ("n1", "/p/n0/out"),
            ("n2", "/p/n1/out"),
            ("n3", "/p/n2/out"),
        ],
    );
    assert_eq!(small_levels.len(), 4, "smaller chain has 4 levels");
    assert_eq!(
        small_levels.level_of("n4"),
        None,
        "n4 absent from small levels"
    );

    let err = derive_process_groups(&full_config, &small_levels)
        .expect_err("derive must err when a member has no level in the levelization")
        .to_string();
    assert!(
        err.contains("n4") && err.contains("no level"),
        "mismatch error must name the level-less member; got: {err}"
    );
}

#[test]
fn single_group_all_some() {
    // One group owning the whole chain → every global level owned, locals
    // contiguous 0..5, rank 0.
    let config = chain_config(groups(&[("P0", &["n0", "n1", "n2", "n3", "n4"])]));
    let levels = chain_levels(&config);
    let derived = derive_process_groups(&config, &levels).expect("derive");
    assert_eq!(derived.len(), 1);
    assert_eq!(derived[0].rank, 0);
    assert_eq!(
        derived[0].global_level_map,
        vec![Some(0), Some(1), Some(2), Some(3), Some(4)],
        "the sole group owns every global level"
    );
}

#[test]
fn process_group_order_without_groups_rejected() {
    // `process_groups` and `process_group_order` are independent
    // `#[serde(default)]` fields: an order list set with NO groups (e.g. YAML
    // `process_group_order: [foo]` and no `process_groups:`) used to pass
    // validation silently AND make `derive_process_groups` panic on the empty
    // map. It must now be rejected LOUDLY at validation.
    let mut config = chain_config(IndexMap::new());
    config.process_group_order = vec!["foo".to_string()];
    let err = validate_process_groups(&config)
        .expect_err("process_group_order with empty process_groups must be rejected")
        .to_string();
    assert!(
        err.contains("process_groups is empty"),
        "error must say process_groups is empty; got: {err}"
    );
}

#[test]
fn derive_rejects_invalid_order_via_validation() {
    // A VALID-membership partition (the 5-node chain {P0:[n0,n1], P1:[n2,n3,n4]})
    // but with an order list naming an UNKNOWN extra group ("ghost"). `derive`
    // validates FIRST, so it must return an Err (not panic at the override
    // branch's `.expect()` on the missing key).
    let mut config = chain_config(groups(&[
        ("P0", &["n0", "n1"]),
        ("P1", &["n2", "n3", "n4"]),
    ]));
    config.process_group_order = vec!["P0".to_string(), "P1".to_string(), "ghost".to_string()];
    let levels = chain_levels(&config);
    let err = derive_process_groups(&config, &levels)
        .expect_err("derive must reject an unknown name in process_group_order via validation")
        .to_string();
    assert!(
        err.contains("not a declared group"),
        "derive must surface the unknown-group validation error; got: {err}"
    );
}

#[test]
fn has_process_groups_true() {
    // The existing `empty_process_groups_is_ok` covers the false case; pin true.
    let config = chain_config(groups(&[
        ("P0", &["n0", "n1"]),
        ("P1", &["n2", "n3", "n4"]),
    ]));
    assert!(
        config.has_process_groups(),
        "a non-empty process_groups map must report has_process_groups() == true"
    );
}

// ==========================================================================
// `compress_group_level_assignments` + the told-levels
// bijection under a `level_assignments` override.
// ==========================================================================

/// Build an assignments map from `(node, level)` literals.
fn assign(pairs: &[(&str, usize)]) -> IndexMap<String, usize> {
    pairs.iter().map(|(n, l)| (n.to_string(), *l)).collect()
}

#[test]
fn compress_group_level_assignments_vectors() {
    use cerulion_core::graph::partition::compress_group_level_assignments as compress;
    let global = assign(&[("a", 0), ("b", 2), ("c", 4), ("d", 2), ("e", 7)]);

    // Contiguous-from-zero identity.
    let out = compress(&assign(&[("a", 0), ("b", 1)]), &["a", "b"]);
    assert_eq!(
        out.iter()
            .map(|(k, v)| (k.as_str(), *v))
            .collect::<Vec<_>>(),
        vec![("a", 0), ("b", 1)]
    );
    // GAPPED owned band {0,2,4,7} rank-compresses to 0..=3; members in the
    // GIVEN (graph) order, not level order.
    let out = compress(&global, &["e", "a", "c", "b"]);
    assert_eq!(
        out.iter()
            .map(|(k, v)| (k.as_str(), *v))
            .collect::<Vec<_>>(),
        vec![("e", 3), ("a", 0), ("c", 2), ("b", 1)],
        "ranks among sorted distinct owned levels; key order = member order"
    );
    // SHARED levels dedup: b and d both at global 2 share local rank.
    let out = compress(&global, &["b", "d", "c"]);
    assert_eq!(
        out.iter()
            .map(|(k, v)| (k.as_str(), *v))
            .collect::<Vec<_>>(),
        vec![("b", 0), ("d", 0), ("c", 1)],
        "co-located members share a local level"
    );
    // A member missing from the map is SKIPPED (loud downstream via the
    // worker's own coverage validation).
    let out = compress(&global, &["a", "ghost"]);
    assert_eq!(
        out.iter()
            .map(|(k, v)| (k.as_str(), *v))
            .collect::<Vec<_>>(),
        vec![("a", 0)]
    );
    // Empty members / empty map are total.
    assert!(compress(&global, &[]).is_empty());
    assert!(compress(&IndexMap::new(), &["a"]).is_empty());
}

/// THE load-bearing told-levels pin (mutation kills the
/// `subgraph_local_levels` compression wiring): a group whose two members
/// are Kahn-CO-LOCATED in its induced subgraph (both roots — no in-group
/// edge) but SEPARATED by a `level_assignments` override passes
/// `validate_partition` ONLY because the group-local bijection model runs
/// the COMPRESSED assignment instead of re-Kahning. Reverting the
/// compression (sub-config `level_assignments: None`) re-Kahns both members
/// to local 0 while the override expects ranks {0,1} — a Bridged refusal.
#[test]
fn override_separated_colocated_members_validate_via_compression() {
    // x and y are independent sources (no edge between them); z consumes y.
    // Kahn: x@0, y@0, z@1. Override: x@0, y@1, z@2 (legal: y->z stays
    // increasing, levels contiguous + non-empty).
    let mut config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        name: None,
        identity: "sep".to_string(),
        prefix: PREFIX.to_string(),
        nodes: vec![
            node("x", &[], &["out"]),
            node("y", &[], &["out"]),
            node("z", &[("inp", "y/out")], &[]),
        ],
        multi_publisher_topics: Vec::new(),
        process_groups: groups(&[("g0", &["x", "y"]), ("g1", &["z"])]),
        process_group_order: Vec::new(),
    };
    config.level_assignments = Some(assign(&[("x", 0), ("y", 1), ("z", 2)]));

    let entry_infos: IndexMap<String, cerulion_core::graph::NodeInfo> = config
        .nodes
        .iter()
        .map(|n| {
            (
                n.id.clone(),
                cerulion_core::graph::NodeInfo::with_meta(Vec::new(), Vec::new()),
            )
        })
        .collect();
    let mut edges = TriggerEdges::new();
    edges.insert("z", "/p/y/out");

    // WITH the override: g0 owns global {0,1}; its compressed local band is
    // x@0, y@1 — the bijection holds by construction (told levels).
    cerulion_core::graph::partition::validate_partition(
        &config.process_groups,
        &config,
        &entry_infos,
        &edges,
    )
    .expect("the compression makes the override-separated group spawner-consumable");

    // CONTROL (anti-tautology): the identical partition WITHOUT the override
    // is also valid — g0 owns only Kahn level {0} (both members co-located),
    // a 1-level band. The pin above is NOT vacuous validity: only the
    // compression arm reconciles local levels with the 2-level owned band.
    let mut plain = config.clone();
    plain.level_assignments = None;
    cerulion_core::graph::partition::validate_partition(
        &plain.process_groups,
        &plain,
        &entry_infos,
        &edges,
    )
    .expect("the Kahn shape of the same partition is independently valid");
}
