// SPDX-License-Identifier: AGPL-3.0-only
//! PURE engine tests for `cerulion graph partition`
//! ([`graph_partition`]) — cost-mode selection (default-absent ⇒ baseline,
//! present ⇒ fused, malformed ⇒ hard Err), the consent ladder (dry-run /
//! unchanged / `--yes` / no-TTY refusal / interactive confirm), the
//! `process_group_order` removal, and the preview contents.
//!
//! No iceoryx2, no transport, no `GraphRuntime::build` — the verb is static
//! analysis over YAML + node source metadata (the same `parse_node_metadata`
//! path production uses), so the whole file is parallel-safe. Node types are
//! scaffolded via the real `node_cmd` helpers (cribbed from
//! `graph_levels_test.rs`). Oracles are hand-built group shapes and byte
//! comparisons — never a self-compare.

use std::path::Path;

use cerulion_cli_engine::error::CliResult;
use cerulion_cli_engine::partition_emit::{
    graph_partition, PartitionMode, PartitionOptions, PartitionOutcome,
};
use cerulion_cli_engine::{graph_cmd, node_cmd, workspace};
use cerulion_core::graph::parse_graph_raw;
use cerulion_core::MacroPolicy;
use indexmap::IndexMap;

/// Scaffold a workspace with the chain node types:
/// * `src`   — `period_ms = 10`, one output `out`.
/// * `relay` — DataTrigger on input `inp`, one output `out`.
/// * `sink`  — DataTrigger on input `inp`, no outputs.
fn setup_workspace(root: &Path) -> workspace::CerulionWorkspace {
    let ws = workspace::workspace_create(root, "part_ws").expect("workspace");
    let cargo_toml = ws.root.join("Cargo.toml");

    node_cmd::node_create(
        &ws.nodes_dir,
        &cargo_toml,
        "src",
        Some(MacroPolicy::Period { period_ms: 10 }),
    )
    .expect("create src");
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "src",
        "out",
        Some("std_msgs::Int32"),
        true,
        false,
    )
    .expect("src out");

    node_cmd::node_create(&ws.nodes_dir, &cargo_toml, "relay", None).expect("create relay");
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "relay",
        "inp",
        Some("std_msgs::Int32"),
        false,
        true,
    )
    .expect("relay inp");
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "relay",
        "out",
        Some("std_msgs::Int32"),
        true,
        false,
    )
    .expect("relay out");

    node_cmd::node_create(&ws.nodes_dir, &cargo_toml, "sink", None).expect("create sink");
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "sink",
        "inp",
        Some("std_msgs::Int32"),
        false,
        true,
    )
    .expect("sink inp");

    // A 2-input Sync node — the consumer of a gap-spanning direct edge
    // (needs a second input to be pushed up the DAG past the direct producer).
    // Both aligned inputs are TRIGGER-marked (sync aligns only
    // `#[input(trigger)]` ports — a non-trigger input would no longer be a DAG
    // edge, collapsing the gap-spanning premise).
    node_cmd::node_create(&ws.nodes_dir, &cargo_toml, "join", None).expect("create join");
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "join",
        "a",
        Some("std_msgs::Int32"),
        false,
        true,
    )
    .expect("join a");
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "join",
        "b",
        Some("std_msgs::Int32"),
        false,
        true,
    )
    .expect("join b");
    node_cmd::node_modify_set_sync(&ws.nodes_dir, "join", 25).expect("join sync");

    ws
}

/// The canonical 3-node chain graph, WITH comments so byte-preservation is
/// observable on the write paths.
const CHAIN_YAML: &str = r#"# chain demo — this comment must survive the rewrite
name: chain
prefix: p

# --- topology (untouched) ---
nodes:
  - id: n0
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: n1
    type: relay
    inputs:
      - name: inp
        source: n0/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: n2
    type: sink
    inputs:
      - name: inp
        source: n1/out
"#;

fn write_graph(ws: &workspace::CerulionWorkspace, name: &str, yaml: &str) {
    std::fs::write(ws.graphs_dir.join(format!("{name}.yaml")), yaml).expect("write graph yaml");
}

/// A well-formed cost artifact for the chain: all three nodes costed, both
/// trigger edges rated — with an unbounded budget everything fuses into ONE
/// group led by `n0`.
const CHAIN_COSTS_YAML: &str = r#"version: 1
graph: chain
window_ns: 5000000000
nodes:
  n0: 1000
  n1: 900
  n2: 800
edges:
- producer: n0
  consumer: n1
  rate_mhz: 20000
- producer: n1
  consumer: n2
  rate_mhz: 20000
isolated: []
hop:
  intra_ns: 1000
  cross_ns: 6800
"#;

/// Default options: NO budget override (the resolution point picks
/// the artifact's frozen budget; the v1 `CHAIN_COSTS_YAML` fixture has none,
/// so these tests keep their earlier unbounded behavior).
fn opts(dry_run: bool, assume_yes: bool) -> PartitionOptions {
    PartitionOptions {
        costs_path: None,
        budget_ns: None,
        dry_run,
        assume_yes,
    }
}

/// A confirm provider that must never be reached (proves the arm under test
/// does not consult it).
fn panic_confirm(_preview: &str) -> CliResult<bool> {
    panic!("the confirm provider must not be invoked on this path")
}

/// Flatten a groups map into ordered `(name, members)` pairs for oracles.
fn shape(map: &IndexMap<String, Vec<String>>) -> Vec<(String, Vec<String>)> {
    map.iter().map(|(k, v)| (k.clone(), v.clone())).collect()
}

fn expect(pairs: &[(&str, &[&str])]) -> Vec<(String, Vec<String>)> {
    pairs
        .iter()
        .map(|(n, m)| (n.to_string(), m.iter().map(|s| s.to_string()).collect()))
        .collect()
}

// ==========================================================================
// Cost-mode selection.
// ==========================================================================

#[test]
fn absent_default_artifact_emits_baseline() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);

    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "chain", &opts(true, false), false, &mut confirm)
        .expect("baseline dry run");

    assert_eq!(
        report.mode,
        PartitionMode::Baseline {
            missing_artifact_path: graph_cmd::default_artifact_path(&ws.root, "chain"),
        },
        "absent default artifact => baseline mode naming the checked path"
    );
    assert_eq!(
        shape(&report.groups),
        expect(&[
            ("grp_n0", &["n0"]),
            ("grp_n1", &["n1"]),
            ("grp_n2", &["n2"]),
        ]),
        "baseline = process-per-node in pipeline order"
    );
    assert_eq!(report.outcome, PartitionOutcome::DryRun);
    assert!(
        report.preview.contains("process-per-node baseline")
            && report.preview.contains("graph profile"),
        "the preview names the mode and the remedy; got:\n{}",
        report.preview
    );
}

#[test]
fn artifact_at_default_path_fuses() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    let artifact_path = graph_cmd::default_artifact_path(&ws.root, "chain");
    std::fs::write(&artifact_path, CHAIN_COSTS_YAML).expect("write artifact");

    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "chain", &opts(true, false), false, &mut confirm)
        .expect("fused dry run");

    assert_eq!(
        report.mode,
        PartitionMode::Fused { artifact_path },
        "present default artifact => cost-aware mode"
    );
    assert_eq!(
        shape(&report.groups),
        expect(&[("grp_n0", &["n0", "n1", "n2"])]),
        "both rated edges fuse under the unbounded budget"
    );
    assert!(
        report.preview.contains("cost-aware fusion"),
        "the preview names the fused mode; got:\n{}",
        report.preview
    );
}

#[test]
fn budget_flag_reaches_the_partitioner() {
    // The SAME artifact under a budget too small for any pair (1000+900 and
    // 900+800 both exceed 1500) must stay process-per-node — pins that
    // `budget_ns` is threaded through to `auto_partition`, not dropped.
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "chain"),
        CHAIN_COSTS_YAML,
    )
    .expect("write artifact");

    let tight = PartitionOptions {
        costs_path: None,
        budget_ns: Some(1_500),
        dry_run: true,
        assume_yes: false,
    };
    let mut confirm = panic_confirm;
    let report =
        graph_partition(&ws.root, "chain", &tight, false, &mut confirm).expect("tight budget");
    assert_eq!(
        shape(&report.groups),
        expect(&[
            ("grp_n0", &["n0"]),
            ("grp_n1", &["n1"]),
            ("grp_n2", &["n2"]),
        ]),
        "a 1500 ns budget rejects every fusion (each pair sums to >1500)"
    );
}

// ==========================================================================
// The budget RESOLUTION POINT — explicit --budget-ns override >
// the artifact's FROZEN core-count budget > unbounded fallback (loud). Each
// arm is a group-count-CHANGING behavioral vector (an ignored-frozen-budget
// variant that falls through to u64::MAX fuses everything into ONE group and
// fails the split oracles).
// ==========================================================================

/// The chain artifact as a v2 file carrying a FROZEN budget of
/// 1500 ns — too small for ANY pair (1000+900 and 900+800 both exceed it),
/// so a no-flag run gated by the frozen value stays process-per-node.
const CHAIN_COSTS_V2_FROZEN_YAML: &str = r#"version: 2
graph: chain
window_ns: 5000000000
nodes:
  n0: 1000
  n1: 900
  n2: 800
edges:
- producer: n0
  consumer: n1
  rate_mhz: 20000
- producer: n1
  consumer: n2
  rate_mhz: 20000
isolated: []
hop:
  intra_ns: 1000
  cross_ns: 6800
derived_budget_ns: 1500
profile_cores: 2
"#;

#[tracing_test::traced_test]
#[test]
fn frozen_artifact_budget_gates_fusion_without_flag() {
    // NO --budget-ns: the resolution point must consume the artifact's FROZEN
    // 1500 ns budget => process-per-node (3 groups). A variant that ignores
    // the frozen value (u64::MAX fallback) fuses ONE group and fails here.
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "chain"),
        CHAIN_COSTS_V2_FROZEN_YAML,
    )
    .expect("write v2 artifact");

    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "chain", &opts(true, false), false, &mut confirm)
        .expect("frozen-budget dry run");
    assert_eq!(
        shape(&report.groups),
        expect(&[
            ("grp_n0", &["n0"]),
            ("grp_n1", &["n1"]),
            ("grp_n2", &["n2"]),
        ]),
        "the FROZEN 1500 ns budget rejects every fusion — process-per-node"
    );
    assert!(
        logs_contain("using the artifact's frozen core-count budget"),
        "the frozen-budget consumption is announced loudly"
    );
}

#[test]
fn explicit_budget_overrides_frozen_value() {
    // The SAME frozen-1500 artifact under an EXPLICIT --budget-ns 5000
    // (1000+900+800 = 2700 <= 5000): the override wins over the frozen value
    // => the whole chain fuses into ONE group (hand oracle).
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "chain"),
        CHAIN_COSTS_V2_FROZEN_YAML,
    )
    .expect("write v2 artifact");

    let over = PartitionOptions {
        costs_path: None,
        budget_ns: Some(5_000),
        dry_run: true,
        assume_yes: false,
    };
    let mut confirm = panic_confirm;
    let report =
        graph_partition(&ws.root, "chain", &over, false, &mut confirm).expect("override run");
    assert_eq!(
        shape(&report.groups),
        expect(&[("grp_n0", &["n0", "n1", "n2"])]),
        "an explicit --budget-ns overrides the frozen 1500 => full fusion"
    );
}

#[tracing_test::traced_test]
#[test]
fn v1_artifact_without_frozen_budget_falls_back_unbounded_with_info() {
    // A earlier v1 artifact (no derived_budget_ns) + no override: the
    // fallback is UNBOUNDED fusion — byte-identical to the earlier
    // grouping (ONE fused group) — announced with the loud re-profile info.
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "chain"),
        CHAIN_COSTS_YAML, // version: 1 — no frozen budget fields
    )
    .expect("write v1 artifact");

    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "chain", &opts(true, false), false, &mut confirm)
        .expect("v1 fallback run");
    assert_eq!(
        shape(&report.groups),
        expect(&[("grp_n0", &["n0", "n1", "n2"])]),
        "no frozen budget + no override => the earlier unbounded grouping"
    );
    assert!(
        logs_contain("no frozen budget in the artifact"),
        "the unbounded fallback on a budget-less artifact is announced loudly"
    );
}

#[test]
fn frozen_zero_budget_is_refused_loudly_not_silently_consumed() {
    // `from_profile` floors the frozen value at 1, but the artifact
    // is a DOCUMENTED hand-edit surface — a hand-edited `derived_budget_ns: 0`
    // must be a LOUD Err mirroring the --budget-ns override guard, never a
    // silent force-to-process-per-node with only an info breadcrumb.
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    let zero_frozen =
        CHAIN_COSTS_V2_FROZEN_YAML.replace("derived_budget_ns: 1500", "derived_budget_ns: 0");
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "chain"),
        zero_frozen,
    )
    .expect("write artifact");

    let mut confirm = panic_confirm;
    let err = graph_partition(&ws.root, "chain", &opts(true, false), false, &mut confirm)
        .expect_err("a hand-edited frozen 0 budget must be refused")
        .to_string();
    assert!(
        err.contains("derived_budget_ns is 0")
            && err.contains("re-profile")
            && err.contains("--budget-ns"),
        "the refusal names the zero frozen value AND both remedies; got: {err}"
    );
}

#[test]
fn explicit_override_bypasses_a_frozen_zero_budget() {
    // The override never touches the frozen value: the SAME zero-frozen
    // artifact partitions fine under an explicit --budget-ns (full fusion at
    // 5000 >= 2700 — hand oracle).
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    let zero_frozen =
        CHAIN_COSTS_V2_FROZEN_YAML.replace("derived_budget_ns: 1500", "derived_budget_ns: 0");
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "chain"),
        zero_frozen,
    )
    .expect("write artifact");

    let over = PartitionOptions {
        costs_path: None,
        budget_ns: Some(5_000),
        dry_run: true,
        assume_yes: false,
    };
    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "chain", &over, false, &mut confirm)
        .expect("an explicit override never consults the frozen value");
    assert_eq!(
        shape(&report.groups),
        expect(&[("grp_n0", &["n0", "n1", "n2"])]),
        "override 5000 >= 2700 total => full fusion despite the zero frozen field"
    );
}

#[test]
fn explicit_costs_flag_missing_file_is_hard_err() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);

    let explicit = PartitionOptions {
        costs_path: Some(tmp.path().join("nope.costs.yaml")),
        budget_ns: None,
        dry_run: true,
        assume_yes: false,
    };
    let mut confirm = panic_confirm;
    let err = graph_partition(&ws.root, "chain", &explicit, false, &mut confirm)
        .expect_err("an explicitly named missing artifact must be refused")
        .to_string();
    assert!(
        err.contains("nope.costs.yaml") && err.contains("--costs"),
        "the error names the missing file and the flag; got: {err}"
    );
}

#[test]
fn malformed_default_artifact_is_hard_err_never_baseline() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    let artifact_path = graph_cmd::default_artifact_path(&ws.root, "chain");
    std::fs::write(&artifact_path, "nodes: [this is not the schema\n").expect("write junk");

    let mut confirm = panic_confirm;
    let err = graph_partition(&ws.root, "chain", &opts(true, false), false, &mut confirm)
        .expect_err("a present-but-malformed artifact must be a hard error, never baseline")
        .to_string();
    assert!(
        err.contains("MALFORMED") && err.contains("chain.costs.yaml"),
        "the error names the malformed file; got: {err}"
    );
}

// ==========================================================================
// The cost artifact's `graph:` PROVENANCE is checked.
//
// `graph profile` stamps the graph it harvested from, and nothing read it. A
// stale or copied `graphs/<name>.costs.yaml` whose node ids OVERLAP fuses a
// partition from another graph's measured costs — process groups that are
// credible and wrong. Both arms below use an artifact that is otherwise
// PERFECTLY VALID (it parses, its version is current, it costs every node this
// graph declares), so the ONLY thing under test is the provenance field: a
// gate that let it through would fuse and both arms would report groups.
// ==========================================================================

/// The chain artifact, byte-for-byte, except that its `graph:` names something
/// else. Overlapping node ids are the POINT — an artifact whose ids did not
/// match would already be refused by `to_costs`, so it could not distinguish a
/// provenance gate from the checks that were there before.
const FOREIGN_COSTS_YAML: &str = r#"version: 1
graph: some_other_graph
window_ns: 5000000000
nodes:
  n0: 1000
  n1: 900
  n2: 800
edges:
- producer: n0
  consumer: n1
  rate_mhz: 20000
- producer: n1
  consumer: n2
  rate_mhz: 20000
isolated: []
hop:
  intra_ns: 1000
  cross_ns: 6800
"#;

#[test]
fn explicit_costs_from_another_graph_is_hard_err_naming_both_graphs() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    // Deliberately NOT at the default path: the user pointed at this file.
    let explicit_path = tmp.path().join("borrowed.costs.yaml");
    std::fs::write(&explicit_path, FOREIGN_COSTS_YAML).expect("write foreign artifact");

    let explicit = PartitionOptions {
        costs_path: Some(explicit_path.clone()),
        budget_ns: None,
        dry_run: true,
        assume_yes: false,
    };
    let mut confirm = panic_confirm;
    let err = graph_partition(&ws.root, "chain", &explicit, false, &mut confirm)
        .expect_err("an explicitly named FOREIGN artifact must be refused")
        .to_string();

    // BOTH graphs, because either alone leaves the operator guessing: the
    // found one says which file this is, the expected one says which it
    // should have been.
    assert!(
        err.contains("some_other_graph"),
        "the error names the graph the artifact was harvested from; got: {err}"
    );
    assert!(
        err.contains("chain"),
        "the error names the graph being partitioned; got: {err}"
    );
    assert!(
        err.contains("borrowed.costs.yaml"),
        "the error names the offending file; got: {err}"
    );
    assert!(
        err.contains("graph profile"),
        "the error names the remedy; got: {err}"
    );
}

#[test]
fn foreign_default_artifact_is_hard_err_mirroring_malformed() {
    // The DEFAULT-path arm on the VERB: `lenient_default` is false here, so a
    // foreign artifact takes exactly the same posture as every other
    // present-but-unusable class (cf.
    // `malformed_default_artifact_is_hard_err_never_baseline`) — refused, never
    // a silent baseline. The `graph run` zero-flag default degrades LOUDLY
    // instead; that half is
    // `graph_run_preflight_test::lenient_default_degrades_a_foreign_graphs_artifact_to_baseline_with_warn`.
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "chain"),
        FOREIGN_COSTS_YAML,
    )
    .expect("write foreign artifact");

    let mut confirm = panic_confirm;
    let err = graph_partition(&ws.root, "chain", &opts(true, false), false, &mut confirm)
        .expect_err("a foreign DEFAULT artifact must be a hard error, never baseline")
        .to_string();
    assert!(
        err.contains("some_other_graph") && err.contains("chain.costs.yaml"),
        "the error names the foreign graph and the file; got: {err}"
    );
}

#[test]
fn an_explicit_costs_artifact_naming_this_graph_still_fuses() {
    // ANTI-TAUTOLOGY, and the FIRST happy-path coverage the explicit `--costs`
    // arm has ever had: before this, that arm's only test was the
    // missing-file refusal, so a gate rejecting EVERY explicit artifact would
    // pass the whole file. The artifact differs from `FOREIGN_COSTS_YAML` in
    // exactly one field.
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    let explicit_path = tmp.path().join("elsewhere.costs.yaml");
    std::fs::write(
        &explicit_path,
        FOREIGN_COSTS_YAML.replace("graph: some_other_graph", "graph: chain"),
    )
    .expect("write matching artifact");

    let explicit = PartitionOptions {
        costs_path: Some(explicit_path.clone()),
        budget_ns: None,
        dry_run: true,
        assume_yes: false,
    };
    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "chain", &explicit, false, &mut confirm)
        .expect("an artifact whose provenance AGREES must still fuse");

    assert_eq!(
        report.mode,
        PartitionMode::Fused {
            artifact_path: explicit_path,
        },
        "a matching explicit artifact => cost-aware mode"
    );
    assert_eq!(
        shape(&report.groups),
        expect(&[("grp_n0", &["n0", "n1", "n2"])]),
        "both rated edges fuse under the unbounded budget — the costs were really consumed"
    );
}

#[test]
fn unsupported_artifact_version_is_hard_err() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    let versioned = CHAIN_COSTS_YAML.replace("version: 1", "version: 99");
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "chain"),
        versioned,
    )
    .expect("write artifact");

    let mut confirm = panic_confirm;
    // Since the frozen-budget change the rejection fires in `read_costs_artifact`'s
    // version-FIRST probe (`parse_profile_artifact`), BEFORE `to_costs` is
    // ever reached on this path; `to_costs`' own version gate remains as
    // defense-in-depth on paths that receive an already-parsed artifact.
    let err = graph_partition(&ws.root, "chain", &opts(true, false), false, &mut confirm)
        .expect_err("an unsupported artifact version must be refused")
        .to_string();
    assert!(
        err.contains("version 99"),
        "the error names the unsupported version; got: {err}"
    );
}

#[test]
fn future_version_with_unknown_field_fails_on_version_message_at_the_real_surface() {
    // The version-FIRST contract pinned THROUGH the production read
    // site (`graph_partition` → `read_costs_artifact`), not just the
    // `parse_profile_artifact` helper. A future-version artifact carrying a
    // field this build has never heard of must surface the actionable VERSION
    // message — a revert of `read_costs_artifact` to direct serde parsing
    // dies on the raw unknown-field error instead and fails BOTH asserts.
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    let future = CHAIN_COSTS_YAML.replace("version: 1", "version: 3") + "future_field: x\n";
    std::fs::write(graph_cmd::default_artifact_path(&ws.root, "chain"), future)
        .expect("write artifact");

    let mut confirm = panic_confirm;
    let err = graph_partition(&ws.root, "chain", &opts(true, false), false, &mut confirm)
        .expect_err("a future-version artifact must be refused")
        .to_string();
    assert!(
        err.contains("version 3") && err.contains("1..=2"),
        "the refusal must be the VERSION message naming the supported range; got: {err}"
    );
    assert!(
        !err.contains("unknown field"),
        "the raw serde unknown-field fragment must never be the diagnosis; got: {err}"
    );
}

#[test]
fn zero_budget_is_refused_by_the_engine_too() {
    // clap rejects 0 at parse; the engine re-validates (contract alignment).
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);

    let zero = PartitionOptions {
        costs_path: None,
        budget_ns: Some(0),
        dry_run: true,
        assume_yes: false,
    };
    let mut confirm = panic_confirm;
    let err = graph_partition(&ws.root, "chain", &zero, false, &mut confirm)
        .expect_err("budget 0 must be refused")
        .to_string();
    assert!(err.contains("--budget-ns"), "names the flag; got: {err}");
}

// ==========================================================================
// The consent ladder.
// ==========================================================================

#[test]
fn no_tty_no_yes_refuses_and_mutates_nothing() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    let graph_path = ws.graphs_dir.join("chain.yaml");

    let mut confirm = panic_confirm;
    let err = graph_partition(&ws.root, "chain", &opts(false, false), false, &mut confirm)
        .expect_err("no TTY + no --yes must refuse to mutate")
        .to_string();
    assert!(
        err.contains("--yes") && err.contains("--dry-run"),
        "the refusal names both escape hatches; got: {err}"
    );
    assert_eq!(
        std::fs::read_to_string(&graph_path).expect("readable"),
        CHAIN_YAML,
        "the graph file must be byte-untouched after the refusal"
    );
    assert!(
        !ws.graphs_dir.join("chain.yaml.bak").exists(),
        "no backup churn on the refusal path"
    );
}

#[test]
fn dry_run_writes_nothing_and_wins_over_yes() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    let graph_path = ws.graphs_dir.join("chain.yaml");

    // BOTH --dry-run and --yes: dry-run wins (the documented precedence).
    let mut confirm = panic_confirm;
    let report =
        graph_partition(&ws.root, "chain", &opts(true, true), true, &mut confirm).expect("dry run");
    assert_eq!(report.outcome, PartitionOutcome::DryRun);
    assert!(!report.preview_shown, "dry run never consults the confirm");
    assert_eq!(
        std::fs::read_to_string(&graph_path).expect("readable"),
        CHAIN_YAML,
        "dry run must not touch the file"
    );
    assert!(
        !ws.graphs_dir.join("chain.yaml.bak").exists(),
        "dry run creates no backup"
    );
}

#[test]
fn yes_writes_through_atomic_path_with_backup_and_preserves_comments() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    let graph_path = ws.graphs_dir.join("chain.yaml");

    let mut confirm = panic_confirm; // --yes must skip the confirm entirely
    let report = graph_partition(&ws.root, "chain", &opts(false, true), false, &mut confirm)
        .expect("--yes write");

    let backup = match &report.outcome {
        PartitionOutcome::Written { backup } => backup.clone().expect("graph file pre-exists"),
        other => panic!("expected Written, got {other:?}"),
    };
    assert!(!report.preview_shown, "--yes never consults the confirm");
    assert_eq!(
        std::fs::read_to_string(&backup).expect("bak readable"),
        CHAIN_YAML,
        "the backup carries the ORIGINAL bytes"
    );

    let written = std::fs::read_to_string(&graph_path).expect("readable");
    assert!(
        written.contains("# chain demo — this comment must survive the rewrite")
            && written.contains("# --- topology (untouched) ---"),
        "comments survive the surgical write; got:\n{written}"
    );
    let parsed = parse_graph_raw(&written).expect("written graph re-parses");
    assert_eq!(
        shape(&parsed.process_groups),
        shape(&report.groups),
        "the written file carries exactly the derived groups"
    );
}

#[test]
fn interactive_confirm_yes_writes_and_shows_preview() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);

    let mut seen: Option<String> = None;
    let mut confirm = |preview: &str| -> CliResult<bool> {
        seen = Some(preview.to_string());
        Ok(true)
    };
    let report = graph_partition(&ws.root, "chain", &opts(false, false), true, &mut confirm)
        .expect("interactive yes");
    assert!(matches!(report.outcome, PartitionOutcome::Written { .. }));
    assert!(report.preview_shown, "the confirm displayed the preview");
    assert_eq!(
        seen.as_deref(),
        Some(report.preview.as_str()),
        "the confirm receives the SAME preview the report carries"
    );
}

#[test]
fn interactive_confirm_no_declines_and_writes_nothing() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    let graph_path = ws.graphs_dir.join("chain.yaml");

    let mut confirm = |_: &str| -> CliResult<bool> { Ok(false) };
    let report = graph_partition(&ws.root, "chain", &opts(false, false), true, &mut confirm)
        .expect("interactive no");
    assert_eq!(report.outcome, PartitionOutcome::Declined);
    assert_eq!(
        std::fs::read_to_string(&graph_path).expect("readable"),
        CHAIN_YAML,
        "a declined confirm must not touch the file"
    );
    assert!(
        !ws.graphs_dir.join("chain.yaml.bak").exists(),
        "no backup churn on decline"
    );
}

#[test]
fn second_run_is_unchanged_with_no_backup_churn() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    let graph_path = ws.graphs_dir.join("chain.yaml");
    let bak_path = ws.graphs_dir.join("chain.yaml.bak");

    let mut confirm = panic_confirm;
    let first = graph_partition(&ws.root, "chain", &opts(false, true), false, &mut confirm)
        .expect("first write");
    assert!(matches!(first.outcome, PartitionOutcome::Written { .. }));
    std::fs::remove_file(&bak_path).expect("clear the first run's backup");
    let after_first = std::fs::read_to_string(&graph_path).expect("readable");

    let second = graph_partition(&ws.root, "chain", &opts(false, true), false, &mut confirm)
        .expect("second run");
    assert_eq!(
        second.outcome,
        PartitionOutcome::Unchanged,
        "an identical re-derivation must be a no-op"
    );
    assert_eq!(
        std::fs::read_to_string(&graph_path).expect("readable"),
        after_first,
        "the file is byte-identical after the no-op"
    );
    assert!(!bak_path.exists(), "no backup churn on the Unchanged path");
    assert!(
        second.preview.contains("no change"),
        "the preview says so; got:\n{}",
        second.preview
    );
}

// ==========================================================================
// process_group_order removal.
// ==========================================================================

/// The chain graph WITH a hand partition and an explicit (soon-stale) rank
/// order — the emit must remove `process_group_order:` and rewrite
/// `process_groups:`.
const CHAIN_WITH_ORDER_YAML: &str = r#"name: chain
prefix: p

process_groups:
  g_a: [n0]
  g_b: [n1, n2]

process_group_order: [g_b, g_a]

# nodes below
nodes:
  - id: n0
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: n1
    type: relay
    inputs:
      - name: inp
        source: n0/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: n2
    type: sink
    inputs:
      - name: inp
        source: n1/out
"#;

#[tracing_test::traced_test]
#[test]
fn process_group_order_is_removed_warned_and_shown_in_preview() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_WITH_ORDER_YAML);
    let graph_path = ws.graphs_dir.join("chain.yaml");

    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "chain", &opts(false, true), false, &mut confirm)
        .expect("--yes write");
    assert!(matches!(report.outcome, PartitionOutcome::Written { .. }));
    assert!(report.removed_process_group_order);

    let written = std::fs::read_to_string(&graph_path).expect("readable");
    assert!(
        !written.contains("process_group_order"),
        "the stale order block must be gone; got:\n{written}"
    );
    assert!(
        written.contains("# nodes below"),
        "unrelated comments survive; got:\n{written}"
    );
    let parsed = parse_graph_raw(&written).expect("written graph re-parses");
    assert!(parsed.process_group_order.is_empty());
    assert_eq!(shape(&parsed.process_groups), shape(&report.groups));

    // The removal is LOUD and previewed.
    assert!(
        logs_contain("removed the process_group_order block"),
        "the removal must warn"
    );
    assert!(
        report.preview.contains("-process_group_order: [g_b, g_a]"),
        "the diff shows the removed order line; got:\n{}",
        report.preview
    );
    assert!(
        report
            .preview
            .contains("process_group_order removed: the emitted process_groups listing order"),
        "the diff explains WHY; got:\n{}",
        report.preview
    );
}

#[test]
fn graph_without_order_reports_no_removal() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);

    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "chain", &opts(true, false), false, &mut confirm)
        .expect("dry run");
    assert!(
        !report.removed_process_group_order,
        "no order block => nothing removed"
    );
    assert!(
        !report.preview.contains("process_group_order"),
        "the preview mentions no removal; got:\n{}",
        report.preview
    );
}

// ==========================================================================
// Preview smoke: bands + diff markers + group names in one place.
// ==========================================================================

#[test]
fn preview_carries_bands_summary_and_block_diff() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "chain"),
        CHAIN_COSTS_YAML,
    )
    .expect("write artifact");

    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "chain", &opts(true, false), false, &mut confirm)
        .expect("dry run");
    let p = &report.preview;

    // Summary + mode.
    assert!(p.contains("graph partition: chain"), "header; got:\n{p}");
    assert!(p.contains("nodes: 3  groups: 1"), "summary; got:\n{p}");
    // The levels renderer's band section for the PROPOSED grouping.
    assert!(p.contains("level 0: n0"), "levelization rows; got:\n{p}");
    assert!(
        p.contains("rank 0  grp_n0  levels 0-2  nodes: n0, n1, n2"),
        "the proposed group band row; got:\n{p}"
    );
    assert!(
        p.contains("partition: spawner-consumable"),
        "the validity verdict; got:\n{p}"
    );
    // The block-scoped diff (insert arm: no '-' block, one '+' block).
    assert!(
        p.contains("--- current") && p.contains("+++ proposed"),
        "diff headers; got:\n{p}"
    );
    assert!(
        p.contains("(no existing process_groups block — inserting before `nodes:`)"),
        "the insert-arm note; got:\n{p}"
    );
    assert!(
        p.contains("+process_groups:") && p.contains("+  grp_n0: [n0, n1, n2]"),
        "the added block lines; got:\n{p}"
    );
}

// ==========================================================================
// The order-removal warn is gated to WRITE-committing paths.
// ==========================================================================

#[tracing_test::traced_test]
#[test]
fn dry_run_shows_order_removal_in_preview_without_warning() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_WITH_ORDER_YAML);
    let graph_path = ws.graphs_dir.join("chain.yaml");

    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "chain", &opts(true, false), false, &mut confirm)
        .expect("dry run");
    assert_eq!(report.outcome, PartitionOutcome::DryRun);
    assert!(report.removed_process_group_order);

    // The removal is in the DIFF...
    assert!(
        report.preview.contains("-process_group_order: [g_b, g_a]"),
        "the diff shows the removal; got:\n{}",
        report.preview
    );
    // ...but NOTHING was written, so the warn must NOT fire (by design: a warn
    // claiming a removal happened when the file is untouched is misleading).
    assert!(
        !logs_contain("removed the process_group_order block"),
        "dry run must not warn about a removal that was never written"
    );
    assert_eq!(
        std::fs::read_to_string(&graph_path).expect("readable"),
        CHAIN_WITH_ORDER_YAML,
        "dry run leaves the file byte-untouched"
    );
}

#[tracing_test::traced_test]
#[test]
fn declined_confirm_does_not_warn_about_order_removal() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_WITH_ORDER_YAML);

    let mut confirm = |_: &str| -> CliResult<bool> { Ok(false) };
    let report = graph_partition(&ws.root, "chain", &opts(false, false), true, &mut confirm)
        .expect("declined");
    assert_eq!(report.outcome, PartitionOutcome::Declined);
    assert!(
        !logs_contain("removed the process_group_order block"),
        "a declined (unwritten) rewrite must not warn about the removal"
    );
}

// ==========================================================================
// REPLACE-SCOPED validation: the verb is the RECOVERY tool for
// a stale/broken partition block.
// ==========================================================================

/// The chain graph with a BROKEN hand partition: a dangling node reference
/// (`ghost`) AND an orphan (`n2` unassigned) — both hard `validate_graph`
/// errors. Every non-partition aspect of the graph is valid.
const CHAIN_WITH_STALE_GROUPS_YAML: &str = r#"name: chain
prefix: p

# stale partition from before a node was renamed/deleted
process_groups:
  old_a: [n0, ghost]
  old_b: [n1]

nodes:
  - id: n0
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: n1
    type: relay
    inputs:
      - name: inp
        source: n0/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: n2
    type: sink
    inputs:
      - name: inp
        source: n1/out
"#;

#[test]
fn stale_partition_block_is_recoverable_by_the_verb() {
    // Earlier this Err'd at validate_graph (dangling ref + orphan) before
    // the verb could do anything — refusing to fix the exact blocks it exists
    // to rewrite. Replace-scoped validation skips ONLY those arms.
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_WITH_STALE_GROUPS_YAML);
    let graph_path = ws.graphs_dir.join("chain.yaml");

    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "chain", &opts(false, true), false, &mut confirm)
        .expect(
            "a stale partition block must be RECOVERABLE (mutation pin: reverting \
                 replace-scoped validation back to validate_graph fails here)",
        );
    assert!(matches!(report.outcome, PartitionOutcome::Written { .. }));

    // The written file is now FULLY valid (the broken block was replaced).
    let written = std::fs::read_to_string(&graph_path).expect("readable");
    let parsed = parse_graph_raw(&written).expect("re-parses");
    cerulion_core::graph::validate_graph(&parsed)
        .expect("the recovered file passes FULL validation");
    assert_eq!(
        shape(&parsed.process_groups),
        shape(&report.groups),
        "the stale block was replaced by the derived one"
    );
    assert!(
        !written.contains("ghost"),
        "the dangling reference is gone; got:\n{written}"
    );
    assert!(
        written.contains("# stale partition from before a node was renamed/deleted"),
        "comments outside the replaced block survive"
    );
}

#[test]
fn non_partition_validation_errors_still_refuse() {
    // The scoping must skip ONLY the partition arms: a graph broken elsewhere
    // (empty nodes list) is still refused loudly.
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "empty", "name: empty\nprefix: p\nnodes: []\n");

    let mut confirm = panic_confirm;
    let err = graph_partition(&ws.root, "empty", &opts(true, false), false, &mut confirm)
        .expect_err("non-partition validation arms stay mandatory")
        .to_string();
    assert!(
        err.contains("at least one node"),
        "the non-partition validation error surfaces; got: {err}"
    );
}

// ==========================================================================
// The verb NEVER writes a NON-CONTIGUOUS block. Given a graph whose
// top-coupling edge is a gap-spanning DIRECT edge (d@L0 → c@L4 under Kahn,
// with a deep sibling chain pushing c up), the WRITTEN partition is always
// spawner-consumable (contiguous). A `check_group` without the contiguity rule
// PASSES the {d,c} fusion (the direct edge keeps the bijection intact), so
// the verb would SILENTLY WRITE the non-contiguous {d,c} group.
//
// Cost-aware refinement changes this shape's outcome: `d` is costed with slack
// (ASAP 0, ALAP 3 — its only consumer c sits at L4) and its 20 Hz gating
// rate clears L3's (unrated ⇒ 0) rate guard, so cost-aware refinement moves
// d → L3 and BAKES `level_assignments:` into the written yaml. The d→c
// fusion's owned band becomes the CONTIGUOUS {3,4}, so the top-coupling
// fusion is legitimately ACCEPTED — the verb heals the gap by moving
// the node rather than refusing the fusion. The invariant under test is
// UNCHANGED: the written block is contiguous + spawner-consumable.
// ==========================================================================

const SKEWED_YAML: &str = r#"name: skewed
prefix: p
nodes:
  - id: d
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: s0
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: s1
    type: relay
    inputs:
      - name: inp
        source: s0/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: s2
    type: relay
    inputs:
      - name: inp
        source: s1/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: s3
    type: relay
    inputs:
      - name: inp
        source: s2/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: c
    type: join
    inputs:
      - name: a
        source: d/out
      - name: b
        source: s3/out
"#;

/// Costs making the gap-spanning `d→c` edge the SINGLE positive-coupling
/// candidate (all six nodes costed; only `d→c` rated). v1 artifact ⇒ unbounded
/// budget, so CONTIGUITY (not budget) is the gate that rejects the fusion.
const SKEWED_COSTS_YAML: &str = r#"version: 1
graph: skewed
window_ns: 5000000000
nodes:
  d: 1000
  s0: 1000
  s1: 1000
  s2: 1000
  s3: 1000
  c: 1000
edges:
- producer: d
  consumer: c
  rate_mhz: 20000
isolated: []
hop:
  intra_ns: 1000
  cross_ns: 6800
"#;

#[test]
fn verb_never_writes_a_non_contiguous_block_self_heals() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "skewed", SKEWED_YAML);
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "skewed"),
        SKEWED_COSTS_YAML,
    )
    .expect("write artifact");

    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "skewed", &opts(false, true), false, &mut confirm)
        .expect("the verb self-heals to a valid partition (never errors)");
    assert!(
        matches!(report.outcome, PartitionOutcome::Written { .. }),
        "the derived (contiguous) partition is written; got {:?}",
        report.outcome
    );
    // Refinement moved d@L0 -> L3 (slack to its only consumer c@L4;
    // the 20 Hz edge clears L3's rate guard), so the d->c fusion's owned band
    // is the CONTIGUOUS {3,4} and the top-coupling fusion is ACCEPTED. The
    // groups band over the REFINED levels, pipeline order.
    assert_eq!(
        shape(&report.groups),
        expect(&[
            ("grp_s0", &["s0"]),
            ("grp_s1", &["s1"]),
            ("grp_s2", &["s2"]),
            ("grp_d", &["d", "c"]),
            ("grp_s3", &["s3"]),
        ]),
        "refinement makes the d->c fusion contiguous, so it IS fused (banded over refined levels)"
    );
    assert!(
        report.level_assignments_written,
        "moving d off L0 must bake the level_assignments block"
    );
    let written = std::fs::read_to_string(&report.graph_path).expect("read written graph");
    assert!(
        written.contains("level_assignments:") && written.contains("d: 3"),
        "the written yaml carries the refined assignment (d at level 3); got:\n{written}"
    );
    // END-TO-END validity: the levels verdict on the WRITTEN file is
    // spawner-consumable (contiguous) — NOT a silent non-contiguous block.
    let levels = graph_cmd::graph_levels(&ws.root, "skewed").expect("levels on written graph");
    assert!(
        levels.partition_error.is_none(),
        "the written partition must be spawner-consumable; got error: {:?}",
        levels.partition_error
    );
    assert!(
        levels.rendered.contains("partition: spawner-consumable")
            && !levels.rendered.contains("non-contiguous"),
        "the written block is contiguous + verdicted valid; got:\n{}",
        levels.rendered
    );
}

// ==========================================================================
// The level-refinement emit — `level_assignments:` written /
// omitted / removed / replaced, coherently with `process_groups:` in ONE
// rewrite. All pins reuse the SKEWED shape (d@L0 has slack to L3; its 20 Hz
// edge clears L3's rate guard, so costs MOVE d) and the CHAIN shape (a
// straight pipeline — zero slack, refinement can never move anything).
// ==========================================================================

/// (a) Costs that move a node ⇒ ONE consented rewrite carries BOTH blocks:
/// `level_assignments` with FULL coverage (which validation requires) at the
/// hand-computed refined levels, and `process_groups` banded over the
/// REFINED levels. Re-running the verb is idempotent (`Unchanged`).
#[test]
fn refined_emit_writes_both_blocks_full_coverage_then_reruns_unchanged() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "skewed", SKEWED_YAML);
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "skewed"),
        SKEWED_COSTS_YAML,
    )
    .expect("write artifact");

    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "skewed", &opts(false, true), false, &mut confirm)
        .expect("refined emit succeeds");
    assert!(matches!(report.outcome, PartitionOutcome::Written { .. }));
    assert!(report.level_assignments_written, "d moved ⇒ block written");
    assert!(!report.removed_level_assignments);
    // The preview names the write + the moved-node count.
    assert!(
        report
            .preview
            .contains("level_assignments written: cost-aware refinement moved 1 node(s)"),
        "preview names the level delta; got:\n{}",
        report.preview
    );

    // FULL coverage at the hand-computed refined levels: the chain s0..s3
    // and c keep their Kahn levels (zero slack / sink-consumer), d moves to
    // its ALAP L3. Re-parse the WRITTEN file — the block is the config.
    let written = std::fs::read_to_string(&report.graph_path).expect("read written");
    let config = parse_graph_raw(&written).expect("written graph parses");
    let block = config
        .level_assignments
        .as_ref()
        .expect("block present in the written yaml");
    let got: Vec<(String, usize)> = block.iter().map(|(k, v)| (k.clone(), *v)).collect();
    assert_eq!(
        got,
        vec![
            ("d".to_string(), 3),
            ("s0".to_string(), 0),
            ("s1".to_string(), 1),
            ("s2".to_string(), 2),
            ("s3".to_string(), 3),
            ("c".to_string(), 4),
        ],
        "full coverage in graph order, d refined 0 -> 3"
    );
    // Both blocks landed; the .bak carries the pre-write bytes.
    assert!(written.contains("process_groups:"));
    let bak = std::fs::read_to_string(ws.graphs_dir.join("skewed.yaml.bak")).expect("bak");
    assert_eq!(bak, SKEWED_YAML, ".bak is the original file");

    // Idempotence: the second run derives the identical rewrite ⇒ Unchanged,
    // nothing written, no backup churn (bak still the ORIGINAL bytes).
    let report2 = graph_partition(&ws.root, "skewed", &opts(false, true), false, &mut confirm)
        .expect("re-run succeeds");
    assert!(
        matches!(report2.outcome, PartitionOutcome::Unchanged),
        "same costs ⇒ same refinement ⇒ same file; got {:?}",
        report2.outcome
    );
    let bak2 = std::fs::read_to_string(ws.graphs_dir.join("skewed.yaml.bak")).expect("bak");
    assert_eq!(bak2, SKEWED_YAML, "no backup churn on Unchanged");
}

/// (b) The no-costs baseline OMITS the block (RULE: writing Kahn explicitly
/// adds hand-maintenance burden with zero behavior change — the block exists
/// only when it changes something).
#[test]
fn baseline_no_costs_omits_level_assignments_block() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "skewed", SKEWED_YAML);
    // No artifact at the default path ⇒ baseline mode.

    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "skewed", &opts(false, true), false, &mut confirm)
        .expect("baseline emit succeeds");
    assert!(matches!(report.mode, PartitionMode::Baseline { .. }));
    assert!(matches!(report.outcome, PartitionOutcome::Written { .. }));
    assert!(!report.level_assignments_written);
    assert!(!report.removed_level_assignments);
    let written = std::fs::read_to_string(&report.graph_path).expect("read written");
    assert!(
        !written.contains("level_assignments"),
        "no costs ⇒ no refinement ⇒ NO block; got:\n{written}"
    );
}

/// (c) Refinement no-op (costs present, zero slack) with a STALE Kahn-equal
/// block in the file ⇒ the block is REMOVED (its presence must always mean
/// "differs from Kahn") and the preview names the removal — including the
/// hand-written-block-that-equals-Kahn case, which this literally is.
#[test]
fn noop_refinement_removes_stale_kahn_equal_block_and_names_it() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    // CHAIN with a hand-written block equal to Kahn (n0:0, n1:1, n2:2).
    let chain_with_block = CHAIN_YAML.replace(
        "# --- topology (untouched) ---\nnodes:",
        "level_assignments:\n  n0: 0\n  n1: 1\n  n2: 2\n\n# --- topology (untouched) ---\nnodes:",
    );
    assert!(
        chain_with_block.contains("level_assignments:"),
        "fixture sane"
    );
    write_graph(&ws, "chain", &chain_with_block);
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "chain"),
        CHAIN_COSTS_YAML,
    )
    .expect("write artifact");

    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "chain", &opts(false, true), false, &mut confirm)
        .expect("no-op refinement emit succeeds");
    assert!(matches!(report.outcome, PartitionOutcome::Written { .. }));
    assert!(!report.level_assignments_written);
    assert!(
        report.removed_level_assignments,
        "the Kahn-equal block is removed"
    );
    assert!(
        report.preview.contains("level_assignments removed:")
            && report.preview.contains("-level_assignments:"),
        "preview names + diffs the removal; got:\n{}",
        report.preview
    );
    let written = std::fs::read_to_string(&report.graph_path).expect("read written");
    assert!(
        !written.contains("level_assignments"),
        "the block is gone; got:\n{written}"
    );
    // The surrounding comment survived the surgical removal.
    assert!(written.contains("# --- topology (untouched) ---"));
}

/// (e) Replace-scoped recovery: a stale INVALID `level_assignments:` block
/// (unknown node, broken coverage) is REPLACED by the re-derivation — never
/// fatal. The verb is the recovery tool for this block exactly as for
/// `process_groups:`.
#[test]
fn stale_invalid_level_assignments_block_is_replaced_not_fatal() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    let broken = format!("level_assignments:\n  ghost: 99\n\n{SKEWED_YAML}");
    write_graph(&ws, "skewed", &broken);
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "skewed"),
        SKEWED_COSTS_YAML,
    )
    .expect("write artifact");

    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "skewed", &opts(false, true), false, &mut confirm)
        .expect("an invalid stale block is replaced, not fatal");
    assert!(matches!(report.outcome, PartitionOutcome::Written { .. }));
    assert!(report.level_assignments_written);
    let written = std::fs::read_to_string(&report.graph_path).expect("read written");
    assert!(!written.contains("ghost"), "the broken block is gone");
    assert!(written.contains("d: 3"), "the refined block replaced it");
    // And the written file VALIDATES end-to-end (graph levels consumes it).
    let levels = graph_cmd::graph_levels(&ws.root, "skewed").expect("written graph is valid");
    assert!(levels.partition_error.is_none());
    assert!(
        levels
            .rendered
            .contains("levels source: level_assignments: block"),
        "graph levels renders the override source; got:\n{}",
        levels.rendered
    );
}

/// (f) The consent ladder covers the new block: --dry-run previews the level
/// delta and writes NOTHING; no-TTY without --yes refuses with the file
/// untouched.
#[test]
fn dry_run_and_no_tty_floor_cover_the_level_block() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "skewed", SKEWED_YAML);
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "skewed"),
        SKEWED_COSTS_YAML,
    )
    .expect("write artifact");

    let mut confirm = panic_confirm;
    // --dry-run (wins over --yes): preview carries the level block, file untouched.
    let report = graph_partition(&ws.root, "skewed", &opts(true, true), false, &mut confirm)
        .expect("dry run succeeds");
    assert!(matches!(report.outcome, PartitionOutcome::DryRun));
    assert!(
        report.preview.contains("+level_assignments:") && report.preview.contains("+  d: 3"),
        "dry-run preview carries the proposed block; got:\n{}",
        report.preview
    );
    let after = std::fs::read_to_string(&report.graph_path).expect("read");
    assert_eq!(after, SKEWED_YAML, "dry-run never writes");
    assert!(
        !ws.graphs_dir.join("skewed.yaml.bak").exists(),
        "no bak on dry-run"
    );

    // No TTY, no --yes: loud refusal, file untouched.
    let err = graph_partition(&ws.root, "skewed", &opts(false, false), false, &mut confirm)
        .expect_err("no-TTY without --yes refuses")
        .to_string();
    assert!(
        err.contains("--yes") && err.contains("--dry-run"),
        "got: {err}"
    );
    let after = std::fs::read_to_string(ws.graphs_dir.join("skewed.yaml")).expect("read");
    assert_eq!(after, SKEWED_YAML, "refusal never writes");
}
// ==========================================================================
// Shape-gated level growth. The verb PRE-BANDS the Kahn view (the
// real emitted-shape signal): a multi-group destination refines with
// `LevelGrowth::Deny` (no appended level — +1 level = +1 barrier generation
// per step on the split), a single-group destination keeps today's Allow
// growth. FIXED POINT: single-group pre-band whose GROWN levels band
// multi-group re-refines ONCE with Deny. Every oracle below is hand-derived
// (couplings, marches, gates traced in the comments) — never a self-compare.
// ==========================================================================

/// Two DISCONNECTED chains (multi-group by construction — union-find only
/// merges along trigger edges): a cheap 1 kHz 4-deep chain `f0→f1→f2→f3`
/// (Kahn L0..L3, count 4) and a slow 2-node chain `s0→s1` (30 Hz) whose
/// root `s0` is expensive (100000) — the phase-2 march shape.
const TWO_CHAIN_YAML: &str = r#"name: twochain
prefix: p
nodes:
  - id: f0
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: f1
    type: relay
    inputs:
      - name: inp
        source: f0/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: f2
    type: relay
    inputs:
      - name: inp
        source: f1/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: f3
    type: sink
    inputs:
      - name: inp
        source: f2/out
  - id: s0
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: s1
    type: sink
    inputs:
      - name: inp
        source: s0/out
"#;

/// Costs for [`TWO_CHAIN_YAML`]: the f-chain is 1 kHz (1_000_000 mHz) and
/// cheap; `s0` is expensive and 30 Hz. v1 artifact ⇒ unbounded budget, so
/// the only fusion gates are contiguity/bridging over the refined levels.
const TWO_CHAIN_COSTS_YAML: &str = r#"version: 1
graph: twochain
window_ns: 5000000000
nodes:
  f0: 10
  f1: 10
  f2: 10
  f3: 10
  s0: 100000
  s1: 5
edges:
- producer: f0
  consumer: f1
  rate_mhz: 1000000
- producer: f1
  consumer: f2
  rate_mhz: 1000000
- producer: f2
  consumer: f3
  rate_mhz: 1000000
- producer: s0
  consumer: s1
  rate_mhz: 30000
isolated: []
hop:
  intra_ns: 1000
  cross_ns: 6800
"#;

/// (a) DENY on a multi-group shape: the emitted block carries NO grown level
/// while the NON-growing refinement moves still land, and the notice names
/// the denial.
///
/// Hand-derivation:
/// * Kahn: f0@0,s0@0 / f1@1,s1@1 / f2@2 / f3@3 — count 4. Two weakly-
///   connected components ⇒ the PRE-BAND is 2 groups (multi) ⇒ Deny.
/// * Deny refinement: s0 (rate 30000, cost 100000) has relief (sink f3@L3,
///   rate 1000000, outside its cone) + materiality (100000 > its level
///   mates) ⇒ marches +1 per pass dragging s1: pass 1 → {s0:1,s1:2}, pass 2
///   → {s0:2,s1:3} (both non-growing, APPLIED); pass 3 would put s1 at 4 ≥
///   count 4 ⇒ GROWTH REFUSED — s0 strands at L2 (latency-neutral). Final:
///   f-chain unmoved, s0:2, s1:3 — 2 nodes moved, max level 3 < 4.
/// * Banding over the refined told-levels: (f0,f1),(f1,f2),(f2,f3) fuse the
///   f-chain (owned {0..3}); (s0,s1) fuses (owned {2,3} contiguous) ⇒
///   [grp_f0: f-chain, grp_s0: slow chain] (lead levels 0 and 2).
///
/// Under Allow the same march would append L4 (s0:3, s1:4) — the block
/// equality below catches a variant with no growth gate.
#[test]
fn deny_growth_multi_group_shape_emits_no_grown_level_and_names_the_denial() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "twochain", TWO_CHAIN_YAML);
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "twochain"),
        TWO_CHAIN_COSTS_YAML,
    )
    .expect("write artifact");

    let mut confirm = panic_confirm;
    let report = graph_partition(
        &ws.root,
        "twochain",
        &opts(false, true),
        false,
        &mut confirm,
    )
    .expect("deny-shaped emit succeeds");
    assert!(matches!(report.outcome, PartitionOutcome::Written { .. }));
    assert_eq!(
        shape(&report.groups),
        expect(&[
            ("grp_f0", &["f0", "f1", "f2", "f3"]),
            ("grp_s0", &["s0", "s1"]),
        ]),
        "two components band two groups (multi ⇒ growth denied)"
    );
    assert!(
        report.level_assignments_written,
        "s0/s1 moved ⇒ block written"
    );

    // The written block: the NON-growing part of the march landed (s0:2,
    // s1:3), NO level reaches the Kahn count 4 (an ungated Allow variant
    // writes s0:3, s1:4 and fails here).
    let written = std::fs::read_to_string(&report.graph_path).expect("read written");
    let config = parse_graph_raw(&written).expect("written graph parses");
    let block = config
        .level_assignments
        .as_ref()
        .expect("block present in the written yaml");
    let got: Vec<(String, usize)> = block.iter().map(|(k, v)| (k.clone(), *v)).collect();
    assert_eq!(
        got,
        vec![
            ("f0".to_string(), 0),
            ("f1".to_string(), 1),
            ("f2".to_string(), 2),
            ("f3".to_string(), 3),
            ("s0".to_string(), 2),
            ("s1".to_string(), 3),
        ],
        "non-growing moves land; no level >= the Kahn count 4"
    );
    assert!(
        block.values().all(|&l| l < 4),
        "Deny NEVER emits a grown level"
    );

    // The notice names both the move count and the denial.
    assert!(
        report
            .preview
            .contains("level_assignments written: cost-aware refinement moved 2 node(s)"),
        "preview names the move count; got:\n{}",
        report.preview
    );
    assert!(
        report
            .preview
            .contains("level-count growth denied (multi-process shape)"),
        "the notice states the growth policy; got:\n{}",
        report.preview
    );
    // The fixed point did NOT fire (the pre-band already predicted multi).
    assert!(
        !report.preview.contains("re-refined with growth denied"),
        "pre-band Deny is not the fixed-point path; got:\n{}",
        report.preview
    );

    // End-to-end: the written file is spawner-consumable.
    let levels = graph_cmd::graph_levels(&ws.root, "twochain").expect("levels on written graph");
    assert!(levels.partition_error.is_none());
}

/// A connected FAN-OUT shape shared by pins (b) and (c): root `r` feeds a
/// fast 1 kHz chain `r→f1→f2` AND a slow chain `r→s1→s2` (`s1` expensive,
/// fires 30 Hz). Kahn: r@0 / f1,s1@1 / f2,s2@2 — count 3. The Allow
/// refinement is IDENTICAL under both cost files (only the `r→s1` INCOMING
/// rate differs, and a non-sink's chain rate is its own gating rate): s1
/// marches to the fast sink f2's level dragging s2 → {s1:2, s2:3}, count
/// 3→4 (GROWTH). What differs is the greedy fusion ORDER over the grown
/// levels — the (b)/(c) discriminator.
const FANOUT_YAML: &str = r#"name: fanout
prefix: p
nodes:
  - id: r
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: f1
    type: relay
    inputs:
      - name: inp
        source: r/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: f2
    type: sink
    inputs:
      - name: inp
        source: f1/out
  - id: s1
    type: relay
    inputs:
      - name: inp
        source: r/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: s2
    type: sink
    inputs:
      - name: inp
        source: s1/out
"#;

/// (b) costs: `r→s1` at 30000 mHz — the slow edge is fused LAST, after the
/// fast chain has formed one component, so the grown banding stays SINGLE.
const FANOUT_MONO_COSTS_YAML: &str = r#"version: 1
graph: fanout
window_ns: 5000000000
nodes:
  r: 10
  f1: 10
  f2: 10
  s1: 100000
  s2: 5
edges:
- producer: r
  consumer: f1
  rate_mhz: 1000000
- producer: f1
  consumer: f2
  rate_mhz: 1000000
- producer: r
  consumer: s1
  rate_mhz: 30000
- producer: s1
  consumer: s2
  rate_mhz: 30000
isolated: []
hop:
  intra_ns: 1000
  cross_ns: 6800
"#;

/// (c) costs: `r→s1` at 2000000 mHz (2 kHz arrivals into the 30 Hz
/// decimating relay) — the TOP coupling, so the greedy fuses (r,s1) FIRST.
/// Under Kahn that pair is adjacent ({0,1} — fuses, single group); under the
/// GROWN levels it is gapped ({0,2} — NonContiguous, rejected) and the two
/// halves never reconnect ⇒ the grown banding flips MULTI: the fixed-point
/// shape.
const FANOUT_FLIP_COSTS_YAML: &str = r#"version: 1
graph: fanout
window_ns: 5000000000
nodes:
  r: 10
  f1: 10
  f2: 10
  s1: 100000
  s2: 5
edges:
- producer: r
  consumer: f1
  rate_mhz: 1000000
- producer: f1
  consumer: f2
  rate_mhz: 1000000
- producer: r
  consumer: s1
  rate_mhz: 2000000
- producer: s1
  consumer: s2
  rate_mhz: 30000
isolated: []
hop:
  intra_ns: 1000
  cross_ns: 6800
"#;

/// (b) A single-group shape KEEPS growth (today's Allow behavior).
///
/// Hand-derivation:
/// * PRE-BAND over Kahn (r@0 / f1,s1@1 / f2,s2@2): couplings (r,f1)=(f1,f2)
///   =1M > (r,s1)=(s1,s2)=30k; every merge is contiguous+unbridged ⇒ ONE
///   group ⇒ Allow.
/// * Allow refinement: s1 (gating rate 30k, cost 100000) has relief (sink
///   f2@2, incoming rate 1M, outside its cone {s1,s2}) + materiality
///   (100000 > f1's 10) ⇒ {s1:1→2, s2:2→3} — count 3→4, GROWTH KEPT.
/// * Banding over the GROWN told-levels, greedy order (r,f1),(f1,f2),
///   (r,s1),(s1,s2): {r,f1}{0,1} ✓; +f2 {0,1,2} ✓; +s1 (s1@2 joins the
///   owned span) ✓; +s2 {0,1,2,3} ✓ ⇒ still ONE group ⇒ no fixed point.
#[test]
fn single_group_shape_keeps_growth_in_the_emitted_block() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "fanout", FANOUT_YAML);
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "fanout"),
        FANOUT_MONO_COSTS_YAML,
    )
    .expect("write artifact");

    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "fanout", &opts(false, true), false, &mut confirm)
        .expect("single-group emit succeeds");
    assert!(matches!(report.outcome, PartitionOutcome::Written { .. }));
    assert_eq!(
        shape(&report.groups),
        expect(&[("grp_r", &["r", "f1", "f2", "s1", "s2"])]),
        "single-group destination"
    );
    assert!(
        report.level_assignments_written,
        "s1/s2 moved ⇒ block written"
    );

    // The GROWN block: s2 lands at level 3 == the Kahn count (a 4th level
    // was appended) — growth preserved for the monolith destination.
    let written = std::fs::read_to_string(&report.graph_path).expect("read written");
    let config = parse_graph_raw(&written).expect("written graph parses");
    let block = config
        .level_assignments
        .as_ref()
        .expect("block present in the written yaml");
    let got: Vec<(String, usize)> = block.iter().map(|(k, v)| (k.clone(), *v)).collect();
    assert_eq!(
        got,
        vec![
            ("r".to_string(), 0),
            ("f1".to_string(), 1),
            ("f2".to_string(), 2),
            ("s1".to_string(), 2),
            ("s2".to_string(), 3),
        ],
        "the grown assignment is emitted (s2 at the appended level 3)"
    );
    assert!(
        report
            .preview
            .contains("level_assignments written: cost-aware refinement moved 2 node(s)"),
        "preview names the move count; got:\n{}",
        report.preview
    );
    // No denial clause, no fixed-point line — this is the Allow arm.
    assert!(
        !report.preview.contains("level-count growth denied"),
        "Allow must not claim a denial; got:\n{}",
        report.preview
    );
    assert!(
        !report.preview.contains("re-refined with growth denied"),
        "no fixed point on a stable single-group shape; got:\n{}",
        report.preview
    );
    let levels = graph_cmd::graph_levels(&ws.root, "fanout").expect("levels on written graph");
    assert!(levels.partition_error.is_none());
}

/// (c) THE FIXED POINT: a single-group Kahn pre-band whose GROWN levels band
/// multi-group re-refines ONCE with Deny.
///
/// Hand-derivation (same graph as (b); only the `r→s1` rate differs, 2 kHz):
/// * PRE-BAND over Kahn: greedy order is now (r,s1)=2M FIRST, then (r,f1),
///   (f1,f2)=1M, (s1,s2)=30k. {r,s1} owned {0,1} ✓ fuses; the rest join ⇒
///   ONE group ⇒ Allow (the signal).
/// * Allow refinement: IDENTICAL to (b) — s1's chain rate is its OWN gating
///   rate (30k; the 2 kHz is only its incoming arrivals — sink-only
///   inheritance), so {s1:2, s2:3}, count 3→4.
/// * Banding the GROWN levels: (r,s1) FIRST again, but now owned {0,2} —
///   level 1 is f1's (foreign) ⇒ NonContiguous ⇒ REJECTED; then {r,f1}
///   {0,1} ✓, +f2 ✓, and (s1,s2) {2,3} ✓ — the halves never reconnect ⇒
///   TWO groups. Growth falsified its own single-group signal.
/// * FIXED POINT: re-refine with Deny — s1's only cascade is growing
///   (s2 would hit level 3 ≥ count 3) ⇒ refused ⇒ Deny output == Kahn ⇒ NO
///   block, and the banding over Kahn is the pre-band's ONE group again.
#[tracing_test::traced_test]
#[test]
fn fixed_point_denies_growth_when_grown_banding_flips_multi_group() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "fanout", FANOUT_YAML);
    std::fs::write(
        graph_cmd::default_artifact_path(&ws.root, "fanout"),
        FANOUT_FLIP_COSTS_YAML,
    )
    .expect("write artifact");

    let mut confirm = panic_confirm;
    let report = graph_partition(&ws.root, "fanout", &opts(false, true), false, &mut confirm)
        .expect("fixed-point emit succeeds");
    assert!(matches!(report.outcome, PartitionOutcome::Written { .. }));
    assert_eq!(
        shape(&report.groups),
        expect(&[("grp_r", &["r", "f1", "f2", "s1", "s2"])]),
        "the Deny fixed point restores the single-group banding"
    );
    // Deny refused the only cascade ⇒ refined == Kahn ⇒ NO block.
    assert!(
        !report.level_assignments_written,
        "the denied refinement equals Kahn — no block"
    );
    assert!(!report.removed_level_assignments);
    let written = std::fs::read_to_string(&report.graph_path).expect("read written");
    assert!(
        !written.contains("level_assignments"),
        "no grown (or any) assignment is emitted; got:\n{written}"
    );
    // The operator sees the flip: preview line + the loud info.
    assert!(
        report.preview.contains("re-refined with growth denied"),
        "the preview surfaces the fixed point; got:\n{}",
        report.preview
    );
    assert!(
        logs_contain("level-growth fixed point"),
        "the fixed point is announced loudly"
    );
    let levels = graph_cmd::graph_levels(&ws.root, "fanout").expect("levels on written graph");
    assert!(levels.partition_error.is_none());
}

/// (d) Determinism on the Deny-shaped fixture: two FRESH identical
/// workspaces produce byte-identical written yaml, and a re-run over the
/// written file is `Unchanged` (no backup churn).
#[test]
fn deny_shaped_verb_is_deterministic_across_fresh_workspaces() {
    let run = |root: &Path| -> String {
        let ws = setup_workspace(root);
        write_graph(&ws, "twochain", TWO_CHAIN_YAML);
        std::fs::write(
            graph_cmd::default_artifact_path(&ws.root, "twochain"),
            TWO_CHAIN_COSTS_YAML,
        )
        .expect("write artifact");
        let mut confirm = panic_confirm;
        let report = graph_partition(
            &ws.root,
            "twochain",
            &opts(false, true),
            false,
            &mut confirm,
        )
        .expect("deny-shaped emit succeeds");
        assert!(matches!(report.outcome, PartitionOutcome::Written { .. }));
        std::fs::read_to_string(&report.graph_path).expect("read written")
    };

    let tmp_a = tempfile::tempdir().unwrap();
    let tmp_b = tempfile::tempdir().unwrap();
    let written_a = run(tmp_a.path());
    let written_b = run(tmp_b.path());
    assert_eq!(
        written_a, written_b,
        "the Deny-gated derivation is byte-reproducible across fresh workspaces"
    );

    // Idempotence over the written file (workspace A).
    let ws_a_graph = tmp_a.path().join("part_ws/graphs/twochain.yaml");
    assert!(ws_a_graph.exists(), "fixture path sane");
    let mut confirm = panic_confirm;
    let report2 = graph_partition(
        &tmp_a.path().join("part_ws"),
        "twochain",
        &opts(false, true),
        false,
        &mut confirm,
    )
    .expect("re-run succeeds");
    assert!(
        matches!(report2.outcome, PartitionOutcome::Unchanged),
        "same costs ⇒ same gate ⇒ same file; got {:?}",
        report2.outcome
    );
}

// ==========================================================================
// The verb never WRITES a partition that splits a `block` edge.
// ==========================================================================

/// Turn a scaffolded node type's `#[input(trigger)]` into a `block` input by
/// editing its SOURCE — which is exactly what `source_entry_infos` parses, so
/// this exercises the real source-parsing plumbing rather than a hand-built `InputMeta`.
fn make_input_block(ws: &workspace::CerulionWorkspace, node_type: &str) {
    let lib = ws.nodes_dir.join(node_type).join("src/lib.rs");
    let src = std::fs::read_to_string(&lib).expect("read node source");
    assert!(
        src.contains("#[input(trigger)]"),
        "fixture drift: `{node_type}` no longer scaffolds a trigger input"
    );
    std::fs::write(
        &lib,
        src.replace(
            "#[input(trigger)]",
            "#[input(trigger, backpressure = block, depth = 4)]",
        ),
    )
    .expect("write node source");
}

/// The graph a `graph partition` run would otherwise split: `n0 -> n1 -> n2`
/// where `n2`'s input is `block`. Writes the partition with `--yes` and pins
/// that the PERSISTED block co-locates `n1` with `n2`.
#[test]
fn the_verb_never_writes_a_partition_that_splits_a_block_edge() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    make_input_block(&ws, "sink");
    write_graph(&ws, "chain", CHAIN_YAML);

    let mut confirm = panic_confirm; // --yes never consults it
    let report = graph_partition(&ws.root, "chain", &opts(false, true), false, &mut confirm)
        .expect("write the derived partition");
    assert!(
        matches!(report.outcome, PartitionOutcome::Written { .. }),
        "expected a write; got {:?}",
        report.outcome
    );

    // The oracle is the WRITTEN file, re-parsed — not the in-memory report.
    let written =
        parse_graph_raw(&std::fs::read_to_string(ws.graphs_dir.join("chain.yaml")).expect("read"))
            .expect("re-parse");
    let owner = |node: &str| -> String {
        written
            .process_groups
            .iter()
            .find(|(_, m)| m.iter().any(|x| x == node))
            .map(|(n, _)| n.clone())
            .unwrap_or_else(|| panic!("{node} is unplaced in {:?}", written.process_groups))
    };
    assert_eq!(
        owner("n1"),
        owner("n2"),
        "the `block` producer n1 and its `block` consumer n2 must be persisted in ONE \
         group; got {:?}",
        written.process_groups
    );

    // And the partition it wrote passes the plan-time gate it will be run
    // through — a verb that wrote a refusable block would be worse than one
    // that wrote nothing. `graph levels` is the read-only surface of exactly
    // that gate (`validate_partition`), so this also pins that the refusal
    // reaches the INSPECTION verb and not only the spawn path.
    let report = graph_cmd::graph_levels(&ws.root, "chain").expect("levels");
    assert_eq!(
        report.partition_error, None,
        "the written partition must be spawner-consumable; got: {:?}",
        report.partition_error
    );
}

/// Cross-process credit: the consent PREVIEW must not claim a split
/// flow "would refuse to build" — it says co-location is a latency preference
/// and names the one split shape that IS legal.
///
/// User-facing text on the surface an operator reads before consenting, and
/// a refusal claim there is FALSE under cross-process credit.
#[test]
fn the_consent_preview_frames_co_location_as_a_preference_not_a_refusal() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    make_input_block(&ws, "sink");
    write_graph(&ws, "chain", CHAIN_YAML);

    let seen = std::cell::RefCell::new(String::new());
    let mut confirm = |preview: &str| -> CliResult<bool> {
        seen.borrow_mut().push_str(preview);
        Ok(false) // decline: this arm is about the TEXT, not the write
    };
    let _ = graph_partition(&ws.root, "chain", &opts(false, false), true, &mut confirm);
    let preview = seen.borrow().clone();
    assert!(
        preview.contains("`block` co-location:"),
        "the fixture must reach the co-location section; preview:\n{preview}"
    );
    assert!(
        preview.contains("co-located because crossing a process boundary costs a real hop"),
        "the preview must frame it as a PREFERENCE; preview:\n{preview}"
    );
    assert!(
        preview.contains("one in-graph producer, no non-`block` consumers"),
        "and name the split shape that IS legal; preview:\n{preview}"
    );
    assert!(
        !preview.contains("a split flow would refuse to build"),
        "the split-flow-refuses claim is FALSE and must be gone; preview:\n{preview}"
    );
}

/// Writing the derived partition over a
/// hand-written CREDITED split is a real loss, and the preview says so.
///
/// The derivation always co-locates, so `graph partition` silently replaced a
/// deliberate, legal, credited split with a co-located one. `.bak` and the
/// diff mitigate it; neither NAMES what was given up, and `--yes` skips the
/// diff entirely. This is also the disposition of the plan's "`graph
/// partition` accept-with-credit arm": the verb DERIVES, so what it does with
/// a hand-written credited split is warn before overwriting it.
#[test]
fn the_preview_warns_before_overwriting_a_hand_written_creditable_split() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    make_input_block(&ws, "sink");
    let split = CHAIN_YAML.replace(
        "name: chain\n",
        "name: chain\nprocess_groups:\n  front: [n0, n1]\n  back: [n2]\n",
    );
    write_graph(&ws, "chain", &split);

    let seen = std::cell::RefCell::new(String::new());
    let mut confirm = |preview: &str| -> CliResult<bool> {
        seen.borrow_mut().push_str(preview);
        Ok(false)
    };
    let _ = graph_partition(&ws.root, "chain", &opts(false, false), true, &mut confirm);
    let preview = seen.borrow().clone();
    for needle in [
        "deliberately SPLITS",
        "cross-process word",
        // CHAIN_YAML's sink input is `inp` (the `trigger_in` name belongs to
        // the mp_auto_partition fixture) — and the edge must be named in FULL,
        // topic + consumer + both groups, or the warning cannot tell an
        // operator WHICH choice they are about to lose.
        "/p/n1/out -> n2.inp (front -> back)",
        "`.bak`",
    ] {
        assert!(
            preview.contains(needle),
            "the overwrite warning must name `{needle}`; preview:\n{preview}"
        );
    }
    // ANTI-TAUTOLOGY: the SAME verb on the CO-LOCATED graph warns nothing —
    // otherwise this arm would pass against a preview that always warns.
    let tmp2 = tempfile::tempdir().unwrap();
    let ws2 = setup_workspace(tmp2.path());
    make_input_block(&ws2, "sink");
    write_graph(&ws2, "chain", CHAIN_YAML);
    let seen2 = std::cell::RefCell::new(String::new());
    let mut confirm2 = |preview: &str| -> CliResult<bool> {
        seen2.borrow_mut().push_str(preview);
        Ok(false)
    };
    let _ = graph_partition(&ws2.root, "chain", &opts(false, false), true, &mut confirm2);
    // The control is only meaningful if `confirm` actually RAN: with
    // `is_tty=false` the verb returns before ever calling it, so an empty
    // `seen2` would satisfy the absence assertion for the wrong reason.
    assert!(
        !seen2.borrow().is_empty(),
        "the control must have RENDERED a preview, or its absence assertion is vacuous"
    );
    assert!(
        !seen2.borrow().contains("deliberately SPLITS"),
        "an unpartitioned graph destroys nothing and must not warn; preview:\n{}",
        seen2.borrow()
    );
}

/// The other direction of the same seam — INVERTED by cross-process credit: a
/// HAND-WRITTEN partition that splits a SINGLE-PRODUCER, ALL-`block` edge is
/// ACCEPTED by `graph levels`, because the supervisor mints that edge a
/// cross-process credit word.
///
/// The verdict must reach the read-only INSPECTION verb, not only the spawn
/// path: `graph levels` and `graph run`'s pre-flight share `validate_partition`,
/// and an operator asks `levels` precisely to find out whether a hand-written
/// block will run.
#[test]
fn graph_levels_accepts_a_hand_written_split_of_a_creditable_block_edge() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    make_input_block(&ws, "sink");
    let split = CHAIN_YAML.replace(
        "name: chain\n",
        "name: chain\nprocess_groups:\n  front: [n0, n1]\n  back: [n2]\n",
    );
    assert!(
        split.contains("process_groups:"),
        "fixture must carry a partition"
    );
    write_graph(&ws, "chain", &split);

    let report = graph_cmd::graph_levels(&ws.root, "chain").expect("levels");
    assert_eq!(
        report.partition_error, None,
        "a single-producer all-`block` split edge carries a credit word and must pass; got: {:?}",
        report.partition_error
    );
    // PRECONDITION: the split really is the creditable shape, or a verdict
    // saying "0 creditable" would satisfy every assertion below by accident.
    assert!(
        report
            .rendered
            .contains("rank 0  front  levels 0-1  nodes: n0, n1"),
        "the bands must place n0/n1 in `front`; rendered:\n{}",
        report.rendered
    );
    assert!(
        report
            .rendered
            .contains("rank 1  back  levels 2  nodes: n2"),
        "and n2 in `back` — a DIFFERENT group, or nothing is split; rendered:\n{}",
        report.rendered
    );
    // THE line. Asserted as ONE clause, not a bag of substrings: the verdict
    // an operator reads is the whole sentence, and a partial match cannot tell
    // "1 creditable" from "0 creditable" or catch a mangled join.
    assert!(
        report.rendered.contains(
            "partition: spawner-consumable (1 split `block` edge(s) creditable: \
             /p/n1/out -> n2.inp (front -> back); judged on source metadata — \
             `graph run` re-checks the built cdylibs and refuses on drift)"
        ),
        "the levels verdict must NAME the creditable edge and carry the stale-build \
         caveat; rendered:\n{}",
        report.rendered
    );
}

/// The twin that did NOT flip: a MULTI-PRODUCER split `block` edge is still
/// refused by `graph levels`, with the same needle set the arm pinned
/// plus the reason it could not be credited.
///
/// `multi_publisher_topics:` + absolute `topic:` overrides are the only way to
/// give one topic two in-graph producers — `GraphTopology::build` refuses a
/// second producer otherwise.
#[test]
fn graph_levels_still_refuses_a_split_multi_producer_block_edge() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    make_input_block(&ws, "sink");
    write_graph(
        &ws,
        "mp",
        r#"name: mp
prefix: p
multi_publisher_topics:
  - /shared
process_groups:
  front: [n0, n0b]
  back: [n2]
nodes:
  - id: n0
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
        topic: /shared
  - id: n0b
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
        topic: /shared
  - id: n2
    type: sink
    inputs:
      - name: inp
        source: /shared
"#,
    );

    let report = graph_cmd::graph_levels(&ws.root, "mp").expect("levels renders even on refusal");
    let err = report
        .partition_error
        .expect("a split MULTI-PRODUCER block edge must still be refused");
    for needle in [
        "n0",
        "n2.inp",
        "front",
        "back",
        "--single-process",
        "has 2 in-graph producers",
    ] {
        assert!(
            err.contains(needle),
            "the refusal must name `{needle}`; got: {err}"
        );
    }
}

/// ANTI-TAUTOLOGY: the same chain with NO `block` input writes the untouched
/// process-per-node baseline (`n1` and `n2` in DIFFERENT groups), so the arm
/// above cannot pass against a verb that fuses everything.
#[test]
fn the_verb_without_a_block_input_still_writes_the_split_baseline() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);

    let mut confirm = panic_confirm;
    graph_partition(&ws.root, "chain", &opts(false, true), false, &mut confirm).expect("write");
    let written =
        parse_graph_raw(&std::fs::read_to_string(ws.graphs_dir.join("chain.yaml")).expect("read"))
            .expect("re-parse");
    assert_eq!(
        shape(&written.process_groups),
        expect(&[
            ("grp_n0", &["n0"]),
            ("grp_n1", &["n1"]),
            ("grp_n2", &["n2"])
        ]),
        "no block ⇒ the documented process-per-node baseline, unchanged"
    );
}
