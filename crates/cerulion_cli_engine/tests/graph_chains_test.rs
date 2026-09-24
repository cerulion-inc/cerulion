// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion graph chains <graph>` — engine-level oracle tests.
//!
//! Pure (the census is static analysis over YAML plus source metadata), so
//! parallel-safe. Every report is asserted against a HAND-WRITTEN string, never
//! a self-compare, and the node types are scaffolded through the real
//! `node_cmd` helpers so the policies come from actual source parsing, exactly
//! as the verb reads them in production.
//!
//! The per-rule oracles live beside the analysis
//! (`cerulion_core/tests/chain_census_test.rs`); this file pins the verb: what
//! the operator sees, and that the declarations the verb reads from source
//! reach the census at all.

use std::path::Path;

use cerulion_cli_engine::{graph_cmd, node_cmd, workspace};
use cerulion_core::MacroPolicy;

/// Scaffold the node types every fixture here wires:
/// * `src`   — `period_ms = 10`, one output `out`.
/// * `relay` — data-triggered on `inp`, one output `out`.
/// * `sink`  — data-triggered on `inp`, no outputs.
fn setup_workspace(root: &Path) -> workspace::CerulionWorkspace {
    let ws = workspace::workspace_create(root, "chains_ws").expect("workspace");
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

    ws
}

fn write_graph(ws: &workspace::CerulionWorkspace, name: &str, yaml: &str) {
    std::fs::write(ws.graphs_dir.join(format!("{name}.yaml")), yaml).expect("write graph yaml");
}

/// The 3-node chain every arm starts from.
const CHAIN_YAML: &str = r#"prefix: cw
nodes:
  - id: cam
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: rect
    type: relay
    inputs:
      - name: inp
        source: cam/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: det
    type: sink
    inputs:
      - name: inp
        source: rect/out
"#;

#[test]
fn a_linear_chain_renders_the_exact_report() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);

    let report = graph_cmd::graph_chains(&ws.root, "chain").expect("chains");
    assert_eq!(
        report,
        "\
graph: chain  prefix: cw
levels source: derived (trigger-aware Kahn levelization)
colocation: one process (this graph declares no process_groups:; \
`cerulion graph run` may derive a partition, and each run writes its own census into its \
run directory)
nodes: 3  levels: 3  consumer edges: 2
fusable: 1 chain(s), 2 hop(s)  queued: 0 edge(s)

chain 0  levels 0-2  3 nodes, 2 hop(s)
  cam -> rect  on /cw/cam/out
  rect -> det  on /cw/rect/out
"
    );
}

#[test]
fn a_fan_out_renders_its_reason_for_every_edge_it_refuses() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = setup_workspace(tmp.path());
    write_graph(
        &ws,
        "fanout",
        r#"prefix: cw
nodes:
  - id: cam
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: rect
    type: sink
    inputs:
      - name: inp
        source: cam/out
  - id: view
    type: sink
    inputs:
      - name: inp
        source: cam/out
"#,
    );

    let report = graph_cmd::graph_chains(&ws.root, "fanout").expect("chains");
    assert_eq!(
        report,
        "\
graph: fanout  prefix: cw
levels source: derived (trigger-aware Kahn levelization)
colocation: one process (this graph declares no process_groups:; \
`cerulion graph run` may derive a partition, and each run writes its own census into its \
run directory)
nodes: 3  levels: 2  consumer edges: 2
fusable: 0 chain(s), 0 hop(s)  queued: 2 edge(s)

queued edges by reason:
  fan-out  2

queued edges:
  /cw/cam/out  cam -> rect.inp
    fan-out: the topic has 2 consumers, and a chain ends at a fan-out
  /cw/cam/out  cam -> view.inp
    fan-out: the topic has 2 consumers, and a chain ends at a fan-out
"
    );
}

#[test]
fn a_declared_partition_splits_the_chain_at_the_group_boundary() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = setup_workspace(tmp.path());
    let mut yaml = CHAIN_YAML.to_string();
    yaml.push_str("process_groups:\n  g1: [cam]\n  g2: [rect, det]\n");
    write_graph(&ws, "split", &yaml);

    let report = graph_cmd::graph_chains(&ws.root, "split").expect("chains");
    assert_eq!(
        report,
        "\
graph: split  prefix: cw
levels source: derived (trigger-aware Kahn levelization)
colocation: process_groups: 2 group(s) (g1, g2)
nodes: 3  levels: 3  consumer edges: 2
fusable: 1 chain(s), 1 hop(s)  queued: 1 edge(s)

chain 0  levels 1-2  2 nodes, 1 hop(s)
  rect -> det  on /cw/rect/out

queued edges by reason:
  separate-processes  1

queued edges:
  /cw/cam/out  cam -> rect.inp
    separate-processes: the producer runs in group 'g1' and the consumer in group 'g2'
"
    );
}

#[test]
fn a_rate_cap_declared_in_node_source_reaches_the_census() {
    // The verb reads every declaration from SOURCE, so a rule whose value this
    // path dropped would be decidable in principle and never decided. The cap
    // is written into the scaffolded node exactly as an author would.
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = setup_workspace(tmp.path());
    let lib = ws.nodes_dir.join("relay").join("src").join("lib.rs");
    let source = std::fs::read_to_string(&lib).expect("read relay source");
    assert!(
        source.contains("#[cerulion_node]\n"),
        "the scaffold emits a bare attribute for a data-triggered node; got:\n{source}"
    );
    std::fs::write(
        &lib,
        source.replace("#[cerulion_node]\n", "#[cerulion_node(throttle_ms = 5)]\n"),
    )
    .expect("write relay source");
    write_graph(&ws, "capped", CHAIN_YAML);

    let report = graph_cmd::graph_chains(&ws.root, "capped").expect("chains");
    assert!(
        report.contains(
            "    throttle: the consumer declares `throttle_ms = 5`, and a direct call has \
             nowhere to defer to"
        ),
        "the rate cap must refuse the hop into the capped node; got:\n{report}"
    );
    // The cap refuses the hop INTO `rect`, not the hop out of it: a capped
    // node is a fine chain head.
    assert!(
        report.contains("chain 0  levels 1-2  2 nodes, 1 hop(s)\n  rect -> det  on /cw/rect/out"),
        "got:\n{report}"
    );
}

#[test]
fn a_missing_graph_is_a_loud_refusal_with_no_partial_report() {
    let tmp = tempfile::tempdir().expect("tempdir");
    let ws = setup_workspace(tmp.path());
    let err = graph_cmd::graph_chains(&ws.root, "absent").expect_err("must refuse");
    assert!(
        err.to_string().contains("absent"),
        "the refusal must name the graph; got: {err}"
    );
}
