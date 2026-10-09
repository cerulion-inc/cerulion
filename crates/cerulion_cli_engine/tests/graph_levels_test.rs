// SPDX-License-Identifier: AGPL-3.0-only
//! `cerulion graph levels <graph>` — engine-level oracle tests.
//!
//! Pure (NO iceoryx2 — the levelization is static analysis over YAML + source
//! metadata), so parallel-safe. Every rendered report is asserted against a
//! HAND-WRITTEN oracle string (never a self-compare); the fixture test covers
//! `cerulion_core/fixtures/test_graph.yaml` end-to-end with all four trigger
//! policies. Node types are scaffolded via the real `node_cmd` helpers so the
//! policy metadata comes from ACTUAL source parsing (`parse_node_metadata`),
//! the same path the verb uses in production.
//!
//! The report CLOSES with the chain-fusion census, so every oracle here pins
//! it too: which linear single-consumer trigger chains could run as one fused
//! call sequence, and for every other consumer edge the reason it keeps the
//! queued path. That half is pinned in the SAME strings as the levels half on
//! purpose, because a fused hop is a level boundary the executor would not
//! have to cross, and an oracle that split them could let the two disagree.
//! The per-rule oracles live beside the analysis
//! (`cerulion_core/tests/chain_census_test.rs`).

use std::path::Path;

use cerulion_cli_engine::{graph_cmd, node_cmd, workspace};
use cerulion_core::MacroPolicy;

/// Scaffold a workspace with the standard test node types:
/// * `src`    — `period_ms = 10`, one output `out`.
/// * `relay`  — DataTrigger on input `inp`, one output `out`.
/// * `sink`   — DataTrigger on input `inp`, no outputs.
/// * `join`   — `sync_window_ms = 25`, trigger inputs `a` + `b`, output `out`
///   (sync aligns only `#[input(trigger)]`-marked ports).
fn setup_workspace(root: &Path) -> workspace::CerulionWorkspace {
    let ws = workspace::workspace_create(root, "levels_ws").expect("workspace");
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
        true, // trigger — flips the policy to DataTrigger("inp")
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

    // A sync node aligns ONLY `#[input(trigger)]`-marked inputs, so
    // the aligned ports are declared as triggers (also what the macro REQUIRES
    // for compilable `sync_window_ms` source — ≥2 trigger inputs).
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
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "join",
        "out",
        Some("std_msgs::Int32"),
        true,
        false,
    )
    .expect("join out");
    node_cmd::node_modify_set_sync(&ws.nodes_dir, "join", 25).expect("join sync");

    ws
}

fn write_graph(ws: &workspace::CerulionWorkspace, name: &str, yaml: &str) {
    std::fs::write(ws.graphs_dir.join(format!("{name}.yaml")), yaml).expect("write graph yaml");
}

// ==========================================================================
// Oracle: 3-node chain.
// ==========================================================================

#[test]
fn chain_levels_exact_oracle() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(
        &ws,
        "chain",
        r#"
name: chain
prefix: p
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
"#,
    );

    let report = graph_cmd::graph_levels(&ws.root, "chain").expect("levels");
    assert!(
        report.partition_error.is_none(),
        "no process_groups => no partition error"
    );
    let report = report.rendered;
    let expected = "\
graph: chain  prefix: p
levels source: derived (trigger-aware Kahn levelization)
nodes: 3  levels: 3
level 0: n0 [period(10ms)]
  -> /p/n0/out  n0 -> n1 (level 1)
level 1: n1 [data(inp)]
  -> /p/n1/out  n1 -> n2 (level 2)
level 2: n2 [data(inp)]
chains: 1 fusable, 2 hop(s), 0 of 2 consumer edge(s) queued
  colocation: one process (this graph declares no process_groups:; \
`cerulion graph run` may derive a partition, and each run writes its own census into its \
run directory)
  chain 0  levels 0-2  3 nodes, 2 hop(s)
    n0 -> n1  on /p/n0/out
    n1 -> n2  on /p/n1/out
";
    assert_eq!(report, expected, "chain report must match the hand oracle");
}

// ==========================================================================
// Oracle: diamond (wide level + sync fan-in).
// ==========================================================================

#[test]
fn diamond_wide_level_and_edges_oracle() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(
        &ws,
        "diamond",
        r#"
name: diamond
prefix: p
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
    type: relay
    inputs:
      - name: inp
        source: n0/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: n3
    type: join
    inputs:
      - name: a
        source: n1/out
      - name: b
        source: n2/out
    outputs:
      - name: out
        schema: std_msgs/Int32
"#,
    );

    let report = graph_cmd::graph_levels(&ws.root, "diamond")
        .expect("levels")
        .rendered;
    // n1 AND n2 share the wide level 1; the Sync join triggers on BOTH inputs.
    let expected = "\
graph: diamond  prefix: p
levels source: derived (trigger-aware Kahn levelization)
nodes: 4  levels: 3
level 0: n0 [period(10ms)]
  -> /p/n0/out  n0 -> n1 (level 1)
  -> /p/n0/out  n0 -> n2 (level 1)
level 1: n1 [data(inp)], n2 [data(inp)]
  -> /p/n1/out  n1 -> n3 (level 2)
  -> /p/n2/out  n2 -> n3 (level 2)
level 2: n3 [sync(25ms)]
chains: 0 fusable, 0 hop(s), 4 of 4 consumer edge(s) queued
  colocation: one process (this graph declares no process_groups:; \
`cerulion graph run` may derive a partition, and each run writes its own census into its \
run directory)
  queued by reason: consumer-policy 2, fan-out 2
  queued  /p/n0/out  n0 -> n1.inp
    fan-out: the topic has 2 consumers, and a chain ends at a fan-out
  queued  /p/n0/out  n0 -> n2.inp
    fan-out: the topic has 2 consumers, and a chain ends at a fan-out
  queued  /p/n1/out  n1 -> n3.a
    consumer-policy: the consumer fires on sync, not on this frame
  queued  /p/n2/out  n2 -> n3.b
    consumer-policy: the consumer fires on sync, not on this frame
";
    assert_eq!(
        report, expected,
        "diamond report must match the hand oracle"
    );
}

// ==========================================================================
// process_groups band mapping.
// ==========================================================================

#[test]
fn process_groups_band_mapping_oracle() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(
        &ws,
        "banded",
        r#"
name: banded
prefix: p
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
process_groups:
  P0: [n0, n1]
  P1: [n2]
"#,
    );

    let report = graph_cmd::graph_levels(&ws.root, "banded").expect("levels");
    // The exits-0 control: a VALID declared partition carries NO error — the
    // binary maps `partition_error: None` to a zero exit.
    assert!(
        report.partition_error.is_none(),
        "a valid partition must carry no partition_error (exit 0)"
    );
    let report = report.rendered;
    let expected = "\
graph: banded  prefix: p
levels source: derived (trigger-aware Kahn levelization)
nodes: 3  levels: 3
level 0: n0 [period(10ms)]
  -> /p/n0/out  n0 -> n1 (level 1)
level 1: n1 [data(inp)]
  -> /p/n1/out  n1 -> n2 (level 2)
level 2: n2 [data(inp)]
process groups: 2
  rank 0  P0  levels 0-1  nodes: n0, n1
  rank 1  P1  levels 2  nodes: n2
  partition: spawner-consumable
chains: 1 fusable, 1 hop(s), 1 of 2 consumer edge(s) queued
  colocation: the 2 declared process group(s) (P0, P1)
  chain 0  levels 0-1  2 nodes, 1 hop(s)
    n0 -> n1  on /p/n0/out
  queued by reason: separate-processes 1
  queued  /p/n1/out  n1 -> n2.inp
    separate-processes: the producer runs in group 'P0' and the consumer in group 'P1'
";
    assert_eq!(
        report, expected,
        "banded report must match the hand oracle (contiguous bands + verdict)"
    );
}

#[test]
fn bridged_partition_reported_invalid_naming_bridge() {
    // 4-node chain, groups {X:[n0,n2,n3], Y:[n1]}: n1 is a FOREIGN bridge
    // inside X's non-contiguous band. The report must render (read-only) and
    // carry the INVALID verdict naming the bridge node — the mapping itself
    // shows the non-contiguous ownership.
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(
        &ws,
        "bridged",
        r#"
name: bridged
prefix: p
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
    type: relay
    inputs:
      - name: inp
        source: n1/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: n3
    type: sink
    inputs:
      - name: inp
        source: n2/out
process_groups:
  X: [n0, n2, n3]
  Y: [n1]
"#,
    );

    let report = graph_cmd::graph_levels(&ws.root, "bridged")
        .expect("the report still renders in full (inspect-then-fail)");
    // The NONZERO path: the engine returns the partition diagnostic in
    // `partition_error`; the binary prints the report THEN maps this to a
    // CliError::Validation (nonzero exit) — CI-gateable on partition validity.
    let diagnostic = report
        .partition_error
        .as_deref()
        .expect("an invalid partition must set partition_error (nonzero exit)");
    assert!(
        diagnostic.contains("bridge") && diagnostic.contains("'n1'"),
        "partition_error must carry the partition diagnostic naming the bridge; got: {diagnostic}"
    );
    let report = report.rendered;
    assert!(
        report.contains("rank 0  X  levels 0,2,3 (non-contiguous)  nodes: n0, n2, n3"),
        "X's non-contiguous ownership must be marked; got:\n{report}"
    );
    assert!(
        report.contains("rank 1  Y  levels 1  nodes: n1"),
        "Y's single-level band must render; got:\n{report}"
    );
    assert!(
        report.contains("partition: INVALID"),
        "the spawner-consumability verdict must be INVALID; got:\n{report}"
    );
    assert!(
        report.contains("bridge") && report.contains("'n1'"),
        "the diagnostic must NAME the bridge node n1; got:\n{report}"
    );
}

#[test]
fn non_contiguous_direct_edge_partition_reported_invalid() {
    // The LYING-VERDICT regression. A source `d`@L0 has a DIRECT edge
    // to consumer `c` which ALSO consumes a 4-deep sibling chain
    // (`s0→s1→s2→s3`, s3@L3), so `c`@L4. Grouping {G_dc:[d,c], G_chain:[s0..s3]}
    // gives G_dc the NON-CONTIGUOUS band {0,4}. A check keyed on the
    // re-levelization bijection alone prints the "0,4 (non-contiguous)" row
    // yet verdicts "spawner-consumable" — the direct d→c edge keeps G_dc's
    // bijection intact, so a holey `check_group` passes it. The verdict must
    // be INVALID with a nonzero exit (the report still renders in full — inspect-then-fail).
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(
        &ws,
        "noncontig",
        r#"
name: noncontig
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
process_groups:
  G_dc: [d, c]
  G_chain: [s0, s1, s2, s3]
"#,
    );

    let report = graph_cmd::graph_levels(&ws.root, "noncontig")
        .expect("the report still renders in full (inspect-then-fail)");
    // NONZERO path: the engine surfaces the contiguity diagnostic in
    // `partition_error` (the binary maps this to a nonzero exit).
    let diagnostic = report
        .partition_error
        .as_deref()
        .expect("a non-contiguous partition must set partition_error (nonzero exit)");
    assert!(
        diagnostic.contains("non-adjacent global DAG levels")
            && diagnostic.contains("[0, 4]")
            && diagnostic.contains("CONTIGUOUS-split partition")
            && diagnostic.contains("G_dc"),
        "partition_error must carry the contiguity diagnostic naming the group + \
         gapped band {{0,4}}; got: {diagnostic}"
    );
    let report = report.rendered;
    // The mapping row marks the non-contiguous ownership...
    assert!(
        report.contains("rank 0  G_dc  levels 0,4 (non-contiguous)  nodes: d, c"),
        "G_dc's non-contiguous ownership must be marked; got:\n{report}"
    );
    // ...and the anti-tautology sibling band is CONTIGUOUS + present.
    assert!(
        report.contains("rank 1  G_chain  levels 0-3  nodes: s0, s1, s2, s3"),
        "G_chain's contiguous 0-3 band must render; got:\n{report}"
    );
    // ...and the verdict FLIPS to INVALID (the lying-verdict fix).
    assert!(
        report.contains("partition: INVALID"),
        "the spawner-consumability verdict must be INVALID; got:\n{report}"
    );
    assert!(
        !report.contains("partition: spawner-consumable"),
        "the verdict must NOT lie 'spawner-consumable'; got:\n{report}"
    );
}

// ==========================================================================
// The reference fixture graph (all four trigger policies).
// ==========================================================================

#[test]
fn fixture_graph_levels_oracle() {
    // cerulion_core/fixtures/test_graph.yaml: camera(Period), detector
    // (DataTrigger via `topic: /detections` override), imu(Period),
    // fusion(Sync), tracker(Period, disconnected), diagnostics(External,
    // absolute producer-less source). Node types are scaffolded to carry
    // exactly the policies the fixture's comments declare.
    let tmp = tempfile::tempdir().unwrap();
    let ws = workspace::workspace_create(tmp.path(), "fixture_ws").expect("workspace");
    let cargo_toml = ws.root.join("Cargo.toml");

    // camera / imu / tracker: Period (the node_create default 100ms).
    for t in ["camera", "imu", "tracker"] {
        node_cmd::node_create(&ws.nodes_dir, &cargo_toml, t, None).expect("create");
    }
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "camera",
        "image",
        Some("sensor_msgs::Image"),
        true,
        false,
    )
    .expect("camera image");
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "imu",
        "data",
        Some("sensor_msgs::Imu"),
        true,
        false,
    )
    .expect("imu data");

    // detector: DataTrigger on `image`.
    node_cmd::node_create(&ws.nodes_dir, &cargo_toml, "detector", None).expect("create");
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "detector",
        "image",
        Some("sensor_msgs::Image"),
        false,
        true,
    )
    .expect("detector image");
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "detector",
        "detections",
        Some("geometry_msgs::PoseArray"),
        true,
        false,
    )
    .expect("detector detections");

    // fusion: Sync(50) over image + imu — both TRIGGER-marked (sync
    // aligns only `#[input(trigger)]` ports; also the macro's compile
    // requirement for `sync_window_ms`).
    node_cmd::node_create(&ws.nodes_dir, &cargo_toml, "fusion", None).expect("create");
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "fusion",
        "image",
        Some("sensor_msgs::Image"),
        false,
        true,
    )
    .expect("fusion image");
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "fusion",
        "imu",
        Some("sensor_msgs::Imu"),
        false,
        true,
    )
    .expect("fusion imu");
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "fusion",
        "state",
        Some("nav_msgs::Odometry"),
        true,
        false,
    )
    .expect("fusion state");
    node_cmd::node_modify_set_sync(&ws.nodes_dir, "fusion", 50).expect("fusion sync");

    // diagnostics: External, one input from the absolute external source.
    node_cmd::node_create(&ws.nodes_dir, &cargo_toml, "diagnostics", None).expect("create");
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "diagnostics",
        "health",
        Some("std_msgs::Int32"),
        false,
        false,
    )
    .expect("diagnostics health");
    node_cmd::node_modify_set_policy(&ws.nodes_dir, "diagnostics", &MacroPolicy::External)
        .expect("diagnostics external");

    // Copy the REAL fixture YAML in verbatim.
    let fixture = std::fs::read_to_string(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../cerulion_core/fixtures/test_graph.yaml"),
    )
    .expect("read fixture");
    std::fs::write(ws.graphs_dir.join("perception.yaml"), &fixture).expect("write fixture");

    let report = graph_cmd::graph_levels(&ws.root, "perception")
        .expect("levels")
        .rendered;

    // The fixture has NO `prefix:` — the verb resolves the same hostname
    // default `graph_run` uses. Substitute it into the hand oracle (the
    // oracle pins the LEVELIZATION + rendering, not hostname resolution).
    let prefix = cerulion_core::graph::default_prefix("perception");
    let expected = format!(
        "\
graph: perception  prefix: {prefix}
levels source: derived (trigger-aware Kahn levelization)
nodes: 6  levels: 2
level 0: camera [period(100ms)], imu [period(100ms)], tracker [period(100ms)], diagnostics [external]
  -> /{prefix}/camera/image  camera -> detector (level 1)
  -> /{prefix}/camera/image  camera -> fusion (level 1)
  -> /{prefix}/imu/data  imu -> fusion (level 1)
level 1: detector [data(image)], fusion [sync(50ms)]
chains: 0 fusable, 0 hop(s), 4 of 4 consumer edge(s) queued
  colocation: one process (this graph declares no process_groups:; \
`cerulion graph run` may derive a partition, and each run writes its own census into its \
run directory)
  queued by reason: fan-out 2, consumer-policy 1, latest-value-read 1
  queued  /{prefix}/camera/image  camera -> detector.image
    fan-out: the topic has 2 consumers, and a chain ends at a fan-out
  queued  /{prefix}/camera/image  camera -> fusion.image
    fan-out: the topic has 2 consumers, and a chain ends at a fan-out
  queued  /{prefix}/imu/data  imu -> fusion.imu
    consumer-policy: the consumer fires on sync, not on this frame
  queued  /external/health  (no in-graph producer) -> diagnostics.health
    latest-value-read: the consumer reads this topic as a latest value and is not woken by it
"
    );
    assert_eq!(
        report, expected,
        "fixture report must match the hand oracle (4 policies, wide root level, \
         absolute external source as a root, `topic:` override edge-less)"
    );
}

// ==========================================================================
// The `data(any — inferred)` and `unbounded_sync`
// policy-render branches, each with its own oracle.
// ==========================================================================

#[test]
fn inferred_data_any_label_oracle() {
    // A node with a NON-trigger input and NO declared policy parses to policy
    // None → the runtime INFERS a Data-on-ANY-input fallback (firing on any
    // input) at build, with its own loud warn. The `graph levels` label must
    // mark this INFERRED fallback `data(any — inferred)`, visibly distinct from
    // a DECLARED `data(<input>)` policy — silent inference must never
    // masquerade as a user choice on user-facing output.
    let tmp = tempfile::tempdir().unwrap();
    let ws = workspace::workspace_create(tmp.path(), "inferred_ws").expect("workspace");
    let cargo_toml = ws.root.join("Cargo.toml");

    // `src`: a Period source.
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

    // `passive`: one NON-trigger input, NO policy → parses to None (the strict
    // engine ctor writes `#[cerulion_node]` with no policy attr + a plain
    // `#[input]` field). The None fallback fires on any input, so `inp` is a
    // triggering edge and `passive` re-levelizes below `src`.
    node_cmd::node_create_with_options(
        &ws.nodes_dir,
        &cargo_toml,
        "passive",
        None,
        &node_cmd::NodeCreateOptions {
            inputs: vec![("std_msgs::Int32".to_string(), "inp".to_string())],
            ..Default::default()
        },
    )
    .expect("create passive");

    write_graph(
        &ws,
        "inferred",
        r#"
name: inferred
prefix: p
nodes:
  - id: s
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: c
    type: passive
    inputs:
      - name: inp
        source: s/out
"#,
    );

    let report = graph_cmd::graph_levels(&ws.root, "inferred")
        .expect("levels")
        .rendered;
    let expected = "\
graph: inferred  prefix: p
levels source: derived (trigger-aware Kahn levelization)
nodes: 2  levels: 2
level 0: s [period(10ms)]
  -> /p/s/out  s -> c (level 1)
level 1: c [data(any — inferred)]
chains: 1 fusable, 1 hop(s), 0 of 1 consumer edge(s) queued
  colocation: one process (this graph declares no process_groups:; \
`cerulion graph run` may derive a partition, and each run writes its own census into its \
run directory)
  chain 0  levels 0-1  2 nodes, 1 hop(s)
    s -> c  on /p/s/out
";
    assert_eq!(
        report, expected,
        "the inferred data-on-any fallback must render `data(any — inferred)`, \
         distinct from a declared `data(<input>)`"
    );
}

#[test]
fn unbounded_sync_label_oracle() {
    // A node with the `unbounded_sync` node-level policy must render the
    // `unbounded_sync` label. Two Period sources fan into an unbounded-sync
    // merger, which fires when BOTH inputs have data (all inputs triggering →
    // it re-levelizes below both sources).
    let tmp = tempfile::tempdir().unwrap();
    let ws = workspace::workspace_create(tmp.path(), "usync_ws").expect("workspace");
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

    // `merger`: two TRIGGER inputs + the `unbounded_sync` node-level policy
    // (sync aligns only `#[input(trigger)]`-marked ports — also the
    // macro's ≥2-trigger compile requirement for `unbounded_sync`).
    node_cmd::node_create(&ws.nodes_dir, &cargo_toml, "merger", None).expect("create merger");
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "merger",
        "a",
        Some("std_msgs::Int32"),
        false,
        true,
    )
    .expect("merger a");
    node_cmd::node_modify_add_port(
        &ws.nodes_dir,
        "merger",
        "b",
        Some("std_msgs::Int32"),
        false,
        true,
    )
    .expect("merger b");
    node_cmd::node_modify_set_policy(&ws.nodes_dir, "merger", &MacroPolicy::UnboundedSync)
        .expect("merger unbounded_sync");

    write_graph(
        &ws,
        "usync",
        r#"
name: usync
prefix: p
nodes:
  - id: sa
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: sb
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: m
    type: merger
    inputs:
      - name: a
        source: sa/out
      - name: b
        source: sb/out
"#,
    );

    let report = graph_cmd::graph_levels(&ws.root, "usync")
        .expect("levels")
        .rendered;
    let expected = "\
graph: usync  prefix: p
levels source: derived (trigger-aware Kahn levelization)
nodes: 3  levels: 2
level 0: sa [period(10ms)], sb [period(10ms)]
  -> /p/sa/out  sa -> m (level 1)
  -> /p/sb/out  sb -> m (level 1)
level 1: m [unbounded_sync]
chains: 0 fusable, 0 hop(s), 2 of 2 consumer edge(s) queued
  colocation: one process (this graph declares no process_groups:; \
`cerulion graph run` may derive a partition, and each run writes its own census into its \
run directory)
  queued by reason: consumer-policy 2
  queued  /p/sa/out  sa -> m.a
    consumer-policy: the consumer fires on unbounded sync, not on this frame
  queued  /p/sb/out  sb -> m.b
    consumer-policy: the consumer fires on unbounded sync, not on this frame
";
    assert_eq!(
        report, expected,
        "the unbounded_sync policy must render the `unbounded_sync` label"
    );
}

// ==========================================================================
// Loud error arms (no partial output — the fn returns Err, nothing renders).
// ==========================================================================

#[test]
fn validation_failure_is_loud_no_partial_output() {
    // n1 reads from an undeclared node `ghost` — validate_graph rejects.
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(
        &ws,
        "badref",
        r#"
name: badref
prefix: p
nodes:
  - id: n1
    type: relay
    inputs:
      - name: inp
        source: ghost/out
    outputs:
      - name: out
        schema: std_msgs/Int32
"#,
    );

    let err = graph_cmd::graph_levels(&ws.root, "badref")
        .expect_err("an invalid graph must be a loud error, not a partial print")
        .to_string();
    assert!(
        err.contains("ghost"),
        "the existing validate_graph diagnostic must surface; got: {err}"
    );
}

#[test]
fn missing_graph_is_loud() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    let err = graph_cmd::graph_levels(&ws.root, "nope")
        .expect_err("a missing graph must error via the existing GraphNotFound path")
        .to_string();
    assert!(
        err.contains("nope"),
        "error must name the graph; got: {err}"
    );
}

#[test]
fn missing_node_crate_is_loud() {
    // The graph references a node type with no `nodes/<type>/` directory —
    // the policy cannot be derived, so the verb refuses (names the type + the
    // fix) instead of guessing a levelization.
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(
        &ws,
        "notype",
        r#"
name: notype
prefix: p
nodes:
  - id: n0
    type: phantom
    outputs:
      - name: out
        schema: std_msgs/Int32
"#,
    );

    let err = graph_cmd::graph_levels(&ws.root, "notype")
        .expect_err("a missing node crate must be a loud error")
        .to_string();
    assert!(
        err.contains("phantom") && err.contains("cerulion node create"),
        "error must name the type and the fix; got: {err}"
    );
}

#[test]
fn algebraic_cycle_is_loud() {
    // a triggers on b's output and b triggers on a's — a pure trigger-edge
    // ring. validate_graph passes (structural refs are fine); derive_levels
    // rejects with the cycle diagnostic, which the verb surfaces loudly.
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(
        &ws,
        "ring",
        r#"
name: ring
prefix: p
nodes:
  - id: a
    type: relay
    inputs:
      - name: inp
        source: b/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: b
    type: relay
    inputs:
      - name: inp
        source: a/out
    outputs:
      - name: out
        schema: std_msgs/Int32
"#,
    );

    let err = graph_cmd::graph_levels(&ws.root, "ring")
        .expect_err("an algebraic trigger cycle must be a loud error")
        .to_string();
    // `graph_levels` routes through `resolve_levels`, whose
    // one-voice diagnostic reads "cannot be scheduled — algebraic trigger
    // cycle: ..." (identical to the runtime build error).
    assert!(
        err.contains("algebraic trigger cycle") && err.contains("a") && err.contains("b"),
        "error must carry the cycle diagnostic; got: {err}"
    );
}

// ==========================================================================
// `graph levels` renders the levels the runtime RUNS —
// routed through `resolve_levels`, with the source named.
// ==========================================================================

/// A valid NON-Kahn `level_assignments:` block (the lonely source `x` is
/// hand-delayed to level 1 — legal: the yaml path has no sink pin) renders
/// (a) the override source line and (b) the ASSIGNED levels, with
/// within-level order = graph (`nodes:`) order, never map order.
#[test]
fn override_block_renders_source_line_and_assignment_levels() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(
        &ws,
        "ovr",
        r#"name: ovr
prefix: p
level_assignments:
  n1: 1
  x: 1
  n0: 0
nodes:
  - id: n0
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: x
    type: src
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: n1
    type: relay
    inputs:
      - name: inp
        source: n0/out
"#,
    );

    let report = graph_cmd::graph_levels(&ws.root, "ovr").expect("levels");
    assert!(report.partition_error.is_none());
    let expected = "\
graph: ovr  prefix: p
levels source: level_assignments: block (baked override in the graph yaml)
nodes: 3  levels: 2
level 0: n0 [period(10ms)]
  -> /p/n0/out  n0 -> n1 (level 1)
level 1: x [period(10ms)], n1 [data(inp)]
chains: 1 fusable, 1 hop(s), 0 of 1 consumer edge(s) queued
  colocation: one process (this graph declares no process_groups:; \
`cerulion graph run` may derive a partition, and each run writes its own census into its \
run directory)
  chain 0  levels 0-1  2 nodes, 1 hop(s)
    n0 -> n1  on /p/n0/out
";
    assert_eq!(
        report.rendered, expected,
        "the override levels render (x delayed to L1, graph order within the level)"
    );
}

/// An INVALID block is a loud `graph levels` error carrying the validation
/// diagnostic (the view refuses to render levels the runtime would refuse).
#[test]
fn invalid_override_block_is_a_loud_levels_error() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(
        &ws,
        "bad",
        r#"name: bad
prefix: p
level_assignments:
  n0: 0
  n1: 0
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
"#,
    );
    let err = graph_cmd::graph_levels(&ws.root, "bad")
        .expect_err("a co-located trigger edge must be rejected")
        .to_string();
    assert!(
        err.contains("not strictly level-increasing"),
        "the validation diagnostic surfaces; got: {err}"
    );
}

// ==========================================================================
// The chains block reads DECLARATIONS from node source.
// ==========================================================================

/// A node-level rate cap written in node SOURCE refuses the hop INTO the
/// capped node, and leaves the hop out of it alone.
///
/// Every declaration the census reads comes from source, so a rule whose value
/// this path dropped would be decidable in principle and never decided, and
/// the report would silently over-count fusable hops. The cap is written into
/// the scaffolded node exactly as an author would write it.
#[test]
fn a_rate_cap_in_node_source_refuses_the_hop_into_the_capped_node() {
    let tmp = tempfile::tempdir().unwrap();
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
    write_graph(
        &ws,
        "capped",
        r#"
name: capped
prefix: p
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
"#,
    );

    let report = graph_cmd::graph_levels(&ws.root, "capped")
        .expect("levels")
        .rendered;
    let expected = "\
graph: capped  prefix: p
levels source: derived (trigger-aware Kahn levelization)
nodes: 3  levels: 3
level 0: n0 [period(10ms)]
  -> /p/n0/out  n0 -> n1 (level 1)
level 1: n1 [data(inp)]
  -> /p/n1/out  n1 -> n2 (level 2)
level 2: n2 [data(inp)]
chains: 1 fusable, 1 hop(s), 1 of 2 consumer edge(s) queued
  colocation: one process (this graph declares no process_groups:; \
`cerulion graph run` may derive a partition, and each run writes its own census into its \
run directory)
  chain 0  levels 1-2  2 nodes, 1 hop(s)
    n1 -> n2  on /p/n1/out
  queued by reason: throttle 1
  queued  /p/n0/out  n0 -> n1.inp
    throttle: the consumer declares `throttle_ms = 5`, and a direct call has nowhere to \
defer to
";
    assert_eq!(
        report, expected,
        "the cap must refuse the hop INTO the capped node, and a capped node must still \
         head a chain of its own"
    );
}
