// SPDX-License-Identifier: AGPL-3.0-only
//! PURE tests for the `graph run` LITERAL multi-process
//! default — the partition-intent decision matrix
//! ([`resolve_partition_intent`]), the auto-partition pre-flight consent
//! ladder ([`run_auto_partition_preflight`]: --yes / no-TTY floor / TTY
//! confirm / decline semantics for both the unpartitioned default and the
//! `--auto-partition` re-derive), the notice-text pins, and the load-bearing
//! IN-MEMORY == WRITTEN deployment-equality pin (the derived-in-memory config
//! must produce EXACTLY the `plan_deployment` a written file would).
//!
//! No iceoryx2, no transport, no `GraphRuntime::build` — parallel-safe.
//! Workspace scaffold cribbed from `graph_partition_test.rs` (the real
//! `node_cmd` helpers, so policy metadata comes from actual source parsing).

use std::path::Path;

use cerulion_cli_engine::error::CliResult;
use cerulion_cli_engine::graph_cmd::TimeSource;
use cerulion_cli_engine::partition_emit::{
    resolve_partition_intent, run_auto_partition_preflight, PartitionIntent, PreflightOptions,
    RunPartitionOutcome, NON_UNIX_AUTO_PARTITION_IGNORED, SINGLE_PROCESS_AUTO_PARTITION_REJECTION,
};
use cerulion_cli_engine::{node_cmd, workspace};
use cerulion_core::graph::config::GraphConfig;
use cerulion_core::graph::parse_graph_raw;
use cerulion_core::MacroPolicy;
use indexmap::IndexMap;

// ==========================================================================
// The pure intent matrix (mode-selection oracles).
// ==========================================================================

#[test]
fn intent_unpartitioned_unix_real_derives_the_literal_default() {
    assert_eq!(
        resolve_partition_intent(false, false, false, true, TimeSource::Real),
        Ok(PartitionIntent::Derive { re_derive: false }),
        "THE LITERAL DEFAULT: unpartitioned + Unix + Real clock derives multi-process"
    );
}

#[test]
fn intent_hand_written_groups_are_respected_without_the_flag() {
    for ts in [TimeSource::Real, TimeSource::Virtual, TimeSource::External] {
        assert_eq!(
            resolve_partition_intent(true, false, false, true, ts),
            Ok(PartitionIntent::RespectFile {
                auto_partition_ignored: None
            }),
            "a hand-written process_groups block is respected exactly as today (ts={ts:?})"
        );
    }
}

#[test]
fn intent_auto_partition_rederives_over_an_existing_block() {
    assert_eq!(
        resolve_partition_intent(true, false, true, true, TimeSource::Real),
        Ok(PartitionIntent::Derive { re_derive: true }),
        "--auto-partition re-derives; re_derive=true flips the decline semantics"
    );
}

#[test]
fn intent_auto_partition_on_unpartitioned_graph_derives() {
    // Explicit --auto-partition derives under ANY clock (parity with a
    // hand-written block: the supervisor warns on Virtual, and External is
    // rejected downstream by resolve_deployment itself).
    for ts in [TimeSource::Real, TimeSource::Virtual, TimeSource::External] {
        assert_eq!(
            resolve_partition_intent(false, false, true, true, ts),
            Ok(PartitionIntent::Derive { re_derive: false }),
            "explicit --auto-partition derives (ts={ts:?})"
        );
    }
}

#[test]
fn intent_single_process_skips_derivation_entirely() {
    for has_groups in [false, true] {
        for ts in [TimeSource::Real, TimeSource::Virtual, TimeSource::External] {
            assert_eq!(
                resolve_partition_intent(has_groups, true, false, true, ts),
                Ok(PartitionIntent::RespectFile {
                    auto_partition_ignored: None
                }),
                "--single-process = monolith opt-out AND no derivation \
                 (has_groups={has_groups}, ts={ts:?})"
            );
        }
    }
}

#[test]
fn intent_single_process_plus_auto_partition_is_rejected() {
    let err = resolve_partition_intent(false, true, true, true, TimeSource::Real)
        .expect_err("contradictory intent must be refused");
    assert_eq!(err, SINGLE_PROCESS_AUTO_PARTITION_REJECTION);
    assert!(
        err.contains("--single-process") && err.contains("--auto-partition"),
        "the rejection names both flags; got: {err}"
    );
}

#[test]
fn intent_virtual_and_external_keep_todays_monolith_on_unpartitioned_graphs() {
    for ts in [TimeSource::Virtual, TimeSource::External] {
        assert_eq!(
            resolve_partition_intent(false, false, false, true, ts),
            Ok(PartitionIntent::RespectFile {
                auto_partition_ignored: None
            }),
            "the mp default applies to the REAL-clock live path ONLY (ts={ts:?})"
        );
    }
}

#[test]
fn intent_non_unix_never_derives_and_flags_an_ignored_auto_partition() {
    // Without the flag: silent respect (deriving groups that cannot run mp
    // would be noise).
    assert_eq!(
        resolve_partition_intent(false, false, false, false, TimeSource::Real),
        Ok(PartitionIntent::RespectFile {
            auto_partition_ignored: None
        })
    );
    // With an explicit --auto-partition: still respect, but carry the loud
    // ignore reason for the dispatch site to warn with.
    assert_eq!(
        resolve_partition_intent(false, false, true, false, TimeSource::Real),
        Ok(PartitionIntent::RespectFile {
            auto_partition_ignored: Some(NON_UNIX_AUTO_PARTITION_IGNORED)
        })
    );
    assert!(
        NON_UNIX_AUTO_PARTITION_IGNORED.contains("--auto-partition")
            && NON_UNIX_AUTO_PARTITION_IGNORED.contains("monolith"),
        "the ignore notice names the flag and the fallback"
    );
}

// ==========================================================================
// Workspace scaffold (cribbed from graph_partition_test.rs).
// ==========================================================================

fn setup_workspace(root: &Path) -> workspace::CerulionWorkspace {
    let ws = workspace::workspace_create(root, "run_ws").expect("workspace");
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

    // A 2-input Sync node for the SKEWED shape (cribbed from
    // graph_partition_test.rs) — the consumer of a gap-spanning direct edge.
    // Both inputs are TRIGGER-marked: a Sync node's DAG edges are
    // its `#[input(trigger)]` set only (unmarked = latest-value context), and
    // the sync_window contract itself requires >= 2 trigger inputs.
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

const CHAIN_YAML: &str = r#"# run-preflight fixture — comment must survive
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
"#;

/// The chain with a HAND-WRITTEN partition (for the re-derive arms).
const CHAIN_WITH_GROUPS_YAML: &str = r#"name: chain
prefix: p
process_groups:
  mine_a: [n0, n1]
  mine_b: [n2]
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

/// The graph's exact BYTES. `run_auto_partition_preflight` takes the caller's
/// read rather than making its own, so the tests hand it the same bytes
/// `read_config` parsed — which is the property under test.
fn read_raw(ws: &workspace::CerulionWorkspace, name: &str) -> String {
    std::fs::read_to_string(ws.graphs_dir.join(format!("{name}.yaml"))).expect("read")
}

fn read_config(ws: &workspace::CerulionWorkspace, name: &str) -> GraphConfig {
    parse_graph_raw(
        &std::fs::read_to_string(ws.graphs_dir.join(format!("{name}.yaml"))).expect("read"),
    )
    .expect("parse")
}

fn panic_confirm(_preview: &str) -> CliResult<bool> {
    panic!("the confirm provider must not be invoked on this path")
}

fn shape(map: &IndexMap<String, Vec<String>>) -> Vec<(String, Vec<String>)> {
    map.iter().map(|(k, v)| (k.clone(), v.clone())).collect()
}

fn expect(pairs: &[(&str, &[&str])]) -> Vec<(String, Vec<String>)> {
    pairs
        .iter()
        .map(|(n, m)| (n.to_string(), m.iter().map(|s| s.to_string()).collect()))
        .collect()
}

/// The baseline derivation oracle for the chain (pipeline order).
fn baseline_shape() -> Vec<(String, Vec<String>)> {
    expect(&[
        ("grp_n0", &["n0"]),
        ("grp_n1", &["n1"]),
        ("grp_n2", &["n2"]),
    ])
}

// ==========================================================================
// The consent ladder on the run path.
// ==========================================================================

#[tracing_test::traced_test]
#[test]
fn yes_persists_and_runs_the_derived_groups() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    let graph_path = ws.graphs_dir.join("chain.yaml");

    let mut confirm = panic_confirm; // --yes never consults the confirm
    let pre = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: true,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect("--yes preflight");

    let backup = match &pre.outcome {
        RunPartitionOutcome::Persisted { backup } => backup.clone().expect("file pre-exists"),
        other => panic!("expected Persisted, got {other:?}"),
    };
    assert_eq!(shape(&pre.config.process_groups), baseline_shape());
    assert_eq!(
        std::fs::read_to_string(&backup).expect("bak readable"),
        CHAIN_YAML,
        "the backup carries the original bytes"
    );
    let written = std::fs::read_to_string(&graph_path).expect("readable");
    assert!(
        written.contains("# run-preflight fixture — comment must survive"),
        "the surgical write preserves comments"
    );
    let parsed = parse_graph_raw(&written).expect("re-parses");
    assert_eq!(
        shape(&parsed.process_groups),
        shape(&pre.config.process_groups),
        "the written file and the running config carry the SAME groups"
    );
    assert!(logs_contain("partition written to the graph file"));
}

#[tracing_test::traced_test]
#[test]
fn no_tty_floor_runs_in_memory_never_mutates_and_names_both_escape_hatches() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    let graph_path = ws.graphs_dir.join("chain.yaml");

    let mut confirm = panic_confirm; // no TTY => no confirm
    let pre = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: false,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect("floor preflight");

    assert_eq!(pre.outcome, RunPartitionOutcome::InMemory);
    assert_eq!(
        shape(&pre.config.process_groups),
        baseline_shape(),
        "the run proceeds with the derived groups held in-memory"
    );
    assert_eq!(
        std::fs::read_to_string(&graph_path).expect("readable"),
        CHAIN_YAML,
        "THE CONSENT FLOOR: the file is NEVER mutated without a TTY confirm or --yes"
    );
    assert!(
        !ws.graphs_dir.join("chain.yaml.bak").exists(),
        "no backup churn on the floor path"
    );
    // The loud one-line notice names both escape hatches.
    assert!(logs_contain("IN-MEMORY"), "the notice says in-memory");
    assert!(logs_contain("--yes"), "the notice names --yes (persist)");
    assert!(
        logs_contain("--single-process"),
        "the notice names --single-process (monolith)"
    );
    // LOUD means WARN, and a text match cannot tell: nobody was asked here,
    // so the run adopted a layout without consent and must say so above the
    // level a quiet filter keeps. The decline arm below is the opposite case.
    logs_assert(|lines: &[&str]| in_memory_notice_level(lines, "no TTY to confirm", "WARN"));
}

#[test]
fn tty_confirm_yes_writes_and_hint_line_names_the_decline_semantics() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);

    let mut seen: Option<String> = None;
    let mut confirm = |preview: &str| -> CliResult<bool> {
        seen = Some(preview.to_string());
        Ok(true)
    };
    let pre = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: false,
            is_tty: true,
        },
        &mut confirm,
    )
    .expect("confirmed preflight");
    assert!(matches!(pre.outcome, RunPartitionOutcome::Persisted { .. }));

    let shown = seen.expect("the confirm displayed the preview");
    assert!(
        shown.contains("IN-MEMORY — file untouched"),
        "the interactive preview tells the user what N does BEFORE they answer; got:\n{shown}"
    );
    assert!(
        shown.contains("process-per-node baseline"),
        "the preview carries the mode line; got:\n{shown}"
    );
}

#[tracing_test::traced_test]
#[test]
fn tty_decline_on_unpartitioned_graph_runs_derived_in_memory() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    let graph_path = ws.graphs_dir.join("chain.yaml");

    let mut confirm = |_: &str| -> CliResult<bool> { Ok(false) };
    let pre = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: false,
            is_tty: true,
        },
        &mut confirm,
    )
    .expect("declined preflight");

    // Decline ≠ abort: the run still goes multi-process with the derived
    // groups in-memory (Ctrl-C is abort; --single-process is the monolith).
    assert_eq!(pre.outcome, RunPartitionOutcome::InMemory);
    assert_eq!(shape(&pre.config.process_groups), baseline_shape());
    assert_eq!(
        std::fs::read_to_string(&graph_path).expect("readable"),
        CHAIN_YAML,
        "decline leaves the file byte-untouched"
    );
    // The post-decline notice says exactly what is happening.
    assert!(logs_contain("auto-partition declined"));
    assert!(logs_contain("IN-MEMORY"));
    assert!(logs_contain("--single-process"));
    // INFO, not WARN: the person saw the preview and answered no, which is
    // the documented default. A warn here tells every first-time user that
    // doing what the quickstart says is a problem.
    logs_assert(|lines: &[&str]| in_memory_notice_level(lines, "auto-partition declined", "INFO"));
}

/// Exactly one captured line carries `marker`, and its level is `level`.
///
/// Both in-memory notices share most of their words, so each caller passes
/// the phrase only ITS arm emits. The count is what makes the level check
/// non-vacuous: a marker that matched nothing would otherwise pass. Takes the
/// lines because `logs_assert` exists only inside a `traced_test` body.
fn in_memory_notice_level(lines: &[&str], marker: &str, level: &str) -> Result<(), String> {
    let hits: Vec<&&str> = lines.iter().filter(|l| l.contains(marker)).collect();
    match hits.as_slice() {
        [line] if line_level(line) == Some(level) => Ok(()),
        [line] => Err(format!("expected the notice at {level}, got: {line}")),
        other => Err(format!(
            "expected exactly one line carrying {marker:?}, got {}",
            other.len()
        )),
    }
}

/// Read a captured line's level as a whole whitespace token out of its header.
///
/// Not a substring match: `tracing-test` renders the span name, which is the
/// test function's own name, into every captured line, so `contains("WARN")`
/// can be satisfied by a rename. An unparseable line yields `None` and matches
/// no level, which fails the check above rather than passing it.
fn line_level(line: &str) -> Option<&'static str> {
    const LEVELS: [&str; 5] = ["ERROR", "WARN", "INFO", "DEBUG", "TRACE"];
    let header = line.split(": ").next().unwrap_or(line);
    header
        .split_whitespace()
        .find_map(|token| LEVELS.into_iter().find(|level| *level == token))
}

#[tracing_test::traced_test]
#[test]
fn rederive_tty_decline_keeps_the_existing_hand_written_groups() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_WITH_GROUPS_YAML);
    let graph_path = ws.graphs_dir.join("chain.yaml");

    let mut seen: Option<String> = None;
    let mut confirm = |preview: &str| -> CliResult<bool> {
        seen = Some(preview.to_string());
        Ok(false)
    };
    let pre = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: true,
            lenient_costs: false,
            assume_yes: false,
            is_tty: true,
        },
        &mut confirm,
    )
    .expect("re-derive declined");

    // Decline on a re-derive = "don't change what I wrote": the run keeps the
    // HAND-WRITTEN groups, not the derivation.
    assert_eq!(pre.outcome, RunPartitionOutcome::KeptExisting);
    assert_eq!(
        shape(&pre.config.process_groups),
        expect(&[("mine_a", &["n0", "n1"]), ("mine_b", &["n2"])]),
        "the ORIGINAL hand-written groups survive a re-derive decline"
    );
    assert_eq!(
        std::fs::read_to_string(&graph_path).expect("readable"),
        CHAIN_WITH_GROUPS_YAML
    );
    assert!(logs_contain("keeping the existing process_groups"));
    // The hint told the user N keeps their block.
    let shown = seen.expect("preview shown");
    assert!(
        shown.contains("keep the existing"),
        "the re-derive hint names the keep-existing semantics; got:\n{shown}"
    );
    // The diff shows their block being replaced (had they said y).
    assert!(
        shown.contains("-process_groups:") && shown.contains("-  mine_a: [n0, n1]"),
        "the diff shows the existing block; got:\n{shown}"
    );
}

#[tracing_test::traced_test]
#[test]
fn rederive_already_current_file_is_a_quiet_no_op() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);

    // First: persist the derivation (--yes).
    let mut confirm = panic_confirm;
    let first = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: true,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect("persist");
    assert!(matches!(
        first.outcome,
        RunPartitionOutcome::Persisted { .. }
    ));
    std::fs::remove_file(ws.graphs_dir.join("chain.yaml.bak")).expect("clear bak");

    // Re-derive over the now-current file: nothing to write, nothing to ask.
    let second = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: true,
            lenient_costs: false,
            assume_yes: false,
            is_tty: true,
        },
        &mut confirm,
    )
    .expect("already current");
    assert_eq!(second.outcome, RunPartitionOutcome::AlreadyCurrent);
    assert_eq!(shape(&second.config.process_groups), baseline_shape());
    assert!(
        !ws.graphs_dir.join("chain.yaml.bak").exists(),
        "no write, no backup churn"
    );
    assert!(logs_contain(
        "already carries exactly the derived partition"
    ));
}

#[test]
fn costs_artifact_at_default_path_makes_the_run_default_fused() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    std::fs::write(
        ws.graphs_dir.join("chain.costs.yaml"),
        "version: 1\ngraph: chain\nwindow_ns: 5000000000\nnodes:\n  n0: 1000\n  n1: 900\n  n2: \
         800\nedges:\n- producer: n0\n  consumer: n1\n  rate_mhz: 20000\n- producer: n1\n  \
         consumer: n2\n  rate_mhz: 20000\nisolated: []\nhop:\n  intra_ns: 1000\n  cross_ns: \
         6800\n",
    )
    .expect("write artifact");

    let mut confirm = panic_confirm;
    let pre = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: false,
            is_tty: false, // floor: in-memory
        },
        &mut confirm,
    )
    .expect("fused floor preflight");
    assert_eq!(
        shape(&pre.config.process_groups),
        expect(&[("grp_n0", &["n0", "n1", "n2"])]),
        "a default-path costs artifact makes the run default the FUSED optimum"
    );
    assert!(pre.preview.contains("cost-aware fusion"));
}

#[test]
fn run_default_consumes_the_artifacts_frozen_budget_not_unbounded() {
    // The run default routes through the SAME budget resolution
    // point as the verb — a v2 artifact whose FROZEN budget (1500 ns) is too
    // small for any pair (1000+900, 900+800) must yield PROCESS-PER-NODE.
    // Mutation-worthy: reverting the preflight's `budget_ns: None` back to
    // the earlier hardcoded `u64::MAX` fuses everything into ONE group
    // and fails this oracle.
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    std::fs::write(
        ws.graphs_dir.join("chain.costs.yaml"),
        "version: 2\ngraph: chain\nwindow_ns: 5000000000\nnodes:\n  n0: 1000\n  n1: 900\n  n2: \
         800\nedges:\n- producer: n0\n  consumer: n1\n  rate_mhz: 20000\n- producer: n1\n  \
         consumer: n2\n  rate_mhz: 20000\nisolated: []\nhop:\n  intra_ns: 1000\n  cross_ns: \
         6800\nderived_budget_ns: 1500\nprofile_cores: 2\n",
    )
    .expect("write v2 artifact");

    let mut confirm = panic_confirm;
    let pre = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: false,
            is_tty: false, // floor: in-memory
        },
        &mut confirm,
    )
    .expect("frozen-budget preflight");
    assert_eq!(
        shape(&pre.config.process_groups),
        expect(&[
            ("grp_n0", &["n0"]),
            ("grp_n1", &["n1"]),
            ("grp_n2", &["n2"]),
        ]),
        "the run default gates fusion by the artifact's FROZEN 1500 ns budget, \
         not the earlier unbounded hardcode"
    );
}

#[tracing_test::traced_test]
#[test]
fn frozen_zero_budget_degrades_the_lenient_run_default_to_baseline() {
    // The zero-budget guard + the lenient-default symmetry: a hand-edited
    // `derived_budget_ns: 0` on the ZERO-FLAG run default (lenient) must
    // DEGRADE to the process-per-node baseline with the loud
    // unusable-artifact warn class — NOT a hard Err (the verb keeps that; a
    // hand-edit typo must not abort a plain run), and NOT the earlier silent
    // consumption (zero-budget fusion happens to also be per-node, so the
    // BaselineDegraded mode + warn are the distinguishing pins).
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    std::fs::write(
        ws.graphs_dir.join("chain.costs.yaml"),
        "version: 2\ngraph: chain\nwindow_ns: 5000000000\nnodes:\n  n0: 1000\n  n1: 900\n  n2: \
         800\nedges:\n- producer: n0\n  consumer: n1\n  rate_mhz: 20000\n- producer: n1\n  \
         consumer: n2\n  rate_mhz: 20000\nisolated: []\nhop:\n  intra_ns: 1000\n  cross_ns: \
         6800\nderived_budget_ns: 0\nprofile_cores: 2\n",
    )
    .expect("write zero-frozen v2 artifact");

    let mut confirm = panic_confirm;
    let pre = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true, // the zero-flag `graph run` default path
            assume_yes: false,
            is_tty: false, // floor: in-memory
        },
        &mut confirm,
    )
    .expect("a hand-edited frozen 0 must not abort the zero-flag run");
    assert_eq!(
        shape(&pre.config.process_groups),
        expect(&[
            ("grp_n0", &["n0"]),
            ("grp_n1", &["n1"]),
            ("grp_n2", &["n2"]),
        ]),
        "the degraded run uses the process-per-node BASELINE (no costs consumed)"
    );
    assert!(
        logs_contain("PRESENT but UNUSABLE"),
        "the shared unusable-artifact warn class fires"
    );
    assert!(
        logs_contain("derived_budget_ns is 0"),
        "the warn names the zero frozen value (the hand-edit suspicion + both remedies \
         ride the same message)"
    );
    assert!(
        pre.preview.contains("PRESENT but UNUSABLE"),
        "the preview mode line stays accurate (BaselineDegraded); got:\n{}",
        pre.preview
    );
}

// ==========================================================================
// THE load-bearing correctness pin: in-memory == written deployment.
// ==========================================================================

#[test]
fn in_memory_and_written_configs_produce_identical_deployment_plans() {
    use cerulion_core::graph::{build_trigger_edges, GraphTopology, NodeInfo};

    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);

    // Path A: the no-TTY floor — derived groups IN-MEMORY, file untouched.
    let mut confirm = panic_confirm;
    let in_memory = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: false,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect("in-memory preflight");
    assert_eq!(in_memory.outcome, RunPartitionOutcome::InMemory);

    // Path B: --yes — the SAME derivation written to disk, then re-read the
    // way a later `graph run` would.
    let persisted = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: true,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect("persisted preflight");
    assert!(matches!(
        persisted.outcome,
        RunPartitionOutcome::Persisted { .. }
    ));
    let written_config = read_config(&ws, "chain");
    assert!(written_config.has_process_groups());

    // Both configs feed plan_deployment with IDENTICAL non-config inputs
    // (levels are derived per-config through the same pure pipeline; the
    // entry-info policies model the scaffolded types and are shared, so any
    // modeling imprecision cancels).
    let entry_infos = |config: &GraphConfig| -> IndexMap<String, NodeInfo> {
        config
            .nodes
            .iter()
            .map(|n| {
                let info = NodeInfo::with_meta(Vec::new(), Vec::new());
                let info = match n.node_type.as_str() {
                    "src" => info.with_policy(MacroPolicy::Period { period_ms: 10 }),
                    _ => info.with_policy(MacroPolicy::DataTrigger {
                        input_name: "inp".to_string(),
                    }),
                };
                (n.id.clone(), info)
            })
            .collect()
    };
    let plan_for = |config: &GraphConfig| -> serde_json::Value {
        let mut config = config.clone();
        if config.prefix.is_empty() {
            config.prefix = cerulion_core::graph::default_prefix(config.identity());
        }
        let infos = entry_infos(&config);
        let edges = build_trigger_edges(&config, &infos);
        let topo = GraphTopology::build(&config, &infos).expect("topology");
        let levels = topo.derive_levels(&edges).expect("levels");
        let plan = cerulion_cli_engine::multiprocess::plan_deployment(
            &config,
            &levels,
            Some(10_000_000),
            "equality_pin_nonce",
        )
        .expect("plan");
        serde_json::to_value(&plan).expect("serialize plan")
    };

    assert_eq!(
        plan_for(&in_memory.config),
        plan_for(&written_config),
        "THE PIN: an in-memory-derived partition must plan EXACTLY the deployment a written \
         file would (same workers, ranks, subgraphs, participant maps, barrier names)"
    );
}

#[test]
fn yes_inert_notice_names_the_flag_and_the_remedy() {
    // The loud-inference rule: `--yes` on a run that derives nothing
    // is inert and must SAY so. `graph_run` emits this const when assume_yes
    // is set and the intent resolved to RespectFile; the text is pinned here
    // (pure) since the emission site needs a full run.
    use cerulion_cli_engine::partition_emit::YES_INERT_NOTICE;
    assert!(
        YES_INERT_NOTICE.contains("--yes is inert"),
        "names the inert flag"
    );
    assert!(
        YES_INERT_NOTICE.contains("--auto-partition"),
        "names the remedy (re-derive)"
    );
    assert!(
        YES_INERT_NOTICE.contains("no partition is being derived"),
        "explains WHY it is inert"
    );
}

// ==========================================================================
// The ZERO-FLAG default path must not abort a
// plain run over a broken sidecar the user never asked to consume; the
// explicit contracts (the verb / --auto-partition) keep the hard Err.
// ==========================================================================

#[tracing_test::traced_test]
#[test]
fn lenient_default_degrades_malformed_artifact_to_baseline_with_warn() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    std::fs::write(
        ws.graphs_dir.join("chain.costs.yaml"),
        "nodes: [this is not the schema\n",
    )
    .expect("write junk artifact");

    // The zero-flag literal default (lenient) on the no-TTY floor: the run
    // PROCEEDS on the baseline instead of aborting.
    let mut confirm = panic_confirm;
    let pre = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: false,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect("a broken DEFAULT artifact must not abort the zero-flag run");
    assert_eq!(pre.outcome, RunPartitionOutcome::InMemory);
    assert_eq!(
        shape(&pre.config.process_groups),
        baseline_shape(),
        "the degraded run uses the process-per-node BASELINE"
    );
    // The degradation is LOUD: names the file + the remedy.
    assert!(
        logs_contain("PRESENT but UNUSABLE"),
        "the degrade warn must fire"
    );
    assert!(
        logs_contain("chain.costs.yaml") && logs_contain("graph profile"),
        "the warn names the artifact and the regenerate remedy"
    );
    // And the preview states WHY it is baseline (not \"no artifact\").
    assert!(
        pre.preview.contains("PRESENT but UNUSABLE"),
        "the preview mode line names the degraded cause; got:\n{}",
        pre.preview
    );
}

// ==========================================================================
// An artifact harvested from ANOTHER graph is a present-but-unusable
// class like any other on this path — a loud degrade to the baseline, never a
// silent fuse from another graph's measured costs.
// ==========================================================================

/// The chain artifact, byte-for-byte, except that its `graph:` names something
/// else. The node ids OVERLAP deliberately: an artifact whose ids did not
/// match this graph would already be rejected by `to_costs`, so it could not
/// tell a provenance gate apart from the checks that were there before. This
/// one is otherwise perfectly valid, so a run that fuses from it produces
/// credible-but-wrong groups rather than an error.
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

#[tracing_test::traced_test]
#[test]
fn lenient_default_degrades_a_foreign_graphs_artifact_to_baseline_with_warn() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    std::fs::write(ws.graphs_dir.join("chain.costs.yaml"), FOREIGN_COSTS_YAML)
        .expect("write foreign artifact");

    let mut confirm = panic_confirm;
    let pre = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: false,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect("a FOREIGN default artifact must not abort the zero-flag run");
    assert_eq!(pre.outcome, RunPartitionOutcome::InMemory);
    assert_eq!(
        shape(&pre.config.process_groups),
        baseline_shape(),
        "the degraded run uses the process-per-node BASELINE — the foreign costs were \
         NOT consumed (fusing them would give the single group `[n0, n1, n2]`)"
    );
    // The degradation is LOUD, and names BOTH graphs: the found one says which
    // file this is, the expected one says which it should have been.
    assert!(
        logs_contain("PRESENT but UNUSABLE"),
        "the degrade warn must fire"
    );
    assert!(
        logs_contain("some_other_graph"),
        "the warn names the graph the artifact was harvested from"
    );
    assert!(
        logs_contain("chain.costs.yaml") && logs_contain("graph profile"),
        "the warn names the artifact and the re-profile remedy"
    );
    assert!(
        pre.preview.contains("PRESENT but UNUSABLE"),
        "the preview mode line names the degraded cause; got:\n{}",
        pre.preview
    );
}

#[test]
fn a_default_artifact_naming_this_graph_still_fuses_on_the_lenient_run_default() {
    // ANTI-TAUTOLOGY for the arm above: without it, "degrades to the baseline"
    // is satisfied by a gate that refuses EVERY artifact — which on this path
    // is silent (a degrade, not an error), so nothing else would notice. The
    // artifact differs from `FOREIGN_COSTS_YAML` in exactly one field, and the
    // group shape is the discriminator: fused is ONE group, degraded is three.
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    std::fs::write(
        ws.graphs_dir.join("chain.costs.yaml"),
        FOREIGN_COSTS_YAML.replace("graph: some_other_graph", "graph: chain"),
    )
    .expect("write matching artifact");

    let mut confirm = panic_confirm;
    let pre = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: false,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect("an artifact whose provenance AGREES must still fuse");
    assert_eq!(
        shape(&pre.config.process_groups),
        expect(&[("grp_n0", &["n0", "n1", "n2"])]),
        "both rated edges fuse under the unbounded budget — the costs were really consumed"
    );
}

#[test]
fn lenient_default_degrades_wrong_version_artifact_too() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    // Parses fine as YAML — fails the to_costs VERSION gate (the class a
    // parse-only probe would miss).
    std::fs::write(
        ws.graphs_dir.join("chain.costs.yaml"),
        "version: 99\ngraph: chain\nwindow_ns: 1\nnodes: {}\nedges: []\nisolated: []\n",
    )
    .expect("write future-version artifact");

    let mut confirm = panic_confirm;
    let pre = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: false,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect("a future-version DEFAULT artifact must not abort the zero-flag run");
    assert_eq!(shape(&pre.config.process_groups), baseline_shape());
}

#[test]
fn auto_partition_contract_keeps_the_hard_err_on_a_broken_artifact() {
    // --auto-partition asked for cost-aware behavior — a broken artifact is
    // a refusal, exactly like the verb (whose pin lives in
    // graph_partition_test::malformed_default_artifact_is_hard_err_never_baseline).
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    std::fs::write(
        ws.graphs_dir.join("chain.costs.yaml"),
        "nodes: [this is not the schema\n",
    )
    .expect("write junk artifact");

    let mut confirm = panic_confirm;
    let err = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: false, // graph_run passes !auto_partition
            assume_yes: true,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect_err("the explicit cost-aware contract must refuse a broken artifact")
    .to_string();
    assert!(
        err.contains("MALFORMED") && err.contains("chain.costs.yaml"),
        "the hard error names the file; got: {err}"
    );
    // And nothing was written on the failed path.
    assert_eq!(
        std::fs::read_to_string(ws.graphs_dir.join("chain.yaml")).expect("readable"),
        CHAIN_YAML,
        "a refused preflight must leave the file untouched"
    );
}

/// The lenient blanket extends past unreadable/wrong-
/// version to ANY cost-DERIVATION failure — here a stale-but-WELL-FORMED
/// default artifact missing a newly-added node's cost (auto_partition's
/// missing-p50 error). The zero-flag run degrades to the baseline, loudly.
const STALE_COSTS_MISSING_N2: &str = "\
version: 1
graph: chain
window_ns: 5000000000
nodes:
  n0: 1000
  n1: 900
edges:
- producer: n0
  consumer: n1
  rate_mhz: 20000
isolated: []
hop:
  intra_ns: 1000
  cross_ns: 6800
";

#[tracing_test::traced_test]
#[test]
fn lenient_default_degrades_stale_artifact_missing_a_node_cost() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML); // n0/n1/n2 — n2 is "newly added"
    std::fs::write(
        ws.graphs_dir.join("chain.costs.yaml"),
        STALE_COSTS_MISSING_N2,
    )
    .expect("write stale artifact");

    let mut confirm = panic_confirm;
    let pre = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: false,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect("a stale DEFAULT artifact must not abort the zero-flag run");
    assert_eq!(pre.outcome, RunPartitionOutcome::InMemory);
    assert_eq!(
        shape(&pre.config.process_groups),
        baseline_shape(),
        "the degraded run uses the process-per-node BASELINE (no costs consumed)"
    );
    // One warn surface, cause-parameterized: names the failing node (via
    // auto_partition's message riding the error field) + both remedies.
    assert!(
        logs_contain("PRESENT but UNUSABLE"),
        "the shared degrade warn fires"
    );
    assert!(
        logs_contain("n2") && logs_contain("p50"),
        "the warn's error field carries auto_partition's cause naming the node"
    );
    assert!(
        logs_contain("graph profile") && logs_contain("--auto-partition"),
        "the warn names the refresh remedy and the strict alternative"
    );
    assert!(
        pre.preview.contains("PRESENT but UNUSABLE"),
        "the preview mode line stays accurate (BaselineDegraded); got:\n{}",
        pre.preview
    );
}

#[test]
fn auto_partition_keeps_hard_err_on_stale_artifact_missing_a_node_cost() {
    // The strict twin: the SAME stale input under --auto-partition refuses —
    // the user asked for cost-aware behavior.
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);
    std::fs::write(
        ws.graphs_dir.join("chain.costs.yaml"),
        STALE_COSTS_MISSING_N2,
    )
    .expect("write stale artifact");

    let mut confirm = panic_confirm;
    let err = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: false, // graph_run passes !auto_partition
            assume_yes: true,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect_err("the explicit cost-aware contract must refuse a stale artifact")
    .to_string();
    assert!(
        err.contains("n2") && err.contains("p50"),
        "the hard error names the un-costed node; got: {err}"
    );
    assert_eq!(
        std::fs::read_to_string(ws.graphs_dir.join("chain.yaml")).expect("readable"),
        CHAIN_YAML,
        "a refused preflight leaves the file untouched"
    );
}

/// The negative pin for the lenient-retry discriminator:
/// a STRUCTURAL derive failure (a trigger cycle) with a perfectly VALID
/// artifact must surface the REAL error with ZERO artifact-blaming warns.
/// Firing the "cost artifact PRESENT but UNUSABLE" warn BEFORE the
/// baseline retry would blame a valid artifact for a broken graph.
#[tracing_test::traced_test]
#[test]
fn structural_failure_with_valid_artifact_propagates_without_artifact_warn() {
    // Two data-trigger relays feeding each other: a trigger-DAG cycle. The
    // artifact is VALID (both nodes costed, the edge rated) — the fused
    // derive reaches the levelizer and fails STRUCTURALLY; so does the
    // silent baseline retry (the cycle is cost-independent).
    const CYCLE_YAML: &str = r#"name: chain
prefix: p
nodes:
  - id: r1
    type: relay
    inputs:
      - name: inp
        source: r2/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: r2
    type: relay
    inputs:
      - name: inp
        source: r1/out
    outputs:
      - name: out
        schema: std_msgs/Int32
"#;
    const VALID_CYCLE_COSTS: &str = "\
version: 1
graph: chain
window_ns: 5000000000
nodes:
  r1: 1000
  r2: 900
edges:
- producer: r1
  consumer: r2
  rate_mhz: 20000
isolated: []
hop:
  intra_ns: 1000
  cross_ns: 6800
";
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CYCLE_YAML);
    std::fs::write(ws.graphs_dir.join("chain.costs.yaml"), VALID_CYCLE_COSTS)
        .expect("write valid artifact");

    let mut confirm = panic_confirm;
    let err = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true, // the zero-flag default — leniency in play
            assume_yes: false,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect_err("a structural (cycle) failure must propagate even on the lenient path")
    .to_string();
    // The cycle diagnostic is unified onto `resolve_levels`'
    // one-voice wording ("cannot be scheduled — algebraic trigger cycle: ...")
    // shared by the runtime, the partitioner, and `graph levels`.
    assert!(
        err.contains("algebraic trigger cycle"),
        "the REAL structural error surfaces; got: {err}"
    );
    // ZERO artifact-blaming warns: the artifact was never the problem.
    logs_assert(|lines: &[&str]| {
        let blamed = lines
            .iter()
            .filter(|l| l.contains("PRESENT but UNUSABLE"))
            .count();
        if blamed == 0 {
            Ok(())
        } else {
            Err(format!(
                "the artifact-unusable warn must NOT fire on a structural failure \
                 (found {blamed} occurrence(s))"
            ))
        }
    });
}

// ==========================================================================
// The `graph run` preflight never refines: an
// in-memory refined-levels override would band groups over levels the yaml
// does not carry (the runtime + a bag replay would run Kahn while the bands
// assumed refined levels). Refinement is opt-in via `graph partition`.
// The SKEWED shape is the discriminator: the VERB refines d@L0 → L3 and
// fuses {d,c} (see graph_partition_test.rs); the preflight must NOT.
// ==========================================================================

/// The skewed shape (crib of graph_partition_test.rs): d@L0's only consumer
/// c sits at L4 behind a deep sibling chain — costed, d has slack 0→3.
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

/// The preflight bands over KAHN (the file carries no block): on Kahn levels
/// the d→c fusion is gap-spanning ({0,4} — refused), so the derived groups
/// are the process-per-node singletons; the PERSISTED yaml carries
/// `process_groups:` but NO `level_assignments:`, and the adopted config's
/// `level_assignments` stays `None`. (The verb on this exact workspace
/// refines d → L3 and fuses {d,c} — the contrast proving `refine=false`.)
#[test]
fn preflight_never_refines_bands_over_kahn_and_writes_no_level_block() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "skewed", SKEWED_YAML);
    std::fs::write(
        cerulion_cli_engine::graph_cmd::default_artifact_path(&ws.root, "skewed"),
        SKEWED_COSTS_YAML,
    )
    .expect("write artifact");

    let mut confirm = |_p: &str| -> CliResult<bool> {
        panic!("assume_yes path must not consult the confirm provider")
    };
    let pre = run_auto_partition_preflight(
        &ws.root,
        "skewed",
        read_config(&ws, "skewed"),
        &read_raw(&ws, "skewed"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: true,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect("preflight persists");
    assert!(matches!(pre.outcome, RunPartitionOutcome::Persisted { .. }));

    // Kahn banding: the gap-spanning fusion is refused ⇒ singletons.
    let names: Vec<&str> = pre
        .config
        .process_groups
        .keys()
        .map(|k| k.as_str())
        .collect();
    assert_eq!(
        names,
        vec!["grp_d", "grp_s0", "grp_s1", "grp_s2", "grp_s3", "grp_c"],
        "preflight bands over KAHN levels — the d->c fusion stays refused"
    );
    assert!(
        pre.config.level_assignments.is_none(),
        "the adopted config never carries an in-memory refinement"
    );
    let written = std::fs::read_to_string(ws.graphs_dir.join("skewed.yaml")).expect("read");
    assert!(
        written.contains("process_groups:") && !written.contains("level_assignments"),
        "the persisted yaml has groups but NO level block; got:\n{written}"
    );
}

/// A PERSISTED `level_assignments:` block (written by a prior
/// `graph partition`) IS respected: the preflight re-derive bands over the
/// persisted refined levels — reproducing the file's own fused groups
/// byte-identically ⇒ `AlreadyCurrent`, file untouched, block preserved.
/// (If the preflight banded over Kahn instead, it would derive the singleton
/// groups, disagree with the file, and rewrite — this pin kills that.)
#[test]
fn preflight_respects_a_persisted_level_block_and_bands_over_it() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "skewed", SKEWED_YAML);
    std::fs::write(
        cerulion_cli_engine::graph_cmd::default_artifact_path(&ws.root, "skewed"),
        SKEWED_COSTS_YAML,
    )
    .expect("write artifact");

    // Step 1: the VERB persists the refined blocks (d -> L3, {d,c} fused).
    let mut confirm = |_p: &str| -> CliResult<bool> {
        panic!("assume_yes paths must not consult the confirm provider")
    };
    let report = cerulion_cli_engine::partition_emit::graph_partition(
        &ws.root,
        "skewed",
        &cerulion_cli_engine::partition_emit::PartitionOptions {
            costs_path: None,
            budget_ns: None,
            dry_run: false,
            assume_yes: true,
        },
        false,
        &mut confirm,
    )
    .expect("verb persists the refined emit");
    assert!(
        report.level_assignments_written,
        "precondition: block persisted"
    );
    let persisted = std::fs::read_to_string(ws.graphs_dir.join("skewed.yaml")).expect("read");

    // Step 2: the preflight re-derive over the PERSISTED file reproduces the
    // file's own (refined-banded) groups ⇒ AlreadyCurrent, byte-untouched.
    let pre = run_auto_partition_preflight(
        &ws.root,
        "skewed",
        read_config(&ws, "skewed"),
        &read_raw(&ws, "skewed"),
        &PreflightOptions {
            re_derive: true,
            lenient_costs: false,
            assume_yes: true,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect("preflight over the persisted file");
    assert!(
        matches!(pre.outcome, RunPartitionOutcome::AlreadyCurrent),
        "banding over the persisted refined levels reproduces the file exactly; got {:?}",
        pre.outcome
    );
    let after = std::fs::read_to_string(ws.graphs_dir.join("skewed.yaml")).expect("read");
    assert_eq!(
        after, persisted,
        "the persisted file (incl. the block) is byte-untouched"
    );
    // And the config the run proceeds with carries the persisted block.
    assert!(
        pre.config.level_assignments.is_some(),
        "the run consumes the file's persisted level_assignments"
    );
}

// ==========================================================================
// The `block` co-location constraint on the RUN preflight — the
// literal default path (`cerulion graph run` on an unpartitioned graph).
// ==========================================================================

/// Turn a scaffolded node type's `#[input(trigger)]` into a `block` input by
/// editing its SOURCE, which is what `source_entry_infos` parses — so these
/// arms exercise the real metadata plumbing rather than a hand-built
/// `InputMeta`.
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

/// HEADLINE for the run path: the no-TTY floor derives IN MEMORY, and that
/// derivation must co-locate the `block` edge. This is the arm that goes
/// through `source_entry_infos` over real node SOURCE, so it is what kills a
/// revert of the source-parsing plumbing at the pure layer.
#[test]
fn the_no_tty_default_derives_a_partition_that_co_locates_the_block_edge() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    make_input_block(&ws, "sink");
    write_graph(&ws, "chain", CHAIN_YAML);

    let mut confirm = panic_confirm;
    let pre = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: false,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect("in-memory preflight");
    assert_eq!(pre.outcome, RunPartitionOutcome::InMemory);
    assert_eq!(
        shape(&pre.config.process_groups),
        expect(&[("grp_n0", &["n0"]), ("grp_n1", &["n1", "n2"])]),
        "the `block` producer n1 and its `block` consumer n2 must be ONE group; n0 is \
         unconstrained and stays a singleton"
    );
    // The floor never mutates the file.
    assert_eq!(
        std::fs::read_to_string(ws.graphs_dir.join("chain.yaml")).unwrap(),
        CHAIN_YAML
    );
}

/// ANTI-TAUTOLOGY: the identical workspace with no `block` input derives the
/// documented process-per-node baseline.
#[test]
fn the_no_tty_default_without_block_derives_the_untouched_baseline() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    write_graph(&ws, "chain", CHAIN_YAML);

    let mut confirm = panic_confirm;
    let pre = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: false,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect("in-memory preflight");
    assert_eq!(shape(&pre.config.process_groups), baseline_shape());
}

/// PARITY: the co-location rule must be applied on BOTH adoption paths. A rule
/// wired into only one of them is exactly what this pin's sibling
/// (`in_memory_and_written_configs_produce_identical_deployment_plans`) exists
/// to catch, re-run over a block-carrying graph.
#[test]
fn in_memory_and_written_block_partitions_are_identical() {
    let tmp = tempfile::tempdir().unwrap();
    let ws = setup_workspace(tmp.path());
    make_input_block(&ws, "sink");
    write_graph(&ws, "chain", CHAIN_YAML);

    let mut confirm = panic_confirm;
    let in_memory = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: false,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect("in-memory preflight");
    let persisted = run_auto_partition_preflight(
        &ws.root,
        "chain",
        read_config(&ws, "chain"),
        &read_raw(&ws, "chain"),
        &PreflightOptions {
            re_derive: false,
            lenient_costs: true,
            assume_yes: true,
            is_tty: false,
        },
        &mut confirm,
    )
    .expect("persisted preflight");
    assert!(matches!(
        persisted.outcome,
        RunPartitionOutcome::Persisted { .. }
    ));
    let written = read_config(&ws, "chain");
    assert_eq!(
        shape(&in_memory.config.process_groups),
        shape(&written.process_groups),
        "the in-memory and written partitions must agree on the block co-location"
    );
}

// ==========================================================================
// The preflight takes the CALLER's bytes.
// ==========================================================================

/// `run_auto_partition_preflight` must not read the graph file itself.
///
/// If it did, `graph_run` would parse `config` from its own read, the preflight
/// would re-read the file for the splice, and the write would hold
/// `ExpectedPrior::Contents` captured at that LATER read. The derivation
/// input and the held precondition could then describe different revisions, and a
/// save landing in the gap between them — cdylib staleness walks plus a
/// syn-parsing validation report, tens to hundreds of ms, not the microseconds
/// `node_stage` closes — would satisfy the precondition and be silently replaced
/// by groups derived from the pre-save revision. That is the lost-update class
/// the precondition exists to close; `graph partition` (the verb) holds the
/// same single-read shape.
///
/// STRUCTURAL, mirroring `node_stage_preservation_test`'s single-read pin, and
/// for the same reason: the window is machine-speed inside one call, so making
/// it behavioural would need a fault seam in a shipping path.
#[test]
fn the_preflight_reads_no_file_and_holds_the_callers_bytes() {
    let src = std::fs::read_to_string("src/partition_emit.rs").expect("readable");
    let body = code_only(fn_body(&src, "pub fn run_auto_partition_preflight("));

    for spelling in ["read_to_string", "fs::read(", "graph_read"] {
        assert!(
            !body.contains(spelling),
            "the preflight must not read the graph file (`{spelling}`) — it takes \
             the caller's bytes\n{body}"
        );
    }
    assert!(
        body.contains("ExpectedPrior::Contents(raw)"),
        "and it must hold exactly those bytes as the write precondition\n{body}"
    );

    // ANTI-TAUTOLOGY: the verb, which legitimately DOES its own read, still
    // does — so this pins where the read belongs rather than banning reads.
    let verb = code_only(fn_body(&src, "pub fn graph_partition("));
    assert!(
        verb.contains("read_to_string"),
        "`graph partition` owns its read (it parses its config from it)\n{verb}"
    );
}

/// ANTI-TAUTOLOGY for the two helpers the walk depends on.
#[test]
fn the_walk_helpers_are_sound() {
    let one = fn_body(
        "fn a() {\n  read_to_string(x);\n}\nfn b() {\n  let y = 2;\n}\n",
        "fn b(",
    );
    assert!(one.contains("let y") && !one.contains("read_to_string"));
    let stripped = code_only("let a = 1; // read_to_string\nlet b = 2;");
    assert!(stripped.contains("let a = 1;") && !stripped.contains("read_to_string"));
}

/// `src` with COMMENTS removed — line and (depth-tracked) block. A comment
/// that merely NAMES a token must neither satisfy a requirement nor trip a
/// prohibition in the walks above.
fn code_only(src: &str) -> String {
    let chars: Vec<char> = src.chars().collect();
    let mut out = String::with_capacity(src.len());
    let (mut i, mut depth) = (0usize, 0usize);
    while i < chars.len() {
        if depth == 0 && chars[i] == '/' && chars.get(i + 1) == Some(&'/') {
            while i < chars.len() && chars[i] != '\n' {
                i += 1;
            }
            continue;
        }
        if chars[i] == '/' && chars.get(i + 1) == Some(&'*') {
            depth += 1;
            i += 2;
            continue;
        }
        if depth > 0 && chars[i] == '*' && chars.get(i + 1) == Some(&'/') {
            depth -= 1;
            i += 2;
            continue;
        }
        if depth == 0 {
            out.push(chars[i]);
        }
        i += 1;
    }
    out
}

/// The brace-matched body of the function whose signature starts with
/// `signature`.
fn fn_body<'a>(src: &'a str, signature: &str) -> &'a str {
    let start = src
        .find(signature)
        .unwrap_or_else(|| panic!("no function matching {signature:?}"));
    let open = start + src[start..].find('{').expect("a body");
    let mut depth = 0usize;
    for (offset, ch) in src[open..].char_indices() {
        match ch {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return &src[open..=open + offset];
                }
            }
            _ => {}
        }
    }
    panic!("unbalanced body for {signature:?}");
}
