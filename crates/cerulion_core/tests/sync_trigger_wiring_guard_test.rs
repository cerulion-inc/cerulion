// SPDX-License-Identifier: AGPL-3.0-only
//! The never-fire re-audit: loud guards on
//! every path where the narrowed sync trigger set (Sync aligns
//! ONLY `#[input(trigger)]`-marked inputs) could silently starve a node.
//!
//! Guards pinned here, over `build_for_test` (per-test SHM root —
//! parallel-safe) with closure entries carrying hand-built `InputMeta`:
//!
//! 1. **Unwired trigger port = loud build error** —
//!    `validate_macro_sync_trigger_wiring` (the Sync analogue of
//!    `resolve_macro_data_trigger_input`): a trigger-marked port missing
//!    from the node's YAML `inputs:` would silently SHRINK the alignment
//!    set (the node fires without waiting for it). Error names the node,
//!    the field, and the YAML fix. Its self-loop arm is defense-in-depth
//!    (the trigger-aware DAG cycle check catches it upstream) and is
//!    unit-pinned inline in `graph/runtime.rs`.
//! 2. **<2 wired trigger inputs warns** — the pre-existing
//!    `validate_no_silent_data_trigger` 1-trigger warn covers the
//!    degenerate-but-functional single-trigger sync (severity WARN: the
//!    macro accepts the combination; the node still fires). Zero trigger
//!    marks anywhere = the terminal ERROR below (the node can NEVER fire).
//! 3. **The terminal empty-set error** (`Scheduler::add_node`) — reworded
//!    to name the CAUSE (zero `#[input(trigger)]`-marked wired inputs) and
//!    the FIX (mark >=2 inputs trigger, or use a different policy).
//!
//! Sibling files: `sync_attr_1_trigger_warn_test.rs` (the 1-trigger warn
//! matrix), `macro_sync_threading_test.rs` (the cdylib zero-wired arm),
//! `scheduler_test.rs::test_empty_sync_inputs_rejected` (the terminal
//! message at the scheduler seam), `sync_fire_iox2_test.rs` (fire
//! semantics + the starved-trigger watchdog arm).

use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{NodeContext, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::{BackpressurePolicy, InputMeta};
use cerulion_core::{ClosureNodeEntry, MacroPolicy, NodeInfo};
use indexmap::IndexMap;
use tracing_test::traced_test;

const TEST_PREFIX: &str = "wiring_guard";

/// Load-bearing message substrings (oracle constants — a reword upstream
/// must consciously update BOTH sides).
const UNWIRED_PHRASE: &str = "doesn't wire an input named";
const EMPTY_SET_PHRASE: &str = "Sync trigger set is empty";
const EMPTY_SET_FIX_PHRASE: &str = "Mark >=2 wired inputs `#[input(trigger)]`";
const ZERO_RESOLVED_WARN_PHRASE: &str = "resolved ZERO `#[input(trigger)]`-marked wired inputs";
const SILENT_IGNORE_PHRASE: &str = "`sync_window_ms` is silently ignored";

fn meta_input(name: &str, trigger: bool) -> InputMeta {
    InputMeta {
        name: name.to_string(),
        schema_hash: 0xC425,
        trigger,
        depth: 1,
        backpressure: BackpressurePolicy::DropOldest,
        expect_within_ms: None,
    }
}

fn make_entry(info: NodeInfo) -> Box<dyn NodeEntry> {
    Box::new(ClosureNodeEntry::new(info, |_ctx: &mut NodeContext| Ok(())).with_label("guard"))
}

fn out_def(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "u8".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    }
}

/// Two-node graph: `source` (Period, outputs `out_a` + `out_b`) feeding
/// `fuse`, whose YAML `inputs:` list is supplied by the caller.
fn two_node_config(fuse_inputs: Vec<InputDef>) -> GraphConfig {
    GraphConfig {
        execution: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        level_assignments: None,
        network: None,
        name: None,
        identity: "guard_graph".to_string(),
        prefix: TEST_PREFIX.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "source".to_string(),
                node_type: "source".to_string(),
                inputs: vec![],
                outputs: vec![out_def("out_a"), out_def("out_b")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "fuse".to_string(),
                node_type: "fuse".to_string(),
                inputs: fuse_inputs,
                outputs: vec![],
            },
        ],
    }
}

fn build(config: GraphConfig, fuse_info: NodeInfo) -> cerulion_core::TransportResult<GraphRuntime> {
    let source_info = NodeInfo::from_names(vec![], vec!["out_a".to_string(), "out_b".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("source".to_string(), make_entry(source_info));
    factories.insert("fuse".to_string(), make_entry(fuse_info));
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 4)
}

fn wired(name: &str, source: &str) -> InputDef {
    InputDef {
        name: name.to_string(),
        source: source.to_string(),
    }
}

// ===========================================================================
// Guard 1: an UNWIRED `#[input(trigger)]` port on a Sync node fails the
// build, naming the node, the field, and the YAML fix.
// ===========================================================================

#[test]
fn unwired_sync_trigger_port_fails_build_naming_field_and_hint() {
    // fuse declares trigger-marked `a` AND `b`, but YAML wires only `a`.
    // Pre-guard, the sync alignment set silently shrank to {a} — the node
    // fired without ever waiting for `b`.
    let fuse_info = NodeInfo::with_meta(
        vec![meta_input("a", true), meta_input("b", true)],
        Vec::new(),
    )
    .with_policy(MacroPolicy::Sync { window_ms: 50 });
    let config = two_node_config(vec![wired("a", "source/out_a")]);

    let err = match build(config, fuse_info) {
        Ok(_) => panic!("an unwired `#[input(trigger)]` port must fail the build"),
        Err(e) => e,
    };
    let msg = err.to_string();
    assert!(msg.contains("fuse"), "must name the node; got: {msg}");
    assert!(
        msg.contains("'b'"),
        "must name the unwired trigger field; got: {msg}"
    );
    assert!(
        msg.contains(UNWIRED_PHRASE),
        "must state the cause; got: {msg}"
    );
    assert!(
        msg.contains("(YAML inputs: [a])"),
        "must list the existing YAML inputs (typo-spotting hint); got: {msg}"
    );
    assert!(
        msg.contains("a `name: b` entry with its `source: <topic>`"),
        "must state the YAML fix naming both keys; got: {msg}"
    );
    assert!(
        msg.contains("silently shrink"),
        "must explain WHY (the alignment set would shrink); got: {msg}"
    );
}

/// The UnboundedSync arm routes through the SAME guard (the call site gates
/// on both policy variants).
#[test]
fn unwired_unbounded_sync_trigger_port_also_rejected() {
    let fuse_info = NodeInfo::with_meta(
        vec![meta_input("a", true), meta_input("b", true)],
        Vec::new(),
    )
    .with_policy(MacroPolicy::UnboundedSync);
    let config = two_node_config(vec![wired("b", "source/out_b")]);

    let err = match build(config, fuse_info) {
        Ok(_) => panic!("an unwired trigger port on an UnboundedSync node must fail the build"),
        Err(e) => e,
    };
    let msg = err.to_string();
    assert!(
        msg.contains("'a'") && msg.contains(UNWIRED_PHRASE),
        "UnboundedSync must hit the same unwired-trigger guard (field 'a'); got: {msg}"
    );
    assert!(
        msg.contains("(YAML inputs: [b])"),
        "hint must list the wired inputs; got: {msg}"
    );
}

// ===========================================================================
// Guard 2 (severity pin): exactly ONE wired trigger input + a plain sibling
// → the degenerate single-trigger sync WARNS (build still succeeds — the
// macro accepts this combination; the node fires on the single trigger).
// ===========================================================================

#[test]
#[traced_test]
fn single_wired_trigger_with_plain_sibling_warns_and_builds() {
    let fuse_info = NodeInfo::with_meta(
        vec![meta_input("data", true), meta_input("ctx", false)],
        Vec::new(),
    )
    .with_policy(MacroPolicy::Sync { window_ms: 50 });
    let config = two_node_config(vec![
        wired("data", "source/out_a"),
        wired("ctx", "source/out_b"),
    ]);

    let built = build(config, fuse_info);
    assert!(
        built.is_ok(),
        "1 wired trigger + 1 plain input is degenerate-but-functional — \
         severity is WARN, not error; got {:?}",
        built.err()
    );
    assert!(
        logs_contain(SILENT_IGNORE_PHRASE),
        "the <2-trigger sync warn must fire (the plain `ctx` input does NOT \
         count toward the alignment set under trigger-scoped Sync)"
    );
}

// ===========================================================================
// Guard 3: ZERO trigger marks anywhere + a Sync policy → the REWORDED
// terminal empty-set error (cause + fix + node id), preceded by the
// reworded macro_policy_to_trigger warn.
// ===========================================================================

#[test]
#[traced_test]
fn closure_sync_with_no_trigger_marks_hits_reworded_terminal_error() {
    // `from_names` carries NO InputMeta → zero trigger marks → the unwired
    // guard is vacuous, macro_policy_to_trigger resolves an EMPTY trigger
    // set (reworded warn), and Scheduler::add_node rejects with the
    // reworded terminal error.
    let fuse_info =
        NodeInfo::from_names(vec![], vec![]).with_policy(MacroPolicy::Sync { window_ms: 50 });
    let config = two_node_config(vec![]);

    let err = match build(config, fuse_info) {
        Ok(_) => panic!("a Sync node with zero trigger marks must fail the build"),
        Err(e) => e,
    };
    let msg = err.to_string();
    assert!(
        msg.contains(EMPTY_SET_PHRASE),
        "terminal error must name the cause; got: {msg}"
    );
    assert!(msg.contains("fuse"), "must name the node; got: {msg}");
    assert!(
        msg.contains(EMPTY_SET_FIX_PHRASE),
        "terminal error must state the fix; got: {msg}"
    );
    assert!(
        logs_contain(ZERO_RESOLVED_WARN_PHRASE),
        "the reworded macro_policy_to_trigger warn must fire before the reject"
    );
}

// ===========================================================================
// Control (anti-tautology): a correctly-wired 2-trigger sync node builds
// with ZERO wiring-guard diagnostics — the guards bite misconfigurations only.
// ===========================================================================

#[test]
#[traced_test]
fn well_formed_two_trigger_sync_builds_with_zero_new_warns() {
    let fuse_info = NodeInfo::with_meta(
        vec![meta_input("a", true), meta_input("b", true)],
        Vec::new(),
    )
    .with_policy(MacroPolicy::Sync { window_ms: 50 });
    let config = two_node_config(vec![wired("a", "source/out_a"), wired("b", "source/out_b")]);

    let built = build(config, fuse_info);
    assert!(
        built.is_ok(),
        "a correctly-wired 2-trigger sync node must build; got {:?}",
        built.err()
    );
    assert!(
        !logs_contain(UNWIRED_PHRASE),
        "no unwired-trigger diagnostic on a well-formed node"
    );
    assert!(
        !logs_contain(EMPTY_SET_PHRASE),
        "no empty-set diagnostic on a well-formed node"
    );
    assert!(
        !logs_contain(ZERO_RESOLVED_WARN_PHRASE),
        "no zero-resolved warn on a well-formed node"
    );
    assert!(
        !logs_contain("silently ignored"),
        "no single-trigger silent-ignore warn on a 2-trigger node"
    );
}
