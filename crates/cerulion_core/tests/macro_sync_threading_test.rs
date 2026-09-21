// SPDX-License-Identifier: AGPL-3.0-only
//! Tests that
//! `MacroPolicy::Sync` threads YAML-wired inputs through
//! `macro_policy_to_trigger`. Without the threading the conversion produces
//! `TriggerPolicy::Sync { inputs: Vec::new(), .. }` — empty inputs
//! mean the scheduler never fires the node, silently failing.
//!
//! These tests load the `test_node_macro_sync_cdylib` fixture (which
//! declares `#[cerulion_node(sync_window_ms = 25)]` with two trigger
//! inputs `cam` and `imu`) and verify the runtime synthesizes a
//! `TriggerPolicy::Sync` with both inputs resolved against the graph
//! prefix when the YAML doesn't override.
//!
//! We can't directly read the scheduler's stored `TriggerPolicy` from
//! outside `cerulion_core::graph::runtime`, so the tests assert
//! observable behavior:
//!
//! 1. Building a `GraphRuntime` with a macro Sync node + zero
//!    YAML-wired inputs emits the configured warn (caught indirectly:
//!    `TriggerPolicy::Sync { inputs: empty }` means the node never
//!    fires).
//! 2. Building a `GraphRuntime` with macro Sync + both inputs wired
//!    in YAML succeeds and the node fires when both topics receive
//!    data within the window.

use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeContext, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::ClosureNodeEntry;
use cerulion_core::NodeInfo;
use indexmap::IndexMap;

/// Locate the macro sync cdylib fixture.
fn find_sync_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_sync_cdylib")
}

/// Smoke test: macro Sync fixture has the expected info() shape with
/// 2 input ports + Sync policy. Locks the test fixture's contract so
/// the synthesis tests below have a known starting state.
#[test]
fn sync_cdylib_has_two_inputs_and_sync_policy() {
    let path = find_sync_cdylib();
    let node = DylibNodeEntry::load(&path).expect("load sync cdylib");
    let info = node.info().expect("info should parse");
    assert_eq!(info.input_names(), &["cam".to_string(), "imu".to_string()]);
    assert!(matches!(
        info.policy(),
        Some(cerulion_core::graph::node::MacroPolicy::Sync { window_ms: 25 })
    ));
}

/// Building a GraphRuntime with a macro Sync node
/// AND YAML-wired inputs (`cam` ← `producer/cam_out`, `imu` ←
/// `producer/imu_out`) succeeds. The runtime threads both inputs
/// through `macro_policy_to_trigger` (synthesizing a `TriggerPolicy::Sync`
/// with both topics resolved against the graph prefix). Without the threading this
/// would build a Sync trigger with EMPTY inputs and the node
/// would never fire — but the build itself would not fail.
///
/// Building succeeds is the load-bearing first half. End-to-end
/// firing is hard to assert here without wiring real publishers for
/// both topics; `sync_fire_iox2_test.rs` covers it (the cdylib
/// loader exposes no way to inspect the stored
/// TriggerPolicy from here).
#[test]
fn macro_sync_with_two_yaml_wired_inputs_builds_successfully() {
    let path = find_sync_cdylib();
    let fuser_entry = DylibNodeEntry::load(&path).expect("load fuser");

    // Producer node: publishes to both `cam_out` and `imu_out` so the
    // fuser's inputs resolve. Period-driven (10ms) so the producer is
    // always-firing; the test only checks that the runtime BUILDS,
    // not that it runs.
    let producer_info =
        NodeInfo::from_names(vec![], vec!["cam_out".to_string(), "imu_out".to_string()])
            .with_policy(cerulion_core::MacroPolicy::Period { period_ms: 10 });
    let producer = ClosureNodeEntry::new(producer_info, |_ctx: &mut NodeContext| Ok(()));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "macro_sync_yaml_wired".to_string(),
        prefix: "msyw".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "producer".to_string(),
                node_type: "producer".to_string(),
                inputs: vec![],
                outputs: vec![
                    OutputDef {
                        name: "cam_out".to_string(),
                        schema: "u8".to_string(),
                        max_slice_len: None,
                        history_size: 0,
                        topic: None,
                    },
                    OutputDef {
                        name: "imu_out".to_string(),
                        schema: "u8".to_string(),
                        max_slice_len: None,
                        history_size: 0,
                        topic: None,
                    },
                ],
            },
            NodeDef {
                ros2: None,
                id: "fuser".to_string(),
                node_type: "fuser".to_string(),
                // macro policy drives — Sync window_ms=25
                inputs: vec![
                    InputDef {
                        name: "cam".to_string(),
                        source: "producer/cam_out".to_string(),
                    },
                    InputDef {
                        name: "imu".to_string(),
                        source: "producer/imu_out".to_string(),
                    },
                ],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(producer));
    factories.insert("fuser".to_string(), Box::new(fuser_entry));

    let clock = std::sync::Arc::new(cerulion_core::clock::VirtualClock::new());
    // build_for_test is sufficient — we're testing trigger synthesis,
    // not iceoryx2 transport.
    let result = GraphRuntime::build_for_test(config, factories, clock, 4);
    assert!(
        result.is_ok(),
        "build_for_test must succeed for macro Sync + YAML-wired inputs (= proof that \
         macro_policy_to_trigger threaded both 'cam_out' and 'imu_out' into the \
         TriggerPolicy::Sync — with an empty inputs vec the scheduler would have \
         rejected the build with the empty-sync-trigger-set error); got {:?}",
        result.err()
    );
}

/// Edge case: macro
/// Sync with ZERO YAML-wired inputs. The sync cdylib's `cam`/`imu` fields
/// are `#[input(trigger)]`-marked (ABI v9 carries the marks), so the build
/// fails FAST at `validate_macro_sync_trigger_wiring` — the FIRST
/// trigger-marked port (`cam`) is unwired, and an unwired trigger port
/// would silently shrink the sync alignment set. The reject is a hard
/// error naming the node, the field, and the YAML fix — strictly more
/// precise than the backstop (empty-set rejection at
/// `Scheduler::add_node`), and it fires BEFORE any iceoryx2 allocation.
/// The silent-failure mode (build succeeds, node never fires,
/// no diagnostic) stays impossible.
#[test]
fn macro_sync_with_zero_yaml_wired_inputs_fails_to_build_with_diagnostic() {
    let path = find_sync_cdylib();
    let entry = DylibNodeEntry::load(&path).expect("load");

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "macro_sync_no_inputs".to_string(),
        prefix: "msni".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "fuser".to_string(),
            node_type: "fuser".to_string(),
            inputs: vec![], // ZERO YAML-wired inputs
            outputs: vec![],
        }],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuser".to_string(), Box::new(entry));

    let clock = std::sync::Arc::new(cerulion_core::clock::VirtualClock::new());
    let result = GraphRuntime::build_for_test(config, factories, clock, 4);
    let err = match result {
        Ok(_) => panic!(
            "macro Sync with zero YAML inputs MUST fail the build (an unwired \
             `#[input(trigger)]` port is rejected loudly rather than silently \
             shrinking the sync alignment set / never firing the node)"
        ),
        Err(e) => e,
    };
    let msg = err.to_string();
    assert!(
        msg.contains("fuser"),
        "build error must name the node; got: {msg}"
    );
    assert!(
        msg.contains("'cam'"),
        "build error must name the first unwired trigger field; got: {msg}"
    );
    assert!(
        msg.contains("doesn't wire an input named"),
        "build error must state the cause (unwired trigger port); got: {msg}"
    );
    assert!(
        msg.contains("node has no YAML inputs declared"),
        "build error must carry the empty-inputs YAML hint; got: {msg}"
    );
}
