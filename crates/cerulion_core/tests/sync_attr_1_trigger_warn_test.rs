// SPDX-License-Identifier: AGPL-3.0-only
//! Verify the runtime warn fires when
//! a node declares `sync_window_ms` or `unbounded_sync` but only has 1
//! `#[input(trigger)]` field. The macro validator silently accepts this
//! combination (since the sync attr is silently ignored with a single
//! trigger), so the loud warn surfaces the surprise at graph-build time.
//!
//! Sibling pattern to `graph_default_policy_warn_test.rs` — uses
//! `tracing-test` to capture the warn into a thread-local subscriber.

use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{NodeContext, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::{BackpressurePolicy, InputMeta};
use cerulion_core::{ClosureNodeEntry, MacroPolicy, NodeInfo, TransportResult};
use indexmap::IndexMap;
use tracing_test::traced_test;

const TEST_PREFIX: &str = "sync_attr_1_trigger_warn";

const SYNC_WINDOW_PHRASE: &str = "`sync_window_ms` is silently ignored";
const UNBOUNDED_SYNC_PHRASE: &str = "`unbounded_sync` is silently ignored";

fn meta_input(name: &str, trigger: bool) -> InputMeta {
    InputMeta {
        name: name.to_string(),
        schema_hash: 0xC0FFEE,
        trigger,
        depth: 1,
        backpressure: BackpressurePolicy::DropOldest,
        expect_within_ms: None,
    }
}

fn make_entry(info: NodeInfo) -> Box<dyn NodeEntry> {
    Box::new(
        ClosureNodeEntry::new(info, |_ctx: &mut NodeContext| Ok(())).with_label("sync_warn_test"),
    )
}

fn graph_config(node_id: &str, source_id: Option<&str>) -> GraphConfig {
    let mut nodes = Vec::new();
    if let Some(src) = source_id {
        nodes.push(NodeDef {
            ros2: None,
            id: src.to_string(),
            node_type: src.to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "u8".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        });
    }
    nodes.push(NodeDef {
        ros2: None,
        id: node_id.to_string(),
        node_type: node_id.to_string(),
        inputs: source_id
            .map(|src| {
                vec![cerulion_core::graph::config::InputDef {
                    name: "data".to_string(),
                    source: format!("{src}/out"),
                }]
            })
            .unwrap_or_default(),
        outputs: vec![],
    });
    GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("warn_{node_id}"),
        prefix: TEST_PREFIX.to_string(),
        nodes,
    }
}

fn try_build(
    config: GraphConfig,
    entries: Vec<(String, Box<dyn NodeEntry>)>,
) -> TransportResult<GraphRuntime> {
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for (id, entry) in entries {
        factories.insert(id, entry);
    }
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 4)
}

// ===========================================================================
// 1. `sync_window_ms` + 1 trigger input → warn fires.
// ===========================================================================

#[test]
#[traced_test]
fn sync_window_ms_with_one_trigger_input_warns() {
    let src_info = NodeInfo::from_names(vec![], vec!["out".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let consumer_info = NodeInfo::with_meta(vec![meta_input("data", true)], vec![])
        .with_policy(MacroPolicy::Sync { window_ms: 50 });

    let config = graph_config("consumer", Some("source"));
    let entries = vec![
        ("source".to_string(), make_entry(src_info)),
        ("consumer".to_string(), make_entry(consumer_info)),
    ];
    if try_build(config, entries).is_err() {
        panic!("build_for_test should succeed (the validator silently ignores sync_window_ms with 1 trigger; the warn is informational)");
    }

    assert!(
        logs_contain(SYNC_WINDOW_PHRASE),
        "1-trigger + sync_window_ms must emit the silently-ignored warn"
    );
    assert!(
        logs_contain("node_id=consumer"),
        "warn must carry the structured `node_id=consumer` field"
    );
}

// ===========================================================================
// 2. `unbounded_sync` + 1 trigger input → warn fires.
// ===========================================================================

#[test]
#[traced_test]
fn unbounded_sync_with_one_trigger_input_warns() {
    let src_info = NodeInfo::from_names(vec![], vec!["out".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let consumer_info = NodeInfo::with_meta(vec![meta_input("data", true)], vec![])
        .with_policy(MacroPolicy::UnboundedSync);

    let config = graph_config("consumer", Some("source"));
    let entries = vec![
        ("source".to_string(), make_entry(src_info)),
        ("consumer".to_string(), make_entry(consumer_info)),
    ];
    if try_build(config, entries).is_err() {
        panic!("build_for_test should succeed (the validator silently ignores unbounded_sync with 1 trigger; the warn is informational)");
    }

    assert!(
        logs_contain(UNBOUNDED_SYNC_PHRASE),
        "1-trigger + unbounded_sync must emit the silently-ignored warn"
    );
}

// ===========================================================================
// 3. Negative coverage — 2+ trigger inputs do NOT trip the silent-ignore warn
// (the sync attr is meaningfully applied, not silently ignored).
// ===========================================================================

#[test]
#[traced_test]
fn sync_window_ms_with_two_trigger_inputs_no_silent_ignore_warn() {
    let info = NodeInfo::from_names(
        vec!["a".to_string(), "b".to_string()],
        vec!["out".to_string()],
    )
    .with_policy(MacroPolicy::Sync { window_ms: 50 });
    // from_names doesn't set the trigger marker; rebuild with explicit
    // InputMeta carrying `trigger: true` on both.
    let _ = info;
    let info = NodeInfo::with_meta(
        vec![meta_input("a", true), meta_input("b", true)],
        Vec::new(),
    )
    .with_policy(MacroPolicy::Sync { window_ms: 50 });

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "sync_2trig".to_string(),
        prefix: TEST_PREFIX.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "consumer".to_string(),
            node_type: "consumer".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "u8".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("consumer".to_string(), make_entry(info));
    let clock = Arc::new(VirtualClock::new());
    // The build may succeed or fail depending on YAML wiring; what we
    // care about is whether the silent-ignore warn fires (it must NOT).
    let _ = GraphRuntime::build_for_test(config, factories, clock, 4);

    assert!(
        !logs_contain(SYNC_WINDOW_PHRASE),
        "2-trigger sync_window_ms must NOT emit the silently-ignored warn — sync is meaningfully applied"
    );
    assert!(
        !logs_contain(UNBOUNDED_SYNC_PHRASE),
        "2-trigger node must not emit the unbounded_sync silently-ignored warn either"
    );
}
