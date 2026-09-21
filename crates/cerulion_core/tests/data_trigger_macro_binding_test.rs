// SPDX-License-Identifier: AGPL-3.0-only
//! The runtime synthesizes a `DataTriggerBinding`
//! from `MacroPolicy::DataTrigger` for nodes the macro reports a
//! trigger field for.
//!
//! Graph YAML now carries no `policy:` block — the macro
//! is the only signal, so this file's prior coverage of the YAML
//! `policy: { trigger: ... }` shape (and the YAML-vs-macro parity
//! test) is gone with it.
//!
//! These tests cover the runtime synthesis logic — they exercise
//! `GraphRuntime::build_for_test` with crafted
//! `ClosureNodeEntry`-backed `NodeInfo`s carrying
//! `MacroPolicy::DataTrigger { input_name }` so the test fixture is
//! parallel-safe (no iceoryx2 SHM).
//!
//! The macro→cdylib emission half is covered separately by
//! `macro_cdylib_policy_round_trip_test::cdylib_dedicated_data_trigger_fixture_round_trips`
//! (load → `info()` JSON parse).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{NodeContext, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::{BackpressurePolicy, InputMeta};
use cerulion_core::{ClosureNodeEntry, MacroPolicy, NodeInfo, TransportError};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

const TEST_PREFIX: &str = "a_a3";

fn meta_input(name: &str, trigger: bool) -> InputMeta {
    InputMeta {
        name: name.to_string(),
        schema_hash: 0xCAFE_BABE,
        trigger,
        depth: 1,
        backpressure: BackpressurePolicy::DropOldest,
        expect_within_ms: None,
    }
}

// ===========================================================================
// 1. Happy path — macro DataTrigger + matching YAML input → build succeeds.
// ===========================================================================

#[test]
fn build_for_test_synthesizes_binding_from_macro_data_trigger() {
    let src_info = NodeInfo::from_names(vec![], vec!["out".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 1 });
    let src_entry = ClosureNodeEntry::new(src_info, |_ctx: &mut NodeContext| Ok(()));

    let consumer_info = NodeInfo::with_meta(vec![meta_input("count", true)], vec![]).with_policy(
        MacroPolicy::DataTrigger {
            input_name: "count".to_string(),
        },
    );
    let consumer_entry = ClosureNodeEntry::new(consumer_info, |_ctx: &mut NodeContext| Ok(()));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "a3_happy".to_string(),
        prefix: TEST_PREFIX.to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "src".to_string(),
                node_type: "src".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "geometry_msgs/Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                ros2: None,
                id: "consumer".to_string(),
                node_type: "consumer".to_string(),
                inputs: vec![InputDef {
                    name: "count".to_string(),
                    source: "src/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("src".to_string(), Box::new(src_entry));
    factories.insert("consumer".to_string(), Box::new(consumer_entry));

    let clock = Arc::new(VirtualClock::new());
    if GraphRuntime::build_for_test(config, factories, clock, 4).is_err() {
        panic!("build with macro DataTrigger + matching YAML input must succeed");
    }
}

// ===========================================================================
// 2. End-to-end fire — publish on the trigger source, consumer ticks.
// ===========================================================================

#[test]
fn macro_data_trigger_consumer_fires_when_source_publishes() {
    let consumer_fire_count = Arc::new(AtomicU64::new(0));
    let cfc_for_consumer = Arc::clone(&consumer_fire_count);

    let src_info = NodeInfo::from_names(vec![], vec!["out".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 10 });
    let src_entry = ClosureNodeEntry::new(src_info, |ctx: &mut NodeContext| {
        let mut proxy = ctx
            .publisher_mut("out")
            .expect("publisher 'out' must exist")
            .loan_proxy::<Vector3>()?;
        proxy.x = 1.0;
        proxy.y = 2.0;
        proxy.z = 3.0;
        Ok(())
    });

    let consumer_info = NodeInfo::with_meta(vec![meta_input("count", true)], vec![]).with_policy(
        MacroPolicy::DataTrigger {
            input_name: "count".to_string(),
        },
    );
    let consumer_entry = ClosureNodeEntry::new(consumer_info, move |_ctx: &mut NodeContext| {
        cfc_for_consumer.fetch_add(1, Ordering::Relaxed);
        Ok(())
    });

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "a3_fire".to_string(),
        prefix: TEST_PREFIX.to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "src".to_string(),
                node_type: "src".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "geometry_msgs/Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                ros2: None,
                id: "consumer".to_string(),
                node_type: "consumer".to_string(),
                inputs: vec![InputDef {
                    name: "count".to_string(),
                    source: "src/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("src".to_string(), Box::new(src_entry));
    factories.insert("consumer".to_string(), Box::new(consumer_entry));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = match GraphRuntime::build_for_test(config, factories, clock, 16) {
        Ok(r) => r,
        Err(_) => panic!("build must succeed"),
    };

    for _ in 0..5 {
        runtime.step(Duration::from_millis(10));
    }

    let count = consumer_fire_count.load(Ordering::Relaxed);
    assert!(
        count >= 4,
        "macro DataTrigger consumer must fire at least 4 times for 5 source \
         publishes (allowing one-step pre-drain lag); got {count}"
    );
    assert!(
        count <= 5,
        "macro DataTrigger consumer must not fire more than 5 times for 5 \
         source publishes; got {count} (suggests double-binding bug)"
    );
}

// ===========================================================================
// 3. Error — macro names a trigger field that YAML doesn't wire as input.
// ===========================================================================

#[test]
fn build_fails_when_macro_data_trigger_input_name_not_in_yaml_inputs() {
    let consumer_info = NodeInfo::with_meta(vec![meta_input("count", true)], vec![]).with_policy(
        MacroPolicy::DataTrigger {
            input_name: "count".to_string(),
        },
    );
    let consumer_entry = ClosureNodeEntry::new(consumer_info, |_ctx: &mut NodeContext| Ok(()));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "a3_missing_input".to_string(),
        prefix: TEST_PREFIX.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "consumer".to_string(),
            node_type: "consumer".to_string(),
            inputs: vec![],
            outputs: vec![],
        }],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("consumer".to_string(), Box::new(consumer_entry));

    let clock = Arc::new(VirtualClock::new());
    let err = match GraphRuntime::build_for_test(config, factories, clock, 4) {
        Ok(_) => panic!("build must fail when macro input_name is unwired"),
        Err(e) => e,
    };
    let reason = match err {
        TransportError::GraphError { reason } => reason,
        other => panic!("expected GraphError, got: {other:?}"),
    };
    assert!(reason.contains("consumer"), "node id missing: {reason}");
    assert!(reason.contains("count"), "input name missing: {reason}");
    assert!(
        reason.contains("macro"),
        "error must disambiguate macro-path: {reason}"
    );
    assert!(
        reason.contains("inputs:"),
        "remediation hint missing: {reason}"
    );
    assert!(
        reason.contains("no YAML inputs declared"),
        "empty-inputs hint missing: {reason}"
    );
}

// ===========================================================================
// 3a. Error — macro names a trigger field whose source is the node's own
// output (infinite loop).
// ===========================================================================

#[test]
fn build_fails_when_macro_data_trigger_input_loops_to_self() {
    let consumer_info = NodeInfo::with_meta(vec![meta_input("self_loop", true)], vec![])
        .with_policy(MacroPolicy::DataTrigger {
            input_name: "self_loop".to_string(),
        });
    let consumer_entry = ClosureNodeEntry::new(consumer_info, |_ctx: &mut NodeContext| Ok(()));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "a3_self_loop".to_string(),
        prefix: TEST_PREFIX.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "looper".to_string(),
            node_type: "looper".to_string(),
            inputs: vec![InputDef {
                name: "self_loop".to_string(),
                source: "looper/out".to_string(),
            }],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "geometry_msgs/Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("looper".to_string(), Box::new(consumer_entry));

    let clock = Arc::new(VirtualClock::new());
    let err = match GraphRuntime::build_for_test(config, factories, clock, 4) {
        Ok(_) => panic!("build must fail on self-loop macro DataTrigger"),
        Err(e) => e,
    };
    let reason = match err {
        TransportError::GraphError { reason } => reason,
        other => panic!("expected GraphError, got: {other:?}"),
    };
    assert!(reason.contains("looper"), "node id missing: {reason}");
    // Level executor: the trigger-aware DAG level derivation rejects a
    // DataTrigger self-loop as an algebraic cycle in `derive_levels`, BEFORE
    // the defense-in-depth `infinite loop` check in
    // `resolve_macro_data_trigger_input` runs (see its doc comment). The
    // shipped error is the cycle diagnostic, which names the closed ring
    // rather than the offending output topic.
    assert!(
        reason.contains("algebraic trigger cycle"),
        "must name the algebraic trigger cycle: {reason}"
    );
    assert!(
        reason.contains("looper -> looper"),
        "must show the self-cycle ring naming the offending node: {reason}"
    );
}

// ===========================================================================
// 3a'. The self-loop check compares
// RESOLVED topics — a node triggering on its own `topic:`-OVERRIDDEN
// output via an absolute source is the same infinite loop (a revert to
// short-ref equality lets this build and tick forever).
// ===========================================================================

#[test]
fn build_fails_when_macro_data_trigger_loops_to_own_overridden_output() {
    let consumer_info = NodeInfo::with_meta(vec![meta_input("self_loop", true)], vec![])
        .with_policy(MacroPolicy::DataTrigger {
            input_name: "self_loop".to_string(),
        });
    let consumer_entry = ClosureNodeEntry::new(consumer_info, |_ctx: &mut NodeContext| Ok(()));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "a3_override_loop".to_string(),
        prefix: TEST_PREFIX.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "looper".to_string(),
            node_type: "looper".to_string(),
            inputs: vec![InputDef {
                name: "self_loop".to_string(),
                source: "/loop_tf".to_string(),
            }],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "geometry_msgs/Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: Some("/loop_tf".to_string()),
            }],
        }],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("looper".to_string(), Box::new(consumer_entry));

    let clock = Arc::new(VirtualClock::new());
    let err = match GraphRuntime::build_for_test(config, factories, clock, 4) {
        Ok(_) => panic!("build must fail: absolute source names the node's own overridden output"),
        Err(e) => e,
    };
    let reason = match err {
        TransportError::GraphError { reason } => reason,
        other => panic!("expected GraphError, got: {other:?}"),
    };
    // Level executor: caught as an algebraic cycle by `derive_levels`.
    // `build_trigger_edges` RECORDS this node's self-edge using the RESOLVED
    // `/loop_tf` topic (the resolved-topic comparison this test pins — now in
    // the level-derivation layer); `derive_levels` then REJECTS the self-cycle.
    assert!(
        reason.contains("algebraic trigger cycle"),
        "must name the algebraic trigger cycle: {reason}"
    );
    assert!(
        reason.contains("looper -> looper"),
        "must show the self-cycle ring: {reason}"
    );
}

// ===========================================================================
// 3b. Error — name-mismatch error lists existing YAML inputs so an
// operator with a typo can spot the right name.
// ===========================================================================

#[test]
fn name_mismatch_error_lists_existing_yaml_inputs() {
    let src_info = NodeInfo::from_names(
        vec![],
        vec!["tally_out".to_string(), "total_out".to_string()],
    )
    .with_policy(MacroPolicy::Period { period_ms: 10 });
    let src_entry = ClosureNodeEntry::new(src_info, |_ctx: &mut NodeContext| Ok(()));

    let consumer_info = NodeInfo::with_meta(vec![meta_input("count", true)], vec![]).with_policy(
        MacroPolicy::DataTrigger {
            input_name: "count".to_string(),
        },
    );
    let consumer_entry = ClosureNodeEntry::new(consumer_info, |_ctx: &mut NodeContext| Ok(()));

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "a3_typo_hint".to_string(),
        prefix: TEST_PREFIX.to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "src".to_string(),
                node_type: "src".to_string(),
                inputs: vec![],
                outputs: vec![
                    OutputDef {
                        name: "tally_out".to_string(),
                        schema: "geometry_msgs/Vector3".to_string(),
                        max_slice_len: None,
                        history_size: 0,
                        topic: None,
                    },
                    OutputDef {
                        name: "total_out".to_string(),
                        schema: "geometry_msgs/Vector3".to_string(),
                        max_slice_len: None,
                        history_size: 0,
                        topic: None,
                    },
                ],
            },
            NodeDef {
                ros2: None,
                id: "consumer".to_string(),
                node_type: "consumer".to_string(),
                inputs: vec![
                    InputDef {
                        name: "tally".to_string(),
                        source: "src/tally_out".to_string(),
                    },
                    InputDef {
                        name: "total".to_string(),
                        source: "src/total_out".to_string(),
                    },
                ],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("src".to_string(), Box::new(src_entry));
    factories.insert("consumer".to_string(), Box::new(consumer_entry));

    let clock = Arc::new(VirtualClock::new());
    let err = match GraphRuntime::build_for_test(config, factories, clock, 4) {
        Ok(_) => panic!("build must fail when macro input_name unwired"),
        Err(e) => e,
    };
    let reason = match err {
        TransportError::GraphError { reason } => reason,
        other => panic!("expected GraphError, got: {other:?}"),
    };
    assert!(
        reason.contains("YAML inputs:"),
        "error must list existing YAML inputs: {reason}"
    );
    assert!(reason.contains("tally"), "must list `tally`: {reason}");
    assert!(reason.contains("total"), "must list `total`: {reason}");
}
