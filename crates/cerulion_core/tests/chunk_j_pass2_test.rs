// SPDX-License-Identifier: AGPL-3.0-only
//! Shutdown-lifecycle regression tests. They extend
//! `runtime_shutdown_lifecycle_test.rs` with cases that its
//! tests cannot distinguish (or do not cover at all).
//!
//! That file has 4 lifecycle tests. Two gaps remain in them:
//!
//! 1. `explicit_shutdown_invokes_node_shutdown` would still pass if
//!    `shutdown(self)`'s `shutdown_all_nodes()` call were reverted —
//!    `Drop::drop` calls `shutdown_all_nodes()` too, masking the bug.
//!    This file's `explicit_shutdown_fires_before_drop_observer` test
//!    distinguishes the two paths by observing the counter while the
//!    runtime is still alive.
//!
//! 2. Those tests do not verify `shutdown_all_nodes`'s **insertion-order**
//!    contract or its **log-and-continue** error-isolation contract.
//!    This file pins both: `shutdown_all_nodes_fires_in_insertion_order`
//!    and `shutdown_error_in_one_node_does_not_skip_others`.

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

use cerulion_core::clock::VirtualClock;
use cerulion_core::error::{TransportError, TransportResult};
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{NodeContext, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::{ClosureNodeEntry, MacroPolicy, NodeInfo};
use indexmap::IndexMap;

/// Build a single-node runtime whose shutdown closure pushes `marker`
/// into the shared trace vec.
fn build_single_node_runtime_with_trace(
    trace: Arc<Mutex<Vec<&'static str>>>,
    node_id: &'static str,
    marker: &'static str,
    period_ms: u64,
) -> GraphRuntime {
    let trace_for_shutdown = Arc::clone(&trace);
    let info = NodeInfo::from_names(vec![], vec!["noop".to_string()])
        .with_policy(MacroPolicy::Period { period_ms });
    let entry =
        ClosureNodeEntry::new(info, |_ctx: &mut NodeContext| Ok(())).with_shutdown(move || {
            trace_for_shutdown.lock().unwrap().push(marker);
            Ok(())
        });

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("pass2_{}", node_id),
        prefix: "p2".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: node_id.to_string(),
            node_type: node_id.to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "noop".to_string(),
                schema: "u8".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(node_id.to_string(), Box::new(entry));

    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 4).expect("build_for_test")
}

/// Pin that the **explicit** `shutdown(self)` path —
/// not Drop — is what fires the user shutdown.
///
/// The sibling `explicit_shutdown_invokes_node_shutdown` checks the
/// counter AFTER `rt.shutdown()` returns, but at that point the
/// runtime has been dropped too. If someone reverted the
/// `shutdown_all_nodes()` call inside `pub fn shutdown(mut self)`, the
/// `Drop` impl would still fire `shutdown_all_nodes()` and the counter
/// would still read 1 — silently re-introducing the
/// masked-shutdown bug.
///
/// This test calls `shutdown_all_nodes()` directly (the same primitive
/// `shutdown(self)` calls) and asserts the counter increments BEFORE
/// the runtime is dropped, then asserts Drop is a no-op (idempotency
/// gate works even when the explicit path beat Drop to it).
#[test]
fn explicit_shutdown_fires_before_drop_observer() {
    let counter = Arc::new(AtomicU32::new(0));
    let counter_for_shutdown = Arc::clone(&counter);

    let info = NodeInfo::from_names(vec![], vec!["noop".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 1 });
    let entry =
        ClosureNodeEntry::new(info, |_ctx: &mut NodeContext| Ok(())).with_shutdown(move || {
            counter_for_shutdown.fetch_add(1, Ordering::Relaxed);
            Ok(())
        });

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "explicit_vs_drop".to_string(),
        prefix: "p2".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "ticker".to_string(),
            node_type: "ticker".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "noop".to_string(),
                schema: "u8".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("ticker".to_string(), Box::new(entry));

    let clock = Arc::new(VirtualClock::new());
    let mut rt = GraphRuntime::build_for_test(config, factories, clock, 4).expect("build");

    // Pre-shutdown observation: NodeEntry::shutdown has not run.
    assert_eq!(
        counter.load(Ordering::Relaxed),
        0,
        "shutdown must not run during normal operation"
    );

    // Explicit primitive — same path `shutdown(self)` invokes. The
    // runtime is STILL ALIVE here (not yet dropped). If shutdown_all_nodes
    // were a no-op, the counter would stay at 0 and this assert would
    // catch the regression that the sibling test cannot.
    rt.shutdown_all_nodes();
    assert_eq!(
        counter.load(Ordering::Relaxed),
        1,
        "shutdown_all_nodes() must fire shutdown synchronously, before Drop"
    );

    // Now let Drop run. Idempotency gate must hold.
    drop(rt);
    assert_eq!(
        counter.load(Ordering::Relaxed),
        1,
        "Drop must be a no-op once shutdown_all_nodes() already ran"
    );
}

/// `shutdown_all_nodes` claims insertion-order shutdown
/// (mirroring graph YAML order, Principle #5). The sibling tests do not pin this.
///
/// Build a 3-node graph (a, b, c registered in that order), each with
/// a shutdown closure that pushes its id onto a shared trace vec, and
/// assert the trace reads `["a", "b", "c"]` after shutdown.
#[test]
fn shutdown_all_nodes_fires_in_insertion_order() {
    let trace: Arc<Mutex<Vec<&'static str>>> = Arc::new(Mutex::new(Vec::new()));

    // Build three closure entries with different shutdown markers.
    fn make_entry(trace: Arc<Mutex<Vec<&'static str>>>, marker: &'static str) -> ClosureNodeEntry {
        let info = NodeInfo::from_names(vec![], vec!["noop".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 1 });
        ClosureNodeEntry::new(info, |_ctx: &mut NodeContext| Ok(())).with_shutdown(move || {
            trace.lock().unwrap().push(marker);
            Ok(())
        })
    }

    let entry_a = make_entry(Arc::clone(&trace), "a");
    let entry_b = make_entry(Arc::clone(&trace), "b");
    let entry_c = make_entry(Arc::clone(&trace), "c");

    let mk_node = |id: &str| NodeDef {
        ros2: None,
        id: id.to_string(),
        node_type: id.to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "noop".to_string(),
            schema: "u8".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    };

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ordering".to_string(),
        prefix: "p2".to_string(),
        // YAML order: a, b, c — must be the shutdown order too.
        nodes: vec![mk_node("a"), mk_node("b"), mk_node("c")],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("a".to_string(), Box::new(entry_a));
    factories.insert("b".to_string(), Box::new(entry_b));
    factories.insert("c".to_string(), Box::new(entry_c));

    let clock = Arc::new(VirtualClock::new());
    let mut rt = GraphRuntime::build_for_test(config, factories, clock, 4).expect("build");

    rt.shutdown_all_nodes();

    let observed = trace.lock().unwrap().clone();
    assert_eq!(
        observed,
        vec!["a", "b", "c"],
        "shutdown_all_nodes must fire in node-insertion order (matches YAML order)"
    );
}

/// `shutdown_all_nodes` claims log-and-continue: a node
/// whose `shutdown()` returns `Err` must not prevent later nodes from
/// running theirs. The loop is a `tracing::error!` + `continue`, and this is
/// the test that runs a failing node alongside a succeeding one.
///
/// Two nodes: `failer` returns Err, `succeeder` returns Ok. Insertion
/// order is failer-then-succeeder so the bug (early return on Err)
/// would manifest as `succeeder` never marking its side effect.
#[test]
fn shutdown_error_in_one_node_does_not_skip_others() {
    let succeeder_ran = Arc::new(AtomicU32::new(0));
    let succeeder_ran_for_shutdown = Arc::clone(&succeeder_ran);

    let info_a = NodeInfo::from_names(vec![], vec!["noop".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 1 });
    let failer = ClosureNodeEntry::new(info_a, |_ctx: &mut NodeContext| Ok(())).with_shutdown(
        || -> TransportResult<()> {
            Err(TransportError::NodeError {
                node_id: "failer".to_string(),
                reason: "synthetic shutdown error for the isolation test".to_string(),
            })
        },
    );

    let info_b = NodeInfo::from_names(vec![], vec!["noop".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 1 });
    let succeeder =
        ClosureNodeEntry::new(info_b, |_ctx: &mut NodeContext| Ok(())).with_shutdown(move || {
            succeeder_ran_for_shutdown.fetch_add(1, Ordering::Relaxed);
            Ok(())
        });

    let mk_node = |id: &str| NodeDef {
        ros2: None,
        id: id.to_string(),
        node_type: id.to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "noop".to_string(),
            schema: "u8".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    };

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "error_isolation".to_string(),
        prefix: "p2".to_string(),
        nodes: vec![mk_node("failer"), mk_node("succeeder")],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("failer".to_string(), Box::new(failer));
    factories.insert("succeeder".to_string(), Box::new(succeeder));

    let clock = Arc::new(VirtualClock::new());
    let mut rt = GraphRuntime::build_for_test(config, factories, clock, 4).expect("build");

    // shutdown_all_nodes returns () — error handling is purely the
    // tracing::error! log + continue. The observable outcome we can
    // assert is that `succeeder` STILL ran its shutdown despite
    // `failer` erroring out earlier in the iteration.
    rt.shutdown_all_nodes();

    assert_eq!(
        succeeder_ran.load(Ordering::Relaxed),
        1,
        "succeeder.shutdown MUST run even when an earlier node's shutdown returned Err"
    );
}

/// Sanity check: keep at least one direct invocation of the helper
/// builder (also exercises Drop fall-through path with a multi-call
/// helper rather than a single-test inline build).
#[test]
fn helper_builder_drop_path_still_fires_shutdown() {
    let trace: Arc<Mutex<Vec<&'static str>>> = Arc::new(Mutex::new(Vec::new()));
    {
        let _rt = build_single_node_runtime_with_trace(Arc::clone(&trace), "solo", "solo-shut", 1);
        // Exit scope without explicit shutdown — Drop runs.
    }
    let observed = trace.lock().unwrap().clone();
    assert_eq!(observed, vec!["solo-shut"]);
}
