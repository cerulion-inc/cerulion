// SPDX-License-Identifier: AGPL-3.0-only
//! Regression test: `GraphRuntime` actually
//! invokes `NodeEntry::shutdown()` on every node when the runtime
//! shuts down (explicit `shutdown(self)`, `run_until_shutdown`, or
//! `Drop` fall-through).
//!
//! Were entries move-captured into the scheduler's tick
//! callback, `GraphRuntime::shutdown(self)` would only `drop` the runtime
//! — the user's `shutdown()` lifecycle method would NEVER run, and the bench
//! `latency_node`'s CSV write would silently never happen.
//!
//! Instead, entries are shared via `Arc<Mutex<Box<dyn NodeEntry>>>`
//! between the scheduler and the runtime. `shutdown_all_nodes` (called
//! by `shutdown(self)`, `run_until_shutdown`, and `Drop`) iterates and
//! calls `shutdown()` on each, logging errors but continuing.

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{NodeContext, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::{ClosureNodeEntry, MacroPolicy, NodeInfo};
use indexmap::IndexMap;

/// Counts each lifecycle event so the tests can assert
/// init-then-tick-then-shutdown semantics.
#[derive(Default)]
struct LifecycleCounters {
    inits: AtomicU32,
    ticks: AtomicU32,
    shutdowns: AtomicU32,
}

fn build_runtime_with_counters(counters: Arc<LifecycleCounters>, period_ms: u64) -> GraphRuntime {
    let info = NodeInfo::from_names(vec![], vec!["noop".to_string()])
        .with_policy(MacroPolicy::Period { period_ms });
    let counters_for_init = Arc::clone(&counters);
    let counters_for_tick = Arc::clone(&counters);
    let counters_for_shutdown = Arc::clone(&counters);

    let entry = ClosureNodeEntry::new(info, move |_ctx: &mut NodeContext| {
        counters_for_tick.ticks.fetch_add(1, Ordering::Relaxed);
        Ok(())
    })
    .with_init(move |_ctx: &mut NodeContext| {
        counters_for_init.inits.fetch_add(1, Ordering::Relaxed);
        Ok(())
    })
    .with_shutdown(move || {
        counters_for_shutdown
            .shutdowns
            .fetch_add(1, Ordering::Relaxed);
        Ok(())
    });

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "shutdown_lifecycle_test".to_string(),
        prefix: "sl_test".to_string(),
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
    GraphRuntime::build_for_test(config, factories, clock, 4).expect("build_for_test")
}

#[test]
fn explicit_shutdown_invokes_node_shutdown() {
    let counters = Arc::new(LifecycleCounters::default());
    let mut rt = build_runtime_with_counters(Arc::clone(&counters), 1);
    assert_eq!(
        counters.inits.load(Ordering::Relaxed),
        1,
        "init runs once at build time"
    );

    rt.step(Duration::from_millis(1));
    rt.step(Duration::from_millis(1));
    assert_eq!(counters.ticks.load(Ordering::Relaxed), 2);
    assert_eq!(counters.shutdowns.load(Ordering::Relaxed), 0);

    rt.shutdown();
    // Without the shared entries this would still be 0; `shutdown_all_nodes` forces it to 1.
    assert_eq!(
        counters.shutdowns.load(Ordering::Relaxed),
        1,
        "explicit GraphRuntime::shutdown(self) MUST invoke NodeEntry::shutdown"
    );
}

#[test]
fn run_until_shutdown_invokes_node_shutdown() {
    let counters = Arc::new(LifecycleCounters::default());
    let mut rt = build_runtime_with_counters(Arc::clone(&counters), 1);

    let signal = rt.shutdown_signal().clone();
    // Trip the signal externally so run_until_shutdown exits.
    signal.request();
    let steps = rt.run_until_shutdown(Duration::from_millis(1), Some(8));
    assert_eq!(steps, 0, "pre-tripped signal should skip all steps");
    assert_eq!(
        counters.shutdowns.load(Ordering::Relaxed),
        1,
        "run_until_shutdown MUST invoke NodeEntry::shutdown on exit"
    );

    // Subsequent explicit shutdown() must NOT double-fire (idempotent
    // via the nodes_shut_down flag).
    rt.shutdown();
    assert_eq!(
        counters.shutdowns.load(Ordering::Relaxed),
        1,
        "shutdown_all_nodes is idempotent"
    );
}

#[test]
fn drop_fallthrough_invokes_node_shutdown() {
    let counters = Arc::new(LifecycleCounters::default());
    {
        let _rt = build_runtime_with_counters(Arc::clone(&counters), 1);
        // Drop happens at end of scope without explicit shutdown call.
    }
    assert_eq!(
        counters.shutdowns.load(Ordering::Relaxed),
        1,
        "Drop fall-through MUST invoke NodeEntry::shutdown as a safety net"
    );
}

#[test]
fn init_failure_midway_shuts_down_already_initd_nodes() {
    // Regression test.
    //
    // `build_for_test` iterates nodes, calling
    // `entry.init(context)?`. If node N's init returns Err, nodes
    // 0..N-1 are already init'd but not yet wrapped in Arc / pushed
    // into `self.nodes` — without a guard `?` returns Err and they get dropped
    // silently without `shutdown()` ever running.
    //
    // So a `PartialNodes` RAII guard owns the partial map
    // during the build loop. On `?`-return, its Drop iterates and
    // calls `shutdown()` on every entry registered so far.

    let counters_a = Arc::new(LifecycleCounters::default());
    let counters_b = Arc::new(LifecycleCounters::default());

    let info_a = NodeInfo::from_names(vec![], vec!["a_out".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 1 });
    let info_b = NodeInfo::from_names(vec![], vec!["b_out".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 1 });

    // Node A: init succeeds, shutdown bumps counter.
    let a_init = Arc::clone(&counters_a);
    let a_shutdown = Arc::clone(&counters_a);
    let a = ClosureNodeEntry::new(info_a, |_ctx: &mut NodeContext| Ok(()))
        .with_init(move |_ctx| {
            a_init.inits.fetch_add(1, Ordering::Relaxed);
            Ok(())
        })
        .with_shutdown(move || {
            a_shutdown.shutdowns.fetch_add(1, Ordering::Relaxed);
            Ok(())
        });

    // Node B: init RETURNS ERR.
    let b_init = Arc::clone(&counters_b);
    let b_shutdown = Arc::clone(&counters_b);
    let b = ClosureNodeEntry::new(info_b, |_ctx: &mut NodeContext| Ok(()))
        .with_init(move |_ctx| {
            b_init.inits.fetch_add(1, Ordering::Relaxed);
            Err(cerulion_core::error::TransportError::NodeError {
                node_id: "node_b".to_string(),
                reason: "simulated init failure".to_string(),
            })
        })
        .with_shutdown(move || {
            b_shutdown.shutdowns.fetch_add(1, Ordering::Relaxed);
            Ok(())
        });

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "init_failure_test".to_string(),
        prefix: "ift".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "node_a".to_string(),
                node_type: "a".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "a_out".to_string(),
                    schema: "u8".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                ros2: None,
                id: "node_b".to_string(),
                node_type: "b".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "b_out".to_string(),
                    schema: "u8".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("node_a".to_string(), Box::new(a));
    factories.insert("node_b".to_string(), Box::new(b));

    let clock = Arc::new(VirtualClock::new());
    let result = GraphRuntime::build_for_test(config, factories, clock, 4);

    // Build returns Err because node_b's init failed.
    assert!(result.is_err(), "build must fail when node_b's init errors");

    // Node A: init ran (count = 1), shutdown ran via PartialNodes RAII
    // cleanup (count = 1). Without the guard this is 0 — the load-bearing
    // assertion.
    assert_eq!(
        counters_a.inits.load(Ordering::Relaxed),
        1,
        "node A init must have run"
    );
    assert_eq!(
        counters_a.shutdowns.load(Ordering::Relaxed),
        1,
        "node A shutdown MUST run via PartialNodes RAII when subsequent init fails (without the guard this is 0)"
    );

    // Node B: init ran (count = 1) and FAILED. Its shutdown
    // intentionally did NOT run because the failed init means the
    // node is in an indeterminate state — calling shutdown on it
    // could double-cleanup. (PartialNodes registers AFTER successful
    // init.)
    assert_eq!(counters_b.inits.load(Ordering::Relaxed), 1);
    assert_eq!(
        counters_b.shutdowns.load(Ordering::Relaxed),
        0,
        "node B shutdown must NOT run — its init failed so its state is indeterminate"
    );
}

#[test]
fn shutdown_all_nodes_is_idempotent_explicit() {
    let counters = Arc::new(LifecycleCounters::default());
    let mut rt = build_runtime_with_counters(Arc::clone(&counters), 1);

    rt.shutdown_all_nodes();
    assert_eq!(counters.shutdowns.load(Ordering::Relaxed), 1);
    rt.shutdown_all_nodes();
    rt.shutdown_all_nodes();
    assert_eq!(
        counters.shutdowns.load(Ordering::Relaxed),
        1,
        "repeat calls to shutdown_all_nodes MUST NOT re-invoke NodeEntry::shutdown"
    );
    // And Drop, when it fires, also won't re-fire.
    drop(rt);
    assert_eq!(counters.shutdowns.load(Ordering::Relaxed), 1);
}
