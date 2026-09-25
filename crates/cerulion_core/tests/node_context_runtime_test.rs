// SPDX-License-Identifier: AGPL-3.0-only
//! Smoke tests for `NodeContext`'s runtime surface:
//! `env`, `env_str`, `clock`, `request_shutdown`, plus the
//! `ShutdownSignal` type and the `GraphRuntime::run_until_shutdown` helper.
//!
//! These exercise the framework surface in isolation (no macros, no
//! transport) so failures localise to the plumbing rather than to
//! the macro layer above it.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::{RealClock, VirtualClock};
use cerulion_core::graph::node::{NodeContext, ShutdownSignal};
use indexmap::IndexMap;

/// `NodeContext::for_tests(IndexMap::new(), IndexMap::new())` never falls back
/// to live `std::env::var`. This helper builds a context with a single
/// env entry pre-stuffed so env-parsing tests below still exercise the
/// same code paths.
fn ctx_with_env(key: &str, value: &str) -> NodeContext {
    use indexmap::IndexMap;
    let mut env = std::collections::HashMap::new();
    env.insert(key.to_string(), value.to_string());
    NodeContext::with_runtime_env(
        IndexMap::new(),
        IndexMap::new(),
        Arc::new(RealClock),
        ShutdownSignal::new(),
        Arc::new(env),
    )
}

#[test]
fn env_falls_back_to_default_when_unset() {
    // The env var name is intentionally absurd to avoid colliding with
    // anything a developer might have exported in their shell.
    std::env::remove_var("TEST_NEVER_SET");
    let ctx = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    let v: usize = ctx.env("TEST_NEVER_SET", 42);
    assert_eq!(v, 42);
}

#[test]
fn env_parses_set_value() {
    let ctx = ctx_with_env("TEST_USIZE_PARSE", "9000");
    let v: usize = ctx.env("TEST_USIZE_PARSE", 1);
    assert_eq!(v, 9000);
}

#[test]
fn env_falls_back_when_unparseable() {
    let ctx = ctx_with_env("TEST_BAD_USIZE", "not-a-number");
    let v: usize = ctx.env("TEST_BAD_USIZE", 7);
    assert_eq!(v, 7, "parse failure should fall back to default");
}

#[test]
fn env_str_default() {
    let ctx = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    let s = ctx.env_str("TEST_STR_DEFAULT", "/tmp/fallback.csv");
    assert_eq!(s, "/tmp/fallback.csv");
}

#[test]
fn env_str_uses_set_value() {
    let ctx = ctx_with_env("TEST_STR_SET", "/var/log/cerulion.csv");
    let s = ctx.env_str("TEST_STR_SET", "/tmp/wrong.csv");
    assert_eq!(s, "/var/log/cerulion.csv");
}

#[test]
fn clock_default_is_real_clock_and_returns_nonzero() {
    let ctx = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    let now = ctx.clock().now_ns();
    assert!(
        now > 0,
        "RealClock should return non-zero CLOCK_MONOTONIC ns"
    );
}

#[test]
fn clock_with_runtime_uses_provided_clock() {
    use indexmap::IndexMap;

    let sim = Arc::new(VirtualClock::new());
    sim.advance(123_456_789);
    let sig = ShutdownSignal::new();
    let ctx = NodeContext::with_runtime_env(
        IndexMap::new(),
        IndexMap::new(),
        sim.clone(),
        sig,
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );
    assert_eq!(ctx.clock().now_ns(), 123_456_789);
    sim.advance(1);
    // Reading through the context observes subsequent advances because
    // both share the same Arc.
    assert_eq!(ctx.clock().now_ns(), 123_456_790);
}

#[test]
fn shutdown_signal_default_is_unrequested() {
    let s = ShutdownSignal::new();
    assert!(!s.is_requested());
}

#[test]
fn shutdown_signal_request_is_observable_and_idempotent() {
    let s = ShutdownSignal::new();
    s.request();
    assert!(s.is_requested());
    // Idempotent: requesting twice still reads true once.
    s.request();
    assert!(s.is_requested());
}

#[test]
fn shutdown_signal_clones_share_state() {
    let a = ShutdownSignal::new();
    let b = a.clone();
    assert!(!b.is_requested());
    a.request();
    assert!(
        b.is_requested(),
        "clones must share Arc<AtomicBool>; flipping in one is visible in the other"
    );
}

#[test]
fn node_context_request_shutdown_propagates_to_signal() {
    use indexmap::IndexMap;

    let clock: Arc<dyn cerulion_core::clock::Clock> = Arc::new(RealClock);
    let sig = ShutdownSignal::new();
    let ctx = NodeContext::with_runtime_env(
        IndexMap::new(),
        IndexMap::new(),
        clock,
        sig.clone(),
        Arc::new(std::collections::HashMap::new()),
    );
    assert!(!sig.is_requested());
    ctx.request_shutdown();
    assert!(
        sig.is_requested(),
        "request_shutdown must flip the shared signal"
    );
}

#[test]
fn graph_runtime_run_until_shutdown_exits_when_signal_fires() {
    use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
    use cerulion_core::graph::node::NodeEntry;
    use cerulion_core::graph::GraphRuntime;
    use cerulion_core::{ClosureNodeEntry, MacroPolicy, NodeInfo};
    use indexmap::IndexMap;

    // Single-node graph with a Period trigger. The tick callback flips
    // the runtime's shutdown signal on the third call. We expect
    // run_until_shutdown to return after exactly 3 steps (one extra
    // because the signal is checked at the *top* of the loop, after the
    // step that set it).

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "smoke".to_string(),
        prefix: "smoke".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
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

    // ClosureNodeEntry tick takes &mut NodeContext but doesn't surface the
    // shutdown signal directly — capture it via a clone wired into the
    // closure and trip it after 3 ticks. The runtime itself owns its own
    // signal but for this smoke test we manually mirror the trip via the
    // runtime's accessor (see below).
    let info = NodeInfo::from_names(vec![], vec!["noop".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 1 });
    let counter = Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let counter_in = counter.clone();
    let entry = ClosureNodeEntry::new(info, move |_ctx: &mut NodeContext| {
        counter_in.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        // Cooperative shutdown via the context that was passed into init —
        // reaches the runtime's own signal (via with_runtime).
        if counter_in.load(std::sync::atomic::Ordering::Relaxed) >= 3 {
            _ctx.request_shutdown();
        }
        Ok(())
    });
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("ticker".to_string(), Box::new(entry));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock.clone(), 4).expect("build");

    assert!(!runtime.shutdown_requested());
    let steps = runtime.run_until_shutdown(Duration::from_millis(1), Some(100));
    assert!(
        runtime.shutdown_requested(),
        "signal should be set after node trip"
    );
    assert_eq!(
        counter.load(std::sync::atomic::Ordering::Relaxed),
        3,
        "node should have ticked exactly 3 times before shutdown"
    );
    assert_eq!(steps, 3, "loop should exit immediately after the trip");
}
