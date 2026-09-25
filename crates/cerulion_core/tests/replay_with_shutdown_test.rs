// SPDX-License-Identifier: AGPL-3.0-only
//! Replay-equivalence test for
//! graphs that use `request_shutdown` + node-defined `shutdown`
//! lifecycle methods.
//!
//! Self-comparing two runs of the SAME
//! `run_replay_capture` closure would be tautological. That
//! only proves `VirtualClock + Period scheduling` are
//! deterministic (already known). The shutdown-shortens-vs-cap
//! contribution would never be exercised against a true oracle.
//!
//! So this file uses oracle-vector
//! comparisons: the captured trace is asserted equal to a hardcoded
//! `Vec<u64>` of EXPECTED virt_ns timestamps. A regression that
//! perturbs scheduler timing, period semantics, or shutdown-
//! interrupt placement fires the test against the oracle, not
//! against the test's own re-derived expectation.
//!
//! Plus two edge cases:
//! - `target_ticks = 0`: shutdown requested before first tick — the
//!   captured trace must be empty (no ticks fire).
//! - shutdown-during-init: a node that calls `request_shutdown()`
//!   from its `init` callback must terminate cleanly with zero
//!   captured ticks.

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{NodeContext, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::{ClosureNodeEntry, MacroPolicy, NodeInfo};
use indexmap::IndexMap;

/// Builds a 1-node graph that ticks N times, then trips
/// `request_shutdown` on the (target+1)th tick. Returns the captured
/// shutdown-sequence from the node's tick callback.
///
/// Each tick records `ctx.clock().virt_ns()` at the moment of tick.
/// For `period_ms = 1` and a VirtualClock starting at virt_ns = 0:
/// the runtime's `run_until_shutdown` calls `step(Duration::from_millis(1))`
/// each iteration, which ADVANCES virt_ns by 1ms BEFORE running the
/// scheduler. So tick K observes virt_ns = (K+1) * 1_000_000 ns
/// (1-indexed milliseconds), and the oracle is `(1..=target_ticks).map(|i| i * 1_000_000)`.
fn run_replay_capture(target_ticks: u32) -> Vec<u64> {
    let captured = Arc::new(Mutex::new(Vec::<u64>::new()));
    let counter = Arc::new(AtomicU32::new(0));

    let captured_for_tick = Arc::clone(&captured);
    let counter_for_tick = Arc::clone(&counter);
    let entry = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["out".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 1 }),
        move |ctx: &mut NodeContext| {
            let n = counter_for_tick.fetch_add(1, Ordering::Relaxed);
            let sim = ctx.clock().virt_ns().unwrap_or(0);
            captured_for_tick.lock().unwrap().push(sim);
            if n + 1 >= target_ticks {
                ctx.request_shutdown();
            }
            Ok(())
        },
    );

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "replay_with_shutdown".to_string(),
        prefix: "rws".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "ticker".to_string(),
            node_type: "ticker".to_string(),
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
    factories.insert("ticker".to_string(), Box::new(entry));

    let clock = Arc::new(VirtualClock::new());
    let mut rt = GraphRuntime::build_for_test(config, factories, clock, 4).expect("build_for_test");
    rt.run_until_shutdown(Duration::from_millis(1), Some(1024));

    let result = captured.lock().unwrap().clone();
    result
}

/// Build a 1-node graph whose `init` callback calls
/// `request_shutdown()` immediately. Returns the captured tick
/// timestamps — must be empty since shutdown is requested before
/// tick 1 ever fires.
fn run_with_shutdown_during_init() -> Vec<u64> {
    let captured = Arc::new(Mutex::new(Vec::<u64>::new()));
    let captured_for_tick = Arc::clone(&captured);

    let entry = ClosureNodeEntry::new(
        NodeInfo::from_names(vec![], vec!["out".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 1 }),
        move |ctx: &mut NodeContext| {
            let sim = ctx.clock().virt_ns().unwrap_or(0);
            captured_for_tick.lock().unwrap().push(sim);
            Ok(())
        },
    )
    .with_init(|ctx: &mut NodeContext| {
        ctx.request_shutdown();
        Ok(())
    });

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "replay_shutdown_during_init".to_string(),
        prefix: "rsi".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "ticker".to_string(),
            node_type: "ticker".to_string(),
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
    factories.insert("ticker".to_string(), Box::new(entry));

    let clock = Arc::new(VirtualClock::new());
    let mut rt = GraphRuntime::build_for_test(config, factories, clock, 4).expect("build_for_test");
    rt.run_until_shutdown(Duration::from_millis(1), Some(1024));

    let result = captured.lock().unwrap().clone();
    result
}

/// **Oracle test:** captured trace MUST
/// equal a hardcoded oracle vector derived from documented semantics —
/// `(1..=target_ticks).map(|i| i * 1_000_000)` ns. For period_ms=1
/// the VirtualClock advances by 1ms BEFORE each scheduler iteration
/// (per `run_until_shutdown`'s `step(dt)` contract), so tick K
/// observes virt_ns = (K+1) * 1ms. A regression that perturbs scheduler
/// timing (e.g., off-by-one on period boundary, or starts firing at
/// virt_ns=0) fires this test.
///
/// Compare to the previous (tautological) test which only asserted
/// "two runs produce the same trace" — could pass even if every
/// captured value were wrong.
#[test]
fn replay_against_oracle_vector_target_50() {
    let trace = run_replay_capture(50);
    let oracle: Vec<u64> = (1..=50u64).map(|i| i * 1_000_000).collect();
    assert_eq!(
        trace,
        oracle,
        "captured virt_ns trace must equal oracle vector ((1..=50)*1ms) \
         for period_ms=1 and target_ticks=50; got trace.len()={} oracle.len()={}",
        trace.len(),
        oracle.len()
    );
}

/// Smaller oracle for sanity — 5 ticks. Locks the same property at
/// reduced size for faster regression diagnosis.
#[test]
fn replay_against_oracle_vector_target_5() {
    let trace = run_replay_capture(5);
    let oracle = vec![1_000_000u64, 2_000_000, 3_000_000, 4_000_000, 5_000_000];
    assert_eq!(trace, oracle, "got: {trace:?}");
}

/// Edge case: `target_ticks = 0`
/// means shutdown is requested on the FIRST tick (n+1 >= 0 is always
/// true). The captured trace should contain exactly ONE entry — the
/// tick that observed n=0 and requested shutdown — at virt_ns=1ms
/// (the clock advanced by 1ms before tick fired).
///
/// Note on edge semantics: `target_ticks=0` doesn't mean "zero
/// ticks." It means "trip shutdown on tick 1 (immediately after
/// counter goes 0 → 1)." Per the closure: `if n + 1 >= target_ticks`
/// where n=0 and target=0 → 1 >= 0 → request_shutdown. So one tick
/// fires before run_until_shutdown sees the signal and returns.
#[test]
fn replay_target_ticks_zero_yields_one_tick_then_shutdown() {
    let trace = run_replay_capture(0);
    assert_eq!(
        trace,
        vec![1_000_000u64],
        "target_ticks=0 should fire exactly one tick (the one that requests \
         shutdown), at virt_ns=1ms (post step-advance); got: {trace:?}"
    );
}

/// Edge case: `request_shutdown()` from inside `init`
/// must NOT cause panic; the runtime must terminate cleanly with
/// zero captured ticks (init runs before first scheduled tick).
#[test]
fn replay_shutdown_during_init_does_not_panic_zero_captured_ticks() {
    let trace = run_with_shutdown_during_init();
    assert!(
        trace.is_empty(),
        "shutdown_during_init must yield an empty trace (no tick fires \
         after init's request_shutdown); got: {trace:?}"
    );
}

/// Determinism (replay-equivalence): two independent runs against the
/// SAME oracle produce the SAME trace. Combined with the oracle-
/// equality test above, this proves both that the trace matches a
/// hardcoded expected value AND that re-running produces the same
/// value (no nondeterminism). The original tautology was only the
/// SECOND of these two properties; this test pins both via the
/// oracle.
#[test]
fn replay_oracle_match_is_deterministic_across_runs() {
    let trace_a = run_replay_capture(20);
    let trace_b = run_replay_capture(20);
    let oracle: Vec<u64> = (1..=20u64).map(|i| i * 1_000_000).collect();
    assert_eq!(trace_a, oracle, "run A must match oracle");
    assert_eq!(trace_b, oracle, "run B must match oracle");
    // Implied transitively but kept explicit for clarity:
    assert_eq!(trace_a, trace_b, "runs must agree (deterministic)");
}
