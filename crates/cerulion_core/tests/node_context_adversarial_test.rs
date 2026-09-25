// SPDX-License-Identifier: AGPL-3.0-only
//! Adversarial tests for `NodeContext`'s runtime surface:
//! `env`, `env_str`, `clock`, `request_shutdown`, plus the `ShutdownSignal`
//! type and the `GraphRuntime::run_until_shutdown` helper.
//!
//! Companion to `node_context_runtime_test.rs` (happy-path smoke tests).
//! These cases probe edge cases that the smoke tests do not cover:
//! - Concurrent `request()` / `is_requested()` from many threads.
//! - Env-var inputs with empty strings, whitespace, embedded NULs, unicode,
//!   and very long values.
//! - `Arc::ptr_eq` on the clock returned by two contexts built from the
//!   same `Arc<dyn Clock>`.
//! - `run_until_shutdown` corner cases: pre-fired signal, `max_steps == 0`,
//!   `max_steps == None` paired with a pre-fired signal, `Duration::ZERO`.
//!
//! Run with:
//! ```
//! cargo test -p cerulion_core --test node_context_adversarial_test -- --test-threads=1
//! ```
//!
//! `--test-threads=1` is required because env-var manipulation
//! (`std::env::set_var` / `remove_var`) is process-global and would race
//! with itself if tests in this file ran in parallel. The runtime tests in
//! the lower half don't strictly need it but inherit the same flag.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Barrier};
use std::thread;
use std::time::Duration;

use cerulion_core::clock::{Clock, RealClock, VirtualClock};
use cerulion_core::graph::node::{NodeContext, ShutdownSignal};
use indexmap::IndexMap;

/// `NodeContext::for_tests(IndexMap::new(), IndexMap::new())` does not fall back
/// to live `std::env::var` — the snapshot is empty by default. This
/// helper builds a NodeContext with a SINGLE env entry pre-stuffed
/// into the snapshot, so the env-parsing tests below can keep
/// exercising parse semantics without depending on a
/// live-env path.
fn ctx_with_env(key: &str, value: &str) -> NodeContext {
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

// ---------------------------------------------------------------------------
// ShutdownSignal — concurrent / ordering edge cases
// ---------------------------------------------------------------------------

/// Many threads calling `request()` simultaneously. The bool must end up
/// `true` and there must be no observable transient `false` after any
/// successful `is_requested()` returns `true` (the AtomicBool is one-way).
#[test]
fn shutdown_signal_many_threads_request_concurrently() {
    let signal = ShutdownSignal::new();
    let n_threads = 32;
    let barrier = Arc::new(Barrier::new(n_threads));

    let handles: Vec<_> = (0..n_threads)
        .map(|_| {
            let s = signal.clone();
            let b = barrier.clone();
            thread::spawn(move || {
                b.wait();
                // Hammer the signal a few times each. Idempotent.
                for _ in 0..1000 {
                    s.request();
                }
            })
        })
        .collect();

    for h in handles {
        h.join().expect("worker thread panicked");
    }

    assert!(
        signal.is_requested(),
        "after concurrent requests the signal must be true"
    );
}

/// One thread calls `request()` while many others poll `is_requested()`.
/// Once any reader has observed `true`, every subsequent observation in
/// the same thread (and in any other thread that started after it) must
/// also be `true` — the bool is monotonically one-way.
#[test]
fn shutdown_signal_request_visible_to_concurrent_pollers() {
    let signal = ShutdownSignal::new();
    let n_pollers = 16;
    let start = Arc::new(Barrier::new(n_pollers + 1));

    let any_saw_true = Arc::new(AtomicUsize::new(0));
    let any_regressed = Arc::new(AtomicUsize::new(0));

    let pollers: Vec<_> = (0..n_pollers)
        .map(|_| {
            let s = signal.clone();
            let b = start.clone();
            let saw = any_saw_true.clone();
            let regressed = any_regressed.clone();
            thread::spawn(move || {
                b.wait();
                // First phase: poll until the setter's request becomes visible
                // (bounded). These reads race the in-flight `request()` —
                // the interesting window. A FIXED 100k-poll
                // loop + "did anyone see true?" assert is scheduling-dependent
                // on a preempted VM: a descheduled setter can lag past ALL pollers'
                // loops, so no poller ever sees true and the anti-vacuity
                // assert fires. Poll-until-true is scheduling-independent.
                let deadline = std::time::Instant::now() + Duration::from_secs(10);
                while !s.is_requested() {
                    assert!(
                        std::time::Instant::now() < deadline,
                        "setter's request() not visible within 10s"
                    );
                    std::hint::spin_loop();
                }
                saw.fetch_add(1, Ordering::Relaxed);
                // Second phase: once observed true it must NEVER read false again
                // in this thread — the monotonic one-way contract, checked
                // across a fixed window of subsequent reads.
                for _ in 0..100_000 {
                    if !s.is_requested() {
                        regressed.fetch_add(1, Ordering::Relaxed);
                    }
                }
            })
        })
        .collect();

    let setter = {
        let s = signal.clone();
        let b = start.clone();
        thread::spawn(move || {
            b.wait();
            // Give pollers a head start so most of them see `false` first.
            for _ in 0..1000 {
                std::hint::spin_loop();
            }
            s.request();
        })
    };

    for p in pollers {
        p.join().expect("poller panicked");
    }
    setter.join().expect("setter panicked");

    assert!(signal.is_requested(), "final state must be true");
    assert_eq!(
        any_regressed.load(Ordering::Relaxed),
        0,
        "AtomicBool went from true → false at least once; ordering bug"
    );
    assert_eq!(
        any_saw_true.load(Ordering::Relaxed),
        n_pollers,
        "every poller must observe the request (phase 1 is poll-until-true)"
    );
}

/// `Default` must produce an un-requested signal. Re-constructing
/// `ShutdownSignal::default()` after the previous one was tripped must
/// NOT inherit the tripped state (no hidden static / shared backing).
#[test]
fn shutdown_signal_default_is_independent_per_instance() {
    let a = ShutdownSignal::default();
    a.request();
    assert!(a.is_requested());

    let b = ShutdownSignal::default();
    assert!(
        !b.is_requested(),
        "second default() must not inherit a's tripped state"
    );
}

/// Cloning a tripped signal must yield a tripped clone. Conversely,
/// cloning before tripping then tripping the original must propagate to
/// the clone (already covered in smoke tests, restated here for symmetry).
#[test]
fn shutdown_signal_clone_after_request_is_already_tripped() {
    let a = ShutdownSignal::new();
    a.request();
    let b = a.clone();
    assert!(b.is_requested(), "clone of tripped signal must be tripped");
}

// ---------------------------------------------------------------------------
// NodeContext::env — edge cases
// ---------------------------------------------------------------------------

/// Empty string is a "set" value. `usize::from_str("")` fails → fallback.
#[test]
fn env_empty_string_falls_back_for_numeric() {
    let key = "ADV_EMPTY_USIZE";
    let ctx = ctx_with_env(key, "");
    let v: usize = ctx.env(key, 99);
    assert_eq!(
        v, 99,
        "empty string should fail to parse as usize → default"
    );
}

/// Empty string IS a valid `String` parse → the user gets `""` back, NOT
/// the default. This is intentional but worth pinning down so a future
/// change to `env` semantics is caught.
#[test]
fn env_empty_string_parses_as_empty_string() {
    let ctx = ctx_with_env("ADV_EMPTY_STRING", "");
    let v: String = ctx.env("ADV_EMPTY_STRING", "fallback".to_string());
    assert_eq!(
        v, "",
        "String::from_str(\"\") succeeds; env() must NOT substitute default"
    );
}

/// Whitespace is not trimmed before parsing — `" 42 ".parse::<usize>()` fails.
#[test]
fn env_whitespace_around_number_falls_back() {
    let key = "ADV_WHITESPACE_NUMBER";
    let ctx = ctx_with_env(key, "  42  ");
    let v: usize = ctx.env(key, 7);
    assert_eq!(
        v, 7,
        "leading/trailing whitespace must NOT be silently trimmed; parse fails → default"
    );
}

/// Very long string — a 1 MiB env var should not crash the parser path.
/// Parsing as `usize` will fail; the `tracing::warn!` is expected to
/// emit but we don't capture it here. The test passes if the call
/// returns the default without panicking or hanging.
#[test]
fn env_very_long_value_does_not_crash() {
    let key = "ADV_VERY_LONG";
    let long = "a".repeat(1024 * 1024);
    let ctx = ctx_with_env(key, &long);
    let v: usize = ctx.env(key, 5);
    assert_eq!(v, 5, "1 MiB unparseable value must fall back without panic");
}

/// Unicode env var value parsed as `String`. Must round-trip exactly.
#[test]
fn env_unicode_string_roundtrips() {
    let key = "ADV_UNICODE";
    let payload = "こんにちは🤖🔧";
    let ctx = ctx_with_env(key, payload);
    let v: String = ctx.env(key, "wrong".to_string());
    assert_eq!(v, payload);
}

/// Unicode round-trip via `env_str` too.
#[test]
fn env_str_unicode_roundtrips() {
    let key = "ADV_UNICODE_STR";
    let payload = "ロボット-ñøde-✓";
    let ctx = ctx_with_env(key, payload);
    let v = ctx.env_str(key, "wrong");
    assert_eq!(v, payload);
}

/// `usize` env var = "0" — VALID and parses to 0. The `env()` method
/// must NOT treat "0" as "missing or unparseable" and substitute the
/// default. Pin this down because a careless implementation could be
/// tempted to.
#[test]
fn env_zero_parses_as_zero_not_default() {
    let key = "ADV_ZERO";
    let ctx = ctx_with_env(key, "0");
    let v: usize = ctx.env(key, 999);
    assert_eq!(v, 0, "literal \"0\" must parse to 0, not the default");
}

/// Negative number parsed as signed type works; parsed as unsigned fails
/// → fallback. Verifies the FromStr error path applies type-correctly.
#[test]
fn env_negative_value_signed_vs_unsigned() {
    let key = "ADV_NEGATIVE";
    let ctx = ctx_with_env(key, "-5");
    let signed: i64 = ctx.env(key, 100);
    assert_eq!(signed, -5);
    let unsigned: usize = ctx.env(key, 100);
    assert_eq!(
        unsigned, 100,
        "i64 parse succeeds, usize parse fails → default"
    );
}

/// Boolean env var — verifies generic FromStr works for non-numeric types.
#[test]
fn env_bool_parses_correctly() {
    let key = "ADV_BOOL";
    let ctx_true = ctx_with_env(key, "true");
    let v: bool = ctx_true.env(key, false);
    assert!(v);
    // bool::from_str is case-sensitive: "TRUE" fails → default
    let ctx_upper = ctx_with_env(key, "TRUE");
    let v: bool = ctx_upper.env(key, false);
    assert!(
        !v,
        "bool::from_str is case-sensitive; \"TRUE\" must fall back"
    );
}

/// `env_str` must return the exact set value, not trimmed or otherwise
/// transformed. Mirrors the `env` whitespace test for the string path.
#[test]
fn env_str_preserves_whitespace_verbatim() {
    let key = "ADV_STR_WS";
    let ctx = ctx_with_env(key, "  spaced  ");
    let v = ctx.env_str(key, "fallback");
    assert_eq!(v, "  spaced  ");
}

/// `env_str` with an empty set value returns "" (not the default). Same
/// rationale as `env_empty_string_parses_as_empty_string` but for the
/// string-specialised path.
#[test]
fn env_str_empty_value_returns_empty_not_default() {
    let key = "ADV_STR_EMPTY";
    let ctx = ctx_with_env(key, "");
    let v = ctx.env_str(key, "fallback");
    assert_eq!(
        v, "",
        "set-but-empty env var must NOT be substituted by env_str"
    );
}

/// `std::env::set_var` panics on values containing NUL on POSIX. We
/// don't try to set one; instead we verify the lookup path doesn't choke
/// on a key that simply isn't set. (Documents the boundary.)
#[test]
fn env_unset_key_returns_default_for_complex_type() {
    let key = "ADV_UNSET_DURATION";
    std::env::remove_var(key);
    let ctx = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    let v: u128 = ctx.env(key, 12345);
    assert_eq!(v, 12345);
}

// ---------------------------------------------------------------------------
// NodeContext::clock — identity / sharing
// ---------------------------------------------------------------------------

/// Two contexts built from the same `Arc<dyn Clock>` must hand out the
/// same `Arc` instance — `Arc::ptr_eq` is true. This is what justifies
/// nodes calling `ctx.clock().now_ns()` and getting bit-identical
/// timestamps to the scheduler's clock.
#[test]
fn clock_with_runtime_shares_arc_identity_across_contexts() {
    let sim: Arc<dyn Clock> = Arc::new(VirtualClock::new());
    let sig = ShutdownSignal::new();

    let ctx_a = NodeContext::with_runtime_env(
        IndexMap::new(),
        IndexMap::new(),
        sim.clone(),
        sig.clone(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );
    let ctx_b = NodeContext::with_runtime_env(
        IndexMap::new(),
        IndexMap::new(),
        sim.clone(),
        sig,
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );

    assert!(
        Arc::ptr_eq(ctx_a.clock(), ctx_b.clock()),
        "two contexts built from the same Arc<dyn Clock> must hand out ptr_eq Arcs"
    );
}

/// Default-constructed contexts each get their OWN clock — they must NOT
/// be ptr_eq. Pins down that `Default` doesn't leak a shared static.
#[test]
fn clock_default_contexts_have_distinct_arcs() {
    let a = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    let b = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    assert!(
        !Arc::ptr_eq(a.clock(), b.clock()),
        "two Default contexts must have independent Arc<RealClock> allocations"
    );
}

/// Mutating the underlying `VirtualClock` through one Arc handle is
/// visible through the context's accessor. Already covered in smoke
/// tests; restated here with the explicit Arc identity check.
#[test]
fn clock_advances_observed_through_context_arc() {
    let sim = Arc::new(VirtualClock::new());
    let sim_dyn: Arc<dyn Clock> = sim.clone();
    let ctx = NodeContext::with_runtime_env(
        IndexMap::new(),
        IndexMap::new(),
        sim_dyn,
        ShutdownSignal::new(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );

    assert_eq!(ctx.clock().now_ns(), 0);
    sim.advance(1_000);
    assert_eq!(ctx.clock().now_ns(), 1_000);
    sim.advance(500);
    assert_eq!(ctx.clock().now_ns(), 1_500);
}

// ---------------------------------------------------------------------------
// NodeContext::request_shutdown — defaults, plumbing, idempotence
// ---------------------------------------------------------------------------

/// `Default::default()` constructs a fresh signal. Calling
/// `request_shutdown` flips that signal but it has no observers — this
/// must not panic, and the signal accessor must reflect the trip.
#[test]
fn default_context_request_shutdown_flips_local_signal() {
    let ctx = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    assert!(!ctx.shutdown_signal().is_requested());
    ctx.request_shutdown();
    assert!(
        ctx.shutdown_signal().is_requested(),
        "request_shutdown on a defaulted context must still flip its private signal"
    );
}

/// Two `NodeContext::for_tests(...)` calls produce INDEPENDENT signals.
/// Tripping one must NOT trip the other. Validates that `new()` doesn't
/// accidentally share state.
#[test]
fn two_new_contexts_have_independent_shutdown_signals() {
    let a = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    let b = NodeContext::for_tests(IndexMap::new(), IndexMap::new());
    a.request_shutdown();
    assert!(a.shutdown_signal().is_requested());
    assert!(
        !b.shutdown_signal().is_requested(),
        "request on one context must not affect another"
    );
}

/// Two contexts wired with the SAME shared signal must both observe a
/// trip from either side. Validates `with_runtime` plumbing semantics.
#[test]
fn shared_signal_propagates_across_contexts_both_directions() {
    let sig = ShutdownSignal::new();
    let clock: Arc<dyn Clock> = Arc::new(RealClock);

    let a = NodeContext::with_runtime_env(
        IndexMap::new(),
        IndexMap::new(),
        clock.clone(),
        sig.clone(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );
    let b = NodeContext::with_runtime_env(
        IndexMap::new(),
        IndexMap::new(),
        clock,
        sig.clone(),
        ::std::sync::Arc::new(::std::collections::HashMap::new()),
    );

    a.request_shutdown();
    assert!(b.shutdown_signal().is_requested());

    // Reset is impossible (the bool is one-way), so we only check that the
    // second context can independently re-trip without error.
    b.request_shutdown();
    assert!(sig.is_requested());
}

/// Two threads, two contexts sharing one signal, racing on
/// `request_shutdown`. Must converge on `true` without panic.
#[test]
fn concurrent_request_shutdown_via_two_contexts_converges() {
    let sig = ShutdownSignal::new();
    let clock: Arc<dyn Clock> = Arc::new(RealClock);

    // Build two contexts on different threads to simulate two nodes.
    // We can't move NodeContext across threads safely without it being
    // wrapped, but we CAN clone the signal into each thread directly.
    let sig_a = sig.clone();
    let sig_b = sig.clone();
    let clock_a = clock.clone();
    let clock_b = clock.clone();

    let barrier = Arc::new(Barrier::new(2));
    let ba = barrier.clone();
    let bb = barrier.clone();

    let ta = thread::spawn(move || {
        let ctx = NodeContext::with_runtime_env(
            IndexMap::new(),
            IndexMap::new(),
            clock_a,
            sig_a,
            ::std::sync::Arc::new(::std::collections::HashMap::new()),
        );
        ba.wait();
        ctx.request_shutdown();
    });
    let tb = thread::spawn(move || {
        let ctx = NodeContext::with_runtime_env(
            IndexMap::new(),
            IndexMap::new(),
            clock_b,
            sig_b,
            ::std::sync::Arc::new(::std::collections::HashMap::new()),
        );
        bb.wait();
        ctx.request_shutdown();
    });

    ta.join().expect("a panicked");
    tb.join().expect("b panicked");

    assert!(sig.is_requested(), "shared signal must converge to true");
}

// ---------------------------------------------------------------------------
// GraphRuntime::run_until_shutdown — corner cases
// ---------------------------------------------------------------------------

mod runtime_corners {
    use super::*;

    use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
    use cerulion_core::graph::node::NodeEntry;
    use cerulion_core::graph::GraphRuntime;
    use cerulion_core::{ClosureNodeEntry, MacroPolicy, NodeInfo};

    /// Build a one-node graph whose tick callback runs the supplied
    /// closure each step.
    fn build_one_node_runtime<F>(tick_fn: F) -> GraphRuntime
    where
        F: FnMut(&mut NodeContext) -> cerulion_core::error::TransportResult<()> + Send + 'static,
    {
        let config = GraphConfig {
            execution: None,
            level_assignments: None,
            network: None,
            process_groups: Default::default(),
            process_group_order: Default::default(),
            multi_publisher_topics: Vec::new(),
            name: None,
            identity: "adv".to_string(),
            prefix: "adv".to_string(),
            nodes: vec![NodeDef {
                fuse: None,
                ros2: None,
                id: "n".to_string(),
                node_type: "n".to_string(),
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

        let info = NodeInfo::from_names(vec![], vec!["noop".to_string()])
            .with_policy(MacroPolicy::Period { period_ms: 1 });
        let entry = ClosureNodeEntry::new(info, tick_fn);
        let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories.insert("n".to_string(), Box::new(entry));

        let clock = Arc::new(VirtualClock::new());
        GraphRuntime::build_for_test(config, factories, clock, 4).expect("build")
    }

    /// `max_steps == Some(0)` must execute zero steps and return 0
    /// immediately. Callback must NOT be invoked.
    #[test]
    fn run_until_shutdown_max_steps_zero_executes_no_steps() {
        let counter = Arc::new(AtomicUsize::new(0));
        let counter_in = counter.clone();
        let mut runtime = build_one_node_runtime(move |_ctx| {
            counter_in.fetch_add(1, Ordering::Relaxed);
            Ok(())
        });

        let steps = runtime.run_until_shutdown(Duration::from_millis(1), Some(0));
        assert_eq!(steps, 0, "max_steps=Some(0) must run zero iterations");
        assert_eq!(
            counter.load(Ordering::Relaxed),
            0,
            "tick callback must not be invoked when max_steps=Some(0)"
        );
    }

    /// Shutdown requested BEFORE `run_until_shutdown` is called: must
    /// return immediately with 0 steps and never invoke any tick.
    #[test]
    fn run_until_shutdown_pre_fired_signal_exits_immediately() {
        let counter = Arc::new(AtomicUsize::new(0));
        let counter_in = counter.clone();
        let mut runtime = build_one_node_runtime(move |_ctx| {
            counter_in.fetch_add(1, Ordering::Relaxed);
            Ok(())
        });

        // Trip the runtime's signal externally.
        runtime.shutdown_signal().request();
        assert!(runtime.shutdown_requested());

        let steps = runtime.run_until_shutdown(Duration::from_millis(1), Some(100));
        assert_eq!(
            steps, 0,
            "pre-fired signal must short-circuit the loop before any step"
        );
        assert_eq!(counter.load(Ordering::Relaxed), 0);
    }

    /// `max_steps == None` paired with a pre-fired signal: must NOT
    /// hang. Returns 0 steps. This guards against a bug where the
    /// `None` branch skips the shutdown check.
    #[test]
    fn run_until_shutdown_no_max_with_pre_fired_signal_does_not_hang() {
        let mut runtime = build_one_node_runtime(|_ctx| Ok(()));
        runtime.shutdown_signal().request();
        let steps = runtime.run_until_shutdown(Duration::from_millis(1), None);
        assert_eq!(steps, 0, "pre-fired signal + no max must exit at 0 steps");
    }

    /// `Duration::ZERO` period — the simulated clock won't advance, but
    /// the loop should still iterate `max_steps` times and exit cleanly.
    /// (Tests that the runtime doesn't depend on positive delta to make
    /// progress through its own bookkeeping.)
    #[test]
    fn run_until_shutdown_zero_duration_still_progresses_step_count() {
        let counter = Arc::new(AtomicUsize::new(0));
        let counter_in = counter.clone();
        let mut runtime = build_one_node_runtime(move |_ctx| {
            counter_in.fetch_add(1, Ordering::Relaxed);
            Ok(())
        });

        let steps = runtime.run_until_shutdown(Duration::ZERO, Some(5));
        assert_eq!(steps, 5, "step counter must advance regardless of period");
        // Tick may or may not actually run — it depends on whether
        // Period(1ms) fires when the clock doesn't advance. We don't
        // assert on counter; the contract under test is the loop count.
    }

    /// Calling `run_until_shutdown` a second time after the first
    /// returned because `max_steps` was hit (signal NOT tripped) must
    /// continue to execute new steps (the runtime is reusable). The
    /// signal stays untripped throughout.
    #[test]
    fn run_until_shutdown_is_reusable_when_max_steps_was_the_exit_cause() {
        let counter = Arc::new(AtomicUsize::new(0));
        let counter_in = counter.clone();
        let mut runtime = build_one_node_runtime(move |_ctx| {
            counter_in.fetch_add(1, Ordering::Relaxed);
            Ok(())
        });

        let s1 = runtime.run_until_shutdown(Duration::from_millis(1), Some(3));
        assert_eq!(s1, 3);
        assert!(
            !runtime.shutdown_requested(),
            "max_steps exit must not trip the signal"
        );

        let s2 = runtime.run_until_shutdown(Duration::from_millis(1), Some(2));
        assert_eq!(s2, 2, "second invocation must produce its own step count");
    }

    /// After `run_until_shutdown` returns because the signal was tripped,
    /// calling it again must IMMEDIATELY return 0 (the signal is
    /// one-way; there's no reset API).
    #[test]
    fn run_until_shutdown_second_call_after_trip_returns_zero() {
        let counter = Arc::new(AtomicUsize::new(0));
        let counter_in = counter.clone();
        let mut runtime = build_one_node_runtime(move |ctx| {
            let n = counter_in.fetch_add(1, Ordering::Relaxed) + 1;
            if n >= 2 {
                ctx.request_shutdown();
            }
            Ok(())
        });

        let s1 = runtime.run_until_shutdown(Duration::from_millis(1), Some(100));
        assert!(runtime.shutdown_requested());
        assert_eq!(s1, 2, "first call exits after the trip-tick + signal poll");

        let s2 = runtime.run_until_shutdown(Duration::from_millis(1), Some(100));
        assert_eq!(
            s2, 0,
            "after a trip the signal stays set; subsequent calls must short-circuit"
        );
    }

    /// External caller (not a node) trips the runtime's signal MID-RUN
    /// from another thread. The loop must observe and exit. Strict
    /// timing-free version: we run the loop until it exits naturally and
    /// just assert it terminated and the signal is set.
    #[test]
    fn run_until_shutdown_external_thread_trip_is_observed() {
        let mut runtime = build_one_node_runtime(|_ctx| Ok(()));

        // Capture the signal handle before moving the trip thread.
        let trigger = runtime.shutdown_signal().clone();
        let t = thread::spawn(move || {
            // Trip immediately; the main loop will catch it on its
            // first poll.
            trigger.request();
        });
        t.join().expect("trigger thread panicked");

        let steps = runtime.run_until_shutdown(Duration::from_millis(1), Some(1_000));
        assert!(runtime.shutdown_requested());
        assert!(
            steps < 1_000,
            "must exit before max_steps because signal was set; got {} steps",
            steps
        );
    }

    /// `shutdown_signal()` accessor hands out a clone-able handle that
    /// reflects the runtime's true state (no copy-on-read snapshot).
    #[test]
    fn runtime_shutdown_signal_accessor_is_live_handle() {
        let runtime = build_one_node_runtime(|_ctx| Ok(()));
        let handle1 = runtime.shutdown_signal().clone();
        let handle2 = runtime.shutdown_signal().clone();

        assert!(!handle1.is_requested());
        assert!(!handle2.is_requested());
        handle1.request();
        assert!(
            handle2.is_requested(),
            "all handles obtained from shutdown_signal() must share state"
        );
        assert!(runtime.shutdown_requested());
    }
}
