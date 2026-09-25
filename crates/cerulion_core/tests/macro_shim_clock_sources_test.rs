// SPDX-License-Identifier: AGPL-3.0-only
//! Group 3: the macro shim's `self.now_ns` /
//! `self.ext_ns()` reads reach the runtime's ACTIVE clock.
//!
//! `macro_shim_methods_test.rs` already pins that `self.virt_ns()` reaches a
//! `VirtualClock` runtime. These tests extend that to the two other clock reads:
//!
//! - `self.now_ns()` returns the runtime's ACTIVE clock value and tracks the
//!   clock as it advances (driven under a `VirtualClock` runtime).
//! - `self.now_ns()` makes EXACTLY ONE active-clock dispatch per call (pinned
//!   via a counting `Clock` wrapper — mirrors the spirit of the D2 warn-once
//!   count pin).
//! - `self.ext_ns()` is `None` under a non-External runtime (VirtualClock) and
//!   `Some(latched)` under an `ExternalClock` runtime.
//!
//! The External-runtime tests use `GraphRuntime::build_live` (which accepts any
//! `Arc<dyn Clock>`) over an isolated per-test iceoryx2 transport, since
//! `build_for_test` is `VirtualClock`-only. ALL three tests are `#[serial]`:
//! each builds an iceoryx2 runtime (one node singleton per process, Principle
//! #8) AND passes the macro node's tick observations back through a
//! process-global `OnceLock` — both demand one-at-a-time execution, so a
//! parallel libtest run (no `--test-threads=1`) would otherwise race the
//! singleton + the set-once OnceLock. `#[serial]` makes the file parallel-safe
//! under `cargo test` (the pattern).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::{Clock, ExternalClock, VirtualClock};
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

// ===========================================================================
// Test 3a — `self.now_ns()` tracks the active VirtualClock as it advances,
// and makes EXACTLY ONE active-clock dispatch per call.
// ===========================================================================

/// Counting `Clock` wrapper: delegates `now_ns()` to an inner `VirtualClock`
/// but counts every dispatch, so the node can assert its `self.now_ns()` shim
/// makes exactly one underlying call. `virt_ns`/`ext_ns` mirror the inner
/// clock's (delegating, uncounted — only `now_ns` is the pin target).
struct CountingClock {
    inner: VirtualClock,
    now_calls: AtomicU64,
}

impl CountingClock {
    fn new() -> Self {
        Self {
            inner: VirtualClock::new(),
            now_calls: AtomicU64::new(0),
        }
    }
}

impl Clock for CountingClock {
    fn now_ns(&self) -> u64 {
        self.now_calls.fetch_add(1, Ordering::Relaxed);
        self.inner.now_ns()
    }
    fn virt_ns(&self) -> Option<u64> {
        self.inner.virt_ns()
    }
    fn ext_ns(&self) -> Option<u64> {
        self.inner.ext_ns()
    }
}

#[derive(Debug, Default)]
struct NowObs {
    // (active_now_ns observed in tick, dispatch_delta across the single shim call)
    per_tick: Mutex<Vec<(u64, u64)>>,
}

static NOW_OBS: std::sync::OnceLock<Arc<NowObs>> = std::sync::OnceLock::new();
static NOW_CLOCK: std::sync::OnceLock<Arc<CountingClock>> = std::sync::OnceLock::new();

#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct NowSink {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl NowSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let clock = NOW_CLOCK.get().expect("clock set by test");
        // Snapshot the dispatch counter, make EXACTLY ONE shim call, snapshot
        // again. The delta must be 1 — the shim dispatches once.
        let before = clock.now_calls.load(Ordering::Relaxed);
        let t = self.now_ns();
        let after = clock.now_calls.load(Ordering::Relaxed);

        self.out.x = t as f64;
        NOW_OBS
            .get()
            .expect("obs set by test")
            .per_tick
            .lock()
            .unwrap()
            .push((t, after - before));
        Ok(())
    }
}

#[test]
#[serial]
fn shim_now_ns_tracks_active_clock_and_dispatches_once() {
    let obs = Arc::new(NowObs::default());
    NOW_OBS.set(obs.clone()).expect("obs set once");
    let clock = Arc::new(CountingClock::new());
    // `OnceLock::set` returns Err(value) if already set; `CountingClock` is
    // not Debug so we can't `.expect()`. This test owns its own binary and
    // runs once, so a plain `set` (ignoring the Result) is correct.
    let _ = NOW_CLOCK.set(clock.clone());

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "now_shim".to_string(),
        prefix: "now_shim".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "sink".to_string(),
            node_type: "now_sink".to_string(),
            inputs: vec![],
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
    factories.insert("sink".to_string(), Box::new(NowSinkEntry::new()));

    // build_live accepts any Arc<dyn Clock>; use the isolated singleton-free
    // transport carrying the SAME counting clock so the node + publisher share
    // it. We drive Period fires by advancing the inner VirtualClock + stepping.
    let clock_dyn: Arc<dyn Clock> = clock.clone();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "now_shim_test".into(),
            clock: clock_dyn.clone(),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init_for_test");
    let mut runtime =
        GraphRuntime::build_live(config, factories, &mgr, clock_dyn).expect("build_live");

    // Advance the inner virtual clock 1ms per step, three times. Each step
    // fires the period node once; the node observes the cumulative time.
    for _ in 0..3 {
        clock.inner.advance_ms(1);
        runtime.step(Duration::from_millis(1));
    }

    let per_tick = obs.per_tick.lock().unwrap();
    assert_eq!(per_tick.len(), 3, "period node should fire exactly 3 times");

    // now_ns tracks the active clock: strictly increasing, first == 1ms.
    assert_eq!(
        per_tick[0].0, 1_000_000,
        "first tick's now_ns() must equal the active clock at 1ms"
    );
    for w in per_tick.windows(2) {
        assert!(
            w[1].0 > w[0].0,
            "now_ns() must strictly increase as the active clock advances: {:?}",
            (w[0].0, w[1].0)
        );
    }
    // EXACTLY ONE dispatch per shim call (the load-bearing pin).
    for (i, (_, delta)) in per_tick.iter().enumerate() {
        assert_eq!(
            *delta, 1,
            "self.now_ns() must make exactly one active-clock dispatch (tick {i}, got {delta})"
        );
    }
}

// ===========================================================================
// Test 3a' — the determinism-safety contract: under a VirtualClock runtime,
// `self.now_ns()` (active, determinism-safe) reads the SMALL virtual value,
// while `self.real_ns()` (raw hardware wall, NON-deterministic) reads the
// LARGE CLOCK_MONOTONIC/UPTIME_RAW value — they DIVERGE. A regression that
// wired the shim's `now_ns()` to wall time (or `real_ns()` to the active
// clock) collapses the gap and fails here. This is the node-facing twin of
// the publish-stamp lock in clock_model_e2e_iox2_test.
// ===========================================================================

#[derive(Debug, Default)]
struct RealNowObs {
    // (now_ns active, real_ns wall) per tick
    per_tick: Mutex<Vec<(u64, u64)>>,
}

static REAL_NOW_OBS: std::sync::OnceLock<Arc<RealNowObs>> = std::sync::OnceLock::new();

#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct RealNowSink {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl RealNowSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let now = self.now_ns(); // active source — the injected VirtualClock
        let real = self.real_ns(); // raw hardware wall — source-independent
        self.out.x = now as f64;
        REAL_NOW_OBS
            .get()
            .expect("obs set by test")
            .per_tick
            .lock()
            .unwrap()
            .push((now, real));
        Ok(())
    }
}

#[test]
#[serial]
fn shim_now_ns_is_virtual_while_real_ns_is_wall() {
    let obs = Arc::new(RealNowObs::default());
    let _ = REAL_NOW_OBS.set(obs.clone());

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "realnow".to_string(),
        prefix: "realnow".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "sink".to_string(),
            node_type: "real_now_sink".to_string(),
            inputs: vec![],
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
    factories.insert("sink".to_string(), Box::new(RealNowSinkEntry::new()));

    // VirtualClock runtime: now_ns() reads it (small, explicitly advanced);
    // real_ns() reads raw hardware wall, independent of the injected clock.
    let clock = Arc::new(VirtualClock::new());
    let clock_dyn: Arc<dyn Clock> = clock.clone();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "realnow_test".into(),
            clock: clock_dyn.clone(),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init_for_test");
    let mut runtime =
        GraphRuntime::build_live(config, factories, &mgr, clock_dyn).expect("build_live");

    for _ in 0..3 {
        clock.advance_ms(1);
        runtime.step(Duration::from_millis(1));
    }

    let per_tick = obs.per_tick.lock().unwrap();
    assert_eq!(per_tick.len(), 3, "period node should fire 3 times");
    for (i, (now, real)) in per_tick.iter().enumerate() {
        // now_ns is the SMALL active (virtual) value: 1ms, 2ms, 3ms.
        assert_eq!(
            *now,
            (i as u64 + 1) * 1_000_000,
            "tick {i}: self.now_ns() must equal the active VirtualClock, got {now}"
        );
        // real_ns is raw wall (ns since boot) — a DIFFERENT source than the
        // few-ms virtual now: by this tick the runtime build (iceoryx2 init)
        // + prior steps have elapsed far more wall time than the ≤3ms virtual
        // clock. A now_ns→wall or real_ns→active regression collapses them to
        // equal. (No absolute ns-since-boot floor — that would flake on a
        // freshly-booted CI runner where uptime can read < 1s.)
        assert!(
            *real > *now,
            "tick {i}: real_ns ({real}) must be raw wall, NOT the active virtual now_ns ({now}) — different sources"
        );
    }
}

// ===========================================================================
// Test 3b — `self.ext_ns()` is None under VirtualClock, Some(latched) under
// ExternalClock.
// ===========================================================================

#[derive(Debug, Default)]
struct ExtObs {
    per_tick: Mutex<Vec<Option<u64>>>,
}

static EXT_OBS: std::sync::OnceLock<Arc<ExtObs>> = std::sync::OnceLock::new();

#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct ExtSink {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl ExtSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let e = self.ext_ns();
        // touch the output so the per-tick scope runs end-to-end
        self.out.x = e.unwrap_or(0) as f64;
        EXT_OBS
            .get()
            .expect("obs set by test")
            .per_tick
            .lock()
            .unwrap()
            .push(e);
        Ok(())
    }
}

fn ext_sink_config() -> GraphConfig {
    GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "ext_shim".to_string(),
        prefix: "ext_shim".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "sink".to_string(),
            node_type: "ext_sink".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "geometry_msgs/Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    }
}

/// Under a VirtualClock runtime, `self.ext_ns()` is `None` (not external).
#[test]
#[serial]
fn shim_ext_ns_is_none_under_virtual_clock() {
    let obs = Arc::new(ExtObs::default());
    // Each ext_* test installs its own OBS; they run in the same binary but
    // OnceLock is set-once, so only ONE may set it. Guard: set if unset, else
    // reuse — but the two tests assert disjoint contents, so we clear between.
    let _ = EXT_OBS.set(obs.clone());
    let installed = EXT_OBS.get().expect("obs present");
    installed.per_tick.lock().unwrap().clear();

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("sink".to_string(), Box::new(ExtSinkEntry::new()));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(ext_sink_config(), factories, clock, 8)
        .expect("build_for_test");

    for _ in 0..3 {
        runtime.step(Duration::from_millis(1));
    }

    let per_tick = installed.per_tick.lock().unwrap();
    assert_eq!(per_tick.len(), 3, "period node fires 3 times");
    assert!(
        per_tick.iter().all(|e| e.is_none()),
        "ext_ns() must be None under a VirtualClock runtime, got {:?}",
        *per_tick
    );
}

/// Under an ExternalClock runtime, `self.ext_ns()` is `Some(latched)` and
/// tracks `set_external`.
#[test]
#[serial]
fn shim_ext_ns_is_some_under_external_clock() {
    let obs = Arc::new(ExtObs::default());
    let _ = EXT_OBS.set(obs.clone());
    let installed = EXT_OBS.get().expect("obs present");
    installed.per_tick.lock().unwrap().clear();

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("sink".to_string(), Box::new(ExtSinkEntry::new()));

    let ext = Arc::new(ExternalClock::new());
    let clock_dyn: Arc<dyn Clock> = ext.clone();
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "ext_shim_test".into(),
            clock: clock_dyn.clone(),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init_for_test");
    let mut runtime = GraphRuntime::build_live(ext_sink_config(), factories, &mgr, clock_dyn)
        .expect("build_live");

    // Drive the external master forward by 1ms each step so the period node
    // fires. The node should observe ext_ns() == the latched external time.
    let fed = [1_000_000u64, 2_000_000, 3_000_000];
    for &t in &fed {
        ext.set_external(t);
        runtime.step(Duration::from_millis(1));
    }

    let per_tick = installed.per_tick.lock().unwrap();
    assert_eq!(per_tick.len(), fed.len(), "period node fires once per step");
    for (i, (&want, got)) in fed.iter().zip(per_tick.iter()).enumerate() {
        assert_eq!(
            *got,
            Some(want),
            "tick {i}: ext_ns() must equal the latched external master time"
        );
    }
}
