// SPDX-License-Identifier: AGPL-3.0-only
//! Within-level rayon fire: two coverage arms.
//!
//! Covers two arms of the within-level rayon fire
//! path. A third arm (a real `DylibNodeEntry` routed serial under parallel
//! fire) lives in `rayon_fire_cdylib_serial_test.rs`.
//!
//! ## TEST 1 — `CERULION_FIRE_THREADS` bad/zero-value warn branch
//!
//! `GraphRuntime::build_with_scheduler` sizes the within-level fire pool from
//! env `CERULION_FIRE_THREADS`:
//!   - absent          → silent auto-size (`min(available_parallelism, width)`)
//!   - present, `> 0`   → honored
//!   - present, unparseable / `0` → loud `tracing::warn!` + auto-size fallback
//!
//! The warn branch was untested. Cases here:
//!   (a) `"abc"` → warn naming the bad value, runtime still builds + fires
//!   (b) `"0"`   → same (zero rejected by the `> 0` filter)
//!   (c) `"2"`   → VALID → NO warn (negative control)
//!
//! ## TEST 3 — `step()` snapshot-loop registration-desync else-arm
//!
//! `GraphRuntime::step`'s snapshot loop guards a by-construction invariant:
//! every firing node in `snapshot_input_names` is ALSO in `self.nodes`. The
//! else-arm (`debug_assert!(false, "...registration desync")` +
//! `tracing::error!`) is unreachable in normal flow. The
//! `force_snapshot_desync_for_test` seam removes a firing snapshot node from
//! `self.nodes` only (leaving it in the scheduler + `snapshot_input_names`) so
//! the else-arm fires. Tests run in debug, so the `debug_assert!(false)`
//! panics — pinned via `#[should_panic(expected = "registration desync")]`.
//!
//! # Running
//!
//! ```bash
//! cargo build -p cerulion_core --features test-helpers
//! cargo test -p cerulion_core --test fire_threads_env_test -- --test-threads=1
//! ```
//!
//! `#[serial]` because `CERULION_FIRE_THREADS` is process-global; an RAII guard
//! removes it on drop, panic-safe. `build_for_test` reads the env at BUILD
//! time, so the guard must be live across the build call.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing_test::traced_test;

/// RAII guard that sets `CERULION_FIRE_THREADS` and removes it on drop — even
/// if an assertion panics mid-test. Mirrors `rayon_fire_iox2_test`'s
/// `FireThreadsGuard` / `chunk_c_ffi_codes_3_4_test`'s `EnvVarGuard`.
struct FireThreadsGuard;
impl FireThreadsGuard {
    fn set(value: &str) -> Self {
        std::env::set_var("CERULION_FIRE_THREADS", value);
        Self
    }
}
impl Drop for FireThreadsGuard {
    fn drop(&mut self) {
        std::env::remove_var("CERULION_FIRE_THREADS");
    }
}

// ===========================================================================
// Node types: a Period(10) producer + a Period(10) consumer with a plain
// (non-trigger) input. Period ⇒ the input is latest-value ⇒ the consumer is in
// `snapshot_input_names` — the surface both tests exercise. Real iceoryx2
// publishes/reads (no fake data — Principle #13).
// ===========================================================================

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct EnvProducer {
    #[output]
    out: Vector3,
    n: u64,
}

#[cerulion_node_impl]
impl EnvProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct EnvConsumer {
    /// Plain (non-trigger) latest-value input → consumer lands in
    /// `snapshot_input_names`.
    #[input]
    inp: Vector3,
    last: u64,
}

#[cerulion_node_impl]
impl EnvConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // A real read of the (frozen) slot every tick.
        self.last = self.inp.x as u64;
        Ok(())
    }
}

/// One producer (`prod`) + one snapshotting consumer (`cons`), both Period(10),
/// level 0. `cons` carries a plain non-trigger input from `prod/out`, so it is
/// in `snapshot_input_names`.
fn prod_cons_graph(prefix: &str) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "deferral_cov".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "prod".to_string(),
                node_type: "env_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "cons".to_string(),
                node_type: "env_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "prod/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("prod".to_string(), Box::new(EnvProducerEntry::new()));
    factories.insert("cons".to_string(), Box::new(EnvConsumerEntry::new()));
    (config, factories)
}

// ===========================================================================
// TEST 1 — bad / zero CERULION_FIRE_THREADS warns + auto-sizes (still builds).
//
// (a) "abc" and (b) "0" in one #[serial] body: the warn fires, the runtime
// still builds, and a node still fires. The shared per-test log buffer holds
// both warns; the bad value name is asserted for each.
// ===========================================================================
#[test]
#[serial]
#[traced_test]
fn fire_threads_bad_or_zero_value_warns_and_auto_sizes() {
    // (a) unparseable.
    {
        let _guard = FireThreadsGuard::set("abc");
        let (config, factories) = prod_cons_graph("ftbad_abc");
        let clock = Arc::new(VirtualClock::new());
        let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
            .expect("runtime must still build with a bad CERULION_FIRE_THREADS value");
        for _ in 0..3 {
            runtime.step(Duration::from_millis(10));
        }
        assert!(
            runtime
                .node_handle("cons")
                .map(|h| h.fire_count() > 0)
                .unwrap_or(false),
            "the consumer must still fire after the bad-env auto-size fallback"
        );
    }
    assert!(
        logs_contain("CERULION_FIRE_THREADS was set but is not a positive integer"),
        "an unparseable CERULION_FIRE_THREADS must emit the loud auto-size warn"
    );
    assert!(
        logs_contain("bad_value=abc"),
        "the warn must name the offending value 'abc'"
    );

    // (b) zero — rejected by the `> 0` filter.
    {
        let _guard = FireThreadsGuard::set("0");
        let (config, factories) = prod_cons_graph("ftbad_zero");
        let clock = Arc::new(VirtualClock::new());
        let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
            .expect("runtime must still build with CERULION_FIRE_THREADS=0");
        for _ in 0..3 {
            runtime.step(Duration::from_millis(10));
        }
        assert!(
            runtime
                .node_handle("prod")
                .map(|h| h.fire_count() > 0)
                .unwrap_or(false),
            "the producer must still fire after the zero-env auto-size fallback"
        );
    }
    assert!(
        logs_contain("bad_value=0"),
        "the zero-value warn must name the offending value '0'"
    );
}

// (c) VALID value → NO warn. Separate #[traced_test] body so the no-warn
// assertion is not polluted by (a)/(b)'s warns in the shared log buffer.
#[test]
#[serial]
#[traced_test]
fn fire_threads_valid_value_emits_no_warn() {
    let _guard = FireThreadsGuard::set("2");
    let (config, factories) = prod_cons_graph("ftgood_2");
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("runtime must build with a valid CERULION_FIRE_THREADS=2");
    for _ in 0..3 {
        runtime.step(Duration::from_millis(10));
    }
    assert!(
        runtime
            .node_handle("cons")
            .map(|h| h.fire_count() > 0)
            .unwrap_or(false),
        "the consumer must fire under a valid forced thread count"
    );
    assert!(
        !logs_contain("CERULION_FIRE_THREADS was set but is not a positive integer"),
        "a VALID CERULION_FIRE_THREADS value must NOT emit the bad-value warn"
    );
}

// ===========================================================================
// TEST 3 — step() snapshot-loop registration-desync else-arm.
//
// `force_snapshot_desync_for_test("cons")` removes the firing snapshot
// consumer from `self.nodes` only; it stays in the scheduler (still
// fires/decided) and in `snapshot_input_names` (still needs a freeze) → the
// step() snapshot loop hits the else-arm. Debug build ⇒ the
// `debug_assert!(false, "...registration desync")` panics.
// ===========================================================================
#[test]
#[serial]
#[should_panic(expected = "registration desync")]
fn step_snapshot_loop_registration_desync_is_loud() {
    let (config, factories) = prod_cons_graph("desync");
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build desync graph");

    // Warm up one clean step so `cons` is registered + firing normally.
    runtime.step(Duration::from_millis(10));

    // Force the desync: remove the FIRING snapshot consumer from self.nodes
    // only (scheduler + snapshot_input_names retain it) → else-arm trips.
    assert!(
        runtime.force_snapshot_desync_for_test("cons"),
        "the seam must have removed an existing node ('cons' was registered)"
    );

    // This step's snapshot loop hits the else-arm; the debug_assert panics
    // with "registration desync".
    runtime.step(Duration::from_millis(10));
}
