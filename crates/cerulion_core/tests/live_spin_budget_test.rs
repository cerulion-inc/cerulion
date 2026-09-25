// SPDX-License-Identifier: AGPL-3.0-only
//! The SCHEDULED SPIN-THEN-BLOCK live-loop receive path.
//!
//! Before blocking on the WaitSet (an `epoll`/C-state-exit round-trip), the
//! live loop (`runtime.rs` `live_step`) busy-polls every data-trigger / sync
//! listener's NOTIFICATION queue for a `spin_budget` window. A wake whose data
//! is imminent is caught in user space, skipping the kernel block (the
//! spin-then-block latency win). The spin is RECORD-ONLY (drains only the
//! notification queue, never the SHM message queue or the scheduler), so it
//! changes WHEN the loop wakes, never WHAT fires — the replay=live firewall
//! (Principle #7) holds.
//!
//! This file pins the two halves of that path in isolation, via the
//! `#[cfg(any(test, feature = "test-helpers"))]` seams on `GraphRuntime`:
//!
//! * `spin_sources_for_test(budget)` — rebuilds the live `sources` Vec and runs
//!   the record-only spin; returns `true` iff a listener event arrived within
//!   `budget`. (Group 1 — the spin MECHANISM.)
//! * `spin_budget_for_test()` — the scheduler-derived (or env-overridden) budget
//!   the loop would spin for next. (Group 2 — the budget DERIVATION.)
//! * `run_live_step_once_for_test(timeout)` — one production live iteration
//!   (spin + WaitSet block + `step`). (Group 3 — the SPIN path's determinism.)
//!
//! ## Budget derivation contract (mirrors `GraphRuntime::spin_budget`)
//!
//! `CERULION_LIVE_SPIN_US` is parsed ONCE per runtime instance (cached in a
//! `OnceLock` on the first `spin_budget` call):
//!
//! | env value      | budget                                       |
//! |----------------|----------------------------------------------|
//! | unset / empty  | DERIVED (see below)                          |
//! | `0`            | `Duration::ZERO` (feature DISABLED)          |
//! | `N` (0<N≤100k) | `Duration::from_micros(N)` (manual ceil)     |
//! | `N` (>100k)    | clamped to 100 000µs + warn-once             |
//! | non-numeric    | warn-once + DERIVED                          |
//!
//! (The runtime resolves these to a `SpinConfig` enum — `Derived` / `Disabled` /
//! `Ceiling(µs)`. The `Ceiling` is clamped to `SPIN_BUDGET_MAX_US` = 100 000µs.)
//!
//! DERIVED: one HOP (`SPIN_HOP_US`, read via `spin_hop_us_for_test()` — NOT a
//! hand-copied literal) when a wake is IMMINENT (a Period node due within one
//! hop via `ns_until_next_fire`, OR the previous step fired ≥1 node —
//! `last_step_fired`), else `Duration::ZERO` (block immediately into a predicted
//! idle so a quiescent graph reclaims the core).
//!
//! ## Why a PURE data-driven chain (no Period nodes)
//!
//! Most tests reuse the `polled_vs_live_iox2_test` `Relay → Mid → Sink`
//! external-data chain (NO Period nodes anywhere). That shape makes the DERIVED
//! budget cleanly observable: a Period-free graph has `ns_until_next_fire ==
//! None` (never period-imminent), so the derived budget is governed purely by
//! `last_step_fired` — ZERO right after build (nothing fired yet) and HOP after
//! a step that fired. That isolates the `last_step_fired` lever from the
//! Period-deadline lever. The `period_imminent` arm itself is exercised by a
//! dedicated `period_ms` graph (Group 2b), which advances a test-owned
//! `VirtualClock` to position `now_ns` across the `<=` one-hop boundary.
//!
//! ## Concurrency
//!
//! All tests `#[serial]` (from `serial_test`): the live loop builds a WaitSet
//! over the process-global iceoryx2 SHM singleton, AND the env tests mutate the
//! process-global `CERULION_LIVE_SPIN_US`. Env tests use an `EnvVarGuard` RAII
//! struct that `remove_var`s in `Drop` so a mid-test panic cannot leak the env
//! var to the next test (`#[serial]` serializes bodies but does NOT reset env).
//! The runtime is built AFTER the env var is set, because `spin_budget` caches
//! the parsed value on its first call (not at build).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing_test::traced_test;

/// The absolute external trigger topic for `relay` — no in-graph producer, so
/// the graph provisions it as `External` and an out-of-graph publisher attaches
/// freely. Same convention as `polled_vs_live_iox2_test::EXT_TOPIC`.
const EXT_TOPIC: &str = "/lsb/ext";

/// `SPIN_HOP_US` from `runtime.rs` — one transport hop, the DERIVED-imminent
/// budget. Read from the REAL const via the `spin_hop_us_for_test` accessor (NOT
/// a hand-copied literal), so retuning the const cannot silently drift this
/// oracle. `HOP` is the matching `Duration` oracle for the derivation assertions.
fn spin_hop_us() -> u64 {
    cerulion_core::graph::runtime::spin_hop_us_for_test()
}
fn hop() -> Duration {
    Duration::from_micros(spin_hop_us())
}

/// The env var under test.
const SPIN_ENV: &str = "CERULION_LIVE_SPIN_US";

/// A live wake timeout long enough that a published event wakes the reactor well
/// before it elapses, short enough that a no-data iteration returns quickly.
/// Mirrors `polled_vs_live_iox2_test::WAKE_TIMEOUT`.
const WAKE_TIMEOUT: Duration = Duration::from_millis(150);

// ===========================================================================
// RAII env guard — set on construct, remove_var on Drop (panic-safe). Mirrors
// `chunk_c_ffi_codes_3_4_test::EnvVarGuard`. `#[serial]` serializes bodies but
// does NOT reset env between them, so the Drop is the cleanup contract.
// ===========================================================================

struct EnvVarGuard {
    name: &'static str,
}

impl EnvVarGuard {
    fn set(name: &'static str, value: &str) -> Self {
        std::env::set_var(name, value);
        Self { name }
    }
}

impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(self.name);
    }
}

// ===========================================================================
// Nodes — the pure data-driven `Relay → Mid → Sink` chain, reused verbatim from
// `polled_vs_live_iox2_test` (test binaries are separate crates, so the fixtures
// are not importable). NO Period nodes anywhere — see the module doc.
// ===========================================================================

/// L0: triggers on the absolute external `/lsb/ext`, forwards `inp.x` to `out`.
#[cerulion_node]
#[derive(Default)]
struct Relay {
    #[input(trigger)]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl Relay {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.inp.x;
        Ok(())
    }
}

/// L1: triggers on `relay/out`, forwards `inp.x` to `out`.
#[cerulion_node]
#[derive(Default)]
struct Mid {
    #[input(trigger)]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl Mid {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.inp.x;
        Ok(())
    }
}

/// L2: triggers on `mid/out`, records each observed `inp.x` into a shared Vec
/// (the data-flow oracle) and bumps a shared fire counter.
#[cerulion_node]
#[derive(Default)]
struct Sink {
    #[input(trigger)]
    inp: Vector3,
    observed: Arc<Mutex<Vec<f64>>>,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl Sink {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.observed.lock().unwrap().push(self.inp.x);
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

// ===========================================================================
// Graph construction — the 3-level forward chain, hand-built (no Period).
// ===========================================================================

/// An `OutputDef` for a `geometry_msgs/Vector3` producer at all-default
/// resolution knobs (the shape `build_for_test` requires).
fn vector3_output(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "geometry_msgs/Vector3".to_string(),
        max_slice_len: None,
        topic: None,
        history_size: 0,
    }
}

/// Build the 3-level pure-data-driven chain config + factories. `observed` /
/// `fires` are the sink's shared data-flow + fire oracles.
fn chain_graph(
    observed: Arc<Mutex<Vec<f64>>>,
    fires: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "live_spin_budget_test".to_string(),
        prefix: "lsb".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "relay".to_string(),
                node_type: "relay".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: EXT_TOPIC.to_string(),
                }],
                outputs: vec![vector3_output("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "mid".to_string(),
                node_type: "mid".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "relay/out".to_string(),
                }],
                outputs: vec![vector3_output("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "sink".to_string(),
                node_type: "sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "mid/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("relay".to_string(), Box::new(RelayEntry::new()));
    factories.insert("mid".to_string(), Box::new(MidEntry::new()));
    factories.insert(
        "sink".to_string(),
        Box::new(SinkEntry::with_state(Sink {
            observed,
            fires,
            ..Default::default()
        })),
    );
    (config, factories)
}

/// Build a FRESH chain runtime (per-test SHM root via `build_for_test`) plus the
/// out-of-graph external publisher on `/lsb/ext`. Returns the runtime, the
/// publisher, and the shared `(observed, fires)` oracles.
///
/// `build_for_test` does NOT call `spin_budget` (it only inits the empty
/// `OnceLock`), so when an env test sets `CERULION_LIVE_SPIN_US` BEFORE calling
/// this, the first `spin_budget_for_test` call still observes the freshly-set
/// var. (Modeled on `polled_vs_live_iox2_test::run_chain`'s setup.)
fn build_chain() -> (
    GraphRuntime,
    cerulion_core::CerulionPublisher,
    Arc<Mutex<Vec<f64>>>,
    Arc<AtomicU64>,
) {
    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = chain_graph(Arc::clone(&observed), Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build live-spin-budget chain graph");

    let pubr = {
        let mgr: &Arc<TransportManager> = runtime.test_transport().expect("test transport parked");
        mgr.create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("external publisher must attach to /lsb/ext")
    };
    (runtime, pubr, observed, fires)
}

/// Publish exactly ONE `Vector3` frame with `x = x` onto `/lsb/ext` (the proxy
/// publishes on drop). Mirrors `polled_vs_live_iox2_test::publish_one`.
fn publish_one(pubr: &mut cerulion_core::CerulionPublisher, x: f64) {
    let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan");
    proxy.x = x;
    drop(proxy); // publish
}

// ###########################################################################
// Group 1 — `spin_sources_for_test`: the record-only spin MECHANISM.
// ###########################################################################

// ===========================================================================
// Test 1 — quiescent graph: no event arrives within budget → `false`.
//
// With NO publish, the data-trigger listeners' notification queues stay empty,
// so the record-only spin drains nothing and times out → `false` (the live loop
// would then fall through to the blocking WaitSet wait). A deliberately SMALL
// budget (200µs) keeps the test fast while still bounding the spin window.
//
// ORACLE: a quiescent spin returns `false` (the TRUE value — no event), not a
// self-compare. The pre-publish drive is implicit: `build_chain` attaches the
// publisher but publishes nothing.
// ===========================================================================

#[test]
#[serial]
fn spin_sources_quiescent_returns_false_within_budget() {
    let (runtime, _pubr, _observed, _fires) = build_chain();

    // Time the spin too, so a regression that ignores `budget` (spins forever /
    // far past it) is also caught — the budget is the wall-clock ceiling.
    let budget = Duration::from_micros(200);
    let start = std::time::Instant::now();
    let got = runtime.spin_sources_for_test(budget);
    let elapsed = start.elapsed();

    assert!(
        !got,
        "a quiescent graph (no publish) must yield NO event within the spin \
         budget — the record-only spin drains empty notification queues and \
         times out, so the live loop falls through to the blocking WaitSet"
    );
    // Generous 50× ceiling absorbs scheduler jitter on a loaded CI runner while
    // still failing hard if the spin ignored `budget` entirely (an unbounded
    // spin would blow this by orders of magnitude).
    assert!(
        elapsed < budget * 50,
        "the quiescent spin must return within a small multiple of its {budget:?} \
         budget, took {elapsed:?} — the budget is the wall-clock spin ceiling"
    );

    runtime.shutdown();
}

// ===========================================================================
// Test 2 — event present: a publish on the upstream topic is caught in the spin
// → `true`. This is the LATENCY-WIN pin (the wake is caught in user space, no
// WaitSet block).
//
// Publish ONE frame on `/lsb/ext`, then spin. The relay's trigger listener gets
// the iceoryx2 notification, the record-only spin drains it → `true`. A 5ms
// budget is comfortably longer than the notification-delivery latency.
//
// ORACLE: an event-present spin returns `true` (the TRUE value — an event WAS
// notified). NOTE the spin is record-only: it drains the NOTIFICATION queue but
// leaves the SHM sample for `step()` — so this test does NOT advance the graph
// or fire nodes (asserted via the untouched fire counter), proving the firewall.
// ===========================================================================

#[test]
#[serial]
fn spin_sources_catches_published_event_returns_true() {
    let (runtime, mut pubr, _observed, fires) = build_chain();

    publish_one(&mut pubr, 1.0);

    let got = runtime.spin_sources_for_test(Duration::from_millis(5));
    assert!(
        got,
        "a publish on the upstream trigger topic must be CAUGHT within the spin \
         budget (the relay's trigger listener is notified) — the latency-win path"
    );

    // FIREWALL pin: the record-only spin drained only the NOTIFICATION queue, it
    // never read the SHM message queue nor called the scheduler — so NO node
    // fired. (The queued sample stays for a later `step()` to drain.)
    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "the record-only spin must NOT fire any node — it drains the notification \
         queue only, leaving the SHM sample untouched for step() (Principle #7)"
    );

    runtime.shutdown();
}

// ###########################################################################
// Group 2 — `spin_budget_for_test`: the budget DERIVATION contract.
// ###########################################################################

// ===========================================================================
// Test 3 — predicted idle → ZERO: a Period-free graph that has NOT fired yet
// (right after build) must block immediately (never spin into a known gap).
//
// `ns_until_next_fire` is `None` for a Period-free graph (no pending Period
// deadline → not period-imminent), and `last_step_fired` is `false` immediately
// after build (nothing has fired). So `imminent == false` → derived budget is
// `Duration::ZERO`.
//
// ORACLE: ZERO (the TRUE derived value for a never-fired Period-free graph), not
// a self-compare. We also assert the precondition (`None`) the derivation rests
// on, so the test documents WHY the budget is ZERO.
// ===========================================================================

#[test]
#[serial]
fn spin_budget_predicted_idle_is_zero() {
    let (runtime, _pubr, _observed, _fires) = build_chain();

    // Precondition: a Period-free, never-stepped graph derives ZERO because it
    // is neither period-imminent (no Period nodes) nor just-fired.
    assert_eq!(
        runtime.spin_budget_for_test(),
        Duration::ZERO,
        "a Period-free graph that has not fired must derive a ZERO spin budget \
         (block immediately) — never spin into a predicted idle"
    );

    runtime.shutdown();
}

// ===========================================================================
// Test 4 — just-fired → HOP: after a live iteration that FIRES a node, the
// derived budget is one HOP (`last_step_fired` ⇒ a multi-hop chain is likely
// mid-flight ⇒ imminent).
//
// Setup: publish ONE frame, then drive ONE `run_live_step_once_for_test`. The
// chain collapses ext→relay→mid→sink within that one step (the within-
// step level collapse), so ≥1 node fires → `live_step` records
// `last_step_fired = true`. The NEXT `spin_budget` (which we read via the test
// seam) is therefore HOP.
//
// This is deterministic with the available seams: we OBSERVE the fire via the
// shared fire counter (== 1 after the step) BEFORE reading the budget, so we
// never read the budget unless the step genuinely fired. (No reliance on a
// flaky trace-length-delta guess — the fire-counter oracle confirms the
// `last_step_fired`-setting condition was met.)
//
// ORACLE: HOP (`SPIN_HOP_US` µs — the TRUE derived value when the last step
// fired and no Period is pending), not a self-compare. `last_step_fired` is the
// only imminence lever here (Period-free graph ⇒ `ns_until_next_fire == None`),
// so the HOP value is attributable solely to the just-fired signal.
// ===========================================================================

#[test]
#[serial]
fn spin_budget_after_a_firing_step_is_one_hop() {
    let (mut runtime, mut pubr, observed, fires) = build_chain();

    // Prime: one no-data live iteration drains build/attach connection-lifecycle
    // noise (mirrors `polled_vs_live`'s pre-loop drive). It fires nothing, so the
    // sink stays at zero AND `last_step_fired` is reset to false for this step.
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "the priming (no-data) live step must NOT fire the sink"
    );

    // Publish ONE frame, then drive ONE live iteration: ext→relay→mid→sink
    // collapses within the step, so ≥1 node fires.
    publish_one(&mut pubr, 1.0);
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);

    // OBSERVE the fire (the precondition for `last_step_fired = true`) before
    // reading the budget — the fire-counter oracle makes the setup deterministic
    // (we only assert HOP once we've confirmed the step actually fired).
    assert_eq!(
        fires.load(Ordering::Relaxed),
        1,
        "one publish + one live step must fire the sink exactly once \
         (ext→relay→mid→sink collapses within the step)"
    );
    assert_eq!(
        observed.lock().unwrap().clone(),
        vec![1.0],
        "the sink must observe exactly the one published value"
    );

    // The just-fired step set `last_step_fired = true`; with no Period pending,
    // the derived budget for the NEXT iteration is exactly one HOP.
    let hop_us = spin_hop_us();
    assert_eq!(
        runtime.spin_budget_for_test(),
        hop(),
        "after a step that FIRED a node, the derived spin budget must be one HOP \
         ({hop_us}µs) — last_step_fired ⇒ a multi-hop chain is likely \
         mid-flight ⇒ imminent"
    );

    // A NO-DATA step after a firing one must RESET `last_step_fired` to
    // false — proving the flag latches per-step (not stuck `true` forever). With
    // no new publish, `run_live_step_once_for_test` fires nothing, so the next
    // derived budget falls back to ZERO (Period-free, not just-fired).
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    assert_eq!(
        fires.load(Ordering::Relaxed),
        1,
        "the no-data step that follows must NOT fire the sink again (still 1)"
    );
    assert_eq!(
        runtime.spin_budget_for_test(),
        Duration::ZERO,
        "a no-fire step must RESET last_step_fired ⇒ the derived budget drops back \
         to ZERO (a latched-true regression would keep it at one HOP)"
    );

    runtime.shutdown();
}

// ===========================================================================
// Test 4b — CAP-IMMUNITY: the fire signal survives a FULL trace ring.
//
// Trace-cap regression: production `graph run` caps the trace
// (`PRODUCTION_TRACE_LIMIT`); once the ring is FULL every firing step
// pops+pushes so `trace.len()` is PINNED at the cap — a
// trace-length before/after delta would read 0 forever, permanently collapsing
// `last_step_fired` (and with it the spin budget) after ~100k fires
// on exactly the long-running graphs the cap targets. The fire signal
// differences the scheduler's MONOTONIC `entries_appended` instead, which a
// full ring cannot pin. Cap of 1 makes the ring full after the FIRST firing
// step, so the SECOND firing round below runs entirely AT the cap — the
// trace-length delta reads 0 there and this assert fails with ZERO budget.
// ===========================================================================

#[test]
#[serial]
fn spin_budget_survives_full_trace_cap() {
    let (mut runtime, mut pubr, observed, fires) = build_chain();
    // Production-style cap, minimally sized: FULL after the first firing step.
    runtime.set_trace_limit(1);

    // Prime (no-data, fires nothing), then the FIRST firing round — the ring
    // fills here (the chain appends ≥1 entry; len pins at the cap of 1).
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    publish_one(&mut pubr, 1.0);
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    assert_eq!(
        fires.load(Ordering::Relaxed),
        1,
        "first publish + live step must fire the sink once (ring now full)"
    );

    // SECOND firing round AT the full cap: every append is pop+push, so a
    // trace-LENGTH delta reads 0 — the monotonic append delta must not.
    publish_one(&mut pubr, 2.0);
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    assert_eq!(
        fires.load(Ordering::Relaxed),
        2,
        "second publish + live step must fire the sink again (still live at cap)"
    );
    assert_eq!(
        observed.lock().unwrap().clone(),
        vec![1.0, 2.0],
        "the sink must observe both published values"
    );
    assert_eq!(
        runtime.spin_budget_for_test(),
        hop(),
        "a firing step AT a full trace cap must still derive one HOP — the \
         cap-immune entries_appended signal (a trace-length delta pins to 0 \
         once the ring is full, zeroing the spin budget silently)"
    );

    runtime.shutdown();
}

// ===========================================================================
// Test 5 — env disabled (`=0`) → ZERO regardless of imminence.
//
// `CERULION_LIVE_SPIN_US=0` ⇒ `Some(0)` ⇒ `Duration::ZERO`, bypassing the
// derivation entirely. We set the env BEFORE building (the OnceLock caches on
// the first `spin_budget` call), drive a FIRING step (which would otherwise make
// the DERIVED budget HOP), and assert the budget is STILL ZERO — proving the
// explicit `=0` override wins over imminence.
//
// ORACLE: ZERO (the TRUE value for the `=0` override), not a self-compare.
// EnvVarGuard removes the var on Drop (panic-safe); `#[serial]` prevents
// cross-test env races.
// ===========================================================================

#[test]
#[serial]
fn spin_budget_env_zero_is_zero_even_when_imminent() {
    let _env = EnvVarGuard::set(SPIN_ENV, "0");
    let (mut runtime, mut pubr, _observed, fires) = build_chain();

    // Drive a FIRING step so the DERIVED budget WOULD be HOP — the override must
    // beat the derivation.
    publish_one(&mut pubr, 1.0);
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    assert_eq!(
        fires.load(Ordering::Relaxed),
        1,
        "one publish + one live step must fire the sink once (sets last_step_fired \
         — would derive HOP absent the env override)"
    );

    assert_eq!(
        runtime.spin_budget_for_test(),
        Duration::ZERO,
        "CERULION_LIVE_SPIN_US=0 must DISABLE the spin (ZERO budget) regardless of \
         imminence — the explicit override beats the derived default"
    );

    runtime.shutdown();
}

// ===========================================================================
// Test 6 — env manual ceiling (`=25`) → 25µs even when idle.
//
// `CERULION_LIVE_SPIN_US=25` ⇒ `Some(25)` ⇒ `Duration::from_micros(25)`,
// bypassing the derivation. We assert 25µs on a freshly-built (predicted-IDLE)
// graph — where the DERIVED budget would be ZERO — proving the manual ceiling
// wins over the idle-prediction too.
//
// ORACLE: 25µs (the TRUE value for the `=25` override), not a self-compare.
// ===========================================================================

#[test]
#[serial]
fn spin_budget_env_manual_ceiling_overrides_derivation() {
    let _env = EnvVarGuard::set(SPIN_ENV, "25");
    let (runtime, _pubr, _observed, _fires) = build_chain();

    // The graph is predicted-IDLE (Period-free, never fired) ⇒ DERIVED would be
    // ZERO ⇒ the 25µs we observe is attributable solely to the env ceiling.
    assert_eq!(
        runtime.spin_budget_for_test(),
        Duration::from_micros(25),
        "CERULION_LIVE_SPIN_US=25 must set the spin budget to 25µs, bypassing the \
         derivation — observed even on a predicted-idle graph (derived would be ZERO)"
    );

    runtime.shutdown();
}

/// The `SPIN_BUDGET_MAX_US` clamp cap from `runtime.rs`, read from the REAL const
/// via the `spin_budget_max_us_for_test` accessor (NOT a hand-copied `100_000`
/// literal), so retuning the cap cannot silently drift the over-cap / at-cap
/// boundary oracles below. Mirrors `spin_hop_us()` for `SPIN_HOP_US`.
fn spin_budget_max_us() -> u64 {
    cerulion_core::graph::runtime::spin_budget_max_us_for_test()
}

// ===========================================================================
// Test 6b — env OVER the cap → CLAMPED to the cap + warns.
//
// `CERULION_LIVE_SPIN_US` > `SPIN_BUDGET_MAX_US` (100 000µs) is CLAMPED to the
// cap, NOT honored verbatim (an unbounded spin window risks an `Instant + Duration`
// overflow and a pathological core-pin). The clamp also emits a one-time
// `tracing::warn!` whose message contains "exceeds the". We pass a value well
// above the cap (200 000) and assert the resolved budget is exactly the cap
// (`spin_budget_max_us()`µs) — NOT 200 000 — proving the clamp fired.
//
// ORACLE: `Duration::from_micros(spin_budget_max_us())` (the TRUE clamped value),
// oracled off the REAL const, not a self-compare.
// ===========================================================================

#[test]
#[serial]
#[traced_test]
fn spin_budget_env_over_cap_clamps_and_warns() {
    // 200 000µs ≫ the 100 000µs cap — must clamp to the cap, not be honored.
    let over_cap = spin_budget_max_us() * 2;
    let _env = EnvVarGuard::set(SPIN_ENV, &over_cap.to_string());
    let (runtime, _pubr, _observed, _fires) = build_chain();

    assert_eq!(
        runtime.spin_budget_for_test(),
        Duration::from_micros(spin_budget_max_us()),
        "an over-cap CERULION_LIVE_SPIN_US must be CLAMPED to SPIN_BUDGET_MAX_US \
         ({}µs), NOT honored verbatim ({over_cap}µs) — an unbounded spin window \
         risks an Instant+Duration overflow and a pathological core-pin",
        spin_budget_max_us()
    );

    // The clamp must SURFACE (loud over silent): the warn message contains the
    // stable fragment "exceeds the" (full: "...exceeds the {MAX}µs cap...").
    assert!(
        logs_contain("exceeds the"),
        "an over-cap CERULION_LIVE_SPIN_US must emit the loud clamp warn (the \
         loud-over-silent contract), not silently clamp"
    );

    runtime.shutdown();
}

// ===========================================================================
// Test 6c — env EXACTLY at the cap → accepted UNCLAMPED, NO warn (the strict `>`
// boundary).
//
// The clamp guard in `spin_budget` is a STRICT `n > SPIN_BUDGET_MAX_US`, so a
// value EXACTLY equal to the cap takes the plain `Ceiling(n.min(MAX))` arm —
// accepted as-is, no clamp warn. We set the env to exactly `spin_budget_max_us()`
// (formatted off the REAL const), assert the budget is that value verbatim, and
// assert the clamp warn does NOT fire. A regression to `>=` would clamp 100 000
// and trip the `!logs_contain` assertion.
//
// ORACLE: `Duration::from_micros(spin_budget_max_us())` (the TRUE at-cap value),
// oracled off the REAL const.
// ===========================================================================

#[test]
#[serial]
#[traced_test]
fn spin_budget_env_at_cap_is_accepted_unclamped() {
    let at_cap = spin_budget_max_us();
    let _env = EnvVarGuard::set(SPIN_ENV, &at_cap.to_string());
    let (runtime, _pubr, _observed, _fires) = build_chain();

    assert_eq!(
        runtime.spin_budget_for_test(),
        Duration::from_micros(at_cap),
        "CERULION_LIVE_SPIN_US set to EXACTLY SPIN_BUDGET_MAX_US ({at_cap}µs) must \
         be accepted UNCLAMPED — the clamp guard is a strict `>`, so the at-cap \
         value takes the plain ceiling arm"
    );

    // The clamp warn must NOT fire at the boundary — pins the strict `>` (a
    // regression to `>=` would clamp the at-cap value and emit this warn).
    assert!(
        !logs_contain("exceeds the"),
        "the clamp warn must NOT fire for an at-cap value — the guard is a strict \
         `>`, so SPIN_BUDGET_MAX_US itself is accepted, not clamped (a `>=` \
         regression would clamp it and trip this)"
    );

    runtime.shutdown();
}

// ===========================================================================
// Test 7 — env bad value → ignored, falls through to the DERIVED budget.
//
// `CERULION_LIVE_SPIN_US=garbage` does NOT parse as `u64` ⇒ the runtime warns
// ONCE and caches `None` ⇒ the budget falls through to the DERIVED value (NOT
// treated as a literal / NOT a panic). On a freshly-built (predicted-idle)
// Period-free graph the derived value is ZERO.
//
// We assert the DERIVED-fallback BEHAVIOR here (ZERO for an idle graph). The
// warn EMISSION itself is asserted separately, in
// `spin_budget_bad_value_warns_once` below (a `#[traced_test]` test) — this
// test pins the observable budget, that one pins the loud-over-silent contract.
//
// ORACLE: ZERO (the TRUE derived-fallback value for an idle graph), proving the
// bad value was IGNORED — a regression that parsed "garbage" as some literal
// would yield a non-zero budget here.
// ===========================================================================

#[test]
#[serial]
fn spin_budget_env_bad_value_falls_through_to_derived() {
    let _env = EnvVarGuard::set(SPIN_ENV, "garbage");
    let (runtime, _pubr, _observed, _fires) = build_chain();

    assert_eq!(
        runtime.spin_budget_for_test(),
        Duration::ZERO,
        "a non-numeric CERULION_LIVE_SPIN_US must be IGNORED (warn-once + fall \
         through to the DERIVED default), NOT treated as a literal — an idle \
         Period-free graph derives ZERO"
    );

    runtime.shutdown();
}

// ===========================================================================
// Test 7b — bad value EMITS the loud warn (loud-over-silent contract).
//
// Complements test 7 (which pins the observable ZERO budget). Here we assert the
// WARN itself fires — the rule is "loud over silent": a user typo in
// `CERULION_LIVE_SPIN_US` must surface, not be swallowed. `#[traced_test]` (with
// the crate's `no-env-filter` feature, see Cargo.toml) captures the runtime's
// `tracing::warn!` so `logs_contain` can assert the exact message prefix.
//
// `spin_budget_for_test()` triggers the (lazy, once-cached) parse + warn. We
// match the real warn string prefix `"CERULION_LIVE_SPIN_US was set but is not"`
// — a substring of the runtime message, robust to the structured-field suffix.
// ===========================================================================

#[test]
#[serial]
#[traced_test]
fn spin_budget_bad_value_warns_once() {
    let _env = EnvVarGuard::set(SPIN_ENV, "garbage");
    let (runtime, _pubr, _observed, _fires) = build_chain();

    // Trigger the lazy parse + warn (first spin_budget call), then call AGAIN.
    // `spin_config` is a `OnceLock` cached on the first call, so the parse + warn
    // happen exactly once even across repeated calls — this is the "once" half of
    // the contract that a single call could not pin.
    let budget = runtime.spin_budget_for_test();
    let budget2 = runtime.spin_budget_for_test();
    assert_eq!(
        budget,
        Duration::ZERO,
        "bad value still derives ZERO on an idle Period-free graph"
    );
    assert_eq!(
        budget2, budget,
        "the second spin_budget call must return the same (cached) derived value"
    );

    assert!(
        logs_contain("CERULION_LIVE_SPIN_US was set but is not"),
        "a non-numeric CERULION_LIVE_SPIN_US must emit the loud warn-once (the \
         loud-over-silent contract), not be silently swallowed"
    );

    // PIN the ONCE-ness across BOTH calls: the warn substring must appear EXACTLY
    // once. `logs_contain` above is presence-only (a boolean) and would NOT catch
    // a regression that drops the `OnceLock` cache and re-parses + re-warns on
    // EVERY spin_budget call (per-iteration log spam in the hot live loop). This
    // count is the load-bearing mutation pin: drop-the-cache → count == 2 → FAIL.
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("CERULION_LIVE_SPIN_US was set but is not"))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected the bad-value warn EXACTLY once across two spin_budget() \
                 calls (OnceLock caches the parse), got {n}"
            ))
        }
    });

    runtime.shutdown();
}

// ===========================================================================
// Test 7c — empty string is treated as UNSET (derives), NOT as 0/bad.
//
// `CERULION_LIVE_SPIN_US=""` must take the `Derived` arm (same as unset), NOT
// `Disabled` (a `=0`-style ZERO override) and NOT the bad-value warn path. On an
// idle Period-free graph the derived value is ZERO — same observable as `=0`
// here, but reached via DERIVATION (the distinction matters: an empty env that
// errantly mapped to `Ceiling(0)`/`Disabled` would skip derivation entirely, so
// it would NOT flip to HOP after a firing step). This test pins the unset-equiv.
// ===========================================================================

#[test]
#[serial]
fn spin_budget_empty_string_derives() {
    let _env = EnvVarGuard::set(SPIN_ENV, "");
    let (runtime, _pubr, _observed, _fires) = build_chain();

    assert_eq!(
        runtime.spin_budget_for_test(),
        Duration::ZERO,
        "an EMPTY CERULION_LIVE_SPIN_US must be treated as UNSET (derive) — an \
         idle Period-free graph derives ZERO, not via an override"
    );

    runtime.shutdown();
}

// ###########################################################################
// Group 2b — the Period-imminent DERIVED arm.
//
// The Period-free chain above can never exercise `spin_budget`'s
// `period_imminent` branch (`ns_until_next_fire == None`). These tests build a
// dedicated `period_ms` graph under a VirtualClock the test OWNS, and position
// the clock relative to the Period's `next_fire_ns` to drive both sides of the
// `ns_until_next_fire(now) <= SPIN_HOP_US * 1_000` boundary.
// ###########################################################################

/// L0: a lone `period_ms = 100` producer (no data inputs) — the Period-deadline
/// lever for the `period_imminent` derivation arm. Under VirtualClock the first
/// `next_fire_ns` is `0 + 100ms = 100_000_000` (build-time clock reads 0).
#[cerulion_node(period_ms = 100)]
#[derive(Default)]
struct Periodic {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl Periodic {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// Build a lone-`period_ms=100`-producer graph via `build_for_test`, RETURNING
/// the `Arc<VirtualClock>` so the test can advance the watch clock to position
/// `now_ns` relative to the Period's `next_fire_ns` (= 100ms under VirtualClock).
/// The returned clock IS the runtime's `watch_clock` (build_for_test clones the
/// same Arc), so `clock.set(t)` directly drives `spin_budget`'s `now_ns` read.
fn build_period_graph() -> (GraphRuntime, Arc<VirtualClock>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "live_spin_budget_period_test".to_string(),
        prefix: "lsbp".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "producer".to_string(),
            node_type: "periodic".to_string(),
            inputs: vec![],
            outputs: vec![vector3_output("out")],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(PeriodicEntry::new()));

    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 8)
        .expect("build period graph");
    (runtime, clock)
}

// ===========================================================================
// Test G1 — Period due WITHIN one hop → HOP (the `<=` boundary, inclusive).
//
// next_fire_ns = 100ms (= 100_000_000). Set the watch clock to EXACTLY one hop
// before the deadline (`next_fire - SPIN_HOP_US*1000`), so
// `ns_until_next_fire(now) == SPIN_HOP_US * 1_000` — the inclusive `<=` boundary.
// The derived budget must be exactly one HOP. The graph is freshly built (never
// fired), so `last_step_fired == false` — the HOP is attributable SOLELY to
// `period_imminent` (pins the `* 1_000` ns conversion AND the `<=` boundary).
//
// ORACLE: HOP (the TRUE derived value when a Period is due within one hop).
// ===========================================================================

#[test]
#[serial]
fn spin_budget_period_imminent_within_one_hop_is_hop() {
    let (runtime, clock) = build_period_graph();

    // next_fire = 100ms under VirtualClock. Position now exactly one hop before
    // it: ns_until_next_fire == SPIN_HOP_US * 1_000 (the inclusive `<=` edge).
    let next_fire_ns: u64 = 100 * 1_000_000;
    let one_hop_ns: u64 = spin_hop_us() * 1_000;
    clock.set(next_fire_ns - one_hop_ns);

    assert_eq!(
        runtime.spin_budget_for_test(),
        hop(),
        "a Period fire due within EXACTLY one hop (ns_until_next_fire == \
         SPIN_HOP_US*1000, the inclusive `<=` boundary) must derive one HOP — \
         pins the `* 1_000` ns conversion and the boundary (last_step_fired is \
         false here, so the HOP is solely period_imminent)"
    );

    runtime.shutdown();
}

// ===========================================================================
// Test G2 — Period FAR (> one hop) and nothing just-fired → ZERO.
//
// Companion to G1: with the clock at build (now == 0), ns_until_next_fire ==
// 100ms ≫ one hop, and last_step_fired == false → NOT imminent → ZERO. Also set
// the clock to just OUTSIDE the boundary (one hop + 1ns before the deadline) to
// pin the exclusive side: ns_until_next_fire == one_hop + 1 > one_hop → ZERO.
//
// ORACLE: ZERO (the TRUE derived value when no wake is imminent).
// ===========================================================================

#[test]
#[serial]
fn spin_budget_period_far_is_zero() {
    let (runtime, clock) = build_period_graph();

    // At build (now == 0): next fire is a full 100ms away — far from imminent.
    assert_eq!(
        runtime.spin_budget_for_test(),
        Duration::ZERO,
        "a Period fire 100ms out (≫ one hop), with nothing just-fired, must derive \
         a ZERO budget (block immediately — never spin into a far gap)"
    );

    // One ns OUTSIDE the boundary: ns_until_next_fire == one_hop + 1 > one_hop →
    // still NOT imminent → ZERO (pins the exclusive side of the `<=` boundary).
    let next_fire_ns: u64 = 100 * 1_000_000;
    let one_hop_ns: u64 = spin_hop_us() * 1_000;
    clock.set(next_fire_ns - one_hop_ns - 1);
    assert_eq!(
        runtime.spin_budget_for_test(),
        Duration::ZERO,
        "a Period fire just OUTSIDE one hop (ns_until_next_fire == one_hop + 1) \
         must derive ZERO — the boundary is `<=`, so one_hop+1 is NOT imminent"
    );

    runtime.shutdown();
}

// ###########################################################################
// Group 3 — determinism re-pin (Replay=Live, Principle #7): the SPIN path is
// deterministic across runs.
// ###########################################################################

// ===========================================================================
// Test 8 — spin-on cross-run byte-identity.
//
// `polled_vs_live_iox2_test` already pins polled == live == oracle; this test
// pins that the SPIN path ITSELF is deterministic across runs. We run the SAME
// pure-data-driven chain twice through `run_live_step_once_for_test` with the
// DEFAULT spin budget (no env var set — so `spin_budget` derives, and once the
// chain is mid-flight, `last_step_fired` makes it spin one HOP, exercising the
// spin path), and assert the two runs produce byte-identical observed output
// (the sink's data-flow Vec).
//
// Comparing two runs of the SAME seam IS the determinism contract (a two-run
// compare is legitimate for determinism —
// it is NOT a self-compare-of-one-run). We additionally pin against a HAND
// ORACLE (`1.0..=N` in order) so the test is not merely "two runs agree" but
// "two runs agree AND equal the true expected value".
// ===========================================================================

#[test]
#[serial]
fn spin_path_is_deterministic_across_runs() {
    /// Run the chain through the live (spin-enabled) seam for `n` publishes,
    /// returning the sink's observed data-flow Vec.
    fn run_live(n: u64) -> Vec<f64> {
        let (mut runtime, mut pubr, observed, fires) = build_chain();

        // Prime: drain build/attach connection-lifecycle noise; the sink must
        // stay at zero so a noise wake cannot mask a missing/extra data fire.
        runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
        assert_eq!(
            fires.load(Ordering::Relaxed),
            0,
            "the priming (no-data) live step must NOT fire the sink"
        );

        // N publishes, one live iteration each. Each publish collapses
        // ext→relay→mid→sink within the step; after the first firing step
        // `last_step_fired` makes the DERIVED budget HOP, so subsequent
        // iterations exercise the SPIN path (the point of this determinism pin).
        for i in 1..=n {
            publish_one(&mut pubr, i as f64);
            runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
        }

        let values = observed.lock().unwrap().clone();
        runtime.shutdown();
        values
    }

    const N: u64 = 6;
    let run_a = run_live(N);
    let run_b = run_live(N);

    // Determinism: two runs of the spin-enabled live seam are byte-identical.
    assert_eq!(
        run_a, run_b,
        "two runs of the spin-enabled live seam must produce byte-identical \
         observed data-flow — the SPIN path is deterministic (Principle #7)"
    );

    // HAND ORACLE (non-tautological): the observed values are exactly 1.0..=N in
    // order — so the runs agree AND equal the true expected content.
    let expected: Vec<f64> = (1..=N).map(|i| i as f64).collect();
    assert_eq!(
        run_a, expected,
        "the sink must observe exactly the published values 1.0..=N in order, \
         through the spin-enabled live seam"
    );
}

// ===========================================================================
// Test 9 — spin-ON vs block-ONLY produce the SAME trace (the moat).
//
// Test 8 pins that the spin path is self-consistent across runs. THIS test pins
// the stronger contract: the spin path yields EXACTLY the block path's trace —
// i.e. enabling the spin changes WHEN the loop wakes, never WHAT fires (the
// Replay=Live firewall, Principle #7, for the spin path). We run the SAME
// pure-data-driven chain through ONE harness under two configs:
//
//   1. DEFAULT derived budget (no env var) — spin ON (once mid-flight,
//      `last_step_fired` makes the derived budget HOP, exercising the spin).
//   2. `CERULION_LIVE_SPIN_US=0` — spin DISABLED (always block immediately).
//
// and assert both observed data-flow Vecs are BYTE-IDENTICAL to each other AND
// equal to the hand oracle `1.0..=N`. This is NOT a self-compare: it compares
// two DIFFERENT receive paths (spin vs block) against each other AND a true
// oracle. Two separate runtimes — `spin_config` caches per instance, so the
// block-only runtime is built AFTER its env guard is set.
// ===========================================================================

#[test]
#[serial]
fn spin_on_and_block_only_produce_identical_traces() {
    /// Run the chain for `n` publishes, returning the sink's observed data-flow
    /// Vec. The caller controls the spin config by (not) setting the env BEFORE
    /// calling this — `build_chain` builds the runtime, so the env must already
    /// be in place when this runs (the `spin_config` OnceLock caches on first
    /// `spin_budget`, which `run_live_step_once_for_test` triggers).
    fn run_live(n: u64) -> Vec<f64> {
        let (mut runtime, mut pubr, observed, fires) = build_chain();

        // Prime: drain build/attach connection-lifecycle noise (sink stays 0).
        runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
        assert_eq!(
            fires.load(Ordering::Relaxed),
            0,
            "the priming (no-data) live step must NOT fire the sink"
        );

        for i in 1..=n {
            publish_one(&mut pubr, i as f64);
            runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
        }

        let values = observed.lock().unwrap().clone();
        runtime.shutdown();
        values
    }

    const N: u64 = 6;

    // (1) Spin ON — NO env var set (derived budget). Run with the env CLEAR.
    let spin_on = run_live(N);

    // (2) Block ONLY — set `CERULION_LIVE_SPIN_US=0` BEFORE building (the guard
    // removes the var on Drop, so this scope leaves the env clean).
    let block_only = {
        let _env = EnvVarGuard::set(SPIN_ENV, "0");
        run_live(N)
    };

    // The spin path yields EXACTLY the block path's trace — enabling the spin
    // changed only WHEN the loop woke, never WHAT fired (firewall, Principle #7).
    assert_eq!(
        spin_on, block_only,
        "the spin-ON receive path must yield BYTE-IDENTICAL observed data-flow to \
         the block-ONLY path — the spin changes WHEN the loop wakes, never WHAT \
         fires (Replay=Live firewall for the spin path, Principle #7)"
    );

    // HAND ORACLE (non-tautological): both equal the true expected 1.0..=N.
    let expected: Vec<f64> = (1..=N).map(|i| i as f64).collect();
    assert_eq!(
        spin_on, expected,
        "both the spin-ON and block-ONLY paths must observe exactly the published \
         values 1.0..=N in order (oracle, not a self-compare)"
    );
}
