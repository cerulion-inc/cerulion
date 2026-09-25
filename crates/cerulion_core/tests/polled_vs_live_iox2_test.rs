// SPDX-License-Identifier: AGPL-3.0-only
//! The REPLAY=LIVE FIREWALL pin (Principle #7) — the POLLED
//! `GraphRuntime::step()` seam and the LIVE WaitSet seam
//! (`run_live_step_once_for_test`) must fire the SAME nodes in the SAME
//! order and flow the SAME data for the same external-publish schedule.
//!
//! The firewall (`runtime.rs` ~2413): `run_live` only changes WHEN `step()`
//! runs, never its body — `step`/`drain_level` remains the sole, byte-identical
//! firing path. This file is the end-to-end proof of that invariant over real
//! iceoryx2: it drives ONE multi-level pure-data-driven chain through BOTH
//! seams and asserts byte-identical fire sequences + data flow.
//!
//! ## The chain (NO Period nodes — pure external-data-driven)
//!
//! ```text
//! /pvl/ext (absolute external) ─▶ relay ─▶ mid ─▶ sink
//!   (L0 relay's trigger; no in-graph     (L1)    (L2)
//!    producer → provisioned External)
//! ```
//!
//! `relay` / `mid` each `#[input(trigger)] inp` + `#[output] out`, forwarding
//! `out.x = inp.x`; `sink` reads `inp.x` and accumulates it into a shared
//! observed-value `Vec<f64>` (the data-flow oracle). A Period node would break
//! byte-identity: its fire COUNT diverges between the wall-clock live path (real
//! deltas) and the 1 ms-virtual poll path. Pure external-data firing is
//! deterministic in BOTH seams — each publish collapses to one same-step
//! relay→mid→sink fire (the within-step level collapse).
//!
//! ## Fire-sequence source of truth
//!
//! `runtime.trace()` — the scheduler's authoritative `TraceEntry` stream (the
//! replay oracle), NOT a node-self-recorded order. We compare the trace's
//! node-id sequence across seams AND against a hand-built oracle
//! (`["relay","mid","sink"] × N`), so test 1 is NOT a self-compare tautology.
//!
//! All tests `#[serial]` (the live loop builds an iceoryx2 WaitSet over the
//! process-global SHM singleton; `build_for_test` is otherwise per-test-SHM-root
//! parallel-safe, but the WaitSet singleton forces serial — matches
//! `waitset_live_loop_iox2_test`).
//!
//! ## Parked-seam firewall pin
//!
//! `parked_live_seam_is_byte_identical_to_oracle_and_unparked` extends the
//! firewall to the monitor-wait PARK: the live loop's blocking WaitSet wait is
//! replaced by a record-only shallow monitor-wait park when a `MonitorWaitPolicy`
//! is active. The park is a WAIT primitive — it changes only WHEN
//! `step()` runs, NEVER what fires. That test builds the SAME `chain_graph` via
//! the LIVE-path `build_for_test_with_policy` (a FORCED policy) and proves
//! park-on == park-off == the hand oracle `[relay,mid,sink] × N`, isolating the
//! park's effect with a park-OFF control on the SAME live build path (no
//! build-path confound). On Apple Silicon there is no real CPU
//! monitor-wait primitive, so the park degrades to a sleep-recheck; the
//! assertions are about FIRE SEQUENCE + DATA FLOW (never timing), so they hold
//! regardless of whether a real hardware park ran — the real CPU-park latency is
//! measured separately on WAITPKG/WFE hardware.
//!
//! ## The STRENGTHENED firewall (full trace incl. `fire_time_ns`)
//!
//! Tests 1-6 above compare the live seam against the polled seam on the fire
//! SEQUENCE + data flow only, EXCLUDING `fire_time_ns`. That exclusion is
//! LEGITIMATE for the `Mode::Live` (and parked) seams: those build via
//! `build_for_test` (the wall-driven live path, `live_gating_quantum == None`),
//! so the live loop advances the `VirtualClock` by REAL wall-clock deltas between
//! iterations → absolute `fire_time_ns` legitimately differs from the fixed-1ms
//! polled `step()`.
//!
//! `polled_and_barrier_live_produce_byte_identical_full_trace` (test 7) drops
//! that exclusion by adding a THIRD seam — `Mode::LiveBarrier` — built via
//! `GraphRuntime::build_for_test_barrier`, the DETERMINISTIC-LIVE path. There the
//! live loop advances the scheduler's `Barrier` GATING clock by a fixed logical
//! QUANTUM each step (NOT wall elapsed). This chain is pure data-driven (NO Period
//! node ⇒ `tightest_timing_ns() == None`), so the quantum is the 1ms FALLBACK —
//! which EQUALS the polled `step(Duration::from_millis(1))` delta. So the Barrier
//! gating clock and the polled clock advance in lockstep (1ms/step from a fresh
//! `VirtualClock` at 0), making `fire_time_ns` byte-identical step-for-step.
//! Test 7 therefore asserts the FULL `Vec<TraceEntry>` is byte-identical
//! (`node_id` + `step` + `global_level` AND `fire_time_ns`; only the wall-time
//! `duration_ns` stays excluded, by `TraceEntry`'s hand-written `PartialEq`, since
//! it is non-replayable). It anchors BOTH dimensions to HAND ORACLES (sequence:
//! `expected_fire_sequence`; absolute times: `expected_fire_times_ns`) so the pin
//! is not a poll-vs-barrier self-compare. This is the deeper Principle #7 proof:
//! the deterministic-live seam matches the polled seam not just on WHAT/WHEN fires
//! relative to each other, but on the ABSOLUTE logical fire time.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::scheduler::TraceEntry;
use cerulion_core::transport::TransportManager;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// The absolute external trigger topic for `relay`. No in-graph producer, so the
/// graph provisions it as `External` (buffer-ceiling only, no single-writer cap)
/// and an out-of-graph publisher attaches freely (matches `waitset_live_loop`'s
/// `EXT_TOPIC`).
const EXT_TOPIC: &str = "/pvl/ext";

/// A short wake timeout: long enough that a published event wakes the reactor
/// well before it elapses, short enough that a no-data iteration returns quickly
/// (matches `waitset_live_loop_iox2_test::WAKE_TIMEOUT`).
const WAKE_TIMEOUT: Duration = Duration::from_millis(150);

// ===========================================================================
// Nodes — a 3-level forward chain, all data-trigger (NO Period anywhere).
// ===========================================================================

/// L0: triggers on the absolute external `/pvl/ext`, forwards `inp.x` to `out`.
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

/// A data-trigger sink with
/// `sample(2)` backpressure. `sample(N)`'s read-decimation gates the FIRE rate,
/// so the body-subscriber unification is INELIGIBLE — this consumer
/// keeps the legacy dual-subscriber path and so its live WaitSet wake source is
/// a `TriggerSubscriber::Ipc` (NOT the unified `ListenerOnly`). Co-locating it
/// with a plain (unified → `ListenerOnly`) trigger consumer on the SAME topic
/// puts BOTH `TriggerSubscriber` variants in one live source list, exercising
/// the `.map`-over-both-variants dispatch (which replaced `filter_map`).
/// Mirrors `multi_publisher_iox2_test::SampleTriggerConsumer`, replicated here
/// because test binaries are separate crates (the fixture is not importable).
#[cerulion_node]
#[derive(Default)]
struct SampleSink {
    #[input(trigger, backpressure = sample(2))]
    inp: Vector3,
    observed: Arc<Mutex<Vec<f64>>>,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl SampleSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.observed.lock().unwrap().push(self.inp.x);
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

// ===========================================================================
// Graph construction (hand-built, like waitset_live_loop's `live_graph`).
// ===========================================================================

/// An `OutputDef` for a `geometry_msgs/Vector3` producer with all resolution
/// knobs at their defaults (derived topic, runtime-resolved slice len, volatile
/// history) — the shape `build_for_test` requires.
fn vector3_output(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "geometry_msgs/Vector3".to_string(),
        max_slice_len: None,
        topic: None,
        history_size: 0,
    }
}

/// Build the 3-level forward chain config + factories. `observed` / `fires` are
/// the sink's shared data-flow + fire oracles.
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
        identity: "polled_vs_live_test".to_string(),
        prefix: "pvl".to_string(),
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

/// Which seam drives the runtime.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// `GraphRuntime::step(1ms)` — the deterministic polled path. Built via
    /// `build_for_test`.
    Polled,
    /// `run_live_step_once_for_test(WAKE_TIMEOUT)` — the live WaitSet path, built
    /// via `build_for_test` (`live_gating_quantum == None`), so the live loop
    /// advances the `VirtualClock` by WALL deltas → non-deterministic absolute
    /// `fire_time_ns` (tests 1-6 compare the SEQUENCE only).
    Live,
    /// `run_live_step_once_for_test(WAKE_TIMEOUT)` like
    /// `Live`, but built via `build_for_test_barrier` — the DETERMINISTIC-LIVE
    /// path. The live loop advances the scheduler's `Barrier` gating clock by a
    /// fixed logical QUANTUM each step (the 1ms `tightest_timing_ns` fallback for
    /// this pure-data-driven chain), which EQUALS the polled `step(1ms)` delta →
    /// `fire_time_ns` is replay-deterministic AND byte-identical to `Polled`. The
    /// ONLY difference from `Live` is the build path (Barrier gating clock vs
    /// wall-advanced virtual clock); the drive loop is identical.
    LiveBarrier,
}

/// Drive one runtime iteration through the selected seam. `Live` and
/// `LiveBarrier` use the SAME live drive (`run_live_step_once_for_test`); they
/// differ only in how the runtime was BUILT (see [`build_runtime_for_mode`]).
fn drive(mode: Mode, runtime: &mut GraphRuntime) {
    match mode {
        Mode::Polled => runtime.step(Duration::from_millis(1)),
        Mode::Live | Mode::LiveBarrier => runtime.run_live_step_once_for_test(WAKE_TIMEOUT),
    }
}

/// Build the runtime for `mode`. `Polled`/`Live` route through `build_for_test`
/// (`Live` then advances the `VirtualClock` by WALL deltas in its live drive);
/// `LiveBarrier` routes through `build_for_test_barrier` — the DETERMINISTIC-LIVE
/// path whose live drive advances the `Barrier` gating clock by the fixed 1ms
/// quantum (== the polled `step(1ms)` delta for this pure-data-driven chain), so
/// its `fire_time_ns` is byte-identical to the polled seam. The factory map +
/// `subscriber_buffer_size` (8) are identical across modes; ONLY the clock
/// model differs.
fn build_runtime_for_mode(
    mode: Mode,
    config: GraphConfig,
    factories: IndexMap<String, Box<dyn NodeEntry>>,
    clock: Arc<VirtualClock>,
) -> GraphRuntime {
    match mode {
        Mode::Polled | Mode::Live => GraphRuntime::build_for_test(config, factories, clock, 8)
            .expect("build polled-vs-live chain graph"),
        Mode::LiveBarrier => GraphRuntime::build_for_test_barrier(config, factories, clock, 8)
            .expect("build deterministic-live (barrier) chain graph"),
    }
}

/// Build a FRESH chain for `mode`, attach an external publisher on `/pvl/ext`,
/// then publish N frames carrying `x = 1.0..=N`, driving ONE iteration of the
/// selected seam after each publish.
///
/// Returns `(trace, observed_values)`:
/// - `trace` — the FULL `Vec<TraceEntry>` from `runtime.trace()` (the scheduler's
///   authoritative replay trace). The owned `Vec` includes `fire_time_ns` (and
///   `step`/`global_level`); only the wall-time `duration_ns` is excluded by
///   `TraceEntry`'s hand-written `PartialEq`. [`run_chain`] projects this to the
///   node-id `Vec<String>` for the SEQUENCE-only tests; the full-trace test 7
///   compares the owned `Vec<TraceEntry>` directly.
/// - `observed_values` — the sink's accumulated `Vec<f64>` (the data-flow oracle).
///
/// ALL modes run under a fresh `VirtualClock` (the live seams too — see
/// `waitset_live_loop`). The build is mode-dispatched in [`build_runtime_for_mode`]
/// (`Polled`/`Live` → `build_for_test`; `LiveBarrier` → `build_for_test_barrier`).
/// The pre-loop drive is symmetric across modes: ONE seam iteration with no data
/// published. For the live seams this PRIMES — it drains the build/attach
/// connection-lifecycle noise the trigger inputs' `Listener`s multiplex alongside
/// data events (see `waitset_live_loop`'s `prime`); for `Polled`, `step()` drains
/// that noise without firing anyway. We assert zero sink fires after the pre-loop
/// drive in EVERY mode so a connection-noise wake can never mask a missing/extra
/// data fire. (That pre-loop step still ADVANCES the gating clock by one quantum —
/// 1ms — in every mode, so the first DATA fire lands at gating time 2·1ms; this is
/// what makes the polled and `LiveBarrier` absolute `fire_time_ns` align — see
/// [`expected_fire_times_ns`].)
fn run_chain_trace(mode: Mode, n: u64) -> (Vec<TraceEntry>, Vec<f64>) {
    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = chain_graph(Arc::clone(&observed), Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = build_runtime_for_mode(mode, config, factories, clock);

    // Attach the out-of-graph external publisher on the absolute trigger topic.
    let mut pubr = {
        let mgr: &Arc<TransportManager> = runtime.test_transport().expect("test transport parked");
        mgr.create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("external publisher must attach to /pvl/ext")
    };

    // Pre-loop drive (symmetric): one seam iteration, NO publish. Drains
    // connection-lifecycle noise so the sink stays at zero fires either way.
    drive(mode, &mut runtime);
    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "the pre-loop drive (no data published) must NOT fire the sink in {mode:?} \
         mode — a connection-noise wake calls step(), but drain_level finds no data"
    );

    // N publishes, one seam iteration each. One step()/live_step drains all 3
    // levels in order (the within-step collapse), so each publish flows
    // relay→mid→sink in ONE drive.
    for i in 1..=n {
        publish_one(&mut pubr, i as f64);
        drive(mode, &mut runtime);
    }

    let trace = runtime.trace().to_vec();
    let observed_values = observed.lock().unwrap().clone();
    runtime.shutdown();
    (trace, observed_values)
}

/// The node-id fire SEQUENCE projection of [`run_chain_trace`] — the view tests
/// 1-6 use (they compare the `Vec<String>` sequence + data flow, NOT absolute
/// `fire_time_ns`, since the wall-advanced `Live` seam's fire times are
/// non-deterministic). Semantics:
/// `runtime.trace()` mapped to fired node-ids.
fn run_chain(mode: Mode, n: u64) -> (Vec<String>, Vec<f64>) {
    let (trace, observed_values) = run_chain_trace(mode, n);
    let fire_sequence: Vec<String> = trace.iter().map(|e| e.node_id.to_string()).collect();
    (fire_sequence, observed_values)
}

/// The PARKED-seam sibling of [`run_chain`]. Builds the SAME
/// `chain_graph` via the LIVE-path `build_for_test_with_policy` with a FORCED
/// [`cerulion_core::MonitorWaitPolicy`], then drives it LIVE
/// (`run_live_step_once_for_test(WAKE_TIMEOUT)`) for `n` publishes — mirroring
/// [`run_chain`]'s structure EXACTLY (same pre-loop drive + zero-fire assertion,
/// same N-publish loop, same `(fire_sequence, observed_values)` return shape).
///
/// The ONLY difference from `run_chain(Mode::Live, n)` is the BUILDER: this one
/// forces a `MonitorWaitPolicy` so the live loop replaces its blocking WaitSet
/// wait with the record-only monitor-wait park. On Apple Silicon
/// the park has no real CPU primitive, so it degrades to a sleep-recheck —
/// but the firewall holds regardless: the park changes only WHEN `step()` runs,
/// never WHAT fires, so the fire sequence + data flow are decided entirely inside
/// `step()`/`drain_level` (the sole, byte-identical firing path).
///
/// With `policy.doorbell` ON, the producers' owned `Doorbell`s are opened and the
/// consumer `DoorbellRegistry` is built; without the primitive the SHM ring is a no-op stub,
/// so data still flows through real iceoryx2 and the listener poll wakes the loop.
fn run_chain_parked(policy: cerulion_core::MonitorWaitPolicy, n: u64) -> (Vec<String>, Vec<f64>) {
    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = chain_graph(Arc::clone(&observed), Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test_with_policy(config, factories, clock, 8, policy)
        .expect("build parked polled-vs-live chain graph");

    // Attach the out-of-graph external publisher on the absolute trigger topic.
    let mut pubr = {
        let mgr: &Arc<TransportManager> = runtime.test_transport().expect("test transport parked");
        mgr.create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("external publisher must attach to /pvl/ext")
    };

    // Pre-loop drive (same as `run_chain`'s Live arm): one LIVE iteration, NO
    // publish. Drains connection-lifecycle noise so the sink stays at zero fires.
    // Without the primitive the park's sleep-recheck fallback may sleep to ~WAKE_TIMEOUT
    // here — that is the expected no-data park cost, not a hang.
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "the pre-loop drive (no data published) must NOT fire the sink under the \
         monitor-wait park — a connection-noise wake calls step(), but drain_level \
         finds no data"
    );

    // N publishes, one LIVE iteration each — identical to `run_chain`'s loop. One
    // step()/live_step drains all 3 levels in order (the within-step
    // collapse), so each publish flows relay→mid→sink in ONE drive.
    for i in 1..=n {
        publish_one(&mut pubr, i as f64);
        runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    }

    let fire_sequence: Vec<String> = runtime
        .trace()
        .iter()
        .map(|e| e.node_id.to_string())
        .collect();
    let observed_values = observed.lock().unwrap().clone();
    runtime.shutdown();
    (fire_sequence, observed_values)
}

/// Publish exactly ONE `Vector3` frame with `x = x` onto `/pvl/ext` (the proxy
/// publishes on drop).
fn publish_one(pubr: &mut cerulion_core::CerulionPublisher, x: f64) {
    let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan");
    proxy.x = x;
    drop(proxy); // publish
}

/// The hand-built expected fire SEQUENCE for `n` publishes: each publish
/// collapses to one same-step `relay → mid → sink` fire, so the trace is
/// `["relay","mid","sink"]` repeated `n` times. This is the NON-tautological
/// oracle for test 1 (not a self-compare of two closure runs).
fn expected_fire_sequence(n: u64) -> Vec<String> {
    let mut seq = Vec::with_capacity(n as usize * 3);
    for _ in 0..n {
        seq.push("relay".to_string());
        seq.push("mid".to_string());
        seq.push("sink".to_string());
    }
    seq
}

/// The hand-built expected `fire_time_ns` stream for the
/// POLLED and `LiveBarrier` seams over `n` publishes — both step the gating clock
/// by a fixed 1ms quantum from a fresh `VirtualClock` (starts at 0;
/// `advance_by_recorded` returns the POST-advance value, so step `s` stamps
/// `s·1ms`, see `clock.rs`). The pre-loop drive is step 1 (clock 0→1ms, NO fire);
/// then publish `k` (1..=n) fires relay/mid/sink TOGETHER at the (k+1)-th step's
/// post-advance gating time `(k+1)·1ms` (the within-step level collapse
/// → all 3 share the one step's clock value). So the stream is `[(k+1)·1ms; 3]`
/// for `k` in `1..=n` (e.g. `[2ms,2ms,2ms, 3ms,3ms,3ms, ...]`).
///
/// This is the absolute-time HAND ORACLE that makes test 7's `fire_time_ns`
/// dimension non-tautological — NOT merely "polled == barrier" (a cross-build
/// self-compare) but "barrier == this hand-known schedule". It only holds because
/// the `LiveBarrier` quantum (1ms fallback, pure-data-driven chain) EQUALS the
/// polled `step(1ms)` delta — the equivalence test 7 documents and depends on.
fn expected_fire_times_ns(n: u64) -> Vec<u64> {
    const QUANTUM_NS: u64 = 1_000_000; // 1ms — the polled step delta AND the pure-data-driven 1ms quantum fallback
    let mut times = Vec::with_capacity(n as usize * 3);
    for k in 1..=n {
        let t = (k + 1) * QUANTUM_NS;
        times.push(t); // relay
        times.push(t); // mid
        times.push(t); // sink
    }
    times
}

// ===========================================================================
// Test 1 — the HEADLINE: polled and live are byte-identical (+ hand oracle).
// ===========================================================================

#[test]
#[serial]
fn polled_and_live_produce_byte_identical_fire_sequence() {
    const N: u64 = 6;
    let (poll_seq, poll_vals) = run_chain(Mode::Polled, N);
    let (live_seq, live_vals) = run_chain(Mode::Live, N);

    // The FIREWALL: the live WaitSet seam fires the SAME nodes in the SAME order
    // as the polled seam (byte-identical node-id fire SEQUENCE).
    assert_eq!(
        poll_seq, live_seq,
        "the live WaitSet seam must fire the SAME nodes in the SAME order as the \
         polled step() seam — replay=live firewall (Principle #7)"
    );
    // Byte-identical data-flow: each value 1..=N flows ext→relay→mid→sink
    // identically in both seams.
    assert_eq!(
        poll_vals, live_vals,
        "each published value must flow ext→relay→mid→sink identically in both \
         seams — byte-identical data-flow"
    );

    // HAND ORACLE (non-tautological): the absolute expected content, not a
    // self-compare. The fire sequence is `[relay, mid, sink] × N`; the observed
    // values are exactly `1.0..=N` in order.
    let expected_seq = expected_fire_sequence(N);
    assert_eq!(
        poll_seq, expected_seq,
        "fire sequence must be exactly [relay, mid, sink] × N — each publish \
         collapses to one same-step relay→mid→sink fire"
    );
    let expected_vals: Vec<f64> = (1..=N).map(|i| i as f64).collect();
    assert_eq!(
        poll_vals, expected_vals,
        "the sink must observe exactly the published values 1.0..=N in order"
    );

    // WHY `fire_time_ns` is NOT compared: the live seam advances the VirtualClock
    // by real WALL-CLOCK deltas (Instant::now() between iterations), while the
    // poll seam advances by the fixed 1 ms `step()` delta. So absolute
    // `fire_time_ns` legitimately DIFFERS by clock model — that is the ONE
    // legitimate live-vs-replay difference. The firewall invariant is the fire
    // SEQUENCE + data-flow, which step()/drain_level (the sole firing path) makes
    // identical — that is what is pinned here. (Within ONE collapsed step all 3
    // fires share that step's single clock value, so the grouped structure is
    // identical across seams; only the absolute base differs.)
}

// ===========================================================================
// Test 2 — same-seam determinism (Principle #7), independent of the cross-seam
// comparison. NOT a tautology relative to test 1: each side is two runs of the
// SAME seam (the legitimate determinism check), test 1 compares ACROSS seams.
// ===========================================================================

#[test]
#[serial]
fn fire_sequence_is_deterministic_across_runs() {
    const N: u64 = 6;

    // Live seam, twice: byte-identical fire sequence + data-flow.
    let (live_seq_a, live_vals_a) = run_chain(Mode::Live, N);
    let (live_seq_b, live_vals_b) = run_chain(Mode::Live, N);
    assert_eq!(
        (live_seq_a, live_vals_a),
        (live_seq_b, live_vals_b),
        "two live-seam runs must produce byte-identical (sequence, values) — \
         determinism within the live seam (Principle #7)"
    );

    // Polled seam, twice: byte-identical fire sequence + data-flow.
    let (poll_seq_a, poll_vals_a) = run_chain(Mode::Polled, N);
    let (poll_seq_b, poll_vals_b) = run_chain(Mode::Polled, N);
    assert_eq!(
        (poll_seq_a, poll_vals_a),
        (poll_seq_b, poll_vals_b),
        "two polled-seam runs must produce byte-identical (sequence, values) — \
         determinism within the polled seam (Principle #7)"
    );
}

// ===========================================================================
// Test 3 — no data, no fire: an empty drive fires nothing in either seam.
// ===========================================================================

#[test]
#[serial]
fn empty_drive_fires_nothing() {
    // With ZERO publishes (n = 0), `run_chain` performs only the pre-loop drive
    // and never publishes — the trace must be empty and the sink observes
    // nothing, in BOTH seams. The pre-loop assertion inside `run_chain` already
    // proves the sink does not fire; here we additionally pin the EMPTY trace
    // (no node fired at all, including relay/mid).
    let (poll_seq, poll_vals) = run_chain(Mode::Polled, 0);
    assert!(
        poll_seq.is_empty(),
        "no publish → the polled seam must record an EMPTY fire trace, got {poll_seq:?}"
    );
    assert!(
        poll_vals.is_empty(),
        "no publish → the sink must observe NOTHING in the polled seam"
    );

    let (live_seq, live_vals) = run_chain(Mode::Live, 0);
    assert!(
        live_seq.is_empty(),
        "no publish → the live seam must record an EMPTY fire trace, got {live_seq:?}"
    );
    assert!(
        live_vals.is_empty(),
        "no publish → the sink must observe NOTHING in the live seam"
    );
}

// ===========================================================================
// Test 4 — the live loop wakes PROMPTLY on a UNIFIED input.
//
// The chain's trigger inputs (`relay`/`mid`/`sink`'s `#[input(trigger)]` with
// the default `drop_oldest`) are UNIFIED onto their body subscribers,
// which removes the trigger-drain subscriber AND its WaitSet event listener.
// A STANDALONE listener-only `Listener` per unified input is
// the reactor wake source. This is the WALL-TIME latency pin for that restore:
// a publish on the external trigger topic must wake the live loop WELL BEFORE
// the WAKE_TIMEOUT heartbeat fallback elapses.
//
// This is the structural complement to the firewall tests above (which gate
// WHAT fires + the order); here we gate WHEN — the very thing the unification
// once regressed (a pure-external-data unified graph woke only at the heartbeat
// cadence, since the unified inputs were `filter_map`ped OUT of the source
// list). The 4 `waitset_reactor_iox2_test` cases are the in-process structural
// gate (they go green when the source is restored); this is the e2e wall-time
// gate over real iceoryx2.
//
// Removing the standalone listener source (the
// `live_step` source builder `filter_map`s unified bindings out) → `sources`
// for this pure-unified graph is EMPTY → `live_step`'s `blocked = false` →
// it never blocks on the WaitSet, BUT the heartbeat fallback `sleep`s the full
// WAKE_TIMEOUT before stepping → `elapsed >= WAKE_TIMEOUT` → this assertion
// fails. (The sink still fires — `drain_level` drains the body sub every step
// regardless of the wake — so it is the WALL TIME, not the fire, that catches
// the regression.)
// ===========================================================================

#[test]
#[serial]
fn live_step_wakes_promptly_on_unified_input() {
    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = chain_graph(Arc::clone(&observed), Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build polled-vs-live chain graph");

    let mut pubr = {
        let mgr: &Arc<TransportManager> = runtime.test_transport().expect("test transport parked");
        mgr.create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("external publisher must attach to /pvl/ext")
    };

    // Prime: one live iteration with NO data drains the build/attach
    // connection-lifecycle noise the trigger `Listener`s multiplex alongside
    // data events (mirrors `run_chain`'s pre-loop drive). The sink must stay at
    // zero fires so a connection-noise wake can never be mistaken for the
    // data-driven wake we time below.
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "the priming drive (no data published) must NOT fire the sink"
    );

    // Publish ONE frame, then TIME a single live iteration. With the standalone
    // listener source in place, the publish's iceoryx2 event wakes the
    // reactor promptly — well under the WAKE_TIMEOUT heartbeat fallback.
    publish_one(&mut pubr, 1.0);
    let start = std::time::Instant::now();
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    let elapsed = start.elapsed();

    // The data flowed all 3 levels in this ONE iteration (the
    // within-step collapse): the sink fired exactly once and observed the value.
    assert_eq!(
        fires.load(Ordering::Relaxed),
        1,
        "one publish + one live iteration must fire the sink exactly once \
         (ext→relay→mid→sink collapses within the step)"
    );
    assert_eq!(
        observed.lock().unwrap().clone(),
        vec![1.0],
        "the sink must observe exactly the one published value"
    );

    // The headline assertion: the wake was PROMPT. A generous bound (a third of
    // the timeout) absorbs CI scheduler jitter while still failing hard if the
    // loop fell back to the full heartbeat (the regression this arm catches).
    let prompt_bound = WAKE_TIMEOUT / 3;
    // If this ever flakes on a saturated CI runner, WIDEN the bound to
    // `WAKE_TIMEOUT / 2` (still 2× below the 150 ms heartbeat-fallback
    // regression value) — do NOT delete it: this is the only wall-time
    // wake-promptness regression guard for the standalone listener source.
    assert!(
        elapsed < prompt_bound,
        "the live loop must wake PROMPTLY on a unified data-trigger input — \
         elapsed {elapsed:?} should be well under the {prompt_bound:?} prompt \
         bound (WAKE_TIMEOUT/3). A regression that drops the standalone listener \
         source sleeps the full {WAKE_TIMEOUT:?} heartbeat fallback instead."
    );

    runtime.shutdown();
}

// ===========================================================================
// Test 5 — an e2e SMOKE TEST that BOTH `TriggerSubscriber`
// variants COEXIST in one live source list.
//
// Two consumers on the SAME external trigger topic `/pvl/ext`:
//   - `Sink`       — plain `#[input(trigger)]` (drop_oldest default) → UNIFIED
//                    onto its body subscriber → live wake source is a
//                    standalone `TriggerSubscriber::ListenerOnly`.
//   - `SampleSink` — `#[input(trigger, backpressure = sample(2))]` → INELIGIBLE
//                    for unification → keeps the legacy dual-subscriber path →
//                    live wake source is a `TriggerSubscriber::Ipc`.
//
// So this graph's live WaitSet source list contains BOTH `TriggerSubscriber`
// variants at once — the exact case the source builder's `.map` over
// both variants must handle (it replaced a `filter_map` that dropped the unified
// bindings entirely). The byte-identity / wall-time tests above each use a
// PURE-unified graph; none mixes the variants in one source list, leaving the
// mixed-dispatch arm un-smoke-tested e2e until now.
//
// ## What this test pins (and what it does NOT)
//
// This is an END-TO-END SMOKE TEST: both variants coexist in one live source
// list, compile under the `.map`-over-both dispatch, and both fire e2e
// on a single publish. The added WALL-TIME bound (mirroring
// `live_step_wakes_promptly_on_unified_input`) catches a BOTH-sources-dropped
// regression — if the `.map` reverted to a `filter_map` that emptied the
// source list, the live loop would never block on the WaitSet and would `sleep`
// the full heartbeat fallback, blowing the bound.
//
// It does NOT, on its own, pin per-variant source-list membership: `drain_level`
// drains EVERY data-trigger binding each live iteration REGARDLESS of which
// source woke the loop, so a SINGLE dropped variant would still fire via the
// heartbeat fallback (within the wake timeout) and the fire-count oracle below
// would not catch it. The genuine per-variant STRUCTURAL pin (drop EITHER
// variant → it is absent from the reactor's timing-independent fired-set) lives
// in `waitset_reactor_iox2_test::reactor_records_both_unified_and_ipc_sources`.
//
// ORACLE: publish ONE frame, drive ONE live iteration → BOTH consumers fire
// EXACTLY once, WELL UNDER the heartbeat fallback. (Prime out connection-
// lifecycle noise first with a no-data live step asserting 0 fires on both,
// mirroring `live_step_wakes_promptly_on_unified_input`.)
// ===========================================================================

#[test]
#[serial]
fn live_step_wakes_both_unified_and_ipc_sources() {
    let unified_observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let unified_fires = Arc::new(AtomicU64::new(0));
    let sampled_observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let sampled_fires = Arc::new(AtomicU64::new(0));

    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "mixed_source_live_test".to_string(),
        prefix: "pvl".to_string(),
        nodes: vec![
            // UNIFIED consumer (plain trigger → drop_oldest → ListenerOnly).
            NodeDef {
                fuse: None,
                ros2: None,
                id: "unified".to_string(),
                node_type: "sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: EXT_TOPIC.to_string(),
                }],
                outputs: vec![],
            },
            // INELIGIBLE consumer (sample(2) → dual-subscriber → Ipc).
            NodeDef {
                fuse: None,
                ros2: None,
                id: "sampled".to_string(),
                node_type: "sample_sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: EXT_TOPIC.to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    // NOTE: `build_for_test` keys the factory map by node *ID* (`node_def.id`),
    // not by `node_type` — so the keys are "unified"/"sampled", matching the IDs
    // above. (`chain_graph` gets away with type-named keys only because its IDs
    // happen to equal its types.)
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "unified".to_string(),
        Box::new(SinkEntry::with_state(Sink {
            observed: Arc::clone(&unified_observed),
            fires: Arc::clone(&unified_fires),
            ..Default::default()
        })),
    );
    factories.insert(
        "sampled".to_string(),
        Box::new(SampleSinkEntry::with_state(SampleSink {
            observed: Arc::clone(&sampled_observed),
            fires: Arc::clone(&sampled_fires),
            ..Default::default()
        })),
    );

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build mixed-source live graph");

    let mut pubr = {
        let mgr: &Arc<TransportManager> = runtime.test_transport().expect("test transport parked");
        mgr.create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("external publisher must attach to /pvl/ext")
    };

    // Prime: one live iteration with NO data drains the build/attach
    // connection-lifecycle noise both trigger sources multiplex alongside data
    // events. BOTH consumers must stay at zero fires so a connection-noise wake
    // can never be mistaken for the data fire we count below.
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    assert_eq!(
        unified_fires.load(Ordering::Relaxed),
        0,
        "the priming drive (no data) must NOT fire the unified (ListenerOnly) consumer"
    );
    assert_eq!(
        sampled_fires.load(Ordering::Relaxed),
        0,
        "the priming drive (no data) must NOT fire the sampled (Ipc) consumer"
    );

    // Publish ONE frame, then TIME a single live iteration. The publish's
    // iceoryx2 event is delivered to BOTH source variants in the one live source
    // list (the `.map`-over-both dispatch), so both consumers wake and
    // fire in this single iteration. The wall-time bound (below) catches a
    // BOTH-sources-dropped regression: if the `.map` reverted to a `filter_map`
    // that emptied the source list, the loop would never block on the WaitSet and
    // would sleep the full WAKE_TIMEOUT heartbeat fallback.
    publish_one(&mut pubr, 1.0);
    let start = std::time::Instant::now();
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    let elapsed = start.elapsed();

    // The wake was PROMPT. A generous bound (a third of the timeout) absorbs CI
    // scheduler jitter while still failing hard if the loop fell back to the full
    // heartbeat (the BOTH-dropped regression). Mirrors
    // `live_step_wakes_promptly_on_unified_input`; widen to `WAKE_TIMEOUT / 2`
    // (still 2× below the 150 ms heartbeat-fallback value) before deleting if it
    // ever flakes on a saturated CI runner.
    let prompt_bound = WAKE_TIMEOUT / 3;
    assert!(
        elapsed < prompt_bound,
        "the live loop must wake PROMPTLY when BOTH variants are in the source \
         list — elapsed {elapsed:?} should be well under the {prompt_bound:?} \
         prompt bound (WAKE_TIMEOUT/3). A regression that empties the source list \
         (both variants dropped) sleeps the full {WAKE_TIMEOUT:?} heartbeat \
         fallback instead."
    );

    // ORACLE: BOTH consumers fired EXACTLY once. The headline assertion — a
    // ListenerOnly source AND an Ipc source in ONE live source list both wake the
    // loop on the same publish.
    assert_eq!(
        unified_fires.load(Ordering::Relaxed),
        1,
        "the UNIFIED (ListenerOnly source) consumer must fire exactly once on one publish"
    );
    assert_eq!(
        sampled_fires.load(Ordering::Relaxed),
        1,
        "the INELIGIBLE (Ipc source) consumer must fire exactly once on one publish — \
         the mixed live source list must wake both variants"
    );
    assert_eq!(
        unified_observed.lock().unwrap().clone(),
        vec![1.0],
        "the unified consumer must observe exactly the one published value"
    );
    assert_eq!(
        sampled_observed.lock().unwrap().clone(),
        vec![1.0],
        "the sampled consumer must observe exactly the one published value"
    );

    runtime.shutdown();
}

// ===========================================================================
// Test 6: the PARKED-seam firewall pin.
//
// The live loop's blocking WaitSet wait is replaced by a record-only shallow
// monitor-wait park when a `MonitorWaitPolicy` is active. The park is a WAIT
// primitive — it changes only WHEN `step()` runs, NEVER what fires. This test
// proves the park does NOT break / reorder / drop firing: a park-ON live run
// fires EXACTLY the hand oracle `[relay,mid,sink] × N` and flows data
// byte-identically to a park-OFF live run on the SAME build path.
//
// WHY THIS IS NOT A TAUTOLOGY:
//   (1) `parked_seq == expected_fire_sequence(N)` ties park-ON to a HAND-BUILT
//       oracle (the absolute expected content `[relay,mid,sink]×N`), not a
//       self-compare of two closure runs.
//   (2) `parked_seq == unparked_seq` is an INDEPENDENT control: BOTH sides use
//       the SAME live build path (`build_for_test_with_policy`), differing ONLY
//       in the policy (`monitor_wait/doorbell` on vs `off()`), so the comparison
//       ISOLATES the park's effect — there is no build-path confound.
//
// WHY IT DOES NOT COMPARE AGAINST THE VIRTUAL-BUILT POLLED SEAM:
//   Tests 1-5 already chain the virtual-built polled `step()` == live ==
//   hand oracle. Comparing the policy-LIVE-built runtime against the
//   virtual-`build_for_test`-built polled runtime would mix TWO different build
//   paths (`build_for_test_with_policy` vs `build_for_test`), introducing a
//   build-path confound. Instead this test chains live-built park-ON == park-OFF
//   == the SHARED hand oracle — both sides on the policy build path — and the
//   oracle is the SAME `expected_fire_sequence` tests 1-5 pin the polled seam to.
//   So transitively: virtual-polled == virtual-live == oracle == live-park-OFF
//   == live-park-ON, with each link an apples-to-apples comparison.
//
// On Apple Silicon there is NO real CPU monitor-wait primitive (the
// doorbell ring is a no-op stub), so the park degrades to a sleep-recheck. The
// assertions are about FIRE SEQUENCE + DATA FLOW only (never timing/latency), so
// they hold regardless of whether a real hardware park ran — the real CPU-park
// latency is measured separately on WAITPKG/WFE hardware.
// ===========================================================================

#[test]
#[serial]
fn parked_live_seam_is_byte_identical_to_oracle_and_unparked() {
    const N: u64 = 6;

    // Park ON: monitor_wait + doorbell. doorbell ON → the consumer
    // `DoorbellRegistry` is built + the producers' owned doorbells ring; on macOS
    // the ring is a stub no-op, so the data still flows via real iceoryx2 and the
    // listener poll wakes the loop.
    let (parked_seq, parked_vals) = run_chain_parked(
        cerulion_core::MonitorWaitPolicy::new(true, true, "pvl".into()),
        N,
    );
    // Park OFF: the SAME live build path, park OFF — the control isolating the
    // park's effect (no build-path confound).
    let (unparked_seq, unparked_vals) =
        run_chain_parked(cerulion_core::MonitorWaitPolicy::off(), N);

    // (1) HAND ORACLE (non-tautological): park-ON fires EXACTLY the absolute
    // expected sequence `[relay,mid,sink] × N`. The park is record-only — it
    // changes WHEN step() runs, never WHAT fires — so the trace is byte-identical
    // to the unparked firing path.
    assert_eq!(
        parked_seq,
        expected_fire_sequence(N),
        "the monitor-wait park must fire EXACTLY the hand oracle [relay,mid,sink]×N \
         — the park is record-only (changes WHEN step() runs, never WHAT fires)"
    );

    // (2) INDEPENDENT CONTROL: park-ON == park-OFF. Same live build path, only the
    // policy differs → this isolates the park's effect (no build-path confound).
    assert_eq!(
        parked_seq, unparked_seq,
        "the parked live seam must fire the SAME nodes in the SAME order as the \
         unparked live seam on the SAME build path — the monitor-wait park is a \
         WAIT primitive, not a firing-path change (replay=live firewall)"
    );

    // Byte-identical DATA FLOW: each value 1..=N flows ext→relay→mid→sink
    // identically, both against the hand oracle (1.0..=N) and against the park-OFF
    // control.
    let expected_vals: Vec<f64> = (1..=N).map(|i| i as f64).collect();
    assert_eq!(
        parked_vals, expected_vals,
        "the sink under the park must observe exactly the published values 1.0..=N \
         in order (hand oracle)"
    );
    assert_eq!(
        parked_vals, unparked_vals,
        "data flow must be byte-identical with the park ON vs OFF on the same \
         build path"
    );
}

// ===========================================================================
// Test 7: the STRENGTHENED firewall — the polled seam and
// the DETERMINISTIC-LIVE (Barrier) seam produce a byte-identical FULL trace,
// INCLUDING `fire_time_ns` (Principle #7, Replay = Live).
//
// Tests 1-6 compare the live seam to the polled seam EXCLUDING `fire_time_ns`,
// because the `Mode::Live` (and parked) seams build via `build_for_test` (the
// wall-driven live path), whose live loop advances the `VirtualClock` by REAL
// wall-clock deltas → non-deterministic absolute fire times. That exclusion is
// LEGITIMATE there.
//
// The DETERMINISTIC-LIVE path (`Mode::LiveBarrier`, built via
// `build_for_test_barrier`) instead advances the scheduler's `Barrier` GATING
// clock by a fixed logical QUANTUM each step. THE EQUIVALENCE THIS TEST RESTS ON:
// this chain is PURE DATA-DRIVEN (NO Period node), so `tightest_timing_ns() ==
// None` ⇒ the quantum is the 1ms FALLBACK, which EQUALS the polled
// `step(Duration::from_millis(1))` delta. Both seams therefore advance the gating
// clock by exactly 1ms/step from a fresh `VirtualClock` at 0 (`run_live_step_once`
// performs exactly ONE clock advance per call, same as one `step()`), so
// `fire_time_ns` is byte-identical step-for-step. If the polled delta or the
// quantum ever diverged, the absolute-time hand oracle (`expected_fire_times_ns`)
// would fail LOUDLY rather than be fudged.
//
// WHY THIS IS NOT A SELF-COMPARE:
//   (1) `barrier_trace`'s node-id SEQUENCE is anchored to `expected_fire_sequence`
//       — the SAME hand oracle test 1 pins the polled seam to.
//   (2) `barrier_trace`'s `fire_time_ns` STREAM is anchored to
//       `expected_fire_times_ns` — a hand-known absolute schedule (`[(k+1)·1ms;3]`),
//       NOT merely "equal to the polled run".
//   So the `poll_trace == barrier_trace` full-trace assertion (which additionally
//   covers `step` + `global_level`) is ANCHORED to hand oracles on BOTH the WHAT
//   and the WHEN dimensions — the firewall, not a tautology.
// ===========================================================================

#[test]
#[serial]
fn polled_and_barrier_live_produce_byte_identical_full_trace() {
    const N: u64 = 6;

    // Polled seam: `step(1ms)` advances the gating clock by exactly 1ms/step.
    let (poll_trace, poll_vals) = run_chain_trace(Mode::Polled, N);
    // DETERMINISTIC-LIVE (Barrier) seam: `run_live_step_once_for_test` advances the
    // `Barrier` gating clock by the fixed 1ms quantum (pure-data-driven chain ⇒
    // `tightest_timing_ns() == None` ⇒ 1ms fallback == the polled delta), so its
    // absolute `fire_time_ns` is replay-deterministic AND equal to the polled seam.
    let (barrier_trace, barrier_vals) = run_chain_trace(Mode::LiveBarrier, N);

    // (1) HAND ORACLE — fire SEQUENCE. Anchors `barrier_trace` to the SAME absolute
    // expected content test 1 pins the polled seam to (`[relay,mid,sink] × N`), so
    // the full-trace pin below is not a pure cross-build self-compare.
    let barrier_sequence: Vec<String> = barrier_trace
        .iter()
        .map(|e| e.node_id.to_string())
        .collect();
    assert_eq!(
        barrier_sequence,
        expected_fire_sequence(N),
        "the deterministic-live fire sequence must be exactly [relay, mid, sink] × N \
         (the same hand oracle test 1 pins the polled seam to)"
    );

    // (2) HAND ORACLE — absolute `fire_time_ns`. Anchors the deterministic-live
    // seam's fire times to the hand-known 1ms-quantum schedule `[(k+1)·1ms; 3]`,
    // making the `fire_time_ns` dimension non-tautological (NOT just poll==barrier).
    let barrier_times: Vec<u64> = barrier_trace.iter().map(|e| e.fire_time_ns).collect();
    assert_eq!(
        barrier_times,
        expected_fire_times_ns(N),
        "the deterministic-live seam must stamp `fire_time_ns` at the hand-known \
         1ms-quantum schedule [(k+1)·1ms; 3] (pre-loop drive = step 1 / clock→1ms / \
         no fire; publish k fires at (k+1)·1ms) — the absolute-time anchor that \
         makes the full-trace pin a HAND ORACLE, not a poll-vs-barrier self-compare"
    );

    // THE STRENGTHENED FIREWALL: byte-identical over the FULL `Vec<TraceEntry>` —
    // `node_id` + `step` + `global_level` AND `fire_time_ns` (only the wall-time
    // `duration_ns` is excluded, by `TraceEntry`'s hand-written `PartialEq`, since
    // it is non-replayable). Where the wall-advanced `Live` seam (test 1) could only
    // match the SEQUENCE, the deterministic-live seam matches the ABSOLUTE
    // `fire_time_ns` too — the deeper Principle #7 proof. (Transitively, with (1)+(2):
    // polled-full-trace == barrier-full-trace, and barrier == the hand oracles on
    // both sequence and time, so polled is likewise anchored.)
    assert_eq!(
        poll_trace, barrier_trace,
        "the deterministic-live (Barrier) seam must produce a byte-identical FULL \
         trace to the polled seam — node_id + step + global_level + fire_time_ns \
         (replay=live firewall, Principle #7). Only the wall-time duration_ns is \
         excluded (non-replayable)."
    );

    // Byte-identical DATA FLOW + the data hand oracle (1.0..=N in order).
    let expected_vals: Vec<f64> = (1..=N).map(|i| i as f64).collect();
    assert_eq!(
        barrier_vals, expected_vals,
        "the sink must observe exactly the published values 1.0..=N in order (hand oracle)"
    );
    assert_eq!(
        poll_vals, barrier_vals,
        "each published value must flow ext→relay→mid→sink identically in the polled \
         and deterministic-live seams"
    );
}
