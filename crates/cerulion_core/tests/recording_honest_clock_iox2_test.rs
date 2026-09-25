// SPDX-License-Identifier: AGPL-3.0-only
//! The replay-grade recording clock model, e2e over real
//! iceoryx2 (the GraphRuntime half — the bagd handshake/teardown is pinned in
//! `cerulion_bagd`'s subprocess tests).
//!
//! Recording runs the LIVE loop on a CONTROLLED gating clock (a `VirtualClock`
//! advanced only via `begin_step`, so `fire_time_ns` is RECORDED in the trace
//! and re-advanceable in replay) but advanced by the MEASURED wall delta each
//! step (`set_gating_follows_wall(true)`, the wall-following clock model). This
//! file pins two contracts:
//!
//! 1. **Wall-following clock** — with the flag ON, consecutive `fire_time_ns` deltas
//!    track the MEASURED wall elapsed (jitter preserved), NOT the fixed logical
//!    quantum. With the flag OFF (the fixed-quantum deterministic-live path) the
//!    deltas equal the quantum EXACTLY. A controlled sleep between live steps
//!    forces the two to diverge by a wide margin.
//! 2. **Principle #7 firewall** — the flag changes only the recorded
//!    TIMESTAMPS, never WHAT fires: a pure-data-driven chain driven with the flag
//!    ON produces a BYTE-IDENTICAL fire node-id SEQUENCE + data flow to the flag
//!    OFF, and BOTH match a hand oracle (`fire_time_ns` excluded by design — its
//!    semantics change under the wall-following clock).
//!
//! `#[serial]` — the live loop builds an iceoryx2 WaitSet over the process-global
//! SHM singleton (mirrors `polled_vs_live_iox2_test` / `live_gating_clock_iox2_test`).
//! Run with `--test-threads=1`.

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

/// The absolute external trigger topic for `relay` (no in-graph producer → an
/// out-of-graph publisher attaches freely).
const EXT_TOPIC: &str = "/rec/ext";
/// A short per-iteration WaitSet timeout for the data-chain (a published event
/// wakes the reactor well within it).
const WAKE_TIMEOUT: Duration = Duration::from_millis(20);

// ===========================================================================
// Nodes.
// ===========================================================================

/// L0: triggers on the absolute external `/rec/ext`, forwards `inp.x` → `out`.
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

/// L1: triggers on `relay/out`, records each observed `inp.x` + bumps a fire
/// counter (the data-flow + fire oracles).
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
// Fixtures.
// ===========================================================================

fn vector3_output(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "geometry_msgs/Vector3".to_string(),
        max_slice_len: None,
        topic: None,
        history_size: 0,
    }
}

/// A 2-level forward chain: `/rec/ext` → relay → sink. `observed`/`fires` are the
/// sink's shared data-flow + fire oracles.
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
        identity: "rec_firewall".to_string(),
        prefix: "rec".to_string(),
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
                id: "sink".to_string(),
                node_type: "sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "relay/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("relay".to_string(), Box::new(RelayEntry::new()));
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

fn publish_one(pubr: &mut cerulion_core::CerulionPublisher, x: f64) {
    let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan");
    proxy.x = x;
    drop(proxy); // publish
}

/// Run the data chain for `n` publishes, built via the deterministic-live path
/// (`build_for_test_barrier`). `follows_wall` flips the wall-following flag. Returns
/// the fired node-id SEQUENCE + the sink's observed values.
fn run_chain(follows_wall: bool, n: u64) -> (Vec<String>, Vec<f64>) {
    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = chain_graph(Arc::clone(&observed), Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test_barrier(config, factories, clock, 8)
        .expect("build deterministic-live recording chain");
    // The recording mode delta under test: the wall-following gating clock + real tick
    // durations (mirrors the CLI recording path).
    runtime
        .set_gating_follows_wall(follows_wall)
        .expect("a monolith recording build (no participant) accepts the wall-following clock");
    runtime.set_record_tick_durations(true);

    let mut pubr = {
        let mgr: &Arc<TransportManager> = runtime.test_transport().expect("test transport parked");
        mgr.create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("external publisher attaches to /rec/ext")
    };

    // Pre-loop drive (drain connection-lifecycle noise) — no data → no sink fire.
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "the pre-loop drive must not fire the sink (no data published yet)"
    );

    for i in 1..=n {
        publish_one(&mut pubr, i as f64);
        runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    }
    let seq: Vec<String> = runtime
        .trace()
        .iter()
        .map(|e| e.node_id.to_string())
        .collect();
    let obs = observed.lock().unwrap().clone();
    runtime.shutdown();
    (seq, obs)
}

// ===========================================================================
// Test 1 — Principle #7 firewall: wall-following clock ON == OFF == hand oracle
// (fire node-id SEQUENCE + data flow; fire_time_ns excluded by design).
// ===========================================================================

#[test]
#[serial]
fn honest_clock_is_a_firewall_same_fire_sequence_and_data_as_fixed_quantum() {
    const N: u64 = 5;
    let (seq_off, obs_off) = run_chain(false, N);
    let (seq_on, obs_on) = run_chain(true, N);

    // HAND ORACLE (NOT a self-compare): each publish flows relay→sink in ONE
    // step (the within-step level collapse), so the sequence is
    // [relay, sink] repeated N times, and the sink observes 1.0..=N in order.
    let oracle_seq: Vec<String> = (0..N)
        .flat_map(|_| ["relay".to_string(), "sink".to_string()])
        .collect();
    let oracle_obs: Vec<f64> = (1..=N).map(|i| i as f64).collect();

    assert_eq!(
        seq_off, oracle_seq,
        "fixed-quantum fire sequence must match the oracle"
    );
    assert_eq!(
        seq_on, oracle_seq,
        "wall-following fire sequence must be IDENTICAL to fixed-quantum AND the oracle — \
         the gating clock mode must NOT change WHAT fires (Principle #7)"
    );
    assert_eq!(
        obs_off, oracle_obs,
        "fixed-quantum data flow must match the oracle"
    );
    assert_eq!(
        obs_on, oracle_obs,
        "wall-following data flow must be IDENTICAL to fixed-quantum AND the oracle"
    );
}

// ===========================================================================
// Test 2 — wall-following clock: fire_time_ns tracks REAL MEASURED time, not a fixed
// quantum.
//
// A DATA-DRIVEN node (`relay`) fires EXACTLY once per step (its `fire_time_ns`
// is the gating clock at that step). With the wall-following clock ON, the per-step
// gating advance is the REAL measured live-step duration — for an event-driven
// data wake that is genuinely sub-millisecond (~tens of µs). With the flag OFF
// (fixed-quantum deterministic-live), every step advances EXACTLY the 1ms
// quantum regardless of how long it really took. So the two are trivially
// distinguishable: OFF deltas are all exactly 1ms; ON deltas measure real time
// and at least one is well below the quantum (impossible under a fixed quantum).
// ===========================================================================

/// Drive the data chain for `n` publishes and collect the `relay` node's
/// consecutive `fire_time_ns` values. `follows_wall` flips the wall-following flag.
fn relay_fire_times(follows_wall: bool, n: u64) -> Vec<u64> {
    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = chain_graph(Arc::clone(&observed), Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test_barrier(config, factories, clock, 8)
        .expect("build data chain");
    runtime
        .set_gating_follows_wall(follows_wall)
        .expect("a monolith recording build carries no lockstep participant");

    let mut pubr = {
        let mgr: &Arc<TransportManager> = runtime.test_transport().expect("test transport parked");
        mgr.create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("external publisher attaches")
    };
    // Pre-loop drive (no data): drains connection noise + advances one step.
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    for i in 1..=n {
        publish_one(&mut pubr, i as f64);
        runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    }
    let times: Vec<u64> = runtime
        .trace()
        .iter()
        .filter(|e| e.node_id.as_ref() == "relay")
        .map(|e| e.fire_time_ns)
        .collect();
    runtime.shutdown();
    times
}

#[test]
#[serial]
fn honest_clock_fire_time_tracks_measured_wall_not_the_quantum() {
    const N: u64 = 8;
    // The pure-data chain has no declared timing → the fixed quantum is the 1ms
    // fallback. The relay fires once per step in BOTH modes.

    // OFF (fixed-quantum deterministic-live): consecutive relay fire_time deltas
    // are EXACTLY the 1ms quantum, wall-independent.
    let off = relay_fire_times(false, N);
    assert!(
        off.len() >= 3,
        "expected several relay fires, got {}",
        off.len()
    );
    for w in off.windows(2) {
        assert_eq!(
            w[1] - w[0],
            1_000_000,
            "fixed-quantum relay deltas must equal the 1ms quantum EXACTLY (got {} ns)",
            w[1] - w[0]
        );
    }

    // ON (wall-following clock): consecutive relay fire_time deltas are REAL measured
    // step durations. An event-driven data-wake step is genuinely fast, so at
    // least one delta is well under the 1ms quantum — IMPOSSIBLE under the fixed
    // quantum (where EVERY delta is exactly 1ms). This single sub-quantum delta
    // disproves the fixed-quantum model; a fixed clock could never produce it.
    let on = relay_fire_times(true, N);
    assert!(
        on.len() >= 3,
        "expected several relay fires, got {}",
        on.len()
    );
    let min_delta = on
        .windows(2)
        .map(|w| w[1] - w[0])
        .min()
        .expect("at least one delta");
    assert!(
        min_delta < 500_000,
        "wall-following relay deltas must measure REAL time — an event-driven data \
         wake is sub-millisecond, so the smallest delta must be < 500µs (a fixed \
         1ms quantum could never produce a sub-quantum delta). Smallest was {min_delta} ns"
    );
    // And they must NOT all be the 1ms quantum (belt-and-suspenders vs OFF).
    assert!(
        on.windows(2).any(|w| w[1] - w[0] != 1_000_000),
        "wall-following deltas must differ from the fixed 1ms quantum"
    );
}

// ===========================================================================
// Test 3 — the recording setter/accessor contract.
// ===========================================================================

#[test]
#[serial]
fn gating_follows_wall_and_tick_durations_toggle() {
    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let (config, factories) = chain_graph(observed, Arc::new(AtomicU64::new(0)));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test_barrier(config, factories, clock, 8)
        .expect("build chain graph");

    // Both default OFF on a fresh deterministic-live build.
    assert!(
        !runtime.gating_follows_wall_for_test(),
        "gating_follows_wall must default false"
    );
    assert!(
        !runtime.record_tick_durations_enabled(),
        "record_tick_durations must default false"
    );

    // The recording path flips both ON.
    runtime
        .set_gating_follows_wall(true)
        .expect("a monolith recording build carries no lockstep participant");
    runtime.set_record_tick_durations(true);
    assert!(runtime.gating_follows_wall_for_test());
    assert!(runtime.record_tick_durations_enabled());

    // And back OFF (idempotent toggle). BOTH gates, not just the wall one: the
    // `set_record_tick_durations(false)` direction is the one arm no production
    // caller exercises (`graph profile` and the recording paths only ever set it
    // true), so a passthrough that ignored `false` would otherwise be unpinned
    // anywhere in the tree.
    runtime
        .set_gating_follows_wall(false)
        .expect("a monolith recording build carries no lockstep participant");
    assert!(!runtime.gating_follows_wall_for_test());
    runtime.set_record_tick_durations(false);
    assert!(
        !runtime.record_tick_durations_enabled(),
        "set_record_tick_durations(false) must disable the scheduler gate"
    );
    runtime.shutdown();
}

// ===========================================================================
// Test 4 — recording provisioning: a RECORDED topic's
// subscriber_max_borrowed_samples is raised to RECORDING_SUBSCRIBER_MAX_BORROWED
// (bagd held + channel + writer thirds + reserve); a non-recorded
// build leaves it at the iceoryx2 default of 2. The
// raise is the ONLY provisioning delta under --record (the firewall).
// ===========================================================================

/// An isolated per-test transport built on `clock` (the clock contract requires
/// the deterministic-live build to share this exact `VirtualClock`).
fn recording_test_manager(node_name: &str, clock: Arc<VirtualClock>) -> Arc<TransportManager> {
    let ix_config = cerulion_core::testing::iceoryx_test_config();
    let config = cerulion_core::transport::TransportConfig {
        node_name: node_name.into(),
        clock,
        subscriber_buffer_size: 16,
        network: None,
    };
    TransportManager::init_for_test(config, ix_config).expect("init_for_test")
}

/// Build the relay→sink chain deterministic-live with `recorded` topics, then
/// read the relay-output topic's actual `subscriber_max_borrowed_samples`.
fn relay_out_borrow(
    node_name: &str,
    recorded: Option<&std::collections::HashSet<String>>,
) -> usize {
    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let (config, factories) = chain_graph(observed, Arc::new(AtomicU64::new(0)));
    let relay_topic = cerulion_core::graph::resolve_output_topic(
        &config.prefix,
        "relay",
        &config.nodes[0].outputs[0],
    );
    let clock = Arc::new(VirtualClock::new());
    let mgr = recording_test_manager(node_name, Arc::clone(&clock));
    let runtime = GraphRuntime::build_live_deterministic_with_schema_hashes_and_policy(
        config,
        factories,
        &mgr,
        clock,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        recorded,
    )
    .expect("build deterministic-live recording graph");
    let borrow = mgr.default_subscriber_max_borrowed_samples_for_test(&relay_topic);
    runtime.shutdown();
    borrow
}

#[test]
#[serial]
fn recorded_topic_borrow_is_raised_control_is_default() {
    // CONTROL: no recording → the relay-output OWNED topic is CREATED at the
    // owned-topic borrow FLOOR (3), NOT the iceoryx2 raw default 2. (The relay→sink
    // edge is a data-trigger, so the topic is not a snapshot source — the 3
    // comes purely from the create-side floor on the owned create leg.)
    // The recording raise below (→8) is still a distinct anti-tautology (8 vs 3).
    let baseline = relay_out_borrow("rec_prov_off", None);
    assert_eq!(
        baseline, 3,
        "without --record the OWNED topic is created at the owned-topic borrow floor (3); \
         got {baseline}"
    );

    // RECORDING: the relay-output topic is raised to the recording borrow
    // budget (held + channel + writer thirds + reserve).
    let relay_topic = {
        let (config, _) = chain_graph(
            Arc::new(Mutex::new(Vec::new())),
            Arc::new(AtomicU64::new(0)),
        );
        cerulion_core::graph::resolve_output_topic(
            &config.prefix,
            "relay",
            &config.nodes[0].outputs[0],
        )
    };
    let mut recorded = std::collections::HashSet::new();
    recorded.insert(relay_topic);
    let raised = relay_out_borrow("rec_prov_on", Some(&recorded));
    assert_eq!(
        raised,
        cerulion_core::transport::RECORDING_SUBSCRIBER_MAX_BORROWED,
        "under --record the recorded topic's subscriber_max_borrowed_samples must be raised \
         to RECORDING_SUBSCRIBER_MAX_BORROWED; got {raised}"
    );
}

// ===========================================================================
// Test 5 — findings #6 + #20: the PLAIN (non-deterministic-live) build path.
// `gating_follows_wall` defaults OFF there too; toggling it ON with no gating
// quantum is a loudly-WARNED inert operation (the step-live None arm never
// consults the flag); and the plain path's borrow provisioning stays the
// owned-topic borrow floor 3 (no recorded set → no recording raise leaks through).
// ===========================================================================

#[test]
#[serial]
#[tracing_test::traced_test]
fn plain_build_flag_defaults_off_warns_inert_and_borrow_stays_default() {
    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let (config, factories) = chain_graph(observed, Arc::new(AtomicU64::new(0)));
    let relay_topic = cerulion_core::graph::resolve_output_topic(
        &config.prefix,
        "relay",
        &config.nodes[0].outputs[0],
    );
    let clock = Arc::new(VirtualClock::new());
    // build_for_test = the PLAIN path (live_gating_quantum == None).
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build plain chain graph");

    // #20: default OFF on the non-barrier build too.
    assert!(
        !runtime.gating_follows_wall_for_test(),
        "gating_follows_wall must default false on the plain build path"
    );
    // #20: plain-path borrow provisioning stays the owned-topic borrow
    // floor (3) — no recorded set was threaded, so the recording raise (→8)
    // cannot leak here.
    {
        let mgr = runtime.test_transport().expect("test transport parked");
        assert_eq!(
            mgr.default_subscriber_max_borrowed_samples_for_test(&relay_topic),
            3,
            "the plain (non-record) build must keep the owned-topic borrow floor (3), \
             not the recording-raised 8"
        );
    }
    // #6: toggling the wall-following clock ON with NO gating quantum is inert — and
    // must be LOUDLY warned, never silent.
    runtime
        .set_gating_follows_wall(true)
        .expect("a monolith recording build carries no lockstep participant");
    assert!(
        logs_contain("INERT"),
        "set_gating_follows_wall(true) on a no-quantum build must warn the flag is INERT"
    );
    runtime.shutdown();
}
