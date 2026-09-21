// SPDX-License-Identifier: AGPL-3.0-only
//! The monitor-wait park's BARRIER-ARRIVAL wake predicate
//! (`BarrierShared::peers_waiting` polled by `GraphRuntime::monitor_wait_block`)
//! — what keeps the per-node multi-process split from collapsing to the park-timeout
//! cadence (~4Hz at the production 250ms cap).
//!
//! # The bug this pins
//!
//! In a barrier-lockstep split, EVERY context must cross EVERY global-level
//! boundary each step (empty-level participation). A context like g3=[sink]
//! (no period node; its only upstream topic publishes at global L1) has NO wake
//! source for STEP START: the relay cannot fire until g3 arrives at the L0→L1
//! barrier, and anything that could ring g3 only happens AFTER the relay fires
//! — a circular wait that only the park timeout would break. The predicate wakes
//! the park when `remaining < expected` on the shared barrier ("someone has
//! arrived at the current generation and it hasn't opened ⇒ a step has begun
//! and the cohort is waiting on me") — absolute state, no baseline snapshot,
//! no false wakes at idle boundaries (the opener re-arms `remaining ==
//! expected`). Because the mechanism is fixed structurally, no launch refusal
//! for such topologies is needed.
//!
//! # The harness
//!
//! The motivating shape, in-process: a 3-node chain ticker(period_ms=1) → relay
//! (data-trigger) → sink, split across THREE contexts (global_level_maps
//! `[Some(0),None,None]` / `[None,Some(0),None]` / `[None,None,Some(0)]`) over
//! ONE shared iceoryx2 SHM root (three `TransportManager`s), each runtime built
//! on the LIVE path with a FORCED `MonitorWaitPolicy` (doorbell off; park ON in
//! the primary arm, park OFF in the `split_completes_with_park_off_via_barrier_
//! routing` arm — a barrier participant idles in `monitor_wait_block` either
//! way) via `build_live_with_schema_hashes_and_policy`, the
//! shared `MappedBarrier(expected = 3)` injected via
//! `set_barrier_participant_for_test`, and all three driven CONCURRENTLY on OS
//! threads through `run_live_step_once_for_test`.
//!
//! The SINK is a `period_ms = 1` node with a NON-TRIGGER `#[input]` on the
//! relay's absolute topic — deliberately: a non-trigger input contributes NO
//! listener wake source and (doorbell off) no doorbell line, so the sink
//! context's park has NOTHING but the timer without the predicate — the un-ringable
//! step-start topology, at EVERY step, with no stale-listener-event
//! contamination. (A data-trigger sink in a real per-node split shows the same
//! all-timeout wakes; the period+held-input shape reproduces that step-start
//! blindness portably in-process. Its park timeout here, 250ms, models the
//! production liveliness cap.) The sink context is also the
//! held-context consumer: it builds FIRST, creating the relay-out service at
//! `subscriber_max_borrowed_samples = 3`. (Before the borrow-floor change this consumer-first order
//! was census-sanctioned: a producer-first build created the service at the
//! iceoryx2 default 2 and refused the borrow-3 consumer. The borrow-floor change now CREATES
//! every owned topic at the borrow-3 floor — create-side only; an owner
//! tolerates a pre-existing smaller service with a degraded warn — so a
//! producer-first relay-out creation would also carry 3 and the order is
//! retained here only for build determinism.)
//!
//! # Pins
//!
//! 1. LIVENESS (FAILS without the predicate): all `WARMUP + N` lockstep steps complete within
//!    a 3s wall bound. Without it, EVERY step parks the sink context for the full
//!    250ms (its park has no listener, no doorbell — nothing can ring it):
//!    `(4 + 20) × 250ms = 6s` — that exceeds the bound; with it the
//!    barrier predicate wakes each park in ~one 100µs recheck (well under 1s
//!    total).
//! 2. ATTRIBUTION (FAILS on a revert of the `barrier_got` wake even if timing
//!    flukes pass pin 1): the sink context's `park_wakes_barrier > 0` — its
//!    park has no listener and no doorbell, so the barrier predicate is its
//!    ONLY possible non-timeout wake cause.
//! 3. CORRECTNESS/firewall: the sink observed EXACTLY the hand oracle
//!    `[1.0, 2.0, .., N.0]` — at step k it reads the relay's SAME-step forward
//!    (an earlier-level publish flows down the DAG within one step, exactly as
//!    in the monolith; the prior-value rule applies to SAME-level producers
//!    only). The wake change alters WHEN steps run, never WHAT fires. Plus the
//!    lockstep invariant: the shared generation ends at exactly
//!    `3 × (WARMUP + N)`.
//! 4. NO-FALSE-WAKES control: a SOLO barrier(1) context's park never reports a
//!    barrier wake (`park_wakes_barrier == 0` with `park_entries > 0`) — while
//!    parked between steps the barrier is re-armed (`remaining == expected`),
//!    and the solo context is the only arriver.
//!
//! All `#[serial]` (live parks + WaitSet-adjacent machinery over the iceoryx2
//! SHM singleton; barrier names are pid+tag scoped).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use cerulion_core::barrier::MappedBarrier;
use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::MonitorWaitPolicy;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

// ===========================================================================
// Tunables
// ===========================================================================

/// Measured lockstep steps (the ticker fires + publishes once per step).
const N: u64 = 20;

/// No-fire priming steps (no virtual-time advance anywhere ⇒ the period nodes
/// stay before their first deadline and NOTHING publishes). They establish
/// every iceoryx2 connection before the measured window (mirrors
/// `barrier_level_gate_iox2_test`).
const WARMUP: u64 = 4;

/// Global DAG levels (= each global_level_map's length). The shared barrier
/// crosses exactly this many generations per step.
const GLOBAL_LEVELS: u64 = 3;

/// Per-measured-step virtual-time advance on the ticker/sink threads — exactly
/// one `period_ms = 1` deadline, so each node fires exactly once per measured
/// step (no catch-up bursts; `next_fire = now + interval` at build keeps the
/// warmup silent).
const ADVANCE: Duration = Duration::from_millis(1);

/// The ticker context's park timeout — short, modeling the production
/// `live_timeout` a 1ms-period graph derives. The ticker thread is the
/// cadence initiator: between steps everyone idles re-armed (no barrier wake),
/// so this timeout is what starts each step.
const PARK_SHORT: Duration = Duration::from_millis(5);

/// The relay/sink contexts' park timeout — models the production ~250ms
/// liveliness-cap park a period-less context falls to. Without the wake this IS the
/// step cadence (the collapse); the barrier predicate wakes the park
/// long before it.
const PARK_LONG: Duration = Duration::from_millis(250);

/// Pin-1 wall bound over the whole concurrent run (WARMUP + N steps). Collapse
/// floor: (4 + 20) × 250ms = 6s — the sink context's park is timeout-gated on
/// EVERY step (no listener, no doorbell). With the wake: well under 1s. 3s splits
/// them with ≥ 2× margin either side on a noisy CI VM.
const WALL_BOUND: Duration = Duration::from_secs(3);

/// Hard per-thread result deadline — a true wedge (worse than the timeout
/// collapse) fails loudly here instead of hanging CI. Generous: even the
/// timeout collapse completes in ~6s, and a barrier-poisoned run is bounded by
/// the 5s `BARRIER_BOUNDARY_TIMEOUT` per boundary plus fast no-op steps.
const JOIN_TIMEOUT: Duration = Duration::from_secs(60);

/// Ticker context's derived output topic (prefix `bpwt`, node `ticker`,
/// output `out`) — consumed by the relay context as an absolute source.
const TICKER_TOPIC: &str = "/bpwt/ticker/out";

/// Relay context's derived output topic — consumed by the sink context as an
/// absolute NON-TRIGGER (held) source.
const RELAY_TOPIC: &str = "/bpwr/relay/out";

/// g1 = [ticker] at global level 0.
const MAP_T: [Option<usize>; 3] = [Some(0), None, None];
/// g2 = [relay] at global level 1.
const MAP_R: [Option<usize>; 3] = [None, Some(0), None];
/// g3 = [sink] at global level 2.
const MAP_S: [Option<usize>; 3] = [None, None, Some(0)];

// ===========================================================================
// Nodes
// ===========================================================================

/// 1kHz source: publishes its fire count (1.0, 2.0, ..) each fire. Fires only
/// when its thread advances the shared `VirtualClock` (the live-path scheduler
/// reads the clock; it never advances it).
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct C555Ticker {
    #[output]
    out: Vector3,
    count: u64,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl C555Ticker {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.count += 1;
        self.out.x = self.count as f64;
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// Data-trigger relay: forwards `inp.x` on each arrival.
#[cerulion_node]
#[derive(Default)]
struct C555Relay {
    #[input(trigger)]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl C555Relay {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.inp.x;
        Ok(())
    }
}

/// Period sink with a NON-TRIGGER (held) input: records the step-boundary
/// snapshot of `inp.x` each fire. No trigger input ⇒ no listener wake source ⇒
/// (doorbell off) its park is step-start-blind without the barrier wake.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct C555Sink {
    #[input]
    inp: Vector3,
    observed: Arc<Mutex<Vec<f64>>>,
}

#[cerulion_node_impl]
impl C555Sink {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.observed.lock().unwrap().push(self.inp.x);
        Ok(())
    }
}

// ===========================================================================
// Config / transport / barrier helpers.
// ===========================================================================

fn out_def(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "geometry_msgs/Vector3".to_string(),
        max_slice_len: None,
        topic: None,
        history_size: 0,
    }
}

fn in_def(name: &str, source: &str) -> InputDef {
    InputDef {
        name: name.to_string(),
        source: source.to_string(),
    }
}

fn graph_config(name: &str, prefix: &str, nodes: Vec<NodeDef>) -> GraphConfig {
    GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: name.to_string(),
        prefix: prefix.to_string(),
        nodes,
    }
}

/// One "graph process": its own `TransportManager` (iceoryx2 node) over the
/// shared isolated SHM root `ix`. `subscriber_buffer_size = 16` (mirrors
/// `barrier_level_gate_iox2_test`): a context that OWNS a cross-context topic
/// with no in-graph consumer provisions the buffer ceiling from this transport
/// default alone, and the downstream context's subscriber requires
/// `DEFAULT_CONSUMER_DEPTH` (10) — which 16 covers but the stock 8 would not.
fn manager(name: &str, ix: iceoryx2::config::Config) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: name.into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 16,
            network: None,
        },
        ix,
    )
    .expect("init isolated transport manager")
}

/// pid+tag-scoped barrier namespace so concurrent test binaries / re-runs never
/// collide on a POSIX SHM object (any Unix) or a registry key (the
/// non-Unix stub).
fn barrier_ns(tag: &str) -> String {
    format!("park_{}_{tag}", std::process::id())
}

/// The forced park policy: monitor-wait ON or OFF per arm, doorbell OFF.
/// Doorbell OFF is part of the test design — it removes the sink context's
/// only other potential wake line, isolating the barrier predicate as its sole
/// non-timeout cause (pin 2's attribution needs that exclusivity). The
/// park-OFF arm pins the idle routing: a barrier participant
/// idles in `monitor_wait_block` (sleep-recheck paced, no CPU primitive) even
/// when the park policy is off.
fn park_policy(ns: &str, park_on: bool) -> MonitorWaitPolicy {
    MonitorWaitPolicy::new(park_on, false, ns.to_string())
}

/// Build one context on the LIVE path with the forced park policy over the
/// shared SHM root.
fn build_ctx(
    config: GraphConfig,
    factories: IndexMap<String, Box<dyn NodeEntry>>,
    mgr: &TransportManager,
    clock: Arc<VirtualClock>,
    ns: &str,
    park_on: bool,
) -> GraphRuntime {
    let clock_dyn: Arc<dyn cerulion_core::clock::Clock> = clock;
    GraphRuntime::build_live_with_schema_hashes_and_policy(
        config,
        factories,
        mgr,
        clock_dyn,
        None,
        park_policy(ns, park_on),
    )
    .expect("build live context with forced park policy")
}

// ===========================================================================
// Pin 1 + 2 + 3 — the 3-context split.
// ===========================================================================

/// Bounded worker-result collection distinguishing the three exit shapes:
/// `Ok` (normal — join reaps the finished thread), `Disconnected` (the worker
/// PANICKED and its sender dropped — join and `resume_unwind` the REAL panic
/// instead of misreporting a wedge), `Timeout` (a genuine wedge — fail loudly
/// instead of hanging CI).
/// Caveats (accepted for the MODELED failure modes, which are all bounded —
/// barrier timeout ⇒ 5s terminal poison ⇒ fast no-op steps ⇒ send): the
/// collection order is fixed (ticker→relay→sink), so an EARLIER worker's
/// Timeout is reported even if a LATER worker's panic holds the root cause
/// (the Timeout message says so); and the post-Ok `join` is unbounded (the
/// thread already ran its closure to the `send` — only Drop teardown
/// remains).
fn collect_worker<T>(rx: mpsc::Receiver<T>, handle: thread::JoinHandle<()>, who: &str) -> T {
    match rx.recv_timeout(JOIN_TIMEOUT) {
        Ok(v) => {
            // `send` is each worker closure's last statement, so a panic here
            // is teardown-only — still propagate the REAL payload.
            if let Err(panic_payload) = handle.join() {
                std::panic::resume_unwind(panic_payload);
            }
            v
        }
        Err(mpsc::RecvTimeoutError::Disconnected) => match handle.join() {
            Err(panic_payload) => std::panic::resume_unwind(panic_payload),
            Ok(()) => panic!("{who} sender dropped without a result or a panic"),
        },
        Err(mpsc::RecvTimeoutError::Timeout) => {
            panic!(
                "{who} context did not finish within JOIN_TIMEOUT — wedged run (NOTE: collection \
                 order is ticker→relay→sink; a LATER worker's panic may hold the root cause — \
                 check its thread output)"
            )
        }
    }
}

/// The hand oracle for the sink's observations: at measured step k the ticker
/// publishes k, the relay (global L1) forwards it, and the sink (global L2)
/// reads the relay's SAME-step forward — an earlier-level publish flows down
/// the DAG within one step, exactly as in the monolith (the firewall: barrier
/// gating changes WHEN contexts run, never the dataflow). `[1.0, 2.0, ..,
/// n.0]`. (Measured: the one-step-lag guess — "snapshot reads the prior
/// step's forward" — was wrong; that rule is for SAME-level producers, not an
/// upstream level.)
fn expected_sink_values(n: u64) -> Vec<f64> {
    (1..=n).map(|i| i as f64).collect()
}

#[test]
#[serial]
fn split_park_wakes_on_barrier_arrival_liveness_attribution_and_oracle() {
    run_split_pins("split3", true);
}

/// Park-OFF routing pin: the SAME split completes at full cadence with the
/// monitor-wait park OFF — a barrier participant's idle routes through
/// `monitor_wait_block` regardless of the park policy (`park_active() ||
/// barrier_participant.is_some()` in `live_step`), with the opt-out honored
/// inside (sleep-recheck pacing, no CPU monitor-wait primitive touched).
/// Without that routing a park-off participant is barrier-blind — the sink
/// (empty sources: non-trigger input ⇒ no listener) falls to the heartbeat
/// sleep, the relay to the blocking WaitSet — and the split re-collapses to
/// the timeout cadence whenever the park is off (`CERULION_MONITOR_WAIT=0`,
/// or the CLI resolver's no-primitive fallback). On a revert, pins 1
/// (liveness) and 2 (attribution: zero park entries ⇒ zero barrier wakes)
/// fail; pin 3 still PASSES — timeout-paced lockstep delivers identical
/// dataflow, which is precisely the record-only firewall this file pins.
#[test]
#[serial]
fn split_completes_with_park_off_via_barrier_routing() {
    run_split_pins("split3_parkoff", false);
}

fn run_split_pins(barrier_tag: &str, park_on: bool) {
    let ix = cerulion_core::testing::iceoryx_test_config();

    // Build order is a fixed, deterministic sequence: the SINK context
    // first — its non-trigger absolute source makes it the snapshot consumer,
    // and its build CREATES the relay-out service at
    // `subscriber_max_borrowed_samples = 3`. (The order is not load-bearing:
    // owned topics are CREATED at the borrow-3 floor, and an owner
    // tolerates a pre-existing smaller service, e.g. the ticker-out service
    // the relay context creates at the default 2 below, which the ticker
    // owner attaches to degraded-but-warned — so the order is kept only
    // for determinism.) Then the relay context (attaches its publisher to the
    // existing service; creates the ticker-out service as its own absolute
    // consumer), then the ticker context (attaches its publisher).
    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let mgr_s = manager("park_sink", ix.clone());
    let clock_s = Arc::new(VirtualClock::new());
    let mut rt_s = {
        let config = graph_config(
            "sink_ctx",
            "bpws",
            vec![NodeDef {
                ros2: None,
                id: "sink".to_string(),
                node_type: "sink".to_string(),
                inputs: vec![in_def("inp", RELAY_TOPIC)],
                outputs: vec![],
            }],
        );
        let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories.insert(
            "sink".to_string(),
            Box::new(C555SinkEntry::with_state(C555Sink {
                observed: Arc::clone(&observed),
                ..Default::default()
            })),
        );
        build_ctx(
            config,
            factories,
            &mgr_s,
            Arc::clone(&clock_s),
            "bpws",
            park_on,
        )
    };

    let mgr_r = manager("park_relay", ix.clone());
    let mut rt_r = {
        let config = graph_config(
            "relay_ctx",
            "bpwr",
            vec![NodeDef {
                ros2: None,
                id: "relay".to_string(),
                node_type: "relay".to_string(),
                inputs: vec![in_def("inp", TICKER_TOPIC)],
                outputs: vec![out_def("out")],
            }],
        );
        let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories.insert("relay".to_string(), Box::new(C555RelayEntry::new()));
        build_ctx(
            config,
            factories,
            &mgr_r,
            Arc::new(VirtualClock::new()),
            "bpwr",
            park_on,
        )
    };

    let ticker_fires = Arc::new(AtomicU64::new(0));
    let mgr_t = manager("park_ticker", ix);
    let clock_t = Arc::new(VirtualClock::new());
    let mut rt_t = {
        let config = graph_config(
            "ticker_ctx",
            "bpwt",
            vec![NodeDef {
                ros2: None,
                id: "ticker".to_string(),
                node_type: "ticker".to_string(),
                inputs: vec![],
                outputs: vec![out_def("out")],
            }],
        );
        let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories.insert(
            "ticker".to_string(),
            Box::new(C555TickerEntry::with_state(C555Ticker {
                fires: Arc::clone(&ticker_fires),
                ..Default::default()
            })),
        );
        build_ctx(
            config,
            factories,
            &mgr_t,
            Arc::clone(&clock_t),
            "bpwt",
            park_on,
        )
    };

    // ONE shared barrier (expected = 3); the ticker context owns the segment,
    // the other two open it. `probe` reads the shared generation after the run.
    let ns = barrier_ns(barrier_tag);
    let owner = Arc::new(MappedBarrier::create_owned(&ns, "g", 3).expect("barrier owner create"));
    let peer_r =
        Arc::new(MappedBarrier::open_unowned(&ns, "g").expect("barrier peer open (relay)"));
    let peer_s = Arc::new(MappedBarrier::open_unowned(&ns, "g").expect("barrier peer open (sink)"));
    let probe = Arc::clone(&owner);

    rt_t.set_barrier_participant_for_test(owner, MAP_T.to_vec(), 0);
    rt_r.set_barrier_participant_for_test(peer_r, MAP_R.to_vec(), 1);
    rt_s.set_barrier_participant_for_test(peer_s, MAP_S.to_vec(), 2);

    let wall_start = Instant::now();

    // Ticker thread: WARMUP silent iterations (no advance ⇒ no fire), then N
    // measured iterations advancing exactly one period each. Its short park
    // timeout makes it the step initiator.
    let (tx_t, rx_t) = mpsc::channel::<u64>();
    let t_handle = thread::spawn(move || {
        for _ in 0..WARMUP {
            rt_t.run_live_step_once_for_test(PARK_SHORT);
        }
        for _ in 0..N {
            clock_t.advance(ADVANCE.as_nanos() as u64);
            rt_t.run_live_step_once_for_test(PARK_SHORT);
        }
        let fires = ticker_fires.load(Ordering::Relaxed);
        rt_t.shutdown();
        drop(mgr_t);
        tx_t.send(fires).expect("report ticker fires");
    });

    // Relay thread: pure lockstep driver (data-triggered — no clock advance).
    let (tx_r, rx_r) = mpsc::channel::<()>();
    let r_handle = thread::spawn(move || {
        for _ in 0..(WARMUP + N) {
            rt_r.run_live_step_once_for_test(PARK_LONG);
        }
        rt_r.shutdown();
        drop(mgr_r);
        tx_r.send(()).expect("report relay done");
    });

    // Sink thread: WARMUP silent + N measured (advance ⇒ the sink fires and
    // records its held-input snapshot). Reads its park counters BEFORE
    // shutdown.
    let (tx_s, rx_s) = mpsc::channel::<(u64, u64)>();
    let s_handle = thread::spawn(move || {
        for _ in 0..WARMUP {
            rt_s.run_live_step_once_for_test(PARK_LONG);
        }
        for _ in 0..N {
            clock_s.advance(ADVANCE.as_nanos() as u64);
            rt_s.run_live_step_once_for_test(PARK_LONG);
        }
        let barrier_wakes = rt_s.park_wakes_barrier_count_for_test();
        let park_entries = rt_s.park_entry_count_for_test();
        rt_s.shutdown();
        drop(mgr_s);
        tx_s.send((barrier_wakes, park_entries))
            .expect("report sink park counters");
    });

    // Bounded collection — a wedge fails loudly here instead of hanging CI,
    // and a worker PANIC is propagated as itself (the sender drops =>
    // Disconnected => join surfaces the real panic) instead of being
    // misreported as a JOIN_TIMEOUT wedge.
    let ticker_total = collect_worker(rx_t, t_handle, "ticker");
    collect_worker(rx_r, r_handle, "relay");
    let (sink_barrier_wakes, sink_park_entries) = collect_worker(rx_s, s_handle, "sink");
    let elapsed = wall_start.elapsed();

    // Pin 1 — LIVENESS. Without the barrier wake the sink context's park has no wake source at
    // all (no listener, no doorbell), so EVERY step costs the full PARK_LONG:
    // (WARMUP + N) × 250ms = 6s > 3s. With it: comfortably < 1s.
    assert!(
        elapsed < WALL_BOUND,
        "the {} lockstep steps took {elapsed:?} (bound {WALL_BOUND:?}) — the \
         split collapsed to the park-timeout cadence: the sink context's park \
         was not woken by peer barrier arrivals (the barrier-arrival wake fix)",
        WARMUP + N
    );

    // Pin 2 — ATTRIBUTION. The sink context's park has no listener and no
    // doorbell, so a barrier-predicate wake is its only possible non-timeout
    // cause; reverting the `barrier_got` wake in `monitor_wait_block` zeroes
    // this even if timing flukes pass pin 1.
    assert!(
        sink_park_entries > 0,
        "the sink context never entered the park — the harness is not \
         exercising monitor_wait_block (park_entries == 0)"
    );
    assert!(
        sink_barrier_wakes > 0,
        "the sink context's park reported ZERO barrier-arrival wakes across \
         {sink_park_entries} park entries — the peers_waiting wake predicate \
         is not load-bearing (the wake predicate reverted)"
    );

    // Pin 3 — CORRECTNESS/firewall. The ticker fired exactly once per measured
    // step; the sink observed exactly the hand oracle (at step k it reads the
    // relay's SAME-step forward k — an earlier-level publish flows down the
    // DAG within one step, as in the monolith). The wake change alters WHEN
    // steps run, never WHAT fires.
    assert_eq!(
        ticker_total, N,
        "ticker must fire exactly once per measured step"
    );
    let observed = observed.lock().unwrap().clone();
    assert_eq!(
        observed,
        expected_sink_values(N),
        "sink must observe the hand oracle [1.0, 2.0, .., {N}.0] — same-step \
         relay forwards flowing down the DAG"
    );

    // Lockstep invariant: every context crossed every boundary of every step.
    assert_eq!(
        probe.current_generation(),
        GLOBAL_LEVELS * (WARMUP + N),
        "the shared barrier generation must equal levels × steps (lockstep)"
    );
}

// ===========================================================================
// Pin 4 — the solo no-false-wakes control.
// ===========================================================================

/// A SOLO barrier(1) context must never report a barrier wake: its own
/// arrivals happen inside `step()` (never while parked), and each arrival at
/// `expected == 1` opens + re-arms immediately, so every park-loop poll reads
/// `remaining == expected` ⇒ `peers_waiting() == false`.
#[test]
#[serial]
fn solo_context_park_reports_zero_barrier_wakes() {
    const ITERS: u64 = 10;

    let ix = cerulion_core::testing::iceoryx_test_config();
    let fires = Arc::new(AtomicU64::new(0));
    let mgr = manager("park_solo", ix);
    let clock = Arc::new(VirtualClock::new());
    let mut rt = {
        let config = graph_config(
            "solo_ctx",
            "bpwsolo",
            vec![NodeDef {
                ros2: None,
                id: "ticker".to_string(),
                node_type: "ticker".to_string(),
                inputs: vec![],
                outputs: vec![out_def("out")],
            }],
        );
        let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories.insert(
            "ticker".to_string(),
            Box::new(C555TickerEntry::with_state(C555Ticker {
                fires: Arc::clone(&fires),
                ..Default::default()
            })),
        );
        build_ctx(config, factories, &mgr, Arc::clone(&clock), "bpwsolo", true)
    };

    let ns = barrier_ns("solo");
    let owner = Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("solo barrier create"));
    rt.set_barrier_participant_for_test(owner, vec![Some(0)], 0);

    for _ in 0..ITERS {
        clock.advance(ADVANCE.as_nanos() as u64);
        rt.run_live_step_once_for_test(PARK_SHORT);
    }

    let barrier_wakes = rt.park_wakes_barrier_count_for_test();
    let park_entries = rt.park_entry_count_for_test();
    let total_fires = fires.load(Ordering::Relaxed);
    rt.shutdown();

    assert!(
        total_fires > 0,
        "solo control must actually fire (apparatus alive)"
    );
    assert!(
        park_entries > 0,
        "solo control must actually park (otherwise the zero-wakes assert is \
         vacuous)"
    );
    assert_eq!(
        barrier_wakes, 0,
        "a solo barrier(1) context must NEVER report a barrier-arrival park \
         wake — while parked the barrier is re-armed (remaining == expected), \
         so peers_waiting is false (the no-false-wakes control)"
    );
}

// ===========================================================================
// Pin 6 — the wake-word kernel block is LIVE in the park.
// ===========================================================================

/// Three-armed: which park path a machine
/// actually takes, pinned per-tier on the SOLO scaffold. The `!performed`
/// wake-word arm counts kernel BLOCKS (`park_wake_word_blocks` — a parked solo
/// context blocks-and-times-out per slice; the woken-by-arrival behavior is
/// pinned hermetically in
/// `barrier.rs::tests::parked_waiter_on_wake_word_wakes_on_arrival`).
///
/// **Arm A — hardware CPU park performs** (`monitor_wait_available()` true:
/// x86 WAITPKG, aarch64 WFE): `performed == true` means the
/// wake-word arm NEVER runs — `park_wake_word_blocks == 0` AND (the
/// NO-FUTILE-WAKE pin) the parked bit was NEVER set
/// during the run (`park_enter_call_count_for_test` delta == 0), so arrivers
/// on hardware-park machines pay no wake syscall for a waiter the CPU monitor
/// cannot hear. THIS is the arm WAITPKG/WFE hardware exercises.
///
/// **Arm B — no hardware park, wake-word primitive present**
/// (`monitor_wait_available()` false + `wake_word_block_primitive_available()`
/// true: macOS ≥ 14.4; GH CI runners — macOS always, Linux
/// runners without WAITPKG via the futex): the arm-swap pin
/// — `park_wake_word_blocks > 0` (reverting the arm swap zeroes it while
/// pins 1–5 stay green), plus a POSITIVE `park_enter` delta (the
/// anti-tautology twin of arm A's zero: the counter demonstrably moves).
///
/// **Arm C — neither** (macOS < 14.4 / `CERULION_BARRIER_OS_SYNC=0` /
/// future-Windows): the arm is structurally unreachable — both counters 0.
///
/// Compound invariant on EVERY arm: the solo no-false-wakes contract holds
/// (`park_wakes_barrier == 0` — a wake-word block that times out or wakes
/// spuriously never fabricates a barrier ATTRIBUTION; attribution stays the
/// loop-top `peers_waiting` re-derive).
#[test]
#[serial]
fn solo_context_park_takes_wake_word_kernel_block() {
    const ITERS: u64 = 10;
    let park_enters_before = cerulion_core::barrier::park_enter_call_count_for_test();

    let ix = cerulion_core::testing::iceoryx_test_config();
    let fires = Arc::new(AtomicU64::new(0));
    let mgr = manager("park_wword", ix);
    let clock = Arc::new(VirtualClock::new());
    let mut rt = {
        let config = graph_config(
            "wword_ctx",
            "bpwword",
            vec![NodeDef {
                ros2: None,
                id: "ticker".to_string(),
                node_type: "ticker".to_string(),
                inputs: vec![],
                outputs: vec![out_def("out")],
            }],
        );
        let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        factories.insert(
            "ticker".to_string(),
            Box::new(C555TickerEntry::with_state(C555Ticker {
                fires: Arc::clone(&fires),
                ..Default::default()
            })),
        );
        build_ctx(config, factories, &mgr, Arc::clone(&clock), "bpwword", true)
    };

    let ns = barrier_ns("wword");
    let owner = Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("barrier create"));
    rt.set_barrier_participant_for_test(owner, vec![Some(0)], 0);

    for _ in 0..ITERS {
        clock.advance(ADVANCE.as_nanos() as u64);
        rt.run_live_step_once_for_test(PARK_SHORT);
    }

    let wake_word_blocks = rt.park_wake_word_block_count_for_test();
    let barrier_wakes = rt.park_wakes_barrier_count_for_test();
    let park_entries = rt.park_entry_count_for_test();
    let total_fires = fires.load(Ordering::Relaxed);
    rt.shutdown();
    let park_enters_delta =
        cerulion_core::barrier::park_enter_call_count_for_test() - park_enters_before;

    assert!(
        total_fires > 0,
        "the wake-word pin must actually fire (apparatus alive)"
    );
    assert!(
        park_entries > 0,
        "the wake-word pin must actually park (otherwise the counter assert is vacuous)"
    );
    let hw_park = cerulion_core::monitor_wait::monitor_wait_available();
    let primitive = cerulion_core::barrier::wake_word_block_primitive_available();
    eprintln!(
        "barrier-wake pin 6: hw_park={hw_park} wake_word_primitive={primitive} \
         blocks={wake_word_blocks} park_enters_delta={park_enters_delta}"
    );
    if hw_park {
        // Arm A (x86 WAITPKG / aarch64 WFE): the hardware primitive performs the
        // park — the wake-word arm must be structurally unreached AND the
        // parked bit never set (the no-futile-wake pin: arrivers must not
        // pay a wake syscall the hardware park cannot hear).
        assert_eq!(
            wake_word_blocks, 0,
            "hardware-park machine: the !performed wake-word arm must never run"
        );
        assert_eq!(
            park_enters_delta, 0,
            "hardware-park machine: the parked bit must NEVER be set (a set bit \
             taxes every arrive with a futile wake syscall)"
        );
    } else if primitive {
        // Arm B (macOS ≥ 14.4; GH CI macOS + non-WAITPKG Linux):
        // the arm-swap pin.
        assert!(
            wake_word_blocks > 0,
            "a wake-word-eligible barrier participant's park must take the kernel \
             block (park_wake_word_blocks > 0) — 0 means the !performed-arm swap \
             regressed to the unconditional pacing sleep (the wake-word chunk)"
        );
        assert!(
            park_enters_delta > 0,
            "the wake-word arm must set the parked bit around its blocks (the \
             anti-tautology twin of arm A's zero — the counter demonstrably moves)"
        );
    } else {
        // Arm C: no primitive at all — structurally unreachable arm.
        eprintln!(
            "barrier-wake pin 6: no park-wake primitive on this host — asserting the \
             negative (both counters must be 0, the arm is structurally unreachable)"
        );
        assert_eq!(
            wake_word_blocks, 0,
            "without a park-wake primitive the kernel-block arm must be unreachable"
        );
        assert_eq!(
            park_enters_delta, 0,
            "without a park-wake primitive the parked bit must never be set"
        );
    }
    assert_eq!(
        barrier_wakes, 0,
        "the kernel block must never fabricate a barrier-arrival ATTRIBUTION — \
         the solo no-false-wakes contract holds with the wake word live"
    );
}
