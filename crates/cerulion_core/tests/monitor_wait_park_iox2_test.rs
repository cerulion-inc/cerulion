// SPDX-License-Identifier: AGPL-3.0-only
//! FUNCTIONAL behavior of the live loop's monitor-wait park.
//!
//! The production live loop (`GraphRuntime::run_live` → `live_step`) replaces its
//! blocking WaitSet wait with a record-only shallow CPU monitor-wait park when a
//! [`MonitorWaitPolicy`] is active. This file pins the park's FUNCTIONAL behavior
//! — NOT its latency — over real iceoryx2:
//!
//!  - **(a) the PURE-PERIOD park** (the headline): a period graph with EMPTY
//!    `sources` (no data-trigger inputs) still FIRES under the park. The
//!    "skip the C-state cap + shallow-park pure-Period graphs" rule routes
//!    an empty-`sources` graph through `monitor_wait_block` instead of the
//!    deep-idle heartbeat sleep; this test is its regression guard.
//!  - **(b) the DOORBELL DATA path**: a data-trigger graph built with the
//!    doorbell policy builds its consumer `DoorbellRegistry`, opens producer
//!    doorbells, and flows data e2e identically to the unparked path.
//!  - **(c) the no-data-input EDGE**: a period graph with `doorbell: true` has an
//!    EMPTY registry (`primary_addr() == None`) → the park must degrade to the
//!    timer-only path and still fire, never panic or hang.
//!
//! ## A target with NO real monitor-wait primitive
//!
//! On Apple-Silicon (aarch64) macOS there is no x86 WAITPKG and no
//! Linux aarch64 `WFE` backend compiled in, so `monitor_wait_until*` resolves to
//! `Unavailable` and `monitor_wait_block` degrades to a chunked recheck NAP (it
//! does NOT busy-spin and NEVER UMWAIT/WFE-parks there). The nap
//! itself is `os_sync_wait_on_address_with_timeout` on macOS ≥ 14.4 (half the
//! `nanosleep` coalescing slop; `thread::sleep` on older hosts / under
//! `CERULION_PARK_OS_SYNC=0` — the tier-routing pin is
//! `degraded_nap_tier_matches_the_resolved_os_sync_availability`). That is
//! EXPECTED — these tests prove the WIRING + that the park does not BREAK
//! firing; the real CPU-park latency is measured separately on WAITPKG/WFE hardware. So every
//! assertion below is about FIRE COUNTS / DATA FLOW, never timing/latency.
//!
//! ## Anti-tautology
//!
//! Each test ties to a HAND-BUILT oracle or an INDEPENDENT control: (a) a
//! park-OFF control fires a comparable count (the park does not SUPPRESS period
//! firing); (b) the consumer observes the hand oracle `1.0..=N` and is
//! byte-identical across two runs (determinism); (c) the edge graph fires `> 0`
//! against the timer-park degrade contract.
//!
//! All `#[serial]` — the live loop builds an iceoryx2 WaitSet over the
//! process-global shared-memory singleton (mirrors `polled_vs_live_iox2_test` /
//! `chunk25b_live_default_iox2_test`).

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
use cerulion_core::MonitorWaitPolicy;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing_test::traced_test;

/// The absolute external trigger topic for the doorbell data-graph consumer. No
/// in-graph producer → provisioned `External`, an out-of-graph publisher attaches
/// freely (mirrors `polled_vs_live_iox2_test::EXT_TOPIC`).
const EXT_TOPIC: &str = "/mwp/ext";

/// A pid+tag-scoped DOORBELL namespace, so concurrent test PROCESSES and re-runs
/// never collide on a `/dev/shm` object.
///
/// # Why this exists
///
/// A doorbell's backing object is named `/cer_db_{ns}_{fnv(topic)}` — a PURE
/// function of `(ns, topic)`, carrying no pid and no randomness. With one
/// hardcoded `ns` (say `"mwp"`) every test here would map the SAME
/// physical page.
///
/// That is invisible under `cargo test -- --test-threads=1`, where the whole
/// binary is ONE process running its tests sequentially. Under nextest each test
/// is its OWN process and they run CONCURRENTLY — so
/// `doorbell_ring_during_park_is_attributed_to_doorbell_counter` (Linux-only,
/// which rings `/mwp/ext` every 200 µs) and
/// `doorbell_data_graph_builds_registry_and_flows_data` (which asserts the
/// doorbell counter is ZERO, its producer being out-of-graph and never ringing)
/// would be two processes writing and reading one shared page. The reader sees the
/// writer's ring: `left: 1, right: 0`.
///
/// MEASURED: that failure is byte-identical every time, always at the same
/// test index, and always Linux — the ringer is `#[cfg(target_os = "linux")]`,
/// so Linux runs exactly one more test than macOS and macOS can never
/// reproduce it. `left: 1` (not a large count) says the overlap is brief, which
/// is also why a scheduling change could turn it intermittent; scoping
/// the namespace removes the collision outright rather than making it rarer.
///
/// The iceoryx2 plane is not the problem — these tests already mint isolated
/// SHM roots. POSIX SHM is a SECOND, machine-global name plane, and the repo
/// applies exactly this rule elsewhere too: `doorbell.rs`'s own `test_ns` and
/// `barrier_park_wake_iox2_test.rs`'s `barrier_ns`.
///
/// The `tag` is not redundant with the pid: under a plain `cargo test` all these
/// tests share ONE process, and the ringer's thread is not joined before its
/// test returns, so a per-test tag is what keeps it out of a later test's page.
fn mwp_ns(tag: &str) -> String {
    format!("mwp_{}_{tag}", std::process::id())
}

/// A live wake timeout for the data-flow test: long enough that a published event
/// wakes the loop well before it elapses.
const WAKE_TIMEOUT: Duration = Duration::from_millis(150);

/// The per-iteration park budget for the pure-Period tests. Comfortably larger
/// than the 5ms `Ticker` period so each ~5ms-period iteration with this timeout
/// fires at least once.
const PERIOD_PARK_TIMEOUT: Duration = Duration::from_millis(50);

/// The deterministic virtual-time advance applied before each pure-Period live
/// step (2× the 5ms `Ticker` period, so each step crosses ≥1 period deadline).
/// See `run_ticker_live` for WHY the test advances virtual time itself.
const PERIOD_VTIME_ADVANCE: Duration = Duration::from_millis(10);

// ===========================================================================
// Nodes — replicated inline (test binaries are separate crates).
// ===========================================================================

/// A pure-Period node: NO data-trigger INPUTS → EMPTY `sources` (`sources` is
/// built from trigger inputs; a period output does NOT create one) → the
/// pure-Period park path. Bumps a shared `Arc<AtomicU64>` each tick (the fire
/// oracle).
///
/// NOTE: the `#[cerulion_node]` macro requires ≥1 `#[input]`/`#[output]` field,
/// so this carries an `#[output]` (matching `chunk25b_live_default`'s
/// `C25bPeriod` shape). The output is irrelevant to the empty-`sources` property
/// under test — only INPUTS contribute to the live-loop source list — so this is
/// still the pure-Period (timer-only) park path.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct Ticker {
    #[output]
    out: Vector3,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl Ticker {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// A minimal data-trigger consumer of the absolute external `/mwp/ext`. Records
/// each observed `inp.x` into a shared Vec (the data-flow oracle) and bumps a
/// shared fire counter.
#[cerulion_node]
#[derive(Default)]
struct Consumer {
    #[input(trigger)]
    inp: Vector3,
    observed: Arc<Mutex<Vec<f64>>>,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl Consumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.observed.lock().unwrap().push(self.inp.x);
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

// ===========================================================================
// Graph construction (hand-built).
// ===========================================================================

/// A one-node `Ticker` (period_ms=5) graph + factories, sharing `fires`. NO
/// data-trigger INPUTS → the empty-`sources` pure-Period park path (the lone
/// output does not create a source). `id` == factory-map key (`build_for_test`
/// keys by node ID).
fn ticker_graph(fires: Arc<AtomicU64>) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "mwp_ticker".to_string(),
        prefix: "mwp".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "ticker".to_string(),
            node_type: "ticker".to_string(),
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
    factories.insert(
        "ticker".to_string(),
        Box::new(TickerEntry::with_state(Ticker {
            fires: Arc::clone(&fires),
            ..Default::default()
        })),
    );
    (config, factories)
}

/// A one-node data-trigger `Consumer` of `/mwp/ext` + factories, sharing
/// `observed` / `fires`. `id` == factory-map key.
fn consumer_graph(
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
        identity: "mwp_consumer".to_string(),
        prefix: "mwp".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "consumer".to_string(),
            node_type: "consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: EXT_TOPIC.to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "consumer".to_string(),
        Box::new(ConsumerEntry::with_state(Consumer {
            observed: Arc::clone(&observed),
            fires: Arc::clone(&fires),
            ..Default::default()
        })),
    );
    (config, factories)
}

/// Drive a freshly-built `Ticker` graph LIVE for `iters` iterations under
/// `policy`, returning the total fire count.
///
/// Each iteration: advance virtual time by `PERIOD_VTIME_ADVANCE`, then run one
/// live step (`run_live_step_once_for_test(PERIOD_PARK_TIMEOUT)`). On a no-primitive
/// target the park's sleep-recheck fallback makes each iteration consume
/// ~`PERIOD_PARK_TIMEOUT` (expected, not a hang).
///
/// WHY the TEST advances virtual time: `build_for_test_with_policy` routes
/// through the LIVE build path (`build_live`), whose scheduler treats any
/// `Arc<dyn Clock>` as READ-ONLY (`ClockInner::Real`) — `step()` never advances a
/// `VirtualClock` through it. That is BY DESIGN: production `graph run` drives the
/// live loop with `RealClock` (advanced by the kernel monotonic clock), so a
/// pure-Period node's deadline is crossed by real wall time, not by `step()`.
/// The deterministic `VirtualClock` test seam therefore advances virtual time
/// itself (the canonical way to drive a `VirtualClock`). The monitor-wait PARK
/// still runs every iteration (empty `sources` → `monitor_wait_block`), and
/// `step()` fires the period node once virtual time crosses its deadline — so
/// this still proves the headline: the park does NOT suppress period firing.
/// (Data-trigger graphs fire via `signal_data`, independent of the clock, so
/// `doorbell_data_graph_builds_registry_and_flows_data` needs no such advance.)
/// Returns `(fires, park_entries)`: the period-node fire count AND the number of
/// times the live loop ENTERED `monitor_wait_block` (the park-entry routing
/// seam — see [`GraphRuntime::park_entry_count_for_test`]). Park-ON drives the
/// inter-step idle through the shallow park (`park_entries > 0`); park-OFF stays
/// on the heartbeat path (`park_entries == 0`).
fn run_ticker_live(policy: MonitorWaitPolicy, iters: usize) -> (u64, u64) {
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = ticker_graph(Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    // Hold a clone to advance virtual time (the scheduler shares this same
    // VirtualClock via the read-only Real arm, so advances are visible to it).
    let clock_drive = Arc::clone(&clock);
    let mut runtime = GraphRuntime::build_for_test_with_policy(config, factories, clock, 8, policy)
        .expect("build ticker graph under policy");

    for _ in 0..iters {
        clock_drive.advance(PERIOD_VTIME_ADVANCE.as_nanos() as u64);
        runtime.run_live_step_once_for_test(PERIOD_PARK_TIMEOUT);
    }
    let total = fires.load(Ordering::Relaxed);
    let park_entries = runtime.park_entry_count_for_test();
    runtime.shutdown();
    (total, park_entries)
}

/// Publish exactly ONE `Vector3` frame with `x = x` onto `EXT_TOPIC` (the proxy
/// publishes on drop).
fn publish_one(pubr: &mut cerulion_core::CerulionPublisher, x: f64) {
    let mut proxy = pubr.loan_proxy::<Vector3>().expect("loan");
    proxy.x = x;
    drop(proxy); // publish
}

// ===========================================================================
// Test (a) — THE HEADLINE: a pure-Period graph fires under the park.
//
// A period_ms=5 `Ticker` with EMPTY `sources` (no data inputs) must FIRE under
// the monitor-wait park: every live iteration runs `monitor_wait_block` (the
// empty-`sources` park path) and then `step()` — which fires the period node once
// virtual time crosses its deadline. It does NOT hang on a deep-idle heartbeat.
// This is the regression guard for the "skip the cap + park pure-Period
// graphs" rule (without it empty-`sources` graphs are stuck / never
// stepping under the park).
//
// LIVENESS, not exact count: `run_ticker_live` advances virtual time
// deterministically (see its doc — the live-path scheduler treats the
// VirtualClock as read-only), so firing is in fact deterministic; we assert
// `>= 1` as the robust liveness floor rather than pinning an exact catch-up count.
//
// ANTI-TAUTOLOGY / why fire-count-alone is insufficient: `drain_level` runs every
// step regardless of which idle path the loop took, so even a buggy build that
// left this graph on the heartbeat path would STILL fire. So this test ALSO runs
// a park-OFF control (`MonitorWaitPolicy::off()`, SAME builder, driven live):
// both must fire `> 0`, proving the park does not SUPPRESS period firing (it is a
// wait primitive, not a firing-path change). The park-OFF control is the
// independent oracle isolating the park's effect.
// ===========================================================================
#[test]
#[serial]
fn pure_period_graph_fires_under_the_park() {
    const ITERS: usize = 8;

    // Park ON (monitor_wait, doorbell off — a pure-Period graph has no data
    // doorbell). The headline: the period node fires under the shallow park.
    let (parked_fires, parked_park_entries) = run_ticker_live(
        MonitorWaitPolicy::new(true, false, mwp_ns("pureperiod")),
        ITERS,
    );
    assert!(
        parked_fires >= 1,
        "a period_ms=5 graph with EMPTY sources must FIRE under the monitor-wait \
         park (the park returns at the period deadline and step() fires the period \
         node; it must NOT hang on a deep-idle heartbeat) — got {parked_fires} fires"
    );

    // ROUTING PIN: the park branch WAS entered. An empty-`sources` (pure-Period)
    // graph under an active park must route the inter-step idle through
    // `monitor_wait_block`, NOT the deep-idle heartbeat. A regression sending
    // empty-sources to the heartbeat under an active policy leaves `park_entries`
    // at 0 and fails HERE (fire-count alone can't catch it — `drain_level` fires
    // every step regardless of idle path).
    // EXACT count (not just > 0): empty `sources` deterministically routes EVERY
    // `run_live_step_once_for_test` through `monitor_wait_block` exactly once (one
    // `live_step` per call, one increment per entry), so this must be EXACTLY
    // `ITERS`. Exact-count also catches a "park entered only SOME iterations"
    // regression that `> 0` would pass.
    assert_eq!(
        parked_park_entries, ITERS as u64,
        "park-ON must ENTER monitor_wait_block once per iteration (the \
         empty-sources park routing) — expected {ITERS}, got {parked_park_entries}"
    );

    // Park OFF control: SAME builder, SAME drive, only the policy differs. It must
    // ALSO fire — proving the park does not SUPPRESS period firing (it is the
    // independent oracle isolating the park's effect, not a self-compare).
    let (unparked_fires, unparked_park_entries) = run_ticker_live(MonitorWaitPolicy::off(), ITERS);
    assert!(
        unparked_fires >= 1,
        "the park-OFF control must ALSO fire the period node (it shares the live \
         build path; only the policy differs) — got {unparked_fires} fires"
    );

    // ROUTING PIN (off side): `off()` → `park_active() == false` → the live loop
    // takes the heartbeat path and NEVER enters `monitor_wait_block`.
    assert_eq!(
        unparked_park_entries, 0,
        "park-OFF must NEVER enter monitor_wait_block (it routes through the \
         heartbeat path) — got {unparked_park_entries} park entries"
    );

    // CLOCK FIREWALL PIN: the park-OFF control's fire count is deterministic; if
    // `monitor_wait_block` mutated the graph `VirtualClock` (the doc says it NEVER
    // advances the clock — `advance` is reachable via `&self`/Arc interior
    // mutability), the park-ON count would diverge. Both runs use the same
    // deterministic `PERIOD_VTIME_ADVANCE` × `ITERS`, so they MUST be equal absent
    // a firewall violation.
    assert_eq!(
        parked_fires, unparked_fires,
        "park-ON and park-OFF must fire the SAME number of times (the park must \
         not advance the graph VirtualClock — the determinism firewall)"
    );
}

// ===========================================================================
// Test (b) — the DOORBELL DATA path builds the registry + flows data e2e.
//
// A data-trigger consumer of `/mwp/ext` built with `doorbell: true` builds its
// consumer `DoorbellRegistry`, the producer opens an owned doorbell, and data
// flows e2e identically to the unparked path: publish N frames, drive live each,
// the consumer fires EXACTLY N times and observes the hand oracle `1.0..=N`. On
// a no-primitive target the SHM ring is a no-op stub, so the data still flows via real
// iceoryx2 (the listener poll wakes the loop). Determinism: two runs are
// byte-identical (Principle #7).
// ===========================================================================
#[test]
#[serial]
fn doorbell_data_graph_builds_registry_and_flows_data() {
    /// One run: build a fresh doorbell-policy consumer graph, attach an external
    /// publisher, publish N frames driving live each, return the observed Vec.
    fn run_once(n: u64) -> Vec<f64> {
        let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
        let fires = Arc::new(AtomicU64::new(0));
        let (config, factories) = consumer_graph(Arc::clone(&observed), Arc::clone(&fires));
        let clock = Arc::new(VirtualClock::new());
        let mut runtime = GraphRuntime::build_for_test_with_policy(
            config,
            factories,
            clock,
            8,
            MonitorWaitPolicy::new(true, true, mwp_ns("doorbelldata")),
        )
        .expect("build doorbell consumer graph");

        let mut pubr = {
            let mgr: &Arc<TransportManager> =
                runtime.test_transport().expect("test transport parked");
            mgr.create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
                .expect("external publisher must attach to /mwp/ext")
        };

        // Prime: one live iteration, NO publish — drains connection-lifecycle
        // noise so the fire count is attributable to the data publishes.
        runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
        assert_eq!(
            fires.load(Ordering::Relaxed),
            0,
            "the priming drive (no data) must NOT fire the consumer under the \
             doorbell park"
        );

        for i in 1..=n {
            publish_one(&mut pubr, i as f64);
            runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
        }

        // The consumer fired EXACTLY N times (one per publish — each flows in one
        // live iteration).
        assert_eq!(
            fires.load(Ordering::Relaxed),
            n,
            "the doorbell-policy consumer must fire EXACTLY {n} times for {n} \
             publishes — the registry is built, the producer owns a doorbell, and \
             the data flows e2e via real iceoryx2"
        );

        // Wake-cause attribution over the data flow. The counters
        // attribute PARK exits only — after a step that FIRED, the
        // pre-block spin (`last_step_fired` ⇒ one-hop budget) catches the next
        // publish's already-queued event in user space and the park is never
        // entered for that iteration, so most of the N data wakes are
        // legitimately spin-absorbed. The FIRST data wake, though, follows the
        // no-fire priming step (spin budget zero) and MUST route through the
        // park and exit via its LISTENER poll (the event is queued before the
        // drive). The DOORBELL counter must stay 0 on EVERY platform: this
        // producer is OUT-OF-GRAPH (never `enable_doorbell`ed, so it never
        // rings), and off-Linux the ring is a stub besides — so a doorbell
        // attribution here would mean the wake causes are cross-wired.
        let (entries, listener, doorbell, _timeout) = runtime.park_wake_counts();
        assert!(
            entries > 0,
            "the doorbell-policy live drive must route its idle through the park"
        );
        assert!(
            listener >= 1,
            "the first data publish (following a no-fire step, so no spin \
             budget) must produce a LISTENER-attributed park wake — got {listener}"
        );
        assert_eq!(
            doorbell, 0,
            "no doorbell ever rings in this test (out-of-graph producer) — a \
             nonzero doorbell count means listener wakes are being misattributed"
        );
        let out = observed.lock().unwrap().clone();
        runtime.shutdown();
        out
    }

    const N: u64 = 5;
    let observed = run_once(N);

    // HAND ORACLE: the consumer observed exactly the published values 1.0..=N.
    let expected: Vec<f64> = (1..=N).map(|i| i as f64).collect();
    assert_eq!(
        observed, expected,
        "the doorbell-policy consumer must observe exactly the hand oracle 1.0..=N"
    );

    // Determinism (Principle #7): a second run is byte-identical.
    let observed_b = run_once(N);
    assert_eq!(
        observed, observed_b,
        "two doorbell-policy runs must produce byte-identical observed values"
    );
}

/// Linux-only: a doorbell RING landing inside a park window is
/// attributed to the DOORBELL counter — the `wakes_doorbell` branch's e2e
/// coverage (the doorbell data test above pins it at 0, since its out-of-graph
/// producer never rings; off-Linux the ring is a no-op stub, so only Linux can
/// exercise the real branch — a production-scale Linux run measured
/// `wakes_doorbell=33638/33640`; this is the CI pin).
///
/// A NON-Cerulion ringer thread opens an OWNED doorbell on the SAME `(ns,
/// topic)` the consumer registry mapped (both `shm_open(O_CREAT)` the same
/// name → same physical page) and rings continuously, carrying NO data — so a
/// park exit here can only be doorbell-attributed (`any_advanced_since`), never
/// a listener event from a publish. Rings landing BETWEEN parks are absorbed by
/// the next entry's baseline snapshot; the continuous cadence (~200µs) vs the
/// 50ms park window guarantees rings land INSIDE windows too.
#[cfg(target_os = "linux")]
#[test]
#[serial]
fn doorbell_ring_during_park_is_attributed_to_doorbell_counter() {
    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = consumer_graph(Arc::clone(&observed), Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test_with_policy(
        config,
        factories,
        clock,
        8,
        MonitorWaitPolicy::new(true, true, mwp_ns("ringattr")),
    )
    .expect("build doorbell consumer graph");

    let stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let ringer = {
        let stop = Arc::clone(&stop);
        std::thread::spawn(move || {
            let db = cerulion_core::doorbell::Doorbell::open_owned(&mwp_ns("ringattr"), EXT_TOPIC)
                .expect("open the producer-side ringer doorbell");
            while !stop.load(Ordering::Relaxed) {
                db.ring();
                std::thread::sleep(Duration::from_micros(200));
            }
        })
    };

    // Each live step parks (50ms window); a ring lands inside it and wakes the
    // park early. No data flows — the steps fire nothing (records-only wake).
    for _ in 0..4 {
        runtime.run_live_step_once_for_test(Duration::from_millis(50));
    }
    stop.store(true, Ordering::Relaxed);
    ringer.join().expect("ringer thread panicked");

    let (entries, _listener, doorbell, _timeout) = runtime.park_wake_counts();
    assert!(
        entries > 0,
        "the parked live drive must route its idle through the park"
    );
    assert!(
        doorbell >= 1,
        "a doorbell ring landing inside a park window must be attributed to the \
         DOORBELL wake counter — got {doorbell} across {entries} park entries"
    );
    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "rings carry no data — the wake is record-only and must not fire the consumer"
    );
    runtime.shutdown();
}

// ===========================================================================
// Test (c) — EDGE: a parked graph with NO data inputs does not hang.
//
// Build the `Ticker` graph (NO data-trigger inputs) with `doorbell: true`. The
// consumer registry is EMPTY (no data inputs → no topics) so `primary_addr()`
// is `None` and the park has no hardware-armed line: it must degrade to the
// timer-only park and still FIRE the period node — never panic or hang on an
// empty registry / `None` primary. Drive a few live iterations; assert `> 0`.
// ===========================================================================
#[test]
#[serial]
fn parked_graph_with_no_data_inputs_does_not_hang() {
    const ITERS: usize = 8;

    // doorbell: true on a graph with NO data-trigger inputs → EMPTY registry →
    // primary_addr() == None → the park must fall to the timer-only path.
    let (fires, _park_entries) = run_ticker_live(
        MonitorWaitPolicy::new(true, true, mwp_ns("nodatainputs")),
        ITERS,
    );
    assert!(
        fires >= 1,
        "a doorbell-policy period graph with NO data inputs (EMPTY registry, None \
         primary) must degrade to the timer-only park and STILL fire — never panic \
         or hang — got {fires} fires"
    );
}

/// A graph runnable on the monitor-wait park alone
/// must NOT require — and must NOT build — a WaitSet reactor, so a reactor-build
/// failure can never abort an otherwise-runnable parked live loop. Proven
/// STRUCTURALLY: drive the park path (park ON, doorbell OFF) and assert that NO
/// WaitSet reactor is ever built — there is no build to fail. Pairs the park
/// liveness (`park_entry_count >= 1`, the idle really went through the park)
/// with `reactor_built_for_test() == false` before AND after the run.
#[test]
#[serial]
fn park_only_run_builds_no_waitset_reactor_l6() {
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = ticker_graph(Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let clock_drive = Arc::clone(&clock);
    // Park ON, doorbell OFF — pure monitor-wait park, no reactor needed.
    let mut runtime = GraphRuntime::build_for_test_with_policy(
        config,
        factories,
        clock,
        8,
        MonitorWaitPolicy::new(true, false, mwp_ns("noreactor")),
    )
    .expect("build ticker graph under park policy");

    // Fresh runtime: reactor is lazily built, so none yet.
    assert!(
        !runtime.reactor_built_for_test(),
        "fresh runtime should hold no WaitSet reactor"
    );

    for _ in 0..4 {
        clock_drive.advance(PERIOD_VTIME_ADVANCE.as_nanos() as u64);
        runtime.run_live_step_once_for_test(PERIOD_PARK_TIMEOUT);
    }

    assert!(
        runtime.park_entry_count_for_test() >= 1,
        "the park must have driven the inter-step idle (monitor_wait_block entered)"
    );
    assert!(
        !runtime.reactor_built_for_test(),
        "a park-only run must NEVER build a WaitSet reactor (so a build failure cannot abort it)"
    );
    runtime.shutdown();
}

// ===========================================================================
// Park wake-cause telemetry.
//
// The shallow monitor-wait park attributes every exit to a wake cause
// (`park_wake_counts()` = entries, listener, doorbell, timeout). These are
// PERMANENT record-only diagnostics (Principle #3) — incremented ONLY on the
// idle park path, NEVER read by the scheduler (the determinism firewall).
// ===========================================================================

/// A parked pure-Period run attributes its park exits.
///
/// Drives the SAME empty-`sources` `Ticker` graph as the headline test under an
/// ON policy: every `run_live_step_once_for_test` routes the inter-step idle
/// through `monitor_wait_block`, and on a no-primitive target the park always
/// reaches its deadline with no event → a TIMEOUT wake. So `park_entries ==
/// ITERS`, `park_wakes_timeout == ITERS`, and the listener/doorbell causes stay
/// 0 (no data sources). A weaker floor — `park_entries > 0` AND the wake-cause
/// sum `> 0` — is a subset of this exact oracle.
///
/// ANTI-TAUTOLOGY: a park-OFF control (SAME builder, only the policy differs)
/// never enters `monitor_wait_block`, so ALL FOUR counts stay 0 — the
/// independent oracle proving the counters track the park, not merely stepping.
#[test]
#[serial]
fn park_wake_counts_attribute_pure_period_timeouts() {
    const ITERS: usize = 8;

    /// One run: build a fresh ticker graph under `policy`, drive `ITERS` live
    /// steps advancing virtual time each, return `park_wake_counts()`.
    fn run(policy: MonitorWaitPolicy, iters: usize) -> (u64, u64, u64, u64) {
        let fires = Arc::new(AtomicU64::new(0));
        let (config, factories) = ticker_graph(Arc::clone(&fires));
        let clock = Arc::new(VirtualClock::new());
        let clock_drive = Arc::clone(&clock);
        let mut runtime =
            GraphRuntime::build_for_test_with_policy(config, factories, clock, 8, policy)
                .expect("build ticker graph under policy");
        for _ in 0..iters {
            clock_drive.advance(PERIOD_VTIME_ADVANCE.as_nanos() as u64);
            runtime.run_live_step_once_for_test(PERIOD_PARK_TIMEOUT);
        }
        let counts = runtime.park_wake_counts();
        runtime.shutdown();
        counts
    }

    // Park ON (monitor_wait, doorbell off — a pure-Period graph has no data
    // doorbell). Every park exit is a timeout on this empty-`sources` graph.
    let (entries, listener, doorbell, timeout) = run(
        MonitorWaitPolicy::new(true, false, mwp_ns("pureperiodcounts")),
        ITERS,
    );
    assert!(
        entries > 0,
        "an active park must ENTER monitor_wait_block — got {entries} entries"
    );
    assert!(
        timeout + listener + doorbell > 0,
        "an active park must attribute its exits to a wake cause — got timeout={timeout} \
         listener={listener} doorbell={doorbell}"
    );
    // Exact oracle for this graph shape (empty sources, no doorbell): every one
    // of the ITERS park entries exits via the timeout deadline.
    assert_eq!(
        (entries, timeout),
        (ITERS as u64, ITERS as u64),
        "each of {ITERS} park entries must exit via the timeout deadline \
         (empty sources, no doorbell) — got entries={entries} timeout={timeout}"
    );
    assert_eq!(
        (listener, doorbell),
        (0, 0),
        "a pure-Period graph has no listener/doorbell wake source — got \
         listener={listener} doorbell={doorbell}"
    );

    // ANTI-TAUTOLOGY control: park OFF never enters monitor_wait_block, so every
    // counter stays 0 — the independent oracle isolating the park's effect.
    let off_counts = run(MonitorWaitPolicy::off(), ITERS);
    assert_eq!(
        off_counts,
        (0, 0, 0, 0),
        "park-OFF must NEVER enter monitor_wait_block, so all wake counts stay 0 \
         — got {off_counts:?}"
    );
}

/// `run_live` emits the worker-visible "live loop wait policy"
/// line reflecting the ACTIVE park.
///
/// The wait-policy line is emitted once at `run_live` entry, BEFORE the loop —
/// so a `running` flag that starts `false` lets `run_live` emit the line, skip
/// the (empty) loop, emit the exit telemetry, and return `Ok` without spawning.
///
/// ANTI-TAUTOLOGY: assert the field is `park_active=true` (reflecting the ON
/// policy), NOT merely that the line exists — and that `park_active=false` never
/// appears. A regression hardcoding the field would render `false` and fail here.
#[test]
#[traced_test]
#[serial]
fn run_live_emits_wait_policy_line_with_active_park() {
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = ticker_graph(Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test_with_policy(
        config,
        factories,
        clock,
        8,
        // Park ON — the wait-policy line must report park_active=true.
        MonitorWaitPolicy::new(true, false, mwp_ns("policyline")),
    )
    .expect("build ticker graph under park policy");

    // `running` starts false: run_live emits the wait-policy line, runs zero loop
    // iterations, emits the exit telemetry, and returns Ok on this thread.
    let running = std::sync::atomic::AtomicBool::new(false);
    runtime.run_live(&running).expect("run_live returns Ok");

    assert!(
        logs_contain("live loop wait policy"),
        "run_live must emit the worker-visible wait-policy line"
    );
    assert!(
        logs_contain("park_active=true"),
        "the wait-policy line must report the ACTIVE park (park_active=true)"
    );
    assert!(
        !logs_contain("park_active=false"),
        "the field must reflect the ON policy, not a hardcoded false"
    );
    // The exit telemetry line is also emitted (once, at loop exit).
    assert!(
        logs_contain("live loop park telemetry"),
        "run_live must emit the exit park-telemetry summary line"
    );
    runtime.shutdown();
}

// ===========================================================================
// The DEGRADED park tier is the macOS/no-primitive DEFAULT.
//
// Under that default the CLI resolver arms `MonitorWaitPolicy::new(true,
// false, ns)` (park ON, doorbell FORCED OFF) by default for live runs on
// no-primitive targets — exactly the policy this test builds with. On such a target
// the park degrades to the CHUNKED ~100µs bounded sleep-recheck (never a
// busy-spin; a single-sleep alternative measures a timer-coalesced NULL on
// macOS — chunked is the only production shape).
// ===========================================================================

/// End-to-end: a LIVE runtime under the no-primitive DEFAULT policy shape
/// (park ON, doorbell OFF — what the CLI resolver emits on a no-primitive target) PARKS
/// (`park_entry_count_for_test > 0`) and still fires + DELIVERS: the consumer
/// observes the hand oracle `1.0..=N` (never a self-compare). The park is a
/// WAIT primitive — it must change WHEN the loop wakes, never WHAT fires.
/// Also pins the loud first-park tier line (`#[traced_test]`): the degraded
/// tier names its CHUNKED sleep shape (loud over silent).
#[test]
#[traced_test]
#[serial]
fn degraded_default_policy_parks_fires_and_delivers() {
    const N: u64 = 5;
    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = consumer_graph(Arc::clone(&observed), Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test_with_policy(
        config,
        factories,
        clock,
        8,
        // The no-primitive DEFAULT shape: monitor_wait ON, doorbell
        // FORCED OFF (the SHM ring is a no-op stub off Linux).
        MonitorWaitPolicy::new(true, false, mwp_ns("degraded")),
    )
    .expect("build consumer graph under the degraded default policy");

    let mut pubr = {
        let mgr: &Arc<TransportManager> = runtime.test_transport().expect("test transport parked");
        mgr.create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("external publisher must attach to /mwp/ext")
    };

    // Prime (no data): drains connection-lifecycle noise; must not fire.
    runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "the priming drive (no data) must NOT fire the consumer"
    );

    for i in 1..=N {
        publish_one(&mut pubr, i as f64);
        runtime.run_live_step_once_for_test(WAKE_TIMEOUT);
    }

    // The park was ENTERED (the degraded default routes the idle through
    // monitor_wait_block, not the heartbeat).
    assert!(
        runtime.park_entry_count_for_test() > 0,
        "the degraded DEFAULT policy must route the inter-step idle through the \
         park — got 0 park entries"
    );
    // HAND ORACLE: delivery is intact under the degraded park.
    let expected: Vec<f64> = (1..=N).map(|i| i as f64).collect();
    assert_eq!(
        *observed.lock().unwrap(),
        expected,
        "the consumer must observe exactly the hand oracle 1.0..=N under the \
         degraded default park"
    );
    // Loud-over-silent tier pin — degraded tier only: the tier
    // line is gated on `!monitor_wait_available()` inside `monitor_wait_block`,
    // so on a REAL-primitive machine (x86 WAITPKG / aarch64 WFE) the park runs the
    // true CPU monitor-wait and the line NEVER fires — asserting it there would
    // fail on every host that has the primitive. The park-entry + delivery-oracle
    // asserts above stay UNCONDITIONAL (portable across hosts); only the two log pins
    // are no-primitive-scoped.
    if !cerulion_core::monitor_wait::monitor_wait_available() {
        // The FIRST park on a no-primitive target logs the degraded tier ONCE,
        // naming the CHUNKED nap shape AND the mechanism the nap resolved
        // to: the os_sync timed wait when the dlsym backend is live
        // (macOS ≥ 14.4), the plain sleep otherwise. Loud over
        // silent: an operator reading the log knows which slop class the
        // park's recheck pacing carries.
        if cerulion_core::monitor_wait::park_nap_os_sync_available() {
            assert!(
                logs_contain("DEGRADED os_sync-nap recheck tier"),
                "the first park must log the os_sync-nap degraded tier line"
            );
            assert!(
                logs_contain("chunked 100us bounded os_sync_wait_on_address timed wait"),
                "the tier line must name the CHUNKED os_sync nap shape"
            );
        } else {
            assert!(
                logs_contain("DEGRADED sleep-recheck tier"),
                "the first park must log the degraded tier line"
            );
            assert!(
                logs_contain("chunked 100us bounded sleep"),
                "the tier line must name the CHUNKED sleep shape (the production default)"
            );
        }
    }
    runtime.shutdown();
}

/// The `pace_slice` nap-shape regression pin — the
/// degraded park's nap is shaped by `pace_slice` (single vs chunked), NOT
/// the hardcoded 100µs `recheck`. Uses the injectable single-park seam
/// (`GraphRuntime::set_single_park_for_test`) because the hidden
/// `CERULION_MW_SINGLE_PARK` env knob is read through a process-global
/// `OnceLock` — an in-process env A/B is impossible.
///
/// DEGRADED tier only (runtime-gated, the file's machine-portability discipline):
/// on a real-primitive machine (x86 WAITPKG / aarch64 WFE) `performed == true` and
/// the nap arm never runs (the counter stays 0), so the shape is
/// unobservable there — skip.
///
/// MECHANISM-AGNOSTIC on purpose: the counter counts nap CHUNKS
/// whichever mechanism paced each (`thread::sleep` or the macOS os_sync timed
/// wait — `monitor_wait::park_nap` threads the SAME `pace_slice` cap into
/// both), so this pin holds identically on a pre-14.4 Mac, under
/// `CERULION_PARK_OS_SYNC=0`, and on the os_sync tier. WHICH mechanism ran is
/// `degraded_nap_tier_matches_the_resolved_os_sync_availability`'s pin.
///
/// Oracle: ONE live park window (empty sources, no vtime advance, no data —
/// the park runs the whole passed window; `run_live_step_once_for_test`
/// forwards its timeout to the park unclamped, which
/// `park_wake_counts_attribute_pure_period_timeouts` already relies on) naps
/// EXACTLY once under forced single-park (one nap spans the window, then the
/// deadline exit) and MORE THAN once under forced chunked (100µs chunks across
/// the same 50ms window). REVERTING the nap to the raw `recheck` makes the
/// forced-single-park arm chunk the window into many naps and fail HERE —
/// coverage no other arm provides.
#[test]
#[serial]
fn degraded_sleep_shape_honors_park_recheck() {
    if cerulion_core::monitor_wait::monitor_wait_available() {
        eprintln!(
            "skipping degraded_sleep_shape_honors_park_recheck: this machine has a real \
             CPU monitor-wait primitive, so the degraded sleep arm never runs"
        );
        return;
    }
    const WINDOW: Duration = Duration::from_millis(50);

    /// One park window under a forced single-park decision; returns
    /// `(park_entries, recheck_naps)` from a fresh runtime.
    fn run_one_window(forced_single: bool) -> (u64, u64) {
        let fires = Arc::new(AtomicU64::new(0));
        let (config, factories) = ticker_graph(Arc::clone(&fires));
        let clock = Arc::new(VirtualClock::new());
        let mut runtime = GraphRuntime::build_for_test_with_policy(
            config,
            factories,
            clock,
            8,
            MonitorWaitPolicy::new(true, false, mwp_ns("singlepark")),
        )
        .expect("build ticker graph under park policy");
        runtime.set_single_park_for_test(Some(forced_single));
        // No vtime advance, no data: the park runs the whole window and exits
        // at its deadline.
        runtime.run_live_step_once_for_test(WINDOW);
        let out = (
            runtime.park_entry_count_for_test(),
            runtime.park_recheck_nap_count_for_test(),
        );
        runtime.shutdown();
        out
    }

    let (entries_single, naps_single) = run_one_window(true);
    assert_eq!(entries_single, 1, "one live step must park exactly once");
    assert_eq!(
        naps_single, 1,
        "forced single-park must nap EXACTLY once per park window (the \
         `pace_slice` nap shape — a revert to the hardcoded 100us recheck chunks \
         this window into many naps and fails here) — got {naps_single}"
    );

    let (entries_chunked, naps_chunked) = run_one_window(false);
    assert_eq!(entries_chunked, 1, "one live step must park exactly once");
    assert!(
        naps_chunked > 1,
        "the chunked default must nap multiple 100us chunks across a 50ms \
         window — got {naps_chunked}"
    );
}

/// The degraded nap's TIER ROUTING pin — on a no-primitive target every degraded
/// nap chunk must ride the mechanism `park_nap_os_sync_available()` resolves:
/// os_sync-paced when the dlsym backend is live (macOS ≥ 14.4, no kill switch,
/// no unusable-latch — `os_sync_naps == naps`, so routing the nap back through
/// `thread::sleep` fails HERE while every fire/delivery pin stays green — the
/// mutation seam), and NEVER os_sync-paced otherwise (`os_sync_naps == 0`: a
/// pre-14.4 Mac, `CERULION_PARK_OS_SYNC=0`, Linux-without-WAITPKG). Both arms
/// in one runtime-gated body (the file's machine-portability discipline); on a
/// real-primitive machine (x86 WAITPKG / aarch64 WFE) the `!performed` arm never
/// runs — skip, both counters are structurally 0 there.
///
/// FIRE/DATA assertions are deliberately ABSENT here (the firewall pins live
/// in the (a)-(c) tests above and `polled_vs_live_iox2_test`'s parked arm,
/// which run THROUGH the os_sync nap on a no-primitive target); no wall is asserted in
/// units of the recheck (the macOS timer-coalescing hazard) — the only oracle is
/// the record-only counter pair.
#[test]
#[serial]
fn degraded_nap_tier_matches_the_resolved_os_sync_availability() {
    if cerulion_core::monitor_wait::monitor_wait_available() {
        eprintln!(
            "skipping degraded_nap_tier_matches_the_resolved_os_sync_availability: this machine \
             has a real CPU monitor-wait primitive, so the degraded nap arm never runs"
        );
        return;
    }
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = ticker_graph(Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test_with_policy(
        config,
        factories,
        clock,
        8,
        MonitorWaitPolicy::new(true, false, "mwp".into()),
    )
    .expect("build ticker graph under park policy");
    // No vtime advance, no data: the park runs the whole 50ms window in ~100µs
    // nap chunks and exits at its deadline.
    runtime.run_live_step_once_for_test(Duration::from_millis(50));

    let naps = runtime.park_recheck_nap_count_for_test();
    let os_sync_naps = runtime.park_os_sync_nap_count_for_test();
    assert!(
        naps > 1,
        "a 50ms chunked park window must nap multiple recheck chunks — got {naps}"
    );
    // Print-only diagnostic (never asserted — a wall in recheck units is the
    // macOS timer-coalescing flake class): the effective recheck grid this machine's
    // nap tier delivers. Measured on an M3 Max: ~152µs/chunk on the sleep
    // tier (nanosleep +52% overshoot) vs ~127µs/chunk on the os_sync tier.
    eprintln!(
        "degraded park nap tier: {naps} naps over the 50ms window \
         (avg chunk {:.1}us; os_sync-paced: {os_sync_naps})",
        50_000.0 / naps as f64
    );
    if cerulion_core::monitor_wait::park_nap_os_sync_available() {
        assert_eq!(
            os_sync_naps, naps,
            "the os_sync nap tier is resolved on this host, so EVERY degraded nap \
             chunk must ride it (a nap routed back through thread::sleep is the \
             os_sync-tier regression) — got {os_sync_naps} of {naps}"
        );
    } else {
        assert_eq!(
            os_sync_naps, 0,
            "no os_sync nap tier on this host (pre-14.4 / kill switch / latch / \
             non-macOS), so NO nap chunk may claim it — got {os_sync_naps}"
        );
    }
    runtime.shutdown();
}

// ===========================================================================
// The park loop's per-slice YIELD (the same-core starvation defense).
//
// A `UMWAIT`/`WFE`-parked thread is RUNNING to the OS scheduler, so a park
// that slices without yielding holds its core for the whole idle — a
// co-located runnable peer (graph processes > cores under the
// multi-process default) gets it only at CFS wakeup granularity (measured on
// the rmw park, same primitive: p50 6.997ms RTT with both ping-pong
// processes pinned to one CPU, vs 17µs on the kernel-sleeping tier). So
// (mirroring rmw's `park_block`) the hardware arm slices at the
// shared 20µs `monitor_wait::PARK_RECHECK` and EVERY park-loop iteration
// ends with `std::thread::yield_now()` (counted by `park_yields`).
//
// Three pins, deliberately layered:
//  - the COUNTER pin (below, every tier incl. macOS): a parked run
//    completes + counts slices; a never-parked run counts exactly 0.
//  - the STRUCTURAL pin: the counter pin alone passes a variant that keeps the
//    `park_yields` bump but deletes the `yield_now()` itself (the two are
//    output-equivalent on an idle machine), so the source walk requires the call
//    — comment-stripped, so prose cannot satisfy it.
//  - the hardware-only same-core arm (`box_same_core`, Linux `--ignored`): the
//    behavioral kill — park ON, both threads pinned to one CPU, RTT p50
//    < 100µs; removing the yield (unsliced) reads ~ms (one CFS granularity
//    per hop).
// ===========================================================================

/// Counter pin: every completed park-loop slice yields and is
/// counted by `park_yields`, on EVERY tier (macOS runs the degraded
/// sleep tier; a WAITPKG/WFE machine runs the hardware tier — both slice, both
/// count). The park-OFF control is the anti-tautology arm: the counter lives
/// on the park loop's bottom and NOWHERE else, so a never-parked runtime
/// must read exactly 0.
#[test]
#[serial]
fn park_slices_yield_and_are_counted_on_every_tier() {
    const ITERS: usize = 6;

    /// One live-driven ticker run under `policy`; returns
    /// `(park_entries, park_yields, park_recheck_naps)`.
    fn run_counters(policy: MonitorWaitPolicy, iters: usize) -> (u64, u64, u64) {
        let fires = Arc::new(AtomicU64::new(0));
        let (config, factories) = ticker_graph(Arc::clone(&fires));
        let clock = Arc::new(VirtualClock::new());
        let clock_drive = Arc::clone(&clock);
        let mut runtime =
            GraphRuntime::build_for_test_with_policy(config, factories, clock, 8, policy)
                .expect("build ticker graph under policy");
        for _ in 0..iters {
            clock_drive.advance(PERIOD_VTIME_ADVANCE.as_nanos() as u64);
            runtime.run_live_step_once_for_test(PERIOD_PARK_TIMEOUT);
        }
        let out = (
            runtime.park_entry_count_for_test(),
            runtime.park_yield_count_for_test(),
            runtime.park_recheck_nap_count_for_test(),
        );
        runtime.shutdown();
        out
    }

    // Park ON: each of the ITERS live steps parks a ~50ms timer-only window
    // made of slices (20µs hardware / ~100µs degraded nap). A window is
    // hundreds of slices even on the slowest tier, so `> 0` is a huge-margin
    // floor, not a timing race.
    let (entries, yields, naps) = run_counters(
        MonitorWaitPolicy::new(true, false, mwp_ns("yieldcount")),
        ITERS,
    );
    assert_eq!(
        entries, ITERS as u64,
        "apparatus: every live step must enter monitor_wait_block exactly once"
    );
    assert!(
        yields > 0,
        "a parked run that idled to its timeouts must have completed (and \
         yielded after) at least one park slice — park_yields == 0 means the \
         loop-bottom yield/counter is gone (the per-slice yield is missing)"
    );
    assert!(
        yields >= naps,
        "every degraded nap chunk is followed by exactly one loop-bottom \
         yield, so park_yields ({yields}) must be >= park_recheck_naps \
         ({naps}) on every tier (hardware tiers nap 0 times)"
    );

    // Anti-tautology control: a park-OFF, barrier-less runtime NEVER enters
    // monitor_wait_block (it takes the heartbeat path), so its yield counter
    // must stay EXACTLY 0 — a regression counting yields from anywhere but
    // the park loop's bottom fails here.
    let (off_entries, off_yields, _) = run_counters(MonitorWaitPolicy::off(), ITERS);
    assert_eq!(off_entries, 0, "park-OFF control must never park");
    assert_eq!(
        off_yields, 0,
        "park-OFF control must count ZERO yields (the counter lives on the \
         park loop's bottom and nowhere else)"
    );
}

/// Wake-exit pin: the counter pin above
/// drives only TIMEOUT-exiting parks (a source-less ticker), so it cannot
/// see a variant that leaves the EVENT-return branch (`listener_got || …`)
/// exiting without the yield/bump accounting — the exact path a
/// data-flowing robot rides all day. This drives a park that exits via a
/// real LISTENER wake (an external publisher lands a frame MID-PARK) and
/// pins that the slices completed BEFORE the wake still yielded+counted:
/// during a park attributed to a listener/doorbell wake (and NOT to the
/// timeout), `park_yields` must have advanced. Scheduling-robust: if the
/// publish races the park entry (the loop-top poll returns before any
/// slice ran — a wake attribution with a zero yield delta), the attempt is
/// retried rather than misread; a persistent zero across all attempts is
/// the real regression.
#[test]
#[serial]
fn park_wake_exit_still_yields_its_completed_slices() {
    const ATTEMPTS: usize = 5;
    /// The publisher's mid-park delay: long enough that the driving thread
    /// is parked (µs away) by the time the frame lands, short enough to
    /// keep the test fast. The park timeout (400ms) dwarfs it, so a healthy
    /// attempt exits on the WAKE, never the timeout.
    const MID_PARK_DELAY: Duration = Duration::from_millis(20);

    let observed = Arc::new(Mutex::new(Vec::<f64>::new()));
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = consumer_graph(Arc::clone(&observed), Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test_with_policy(
        config,
        factories,
        clock,
        8,
        MonitorWaitPolicy::new(true, false, mwp_ns("wakeexit")),
    )
    .expect("build consumer graph under park policy");
    let mut pubr = {
        let mgr: &Arc<TransportManager> = runtime.test_transport().expect("test transport parked");
        mgr.create_publisher(EXT_TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("external publisher must attach to /mwp/ext")
    };
    // Warm the connection so the measured attempts' publishes deliver (and
    // their SentSample events reach the consumer's listener).
    publish_one(&mut pubr, 0.0);
    runtime.run_live_step_once_for_test(Duration::from_millis(50));

    let mut proven = false;
    for attempt in 0..ATTEMPTS {
        let (entries0, listener0, doorbell0, timeout0) = runtime.park_wake_counts();
        let yields0 = runtime.park_yield_count_for_test();
        let helper = std::thread::spawn(move || {
            std::thread::sleep(MID_PARK_DELAY);
            publish_one(&mut pubr, 1.0 + attempt as f64);
            pubr
        });
        runtime.run_live_step_once_for_test(Duration::from_millis(400));
        pubr = helper.join().expect("mid-park publisher thread");
        let (entries1, listener1, doorbell1, timeout1) = runtime.park_wake_counts();
        let yields1 = runtime.park_yield_count_for_test();
        assert_eq!(entries1 - entries0, 1, "one live step parks exactly once");
        let woke = listener1 + doorbell1 > listener0 + doorbell0;
        let timed_out = timeout1 > timeout0;
        if woke && !timed_out {
            // The park exited on the EVENT branch. The ~20ms it idled first
            // is >= ~200 slices even on the slowest (degraded 100µs) tier,
            // so a zero delta on a wake-attributed park means the wake-exit
            // accounting was skipped — unless the publish raced the
            // park entry (loop-top poll returned with ZERO completed
            // slices), which the retry absorbs.
            if yields1 > yields0 {
                proven = true;
                break;
            }
            eprintln!(
                "wake-exit attempt {attempt}: wake with zero yield delta \
                 (publish likely raced the park entry) — retrying"
            );
        } else {
            eprintln!(
                "wake-exit attempt {attempt}: park exit not wake-attributed \
                 (woke={woke}, timed_out={timed_out}) — retrying"
            );
        }
    }
    runtime.shutdown();
    assert!(
        proven,
        "across {ATTEMPTS} attempts, no LISTENER-woken park showed a yield \
         delta — the event-return path is exiting without yielding/counting \
         its completed slices (wake-exit regression)"
    );
}

/// STRUCTURAL pin (mirrors rmw's `the_park_slice_yields_the_core`
/// in `guard_wait.rs`): the counter pin above passes a variant that keeps the
/// `park_yields` bump but deletes the `yield_now()` call — the two are
/// output-equivalent on an idle machine (the cost is a ~7ms same-core wall only
/// on an oversubscribed one), and the behavioral kill (`box_same_core`) is
/// hardware-only. So walk the SOURCE of `monitor_wait_block` — comment-stripped,
/// so a call mentioned in prose or commented out cannot satisfy it — for the
/// yield, its counter, and the shared 20µs hardware slice.
#[test]
fn the_native_park_slice_yields_the_core() {
    let src = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/src/graph/runtime.rs"))
        .expect("read runtime.rs");
    let start = src
        .find("fn monitor_wait_block(")
        .expect("monitor_wait_block definition present");
    let end = src[start..]
        .find("fn wake_ahead(")
        .map(|e| start + e)
        .expect("wake_ahead follows monitor_wait_block");
    let body = code_only(&src[start..end]);
    // The yield must live inside the recheck loop —
    // the per-slice contract. A variant that hoists it before the loop
    // (keeping the counter bump inside, or vice versa) passes both a
    // whole-fn containment check and the counter pin, so the walk is scoped
    // to the loop body.
    assert_eq!(
        body.matches("loop {").count(),
        1,
        "monitor_wait_block should contain exactly ONE recheck loop — if \
         that changed, re-scope this pin to the park loop"
    );
    let loop_at = body.find("loop {").expect("recheck loop present");
    // The loop extent is brace-matched, not
    // end-of-function — a check that anchors only the start at `loop {` lets a
    // yield hoisted AFTER the loop's closing brace (the fn epilogue; today
    // that is unreachable code, but the assertion must not rest on that)
    // still pass. Braces are counted over the comment-stripped body;
    // string literals are NOT modeled — none inside `monitor_wait_block`
    // contains a brace today, and a brace in a future string could only
    // move the matched close EARLIER, failing the yield-INSIDE assert
    // loudly rather than silently widening the extent.
    let mut depth = 0usize;
    let mut loop_end = None;
    for (i, b) in body.bytes().enumerate().skip(loop_at) {
        match b {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    loop_end = Some(i + 1);
                    break;
                }
            }
            _ => {}
        }
    }
    let loop_end = loop_end.expect("the recheck loop's closing brace is inside the fn body");
    let loop_body = &body[loop_at..loop_end];
    let after_loop = &body[loop_end..];
    assert!(
        loop_body.contains("std::thread::yield_now()"),
        "monitor_wait_block must yield the core INSIDE the recheck loop, \
         after every park slice (a UMWAIT/WFE-parked thread is \
         RUNNING to the scheduler, so a non-yielding park starves a \
         co-located peer for a CFS tick)"
    );
    assert!(
        !body[..loop_at].contains("std::thread::yield_now()"),
        "the per-slice yield must not (also) sit BEFORE the recheck loop — \
         a pre-loop yield runs once per park ENTRY, not once per slice"
    );
    assert!(
        !after_loop.contains("std::thread::yield_now()"),
        "the per-slice yield must not sit AFTER the recheck loop's closing \
         brace — the loop only exits by `return`, so a post-loop yield never \
         runs on the park path at all"
    );
    // The counter bump rides the yield, inside the loop too — and it is
    // GATED on a genuinely-completed wait (`if parked_slice`): the addr
    // park's RingPending lost-wakeup fast path performs no wait and must not
    // count. (rustfmt splits receiver/call across lines, so compare
    // whitespace-free.)
    let compact_loop: String = loop_body.chars().filter(|c| !c.is_whitespace()).collect();
    assert!(
        compact_loop.contains("ifparked_slice{self.park_yields.fetch_add(1"),
        "…and count it (park_yields) inside the loop, GATED on a completed \
         wait (`if parked_slice {{ … }}`) — an unconditional bump counts the \
         lost-wakeup fast path as a phantom slice"
    );
    let compact_after: String = after_loop.chars().filter(|c| !c.is_whitespace()).collect();
    assert!(
        !compact_after.contains("self.park_yields.fetch_add(1"),
        "…and the bump must not sit after the loop either — unreachable on \
         the park path, so the counter would read 0 forever"
    );
    assert!(
        body.contains("monitor_wait::PARK_RECHECK"),
        "the HARDWARE park slice must be the shared 20µs \
         monitor_wait::PARK_RECHECK (not the 100µs pacing chunk) — the slice \
         bound is the other half of the same-core starvation defense"
    );
}

/// Strip `//` line comments and (nesting) `/* */` block comments, so the
/// structural pin above cannot be satisfied by prose. String literals are
/// deliberately not modeled (no walked token appears inside one in
/// `monitor_wait_block`). Oracle-tested by `code_only_strips_comments`.
fn code_only(src: &str) -> String {
    let mut out = String::with_capacity(src.len());
    let b = src.as_bytes();
    let mut i = 0;
    let mut depth = 0usize;
    while i < b.len() {
        if depth == 0 && b[i..].starts_with(b"//") {
            while i < b.len() && b[i] != b'\n' {
                i += 1;
            }
            continue;
        }
        if b[i..].starts_with(b"/*") {
            depth += 1;
            i += 2;
            continue;
        }
        if depth > 0 && b[i..].starts_with(b"*/") {
            depth -= 1;
            i += 2;
            continue;
        }
        if depth == 0 {
            out.push(b[i] as char);
        }
        i += 1;
    }
    out
}

/// `code_only`'s own oracle — a broken stripper would make the structural
/// pin vacuous (asserting over an empty/garbled body) or porous (a
/// commented-out yield passing).
#[test]
fn code_only_strips_comments() {
    assert_eq!(code_only("a(); // b()\nc();"), "a(); \nc();");
    assert_eq!(code_only("a(); /* b() */ c();"), "a();  c();");
    assert_eq!(code_only("a /* x /* y */ z */ b"), "a  b");
    assert_eq!(
        code_only("keep(); // std::thread::yield_now()\n"),
        "keep(); \n",
        "a yield named only in a comment must not survive the strip"
    );
}

// ===========================================================================
// The hardware-only same-core arm (Linux, `--ignored`).
// ===========================================================================
#[cfg(target_os = "linux")]
mod box_same_core {
    use super::*;
    use std::sync::atomic::AtomicBool;
    use std::time::Instant;

    /// Producer-less absolute ping topic (External provisioning): the driver
    /// attaches a raw external publisher; the parked graph's data-trigger
    /// consumer reads it.
    const PING_TOPIC: &str = "/samecore/ping";
    /// The ponger's graph-owned derived output topic
    /// (`/{prefix}/{node}/{output}`).
    const PONG_TOPIC: &str = "/samecoreg/ponger/out";

    /// Connection warm-up rounds (iceoryx2 connection establishment + the
    /// first send's `update_connections`) — excluded from the measured window.
    const WARMUP_ROUNDS: usize = 20;
    /// Measured RTT rounds.
    const ROUNDS: usize = 300;
    /// Per-round liveness bound — a lost pong fails loudly, never hangs.
    const ROUND_DEADLINE: Duration = Duration::from_secs(2);

    /// Data-trigger replier: forwards each ping's payload onto its pong
    /// output within the same fire.
    #[cerulion_node]
    #[derive(Default)]
    struct SameCorePonger {
        #[input(trigger)]
        inp: Vector3,
        #[output]
        out: Vector3,
    }

    #[cerulion_node_impl]
    impl SameCorePonger {
        fn tick(&mut self) -> Result<(), NodeError> {
            self.out.x = self.inp.x;
            Ok(())
        }
    }

    /// The first CPU this process may run on (the affinity mask is what a
    /// container / CI runner grants, so the pin is always permitted).
    fn first_allowed_cpu() -> Option<usize> {
        // SAFETY: `set` is a live zeroed `cpu_set_t` of exactly the size
        // passed; pid 0 = the calling thread.
        unsafe {
            let mut set: libc::cpu_set_t = std::mem::zeroed();
            if libc::sched_getaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &mut set) != 0 {
                return None;
            }
            (0..libc::CPU_SETSIZE as usize).find(|&i| libc::CPU_ISSET(i, &set))
        }
    }

    /// Pin the CALLING thread to `cpu`.
    fn pin_current_thread(cpu: usize) -> bool {
        // Defensive bound: a wild `PIN_CORE` must fail the pin loudly
        // (the caller asserts on `false`), never index past the fixed bitset.
        if cpu >= libc::CPU_SETSIZE as usize {
            return false;
        }
        // SAFETY: `set` is a live zeroed `cpu_set_t`; `CPU_SET` writes only
        // within its fixed-size bit array (`cpu < CPU_SETSIZE` checked above);
        // `sched_setaffinity` reads exactly the size passed; pid 0 = the
        // calling thread. `CPU_SET` is an `unsafe fn` on current libc (an
        // E0133 error otherwise), so the whole body shares one unsafe block — the same
        // shape as `first_allowed_cpu`.
        unsafe {
            let mut set: libc::cpu_set_t = std::mem::zeroed();
            libc::CPU_SET(cpu, &mut set);
            libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set) == 0
        }
    }

    /// Median duration of a bare timer-bounded hardware park requesting a
    /// 20µs slice — the machine's REAL park quantum. On x86 `UMWAIT` honors the
    /// TSC deadline so this reads ~the requested slice; on aarch64 base
    /// `WFE` has no timeout operand and parks ONE generic-timer event-stream
    /// period per call (machine-specific; a Jetson measures ~131µs — a same-core
    /// round-trip costs ~2-3 of them), which is therefore the wake QUANTUM
    /// the platform bound calibrates to.
    fn measured_park_quantum() -> Duration {
        const CAL_ROUNDS: usize = 50;
        let word: u64 = 0;
        let mut samples: Vec<Duration> = Vec::with_capacity(CAL_ROUNDS);
        for _ in 0..CAL_ROUNDS {
            let t0 = Instant::now();
            // SAFETY: `word` is a live, 8-byte-aligned u64 on this frame,
            // mapped for the whole call; `0` is its current value, so the
            // lost-wakeup early-out cannot fire and the park really runs.
            let outcome = unsafe {
                cerulion_core::monitor_wait::monitor_wait_until_addr(
                    &word as *const u64,
                    0,
                    t0 + Duration::from_millis(5),
                    Duration::from_micros(20),
                )
            };
            assert_eq!(
                outcome,
                cerulion_core::monitor_wait::AddrParkOutcome::Parked,
                "calibration requires a real, completed hardware park"
            );
            samples.push(t0.elapsed());
        }
        samples.sort();
        samples[CAL_ROUNDS / 2]
    }

    /// Hardware pin: with the parked live loop AND its driver pinned to
    /// ONE CPU, park ON (spin irrelevant — the park path never spins), a
    /// ping→pong RTT through the parked data-trigger consumer must hold p50
    /// under the PLATFORM bound. Without the per-slice
    /// yield (unsliced-through-the-window hardware park), a parked thread is
    /// RUNNING to CFS, so the driver gets the core back only at wakeup
    /// granularity — the rmw twin measured p50 6.997ms that way; with the slice yield
    /// x86 measured p50 47.758µs (~147x) and a Jetson p50 344.142µs.
    ///
    /// PLATFORM bound (calibrated from measurement): x86 `UMWAIT`
    /// honors the 20µs TSC deadline ⇒ p50 < 100µs, unchanged. aarch64 base
    /// `WFE` has NO timeout operand — a bounded wait rides the generic-timer
    /// event stream, whose period is machine-specific (kernel target ~100µs,
    /// power-of-two divider of CNTFRQ; a Jetson measures ~131µs — a measured
    /// 393µs band was THREE handoffs, 3x131µs, not one period). A same-core
    /// ROUND-TRIP structurally costs ~2-3 wake handoffs (the parked side
    /// holds the core to a slice boundary before the driver can even
    /// publish, then needs the core back to reply): a Jetson measured p50
    /// = 2.63x the quantum with the slice counter independently reading
    /// 2.59 slices/round — the same number from two instruments. So the ARM
    /// bound is HANDOFF_K (4) x the quantum MEASURED at test start
    /// (`measured_park_quantum`) — ~52% headroom over the structural 2.63x
    /// without weakening intent (a no-yield park reads ~7ms, orders above
    /// 4x131µs = 524µs) — floored at the x86 100µs (an interrupt-shortened
    /// calibration must never make ARM stricter than x86). The catastrophe
    /// backstop is max(1ms, the platform bound), so a large-quantum machine
    /// stays coherent. A SECOND, load-robust oracle pins the geometry
    /// directly: park slices completed per MEASURED round (a live
    /// `park_yields` snapshot over exactly the measured window — the
    /// post-stop tail parks up to one whole 50ms timeout window, which
    /// would smear thousands of unrelated slices into a naive total/rounds
    /// figure) must stay <= MAX_SLICES_PER_ROUND (6): "the park wakes at
    /// its granularity". Without the yield, the park burns the driver's whole
    /// CFS wait in slices (~53/round at a 131µs quantum, ~350/round at 20µs). A
    /// sub-100µs ARM same-core wake is wake-word/futex-arm or
    /// event-stream-divider-knob material, which this park does not attempt.
    ///
    /// The driver side is deliberately COOPERATIVE (`yield_now` per poll),
    /// so the only thing that can hold the core is the park under test — one
    /// runtime, one mechanism, no second park to confound the attribution.
    ///
    /// Hardware-only (`--ignored`): needs real WAITPKG/WFE silicon + exclusive
    /// use of one core (skips loudly on a Linux machine with no primitive — the
    /// degraded nap tier releases the core, so the hazard is absent). On a
    /// machine with the primitive the run must RIDE the hardware arm: zero degraded nap
    /// slices (`park_recheck_naps`), else the nap fallback — which
    /// releases the core on its own — would pass the bounds without the
    /// slice yield under test.
    /// `PIN_CORE=N` overrides the pinned CPU. Prints
    /// p50/p90/p99/max + the measured quantum.
    #[test]
    #[serial]
    #[ignore = "box-run (Linux): pins the parked live loop and its driver to one CPU"]
    fn box_same_core_parked_live_loop_rtt_holds_the_platform_bound() {
        if !cerulion_core::monitor_wait::monitor_wait_available() {
            eprintln!(
                "same-core arm: no hardware monitor-wait primitive on \
                 this machine — the degraded sleep tier releases the core (the \
                 hazard this arm pins is absent); skipping"
            );
            return;
        }
        let cpu = first_allowed_cpu().expect("read this process's CPU affinity mask");
        let cpu = std::env::var("PIN_CORE")
            .ok()
            .and_then(|v| v.parse::<usize>().ok())
            .unwrap_or(cpu);

        // Platform bound (see the fn doc). Calibrated BEFORE the runtime is
        // built, so the measurement is a bare uncontended park. K covers the
        // ~2-3 wake handoffs a same-core round structurally costs (measured
        // 2.63x on a Jetson) with margin to spare.
        const HANDOFF_K: u32 = 4;
        let (threshold, quantum_note) = if cfg!(target_arch = "x86_64") {
            (Duration::from_micros(100), String::new())
        } else {
            let quantum = measured_park_quantum();
            (
                (quantum * HANDOFF_K).max(Duration::from_micros(100)),
                format!(" (measured park quantum {quantum:?}, K={HANDOFF_K})"),
            )
        };
        // Catastrophe line: never below the platform bound, so a
        // large-quantum machine stays coherent (on x86 this is the plain 1ms).
        let backstop = threshold.max(Duration::from_millis(1));

        // The parked context: one data-trigger ponger, park FORCED ON,
        // doorbell OFF — the wake is the per-slice listener poll, the shape
        // that isolates slice cadence + yield (a doorbell would add a second
        // wake mechanism without changing what a missing yield starves).
        let config = GraphConfig {
            execution: None,
            level_assignments: None,
            network: None,
            process_groups: Default::default(),
            process_group_order: Default::default(),
            multi_publisher_topics: Vec::new(),
            name: None,
            identity: "same_core_ponger_ctx".to_string(),
            prefix: "samecoreg".to_string(),
            nodes: vec![NodeDef {
                fuse: None,
                ros2: None,
                id: "ponger".to_string(),
                node_type: "same_core_ponger".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: PING_TOPIC.to_string(),
                }],
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
        factories.insert("ponger".to_string(), Box::new(SameCorePongerEntry::new()));
        let clock = Arc::new(VirtualClock::new());
        let mut runtime = GraphRuntime::build_for_test_with_policy(
            config,
            factories,
            clock,
            8,
            MonitorWaitPolicy::new(true, false, mwp_ns("samecore")),
        )
        .expect("build parked ponger context");

        let mgr = Arc::clone(runtime.test_transport().expect("test transport present"));
        let mut pinger = mgr
            .create_publisher(PING_TOPIC, MaxSliceLen::const_new(256), 0)
            .expect("external ping publisher on the producer-less absolute source");
        let pong_sub = mgr
            .create_subscriber_open_only(PONG_TOPIC)
            .expect("pong tap on the graph-owned output");

        let stop = Arc::new(AtomicBool::new(false));
        let stop_a = Arc::clone(&stop);
        // Live snapshot of the parker's cumulative `park_yields`, published
        // after every live-step iteration — lets the driver read the slice
        // count over EXACTLY the measured window (see the fn doc's
        // slice-geometry oracle).
        let yields_live = Arc::new(AtomicU64::new(0));
        let yields_live_parker = Arc::clone(&yields_live);
        let parker = std::thread::spawn(move || {
            assert!(
                pin_current_thread(cpu),
                "pin the parked live loop to cpu{cpu}"
            );
            while !stop_a.load(Ordering::Relaxed) {
                runtime.run_live_step_once_for_test(Duration::from_millis(50));
                yields_live_parker.store(runtime.park_yield_count_for_test(), Ordering::Relaxed);
            }
            let yields = runtime.park_yield_count_for_test();
            let naps = runtime.park_recheck_nap_count_for_test();
            runtime.shutdown();
            (yields, naps)
        });
        assert!(pin_current_thread(cpu), "pin the driver to cpu{cpu}");

        // Warm-up (connections; not timed), then the measured lockstep rounds:
        // one ping in flight at a time, the driver COOPERATIVE (yield per
        // poll) so only the park under test can hold the core.
        let mut rtts: Vec<Duration> = Vec::with_capacity(ROUNDS);
        let mut yields_at_measure_start = 0u64;
        for round in 0..(WARMUP_ROUNDS + ROUNDS) {
            if round == WARMUP_ROUNDS {
                // The slice-geometry window opens here: the last warm-up
                // pong is in and the parker has published its cumulative
                // count (modulo one in-flight iteration — noise at the
                // /ROUNDS scale).
                yields_at_measure_start = yields_live.load(Ordering::Relaxed);
            }
            let t0 = Instant::now();
            publish_one(&mut pinger, round as f64);
            loop {
                if pong_sub
                    .try_receive_one(|_| {})
                    .expect("receive on the pong tap")
                {
                    break;
                }
                assert!(
                    t0.elapsed() < ROUND_DEADLINE,
                    "round {round}: no pong within {ROUND_DEADLINE:?} — the \
                     parked loop is not serving pings at all"
                );
                std::thread::yield_now();
            }
            if round >= WARMUP_ROUNDS {
                rtts.push(t0.elapsed());
            }
        }
        let yields_at_measure_end = yields_live.load(Ordering::Relaxed);
        stop.store(true, Ordering::Relaxed);
        let (parker_yields, parker_naps) = parker.join().expect("join the parked live loop");
        let slices_per_round =
            (yields_at_measure_end - yields_at_measure_start) as f64 / ROUNDS as f64;

        rtts.sort();
        let pct = |q: f64| rtts[((rtts.len() - 1) as f64 * q) as usize];
        let (p50, p90, p99) = (pct(0.5), pct(0.9), pct(0.99));
        let max = rtts[rtts.len() - 1];
        eprintln!(
            "same-core native park (cpu{cpu}): p50 {p50:?} p90 {p90:?} \
             p99 {p99:?} max {max:?} over {ROUNDS} rounds; parker slices/yields \
             {parker_yields} total, {slices_per_round:.2}/round in the measured \
             window; platform bound {threshold:?}{quantum_note}"
        );
        assert!(
            parker_yields > 0,
            "apparatus: the parked loop must have completed (and yielded after) \
             park slices while idling between pings"
        );
        // The availability gate at entry says a hardware
        // primitive EXISTS; this proves the park actually RODE it — zero
        // degraded nap slices across the whole run. A park that fell to the
        // nap fallback releases the core by sleeping (or, on macOS >= 14.4,
        // an os_sync timed wait), so its same-core RTT would pass
        // the bounds WITHOUT the slice yield under test — the fallback
        // invalidates the measurement, so it fails loudly instead.
        assert_eq!(
            parker_naps, 0,
            "the native park took {parker_naps} DEGRADED nap slices on a \
             machine whose monitor-wait primitive probes available — the same-core \
             measurement must ride the HARDWARE arm"
        );
        // Absolute catastrophe backstop first, on EVERY platform: the failure
        // class this arm exists to catch (a non-yielding/unsliced park's CFS
        // wall) is ~7ms; the event-stream class is hundreds of µs. The line
        // scales with the platform bound so a large-quantum machine stays
        // coherent (on x86 it is the plain 1ms).
        assert!(
            p50 < backstop,
            "same core, park ON: ping→pong p50 {p50:?} breaches the \
             {backstop:?} catastrophe backstop — the CFS-wall class (a \
             non-yielding/unsliced park holds the core against its co-located \
             driver; the rmw twin measured p50 6.997ms)"
        );
        // The platform bound (see the fn doc): x86 keeps the strict 100µs;
        // aarch64 is calibrated to HANDOFF_K x the machine's measured
        // event-stream quantum (a same-core round structurally costs ~2.6
        // handoff quanta), floored at the x86 bound.
        assert!(
            p50 < threshold,
            "same core, park ON: ping→pong p50 {p50:?} vs platform bound \
             {threshold:?}{quantum_note} — the park is holding the core \
             against its co-located driver longer than its own wake \
             granularity allows"
        );
        // The slice-geometry oracle — sharper and load-robust: the park must
        // wake at its granularity, ~2-3 slices per same-core round (measured:
        // x86 ~2.35, a Jetson 2.59). Without the yield, the park burns the
        // driver's whole CFS wait in slices (~53/round at a 131µs quantum,
        // ~350/round at 20µs).
        const MAX_SLICES_PER_ROUND: f64 = 6.0;
        assert!(
            slices_per_round <= MAX_SLICES_PER_ROUND,
            "same core, park ON: {slices_per_round:.2} park slices per \
             measured round (cap {MAX_SLICES_PER_ROUND}) — the parked loop is \
             slicing far past its wake granularity, i.e. holding the core \
             while its driver waits"
        );
    }
}
