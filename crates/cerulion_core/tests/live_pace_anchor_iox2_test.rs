// SPDX-License-Identifier: AGPL-3.0-only
//! The live loop's wait sizing under a CONTROLLED gating clock, over real
//! iceoryx2 (per-test SHM roots — parallel-safe).
//!
//! `live_timeout` sizes the next wait from `Scheduler::ns_until_next_fire(now)`,
//! whose Period deadlines live in the GATING domain. A lockstep worker's gating
//! clock starts at 0 and advances by the handed quantum per step, so reading
//! `CLOCK_MONOTONIC` (boot uptime) as `now` made every deadline look hours
//! past-due: the wait floored to 1 ms and a `period_ms = 20` producer published
//! at ~1 kHz until logical time caught up with uptime (`cerulion graph run` on
//! `examples/obstacle_avoidance` measured ~950 Hz for ~6 min, then 50 Hz). The
//! runtime now anchors `(wall, gating)` at `run_live` entry and compares
//! `gating_at_entry + wall_elapsed` against the deadlines instead.
//!
//! Two pins, each with the bug it catches:
//!
//! * **Sizing**: a freshly built `period_ms = 4` lockstep participant reports a
//!   ~4 ms wait, not the 1 ms floor — before AND after the anchor is taken (the
//!   pre-anchor read uses the gating clock itself).
//! * **Pace**: driving the production `live_step` with its own `live_timeout`
//!   for ~120 ms of wall fires the ticker at its 4 ms period (~30×), not once
//!   per 1 ms floor (~120×) — and the gating clock advanced exactly one
//!   handed quantum per fire, so the anchor changed WHEN the loop woke, never
//!   WHAT fired.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::barrier::MappedBarrier;
use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

const PERIOD_MS: u64 = 4;
const PERIOD_NS: u64 = PERIOD_MS * 1_000_000;
/// The wait the 1 ms floor would report — the bug's signature.
const FLOOR: Duration = Duration::from_millis(1);
const PERIOD: Duration = Duration::from_millis(PERIOD_MS);

#[cerulion_node(period_ms = 4)]
#[derive(Default)]
struct Ticker {
    #[output]
    out: Vector3,
    fires: Arc<AtomicU64>,
    /// A one-shot stall (ns) the NEXT tick sleeps for — the over-quantum
    /// callback. Swapped to 0 as it is consumed.
    stall_ns: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl Ticker {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        self.fires.fetch_add(1, Ordering::Relaxed);
        let stall = self.stall_ns.swap(0, Ordering::Relaxed);
        if stall > 0 {
            std::thread::sleep(Duration::from_nanos(stall));
        }
        Ok(())
    }
}

fn ticker_graph(
    prefix: &str,
    fires: Arc<AtomicU64>,
    stall_ns: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("{prefix}_ticker"),
        prefix: prefix.to_string(),
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
            fires,
            stall_ns,
            ..Default::default()
        })),
    );
    (config, factories)
}

fn manager(node_name: &str, clock: Arc<dyn Clock>) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: node_name.into(),
            clock,
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init per-test transport")
}

/// A one-rank LOCKSTEP participant (the `cerulion graph run` worker shape): a
/// `VirtualClock` gating clock starting at 0, handed quantum = the period.
fn lockstep_ticker(
    tag: &str,
    fires: Arc<AtomicU64>,
) -> (GraphRuntime, Arc<VirtualClock>, Arc<TransportManager>) {
    lockstep_ticker_stalling(tag, fires, Arc::new(AtomicU64::new(0)))
}

fn lockstep_ticker_stalling(
    tag: &str,
    fires: Arc<AtomicU64>,
    stall_ns: Arc<AtomicU64>,
) -> (GraphRuntime, Arc<VirtualClock>, Arc<TransportManager>) {
    lockstep_ticker_with_quantum(tag, fires, stall_ns, PERIOD)
}

/// The handed quantum is the GLOBAL min over every group's tightest timing
/// (`expect_within_ms`, bounded Sync windows, ...), so a Period node routinely
/// runs under a quantum FINER than its own interval.
fn lockstep_ticker_with_quantum(
    tag: &str,
    fires: Arc<AtomicU64>,
    stall_ns: Arc<AtomicU64>,
    quantum: Duration,
) -> (GraphRuntime, Arc<VirtualClock>, Arc<TransportManager>) {
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager(&format!("lpa_{tag}"), clock.clone());
    let (config, factories) = ticker_graph(&format!("lpa{tag}"), fires, stall_ns);
    let ns = format!("lpa_{tag}_{}", std::process::id());
    let barrier = Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("barrier owner"));
    let rt = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        barrier,
        vec![Some(0)],
        vec![false],
        0,
        quantum,
        cerulion_core::graph::runtime::CrossProcessWiring::requirements_only(None),
    )
    .expect("build the lockstep ticker");
    (rt, clock, mgr)
}

#[test]
fn a_lockstep_participant_waits_its_period_not_the_floor() {
    let (mut rt, clock, _mgr) = lockstep_ticker("size", Arc::new(AtomicU64::new(0)));
    assert_eq!(clock.now_ns(), 0, "a lockstep gating clock starts at 0");

    // Before the loop anchors: the deadline domain is the gating clock itself.
    let before = rt.live_timeout_for_test();
    assert!(
        before > FLOOR && before <= PERIOD,
        "pre-anchor live_timeout {before:?} must be the ~4 ms Period deadline, not the 1 ms \
         floor a CLOCK_MONOTONIC `now` (boot uptime ≫ deadline 4 ms) forces"
    );

    // At `run_live` entry: gating_at_entry (0) + wall elapsed (~0).
    rt.anchor_live_pace_for_test();
    let after = rt.live_timeout_for_test();
    assert!(
        after > FLOOR && after <= PERIOD,
        "anchored live_timeout {after:?} must be the ~4 ms Period deadline, not the 1 ms floor"
    );
    rt.shutdown();
}

#[test]
fn a_lockstep_participant_fires_at_its_period_under_the_live_loop() {
    let fires = Arc::new(AtomicU64::new(0));
    let (mut rt, clock, _mgr) = lockstep_ticker("pace", Arc::clone(&fires));
    rt.anchor_live_pace_for_test();

    // Drive the production live step with the timeout `run_live` would use.
    let budget = Duration::from_millis(120);
    let start = Instant::now();
    let mut steps = 0u64;
    while start.elapsed() < budget {
        let timeout = rt.live_timeout_for_test();
        rt.run_live_step_once_for_test(timeout);
        steps += 1;
    }
    let elapsed = start.elapsed();
    let fired = fires.load(Ordering::Relaxed);

    // ~30 fires in 120 ms at 4 ms. The floor bug fires once per 1 ms step
    // (~120): a generous ceiling of 2× the period rate still rejects it, and the
    // lower bound only proves the ticker ran (a stalled runner cannot flake it
    // into the floor arm).
    let period_rate = elapsed.as_nanos() as u64 / PERIOD_NS;
    assert!(
        fired >= 3,
        "the ticker must fire under the live loop (fired {fired} in {elapsed:?}, {steps} steps)"
    );
    assert!(
        fired <= period_rate * 2,
        "a 4 ms ticker fired {fired}× in {elapsed:?} ({steps} steps) — more than 2× its \
         period rate ({period_rate}); the live loop is pacing on the 1 ms floor, i.e. \
         comparing gating-domain deadlines against CLOCK_MONOTONIC"
    );
    // The anchor moved only WHEN the loop woke: the gating timeline still
    // advanced one handed quantum per fire.
    assert_eq!(
        clock.now_ns(),
        fired * PERIOD_NS,
        "the lockstep gating clock must advance exactly one handed quantum per fire"
    );
    rt.shutdown();
}

/// A 4 ms Period under a 1 ms handed quantum: gating must cross FOUR quantum
/// boundaries per fire, and the requested wait must be capped at the quantum.
///
/// Before anchoring, `live_timeout` reads only the controlled gating clock. Its
/// exact 1 ms result rejects the missing boundary cap (which requests 4 ms),
/// regardless of descheduling. Anchored reads also include elapsed wall time,
/// so an overdue deadline could hide that bug behind the 1 ms timeout floor.
///
/// Drive a fixed sequence of real live steps against handwritten gating/fire
/// oracles. A wall-throughput floor, even calibrated on the same thread just
/// beforehand, assumes CPU availability stays constant between the two windows.
/// CI repeatedly violated that assumption while still firing once per four steps
/// Keep the upper rate bound: descheduling only makes it easier
/// to satisfy. This test checks wait sizing and logical cadence, not a minimum
/// wall-throughput guarantee on a shared runner.
#[test]
fn a_period_over_a_finer_quantum_still_fires_at_its_period() {
    const QUANTUM: Duration = Duration::from_millis(1);
    // Expected (gating nanoseconds, cumulative fires) after each live step.
    const EXPECTED: [(u64, u64); 16] = [
        (1_000_000, 0),
        (2_000_000, 0),
        (3_000_000, 0),
        (4_000_000, 1),
        (5_000_000, 1),
        (6_000_000, 1),
        (7_000_000, 1),
        (8_000_000, 2),
        (9_000_000, 2),
        (10_000_000, 2),
        (11_000_000, 2),
        (12_000_000, 3),
        (13_000_000, 3),
        (14_000_000, 3),
        (15_000_000, 3),
        (16_000_000, 4),
    ];

    let fires = Arc::new(AtomicU64::new(0));
    let (mut rt, clock, _mgr) = lockstep_ticker_with_quantum(
        "fineq",
        Arc::clone(&fires),
        Arc::new(AtomicU64::new(0)),
        QUANTUM,
    );
    assert_eq!(
        rt.live_timeout_for_test(),
        QUANTUM,
        "before anchoring, a 4 ms Period over a 1 ms quantum must request exactly 1 ms"
    );

    rt.anchor_live_pace_for_test();
    let first = rt.live_timeout_for_test();
    assert!(
        first <= QUANTUM,
        "the first anchored wait under a 1 ms quantum must be ≤ 1 ms, got {first:?}"
    );

    let start = Instant::now();
    for (expected_ns, expected_fires) in EXPECTED {
        let timeout = rt.live_timeout_for_test();
        rt.run_live_step_once_for_test(timeout);
        assert_eq!(
            clock.now_ns(),
            expected_ns,
            "each live step must advance exactly one handed quantum"
        );
        assert_eq!(
            fires.load(Ordering::Relaxed),
            expected_fires,
            "the 4 ms ticker must fire only at every fourth quantum boundary ({expected_ns} ns)"
        );
    }
    let elapsed = start.elapsed();
    let fired = fires.load(Ordering::Relaxed);
    // Cross-multiply rather than rounding elapsed time down to whole periods:
    // the same 2x ceiling remains valid over this shorter, fixed-step window.
    assert!(
        u128::from(fired) * u128::from(PERIOD_NS) <= elapsed.as_nanos() * 2,
        "a 4 ms ticker fired {fired}× in {elapsed:?} — more than 2× its period rate"
    );
    rt.shutdown();
}

/// Drive the production live step until the ticker has fired `n` times (or
/// `budget` of wall elapses). Returns the number of steps taken.
fn step_until_fired(rt: &mut GraphRuntime, fires: &AtomicU64, n: u64, budget: Duration) -> u64 {
    let start = Instant::now();
    let mut steps = 0u64;
    while fires.load(Ordering::Relaxed) < n && start.elapsed() < budget {
        let timeout = rt.live_timeout_for_test();
        rt.run_live_step_once_for_test(timeout);
        steps += 1;
    }
    steps
}

/// A callback that overruns its handed quantum (3× here) leaves wall time a
/// whole quantum ahead of the gating clock. The loop must SLIP — size the next
/// wait from the gating clock as it stands (≈ period) — not pay the deficit
/// back as a run of 1 ms-floor wakes.
#[test]
fn an_over_quantum_callback_slips_the_pace_instead_of_bursting() {
    let fires = Arc::new(AtomicU64::new(0));
    let stall = Arc::new(AtomicU64::new(0));
    let (mut rt, clock, _mgr) = lockstep_ticker_stalling("slip", Arc::clone(&fires), stall.clone());
    rt.anchor_live_pace_for_test();

    // Settle into the paced regime, then arm a 12 ms stall for the next tick.
    step_until_fired(&mut rt, &fires, 2, Duration::from_secs(2));
    assert_eq!(
        fires.load(Ordering::Relaxed),
        2,
        "the ticker must reach the paced regime"
    );
    stall.store(3 * PERIOD_NS, Ordering::Relaxed);
    step_until_fired(&mut rt, &fires, 3, Duration::from_secs(2));
    assert_eq!(
        fires.load(Ordering::Relaxed),
        3,
        "the stalling tick must have run"
    );
    assert_eq!(
        stall.load(Ordering::Relaxed),
        0,
        "the stall must have been consumed"
    );

    // Right after the slow step: gating = 3 quanta, wall ≈ 5 quanta since anchor.
    let after_slow = rt.live_timeout_for_test();
    assert!(
        after_slow > FLOOR && after_slow <= PERIOD,
        "after a 12 ms callback under a 4 ms quantum the next wait must be ≈ the period, \
         got {after_slow:?} — the mapped wall clock outran the gating clock and the loop \
         is about to burst on the 1 ms floor"
    );

    // And the fires that follow stay at the period rate: no catch-up burst.
    let before = fires.load(Ordering::Relaxed);
    let t0 = Instant::now();
    let steps = step_until_fired(&mut rt, &fires, before + 5, Duration::from_secs(2));
    let elapsed = t0.elapsed();
    let fired = fires.load(Ordering::Relaxed) - before;
    assert_eq!(
        fired, 5,
        "the ticker must keep firing after the stall ({steps} steps)"
    );
    assert!(
        elapsed >= PERIOD * 4,
        "5 fires after the stall took {elapsed:?} ({steps} steps) — fewer than 4 periods: \
         the loop paid the overrun back as a catch-up burst"
    );
    assert_eq!(clock.now_ns(), fires.load(Ordering::Relaxed) * PERIOD_NS);
    rt.shutdown();
}

/// A paused run (no steps for many quanta) re-anchors at `run_live` re-entry:
/// the pause is not paid back as a catch-up burst either.
#[test]
fn a_resumed_run_re_anchors_instead_of_replaying_the_pause() {
    let fires = Arc::new(AtomicU64::new(0));
    let (mut rt, clock, _mgr) = lockstep_ticker("resume", Arc::clone(&fires));
    rt.anchor_live_pace_for_test();
    step_until_fired(&mut rt, &fires, 2, Duration::from_secs(2));
    assert_eq!(fires.load(Ordering::Relaxed), 2);

    // "Pause": the loop is not driven for 10 quanta of wall.
    std::thread::sleep(PERIOD * 10);
    let stale = rt.live_timeout_for_test();
    assert_eq!(
        stale, FLOOR,
        "precondition: without re-anchoring, the stale anchor reports every deadline past-due"
    );

    // Resume = re-enter `run_live`, which re-anchors.
    rt.anchor_live_pace_for_test();
    let resumed = rt.live_timeout_for_test();
    assert!(
        resumed > FLOOR && resumed <= PERIOD,
        "a resumed run must wait ≈ its period, got {resumed:?}"
    );
    let t0 = Instant::now();
    step_until_fired(&mut rt, &fires, 6, Duration::from_secs(2));
    assert_eq!(fires.load(Ordering::Relaxed), 6);
    assert!(
        t0.elapsed() >= PERIOD * 3,
        "4 fires after resume took {:?} — the pause was replayed as a burst",
        t0.elapsed()
    );
    assert_eq!(clock.now_ns(), 6 * PERIOD_NS);
    rt.shutdown();
}

/// The mirror drift: a run of early wakes (a backlogged Data/Sync input, a
/// doorbell, a barrier peer — modelled as zero-timeout steps) advances the
/// gating clock a full quantum per step while consuming almost no wall time.
/// Once traffic quiets, the next Period deadline must still be ≈ one period
/// away in wall terms — not parked out at the liveliness-sweep cap while the
/// stale anchor waits for wall to catch up to the gating clock.
#[test]
fn a_run_of_early_wakes_slips_the_pace_instead_of_stalling() {
    const EARLY_STEPS: u64 = 25; // 100 ms of gating in a few ms of wall
    let fires = Arc::new(AtomicU64::new(0));
    let (mut rt, clock, _mgr) = lockstep_ticker("early", Arc::clone(&fires));
    rt.anchor_live_pace_for_test();

    let t0 = Instant::now();
    for _ in 0..EARLY_STEPS {
        rt.run_live_step_once_for_test(Duration::ZERO);
    }
    let burst = t0.elapsed();
    assert_eq!(clock.now_ns(), EARLY_STEPS * PERIOD_NS);
    assert!(
        burst < PERIOD * (EARLY_STEPS as u32) / 2,
        "precondition: zero-timeout steps must outrun wall, took {burst:?}"
    );

    // The slip re-anchors once drift reaches a quantum, so at most one quantum
    // of residual drift remains: the next wait is bounded by TWO periods, not by
    // the ≈100 ms of gating the burst ran ahead (or the 250 ms sweep cap).
    let after_burst = rt.live_timeout_for_test();
    assert!(
        after_burst <= PERIOD * 2,
        "after gating outran wall by ≈{:?} the next wait must be ≤ 2 periods, got {after_burst:?} — \
         the stale anchor is waiting for wall to catch up to the gating clock",
        PERIOD * (EARLY_STEPS as u32) - burst
    );

    let before = fires.load(Ordering::Relaxed);
    let t1 = Instant::now();
    step_until_fired(&mut rt, &fires, before + 5, Duration::from_secs(2));
    let elapsed = t1.elapsed();
    assert_eq!(fires.load(Ordering::Relaxed), before + 5);
    assert!(
        elapsed < PERIOD * 5 * 3,
        "5 fires after the burst took {elapsed:?} — the loop is stalled at the sweep cap"
    );
    assert_eq!(clock.now_ns(), fires.load(Ordering::Relaxed) * PERIOD_NS);
    rt.shutdown();
}

/// Determinism: the anchor changes only WHEN the loop wakes. Two runs driven for
/// the same number of steps — one paced, one with an over-quantum stall (so its
/// wake schedule differs) — produce the identical fire count and gating
/// timeline.
#[test]
fn two_runs_with_different_wake_schedules_are_bit_identical_in_the_gating_domain() {
    const STEPS: u64 = 12;
    let mut outcomes = Vec::new();
    for (tag, stall_at) in [("det_a", None), ("det_b", Some(3u64))] {
        let fires = Arc::new(AtomicU64::new(0));
        let stall = Arc::new(AtomicU64::new(0));
        let (mut rt, clock, _mgr) =
            lockstep_ticker_stalling(tag, Arc::clone(&fires), Arc::clone(&stall));
        rt.anchor_live_pace_for_test();
        for step in 0..STEPS {
            if stall_at == Some(step) {
                stall.store(3 * PERIOD_NS, Ordering::Relaxed);
            }
            let timeout = rt.live_timeout_for_test();
            rt.run_live_step_once_for_test(timeout);
        }
        outcomes.push((fires.load(Ordering::Relaxed), clock.now_ns()));
        rt.shutdown();
    }
    assert_eq!(
        outcomes[0], outcomes[1],
        "paced vs stalled runs must agree on (fires, gating_ns) after {STEPS} steps"
    );
    assert_eq!(outcomes[0], (STEPS, STEPS * PERIOD_NS));
}
