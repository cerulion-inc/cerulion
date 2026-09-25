// SPDX-License-Identifier: AGPL-3.0-only
//! An `expect_within_ms` window that lapses on a per-message FIFO
//! TRIGGER input carrying unconsumed arrivals is BACKLOG, not silence.
//!
//! `expect_within_ms` is documented as a producer-liveness detector — "N ms
//! elapsed without new data on this input". Since data-trigger inputs became
//! per-message FIFO, the watchdog measured something else on a DEFERRED node:
//! the age of the frame the tick was served. The boundary drain freezes the
//! FIFO HEAD and re-offers it (with its ORIGINAL wire timestamp) on every
//! boundary until a tick takes it, so a `throttle_ms` / `block` defer longer
//! than the window grew `elapsed` past `within_ns` while newer frames sat
//! queued on the very input being called silent — a per-window `warn!` and a
//! climbing counter on a perfectly healthy graph.
//!
//! The rule under test: such a window is reported (a `debug!` per window plus
//! a once-per-regime `info!` head and close) and counted in the DISJOINT
//! `NodeHandle::expect_within_backlogged_count` bucket, never in
//! `expect_within_missed_count`, and it emits no `ExpectWithinEvent` — an
//! `#[on_event]` handler that fails over on staleness must not fire while
//! unconsumed frames are queued on that input.
//!
//! Every oracle here is HAND-DERIVED (never a self-compare), and the headline
//! arms FAIL without the FIFO trigger (nonzero misses on a healthy stream).
//!
//! Scope boundary, pinned rather than assumed: a Data node's NON-trigger
//! latest-value inputs share the same node-level pending count but are fed by
//! unrelated producers, and the held-context staleness detector lives
//! on exactly those — so only the marked FIFO trigger input is ever
//! suppressed.
//!
//! `#[serial]`: one arm mutates the process-global `CERULION_DRAIN_DISCIPLINE`
//! env and two install a global `tracing` subscriber; the whole file runs
//! serially for simplicity.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test expect_within_fifo_iox2_test -- --test-threads=1
//! ```

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{
    BackpressurePolicy, ClosureNodeEntry, InputMeta, NodeEntry, NodeInfo,
};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::testing::debug_lines_expected;
use cerulion_core::MacroPolicy;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing_test::traced_test;

/// RAII env guard (panic-safe removal) for the drain-discipline seam.
struct EnvVarGuard(&'static str);
impl EnvVarGuard {
    fn set(key: &'static str, value: &str) -> Self {
        std::env::set_var(key, value);
        Self(key)
    }
}
impl Drop for EnvVarGuard {
    fn drop(&mut self) {
        std::env::remove_var(self.0);
    }
}

/// The absolute external source every consumer-only graph here consumes.
const EXT_TOPIC: &str = "/ewf/ext";
/// The absolute external source the non-trigger CONTEXT input consumes.
const CTX_TOPIC: &str = "/ewf/ctx";

/// A 10 ms in-graph producer — the "healthy stream" of arms 1/2/7.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct SteadyProducer {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl SteadyProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// A 1 ms in-graph producer — guarantees a permanent trigger backlog against
/// any throttled consumer.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct FloodProducer {
    #[output]
    out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl FloodProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

fn trigger_input(expect_within_ms: Option<u64>) -> InputMeta {
    InputMeta {
        name: "inp".to_string(),
        schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
        trigger: true,
        depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
        backpressure: BackpressurePolicy::DropOldest,
        expect_within_ms,
    }
}

fn consumer_info(expect_within_ms: u64, throttle_ms: Option<u64>) -> NodeInfo {
    let info = NodeInfo::with_meta(vec![trigger_input(Some(expect_within_ms))], vec![])
        .with_policy(MacroPolicy::DataTrigger {
            input_name: "inp".to_string(),
        });
    match throttle_ms {
        Some(ms) => info.with_throttle_ms(ms),
        None => info,
    }
}

/// A consumer that reads its trigger input on every fire, recording values.
fn reading_consumer(
    expect_within_ms: u64,
    throttle_ms: Option<u64>,
    seen: Arc<Mutex<Vec<u64>>>,
) -> ClosureNodeEntry {
    ClosureNodeEntry::new(consumer_info(expect_within_ms, throttle_ms), move |ctx| {
        if let Some(s) = ctx.subscriber_mut("inp") {
            if let Ok(Some(v)) = s.try_view::<Vector3, _>(|view| view.x as u64) {
                seen.lock().unwrap().push(v);
            }
        }
        Ok(())
    })
    .with_label("ewf_consumer")
}

fn consumer_only_graph(prefix: &str) -> GraphConfig {
    GraphConfig {
        execution: None,
        name: None,
        identity: "ewf".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "sink".to_string(),
            node_type: "ewf_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: EXT_TOPIC.to_string(),
            }],
            outputs: vec![],
        }],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    }
}

fn producer_consumer_graph(prefix: &str, producer_type: &str) -> GraphConfig {
    GraphConfig {
        execution: None,
        name: None,
        identity: "ewf".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "prod".to_string(),
                node_type: producer_type.to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "geometry_msgs/Vector3".to_string(),
                    topic: None,
                    max_slice_len: None,
                    history_size: 0,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "sink".to_string(),
                node_type: "ewf_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "prod/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    }
}

/// Publish `values` as `Vector3.x` frames from an external publisher.
fn publish_all(
    p: &mut cerulion_core::transport::publisher::CerulionPublisher,
    values: impl IntoIterator<Item = u64>,
) {
    for v in values {
        let mut proxy = p.loan_proxy::<Vector3>().expect("loan");
        proxy.x = v as f64;
        drop(proxy);
    }
}

/// The two disjoint watchdog buckets plus the fire count.
#[derive(Debug, PartialEq, Eq)]
struct Verdict {
    missed: u64,
    backlogged: u64,
    fires: u64,
}

/// Drive `steps` × 1 ms of a `SteadyProducer` (10 ms) feeding a consumer
/// throttled to one fire per `throttle_ms`, and report the consumer's verdict.
fn run_steady(prefix: &str, throttle_ms: u64, expect_within_ms: u64, steps: u32) -> Verdict {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("prod".to_string(), Box::new(SteadyProducerEntry::new()));
    factories.insert(
        "sink".to_string(),
        Box::new(reading_consumer(
            expect_within_ms,
            Some(throttle_ms),
            Arc::clone(&seen),
        )),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(
        producer_consumer_graph(prefix, "SteadyProducer"),
        factories,
        clock,
        16,
    )
    .expect("build steady graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(1));
    }
    let h = runtime.node_handle("sink").unwrap();
    Verdict {
        missed: h.expect_within_missed_count(),
        backlogged: h.expect_within_backlogged_count(),
        fires: h.fire_count(),
    }
}

// ===========================================================================
// (1) HEADLINE — a healthy stream into a throttled consumer counts NO miss
// ===========================================================================

/// A 10 ms producer feeding a consumer capped to one fire per 100 ms: the
/// consumer is 10x behind BY ITS OWN DECLARATION, and its trigger input is
/// never short of data for a single step after the first frame lands. The
/// 50 ms window therefore lapses repeatedly — but every lapse is BACKLOG.
///
/// Hand oracle, all three parts load-bearing:
///   * `missed == 0` — the contract. Without it this is ~7 (one per 50 ms window
///     across the 400 ms run, minus the fires that re-anchor it), each with
///     its own `warn!`.
///   * `backlogged > 0` — anti-vacuity: the window really did lapse, so
///     `missed == 0` is not merely "nothing ever happened".
///   * `fires >= 2` — the throttle really deferred and the node really ran.
#[test]
#[serial]
fn a_throttled_consumer_on_a_healthy_stream_counts_no_expect_within_miss() {
    let v = run_steady("ewfa", 100, 50, 400);
    assert_eq!(
        v.missed, 0,
        "a window that lapsed while unconsumed arrivals were queued on the \
         trigger input is BACKLOG, not producer silence — it must not count \
         an expect_within miss (got {v:?})"
    );
    assert!(
        v.backlogged > 0,
        "anti-vacuity: the 50 ms window MUST have lapsed under a 100 ms \
         throttle, so the backlog bucket cannot be empty (got {v:?})"
    );
    assert!(
        v.fires >= 2,
        "the consumer must actually run (and be deferred between runs) for \
         this shape to be the one under test (got {v:?})"
    );
}

// ===========================================================================
// (2) The same, on the FORCED-SEPARATE drain discipline
// ===========================================================================

/// The Separate discipline reaches the same false positive by a DIFFERENT
/// mechanism, so a Unified-only fix would not be a fix. There is no frozen
/// slot at the boundary: the trigger-drain subscriber stores the NEWEST
/// timestamp, and then the tick body's own `try_view` pops the FIFO head and
/// rewrites the shared anchor BACKWARDS to that older stamp. `run_qos_windows`
/// detects an arrival with `!=`, not `>`, so the window start rewinds and the
/// next quiet step lapses.
///
/// Same oracle as arm 1. The guard keys on `pending_data_count` and the
/// `fifo_trigger` mark comes from `data_trigger_bindings` — both driven
/// identically by the two disciplines — so the VERDICT is discipline-
/// independent. The COUNTS are not (Separate signals once per drained
/// timestamp, Unified once per boundary), which is why nothing here compares
/// counts across the two legs.
#[test]
#[serial]
fn a_throttled_consumer_on_the_forced_separate_discipline_counts_no_miss_either() {
    let _guard = EnvVarGuard::set("CERULION_DRAIN_DISCIPLINE", "separate");
    let v = run_steady("ewfb", 100, 50, 400);
    assert_eq!(
        v.missed, 0,
        "the forced-Separate leg must reach the same verdict — its anchor \
         REGRESSION is a different mechanism for the same false positive \
         (got {v:?})"
    );
    assert!(
        v.backlogged > 0,
        "anti-vacuity on the Separate leg: the window must really lapse and \
         really be suppressed (got {v:?})"
    );
    assert!(v.fires >= 2, "the consumer must actually run (got {v:?})");
}

// ===========================================================================
// (3) A producer that STOPS is still detected, once the backlog drains
// ===========================================================================

/// The real cost of backlog suppression, pinned with a two-sided band: while the input
/// carries a backlog its liveliness watchdog is quiet, so a producer that
/// dies mid-backlog is detected only after the backlog DRAINS plus one more
/// window.
///
/// Shape: six frames land at t = 0 and the producer then stops forever; the
/// consumer is capped to one fire per 5 ms against a 50 ms window.
///
/// The oracle is anchored on an OBSERVED state transition rather than on a
/// hand-traced step number, so it survives a change to how many pending
/// arrivals one step consumes: `drained_step` is the first step after which
/// `pending_data_count()` reads 0, and the scheduler re-anchors the window at the
/// NEXT step's QoS check (the first one that SEES an empty backlog). So the
/// first miss must land at `drained_step + 1 + within + 1`, and the band
/// below allows ±2 steps of modelling slack around it.
///
/// The lower edge is what kills the missing drain-out edge: without it the
/// window is still measured from the last SUPPRESSED window's advance (here
/// t = 102 ms), so the first miss arrives ~50 steps EARLY — a spurious miss
/// charged to a producer that had already stopped being observable, and on a
/// still-flowing producer, one spurious miss per drained burst.
#[test]
#[serial]
fn a_stopped_producer_still_trips_the_watchdog_after_the_backlog_drains() {
    const WITHIN_MS: u32 = 50;
    // The throttle must EXCEED the window, or there is no backlogged window to
    // observe: at a cap well under the window the drain finishes before the
    // window ever elapses (see the fast-drain twin below). A 5 ms
    // stimulus sees suppressed windows only if
    // re-offer inflation keeps `pending` non-zero long after the queue empties —
    // it would be riding the very defect the mint gate closes.
    const THROTTLE_MS: u64 = 100;
    // 6 frames at one fire per 100 ms need ~501 steps to drain, then one 50 ms
    // window to trip.
    const STEPS: u32 = 700;
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "sink".to_string(),
        Box::new(reading_consumer(
            WITHIN_MS as u64,
            Some(THROTTLE_MS),
            Arc::clone(&seen),
        )),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(consumer_only_graph("ewfc"), factories, clock, 16)
            .expect("build stopped-producer graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher(EXT_TOPIC, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");
    publish_all(&mut ext, [1, 2, 3, 4, 5, 6]);

    let handle = runtime.node_handle("sink").unwrap().clone();
    let mut drained_step: Option<u32> = None;
    let mut first_miss_step: Option<u32> = None;
    for step in 1..=STEPS {
        runtime.step(Duration::from_millis(1));
        // The drain is DELIVERY, not `pending == 0`: the mint
        // gate stops re-offers from inflating the count, so `pending`
        // legitimately returns to 0 between fires while frames are still
        // queued, and a `pending == 0` probe reports the drain hundreds of steps early.
        if drained_step.is_none() && seen.lock().unwrap().len() as u64 == 6 {
            drained_step = Some(step);
        }
        if first_miss_step.is_none() && handle.expect_within_missed_count() > 0 {
            first_miss_step = Some(step);
        }
    }

    let backlogged = handle.expect_within_backlogged_count();
    assert!(
        backlogged > 0,
        "anti-vacuity: the 50 ms window must have lapsed under the backlog \
         before it drained (backlogged = {backlogged})"
    );
    let drained = drained_step.expect("the six-frame backlog must drain within the run");
    let first_miss = first_miss_step.expect("a stopped producer MUST eventually trip the watchdog");
    assert_eq!(
        *seen.lock().unwrap(),
        vec![1, 2, 3, 4, 5, 6],
        "every published frame must still reach its own fire, in order"
    );
    let expected = drained + 1 + WITHIN_MS + 1;
    assert!(
        (expected - 2..=expected + 2).contains(&first_miss),
        "the first miss must be measured from the DRAIN instant \
         (drained at step {drained} ⇒ expected ≈ step {expected}), not from \
         the stamp of the oldest frame the node was only just served — got \
         step {first_miss}"
    );
}

/// A DEFER must not INFLATE the arrival count, or a dead producer stays
/// suppressed for dozens of throttle periods.
///
/// The mechanism, on the Unified drain discipline: `decide_node` runs the
/// `throttle_ms` pre-fire check at its TOP and returns `None`, so a deferred
/// step consumes NOTHING; the tick never runs, so the frozen FIFO head is never
/// taken; and the next boundary RE-OFFERS that same head, reporting
/// `popped = 1`. An ungated mint therefore added one signalled arrival per
/// DEFERRED STEP for ONE held frame, saturating `pending_data_count` at
/// `DATA_PENDING_CARRY_CLAMP` (64) and draining at one per throttle period —
/// so the backlog guard kept the watchdog quiet for up to 64 throttle periods
/// after the producer died (6.4 s at 100 ms; 64 s at 1000 ms), against the
/// "plus at most one more window" bound `NodeHandle` documents.
///
/// The oracle is the ONE number that separates the two designs: the step at
/// which the first miss lands after the queue is genuinely empty. This is a
/// LONG throttle (100 ms) against 1 ms steps, so an inflated count is worth
/// tens of seconds while the correct one is worth one window — a separation of
/// two orders of magnitude, not a tolerance band.
///
/// `pending_data_count()` is asserted directly as well, because the miss timing
/// alone would also be satisfied by a fix that broke the DEFER instead (firing
/// through the throttle would drain the queue early and trip the watchdog on
/// time for the wrong reason); the delivery oracle holds that side down too.
#[test]
#[serial]
fn a_defer_does_not_inflate_the_arrival_count_so_a_dead_producer_is_found_on_time() {
    const WITHIN_MS: u64 = 50;
    const THROTTLE_MS: u64 = 100;
    const FRAMES: u64 = 6;
    // 6 frames at one fire per 100 ms need ~501 steps to all be served; the
    // tail leaves room for the post-drain window (50 ms) plus, under a
    // re-offer-counting design, the 64-period suppression it would have to spend.
    const STEPS: u32 = 900;

    let seen = Arc::new(Mutex::new(Vec::new()));
    let consumer = reading_consumer(WITHIN_MS, Some(THROTTLE_MS), Arc::clone(&seen));
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("sink".to_string(), Box::new(consumer));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(consumer_only_graph("ewfl"), factories, clock, 16)
            .expect("build throttled graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher(EXT_TOPIC, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");
    publish_all(&mut ext, 1..=FRAMES);
    // The producer DIES by going silent, not by tearing its port down: dropping
    // the publisher releases the single-writer slot and its data segment, which
    // is a different event and takes the queued frames with it. Arm 3 models it
    // the same way.
    let _ext = ext;

    let handle = runtime.node_handle("sink").unwrap().clone();
    let mut peak_pending: u64 = 0;
    let mut drained_step: Option<u32> = None;
    let mut first_miss_step: Option<u32> = None;
    for step in 1..=STEPS {
        runtime.step(Duration::from_millis(1));
        peak_pending = peak_pending.max(handle.pending_data_count());
        if drained_step.is_none() && seen.lock().unwrap().len() as u64 == FRAMES {
            drained_step = Some(step);
        }
        if first_miss_step.is_none() && handle.expect_within_missed_count() > 0 {
            first_miss_step = Some(step);
        }
    }

    assert_eq!(
        *seen.lock().unwrap(),
        (1..=FRAMES).collect::<Vec<_>>(),
        "every frame must still reach its own throttled fire, in order — the \
         mint gate must not cost a fire"
    );
    assert!(
        handle.expect_within_backlogged_count() > 0,
        "anti-vacuity: the 50 ms window must have lapsed under the backlog \
         while the 100 ms throttle held the head (backlogged = {})",
        handle.expect_within_backlogged_count()
    );

    // THE headline. One held frame is ONE signalled arrival, however many
    // boundaries re-offer it; a re-offer-counting design reaches the clamp of 64.
    assert!(
        peak_pending <= 1,
        "a re-offered head must not mint a fresh arrival: one held frame is one \
         signalled arrival however long the defer lasts, but the peak count \
         reached {peak_pending} (a re-offer-counting design saturates at \
         DATA_PENDING_CARRY_CLAMP = 64, which is what suppresses the watchdog \
         for 64 throttle periods)"
    );

    let drained = drained_step.expect("the six-frame backlog must drain within the run");
    let first_miss = first_miss_step.expect("a dead producer MUST trip the watchdog");
    // Measured from the DRAIN instant (the falling edge re-anchors there, since
    // this episode really did suppress windows), so the correct answer is one
    // window later. A re-offer-counting design owes 63 further throttle periods of
    // suppression first — 6.3 s against this 1 ms step, i.e. past the end of
    // the run — so the band cannot be met by accident.
    let expected = drained + 1 + WITHIN_MS as u32 + 1;
    assert!(
        (expected - 2..=expected + 2).contains(&first_miss),
        "a producer that dies mid-backlog must be found within ONE window of \
         the real drain (drained at step {drained} ⇒ expected ≈ step \
         {expected}), not after the inflated arrival count has bled off one \
         throttle period at a time — got step {first_miss}"
    );
}

/// The SUPPRESSED branch leaves `tracker.armed` UNTOUCHED, and this is the only
/// arm that can see it.
///
/// The scheduler's backlog branch counts the window and `continue`s WITHOUT
/// disarming, and the code states the consequence as a contract: the first
/// genuinely-silent window after the backlog drains still fires exactly one
/// `ExpectWithinEvent`. Nothing asserted that event. A one-line change adding
/// `tracker.armed = false;` to the suppressed branch passes every other arm on
/// this branch AND the pre-existing suite, while destroying the reactive
/// failover FOREVER on that input — the miss COUNTER keeps climbing (so
/// `a_stopped_producer_still_trips_the_watchdog_after_the_backlog_drains`
/// stays green, since it reads only `expect_within_missed_count`) but no
/// handler ever runs again.
///
/// Why the two neighbouring arms structurally cannot cover it:
/// `a_non_trigger_input_on_a_data_node_is_not_suppressed` observes a trigger
/// event under a PERMANENT backlog and asserts it is ZERO — which that change
/// also satisfies — and its `ctx` input is not `fifo_trigger`, so it never reaches
/// the suppressed branch at all.
///
/// Shape: the throttled-drain shape of arm 3 (a finite burst, `throttle_ms`
/// well under the window so the backlog really drains), but the oracle is the
/// EVENT rather than the counter. EXACTLY ONE, because the emission is
/// edge-triggered: `armed` gates it and the miss branch clears it, so a second
/// event would mean the latch re-armed from somewhere it should not have.
#[test]
#[serial]
fn a_producer_that_dies_mid_backlog_fires_exactly_one_event_once_the_backlog_drains() {
    const WITHIN_MS: u64 = 50;
    // Throttle > window, so the drain really is slower than the window and the
    // suppressed branch is genuinely reached before the backlog clears.
    const THROTTLE_MS: u64 = 100;
    const STEPS: u32 = 700;
    let events = Arc::new(AtomicU64::new(0));
    let events_c = Arc::clone(&events);
    let seen = Arc::new(Mutex::new(Vec::new()));
    let seen_c = Arc::clone(&seen);

    // The reading consumer of arm 3, plus an event drain. Reading the input is
    // load-bearing: the backlog has to actually DRAIN for the falling edge to
    // fire, and a tick that never took the head would hold it forever.
    let consumer = ClosureNodeEntry::new(consumer_info(WITHIN_MS, Some(THROTTLE_MS)), move |ctx| {
        if let Some(s) = ctx.subscriber_mut("inp") {
            if let Ok(Some(v)) = s.try_view::<Vector3, _>(|view| view.x as u64) {
                seen_c.lock().unwrap().push(v);
            }
        }
        if ctx.take_expect_within_event("inp").is_some() {
            events_c.fetch_add(1, Ordering::Relaxed);
        }
        Ok(())
    })
    .with_label("ewf_consumer");

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("sink".to_string(), Box::new(consumer));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(consumer_only_graph("ewfk"), factories, clock, 16)
            .expect("build stopped-producer graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher(EXT_TOPIC, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");
    // The producer publishes six frames and then DIES — nothing else is ever
    // published on this topic for the rest of the run.
    publish_all(&mut ext, [1, 2, 3, 4, 5, 6]);
    // Dies by going SILENT — see the note in the sibling arm.
    let mut ext_live = ext;

    let handle = runtime.node_handle("sink").unwrap().clone();
    for _ in 0..STEPS {
        runtime.step(Duration::from_millis(1));
    }

    // Snapshot what the RUN served, before the wake frame below joins the list:
    // the wake exists only to drain an already-minted event, so it must not be
    // part of the evidence that the backlog drained.
    let served_by_the_run = seen.lock().unwrap().clone();

    // The event was minted at the lapse and QUEUED. A Data node whose producer
    // died never ticks again, so `take_expect_within_event` — which runs inside
    // the tick — would never drain it and the count would read 0 no matter what
    // the scheduler did. Wake the node ONCE with a single frame, purely to let
    // the already-minted event out. This cannot manufacture the result: the
    // arrival RE-ARMS the latch but nothing lapses afterwards, so a second event
    // is impossible, and a run that never minted one still reads 0.
    publish_all(&mut ext_live, [99]);
    for _ in 0..(THROTTLE_MS as u32 + 20) {
        runtime.step(Duration::from_millis(1));
    }
    // Anti-vacuity, both halves: the window must really have lapsed UNDER the
    // backlog (or the suppressed branch was never reached and the arm proves
    // nothing about `armed`), and the backlog must really have DRAINED (or the
    // input never left the suppressed branch and one event is not evidence the
    // latch survived it).
    assert!(
        handle.expect_within_backlogged_count() > 0,
        "anti-vacuity: the window must have lapsed under the backlog before it \
         drained (backlogged = {})",
        handle.expect_within_backlogged_count()
    );
    assert_eq!(
        served_by_the_run,
        vec![1, 2, 3, 4, 5, 6],
        "anti-vacuity: every frame must have been served, so the backlog really \
         drained and the input really went silent"
    );
    assert!(
        handle.expect_within_missed_count() > 0,
        "anti-vacuity: a stopped producer must trip the watchdog after the \
         drain (missed = {})",
        handle.expect_within_missed_count()
    );

    assert_eq!(
        events.load(Ordering::Relaxed),
        1,
        "the first genuinely-silent window after the backlog drains must fire \
         EXACTLY ONE ExpectWithinEvent: the suppressed branch leaves `armed` \
         untouched (a backlog is not a regime boundary), and the miss branch \
         then disarms it — so zero means the backlog silently consumed the \
         latch and this input's reactive failover is dead forever, and more \
         than one means the latch re-armed without a real arrival (got {})",
        events.load(Ordering::Relaxed)
    );
}

// ===========================================================================
// (4) An idle trigger input with an EMPTY queue still trips
// ===========================================================================

/// The negative control for the whole feature: with no backlog there is
/// nothing to suppress, so a silent producer trips the watchdog exactly as
/// before. `backlogged == 0` is the half that kills a guard reading the
/// pending count as permanently non-zero.
#[test]
#[serial]
fn an_idle_trigger_input_with_an_empty_queue_still_trips() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "sink".to_string(),
        Box::new(reading_consumer(50, None, Arc::clone(&seen))),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(consumer_only_graph("ewfd"), factories, clock, 16)
            .expect("build idle graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher(EXT_TOPIC, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");
    publish_all(&mut ext, [1]);
    for _ in 0..300 {
        runtime.step(Duration::from_millis(1));
    }
    let h = runtime.node_handle("sink").unwrap();
    let (missed, backlogged) = (
        h.expect_within_missed_count(),
        h.expect_within_backlogged_count(),
    );
    // One frame served at step 1, then 300 ms of true silence against a
    // 50 ms window ⇒ a miss roughly every 51 steps.
    assert!(
        missed >= 3,
        "an empty trigger queue is SILENCE and must still trip the watchdog \
         (missed = {missed})"
    );
    assert_eq!(
        backlogged, 0,
        "there is no backlog to suppress here — a non-zero backlog bucket \
         means the guard treats a drained input as permanently backlogged \
         (backlogged = {backlogged})"
    );
}

// ===========================================================================
// (5) A NON-trigger input of the same Data node is NOT suppressed
// ===========================================================================

/// The scoping pin, and the only arm that can see it: the node-level counter
/// cannot separate inputs, so the oracle is the PER-INPUT
/// `ExpectWithinEvent`, which carries the input name.
///
/// The consumer is throttled to one fire per 10 ms against a 1 ms producer,
/// so its `trig` input carries a permanent backlog and its 20 ms window is
/// permanently suppressed. Its `ctx` input is a plain latest-value read fed
/// by one external frame and then nothing — genuine silence against a 30 ms
/// window, on a node whose pending count is never zero.
///
/// Oracle: the `ctx` handler fires (>= 1) and the `trig` handler never does.
/// A guard that dropped the `fifo_trigger` conjunct — or a wiring that marked
/// every watched input — suppresses `ctx` too and drives its count to 0.
#[test]
#[serial]
fn a_non_trigger_input_on_a_data_node_is_not_suppressed() {
    let ctx_events = Arc::new(AtomicU64::new(0));
    let trig_events = Arc::new(AtomicU64::new(0));
    let ctx_c = Arc::clone(&ctx_events);
    let trig_c = Arc::clone(&trig_events);

    let info = NodeInfo::with_meta(
        vec![
            InputMeta {
                name: "trig".to_string(),
                schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
                trigger: true,
                depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
                backpressure: BackpressurePolicy::DropOldest,
                expect_within_ms: Some(20),
            },
            InputMeta {
                name: "ctx".to_string(),
                schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
                trigger: false,
                depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
                backpressure: BackpressurePolicy::DropOldest,
                expect_within_ms: Some(30),
            },
        ],
        vec![],
    )
    .with_policy(MacroPolicy::DataTrigger {
        input_name: "trig".to_string(),
    })
    // 30 ms cap against a 1 ms producer and a depth-10 queue: `drop_oldest`
    // pins the FIFO head at most ~9 ms behind the newest frame, and it is then
    // HELD for the 30 ms defer, so the trigger's 20 ms window is certain to
    // lapse WITH a backlog. A cap at or below the window would leave the head
    // too fresh to ever lapse and the arm would prove nothing.
    .with_throttle_ms(30);

    let consumer = ClosureNodeEntry::new(info, move |ctx| {
        if let Some(s) = ctx.subscriber_mut("trig") {
            let _ = s.try_view::<Vector3, _>(|view| view.x);
        }
        if let Some(s) = ctx.subscriber_mut("ctx") {
            let _ = s.try_view::<Vector3, _>(|view| view.x);
        }
        if ctx.take_expect_within_event("ctx").is_some() {
            ctx_c.fetch_add(1, Ordering::Relaxed);
        }
        if ctx.take_expect_within_event("trig").is_some() {
            trig_c.fetch_add(1, Ordering::Relaxed);
        }
        Ok(())
    })
    .with_label("ewf_two_input_consumer");

    let config = GraphConfig {
        execution: None,
        name: None,
        identity: "ewf".to_string(),
        prefix: "ewfe".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "prod".to_string(),
                node_type: "FloodProducer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "geometry_msgs/Vector3".to_string(),
                    topic: None,
                    max_slice_len: None,
                    history_size: 0,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "sink".to_string(),
                node_type: "ewf_two_input_consumer".to_string(),
                inputs: vec![
                    InputDef {
                        name: "trig".to_string(),
                        source: "prod/out".to_string(),
                    },
                    InputDef {
                        name: "ctx".to_string(),
                        source: CTX_TOPIC.to_string(),
                    },
                ],
                outputs: vec![],
            },
        ],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("prod".to_string(), Box::new(FloodProducerEntry::new()));
    factories.insert("sink".to_string(), Box::new(consumer));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build two-input graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher(CTX_TOPIC, MaxSliceLen::const_new(64), 0)
        .expect("external ctx publisher attaches");
    publish_all(&mut ext, [7]);
    for _ in 0..300 {
        runtime.step(Duration::from_millis(1));
    }

    let h = runtime.node_handle("sink").unwrap();
    assert!(
        h.expect_within_backlogged_count() > 0,
        "anti-vacuity: the trigger input's window must really have lapsed \
         under its permanent backlog (backlogged = {})",
        h.expect_within_backlogged_count()
    );
    assert!(
        ctx_events.load(Ordering::Relaxed) >= 1,
        "a NON-trigger latest-value input starved of data must still fire its \
         ExpectWithinEvent — a node-level backlog on an unrelated trigger \
         input says nothing about it (got {})",
        ctx_events.load(Ordering::Relaxed)
    );
    assert_eq!(
        trig_events.load(Ordering::Relaxed),
        0,
        "the backlogged FIFO trigger input must emit NO ExpectWithinEvent — \
         a handler that fails over on staleness must not fire while \
         unconsumed frames are queued on that very input (got {})",
        trig_events.load(Ordering::Relaxed)
    );
}

// ===========================================================================
// (6) Every suppressed window is REPORTED, never silent
// ===========================================================================

// The level of a captured line comes from the ONE shared, header-scoped
// helper (`cerulion_core::testing::line_level`): `tracing-test` renders the
// span name — the test function's own name — into every line, so a bare
// `contains("INFO")` can be satisfied by a rename, and a token match over the
// WHOLE line can be satisfied by a field VALUE.
use cerulion_core::testing::{count_at_exclusively, line_level};

/// `tracing_test` injects `logs_assert` into the annotated test function's
/// own scope, so the count has to expand THERE — a free function cannot see
/// it.
macro_rules! count_at {
    ($level:expr, $marker:expr) => {{
        let n = std::cell::Cell::new(0usize);
        logs_assert(|lines: &[&str]| {
            n.set(
                lines
                    .iter()
                    .filter(|l| l.contains($marker) && line_level(l) == Some($level))
                    .count(),
            );
            Ok(())
        });
        n.get()
    }};
}

/// [`count_at!`] plus the level-free total: the count at `$level`, refusing if
/// a line carrying the same marker sits at any OTHER level.
///
/// THE form for a positive "exactly N lines" claim. The ABSENCE claims below
/// keep the plain [`count_at!`] — an absence holds at every level, and asking
/// for exclusivity at a level the marker never uses would refuse rather than
/// read 0.
macro_rules! count_at_exclusively {
    ($level:expr, $marker:expr) => {{
        let n = std::cell::Cell::new(0usize);
        logs_assert(|lines: &[&str]| {
            n.set(count_at_exclusively(lines, $level, &[$marker])?);
            Ok(())
        });
        n.get()
    }};
}

/// Count lines carrying `$marker` at `$level` that ALSO carry `key=value` as a
/// whole whitespace token.
///
/// Whole-token, per the `has_field` rule: `contains("node_id=sink")`
/// is satisfied by `node_id=sink2`, and `contains("input=")` by any key that
/// merely ends in `input`. `tracing_test` also renders the SPAN NAME (the test
/// function's own name) into every line, so a substring probe can be satisfied
/// by the harness rather than by the event.
macro_rules! count_at_with_field {
    ($level:expr, $marker:expr, $field:expr) => {{
        let n = std::cell::Cell::new(0usize);
        logs_assert(|lines: &[&str]| {
            let carries =
                |l: &str| l.contains($marker) && l.split_whitespace().any(|t| t == $field);
            // The level-free total of the SAME predicate, paired with the
            // level count: a second copy of the line at another level would
            // otherwise pass the level count untouched.
            let total = lines.iter().filter(|l| carries(l)).count();
            let at_level = lines
                .iter()
                .filter(|l| carries(l) && line_level(l) == Some($level))
                .count();
            if total != at_level {
                return Err(format!(
                    "{total} line(s) carry {:?} with {:?}, but only {at_level} at {} — a copy \
                     at another level",
                    $marker, $field, $level
                ));
            }
            n.set(at_level);
            Ok(())
        });
        n.get()
    }};
}

/// Suppression must never be silent: a per-window `debug!` for EVERY
/// suppressed window (so the count is reproducible from the log alone) plus
/// exactly one `info!` when the regime opens and one when it closes.
///
/// The regime shape is arm 3's: two windows lapse under the backlog, then the
/// backlog drains and the regime closes. Levels are matched as whole tokens.
#[test]
#[serial]
#[traced_test]
fn every_suppressed_window_is_reported_not_silent() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "sink".to_string(),
        // Throttle > window: the regime only exists when the drain is SLOWER
        // than the window (see the fast-drain twin).
        Box::new(reading_consumer(50, Some(100), Arc::clone(&seen))),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(consumer_only_graph("ewff"), factories, clock, 16)
            .expect("build reported graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher(EXT_TOPIC, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");
    publish_all(&mut ext, [1, 2, 3, 4, 5, 6]);
    const STEPS: usize = 700;
    for _ in 0..STEPS {
        runtime.step(Duration::from_millis(1));
    }
    let backlogged = runtime
        .node_handle("sink")
        .unwrap()
        .expect_within_backlogged_count();

    // RELEASE-OBSERVABLE, and stated at its real strength. Where `debug!` is
    // compiled out the per-window line DOES NOT EXIST, so no log oracle can
    // survive there — the DEBUG count below is 0 == 0 and proves nothing. What
    // DOES survive is the counter the same arm bumps: in
    // `scheduler/mod.rs`, `expect_backlogged.fetch_add(1, ..)` sits inside the
    // `if backlogged_now` block immediately above the emit, so one bump is one
    // reported window (the counter rule: a counter is the release-safe
    // complement to a log assertion).
    //
    // Pinned EXACTLY, not `> 0`: the stimulus is deterministic — a
    // `VirtualClock`, six frames, a fixed `STEPS` — so a change in how many
    // windows lapse is a change in behaviour, not in load.
    const WINDOWS: u64 = 9;
    assert_eq!(
        backlogged, WINDOWS,
        "the suppressed-window count is the release-safe half of this contract, and it is \
         exact under a VirtualClock: {STEPS} steps over a 50ms window with a 100ms throttle \
         lapse exactly {WINDOWS} windows"
    );
    // ...and the ONE-REPORT-PER-WINDOW cadence, which is what stops the
    // diagnostic becoming a per-step flood, is release-observable too: the
    // report arm ends by re-anchoring the window (`tracker.window_start_ns =
    // new_time;` before its `continue`, `scheduler/mod.rs` — the line whose own
    // comment names this cadence), and without that re-anchor `elapsed_ns >
    // within_ns` stays true on EVERY later step, so the count runs away toward
    // `STEPS`. Asserted as a property rather than a second magic number, and it
    // holds in both profiles. (This test catches removing that re-anchor, not
    // a change to `backlog_suppressed_window`, which is the drain-out-edge
    // correction flag and not the cadence gate.)
    assert!(
        (backlogged as usize) * 10 < STEPS,
        "one report per WINDOW, not per step: {backlogged} reports across {STEPS} steps is a \
         per-step flood (the cadence gate was lost)"
    );

    let windows = count_at_exclusively!("DEBUG", "expect_within backlog window");
    // Level-free: a suppressed window must never be reported LOUDLY.
    for level in ["WARN", "INFO", "ERROR"] {
        assert_eq!(
            count_at!(level, "expect_within backlog window"),
            0,
            "a suppressed window was reported at {level} — the per-window line is debug!"
        );
    }
    let want_windows = debug_lines_expected(backlogged as usize);
    assert_eq!(
        windows, want_windows,
        "EVERY suppressed window must be reported at DEBUG — the log and the \
         counter must agree (counter {backlogged}, DEBUG lines {windows}; 0 where \
         `debug!` is compiled out)"
    );
    assert_eq!(
        count_at_exclusively!("INFO", "expect_within backlog regime opened"),
        1,
        "the regime head is loud (INFO) exactly ONCE — not once per window"
    );
    assert_eq!(
        count_at_exclusively!("INFO", "expect_within backlog regime closed"),
        1,
        "the regime close is reported exactly ONCE, when the backlog drains"
    );
}

/// THE OTHER SIDE of the regime boundary: when the drain OUTRUNS the window,
/// there is no backlog to classify and the dead producer is found on time
/// anyway.
///
/// This is the shape with a cap well UNDER the window.
/// Without the mint gate it shows suppressed windows, but only
/// because re-offer inflation holds `pending_data_count` above zero long
/// after the queue has emptied — the stimulus manufactures its own
/// precondition out of the defect. With the gate in, that shape produces
/// exactly what an operator should want and this arm pins it: NOTHING is
/// classified as backlog, NOTHING is suppressed, and the watchdog still trips
/// one window after the producer's last frame.
///
/// Hand-derived, and every quantity is exact under the `VirtualClock` (no wall
/// clock is read on this path): all six frames are published BEFORE the first
/// step, so each carries wire stamp 0 and the anchor — which stores the FRAME's
/// stamp, never observer-now, on both the boundary (`signal_input_received`)
/// and refill (`note_trigger_arrival`) paths — stays at 0. The 5 ms cap drains
/// all six by ~step 30, well before the 50 ms window can elapse, so no window
/// ever lapses with a frame in hand. The window then trips at the first step
/// whose clock exceeds 50 ms — step 51.
///
/// That step number is also what pins the drain-out re-anchor's SCOPE: an
/// unscoped edge re-anchors on every `pending 1 → 0` transition, and the last
/// one here lands at the drain (~step 30), which would push the first miss out
/// to ~step 81. The ±2 band cannot absorb that.
#[test]
#[serial]
#[traced_test]
fn a_drain_that_outruns_the_window_classifies_no_backlog_and_still_finds_the_dead_producer() {
    const WITHIN_MS: u32 = 50;
    // A cap well UNDER the window: six frames drain in ~30 ms.
    const THROTTLE_MS: u64 = 5;
    const STEPS: u32 = 200;

    let events = Arc::new(AtomicU64::new(0));
    let events_c = Arc::clone(&events);
    let seen = Arc::new(Mutex::new(Vec::new()));
    let seen_c = Arc::clone(&seen);

    let consumer = ClosureNodeEntry::new(
        consumer_info(WITHIN_MS as u64, Some(THROTTLE_MS)),
        move |ctx| {
            if let Some(s) = ctx.subscriber_mut("inp") {
                if let Ok(Some(v)) = s.try_view::<Vector3, _>(|view| view.x as u64) {
                    seen_c.lock().unwrap().push(v);
                }
            }
            if ctx.take_expect_within_event("inp").is_some() {
                events_c.fetch_add(1, Ordering::Relaxed);
            }
            Ok(())
        },
    )
    .with_label("ewf_consumer");

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("sink".to_string(), Box::new(consumer));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(consumer_only_graph("ewfm"), factories, clock, 16)
            .expect("build fast-drain graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher(EXT_TOPIC, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");
    publish_all(&mut ext, [1, 2, 3, 4, 5, 6]);
    // Dies by going SILENT — see the note in the sibling arm.
    let mut ext_live = ext;

    let handle = runtime.node_handle("sink").unwrap().clone();
    let mut first_miss_step: Option<u32> = None;
    for step in 1..=STEPS {
        runtime.step(Duration::from_millis(1));
        if first_miss_step.is_none() && handle.expect_within_missed_count() > 0 {
            first_miss_step = Some(step);
        }
    }

    assert_eq!(
        *seen.lock().unwrap(),
        vec![1, 2, 3, 4, 5, 6],
        "anti-vacuity: every frame must be served, so the drain really did \
         outrun the window"
    );
    assert_eq!(
        handle.expect_within_backlogged_count(),
        0,
        "a drain faster than the window never leaves a window lapsing with a \
         frame in hand, so NOTHING may be classified as backlog (got {})",
        handle.expect_within_backlogged_count()
    );
    assert_eq!(
        count_at!("DEBUG", "expect_within backlog window"),
        0,
        "and nothing may be REPORTED as a suppressed window either — the \
         counter and the log agree on zero"
    );

    // The event was minted at the lapse and QUEUED. A Data node whose producer
    // died never ticks again, so `take_expect_within_event` — which runs inside
    // the tick — would never drain it and the count would read 0 no matter what
    // the scheduler did. Wake the node ONCE with a single frame, purely to let
    // the already-minted event out. This cannot manufacture the result: the
    // arrival RE-ARMS the latch but nothing lapses afterwards, so a second event
    // is impossible, and a run that never minted one still reads 0.
    publish_all(&mut ext_live, [99]);
    for _ in 0..(THROTTLE_MS as u32 + 20) {
        runtime.step(Duration::from_millis(1));
    }

    let first_miss = first_miss_step.expect("a dead producer MUST trip the watchdog");
    // Every frame carries wire stamp 0, so the window trips at the first step
    // past 50 ms.
    let expected = WITHIN_MS + 1;
    assert!(
        (expected - 2..=expected + 2).contains(&first_miss),
        "the miss must land ONE window after the producer's last frame \
         (expected ≈ step {expected}), not one window after the DRAIN — got \
         step {first_miss}"
    );
    assert_eq!(
        events.load(Ordering::Relaxed),
        1,
        "and it must fire exactly one ExpectWithinEvent: no backlog ever \
         suppressed this input, so the latch was never touched (got {})",
        events.load(Ordering::Relaxed)
    );
}

// ===========================================================================
// (7) Determinism (Principle #7)
// ===========================================================================

#[test]
#[serial]
fn the_verdict_is_deterministic_across_two_runs() {
    let a = run_steady("ewfg", 100, 50, 400);
    let b = run_steady("ewfh", 100, 50, 400);
    assert_eq!(
        a, b,
        "both watchdog buckets and the fire count key off wire timestamps + \
         the scheduler clock, never wall time"
    );
}

// ===========================================================================
// (8)+(9) The HELD-HEAD diagnostic — the one case the guard cannot see
// ===========================================================================

/// Build the collapsed-tick graph: the tick returns BEFORE reading its trigger
/// input until `context_ready` flips — the shape where an earlier
/// context input with no delivery yet collapses the generated read chain. The
/// boundary then re-offers the SAME frozen head forever, minting one arrival
/// signal per boundary, so the input's backlog never empties and its
/// `expect_within_ms` watchdog is suppressed indefinitely.
fn collapsed_consumer(context_ready: Arc<AtomicBool>) -> ClosureNodeEntry {
    ClosureNodeEntry::new(consumer_info(50, None), move |ctx| {
        if !context_ready.load(Ordering::Relaxed) {
            return Ok(());
        }
        if let Some(s) = ctx.subscriber_mut("inp") {
            let _ = s.try_view::<Vector3, _>(|view| view.x);
        }
        Ok(())
    })
    .with_label("ewf_consumer")
}

/// A head no tick ever reads warns ONCE (not once per boundary) and reports
/// its recovery when a tick finally takes it.
///
/// This is the residual the backlog guard deliberately cannot close: from the
/// scheduler's seat a re-offered head is indistinguishable from a deferred
/// one, and both keep the pending count above zero. The subscriber is the only
/// place that can tell them apart, so the loud line lives there.
#[test]
#[serial]
#[traced_test]
fn a_held_head_no_tick_reads_warns_once_and_reports_recovery() {
    const MARKER: &str = "held FIFO head re-offered";
    const RECOVERY: &str = "held FIFO head served";
    let ready = Arc::new(AtomicBool::new(false));
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "sink".to_string(),
        Box::new(collapsed_consumer(Arc::clone(&ready))),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(consumer_only_graph("ewfi"), factories, clock, 16)
            .expect("build collapsed graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher(EXT_TOPIC, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");
    publish_all(&mut ext, [1, 2]);

    // One boundary pops+freezes the head; the tick collapses, so every LATER
    // boundary re-offers it. 1200 steps clears the 1024-boundary threshold
    // with room to prove the line does not repeat.
    //
    // CADENCE PIN: the streak counts BOUNDARY
    // re-offers ONLY, never the between-fires REFILL that also finds the head
    // frozen. Both drains run on every one of these steps (a `ClosureNodeEntry`
    // reports `refills_trigger_input`, so the Unified binding installs the
    // refill hook), so a streak fed by both advances TWICE per step and the
    // warn lands near step 512 instead of near 1025 — which the terminal
    // `count_at == 1` cannot see, since either cadence has fired by 1200.
    // Splitting the loop is the discriminator, and it is load-INSENSITIVE:
    // the boundary runs exactly once per `step()` under a `VirtualClock`, so
    // this counts steps, never wall time.
    for _ in 0..900 {
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(
        count_at!("WARN", MARKER),
        0,
        "the threshold is 1024 BOUNDARIES — one per step — so 900 steps must \
         not have reached it; a streak that also counted the refill would have \
         warned around step 512"
    );
    for _ in 0..300 {
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(
        count_at_exclusively!("WARN", MARKER),
        1,
        "a held head warns ONCE per regime, never once per boundary"
    );
    assert_eq!(
        count_at!("INFO", RECOVERY),
        0,
        "nothing recovered yet — the tick still never reads the input"
    );

    // The read chain recovers: the next boundary is free to pop.
    ready.store(true, Ordering::Relaxed);
    for _ in 0..10 {
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(
        count_at_exclusively!("INFO", RECOVERY),
        1,
        "the regime closes with exactly one recovery line once a tick takes \
         the head"
    );
    assert_eq!(
        count_at_exclusively!("WARN", MARKER),
        1,
        "recovery must not re-emit the warn"
    );

    // Both held-head lines must name the node and the input.
    // The remedy text tells an operator to go and look at a node's read chain,
    // so a line carrying only the topic is not actionable on any fan-out topic
    // — and it cannot be grepped by the `node_id=`/`input=` keys every other
    // line in this feature uses, including the scheduler half. The identity is
    // on the struct and always populated on the only path that reaches here.
    assert_eq!(
        count_at_with_field!("WARN", MARKER, "node_id=sink"),
        1,
        "the held-head warn must name the NODE"
    );
    assert_eq!(
        count_at_with_field!("WARN", MARKER, "input=inp"),
        1,
        "the held-head warn must name the INPUT field"
    );
    assert_eq!(
        count_at_with_field!("INFO", RECOVERY, "node_id=sink"),
        1,
        "the recovery line must name the node too — a regime that opens under \
         one key and closes under another cannot be followed with one grep"
    );
    assert_eq!(
        count_at_with_field!("INFO", RECOVERY, "input=inp"),
        1,
        "and the input"
    );
}

/// The control that makes the threshold meaningful: an ordinary bounded defer
/// re-offers the head too, and must NOT warn. A 100 ms throttle against 1 ms
/// steps re-offers ~99 times per fire — two orders of magnitude of headroom
/// below the 1024-boundary threshold.
#[test]
#[serial]
#[traced_test]
fn a_bounded_throttle_defer_never_warns_about_a_held_head() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "sink".to_string(),
        Box::new(reading_consumer(50, Some(100), Arc::clone(&seen))),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(consumer_only_graph("ewfj"), factories, clock, 16)
            .expect("build throttled graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher(EXT_TOPIC, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");
    publish_all(&mut ext, [1, 2, 3, 4, 5, 6]);
    // Six frames at one fire per 100 ms need 1 + 5 x 100 = 501 steps to all be
    // served; 700 leaves headroom while the deepest single-head streak stays
    // ~99, two orders of magnitude below the 1024-boundary threshold.
    for _ in 0..700 {
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(
        count_at!("WARN", "held FIFO head re-offered"),
        0,
        "a bounded throttle defer is normal operation and must stay silent"
    );
    // Anti-vacuity: the run really did re-offer heads (it served every frame
    // through a 100 ms cap), so the silence above is a decision, not an
    // absence of the shape.
    assert_eq!(
        *seen.lock().unwrap(),
        vec![1, 2, 3, 4, 5, 6],
        "every frame still reaches its own throttled fire, in order"
    );
}
