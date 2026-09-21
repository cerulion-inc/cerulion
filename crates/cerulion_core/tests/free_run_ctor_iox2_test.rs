// SPDX-License-Identifier: AGPL-3.0-only
//! The two FREE-RUN build ctors and the gating-epoch
//! primitive, over real iceoryx2 (per-test SHM roots — parallel-safe).
//!
//! # What is pinned, and why each arm is a hand oracle
//!
//! * **The stamped `topic_requirements` union reaches the created service on
//!   BOTH free-run ctors** ([`GraphRuntime::build_live_free_run`] and
//!   [`GraphRuntime::build_live_deterministic_free_run`]). The oracle is the
//!   existing stamp-positive pin's: a borrow requirement STRICTLY ABOVE the
//!   create-side floor (5 vs 3) read off the created service's static config,
//!   so the value is attributable ONLY to the union having been threaded. Each
//!   arm carries its NEGATIVE control in the same body — the existing
//!   single-process ctor, which hardcodes the union to `None` and therefore
//!   creates at the floor — so the pin cannot pass on a ctor that silently
//!   drops the union (a stamped union of `None`).
//! * **The deterministic free-run ctor is the single-process recording ctor
//!   per rank**: the LOCAL quantum (`tightest_timing_ns`, here `period_ms`),
//!   NO barrier participant (the free-run exit contract — `is_barrier_failed`
//!   false, `leave_barrier_cohort` a no-op), and the wall-following clock armable.
//! * **`place_gating_epoch` re-phases what the clock's build-time value
//!   anchored** — the `Period` deadline (the lone fire lands one interval
//!   after the epoch, never a catch-up burst at step 1) and the
//!   `promise_within` window (ZERO phantom misses after a ~1e18 ns jump).
//!   Both are what a bare `clock.set(real_ns())` gets wrong.
//! * **The primitive refuses** a RealClock build, a lockstep barrier
//!   participant, and a placement after the first step — loudly, never a
//!   silent no-op.
//! * **The RealClock free-run build warns INERT** on `set_gating_follows_wall`
//!   (the refusal on the RealClock arm), and the deterministic
//!   sibling does not.
//! * **Refusal and re-phase arms:** the wall-following clock is refused on a lockstep
//!   participant (turning it OFF is accepted; the free-run build accepts ON);
//!   a placed epoch yields a HAND-ORACLE fixed-quantum trace
//!   (`[E+4 ms .. E+20 ms]`, two runs identical); a placed epoch re-seeds the
//!   INPUT watchdog window (the quantum IS the window, so the boundary step
//!   counts no miss); an epoch ARMED for the live anchor is placed when the
//!   anchor is taken and is one-shot.
//! * **Armed-epoch arms:** the real `run_live` spends the armed epoch at
//!   its anchor (zero steps under a pre-flipped `running`); an arm AFTER a
//!   step is refused with the clock untouched and nothing pending; the
//!   arm-time refusal names the ARM entry point, never a placement of `0`;
//!   an armed runtime stepped through the POLLED seam warns once, and one
//!   dropped with the arm pending warns at `Drop`.
//!
//! Every manager is an `init_for_test` per-test SHM root; nothing here touches
//! the process-global singleton, so the file is parallel-safe and needs no
//! nextest fence entry (`serial_discipline_test`).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::barrier::MappedBarrier;
use cerulion_core::clock::{real_ns, Clock, RealClock, VirtualClock};
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::CrossProcessWiring;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::{TopicRequirements, TransportConfig, TransportManager};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// The ticker's period — the deterministic free-run ctor's LOCAL quantum for a
/// graph whose only declared timing this is.
const PERIOD_MS: u64 = 4;
const PERIOD_NS: u64 = PERIOD_MS * 1_000_000;
/// Per live-step WaitSet timeout: the ticker graph has no data sources, so each
/// step sleeps this then steps; the wall elapsed of the sleep IS the measured
/// gating advance once `set_gating_follows_wall(true)` is armed.
const STEP_TIMEOUT: Duration = Duration::from_millis(2);
/// Upper bound on the live steps the epoch arm drives while waiting for two
/// fires — 200 × ≥2 ms ≫ the 8 ms two fires need, so a loaded runner never
/// converts a slow step into a failure.
const MAX_LIVE_STEPS: usize = 200;
/// A hand-stamped requirement STRICTLY ABOVE the create-side borrow
/// floor (3 = `SUBSCRIBER_MAX_BORROWED_HELD`, pub(crate) — literal here), so
/// the created value is attributable ONLY to the union machinery.
const STAMPED_BORROW: usize = 5;
const BORROW_FLOOR: usize = 3;

// ===========================================================================
// Node types — replicated inline (test binaries are separate crates).
// ===========================================================================

/// A pure-`Period` producer with a `promise_within` output: the schedule the
/// epoch re-phases, and the watchdog window it re-seeds, in one node. The
/// 50 ms promise is wide enough that a healthy 4 ms ticker never misses it,
/// and tiny against the ~1e18 ns epoch jump a phantom miss would measure.
#[cerulion_node(period_ms = 4)]
#[derive(Default)]
struct Ticker {
    #[output(promise_within_ms = 50)]
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

/// A host-driven producer that is never fired — the union arms only look at
/// how its owned topic was PROVISIONED at build.
/// A data-trigger consumer whose one trigger
/// input carries an `expect_within_ms` watchdog — the INPUT half of the
/// watchdog re-seed an epoch placement performs (the ticker above covers only
/// the output `promise_within` half).
#[cerulion_node]
#[derive(Default)]
struct Watcher {
    #[input(trigger, expect_within_ms = 50)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl Watcher {
    fn tick(&mut self) -> Result<(), NodeError> {
        Ok(())
    }
}

#[cerulion_node(external)]
#[derive(Default)]
struct Producer {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl Producer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

// ===========================================================================
// Fixtures
// ===========================================================================

fn vec3_out(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "geometry_msgs/Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    }
}

fn one_node_config(prefix: &str, id: &str, node_type: &str) -> GraphConfig {
    GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("{prefix}_{id}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: id.to_string(),
            node_type: node_type.to_string(),
            inputs: vec![],
            outputs: vec![vec3_out("out")],
        }],
    }
}

/// A one-`Producer` graph (never fired) — the provisioning fixture.
fn producer_graph(prefix: &str) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = one_node_config(prefix, "producer", "producer");
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(ProducerEntry::with_state(Producer::default())),
    );
    (config, factories)
}

/// A one-`Ticker` graph — the schedule/watchdog fixture.
fn ticker_graph(
    prefix: &str,
    fires: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = one_node_config(prefix, "ticker", "ticker");
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "ticker".to_string(),
        Box::new(TickerEntry::with_state(Ticker {
            fires,
            ..Default::default()
        })),
    );
    (config, factories)
}

/// An isolated per-test transport on the caller's clock (the deterministic
/// ctors enforce `Arc::ptr_eq(transport.clock, build clock)`).
/// The watcher graph: one data-trigger node reading an ABSOLUTE external
/// source nobody publishes (so the watchdog's only arrivals are none, and its
/// window elapsed is measured against the epoch alone).
fn watcher_graph(prefix: &str) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let mut config = one_node_config(prefix, "watcher", "watcher");
    config.nodes[0].outputs = vec![];
    config.nodes[0].inputs = vec![cerulion_core::graph::config::InputDef {
        name: "inp".to_string(),
        source: format!("/{prefix}/ext"),
    }];
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "watcher".to_string(),
        Box::new(WatcherEntry::with_state(Watcher::default())),
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

/// The supervisor-shaped stamp: one topic, borrow strictly above the floor.
fn stamped_union(topic: &str) -> std::collections::BTreeMap<String, TopicRequirements> {
    let mut m = std::collections::BTreeMap::new();
    m.insert(
        topic.to_string(),
        TopicRequirements {
            min_borrowed_samples: STAMPED_BORROW,
            min_buffer: 16,
            min_subscribers: 5,
            min_event_listeners: 0,
        },
    );
    m
}

// ===========================================================================
// The union reaches the created service on BOTH ctors.
// ===========================================================================

/// The NON-RECORD free-run ctor (RealClock) provisions the owned topic at the
/// STAMPED borrow; the existing single-process live ctor — the same wiring
/// with the union hardcoded `None` — creates at the floor. The two
/// arms share one body so the pin cannot pass on a ctor that drops the union.
#[test]
fn the_free_run_live_ctor_threads_the_stamped_union_into_the_created_service() {
    // POSITIVE: the free-run ctor, union stamped.
    let real: Arc<dyn Clock> = Arc::new(RealClock);
    let mgr = manager("frc_live_pos", Arc::clone(&real));
    let (config, factories) = producer_graph("frclp");
    let topic = "/frclp/producer/out";
    let union = stamped_union(topic);
    let rt = GraphRuntime::build_live_free_run(
        config,
        factories,
        &mgr,
        real,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::requirements_only(Some(&union)),
    )
    .expect("build the free-run live producer");
    assert_eq!(
        mgr.default_subscriber_max_borrowed_samples_for_test(topic),
        STAMPED_BORROW,
        "build_live_free_run must create the owned service at the STAMPED borrow — the \
         create-side floor alone yields {BORROW_FLOOR}, so {STAMPED_BORROW} is attributable \
         only to the union having reached `build_with_scheduler`"
    );
    rt.shutdown();

    // NEGATIVE CONTROL: the single-process live ctor (union `None`) on a fresh
    // root creates at the floor — what a free-run ctor that dropped the union
    // would also do, which is why the positive arm is not vacuous.
    let real2: Arc<dyn Clock> = Arc::new(RealClock);
    let mgr2 = manager("frc_live_neg", Arc::clone(&real2));
    let (config2, factories2) = producer_graph("frcln");
    let topic2 = "/frcln/producer/out";
    let rt2 = GraphRuntime::build_live_with_schema_hashes_and_policy(
        config2,
        factories2,
        &mgr2,
        real2,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
    )
    .expect("build the single-process live producer");
    assert_eq!(
        mgr2.default_subscriber_max_borrowed_samples_for_test(topic2),
        BORROW_FLOOR,
        "the single-process live ctor carries no union and creates at the create-side floor"
    );
    rt2.shutdown();
}

/// The `--record` free-run ctor (controlled clock) provisions at the STAMPED
/// borrow; the existing single-process deterministic ctor creates at the floor.
#[test]
fn the_free_run_deterministic_ctor_threads_the_stamped_union_into_the_created_service() {
    // POSITIVE.
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("frc_det_pos", clock.clone());
    let (config, factories) = producer_graph("frcdp");
    let topic = "/frcdp/producer/out";
    let union = stamped_union(topic);
    let rt = GraphRuntime::build_live_deterministic_free_run(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        None,
        CrossProcessWiring::requirements_only(Some(&union)),
    )
    .expect("build the free-run deterministic producer");
    assert_eq!(
        mgr.default_subscriber_max_borrowed_samples_for_test(topic),
        STAMPED_BORROW,
        "build_live_deterministic_free_run must create the owned service at the STAMPED borrow"
    );
    rt.shutdown();

    // NEGATIVE CONTROL: the single-process deterministic ctor (union `None`).
    let clock2 = Arc::new(VirtualClock::new());
    let mgr2 = manager("frc_det_neg", clock2.clone());
    let (config2, factories2) = producer_graph("frcdn");
    let topic2 = "/frcdn/producer/out";
    let rt2 = GraphRuntime::build_live_deterministic_with_schema_hashes_and_policy(
        config2,
        factories2,
        &mgr2,
        Arc::clone(&clock2),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        None,
    )
    .expect("build the single-process deterministic producer");
    assert_eq!(
        mgr2.default_subscriber_max_borrowed_samples_for_test(topic2),
        BORROW_FLOOR,
        "the single-process deterministic ctor carries no union and creates at the create-side floor"
    );
    rt2.shutdown();
}

// ===========================================================================
// The deterministic free-run ctor IS the single-process recording ctor per rank.
// ===========================================================================

/// LOCAL quantum derived from the graph (`period_ms` → 4 ms), NO barrier
/// participant (the free-run exit contract is structurally inert), and the
/// wall-following clock armable — the monolith recording arm, per rank.
#[test]
fn the_free_run_deterministic_ctor_derives_the_local_quantum_and_installs_no_participant() {
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("frc_quantum", clock.clone());
    let (config, factories) = ticker_graph("frcq", Arc::new(AtomicU64::new(0)));
    let mut rt = GraphRuntime::build_live_deterministic_free_run(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        None,
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the free-run deterministic ticker");
    assert_eq!(
        rt.live_gating_quantum_for_test(),
        Some(Duration::from_nanos(PERIOD_NS)),
        "the quantum is the LOCAL graph's tightest timing, floored at 1 ms — never a handed one"
    );
    assert!(
        !rt.gating_follows_wall_for_test(),
        "a fresh build takes the fixed-quantum branch until the caller arms the wall-following clock"
    );
    // The free-run exit contract: no participant, so the two barrier seams the
    // worker still calls are inert by construction.
    assert!(!rt.is_barrier_failed());
    assert!(
        !rt.leave_barrier_cohort(),
        "a free-run rank has no cohort to leave — the drop must be a `false` no-op"
    );
    rt.set_gating_follows_wall(true)
        .expect("no lockstep participant on this build, so the wall-following clock is accepted");
    assert!(
        rt.gating_follows_wall_for_test(),
        "the deterministic free-run build arms the wall-following clock exactly like the monolith \
         recording arm"
    );
    rt.shutdown();
}

// ===========================================================================
// The epoch primitive.
// ===========================================================================

/// After `place_gating_epoch(real_ns())` on a fresh build: the lone ticker's
/// FIRST fire lands ≥ one interval after the epoch (the `Period` deadline was
/// re-baselined, so there is no catch-up burst — a bare `clock.set` would mint
/// `epoch / 4 ms` fires on step 1, which is a hang), every later fire is
/// monotone and in the epoch domain, and the ticker's `promise_within`
/// window counts ZERO misses (re-seeded — a bare `set` measures an elapsed of
/// `epoch − 0` on the first QoS pass and phantom-misses).
#[test]
fn a_placed_epoch_re_phases_the_period_deadline_and_re_seeds_the_watchdog_window() {
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("frc_epoch", clock.clone());
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = ticker_graph("frce", Arc::clone(&fires));
    let mut rt = GraphRuntime::build_live_deterministic_free_run(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        None,
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the free-run deterministic ticker");
    rt.set_gating_follows_wall(true)
        .expect("no lockstep participant on this build, so the wall-following clock is accepted");

    // A fresh controlled clock reads 0; the epoch is the machine's boot-monotonic
    // now — hand oracle: nonzero, and ≥ the wall read just before it.
    assert_eq!(
        clock.now_ns(),
        0,
        "a fresh VirtualClock starts at the origin"
    );
    let before = real_ns();
    let epoch = real_ns();
    assert!(epoch > 0 && epoch >= before);
    rt.place_gating_epoch(epoch)
        .expect("a fresh deterministic free-run build accepts an epoch");
    assert_eq!(
        clock.now_ns(),
        epoch,
        "the placement moves the controlled clock to the epoch"
    );

    // Drive until two fires (bounded), then read the trace.
    let mut steps = 0usize;
    while fires.load(Ordering::Relaxed) < 2 && steps < MAX_LIVE_STEPS {
        rt.run_live_step_once_for_test(STEP_TIMEOUT);
        steps += 1;
    }
    let fire_times: Vec<u64> = rt
        .trace()
        .iter()
        .filter(|e| &*e.node_id == "ticker")
        .map(|e| e.fire_time_ns)
        .collect();
    assert!(
        fire_times.len() >= 2,
        "the ticker must fire at least twice within {MAX_LIVE_STEPS} wall-following steps (got \
         {}); the epoch placement must not stall the schedule",
        fire_times.len()
    );
    // ORACLE 1 — re-phased, not burst: the first fire is at least one whole
    // interval after the epoch (a build at the epoch baselines `epoch + 4 ms`).
    assert!(
        fire_times[0] >= epoch + PERIOD_NS,
        "the first fire must land ≥ one interval after the epoch (re-baselined deadline); \
         got {} vs epoch {epoch} + {PERIOD_NS}",
        fire_times[0]
    );
    // ORACLE 2 — no catch-up burst: the fire count is bounded by the MEASURED
    // wall elapsed since the epoch (one fire per interval, plus the boundary
    // one), never by the epoch's magnitude. A slow first step legitimately
    // catches up on the intervals its wall really covered (MEASURED:
    // a ~12 ms first live step mints 3 fires — the k-fires model,
    // correct; the seam anchors `last` AFTER its reactor build, so that wall
    // is the step's own, not the build's);
    // what a bare `clock.set` would mint is `epoch / 4 ms` ≈ 1e11 fires, which
    // this bound refuses by ten orders of magnitude (and which hangs the step).
    let wall_elapsed = clock.now_ns().saturating_sub(epoch);
    let fires_bound = wall_elapsed / PERIOD_NS + 1;
    assert!(
        fire_times.len() as u64 <= fires_bound,
        "fires ({}) must be bounded by the wall elapsed since the epoch ({wall_elapsed} ns \
         ⇒ at most {fires_bound} over {steps} steps) — a burst sized by the epoch itself is \
         exactly what the re-phase forbids",
        fire_times.len()
    );
    // ORACLE 3 — every stamp is in the epoch domain and monotone.
    for pair in fire_times.windows(2) {
        assert!(
            pair[1] > pair[0],
            "fire times must be strictly monotone: {pair:?}"
        );
    }
    // ORACLE 4 — the watchdog window was re-seeded: a healthy 4 ms ticker
    // inside a 50 ms promise counts NO miss. Without the re-seed the first QoS
    // pass measures `epoch − 0` against 50 ms and counts one.
    let handle = rt.node_handle("ticker").expect("the ticker's handle");
    assert_eq!(
        handle.promise_within_missed_count(),
        0,
        "a placed epoch must re-seed the promise_within window — a phantom miss here is the \
         `epoch − 0` elapsed a bare clock.set leaves behind"
    );
    rt.shutdown();
}

/// The three refusals, each a loud `Err` naming its reason: a RealClock build
/// has nothing to place; a lockstep participant's clock is not this rank's to
/// move; an epoch is an origin and cannot follow a step.
#[test]
fn the_epoch_is_refused_on_a_real_clock_build_a_lockstep_participant_and_after_a_step() {
    // (a) RealClock free-run build.
    let real: Arc<dyn Clock> = Arc::new(RealClock);
    let mgr_a = manager("frc_refuse_a", Arc::clone(&real));
    let (config_a, factories_a) = producer_graph("frcra");
    let mut rt_a = GraphRuntime::build_live_free_run(
        config_a,
        factories_a,
        &mgr_a,
        real,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the RealClock free-run producer");
    let err = rt_a
        .place_gating_epoch(real_ns())
        .expect_err("a RealClock build has no controlled clock to place");
    assert!(
        err.to_string().contains("NO controlled gating clock"),
        "the refusal must name the missing controlled clock, got: {err}"
    );
    rt_a.shutdown();

    // (b) A LOCKSTEP barrier participant (the (e) ctor, expected = 1).
    let clock_b = Arc::new(VirtualClock::new());
    let mgr_b = manager("frc_refuse_b", clock_b.clone());
    let (config_b, factories_b) = producer_graph("frcrb");
    let ns = format!("frc_refuse_{}", std::process::id());
    let barrier = Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("barrier owner"));
    let mut rt_b = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        config_b,
        factories_b,
        &mgr_b,
        Arc::clone(&clock_b),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        barrier,
        vec![Some(0)],
        vec![false],
        0,
        Duration::from_millis(1),
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the lockstep participant");
    let err = rt_b
        .place_gating_epoch(real_ns())
        .expect_err("a lockstep participant's clock is the shared handed-quantum timeline");
    assert!(
        err.to_string().contains("LOCKSTEP"),
        "the refusal must name the lockstep participant, got: {err}"
    );
    assert_eq!(
        clock_b.now_ns(),
        0,
        "a refused placement moves the participant's clock by NOTHING"
    );
    rt_b.shutdown();

    // (c) After the first step.
    let clock_c = Arc::new(VirtualClock::new());
    let mgr_c = manager("frc_refuse_c", clock_c.clone());
    let (config_c, factories_c) = ticker_graph("frcrc", Arc::new(AtomicU64::new(0)));
    let mut rt_c = GraphRuntime::build_live_deterministic_free_run(
        config_c,
        factories_c,
        &mgr_c,
        Arc::clone(&clock_c),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        None,
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the free-run deterministic ticker");
    rt_c.run_live_step_once_for_test(STEP_TIMEOUT);
    let before_refusal = clock_c.now_ns();
    let err = rt_c
        .place_gating_epoch(real_ns())
        .expect_err("an epoch cannot be placed under a stream that has begun");
    assert!(
        err.to_string().contains("already begun"),
        "the refusal must name the begun step(s), got: {err}"
    );
    assert_eq!(
        clock_c.now_ns(),
        before_refusal,
        "a post-step refusal moves the clock by NOTHING (a placement that moved it before \
         refusing would be the exact skew the primitive exists to prevent)"
    );
    rt_c.shutdown();
}

/// The RealClock free-run build has no controlled clock, so asking it for the
/// wall-following clock is INERT and says so (the existing warn); the
/// deterministic sibling arms it silently. The two are asserted in one
/// captured log so the negative half is not vacuous.
#[test]
#[tracing_test::traced_test]
fn set_gating_follows_wall_on_the_real_clock_free_run_build_warns_inert() {
    let real: Arc<dyn Clock> = Arc::new(RealClock);
    let mgr = manager("frc_inert", Arc::clone(&real));
    let (config, factories) = producer_graph("frci");
    let mut rt = GraphRuntime::build_live_free_run(
        config,
        factories,
        &mgr,
        real,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the RealClock free-run producer");
    rt.set_gating_follows_wall(true)
        .expect("no lockstep participant on this build, so the wall-following clock is accepted");
    assert!(
        logs_contain("set_gating_follows_wall(true) on a build with NO gating quantum"),
        "the RealClock free-run arm must warn that the wall-following clock is INERT on it"
    );
    rt.shutdown();

    // Anti-tautology: the deterministic sibling arms it with NO warn.
    let clock = Arc::new(VirtualClock::new());
    let mgr2 = manager("frc_inert_det", clock.clone());
    let (config2, factories2) = producer_graph("frcid");
    let mut rt2 = GraphRuntime::build_live_deterministic_free_run(
        config2,
        factories2,
        &mgr2,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        None,
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the deterministic free-run producer");
    rt2.set_gating_follows_wall(true)
        .expect("no lockstep participant on this build, so the wall-following clock is accepted");
    logs_assert(|lines| {
        let n = lines
            .iter()
            .filter(|l| l.contains("on a build with NO gating quantum"))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!(
                "exactly ONE inert warn expected (the RealClock build's); got {n}"
            ))
        }
    });
    rt2.shutdown();
}

/// **The wall-following clock is refused
/// on a lockstep participant** — the same desync `place_gating_epoch` refuses,
/// which `configure_recording_runtime_mp`'s contract forbade by discipline
/// alone. Turning the flag OFF on a participant is never a desync and is
/// accepted; the free-run deterministic build (no participant) accepts ON —
/// the control that makes the refusal mean something.
#[test]
fn the_honest_clock_is_refused_on_a_lockstep_participant() {
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("frc_wall_refuse", clock.clone());
    let (config, factories) = producer_graph("frcwr");
    let ns = format!("frc_wall_refuse_{}", std::process::id());
    let barrier = Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("barrier owner"));
    let mut participant = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
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
        Duration::from_millis(1),
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the lockstep participant");
    let err = participant
        .set_gating_follows_wall(true)
        .expect_err("a lockstep participant must not follow the wall");
    assert!(
        err.to_string().contains("LOCKSTEP"),
        "the refusal names the lockstep participant: {err}"
    );
    assert!(
        !participant.gating_follows_wall_for_test(),
        "a refused set leaves the flag OFF"
    );
    participant
        .set_gating_follows_wall(false)
        .expect("turning the wall-following clock OFF is never a desync");
    participant.shutdown();

    let clock2 = Arc::new(VirtualClock::new());
    let mgr2 = manager("frc_wall_accept", clock2.clone());
    let (config2, factories2) = producer_graph("frcwa");
    let mut free = GraphRuntime::build_live_deterministic_free_run(
        config2,
        factories2,
        &mgr2,
        Arc::clone(&clock2),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        None,
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the free-run deterministic producer");
    free.set_gating_follows_wall(true)
        .expect("CONTROL: the free-run deterministic build accepts the wall-following clock");
    assert!(free.gating_follows_wall_for_test());
    free.shutdown();
}

/// **The re-phased schedule is a hand
/// oracle, and deterministic.** With the wall-following clock OFF the live loop
/// advances the controlled clock by the fixed local quantum (the ticker's own
/// 4 ms period), so after an epoch `E` the fires land at EXACTLY
/// `E + 4 ms, E + 8 ms, …` — `epoch + 2·interval` (which the epoch arm's
/// lower-bound oracle admits) is a different vector. Two fresh builds
/// produce the identical trace, and the promise window counts no phantom miss.
#[test]
fn a_placed_epoch_yields_a_deterministic_hand_oracle_trace_on_the_fixed_quantum() {
    const EPOCH: u64 = 1_000_000_000; // 1 s: far from the origin, exact in u64.
    const STEPS: u64 = 5;
    let want: Vec<u64> = (1..=STEPS).map(|k| EPOCH + k * PERIOD_NS).collect();
    let mut traces = Vec::new();
    for run in 0..2 {
        let clock = Arc::new(VirtualClock::new());
        let mgr = manager(&format!("frc_det_{run}"), clock.clone());
        let fires = Arc::new(AtomicU64::new(0));
        let (config, factories) = ticker_graph(&format!("frcd{run}"), Arc::clone(&fires));
        let mut rt = GraphRuntime::build_live_deterministic_free_run(
            config,
            factories,
            &mgr,
            Arc::clone(&clock),
            None,
            cerulion_core::MonitorWaitPolicy::off(),
            None,
            CrossProcessWiring::requirements_only(None),
        )
        .expect("build the free-run deterministic ticker");
        rt.place_gating_epoch(EPOCH)
            .expect("a fresh deterministic free-run build accepts an epoch");
        for _ in 0..STEPS {
            rt.run_live_step_once_for_test(STEP_TIMEOUT);
        }
        let fire_times: Vec<u64> = rt
            .trace()
            .iter()
            .filter(|e| &*e.node_id == "ticker")
            .map(|e| e.fire_time_ns)
            .collect();
        assert_eq!(
            fire_times, want,
            "run {run}: the fixed-quantum schedule after the epoch is the hand oracle"
        );
        assert_eq!(
            rt.node_handle("ticker")
                .expect("the ticker's handle")
                .promise_within_missed_count(),
            0,
            "run {run}: no phantom promise miss"
        );
        traces.push(fire_times);
        rt.shutdown();
    }
    assert_eq!(traces[0], traces[1], "two runs are bit-identical");
}

/// **The input half of the watchdog re-seed.**
/// `place_gating_epoch` re-seeds every `expect_within` AND `promise_within`
/// window to the epoch; the epoch arm above pins only the output half. A
/// data-trigger node with a 50 ms `expect_within` trigger input on a source
/// nobody publishes, given an epoch of 1000 s, steps ONCE. The graph has no
/// Period node, so the local quantum derives from the watchdog window itself
/// (50 ms — the only timing source): with the window re-seeded the elapsed is
/// EXACTLY the window, which the strict `>` check does not count; without it
/// the window starts at 0 and the first check reads an elapsed of 1000 s — a
/// phantom miss on every guarded input.
#[test]
fn a_placed_epoch_re_seeds_the_input_watchdog_window() {
    const EPOCH: u64 = 1_000_000_000_000; // 1000 s
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("frc_watch", clock.clone());
    let (config, factories) = watcher_graph("frcw");
    let mut rt = GraphRuntime::build_live_deterministic_free_run(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        None,
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the free-run deterministic watcher");
    rt.place_gating_epoch(EPOCH)
        .expect("a fresh deterministic free-run build accepts an epoch");
    rt.run_live_step_once_for_test(STEP_TIMEOUT);
    const WINDOW_NS: u64 = 50 * 1_000_000;
    assert_eq!(
        clock.now_ns(),
        EPOCH + WINDOW_NS,
        "one fixed-quantum step past the epoch — the quantum IS the watchdog window, the \
         graph's only timing source"
    );
    let handle = rt.node_handle("watcher").expect("the watcher's handle");
    assert_eq!(
        handle.expect_within_missed_count(),
        0,
        "the input watchdog window was re-seeded to the epoch — a miss here is the \
         `epoch − 0` elapsed a bare clock.set leaves behind"
    );
    rt.shutdown();
}

/// **The epoch can be armed
/// for the live loop's wall-clock anchor and is placed only when that
/// anchor is taken** — so the controlled clock and boot-monotonic time meet
/// to within one clock read instead of one `run_live` setup time. Arming
/// makes every refusal the immediate placement makes (all four; this arm
/// exercises the two build-shape ones — a RealClock build, a lockstep
/// participant — and the post-step one has its own arm below) at arm time,
/// and the refusal names the ARM entry point rather than a placement of an
/// epoch nobody passed; spending it reads `real_ns()` then and there; nothing
/// is pending afterwards.
#[test]
fn an_epoch_armed_for_the_live_anchor_is_placed_when_the_anchor_is_taken() {
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("frc_anchor", clock.clone());
    let (config, factories) = ticker_graph("frca", Arc::new(AtomicU64::new(0)));
    let mut rt = GraphRuntime::build_live_deterministic_free_run(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        None,
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the free-run deterministic ticker");
    assert_eq!(
        rt.place_pending_live_epoch_for_test()
            .expect("nothing pending is not an error"),
        None,
        "nothing is pending on a fresh build"
    );
    rt.place_gating_epoch_at_live_anchor()
        .expect("a fresh deterministic free-run build accepts an armed epoch");
    assert_eq!(
        clock.now_ns(),
        0,
        "arming places nothing — the clock stays at the origin"
    );
    let before = real_ns();
    let placed = rt
        .place_pending_live_epoch_for_test()
        .expect("the pending placement is spent at the anchor")
        .expect("an epoch was pending");
    let after = real_ns();
    assert!(
        before <= placed && placed <= after,
        "the epoch is read from real_ns() AT the anchor: {before} <= {placed} <= {after}"
    );
    assert_eq!(
        clock.now_ns(),
        placed,
        "the controlled clock now sits at the epoch"
    );
    assert_eq!(
        rt.place_pending_live_epoch_for_test().expect("spent"),
        None,
        "the placement is one-shot"
    );
    rt.shutdown();

    // Arm-time refusals: the same two the immediate placement makes.
    let real: Arc<dyn Clock> = Arc::new(RealClock);
    let mgr_r = manager("frc_anchor_real", Arc::clone(&real));
    let (config_r, factories_r) = producer_graph("frcar");
    let mut rt_r = GraphRuntime::build_live_free_run(
        config_r,
        factories_r,
        &mgr_r,
        real,
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the RealClock free-run producer");
    let err = rt_r
        .place_gating_epoch_at_live_anchor()
        .expect_err("a RealClock build has no controlled clock to arm");
    assert!(
        err.to_string().contains("NO controlled gating clock"),
        "{err}"
    );
    // An arm-time refusal names the arm entry point — it must never
    // render `place_gating_epoch(0)`, a placement of an epoch nobody passed.
    assert!(
        err.to_string()
            .contains("place_gating_epoch_at_live_anchor()")
            && !err.to_string().contains("place_gating_epoch(0)"),
        "the arm-time refusal names the arm, not a placement of 0: {err}"
    );
    rt_r.shutdown();

    let clock_p = Arc::new(VirtualClock::new());
    let mgr_p = manager("frc_anchor_part", clock_p.clone());
    let (config_p, factories_p) = producer_graph("frcap");
    let ns = format!("frc_anchor_{}", std::process::id());
    let barrier = Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("barrier owner"));
    let mut rt_p = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        config_p,
        factories_p,
        &mgr_p,
        Arc::clone(&clock_p),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        barrier,
        vec![Some(0)],
        vec![false],
        0,
        Duration::from_millis(1),
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the lockstep participant");
    let err = rt_p
        .place_gating_epoch_at_live_anchor()
        .expect_err("a lockstep participant cannot arm an epoch");
    assert!(err.to_string().contains("LOCKSTEP"), "{err}");
    assert!(
        err.to_string()
            .contains("place_gating_epoch_at_live_anchor()")
            && !err.to_string().contains("place_gating_epoch(0)"),
        "the arm-time refusal names the arm, not a placement of 0: {err}"
    );
    assert_eq!(clock_p.now_ns(), 0, "a refused arm moves nothing");
    rt_p.shutdown();
}

/// **The real `run_live` spends the armed
/// epoch at its anchor.** The seam arm above proves only the seam — deleting
/// `run_live`'s own `place_pending_live_epoch()` call leaves every other core test
/// passing. `run_live` with an already-flipped `running` runs its whole prologue
/// (external-source collection, the reactor build, the pending placement)
/// and then executes ZERO steps, so the clock afterwards sits at an epoch read
/// between two `real_ns()` reads taken around the call, nothing is pending,
/// and the next step fires the ticker one interval after that epoch — the
/// hand oracle a build at the epoch would give.
#[test]
fn run_live_spends_the_armed_epoch_at_its_anchor_before_the_first_step() {
    const PERIOD_NS: u64 = 4_000_000;
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("frc_runlive_anchor", clock.clone());
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = ticker_graph("frcrla", Arc::clone(&fires));
    let mut rt = GraphRuntime::build_live_deterministic_free_run(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        None,
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the free-run deterministic ticker");
    rt.place_gating_epoch_at_live_anchor()
        .expect("a fresh deterministic free-run build accepts an armed epoch");
    let before = real_ns();
    rt.run_live(&std::sync::atomic::AtomicBool::new(false))
        .expect("run_live with a pre-flipped running flag runs its prologue and no step");
    let after = real_ns();
    let placed = clock.now_ns();
    assert!(
        before <= placed && placed <= after,
        "run_live placed the armed epoch from real_ns() at its anchor: {before} <= {placed} <= \
         {after}"
    );
    assert_eq!(
        rt.place_pending_live_epoch_for_test()
            .expect("nothing pending is not an error"),
        None,
        "run_live spent the pending placement"
    );
    assert_eq!(
        fires.load(Ordering::SeqCst),
        0,
        "zero steps ran under a pre-flipped flag"
    );
    // The re-phased deadline: the first step after the placement fires the
    // ticker exactly one interval past the epoch (a build at the epoch would
    // baseline `epoch + 4 ms`) — never a catch-up burst sized by the epoch.
    rt.run_live_step_once_for_test(STEP_TIMEOUT);
    let fire_times: Vec<u64> = rt
        .trace()
        .iter()
        .filter(|e| &*e.node_id == "ticker")
        .map(|e| e.fire_time_ns)
        .collect();
    assert_eq!(
        fire_times,
        vec![placed + PERIOD_NS],
        "the first fire after the anchor placement lands one interval past the epoch"
    );
    rt.shutdown();
}

/// **Arming after a step is refused** —
/// the scheduler's `steps_begun > 0` refusal is the one arm-time check the two
/// build-shape arms above never reach (both refuse earlier, in the runtime's
/// own checks), so without this arm deleting the scheduler's arm-time check passes
/// the whole suite while the doc claims every refusal is made at arm time. One step
/// through the seam, then the arm: a loud `Err` naming "already begun", the
/// clock exactly where the step left it, nothing pending.
#[test]
fn arming_the_epoch_after_a_step_is_refused_with_nothing_pending() {
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("frc_arm_after_step", clock.clone());
    let (config, factories) = ticker_graph("frcaas", Arc::new(AtomicU64::new(0)));
    let mut rt = GraphRuntime::build_live_deterministic_free_run(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        None,
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the free-run deterministic ticker");
    rt.run_live_step_once_for_test(STEP_TIMEOUT);
    let stepped_to = clock.now_ns();
    assert!(stepped_to > 0, "one step advanced the controlled clock");
    let err = rt
        .place_gating_epoch_at_live_anchor()
        .expect_err("an epoch is an origin — arming after a step is refused");
    assert!(
        err.to_string().contains("already begun")
            && err
                .to_string()
                .contains("place_gating_epoch_at_live_anchor()"),
        "the refusal names the begun step and the arm entry point: {err}"
    );
    assert_eq!(
        clock.now_ns(),
        stepped_to,
        "a refused arm moves the clock by nothing"
    );
    assert_eq!(
        rt.place_pending_live_epoch_for_test()
            .expect("nothing pending is not an error"),
        None,
        "a refused arm leaves nothing pending"
    );
    rt.shutdown();
}

/// **An armed epoch that
/// is stepped past or dropped unspent is LOUD.** The arm is spent only by
/// `run_live`; a host that arms and then drives the polled seam runs on the
/// un-placed clock (origin 0), and one that drops the runtime never placed
/// it. Neither is a production shape; both would otherwise be silent. The polled warn
/// fires ONCE (on the first step, not per step — a second step adds no line),
/// the flag stays armed (so a later `run_live` still meets the "already begun"
/// refusal instead of placing a late epoch), and `Drop` says the arm was never
/// placed. A spent arm is the negative control: no line from either site.
#[test]
#[tracing_test::traced_test]
fn an_armed_epoch_stepped_past_or_dropped_unspent_warns_once_at_each_site() {
    const POLLED: &str = "is still PENDING while the runtime is stepped through the polled seam";
    const DROPPED: &str = "was never placed — run_live never reached its";
    let clock = Arc::new(VirtualClock::new());
    let mgr = manager("frc_armed_polled", clock.clone());
    let (config, factories) = ticker_graph("frcap2", Arc::new(AtomicU64::new(0)));
    let mut rt = GraphRuntime::build_live_deterministic_free_run(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        None,
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the free-run deterministic ticker");
    rt.place_gating_epoch_at_live_anchor()
        .expect("a fresh deterministic free-run build accepts an armed epoch");
    rt.step(Duration::from_millis(4));
    rt.step(Duration::from_millis(4));
    logs_assert(|lines| {
        let n = lines.iter().filter(|l| l.contains(POLLED)).count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!(
                "exactly ONE polled-seam warn across two steps, got {n}"
            ))
        }
    });
    let err = rt
        .run_live(&std::sync::atomic::AtomicBool::new(false))
        .expect_err("the arm stays pending, so run_live meets the already-begun refusal");
    assert!(err.to_string().contains("already begun"), "{err}");
    assert!(
        !logs_contain(DROPPED),
        "the Drop-site line must not appear before the runtime is dropped"
    );
    // `shutdown` consumes the runtime — its `Drop` runs here.
    rt.shutdown();
    assert!(
        logs_contain(DROPPED),
        "a runtime dropped with its arm pending says the epoch was never placed"
    );

    // Negative control: a SPENT arm logs from neither site.
    let clock2 = Arc::new(VirtualClock::new());
    let mgr2 = manager("frc_armed_spent", clock2.clone());
    let (config2, factories2) = ticker_graph("frcas2", Arc::new(AtomicU64::new(0)));
    let mut rt2 = GraphRuntime::build_live_deterministic_free_run(
        config2,
        factories2,
        &mgr2,
        Arc::clone(&clock2),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        None,
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the second free-run deterministic ticker");
    rt2.place_gating_epoch_at_live_anchor().expect("armed");
    rt2.place_pending_live_epoch_for_test()
        .expect("spent")
        .expect("an epoch was pending");
    rt2.step(Duration::from_millis(4));
    rt2.shutdown();
    logs_assert(|lines| {
        let polled = lines.iter().filter(|l| l.contains(POLLED)).count();
        let dropped = lines.iter().filter(|l| l.contains(DROPPED)).count();
        if polled == 1 && dropped == 1 {
            Ok(())
        } else {
            Err(format!(
                "a spent arm adds no line at either site: polled {polled} (want 1), dropped \
                 {dropped} (want 1)"
            ))
        }
    });
}

/// Build the watcher graph (one input on the in-prefix absolute topic
/// `/{prefix}/ext`, with no producer in view: a worker's cross-group edge)
/// through one of the two free-run ctors, optionally naming that topic as
/// sibling-produced.
fn build_watcher(prefix: &str, deterministic: bool, name_the_sibling: bool) {
    let siblings = std::collections::BTreeSet::from([format!("/{prefix}/ext")]);
    let mut wiring = CrossProcessWiring::requirements_only(None);
    if name_the_sibling {
        wiring = wiring.with_sibling_topics(&siblings);
    }
    let (config, factories) = watcher_graph(prefix);
    let off = cerulion_core::MonitorWaitPolicy::off();
    let rt = if deterministic {
        let clock = Arc::new(VirtualClock::new());
        let mgr = manager(prefix, clock.clone());
        GraphRuntime::build_live_deterministic_free_run(
            config, factories, &mgr, clock, None, off, None, wiring,
        )
    } else {
        let real: Arc<dyn Clock> = Arc::new(RealClock);
        let mgr = manager(prefix, Arc::clone(&real));
        GraphRuntime::build_live_free_run(config, factories, &mgr, real, None, off, wiring)
    };
    rt.expect("build the watcher").shutdown();
}

/// A worker's build validates its own slice of the graph, so it has to be told
/// which absolute sources a sibling group produces, or it reports a correct
/// cross-group edge as a possible typo. BOTH cross-process build cores are
/// driven (the RealClock one and the controlled-clock one the lockstep ctor
/// shares), each beside its own control: the same graph with nothing named
/// still warns, so the silence is the wiring's doing.
#[test]
#[tracing_test::traced_test]
fn both_cross_process_build_cores_hand_the_sibling_topics_to_validation() {
    build_watcher("frsa", false, true);
    build_watcher("frsb", false, false);
    build_watcher("frsc", true, true);
    build_watcher("frsd", true, false);
    logs_assert(|lines: &[&str]| {
        let warns = |prefix: &str| {
            lines
                .iter()
                .filter(|l| l.contains("matches no declared output"))
                .filter(|l| l.contains(&format!("/{prefix}/ext")))
                .count()
        };
        let got = [warns("frsa"), warns("frsb"), warns("frsc"), warns("frsd")];
        if got == [0, 1, 0, 1] {
            Ok(())
        } else {
            Err(format!(
                "typo warns per build, expected [0, 1, 0, 1], got {got:?}"
            ))
        }
    });
}
