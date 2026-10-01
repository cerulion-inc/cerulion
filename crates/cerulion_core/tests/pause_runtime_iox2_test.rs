// SPDX-License-Identifier: AGPL-3.0-only
//! A paused `GraphRuntime`: the hold, the stopped clock, and the barrier.
//!
//! # What is pinned, and why each arm is a hand oracle
//!
//! * **A paused live loop steps no node and its run clock stands still.** While
//!   the pause page says paused the ticker's fire counter does not move and the
//!   `PausableClock` the runtime was built on reads one value; after the resume
//!   the clock continues from that value, so a `period_ms` node neither skips
//!   ticks nor bursts (the fires in the window after the resume stay under what the
//!   window's own length allows, where a catch-up of the second the run stood still
//!   would add hundreds).
//! * **A clock that follows the wall excludes the paused time.** The recording
//!   shape (a controlled gating clock advanced by each step's measured wall time)
//!   stands still across the pause too, so a recorded timestamp does not jump.
//! * **A barrier wait outlasts a pause, and only a pause.** A lockstep participant
//!   whose peer is held waits past the five-second boundary timeout without
//!   poisoning the runtime; the same wait with no pause poisons it, so the arm
//!   cannot pass on a timeout that never fires.
//!
//! Every manager is an `init_for_test` per-test SHM root and every page is named by
//! a per-test tag, so the file is parallel-safe and needs no nextest fence entry.

#![cfg(unix)]

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use cerulion_core::barrier::MappedBarrier;
use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::CrossProcessWiring;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::pause_page::MappedPausePage;
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use cerulion_core::PausableClock;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// A pure-`Period` producer: the schedule a pause must neither skip nor burst.
#[cerulion_node(period_ms = 4)]
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

fn ticker_graph(
    prefix: &str,
    fires: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
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
            ..Default::default()
        })),
    );
    (config, factories)
}

/// A data-trigger consumer of an absolute external topic: it fires only when a step
/// drains a frame, so it is the node a held loop must NOT fire however many frames
/// arrive.
#[cerulion_node]
#[derive(Default)]
struct Consumer {
    #[input(trigger)]
    inp: Vector3,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl Consumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

fn consumer_graph(
    prefix: &str,
    topic: &str,
    fires: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("{prefix}_consumer"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "consumer".to_string(),
            node_type: "consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: topic.to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "consumer".to_string(),
        Box::new(ConsumerEntry::with_state(Consumer {
            fires,
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

fn page(what: &str) -> Arc<MappedPausePage> {
    Arc::new(
        MappedPausePage::create_owned(&format!("pause_rt_{what}_{}", std::process::id()))
            .expect("create the pause page"),
    )
}

/// What the helper thread observed around a pause.
#[derive(Debug)]
struct Observed {
    /// Fires once the hold is in place, and at the end of the hold.
    held_start: u64,
    held_end: u64,
    /// The clock under test, once the hold is in place, at the end of the hold, and
    /// after the post-resume window.
    clock_held_start: u64,
    clock_held_end: u64,
    clock_after: u64,
    /// Fires after the post-resume window.
    after: u64,
}

/// Drive the helper side of one pause against a runtime `run_live`-ing on the
/// calling thread: let it run, pause it for `hold`, resume it, let it run `window`,
/// stop it. `clock_now` reads the clock under test.
fn pause_around(
    page: &MappedPausePage,
    running: &AtomicBool,
    fires: &AtomicU64,
    clock_now: impl Fn() -> u64,
    hold: Duration,
    window: Duration,
) -> Observed {
    std::thread::sleep(Duration::from_millis(300));
    page.pause();
    // Long enough for a loop in the middle of an idle wait to reach its next step
    // boundary (a wait is at most 250 ms) and take the hold.
    std::thread::sleep(Duration::from_millis(400));
    let held_start = fires.load(Ordering::Relaxed);
    let clock_held_start = clock_now();
    std::thread::sleep(hold);
    let held_end = fires.load(Ordering::Relaxed);
    let clock_held_end = clock_now();
    page.resume();
    std::thread::sleep(window);
    let after = fires.load(Ordering::Relaxed);
    let clock_after = clock_now();
    running.store(false, Ordering::Relaxed);
    Observed {
        held_start,
        held_end,
        clock_held_start,
        clock_held_end,
        clock_after,
        after,
    }
}

/// The plain live build: the scheduler, the transport and the runtime all read the
/// pausable run clock.
#[test]
fn a_paused_run_steps_no_node_and_resumes_without_skipping_or_bursting() {
    let page = page("plain");
    let clock: Arc<dyn Clock> = Arc::new(PausableClock::new(Arc::clone(&page)));
    let fires = Arc::new(AtomicU64::new(0));
    let mgr = manager("pause_rt_plain", Arc::clone(&clock));
    let (config, factories) = ticker_graph("prtplain", Arc::clone(&fires));
    let mut rt = GraphRuntime::build_live_free_run(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the live ticker");
    rt.attach_pause(Arc::clone(&page));

    let running = AtomicBool::new(true);
    let hold = Duration::from_millis(1000);
    let window = Duration::from_millis(400);
    let observed = std::thread::scope(|s| {
        let helper =
            s.spawn(|| pause_around(&page, &running, &fires, || clock.now_ns(), hold, window));
        rt.run_live(&running).expect("run_live");
        helper.join().expect("helper")
    });
    rt.shutdown();

    assert!(
        observed.held_start > 20,
        "the ticker ran before the pause: {observed:?}"
    );
    assert_eq!(
        observed.held_end, observed.held_start,
        "no node steps while the run is paused: {observed:?}"
    );
    assert_eq!(
        observed.clock_held_end, observed.clock_held_start,
        "the run clock stands still while the run is paused: {observed:?}"
    );
    let resumed_fires = observed.after - observed.held_end;
    assert!(
        resumed_fires >= 10,
        "the ticker steps again after the resume: {observed:?}"
    );
    // The window is 400 ms of a 4 ms period: about 100 fires at most. A clock that
    // kept the hardware time would owe the ticker the whole 1 s it stood still (250
    // more) and the clamp that bounds that burst still leaves it far above this.
    assert!(
        resumed_fires <= 130,
        "a resume must not catch up the time the run stood still: {resumed_fires} fires in a \
         400 ms window; {observed:?}"
    );
    let run_time_across = observed.clock_after - observed.clock_held_end;
    assert!(
        run_time_across < 700_000_000,
        "the run clock continues from the frozen value instead of jumping by the pause: it \
         advanced {run_time_across} ns across a 400 ms window after a 1 s hold; {observed:?}"
    );
}

/// The hold is not only a stopped clock: a data-driven node is fed by frames that
/// keep arriving from outside the graph, and a held loop must not step it for them.
/// The frames wait in the input's queue, and the node fires on them after the resume.
#[test]
fn a_held_run_does_not_step_a_data_driven_node_for_frames_that_keep_arriving() {
    const TOPIC: &str = "/prtdata/ext/frames";
    let page = page("data");
    let clock: Arc<dyn Clock> = Arc::new(PausableClock::new(Arc::clone(&page)));
    let fires = Arc::new(AtomicU64::new(0));
    let mgr = manager("pause_rt_data", Arc::clone(&clock));
    let (config, factories) = consumer_graph("prtdata", TOPIC, Arc::clone(&fires));
    let mut rt = GraphRuntime::build_live_free_run(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the live consumer");
    rt.attach_pause(Arc::clone(&page));
    let mut publisher = mgr
        .create_publisher(TOPIC, MaxSliceLen::const_new(256), 0)
        .expect("an out-of-graph publisher attaches to the absolute external topic");

    let running = AtomicBool::new(true);
    let observed = std::thread::scope(|s| {
        // The outside world: one frame every 5 ms for the whole test, paused or not.
        let feeder = s.spawn(|| {
            let mut x = 0.0;
            while running.load(Ordering::Relaxed) {
                {
                    let mut proxy = publisher.loan_proxy::<Vector3>().expect("loan");
                    proxy.x = x;
                }
                x += 1.0;
                std::thread::sleep(Duration::from_millis(5));
            }
        });
        let helper = s.spawn(|| {
            pause_around(
                &page,
                &running,
                &fires,
                || clock.now_ns(),
                Duration::from_millis(1000),
                Duration::from_millis(600),
            )
        });
        rt.run_live(&running).expect("run_live");
        feeder.join().expect("feeder");
        helper.join().expect("helper")
    });
    rt.shutdown();

    assert!(
        observed.held_start >= 20,
        "the consumer ran on the frames before the pause: {observed:?}"
    );
    assert_eq!(
        observed.held_end, observed.held_start,
        "a held run must not step a node for the frames that keep arriving: {observed:?}"
    );
    assert!(
        observed.after > observed.held_end,
        "the frames that queued during the hold are served after the resume: {observed:?}"
    );
}

/// The recording shape: a controlled clock advanced by each step's measured wall time.
#[test]
fn a_clock_that_follows_the_wall_excludes_the_paused_time() {
    let page = page("wall");
    let gating = Arc::new(VirtualClock::new());
    let clock: Arc<dyn Clock> = gating.clone();
    let fires = Arc::new(AtomicU64::new(0));
    let mgr = manager("pause_rt_wall", Arc::clone(&clock));
    let (config, factories) = ticker_graph("prtwall", Arc::clone(&fires));
    let mut rt = GraphRuntime::build_live_deterministic_free_run(
        config,
        factories,
        &mgr,
        Arc::clone(&gating),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        None,
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the free-run recording ticker");
    rt.set_gating_follows_wall(true)
        .expect("a controlled clock can follow the wall");
    rt.attach_pause(Arc::clone(&page));

    let running = AtomicBool::new(true);
    let hold = Duration::from_millis(1000);
    let window = Duration::from_millis(400);
    let observed = std::thread::scope(|s| {
        let helper =
            s.spawn(|| pause_around(&page, &running, &fires, || gating.now_ns(), hold, window));
        rt.run_live(&running).expect("run_live");
        helper.join().expect("helper")
    });
    rt.shutdown();

    assert_eq!(
        observed.held_end, observed.held_start,
        "no node steps while the run is paused: {observed:?}"
    );
    assert_eq!(
        observed.clock_held_end, observed.clock_held_start,
        "the gating clock must not advance during the hold: {observed:?}"
    );
    let across = observed.clock_after - observed.clock_held_end;
    assert!(
        across > 0 && across < 700_000_000,
        "after a 1 s hold the gating clock advances by the 400 ms window, not by the hold: \
         {across} ns; {observed:?}"
    );
}

/// A lockstep participant whose only peer is held. The barrier here has two
/// participants and only this runtime steps; the peer is a thread that arrives at
/// the first boundary after `peer_arrives_after`.
fn lockstep_participant_waiting(
    what: &str,
    with_pause: bool,
    pause_for: Duration,
    peer_arrives_after: Duration,
) -> bool {
    let page = page(what);
    let clock: Arc<dyn Clock> = Arc::new(PausableClock::new(Arc::clone(&page)));
    let mgr = manager(&format!("pause_rt_{what}"), Arc::clone(&clock));
    let (config, factories) = ticker_graph(&format!("prt{what}"), Arc::new(AtomicU64::new(0)));
    let ns = format!("pause_rt_bar_{what}_{}", std::process::id());
    let owner = Arc::new(MappedBarrier::create_owned(&ns, "g", 2).expect("barrier owner"));
    let peer = Arc::new(MappedBarrier::open_unowned(&ns, "g").expect("barrier peer"));
    let mut rt = GraphRuntime::build_live_free_run(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the participant");
    rt.set_barrier_participant_for_test(owner, vec![Some(0)], 0);
    if with_pause {
        rt.attach_pause(Arc::clone(&page));
    }

    std::thread::scope(|s| {
        // The held peer: it reaches the boundary only after the pause is over.
        s.spawn(|| {
            std::thread::sleep(peer_arrives_after);
            let _ = peer.arrive(0);
        });
        if with_pause {
            s.spawn(|| {
                std::thread::sleep(Duration::from_millis(500));
                page.pause();
                std::thread::sleep(pause_for);
                page.resume();
            });
        }
        // One step: it arrives at the first boundary and waits for the peer.
        rt.step(Duration::from_millis(4));
    });
    let failed = rt.is_barrier_failed();
    rt.shutdown();
    failed
}

#[test]
fn a_barrier_wait_that_a_pause_outlasts_does_not_poison_the_runtime() {
    // The peer arrives at 7.5 s; the pause covers 0.5 s to 7.2 s; the boundary
    // timeout is 5 s. Without the pause-aware wait the runtime would poison at 5 s.
    let failed = lockstep_participant_waiting(
        "held",
        true,
        Duration::from_millis(6700),
        Duration::from_millis(7500),
    );
    assert!(
        !failed,
        "a peer held by a pause is not a dead peer: the wait must outlast the 5 s timeout"
    );
}

#[test]
fn the_same_wait_with_no_pause_poisons_the_runtime_at_the_boundary_timeout() {
    // The negative control: nothing is paused, the peer is as late, and the timeout
    // is the stall it has always meant.
    let failed = lockstep_participant_waiting(
        "control",
        false,
        Duration::ZERO,
        Duration::from_millis(7500),
    );
    assert!(
        failed,
        "without a pause the late peer must time the wait out and poison the runtime, or the \
         arm above proves nothing"
    );
}

/// The wait is only extended for a pause that overlaps it: a pause that ended long
/// before the timeout leaves the next five seconds as an ordinary window.
#[test]
fn a_pause_that_ended_early_does_not_excuse_a_peer_that_is_late_afterwards() {
    let started = Instant::now();
    let failed = lockstep_participant_waiting(
        "early",
        true,
        Duration::from_millis(300),
        Duration::from_millis(11_000),
    );
    assert!(
        failed,
        "a 300 ms pause at the start must not make a peer that arrives at 11 s look healthy: the \
         wait owes at most one extra window for the pause, then times out"
    );
    assert!(
        started.elapsed() < Duration::from_secs(13),
        "and the poison arrives promptly"
    );
}
