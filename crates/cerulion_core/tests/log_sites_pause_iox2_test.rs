// SPDX-License-Identifier: AGPL-3.0-only
//! The throttled and once-only log macros across a REAL pause and resume.
//!
//! A `period_ms = 4` node logs through an `info_once!` and a 100 ms
//! `info_throttle!` on every tick, inside a live `GraphRuntime` built on a
//! `PausableClock` with the run's pause page attached, exactly as
//! `pause_runtime_iox2_test` drives a pause. The run is paused for 1 s and then
//! resumed. Hand oracle:
//!
//! - the once line prints one time in the whole run, not again after the
//!   resume;
//! - every gap between throttled lines is at least the 100 ms period of graph
//!   time;
//! - the run clock reads one value through the hold, and the gap between the
//!   last throttled line before the hold and the first one after it is under
//!   the 1 s hold: the clock froze, so the paused second is not graph time the
//!   throttle counted. A clock that kept running through the hold would put
//!   that gap at 1 s or more;
//! - the pause is asked for as soon as a throttled line appears, so it lands
//!   early in a throttle window (under 50 ms of graph time after that line;
//!   a pause that lands later is let go and tried again on the next line),
//!   and the first throttled line after the resume still comes a full 100 ms
//!   of graph time after the last one before it. A throttle that counted the
//!   paused wall time would see its period long over at the resume and print
//!   on the first tick after it, well inside those 100 ms.
//!
//! Every event goes to one process-wide capture (the live loop may fire the
//! node off the calling thread), so this binary holds ONE test. The manager is
//! an `init_for_test` per-test SHM root and the page a per-process tag, so the
//! file needs no nextest fence entry.

#![cfg(unix)]

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::clock::Clock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::CrossProcessWiring;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::pause_page::MappedPausePage;
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::PausableClock;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use tracing::field::{Field, Visit};
use tracing::{Event, Subscriber};
use tracing_subscriber::layer::{Context, Layer};
use tracing_subscriber::prelude::*;

const MS: u64 = 1_000_000;

/// `(message, tick_ns, emit_ns)` of every node line: the clock read before
/// the macro call, and one read inside its arguments, which a call evaluates
/// only after the throttle read the clock itself. On this live clock the two
/// reads differ, and the throttle's own read lies between them.
#[derive(Clone, Default)]
struct Capture(Arc<Mutex<Vec<(String, u64, u64)>>>);

#[derive(Default)]
struct LineVisitor {
    message: String,
    tick_ns: Option<u64>,
    emit_ns: Option<u64>,
}

impl Visit for LineVisitor {
    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        if field.name() == "message" {
            self.message = format!("{value:?}");
        }
    }
    fn record_u64(&mut self, field: &Field, value: u64) {
        if field.name() == "tick_ns" {
            self.tick_ns = Some(value);
        }
        if field.name() == "emit_ns" {
            self.emit_ns = Some(value);
        }
    }
}

impl<S: Subscriber> Layer<S> for Capture {
    fn on_event(&self, event: &Event<'_>, _ctx: Context<'_, S>) {
        let mut v = LineVisitor::default();
        event.record(&mut v);
        if let Some(t) = v.tick_ns {
            let e = v.emit_ns.unwrap_or(t);
            self.0.lock().unwrap().push((v.message, t, e));
        }
    }
}

#[cerulion_node(period_ms = 4)]
#[derive(Default)]
struct Chatty {
    #[output]
    out: Vector3,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl Chatty {
    fn tick(&mut self) -> Result<(), NodeError> {
        let tick_ns = self.now_ns();
        info_throttle!(
            self,
            Duration::from_millis(100),
            tick_ns,
            emit_ns = self.now_ns(),
            "throttled"
        );
        info_once!(self, tick_ns, "first tick");
        self.out.x = 1.0;
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

fn graph(
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
        identity: format!("{prefix}_chatty"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "chatty".to_string(),
            node_type: "chatty".to_string(),
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
        "chatty".to_string(),
        Box::new(ChattyEntry::with_state(Chatty {
            fires,
            ..Default::default()
        })),
    );
    (config, factories)
}

/// Steps the node must have taken before the pause lands.
const RAN_BEFORE_PAUSE: u64 = 60;

#[test]
fn a_real_pause_prints_no_second_once_line_and_the_throttle_counts_no_paused_time() {
    let cap = Capture::default();
    tracing::subscriber::set_global_default(tracing_subscriber::registry().with(cap.clone()))
        .expect("the only subscriber in this binary");

    let page = Arc::new(
        MappedPausePage::create_owned(&format!("log_sites_pause_{}", std::process::id()))
            .expect("create the pause page"),
    );
    let clock: Arc<dyn Clock> = Arc::new(PausableClock::new(Arc::clone(&page)));
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "log_sites_pause".into(),
            clock: Arc::clone(&clock),
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init per-test transport");
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = graph("lspause", Arc::clone(&fires));
    let mut rt = GraphRuntime::build_live_free_run(
        config,
        factories,
        &mgr,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::requirements_only(None),
    )
    .expect("build the live node");
    rt.attach_pause(Arc::clone(&page));

    let hold = Duration::from_millis(1000);
    let running = AtomicBool::new(true);
    let (clock_held_start, clock_held_end, fires_held_start, fires_held_end) =
        std::thread::scope(|s| {
            let helper = s.spawn(|| {
                let started = Instant::now();
                while fires.load(Ordering::Relaxed) < RAN_BEFORE_PAUSE
                    && started.elapsed() < Duration::from_secs(30)
                {
                    std::thread::sleep(Duration::from_millis(5));
                }
                // Pause right after a throttled line, so the hold lands early
                // in its window and most of the period is still to run. A
                // loaded host can delay this thread past that point; then the
                // hold is let go and the next line is tried, a few times.
                for attempt in 1..=PAUSE_ATTEMPTS {
                    let seen = throttled_lines(&cap);
                    while throttled_lines(&cap) == seen
                        && started.elapsed() < Duration::from_secs(30)
                    {
                        std::thread::sleep(Duration::from_millis(1));
                    }
                    page.pause();
                    // A loop in an idle wait reaches its next step boundary (a
                    // wait is at most 250 ms) and takes the hold.
                    std::thread::sleep(Duration::from_millis(400));
                    let into_window = clock.now_ns() - last_throttled_ns(&cap);
                    if into_window < EARLY_IN_WINDOW || attempt == PAUSE_ATTEMPTS {
                        break;
                    }
                    page.resume();
                }
                let held = (clock.now_ns(), fires.load(Ordering::Relaxed));
                std::thread::sleep(hold);
                let held_end = (clock.now_ns(), fires.load(Ordering::Relaxed));
                page.resume();
                std::thread::sleep(Duration::from_millis(600));
                running.store(false, Ordering::Relaxed);
                (held.0, held_end.0, held.1, held_end.1)
            });
            rt.run_live(&running).expect("run_live");
            helper.join().expect("helper")
        });
    rt.shutdown();

    assert!(
        fires_held_start >= RAN_BEFORE_PAUSE,
        "the node ran before the pause"
    );
    assert_eq!(fires_held_end, fires_held_start, "no tick while paused");
    assert_eq!(
        clock_held_end, clock_held_start,
        "the run clock stands still while paused"
    );
    let frozen = clock_held_end;

    let lines = cap.0.lock().unwrap().clone();
    let once: Vec<u64> = lines
        .iter()
        .filter(|(m, _, _)| m == "first tick")
        .map(|&(_, t, _)| t)
        .collect();
    assert_eq!(once.len(), 1, "one once line in the whole run: {lines:?}");
    assert!(once[0] <= frozen, "the once line printed before the pause");

    // `(tick_ns, emit_ns)`: the throttle's own read of each emit lies between.
    let throttled: Vec<(u64, u64)> = lines
        .iter()
        .filter(|(m, _, _)| m == "throttled")
        .map(|&(_, t, e)| (t, e))
        .collect();
    // The throttle's reads were a full period apart, so the later line's
    // upper bound is at least a period past the earlier line's lower bound.
    for w in throttled.windows(2) {
        assert!(
            w[1].1 - w[0].0 >= 100 * MS,
            "throttled lines {} ns apart at most, under the period: {throttled:?}",
            w[1].1 - w[0].0
        );
    }
    let before = throttled.iter().rev().find(|&&(t, _)| t <= frozen).copied();
    let after = throttled.iter().find(|&&(t, _)| t > frozen).copied();
    let (Some(before), Some(after)) = (before, after) else {
        panic!("throttled lines on both sides of the pause at {frozen}: {throttled:?}");
    };
    assert!(
        after.0 - before.0 < hold.as_nanos() as u64,
        "the gap across the pause ({} ns) counts no paused time: {throttled:?}",
        after.0 - before.0
    );
    assert!(
        frozen - before.0 < EARLY_IN_WINDOW,
        "the pause landed {} ns after the last throttled line, not early in its window: \
         {throttled:?}",
        frozen - before.0
    );
    assert!(
        after.1 - before.0 >= 100 * MS,
        "the first throttled line after the resume waits out the rest of the period in graph \
         time (at most {} ns after the last one before the pause): {throttled:?}",
        after.1 - before.0
    );
}

/// Tries at landing the pause early in a throttle window.
const PAUSE_ATTEMPTS: u32 = 5;

/// How far into a 100 ms throttle window the pause must land for the
/// post-resume check to tell a graph-time throttle from a wall-time one.
const EARLY_IN_WINDOW: u64 = 50 * MS;

/// The graph time of the latest throttled line so far.
fn last_throttled_ns(cap: &Capture) -> u64 {
    cap.0
        .lock()
        .unwrap()
        .iter()
        .rev()
        .find(|(m, _, _)| m == "throttled")
        .map(|&(_, t, _)| t)
        .expect("a throttled line before the pause")
}

/// How many throttled lines the capture holds so far.
fn throttled_lines(cap: &Capture) -> usize {
    cap.0
        .lock()
        .unwrap()
        .iter()
        .filter(|(m, _, _)| m == "throttled")
        .count()
}
