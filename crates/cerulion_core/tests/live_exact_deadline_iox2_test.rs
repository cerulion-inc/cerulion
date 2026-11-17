// SPDX-License-Identifier: AGPL-3.0-only
//! The live loop waits to a Period node's exact deadline, over real iceoryx2.
//!
//! The wait for a Period deadline still ahead is the time left, not a 1 ms
//! floor. With a 1 ms floor a `period_ms = 1` node whose tick takes 300 µs
//! waits a full 1 ms after every step, so each fire lands about 300 µs later
//! than the last until a slot is passed and caught up with a double fire in
//! one step: roughly one fire in four. With the exact deadline every fire lands
//! on its 1 ms grid.
//!
//! The pin is at the delivery level: a downstream consumer receives every
//! frame the producer published, and the producer's own tick times show no
//! two fires closer than half a period (a double fire) beyond a small budget.
//! The budget is wide on purpose, so a loaded CI runner passes: the two
//! behaviours differ by an order of magnitude, not by a margin.
//!
//! Drives the production `live_step` with its own `live_timeout`, on the
//! monitor-wait park the live loop runs by default (the degraded sleep-recheck
//! park on a host without the CPU primitive). Per-test SHM roots; `#[serial]`
//! because the assertions are about wall-clock timing.
//!
//! The grid and wait-sizing assertions need a park that wakes within a few
//! hundred µs of its deadline, so they run on Linux. macOS has no CPU
//! monitor-wait primitive and parks in a degraded nap, whose wake on a hosted
//! runner can land later than the 700 µs this tick leaves in each period;
//! there the first arm judges delivery and the fire count only.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::clock::Clock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::RealClock;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// The producer's period and the time each of its ticks takes.
#[cfg(target_os = "linux")]
const PERIOD: Duration = Duration::from_millis(1);
const TICK_WORK: Duration = Duration::from_micros(300);
/// Fires the run collects before it judges.
const FIRES: usize = 1_000;
/// Fires discarded first. The node's first deadline is set when the graph is
/// built, so the first live step finds it several periods behind and catches
/// up with a burst (each tick costing `TICK_WORK`); the grid is judged after.
const WARMUP: usize = 100;
/// The longest the run may take; a stalled loop fails instead of hanging.
const BUDGET: Duration = Duration::from_secs(6);
/// The most fires closer than half a period to the previous one that a passing
/// run may have, as a share of all fires: 2 %, where a 1 ms floor gives about
/// 25 %.
#[cfg(target_os = "linux")]
const DOUBLE_FIRE_BUDGET: f64 = 0.02;

#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct SlowTicker {
    #[output]
    out: Vector3,
    /// Each tick's start, `real_ns()` (the wall clock this test judges).
    ticks: Arc<Mutex<Vec<u64>>>,
    n: u64,
}

#[cerulion_node_impl]
impl SlowTicker {
    fn tick(&mut self) -> Result<(), NodeError> {
        let started = self.real_ns();
        self.ticks
            .lock()
            .map_err(|_| NodeError::Logic("tick log poisoned".to_string()))?
            .push(started);
        while self.real_ns().saturating_sub(started) < TICK_WORK.as_nanos() as u64 {
            std::hint::spin_loop();
        }
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

#[cerulion_node]
#[derive(Default)]
struct Receiver {
    #[input(trigger)]
    inp: Vector3,
    received: Arc<AtomicU64>,
    last: f64,
}

#[cerulion_node_impl]
impl Receiver {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last = self.inp.x;
        self.received.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

fn transport() -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: "cerulion_exact_deadline".into(),
            clock: Arc::new(RealClock) as Arc<dyn Clock>,
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init per-test live transport")
}

fn graph(
    ticks: Arc<Mutex<Vec<u64>>>,
    received: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let node =
        |id: &str, node_type: &str, inputs: Vec<InputDef>, outputs: Vec<OutputDef>| NodeDef {
            isolate: None,
            params: Default::default(),
            ros2: None,
            id: id.to_string(),
            node_type: node_type.to_string(),
            inputs,
            outputs,
        };
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "exact_deadline".to_string(),
        prefix: "xdl".to_string(),
        nodes: vec![
            node(
                "ticker",
                "slow_ticker",
                vec![],
                vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            ),
            node(
                "receiver",
                "receiver",
                vec![InputDef {
                    name: "inp".to_string(),
                    source: "ticker/out".to_string(),
                    ..Default::default()
                }],
                vec![],
            ),
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "ticker".to_string(),
        Box::new(SlowTickerEntry::with_state(SlowTicker {
            ticks,
            ..Default::default()
        })),
    );
    factories.insert(
        "receiver".to_string(),
        Box::new(ReceiverEntry::with_state(Receiver {
            received,
            ..Default::default()
        })),
    );
    (config, factories)
}

/// The live loop's default idle: the monitor-wait park with the doorbell off
/// (the graph has no input written from outside it).
fn park_policy() -> cerulion_core::MonitorWaitPolicy {
    cerulion_core::MonitorWaitPolicy::new(true, false, format!("xdl{}", std::process::id()))
}

#[test]
#[serial]
fn a_one_kilohertz_node_with_a_slow_tick_stays_on_its_grid_and_delivers_every_frame() {
    let transport = transport();
    let ticks = Arc::new(Mutex::new(Vec::with_capacity(FIRES + 64)));
    let received = Arc::new(AtomicU64::new(0));
    let (config, factories) = graph(Arc::clone(&ticks), Arc::clone(&received));
    let mut rt = GraphRuntime::build_live_with_schema_hashes_and_policy(
        config,
        factories,
        &transport,
        Arc::new(RealClock),
        None,
        park_policy(),
    )
    .expect("build the live graph");

    let fired = |rt: &GraphRuntime| {
        rt.node_handle("ticker")
            .expect("ticker handle")
            .fire_count()
    };
    let started = Instant::now();
    // (fires, frames received) at the first step boundary past the warm-up:
    // the burst the warm-up catches up with can overrun the consumer's queue,
    // so delivery is judged from here on.
    let mut at_warmup = None;
    while ticks.lock().expect("tick log").len() < WARMUP + FIRES && started.elapsed() < BUDGET {
        let timeout = rt.live_timeout_for_test();
        rt.run_live_step_once_for_test(timeout);
        if at_warmup.is_none() && ticks.lock().expect("tick log").len() >= WARMUP {
            at_warmup = Some((fired(&rt), received.load(Ordering::Relaxed)));
        }
    }
    // One more pass so the last frame reaches the consumer.
    let timeout = rt.live_timeout_for_test();
    rt.run_live_step_once_for_test(timeout);

    let all = ticks.lock().expect("tick log").clone();
    assert!(
        all.len() >= WARMUP + FIRES,
        "the live loop ran {} fires in {BUDGET:?}; expected {}",
        all.len(),
        WARMUP + FIRES
    );
    #[cfg(target_os = "linux")]
    assert_on_grid(&all[WARMUP..]);

    let fires = fired(&rt);
    assert_eq!(fires as usize, all.len(), "every fire logged its tick");
    let (fires_at, received_at) = at_warmup.expect("the run passed its warm-up");
    assert_eq!(
        received.load(Ordering::Relaxed) - received_at,
        fires - fires_at,
        "the consumer received every frame the producer published after the warm-up"
    );
}

/// No more than [`DOUBLE_FIRE_BUDGET`] of the gaps between `ticks` are under
/// half a period, and the median gap is the period.
#[cfg(target_os = "linux")]
fn assert_on_grid(ticks: &[u64]) {
    let half = PERIOD / 2;
    let gaps: Vec<Duration> = ticks
        .windows(2)
        .map(|w| Duration::from_nanos(w[1].saturating_sub(w[0])))
        .collect();
    let doubles = gaps.iter().filter(|g| **g < half).count();
    let share = doubles as f64 / gaps.len() as f64;
    assert!(
        share <= DOUBLE_FIRE_BUDGET,
        "{doubles} of {} fires came less than half a period after the previous one \
         ({:.1} %, budget {:.1} %): the loop is not waiting to the exact deadline",
        gaps.len(),
        share * 100.0,
        DOUBLE_FIRE_BUDGET * 100.0
    );
    let mut sorted = gaps.clone();
    sorted.sort();
    let median = sorted[sorted.len() / 2];
    assert!(
        median >= PERIOD - Duration::from_micros(100)
            && median <= PERIOD + Duration::from_micros(100),
        "the median gap between fires is {median:?}; the grid is {PERIOD:?}"
    );
}

/// The same graph, stepped by hand: once the node is on its grid, the wait the
/// loop sizes after a step that fired is the time left to the next deadline,
/// not a whole period.
#[cfg(target_os = "linux")]
#[test]
#[serial]
fn after_a_slow_step_the_wait_is_the_time_left_not_a_period() {
    let transport = transport();
    let ticks = Arc::new(Mutex::new(Vec::new()));
    let received = Arc::new(AtomicU64::new(0));
    let (config, factories) = graph(Arc::clone(&ticks), Arc::clone(&received));
    let mut rt = GraphRuntime::build_live_with_schema_hashes_and_policy(
        config,
        factories,
        &transport,
        Arc::new(RealClock),
        None,
        park_policy(),
    )
    .expect("build the live graph");
    let started = Instant::now();
    let mut judged = 0;
    while judged < 20 && started.elapsed() < BUDGET {
        let before = ticks.lock().expect("tick log").len();
        let timeout = rt.live_timeout_for_test();
        rt.run_live_step_once_for_test(timeout);
        let log = ticks.lock().expect("tick log").clone();
        // Judge only a step that fired exactly once, past the warm-up burst.
        if log.len() < WARMUP || log.len() != before + 1 {
            continue;
        }
        let fired_at = log[log.len() - 1];
        let wait = rt.live_timeout_for_test();
        let since = Duration::from_nanos(cerulion_core::clock::real_ns().saturating_sub(fired_at));
        let left = PERIOD.saturating_sub(since);
        assert!(
            wait < PERIOD,
            "the wait after a {TICK_WORK:?} step that fired is {wait:?}: a whole period or more"
        );
        assert!(
            wait <= left + Duration::from_micros(100),
            "the wait {wait:?} overshoots the {left:?} left to the deadline"
        );
        judged += 1;
    }
    assert_eq!(judged, 20, "the run reached 20 judged steps in {BUDGET:?}");
}
