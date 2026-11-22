// SPDX-License-Identifier: AGPL-3.0-only
//! A 1 kHz node keeps its grid on the wall clock, release only, on Linux.
//!
//! The live loop waits to a Period node's exact deadline, so a `period_ms = 1`
//! node whose tick takes 300 µs fires once per 1 ms slot. With the old 1 ms
//! floor on every wait, each fire landed about 300 µs later than the last until
//! a slot was passed and caught up with a double fire: roughly one fire in
//! four. Two wall-clock pins:
//!
//! - **The grid**: after the warm-up, at most 2 % of fires come less than half
//!   a period after the previous one, and the median gap is the period
//!   (within 100 µs).
//! - **The wait after a slow step**: once the node is on its grid, the wait the
//!   loop sizes after a step that fired is the time left to the next deadline,
//!   not a whole period.
//!
//! Both judge wall time, so they live in the release latency tier: the
//! `test-latency` job runs this file on pushes to main
//! (`cargo test -p cerulion_core --test exact_deadline_grid_latency_test
//! --release -- --nocapture --test-threads=1`), and no pull-request shard does,
//! because a loaded shared runner delays the park's wake and the node then
//! catches up in double fires that say nothing about the code. Linux only: the
//! grid needs a park that wakes within a few hundred µs of its deadline, and
//! macOS parks in a degraded nap. The wait sizing itself is pinned without a
//! wall clock in `live_exact_deadline_iox2_test` and by the `live_wait_tests`
//! oracle.
//!
//! Drives the production `live_step` with its own `live_timeout`, on the
//! monitor-wait park the live loop runs by default. Per-test SHM roots;
//! `#[serial]` because the assertions are about wall-clock timing. A green run
//! prints the measured double-fire share and median gap.

#![cfg(all(not(debug_assertions), target_os = "linux"))]

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::clock::Clock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::RealClock;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// The producer's period and the time each of its ticks takes.
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
const DOUBLE_FIRE_BUDGET: f64 = 0.02;

#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct SlowTicker {
    #[output]
    out: Vector3,
    /// Each tick's start, `real_ns()` (the wall clock this test judges).
    ticks: Arc<Mutex<Vec<u64>>>,
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
        self.out.x = 1.0;
        Ok(())
    }
}

fn transport() -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: "cerulion_exact_deadline_grid".into(),
            clock: Arc::new(RealClock) as Arc<dyn Clock>,
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init per-test live transport")
}

/// The one-node graph on the monitor-wait park with the doorbell off (the
/// graph has no input written from outside it).
fn runtime(ticks: Arc<Mutex<Vec<u64>>>, transport: &Arc<TransportManager>) -> GraphRuntime {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "exact_deadline_grid".to_string(),
        prefix: "xdg".to_string(),
        nodes: vec![NodeDef {
            isolate: None,
            params: Default::default(),
            ros2: None,
            id: "ticker".to_string(),
            node_type: "slow_ticker".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "ticker".to_string(),
        Box::new(SlowTickerEntry::with_state(SlowTicker {
            ticks,
            ..Default::default()
        })),
    );
    GraphRuntime::build_live_with_schema_hashes_and_policy(
        config,
        factories,
        transport,
        Arc::new(RealClock),
        None,
        cerulion_core::MonitorWaitPolicy::new(true, false, format!("xdg{}", std::process::id())),
    )
    .expect("build the live graph")
}

#[test]
#[serial]
fn a_one_kilohertz_node_with_a_slow_tick_stays_on_its_grid() {
    let transport = transport();
    let ticks = Arc::new(Mutex::new(Vec::with_capacity(WARMUP + FIRES + 64)));
    let mut rt = runtime(Arc::clone(&ticks), &transport);
    let started = Instant::now();
    while ticks.lock().expect("tick log").len() < WARMUP + FIRES && started.elapsed() < BUDGET {
        let timeout = rt.live_timeout_for_test();
        rt.run_live_step_once_for_test(timeout);
    }
    let all = ticks.lock().expect("tick log").clone();
    assert!(
        all.len() >= WARMUP + FIRES,
        "the live loop ran {} fires in {BUDGET:?}; expected {}",
        all.len(),
        WARMUP + FIRES
    );

    let gaps: Vec<Duration> = all[WARMUP..]
        .windows(2)
        .map(|w| Duration::from_nanos(w[1].saturating_sub(w[0])))
        .collect();
    let doubles = gaps.iter().filter(|g| **g < PERIOD / 2).count();
    let share = doubles as f64 / gaps.len() as f64;
    let mut sorted = gaps.clone();
    sorted.sort();
    let median = sorted[sorted.len() / 2];
    println!(
        "exact-deadline grid: {doubles} of {} fires under half a period after the last \
         ({:.2} %), median gap {median:?}",
        gaps.len(),
        share * 100.0
    );
    assert!(
        share <= DOUBLE_FIRE_BUDGET,
        "{doubles} of {} fires came less than half a period after the previous one \
         ({:.1} %, budget {:.1} %): the loop is not waiting to the exact deadline",
        gaps.len(),
        share * 100.0,
        DOUBLE_FIRE_BUDGET * 100.0
    );
    assert!(
        median >= PERIOD - Duration::from_micros(100)
            && median <= PERIOD + Duration::from_micros(100),
        "the median gap between fires is {median:?}; the grid is {PERIOD:?}"
    );
}

#[test]
#[serial]
fn after_a_slow_step_the_wait_is_the_time_left_not_a_period() {
    let transport = transport();
    let ticks = Arc::new(Mutex::new(Vec::new()));
    let mut rt = runtime(Arc::clone(&ticks), &transport);
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
