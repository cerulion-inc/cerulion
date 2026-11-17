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
//! Two pins, neither judged against the wall clock:
//!
//! - **The wait sizing, on a controlled clock.** A one-rank lockstep runtime
//!   reads its gating clock as `now` before the loop anchors, so the wait
//!   `live_timeout` sizes is a function of where the test puts that clock: a
//!   deadline 700 µs ahead is waited for exactly, one closer than 10 µs waits
//!   10 µs, and a deadline already passed keeps the 1 ms poll. The pure halves
//!   are pinned by `next_fire_split_tests` and `live_wait_tests`; this pins
//!   the live loop composing them.
//! - **Delivery, as an account.** A `period_ms = 1` producer with a 300 µs
//!   tick runs on the production `live_step` and its own `live_timeout`, on
//!   the monitor-wait park the live loop runs by default (the degraded
//!   sleep-recheck park on a host without the CPU primitive). Every frame
//!   published after the warm-up is received or counted by the consumer's own
//!   eviction counter (`backpressure_drop_oldest_count`); on Linux every frame
//!   is received. A node that falls behind its grid catches up in a burst, and
//!   a burst longer than the consumer's queue evicts the oldest frames: on
//!   macOS the degraded park nap can wake later than the 700 µs the tick leaves
//!   in each period.
//!
//! Whether the node keeps its grid is a wall-clock claim, so it is measured in
//! the release latency tier and not here: `exact_deadline_grid_latency_test`
//! (release only, Linux only, run by the `test-latency` job on pushes to main)
//! judges the fire gaps and the wait after a slow step against wall time.
//! Per-test SHM roots; `#[serial]` because the delivery run is live.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::barrier::MappedBarrier;
use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::RealClock;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// The time each of the producer's ticks takes.
const TICK_WORK: Duration = Duration::from_micros(300);
/// Fires the run collects before it judges.
const FIRES: usize = 1_000;
/// Fires discarded first. The node's first deadline is set when the graph is
/// built, so the first live step finds it several periods behind and catches
/// up with a burst (each tick costing `TICK_WORK`); delivery is judged after.
const WARMUP: usize = 100;
/// The producer's period, in ns.
const PERIOD_NS: u64 = 1_000_000;
/// The longest the run may take; a stalled loop fails instead of hanging.
const BUDGET: Duration = Duration::from_secs(6);
/// The longest the consumer may take to empty its queue, at the warm-up and at
/// the end; a consumer that never catches up fails instead of hanging.
const SETTLE_BUDGET: Duration = Duration::from_secs(2);

#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct SlowTicker {
    #[output]
    out: Vector3,
    /// Each tick's start, `real_ns()`.
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
    /// The producer's number on the last frame received (frames are numbered
    /// from 1, one per fire).
    last: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl Receiver {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last.store(self.inp.x as u64, Ordering::Relaxed);
        self.received.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

fn transport(clock: Arc<dyn Clock>) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: "cerulion_exact_deadline".into(),
            clock,
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
    last: Arc<AtomicU64>,
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
            last,
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

/// The delivery account at one step boundary: the producer's fires, the
/// frames the consumer received, and the frames its queue evicted.
#[derive(Debug, Clone, Copy)]
struct Account {
    fires: u64,
    received: u64,
    evicted: u64,
}

fn fire_count(rt: &GraphRuntime) -> u64 {
    rt.node_handle("ticker")
        .expect("ticker handle")
        .fire_count()
}

/// Step `rt` until the consumer's queue is empty: the last frame it received
/// is the producer's last fire, so every earlier frame was received or evicted
/// and the eviction counter has seen each gap. The account there, or `None`
/// when the queue does not empty within [`SETTLE_BUDGET`].
fn settle(rt: &mut GraphRuntime, received: &AtomicU64, last: &AtomicU64) -> Option<Account> {
    let started = Instant::now();
    loop {
        let fires = fire_count(rt);
        if last.load(Ordering::Relaxed) == fires {
            return Some(Account {
                fires,
                received: received.load(Ordering::Relaxed),
                evicted: rt
                    .node_handle("receiver")
                    .expect("receiver handle")
                    .backpressure_drop_oldest_count("inp"),
            });
        }
        if started.elapsed() >= SETTLE_BUDGET {
            return None;
        }
        let timeout = rt.live_timeout_for_test();
        rt.run_live_step_once_for_test(timeout);
    }
}

#[test]
#[serial]
fn every_frame_a_slow_one_kilohertz_node_publishes_is_received_or_counted_evicted() {
    let transport = transport(Arc::new(RealClock));
    let ticks = Arc::new(Mutex::new(Vec::with_capacity(FIRES + 64)));
    let received = Arc::new(AtomicU64::new(0));
    let last = Arc::new(AtomicU64::new(0));
    let (config, factories) = graph(Arc::clone(&ticks), Arc::clone(&received), Arc::clone(&last));
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
    // The account once the warm-up has run and the consumer has emptied its
    // queue: the burst the warm-up catches up with can overrun that queue, and
    // the eviction counter starts counting only at the first frame it sees, so
    // delivery is judged from here on.
    let mut at_warmup = None;
    while ticks.lock().expect("tick log").len() < WARMUP + FIRES && started.elapsed() < BUDGET {
        let timeout = rt.live_timeout_for_test();
        rt.run_live_step_once_for_test(timeout);
        if at_warmup.is_none() && ticks.lock().expect("tick log").len() >= WARMUP {
            at_warmup = Some(
                settle(&mut rt, &received, &last)
                    .expect("the consumer emptied its queue after the warm-up"),
            );
        }
    }
    let at_end = settle(&mut rt, &received, &last).unwrap_or_else(|| {
        panic!(
            "the consumer did not empty its queue within {SETTLE_BUDGET:?}: the last frame it \
             received is {} of {} published",
            last.load(Ordering::Relaxed),
            fire_count(&rt)
        )
    });

    let all = ticks.lock().expect("tick log").clone();
    assert!(
        all.len() >= WARMUP + FIRES,
        "the live loop ran {} fires in {BUDGET:?}; expected {}",
        all.len(),
        WARMUP + FIRES
    );

    assert_eq!(
        at_end.fires as usize,
        all.len(),
        "every fire logged its tick"
    );
    let at_warmup = at_warmup.expect("the run passed its warm-up");
    let published = at_end.fires - at_warmup.fires;
    let delivered = at_end.received - at_warmup.received;
    let evicted = at_end.evicted - at_warmup.evicted;
    assert_eq!(
        delivered + evicted,
        published,
        "every frame published after the warm-up is received or counted as evicted by the \
         consumer's queue: {delivered} received, {evicted} evicted, {published} published"
    );
    // The Linux park wakes promptly enough that the producer never falls far
    // enough behind to overrun the consumer's queue, so every frame arrives.
    #[cfg(target_os = "linux")]
    assert_eq!(
        delivered, published,
        "the consumer received every frame the producer published after the warm-up"
    );
}

/// The wait the live loop sizes, on a controlled clock: a one-rank lockstep
/// runtime (the `CERULION_EXECUTION_MODE=lockstep` worker shape) whose gating
/// clock is a `VirtualClock` the test moves. Before the loop anchors, the wait
/// sizing reads that clock as `now`, so each wait below is exact. The handed
/// quantum is the period (it cannot be coarser than the node's own timing),
/// and with `now` on the gating clock its cap is one whole quantum, 1 ms,
/// which binds none of the three waits: the Period deadline alone sizes them.
#[test]
#[serial]
fn a_period_deadline_still_ahead_is_waited_for_exactly_on_a_controlled_clock() {
    let clock = Arc::new(VirtualClock::new());
    let transport = transport(clock.clone());
    // The producer alone: a one-rank barrier carries one level.
    let (mut config, mut factories) = graph(
        Arc::new(Mutex::new(Vec::new())),
        Arc::new(AtomicU64::new(0)),
        Arc::new(AtomicU64::new(0)),
    );
    config.nodes.retain(|n| n.id == "ticker");
    factories.shift_remove("receiver");
    let ns = format!("xdlv_{}", std::process::id());
    let barrier = Arc::new(MappedBarrier::create_owned(&ns, "g", 1).expect("barrier owner"));
    let rt = GraphRuntime::build_live_deterministic_with_manager_and_barrier(
        config,
        factories,
        &transport,
        Arc::clone(&clock),
        None,
        cerulion_core::MonitorWaitPolicy::off(),
        barrier,
        vec![Some(0)],
        vec![false],
        0,
        Duration::from_nanos(PERIOD_NS),
        cerulion_core::graph::runtime::CrossProcessWiring::requirements_only(None),
    )
    .expect("build the lockstep graph");
    assert_eq!(clock.now_ns(), 0, "the gating clock starts at 0");
    assert_eq!(
        rt.live_timeout_for_test(),
        Duration::from_nanos(PERIOD_NS),
        "a freshly built node's first deadline is one period ahead"
    );

    clock.advance(300_000);
    assert_eq!(
        rt.live_timeout_for_test(),
        Duration::from_micros(700),
        "300 µs into the period the wait is the 700 µs left, not a 1 ms floor"
    );

    clock.advance(695_000);
    assert_eq!(
        rt.live_timeout_for_test(),
        Duration::from_micros(10),
        "5 µs before the deadline the wait is the 10 µs deadline floor"
    );

    clock.advance(205_000);
    assert_eq!(
        rt.live_timeout_for_test(),
        Duration::from_millis(1),
        "a deadline already passed keeps the 1 ms poll"
    );
    rt.shutdown();
}
