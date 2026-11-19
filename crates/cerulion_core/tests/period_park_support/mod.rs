// SPDX-License-Identifier: AGPL-3.0-only
//! The period-park apparatus shared by `period_park_iox2_test` (counts and
//! order, on every pull request) and `period_park_latency_test` (wall-clock
//! bounds, release only, on Linux): the nodes, the two-runtime harness and the
//! 1 kHz ticker run.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::clock::{Clock, VirtualClock};
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::{GraphRuntime, PeriodParkStats};
use cerulion_core::prelude::*;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::{MonitorWaitPolicy, RealClock};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// A park timeout far longer than any arm's run: a wait that ends on it was
/// ended by no frame.
pub const DUE_PARK_TIMEOUT: Duration = Duration::from_secs(2);

fn ns(tag: &str) -> String {
    format!("ppk{}{tag}", std::process::id())
}

/// Does the period-park engage on this host at all? Read from the platform,
/// not from the code under test: a CPU with a monitor-wait primitive.
pub fn engages_here() -> bool {
    cerulion_core::monitor_wait::monitor_wait_available()
}

/// Does the period-park apply to a consumer of a topic written from another
/// runtime on this host?
pub fn applies_to_a_foreign_consumer() -> bool {
    engages_here() && cerulion_core::doorbell::wake_word_block_primitive_available()
}

pub fn transport(
    name: &str,
    ix: iceoryx2::config::Config,
    clock: Arc<dyn Clock>,
) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: name.into(),
            clock,
            subscriber_buffer_size: 32,
            network: None,
        },
        ix,
    )
    .expect("create a transport node on the shared iceoryx2 config")
}

pub fn node(id: &str, node_type: &str, inputs: Vec<InputDef>, outputs: Vec<OutputDef>) -> NodeDef {
    NodeDef {
        isolate: None,
        params: Default::default(),
        ros2: None,
        id: id.to_string(),
        node_type: node_type.to_string(),
        inputs,
        outputs,
    }
}

fn out(topic: Option<&str>) -> OutputDef {
    OutputDef {
        name: "out".to_string(),
        schema: "geometry_msgs/Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: topic.map(str::to_string),
    }
}

pub fn input(source: &str) -> InputDef {
    InputDef {
        name: "inp".to_string(),
        source: source.to_string(),
        ..Default::default()
    }
}

pub fn config(identity: &str, prefix: &str, nodes: Vec<NodeDef>) -> GraphConfig {
    GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: identity.to_string(),
        prefix: prefix.to_string(),
        nodes,
    }
}

/// Numbers its frames 1, 2, 3, ... and stamps each with the wall time it was
/// written (`y`), so a consumer can be held to the sequence and the delay.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct Stamper {
    #[output]
    out: Vector3,
    sent: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl Stamper {
    fn tick(&mut self) -> Result<(), NodeError> {
        let n = self.sent.fetch_add(1, Ordering::Relaxed) + 1;
        self.out.x = n as f64;
        self.out.y = self.real_ns() as f64;
        Ok(())
    }
}

/// Records each frame's number and how long after its stamp it was read.
#[cerulion_node]
#[derive(Default)]
struct Reader {
    #[input(trigger)]
    inp: Vector3,
    seen: Arc<Mutex<Vec<(f64, u64)>>>,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl Reader {
    fn tick(&mut self) -> Result<(), NodeError> {
        let delay = self.real_ns().saturating_sub(self.inp.y as u64);
        self.seen
            .lock()
            .map_err(|_| NodeError::Logic("seen log poisoned".to_string()))?
            .push((self.inp.x, delay));
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// A 1 kHz ticker with a light tick, logging each tick's wall time.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct Ticker {
    #[output]
    out: Vector3,
    ticks: Arc<Mutex<Vec<u64>>>,
}

#[cerulion_node_impl]
impl Ticker {
    fn tick(&mut self) -> Result<(), NodeError> {
        let now = self.real_ns();
        let mut ticks = self
            .ticks
            .lock()
            .map_err(|_| NodeError::Logic("tick log poisoned".to_string()))?;
        ticks.push(now);
        self.out.x = ticks.len() as f64;
        Ok(())
    }
}

/// What a consumer runtime on its own thread reports back.
pub struct Consumed {
    pub seen: Vec<(f64, u64)>,
    pub stats: PeriodParkStats,
    pub timeouts_while_due: u64,
}

/// A producer runtime on this thread and a consumer runtime on its own thread,
/// joined by `publish` (one of the consumer's `feeds`, each read by its own
/// node, declared in order, so the first is the armed line). The producer sends
/// `frames` frames `gap` apart while the consumer waits with the period-park on.
pub fn two_runtimes(
    tag: &str,
    feeds: &[&'static str],
    publish: &'static str,
    frames: u64,
    gap: Duration,
) -> Consumed {
    let ns = ns(tag);
    let ix = cerulion_core::testing::iceoryx_test_config();
    let sent = Arc::new(AtomicU64::new(0));
    let ready = Arc::new(AtomicBool::new(false));
    let stop = Arc::new(AtomicBool::new(false));
    let fires = Arc::new(AtomicU64::new(0));

    let drive = Arc::new(VirtualClock::new());
    let producer_mgr = transport(&format!("ppk_{tag}_p"), ix.clone(), drive.clone());
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "stamper".to_string(),
        Box::new(StamperEntry::with_state(Stamper {
            sent: Arc::clone(&sent),
            ..Default::default()
        })),
    );
    let mut producer = GraphRuntime::build_live_with_schema_hashes_and_policy(
        config(
            &format!("ppk_{tag}_producer"),
            "ppk",
            vec![node("stamper", "stamper", vec![], vec![out(Some(publish))])],
        ),
        factories,
        &producer_mgr,
        drive.clone(),
        None,
        MonitorWaitPolicy::new(true, true, ns.clone()),
    )
    .expect("build the producer runtime");

    let consumer = {
        let (ns, ix, ready, stop, fires) = (
            ns.clone(),
            ix.clone(),
            Arc::clone(&ready),
            Arc::clone(&stop),
            Arc::clone(&fires),
        );
        let feeds: Vec<&'static str> = feeds.to_vec();
        let tag = tag.to_string();
        std::thread::spawn(move || {
            let seen = Arc::new(Mutex::new(Vec::new()));
            let mut nodes = Vec::new();
            let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
            for (i, feed) in feeds.iter().enumerate() {
                let id = format!("reader{i}");
                nodes.push(node(&id, "reader", vec![input(feed)], vec![]));
                factories.insert(
                    id,
                    Box::new(ReaderEntry::with_state(Reader {
                        seen: Arc::clone(&seen),
                        fires: Arc::clone(&fires),
                        ..Default::default()
                    })),
                );
            }
            let mgr = transport(&format!("ppk_{tag}_c"), ix, Arc::new(RealClock));
            let mut rt = GraphRuntime::build_live_with_schema_hashes_and_policy(
                config(&format!("ppk_{tag}_consumer"), "ppk", nodes),
                factories,
                &mgr,
                Arc::new(RealClock),
                None,
                MonitorWaitPolicy::new(true, true, ns),
            )
            .expect("build the consumer runtime");
            rt.set_period_park_for_test(Some(true));
            ready.store(true, Ordering::Release);
            let mut timeouts_while_due = 0;
            while fires.load(Ordering::Relaxed) < frames && !stop.load(Ordering::Acquire) {
                let (_, _, _, before) = rt.park_wake_counts();
                rt.run_live_step_once_for_test(DUE_PARK_TIMEOUT);
                let (_, _, _, after) = rt.park_wake_counts();
                timeouts_while_due += after - before;
            }
            let stats = rt.period_park_stats_for_test();
            rt.shutdown();
            drop(mgr);
            let seen = seen.lock().expect("seen log").clone();
            Consumed {
                seen,
                stats,
                timeouts_while_due,
            }
        })
    };
    let spun_up = Instant::now();
    while !ready.load(Ordering::Acquire) {
        assert!(
            spun_up.elapsed() < Duration::from_secs(20),
            "the consumer runtime never finished building"
        );
        std::thread::sleep(Duration::from_millis(1));
    }
    std::thread::sleep(Duration::from_millis(20));
    while sent.load(Ordering::Relaxed) < frames {
        std::thread::sleep(gap);
        drive.advance(5_000_000);
        producer.run_live_step_once_for_test(Duration::from_millis(1));
    }
    let deadline = Instant::now() + Duration::from_secs(10);
    while fires.load(Ordering::Relaxed) < frames && Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(2));
    }
    stop.store(true, Ordering::Release);
    let consumed = consumer.join().expect("consumer thread panicked");
    producer.shutdown();
    drop(producer_mgr);
    consumed
}

/// One runtime with a 1 kHz node and a consumer of it, driven by the
/// production live step for `fires` ticks with the period-park forced as asked.
/// Returns the tick times, the consumer's numbers and the period-park counters.
pub fn ticker_run(
    tag: &str,
    period_park: bool,
    fires: usize,
) -> (Vec<u64>, Vec<f64>, PeriodParkStats) {
    let mgr = transport(
        &format!("ppk_{tag}"),
        cerulion_core::testing::iceoryx_test_config(),
        Arc::new(RealClock),
    );
    let ticks = Arc::new(Mutex::new(Vec::with_capacity(fires + 64)));
    let seen = Arc::new(Mutex::new(Vec::new()));
    let read = Arc::new(AtomicU64::new(0));
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "ticker".to_string(),
        Box::new(TickerEntry::with_state(Ticker {
            ticks: Arc::clone(&ticks),
            ..Default::default()
        })),
    );
    factories.insert(
        "reader".to_string(),
        Box::new(ReaderEntry::with_state(Reader {
            seen: Arc::clone(&seen),
            fires: Arc::clone(&read),
            ..Default::default()
        })),
    );
    let mut rt = GraphRuntime::build_live_with_schema_hashes_and_policy(
        config(
            &format!("ppk_{tag}"),
            &format!("ppk{tag}"),
            vec![
                node("ticker", "ticker", vec![], vec![out(None)]),
                node("reader", "reader", vec![input("ticker/out")], vec![]),
            ],
        ),
        factories,
        &mgr,
        Arc::new(RealClock),
        None,
        MonitorWaitPolicy::new(true, true, ns(tag)),
    )
    .expect("build the ticker graph");
    rt.set_period_park_for_test(Some(period_park));
    let started = Instant::now();
    while ticks.lock().expect("tick log").len() < fires
        && started.elapsed() < Duration::from_secs(8)
    {
        let timeout = rt.live_timeout_for_test();
        rt.run_live_step_once_for_test(timeout);
    }
    let timeout = rt.live_timeout_for_test();
    rt.run_live_step_once_for_test(timeout);
    let stats = rt.period_park_stats_for_test();
    rt.shutdown();
    drop(mgr);
    let ticks = ticks.lock().expect("tick log").clone();
    let seen = seen
        .lock()
        .expect("seen log")
        .iter()
        .map(|(n, _)| *n)
        .collect();
    (ticks, seen, stats)
}
