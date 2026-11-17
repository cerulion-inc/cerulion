// SPDX-License-Identifier: AGPL-3.0-only
//! The live loop's period-park (`CERULION_PERIOD_PARK=1`) over real iceoryx2.
//!
//! The switch is forced per runtime through `set_period_park_for_test`, so the
//! process environment is never touched. Each arm is judged against the host
//! fact the period-park is gated on: it applies only where the CPU carries a
//! monitor-wait primitive (and, for an input written from another runtime, the
//! doorbell has a wake word). On any other host the arm asserts the switch did
//! nothing, so the binary runs a real test everywhere.
//!
//! Per-test SHM roots; `#[serial]` because several arms judge wall-clock
//! timing.

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
use serial_test::serial;

/// The longest the period-park sleeps in the kernel at once (`period_park::NAP`).
const NAP: Duration = Duration::from_micros(100);
/// A park timeout far longer than any arm's run: a wait that ends on it was
/// ended by no frame.
const DUE_PARK_TIMEOUT: Duration = Duration::from_secs(2);

fn ns(tag: &str) -> String {
    format!("ppk{}{tag}", std::process::id())
}

/// Does the period-park apply to a consumer of a topic written from another
/// runtime on this host? Read from the platform, not from the code under test.
fn applies_to_a_foreign_consumer() -> bool {
    cerulion_core::monitor_wait::monitor_wait_available()
        && cerulion_core::doorbell::wake_word_block_primitive_available()
}

fn transport(
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

fn node(id: &str, node_type: &str, inputs: Vec<InputDef>, outputs: Vec<OutputDef>) -> NodeDef {
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

fn input(source: &str) -> InputDef {
    InputDef {
        name: "inp".to_string(),
        source: source.to_string(),
        ..Default::default()
    }
}

fn config(identity: &str, prefix: &str, nodes: Vec<NodeDef>) -> GraphConfig {
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
struct Consumed {
    seen: Vec<(f64, u64)>,
    stats: PeriodParkStats,
    timeouts_while_due: u64,
}

/// A producer runtime on this thread and a consumer runtime on its own thread,
/// joined by `publish` (one of the consumer's `feeds`, each read by its own
/// node, declared in order, so the first is the armed line). The producer sends
/// `frames` frames `gap` apart while the consumer waits with the period-park on.
fn two_runtimes(
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

fn median(mut v: Vec<u64>) -> u64 {
    v.sort_unstable();
    v[v.len() / 2]
}

/// A ring from another runtime ends the consumer's kernel nap at once: every
/// frame is read in order, no wait ends on its long park timeout, and a ring
/// demonstrably cut a nap short.
#[test]
#[serial]
fn a_ring_from_another_runtime_ends_a_kernel_nap() {
    const FRAMES: u64 = 16;
    let c = two_runtimes(
        "ring",
        &["/ppk/ring/feed"],
        "/ppk/ring/feed",
        FRAMES,
        Duration::from_millis(3),
    );
    let numbers: Vec<f64> = c.seen.iter().map(|(n, _)| *n).collect();
    let expected: Vec<f64> = (1..=numbers.len() as u64).map(|n| n as f64).collect();
    assert!(
        numbers.len() as u64 >= FRAMES,
        "read {} of {FRAMES} frames",
        numbers.len()
    );
    assert_eq!(numbers, expected, "every frame, in order");
    assert_eq!(
        c.timeouts_while_due, 0,
        "every wait while a frame was due ended on a data wake, not on its \
         {DUE_PARK_TIMEOUT:?} timeout"
    );
    let delays: Vec<u64> = c.seen.iter().map(|(_, d)| *d).collect();
    eprintln!(
        "ring: {:?}, median delay {} ns, max {} ns",
        c.stats,
        median(delays.clone()),
        delays.iter().max().copied().unwrap_or(0)
    );
    if applies_to_a_foreign_consumer() {
        assert!(
            c.stats.blocks > 0 && c.stats.kernel_sleeps > 0,
            "{:?}",
            c.stats
        );
        assert!(
            c.stats.kernel_sleeps_cut_by_data > 0,
            "a producer ring must end a kernel nap before its end: {:?}",
            c.stats
        );
        let worst = Duration::from_nanos(delays.iter().max().copied().unwrap_or(0));
        assert!(
            worst <= Duration::from_millis(2),
            "the slowest frame was read {worst:?} after it was written"
        );
    } else {
        assert_eq!(
            c.stats,
            PeriodParkStats::default(),
            "the period-park does not apply here"
        );
    }
}

/// A ring on an input topic other than the armed one does not end a kernel
/// nap (one process blocks on one wake word); the loop top reads every input
/// after each nap, so such a frame is read within about one nap.
#[test]
#[serial]
fn a_ring_on_a_second_input_is_read_within_about_one_nap() {
    const FRAMES: u64 = 16;
    let c = two_runtimes(
        "second",
        &["/ppk/second/a", "/ppk/second/b"],
        "/ppk/second/b",
        FRAMES,
        Duration::from_millis(3),
    );
    let numbers: Vec<f64> = c.seen.iter().map(|(n, _)| *n).collect();
    let expected: Vec<f64> = (1..=numbers.len() as u64).map(|n| n as f64).collect();
    assert!(
        numbers.len() as u64 >= FRAMES,
        "read {} of {FRAMES} frames",
        numbers.len()
    );
    assert_eq!(
        numbers, expected,
        "every frame on the second input, in order"
    );
    let delays: Vec<u64> = c.seen.iter().map(|(_, d)| *d).collect();
    let med = Duration::from_nanos(median(delays.clone()));
    let worst = Duration::from_nanos(delays.iter().max().copied().unwrap_or(0));
    eprintln!(
        "second input: {:?}, median delay {med:?}, max {worst:?}",
        c.stats
    );
    if applies_to_a_foreign_consumer() {
        assert_eq!(
            c.stats.kernel_sleeps_cut_by_data, 0,
            "a ring on a line the nap is not blocked on cannot end the nap: {:?}",
            c.stats
        );
        assert!(c.stats.kernel_sleeps > 0, "{:?}", c.stats);
        // One nap plus the timer's slack and the step: a generous bound.
        assert!(
            med <= NAP * 5,
            "the median frame on the second input was read {med:?} after it was written"
        );
        assert!(
            worst <= Duration::from_millis(5),
            "the slowest was read {worst:?} after"
        );
    } else {
        assert_eq!(
            c.stats,
            PeriodParkStats::default(),
            "the period-park does not apply here"
        );
    }
}

/// One runtime with a 1 kHz node and a consumer of it, driven by the
/// production live step for `fires` ticks with the period-park forced as asked.
/// Returns the tick times, the consumer's numbers and the period-park counters.
fn ticker_run(tag: &str, period_park: bool, fires: usize) -> (Vec<u64>, Vec<f64>, PeriodParkStats) {
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

/// Under the period-park a 1 kHz node still fires on its grid: after its
/// start-up catch-up, almost no fire lands less than half a period after the
/// previous one and the median gap is the period. Judged where the
/// period-park applies; on any other host the live loop's own grid is
/// `live_exact_deadline_iox2_test`'s, and this arm asserts the switch did
/// nothing.
#[test]
#[serial]
fn a_period_node_under_the_period_park_stays_on_its_grid() {
    const WARMUP: usize = 100;
    const FIRES: usize = 800;
    let (ticks, _, stats) = ticker_run("grid", true, WARMUP + FIRES);
    assert!(ticks.len() >= WARMUP + FIRES, "ran {} fires", ticks.len());
    let gaps: Vec<u64> = ticks[WARMUP..]
        .windows(2)
        .map(|w| w[1].saturating_sub(w[0]))
        .collect();
    let doubles = gaps.iter().filter(|g| **g < 500_000).count();
    let med = Duration::from_nanos(median(gaps.clone()));
    eprintln!(
        "grid: {stats:?}, {doubles} doubles of {}, median gap {med:?}",
        gaps.len()
    );
    if cerulion_core::monitor_wait::monitor_wait_available() {
        assert!(
            doubles * 50 <= gaps.len(),
            "{doubles} of {} fires came less than half a period after the previous one",
            gaps.len()
        );
        assert!(
            med >= Duration::from_micros(850) && med <= Duration::from_micros(1_150),
            "the median gap is {med:?}"
        );
        assert!(
            stats.blocks > 0 && stats.kernel_sleeps > 0 && stats.timer_wakes > 0,
            "{stats:?}"
        );
    } else {
        assert_eq!(
            stats,
            PeriodParkStats::default(),
            "the period-park does not apply here"
        );
    }
}

/// With the switch off the period-park never runs: its counters stay at zero.
#[test]
#[serial]
fn with_the_switch_off_the_period_park_never_runs() {
    let (ticks, _, stats) = ticker_run("off", false, 60);
    assert!(ticks.len() >= 60, "apparatus: ran {} fires", ticks.len());
    assert_eq!(stats, PeriodParkStats::default());
}

/// The period-park changes when the loop re-polls, never what fires: past the
/// start-up catch-up (whose burst can overrun the consumer's queue in either
/// mode), the consumer reads every numbered frame, in order, with the switch on
/// as with it off.
#[test]
#[serial]
fn the_period_park_changes_when_not_what() {
    const FIRES: usize = 300;
    const WARMUP: f64 = 100.0;
    for on in [true, false] {
        let (ticks, seen, _) = ticker_run(if on { "whaton" } else { "whatoff" }, on, FIRES);
        assert!(ticks.len() >= FIRES, "apparatus: ran {} fires", ticks.len());
        let after: Vec<f64> = seen.iter().copied().filter(|n| *n > WARMUP).collect();
        assert!(
            after.len() >= FIRES - 120,
            "switch on={on}: read {} frames past the warm-up",
            after.len()
        );
        let first = after[0];
        let expected: Vec<f64> = (0..after.len()).map(|k| first + k as f64).collect();
        assert_eq!(
            after, expected,
            "switch on={on}: every frame past the warm-up, in order"
        );
    }
}
