// SPDX-License-Identifier: AGPL-3.0-only
//! `#[input(trigger, consume = latest)]`: a data-trigger input that fires the
//! node at most once per scheduler step, on the NEWEST queued frame.
//!
//! The `fifo` default fires once per queued frame, in arrival order
//! (`fifo_consume_iox2_test`). A `latest` input drains the queue at the
//! step's read and serves the newest frame; the older frames in that drain
//! are not served. A frame no fire has read yet is kept until a newer one
//! arrives, so a deferred fire still reads it.
//!
//! Every oracle here is hand-written from those rules (never a self-compare),
//! and each headline arm fails when the latest wiring is removed (the input
//! then runs as `fifo` and observes the whole burst). The consumer reads via
//! `try_view`, the read the `#[cerulion_node]` macro's generated tick uses, on
//! both drain disciplines (Unified, and forced-Separate via the
//! `CERULION_DRAIN_DISCIPLINE` seam), and once through a dlopen'd macro
//! cdylib, where the declaration crosses the FFI as the info JSON's
//! `"consume":"latest"` key.
//!
//! The composition matrix with `backpressure`:
//!
//! | backpressure | `consume = latest` |
//! |---|---|
//! | `drop_oldest` | evictions at the queue are counted; the read serves the newest survivor |
//! | `block` | each fire's read empties the queue, so the producer is paced only when `depth` frames queue between two reads, as while a deferred fire spans `depth` publishes; superseded frames are not served |
//! | `sample(N)` | the gate admits the drain's newest frame at most once per N ms; a decimated one is counted |
//!
//! On the separate drain discipline the step counts arrivals off a second
//! (trigger) subscriber and the tick's read drains the body subscriber, so
//! the gap arms put a frame between the two and hold every fire to reading a
//! frame (fires equal reads).
//!
//! `#[serial]`: the forced-Separate arms mutate the process-global
//! `CERULION_DRAIN_DISCIPLINE` env.
//!
//! # Running
//!
//! ```bash
//! cargo build -p test_node_macro_consume_latest_cdylib -p test_node_sync_op_fail_cdylib
//! cargo nextest run -p cerulion_core --test consume_latest_iox2_test
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{
    BackpressurePolicy, ClosureNodeEntry, InputConsume, InputMeta, NodeContext, NodeEntry,
    NodeInfo, TriggerProbe,
};
use cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::{MacroPolicy, SyncHeadOp, SyncOpAnswer};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

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

const EXT: &str = "/latest/ext";

/// The consumer's declaration.
#[derive(Clone, Copy)]
struct Consumer {
    consume: InputConsume,
    depth: usize,
    backpressure: BackpressurePolicy,
    throttle_ms: Option<u64>,
}

impl Consumer {
    fn latest() -> Self {
        Self {
            consume: InputConsume::Latest,
            depth: DEFAULT_CONSUMER_DEPTH,
            backpressure: BackpressurePolicy::DropOldest,
            throttle_ms: None,
        }
    }
}

/// A data-trigger closure consumer recording every `try_view`-observed value.
fn consumer_entry(c: Consumer, seen: Arc<Mutex<Vec<u64>>>) -> ClosureNodeEntry {
    let mut info = NodeInfo::with_meta(
        vec![InputMeta {
            name: "inp".to_string(),
            schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
            trigger: true,
            depth: c.depth,
            backpressure: c.backpressure,
            expect_within_ms: None,
            consume: c.consume,
            accumulate: false,
        }],
        vec![],
    )
    .with_policy(MacroPolicy::DataTrigger {
        input_name: "inp".to_string(),
    });
    if let Some(ms) = c.throttle_ms {
        info = info.with_throttle_ms(ms);
    }
    ClosureNodeEntry::new(info, move |ctx| {
        if let Some(s) = ctx.subscriber_mut("inp") {
            if let Ok(Some(v)) = s.try_view::<Vector3, _>(|view| view.x as u64) {
                seen.lock().unwrap().push(v);
            }
        }
        Ok(())
    })
    .with_label("latest_consumer")
}

fn sink_node(source: &str) -> NodeDef {
    NodeDef {
        isolate: None,
        params: Default::default(),
        ros2: None,
        id: "sink".to_string(),
        node_type: "latest_consumer".to_string(),
        inputs: vec![InputDef {
            name: "inp".to_string(),
            source: source.to_string(),
            ..Default::default()
        }],
        outputs: vec![],
    }
}

fn graph(prefix: &str, nodes: Vec<NodeDef>) -> GraphConfig {
    GraphConfig {
        name: None,
        identity: "consume_latest".to_string(),
        prefix: prefix.to_string(),
        nodes,
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    }
}

/// A runtime with ONE consumer on the external source [`EXT`], plus that
/// source's publisher.
fn external_runtime(
    prefix: &str,
    c: Consumer,
    seen: Arc<Mutex<Vec<u64>>>,
) -> (
    GraphRuntime,
    cerulion_core::transport::publisher::CerulionPublisher,
) {
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("sink".to_string(), Box::new(consumer_entry(c, seen)));
    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(graph(prefix, vec![sink_node(EXT)]), factories, clock, 16)
            .expect("build latest graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let ext = mgr
        .create_publisher(EXT, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");
    (runtime, ext)
}

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

fn fires(runtime: &GraphRuntime) -> u64 {
    runtime.node_handle("sink").unwrap().fire_count()
}

/// Bursts per step, then `idle` empty steps; returns (observed, fire count).
fn run_bursts(prefix: &str, c: Consumer, bursts: &[&[u64]], idle: u32) -> (Vec<u64>, u64) {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let (mut runtime, mut ext) = external_runtime(prefix, c, Arc::clone(&seen));
    for burst in bursts {
        publish_all(&mut ext, burst.iter().copied());
        runtime.step(Duration::from_millis(1));
    }
    for _ in 0..idle {
        runtime.step(Duration::from_millis(1));
    }
    let observed = seen.lock().unwrap().clone();
    let n = fires(&runtime);
    (observed, n)
}

/// HEADLINE: a burst of 5 queued before one step is ONE fire reading the
/// newest frame. Idle steps after it fire nothing. As `fifo` (the latest
/// wiring removed) the same run observes `[1, 2, 3, 4, 5]` in 5 fires.
#[test]
#[serial]
fn a_burst_is_one_fire_on_the_newest_frame() {
    let (observed, n) = run_bursts("latburst", Consumer::latest(), &[&[1, 2, 3, 4, 5]], 6);
    assert_eq!(
        observed,
        vec![5],
        "one fire, on the newest frame of the burst"
    );
    assert_eq!(n, 1, "exactly one fire: the idle steps owe nothing");
}

/// A latest node carries no between-fires refill hook: a refill would fire
/// it a second time in one step on a frame that arrived during its tick. The
/// unified `fifo` control carries one.
#[test]
#[serial]
fn a_latest_node_gets_no_refill_hook_and_the_fifo_control_does() {
    for (consume, want) in [(InputConsume::Latest, false), (InputConsume::Fifo, true)] {
        let c = Consumer {
            consume,
            ..Consumer::latest()
        };
        let (runtime, _ext) = external_runtime("latrefill", c, Arc::new(Mutex::new(Vec::new())));
        assert_eq!(
            runtime.has_trigger_refill_for_test("sink"),
            Some(want),
            "{consume:?}"
        );
    }
}

/// The `fifo` control in the same harness: the declaration is the only
/// difference, and it is what decides between the two oracles.
#[test]
#[serial]
fn the_fifo_control_serves_the_same_burst_per_message() {
    let c = Consumer {
        consume: InputConsume::Fifo,
        ..Consumer::latest()
    };
    let (observed, n) = run_bursts("latfifo", c, &[&[1, 2, 3, 4, 5]], 6);
    assert_eq!(observed, vec![1, 2, 3, 4, 5]);
    assert_eq!(n, 5);
}

/// One fire per step that saw data, each on that step's newest frame; a step
/// with nothing queued does not fire.
#[test]
#[serial]
fn each_step_fires_once_on_its_own_newest_frame() {
    let (observed, n) = run_bursts(
        "latsteps",
        Consumer::latest(),
        &[&[1, 2, 3], &[], &[4, 5], &[6]],
        3,
    );
    assert_eq!(observed, vec![3, 5, 6]);
    assert_eq!(n, 3, "the empty step and the idle tail fire nothing");
}

/// The forced-Separate discipline gives the SAME observation. There the
/// trigger subscriber signals one arrival per queued frame, so this arm is
/// what pins the scheduler's collapse of those arrivals to one fire.
#[test]
#[serial]
fn forced_separate_discipline_gives_the_same_observation() {
    let _guard = EnvVarGuard::set("CERULION_DRAIN_DISCIPLINE", "separate");
    let (observed, n) = run_bursts(
        "latsep",
        Consumer::latest(),
        &[&[1, 2, 3, 4, 5], &[], &[6, 7]],
        4,
    );
    assert_eq!(observed, vec![5, 7]);
    assert_eq!(
        n, 2,
        "the per-arrival signals collapse to one fire per step"
    );
}

/// Two runs of one schedule observe the same sequence, equal to the oracle.
#[test]
#[serial]
fn latest_delivery_is_deterministic_across_runs() {
    let bursts: &[&[u64]] = &[&[1, 2], &[3], &[], &[4, 5, 6]];
    let a = run_bursts("latdeta", Consumer::latest(), bursts, 2);
    let b = run_bursts("latdetb", Consumer::latest(), bursts, 2);
    assert_eq!(a, (vec![2, 3, 6], 3));
    assert_eq!(a, b);
}

/// The live loop's step serves a queued burst the same way: one fire, newest
/// frame.
#[test]
#[serial]
fn a_live_step_serves_the_newest_frame_once() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let (mut runtime, mut ext) = external_runtime("latlive", Consumer::latest(), Arc::clone(&seen));
    // Warm-up: one frame + one live step establishes the connection.
    publish_all(&mut ext, [100]);
    runtime.run_live_step_once_for_test(Duration::from_millis(200));
    assert_eq!(*seen.lock().unwrap(), vec![100]);

    publish_all(&mut ext, [1, 2, 3, 4, 5]);
    runtime.run_live_step_once_for_test(Duration::from_millis(200));
    assert_eq!(*seen.lock().unwrap(), vec![100, 5]);
    assert_eq!(fires(&runtime), 2);
}

/// A DEFERRED fire reads the frame it was deferred on when nothing newer
/// arrives. With `throttle_ms = 5`, frame 2 arrives inside the window: the
/// step drains it and defers the fire, and the steps after it find the queue
/// empty. The read keeps frame 2 instead of dropping it, so the fire at the
/// end of the window serves it.
#[test]
#[serial]
fn a_deferred_fire_still_reads_the_frame_it_was_deferred_on() {
    let c = Consumer {
        throttle_ms: Some(5),
        ..Consumer::latest()
    };
    let (observed, n) = run_bursts("latdefer", c, &[&[1], &[2]], 8);
    assert_eq!(observed, vec![1, 2]);
    assert_eq!(n, 2);
}

/// ...and a NEWER frame arriving during the defer supersedes the held one:
/// the fire at the end of the window reads frame 4, not 2.
#[test]
#[serial]
fn a_newer_frame_during_a_defer_supersedes_the_held_one() {
    let c = Consumer {
        throttle_ms: Some(5),
        ..Consumer::latest()
    };
    let (observed, n) = run_bursts("latsuper", c, &[&[1], &[2], &[3], &[4]], 8);
    assert_eq!(observed, vec![1, 4]);
    assert_eq!(n, 2);
}

/// Two newer frames queued behind a held frame. The boundary drain keeps
/// the held frame borrowed while it receives both, three samples borrowed on
/// one connection, one above the iceoryx2 default, so the latest trigger's
/// source topic is created with that headroom. Without it the drain fails
/// with `ExceedsMaxBorrows` on every other deferred boundary (a failed drain
/// leaves no held frame for the next one), and a fire landing on a failed
/// boundary reads an error instead of a frame. The two schedules differ by
/// one step, so whichever step the throttle releases on, one of them puts a
/// failing boundary there.
#[test]
#[serial]
fn two_newer_frames_behind_a_held_one_drain_without_a_failed_read() {
    let c = Consumer {
        throttle_ms: Some(5),
        ..Consumer::latest()
    };
    for (prefix, lead) in [("lattwoa", 0u64), ("lattwob", 1)] {
        let mut bursts: Vec<Vec<u64>> = vec![vec![1], vec![2]];
        bursts.extend((0..lead).map(|_| Vec::new()));
        bursts.extend((0..8u64).map(|k| vec![3 + 2 * k, 4 + 2 * k]));
        let refs: Vec<&[u64]> = bursts.iter().map(Vec::as_slice).collect();
        let (observed, n) = run_bursts(prefix, c, &refs, 12);
        assert!(
            n >= 3,
            "{prefix}: the throttle releases more than once: {n} fires"
        );
        assert_eq!(
            observed.len() as u64,
            n,
            "{prefix}: every fire reads a frame, none an error: {observed:?} over {n} fires"
        );
        assert_eq!(observed.first(), Some(&1), "{prefix}: {observed:?}");
        assert_eq!(
            observed.last(),
            Some(&18),
            "{prefix}: the newest frame is read last: {observed:?}"
        );
        assert!(
            observed.windows(2).all(|w| w[0] < w[1]),
            "{prefix}: reads only move forward: {observed:?}"
        );
    }
}

/// `drop_oldest` composition: a burst of 8 into a depth-4 queue evicts the
/// oldest 4 at the queue, counted, and the read serves the newest survivor.
#[test]
#[serial]
fn drop_oldest_counts_the_evictions_and_serves_the_newest() {
    let c = Consumer {
        depth: 4,
        ..Consumer::latest()
    };
    let seen = Arc::new(Mutex::new(Vec::new()));
    let (mut runtime, mut ext) = external_runtime("latevict", c, Arc::clone(&seen));
    // The stream's first frame establishes the eviction baseline (uncounted).
    publish_all(&mut ext, [100]);
    runtime.step(Duration::from_millis(1));
    publish_all(&mut ext, [1, 2, 3, 4, 5, 6, 7, 8]);
    for _ in 0..4 {
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(*seen.lock().unwrap(), vec![100, 8]);
    assert_eq!(
        runtime
            .node_handle("sink")
            .unwrap()
            .backpressure_drop_oldest_count("inp"),
        4,
        "the 4 frames evicted at the queue are counted; the 3 the read \
         superseded are not evictions"
    );
}

/// `sample(N)` composition: frames 1..3 share a stamp, so the drain's newest
/// (3) is the one the gate sees, and it is the first, so it is admitted. Frame
/// 4, 6 ms later, is admitted; frame 5, 1 ms after 4, is decimated, counted.
#[test]
#[serial]
fn sample_gates_the_newest_frame_of_each_drain() {
    let c = Consumer {
        backpressure: BackpressurePolicy::Sample(5),
        ..Consumer::latest()
    };
    let seen = Arc::new(Mutex::new(Vec::new()));
    let (mut runtime, mut ext) = external_runtime("latsamp", c, Arc::clone(&seen));
    publish_all(&mut ext, [1, 2, 3]);
    for _ in 0..6 {
        runtime.step(Duration::from_millis(1));
    }
    publish_all(&mut ext, [4]);
    runtime.step(Duration::from_millis(1));
    publish_all(&mut ext, [5]);
    for _ in 0..2 {
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(*seen.lock().unwrap(), vec![3, 4]);
    assert_eq!(
        runtime
            .node_handle("sink")
            .unwrap()
            .backpressure_sampled_count("inp"),
        1,
        "frame 5 is decimated and counted"
    );
}

/// A `period_ms = 1` producer publishing its fire count into a depth-4
/// `block` edge, read by a `consume = latest` consumer throttled to one fire
/// per `throttle_ms`. Returns (observed, the sink's block defers, frames
/// published) after 30 steps.
fn run_block(prefix: &str, throttle_ms: u64) -> (Vec<u64>, u64, u64) {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let counter = Arc::new(AtomicU64::new(0));
    let counter_c = Arc::clone(&counter);
    let producer_info = NodeInfo::from_names(vec![], vec!["out".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 1 });
    let producer = ClosureNodeEntry::new(producer_info, move |ctx| {
        let v = counter_c.fetch_add(1, Ordering::Relaxed) + 1;
        if let Some(p) = ctx.publisher_mut("out") {
            let mut proxy = p.loan_proxy::<Vector3>()?;
            proxy.x = v as f64;
        }
        Ok(())
    })
    .with_label("latest_block_producer");
    let c = Consumer {
        depth: 4,
        backpressure: BackpressurePolicy::Block,
        throttle_ms: Some(throttle_ms),
        ..Consumer::latest()
    };
    let producer_node = NodeDef {
        isolate: None,
        params: Default::default(),
        ros2: None,
        id: "producer".to_string(),
        node_type: "latest_block_producer".to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            topic: None,
            history_size: 0,
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(producer));
    factories.insert(
        "sink".to_string(),
        Box::new(consumer_entry(c, Arc::clone(&seen))),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(
        graph(prefix, vec![producer_node, sink_node("producer/out")]),
        factories,
        clock,
        16,
    )
    .expect("build block graph");
    for _ in 0..30 {
        runtime.step(Duration::from_millis(1));
    }
    let observed = seen.lock().unwrap().clone();
    let deferred = runtime
        .node_handle("sink")
        .unwrap()
        .backpressure_block_fires_deferred_count("inp");
    let published = counter.load(Ordering::Relaxed);
    (observed, deferred, published)
}

/// `block` composition, a consumer that keeps up with the queue: throttled
/// to one fire per 3 ms against a depth of 4, each fire drains the 3 frames
/// queued since the last one, so the queue never fills and the producer is
/// never paced. Each fire reads the newest frame: 1, then every third.
#[test]
#[serial]
fn block_with_latest_serves_the_newest_and_paces_nothing_while_the_queue_fits() {
    let (observed, deferred, published) = run_block("latblock", 3);
    let expected: Vec<u64> = (0..observed.len() as u64).map(|k| 1 + 3 * k).collect();
    assert!(observed.len() >= 9, "a real stream: {observed:?}");
    assert_eq!(observed, expected);
    assert_eq!(
        deferred, 0,
        "the queue never fills, so the block gate never defers"
    );
    assert_eq!(published, 30, "one publish per step");
}

/// `block` composition, a deferred fire spanning `depth` publishes: throttled
/// to one fire per 6 ms against a depth of 4, the queue fills four steps
/// after a fire and the block gate holds the producer until the next fire
/// drains it. Each fire reads the newest frame, which is the fourth published
/// since the last fire: 1, 5, 9, ...
#[test]
#[serial]
fn block_with_latest_paces_the_producer_while_a_deferred_fire_spans_the_depth() {
    let (observed, deferred, published) = run_block("latblockp", 6);
    let expected: Vec<u64> = (0..observed.len() as u64).map(|k| 1 + 4 * k).collect();
    assert!(observed.len() >= 4, "a real stream: {observed:?}");
    assert_eq!(observed, expected);
    assert!(deferred > 0, "the full queue defers the producer");
    assert!(
        published < 30,
        "the producer was paced: {published} publishes"
    );
}

/// Through a dlopen'd `#[cerulion_node]` cdylib: `consume = latest` reaches
/// the host only as the info JSON's `"consume":"latest"` key. The node mirrors
/// its trigger input to `cmd`, so the `cmd` stream is the observation record:
/// one frame, the newest. Without the key the fixture forwards all five.
#[test]
#[serial]
fn a_cdylib_declaring_latest_forwards_only_the_newest_frame() {
    use cerulion_core::graph::node::DylibNodeEntry;

    const IN_TOPIC: &str = "/latdyl/in";
    let entry = DylibNodeEntry::load(&cerulion_core::testing::find_fixture_cdylib(
        "test_node_macro_consume_latest_cdylib",
    ))
    .expect("load consume-latest fixture cdylib");
    let meta = entry.info().expect("fixture info").input_meta().to_vec();
    assert_eq!(meta.len(), 1);
    assert_eq!(
        meta[0].consume,
        InputConsume::Latest,
        "the key crossed the FFI"
    );

    let node = NodeDef {
        isolate: None,
        params: Default::default(),
        ros2: None,
        id: "node".to_string(),
        node_type: "consume_latest_node".to_string(),
        inputs: vec![InputDef {
            name: "trigger_in".to_string(),
            source: IN_TOPIC.to_string(),
            ..Default::default()
        }],
        outputs: vec![OutputDef {
            name: "cmd".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("node".to_string(), Box::new(entry));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(graph("latdyl", vec![node]), factories, clock, 16)
            .expect("build cdylib graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher(IN_TOPIC, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");
    let cmd_sub = mgr
        .create_subscriber_open_only("/latdyl/node/cmd")
        .expect("open the cdylib's cmd topic");

    publish_all(&mut ext, [1, 2, 3, 4, 5]);
    for _ in 0..4 {
        runtime.step(Duration::from_millis(1));
    }
    publish_all(&mut ext, [6, 7]);
    for _ in 0..4 {
        runtime.step(Duration::from_millis(1));
    }

    let mut forwarded: Vec<u64> = Vec::new();
    cmd_sub
        .try_receive(|msg| {
            let x = f64::from_le_bytes(msg.payload()[0..8].try_into().unwrap());
            forwarded.push(x as u64);
        })
        .expect("drain cmd stream");
    assert_eq!(forwarded, vec![5, 7]);
}

/// The subscriber's own state machine, driven by hand on a drain-to-latest
/// subscriber (the mode a `consume = latest` trigger input is wired in):
///
/// * a boundary drain pops the whole queue and freezes the newest frame;
/// * a REFILL with that frame still unread pops nothing, even with newer
///   frames queued behind it, because it asks whether the fire just run took
///   the head;
/// * a boundary with the frame unread and nothing newer re-offers it;
/// * a boundary with the frame unread and a newer frame queued replaces it;
/// * the tick's read serves whatever is frozen.
#[test]
#[serial]
fn the_latest_drain_holds_an_unread_frame_and_never_refills_past_it() {
    use cerulion_core::transport::{TransportConfig, TransportManager};

    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "latest_slot".into(),
            clock: Arc::new(cerulion_core::clock::RealClock),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("per-test SHM transport");
    let topic = "latest_slot";
    let mut publisher = mgr
        .create_publisher(topic, MaxSliceLen::try_new(1024).expect("len"), 0)
        .expect("publisher");
    let mut sub = mgr.create_subscriber(topic).expect("subscriber");
    let mut send = |x: u64| {
        let mut p = publisher.loan_proxy::<Vector3>().expect("loan");
        p.x = x as f64;
    };
    let read = |sub: &mut cerulion_core::transport::subscriber::CerulionSubscriber| {
        sub.try_view::<Vector3, _>(|v| v.x as u64).expect("read")
    };

    send(1);
    send(2);
    send(3);
    let (popped, ts) = sub.snapshot_latest_for_trigger();
    assert_eq!(popped, 3, "the boundary drain pops the whole queue");
    assert!(ts.is_some());

    send(4);
    assert_eq!(
        sub.refill_for_trigger(),
        (0, None),
        "a refill never pops past an unread frame"
    );

    let (popped, _) = sub.snapshot_latest_for_trigger();
    assert_eq!(popped, 1, "frame 4 replaces the unread frame 3");
    let (popped, ts) = sub.snapshot_latest_for_trigger();
    assert_eq!(popped, 1, "nothing newer: the unread frame 4 is re-offered");
    assert!(ts.is_some());
    assert_eq!(read(&mut sub), Some(4), "the tick reads the newest frame");

    assert_eq!(
        sub.snapshot_latest_for_trigger(),
        (0, None),
        "a read frame is not re-offered"
    );
}

// The separate drain discipline (`block` and `sample(N)` inputs, forced
// Separate, and entries without the unified drain) reads the input through
// TWO subscribers on one topic: the trigger subscriber, popped at the level
// boundary to count arrivals, and the body subscriber, drained to its newest
// frame by the tick's read. A frame that lands between those two pops is read
// by the tick and still queued on the trigger subscriber. The arms below put
// a frame in exactly that window and hold every fire to reading a frame.

/// The latest-value source the gap injector reads. Never published: it only
/// gives the injector a latest-value input.
const GAP_CTX: &str = "/latest/gap_ctx";

/// The source [`EXT`]'s one publisher, shared by the test body and the gap
/// injector's tick.
type SharedPublisher = Arc<Mutex<Option<cerulion_core::transport::publisher::CerulionPublisher>>>;

/// A `period_ms = 1` node that publishes `gap` (0 = nothing) onto [`EXT`]
/// from its own tick, then clears it.
///
/// It sits in the consumer's level (neither has an in-graph producer), and
/// its latest-value input puts it on the level's serial pass, which ticks
/// before the rest of the level (`tick_decided_parallel` pass 1). Its frame
/// therefore lands after the level's trigger drain and before the consumer's
/// read: the window between the two pops of the separate discipline.
fn gap_injector(ext: SharedPublisher, gap: Arc<AtomicU64>) -> ClosureNodeEntry {
    let info = NodeInfo::from_names(vec!["ctx".to_string()], vec![])
        .with_policy(MacroPolicy::Period { period_ms: 1 });
    ClosureNodeEntry::new(info, move |_ctx| {
        let v = gap.swap(0, Ordering::Relaxed);
        if v != 0 {
            if let Some(p) = ext.lock().unwrap().as_mut() {
                publish_all(p, [v]);
            }
        }
        Ok(())
    })
    .with_label("latest_gap_injector")
}

fn gap_injector_node() -> NodeDef {
    NodeDef {
        isolate: None,
        params: Default::default(),
        ros2: None,
        id: "inj".to_string(),
        node_type: "latest_gap_injector".to_string(),
        inputs: vec![InputDef {
            name: "ctx".to_string(),
            source: GAP_CTX.to_string(),
            ..Default::default()
        }],
        outputs: vec![],
    }
}

/// A runtime holding the gap injector and `sink` (reading [`EXT`]), plus the
/// shared [`EXT`] publisher and the injector's `gap` cell.
fn gap_runtime(
    prefix: &str,
    sink: Box<dyn NodeEntry>,
    sink_def: NodeDef,
) -> (GraphRuntime, SharedPublisher, Arc<AtomicU64>) {
    let ext: SharedPublisher = Arc::new(Mutex::new(None));
    let gap = Arc::new(AtomicU64::new(0));
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "inj".to_string(),
        Box::new(gap_injector(Arc::clone(&ext), Arc::clone(&gap))),
    );
    factories.insert(sink_def.id.clone(), sink);
    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(
        graph(prefix, vec![gap_injector_node(), sink_def]),
        factories,
        clock,
        16,
    )
    .expect("build gap graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    *ext.lock().unwrap() = Some(
        mgr.create_publisher(EXT, MaxSliceLen::const_new(64), 0)
            .expect("external publisher attaches"),
    );
    (runtime, ext, gap)
}

/// The gap schedule, `idle` = the empty steps between the two halves:
///
/// 1. frame 1 queued, frame 2 armed on the injector, one step: the trigger
///    drain counts frame 1, the injector publishes frame 2, and the
///    consumer's read drains both and serves frame 2;
/// 2. one step: the trigger subscriber still holds frame 2, which the read
///    above already took, so nothing is owed;
/// 3. `idle` empty steps, then frame 3 and two steps: one fire reading 3.
///
/// Oracle, from the rule that a frame a read drained is not counted again:
/// reads `[2, 3]` in exactly 2 fires. Without the body probe, step 2 fires
/// once more on an empty queue: 3 fires.
fn drive_gap(
    runtime: &mut GraphRuntime,
    ext: &SharedPublisher,
    gap: &AtomicU64,
    sink: &str,
    idle: u32,
) -> u64 {
    gap.store(2, Ordering::Relaxed);
    publish_all(ext.lock().unwrap().as_mut().expect("publisher"), [1]);
    runtime.step(Duration::from_millis(1));
    assert_eq!(gap.load(Ordering::Relaxed), 0, "the injector ticked");
    runtime.step(Duration::from_millis(1));
    for _ in 0..idle {
        runtime.step(Duration::from_millis(1));
    }
    publish_all(ext.lock().unwrap().as_mut().expect("publisher"), [3]);
    runtime.step(Duration::from_millis(1));
    runtime.step(Duration::from_millis(1));
    runtime.node_handle(sink).unwrap().fire_count()
}

/// The gap schedule on a closure consumer reading through `try_view`.
fn run_gap_closure(prefix: &str, c: Consumer, opt_out_of_unified: bool) -> (Vec<u64>, u64) {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut entry = consumer_entry(c, Arc::clone(&seen));
    if opt_out_of_unified {
        entry = entry.with_unified_drain(false);
    }
    let (mut runtime, ext, gap) = gap_runtime(prefix, Box::new(entry), sink_node(EXT));
    let fired = drive_gap(&mut runtime, &ext, &gap, "sink", 4);
    let observed = seen.lock().unwrap().clone();
    (observed, fired)
}

/// Forced Separate, `drop_oldest`: a frame landing between the trigger drain
/// and the read is served by that read and fires nothing more. The closure
/// answers the probe, so the build names nothing.
#[test]
#[serial]
#[tracing_test::traced_test]
fn a_frame_landing_between_the_drains_is_read_once_and_fires_nothing_more() {
    let _guard = EnvVarGuard::set("CERULION_DRAIN_DISCIPLINE", "separate");
    let (observed, fired) = run_gap_closure("latgapsep", Consumer::latest(), false);
    assert!(!logs_contain(
        "consume = latest inputs on a hand-written NodeEntry lack the body probe"
    ));
    assert_eq!(observed, vec![2, 3]);
    assert_eq!(
        fired,
        observed.len() as u64,
        "every fire reads a frame: {fired} fires for reads {observed:?}"
    );
}

/// `sample(N)` keeps the separate discipline under the default build. The
/// first read is always admitted and frame 3 comes more than N ms after
/// frame 2, so no read here is decimated and every fire reads a frame.
#[test]
#[serial]
fn a_sampled_latest_input_reads_a_gap_frame_once() {
    let c = Consumer {
        backpressure: BackpressurePolicy::Sample(5),
        ..Consumer::latest()
    };
    let seen = Arc::new(Mutex::new(Vec::new()));
    let (mut runtime, ext, gap) = gap_runtime(
        "latgapsamp",
        Box::new(consumer_entry(c, Arc::clone(&seen))),
        sink_node(EXT),
    );
    let fired = drive_gap(&mut runtime, &ext, &gap, "sink", 6);
    let observed = seen.lock().unwrap().clone();
    assert_eq!(observed, vec![2, 3]);
    assert_eq!(fired, 2, "every fire reads a frame: reads {observed:?}");
    assert_eq!(
        runtime
            .node_handle("sink")
            .unwrap()
            .backpressure_sampled_count("inp"),
        0,
        "no read was decimated"
    );
}

/// A closure opted out of the unified drain keeps two subscribers under the
/// default build, and its body still answers the probe.
#[test]
#[serial]
fn an_entry_without_the_unified_drain_reads_a_gap_frame_once() {
    let (observed, fired) = run_gap_closure("latgapopt", Consumer::latest(), true);
    assert_eq!(observed, vec![2, 3]);
    assert_eq!(fired, 2, "every fire reads a frame: reads {observed:?}");
}

/// The `trigger_in` to `cmd` forwarder's node, id `node`, reading [`EXT`].
fn forwarder_node() -> NodeDef {
    NodeDef {
        isolate: None,
        params: Default::default(),
        ros2: None,
        id: "node".to_string(),
        node_type: "consume_latest_node".to_string(),
        inputs: vec![InputDef {
            name: "trigger_in".to_string(),
            source: EXT.to_string(),
            ..Default::default()
        }],
        outputs: vec![OutputDef {
            name: "cmd".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    }
}

/// The gap schedule on a forwarder (each fire publishes the frame its tick
/// read to `cmd`): returns the `cmd` stream, the read record, and the fires.
fn run_gap_forwarder(prefix: &str, entry: Box<dyn NodeEntry>) -> (Vec<u64>, u64) {
    let (mut runtime, ext, gap) = gap_runtime(prefix, entry, forwarder_node());
    let cmd_sub = runtime
        .test_transport()
        .expect("test transport parked")
        .create_subscriber_open_only(&format!("/{prefix}/node/cmd"))
        .expect("open the forwarder's cmd topic");
    let fired = drive_gap(&mut runtime, &ext, &gap, "node", 4);
    let mut forwarded: Vec<u64> = Vec::new();
    cmd_sub
        .try_receive(|msg| {
            let x = f64::from_le_bytes(msg.payload()[0..8].try_into().unwrap());
            forwarded.push(x as u64);
        })
        .expect("drain cmd stream");
    (forwarded, fired)
}

/// Through a dlopen'd `#[cerulion_node]` cdylib under forced Separate: the
/// body subscriber lives in the cdylib's own copy of the core, and the probe
/// reaches it across the FFI. The node forwards each read frame to `cmd`, so
/// the `cmd` stream is the read record.
#[test]
#[serial]
fn a_cdylib_reads_a_gap_frame_once_on_the_separate_drain() {
    use cerulion_core::graph::node::DylibNodeEntry;

    let _guard = EnvVarGuard::set("CERULION_DRAIN_DISCIPLINE", "separate");
    let entry = DylibNodeEntry::load(&cerulion_core::testing::find_fixture_cdylib(
        "test_node_macro_consume_latest_cdylib",
    ))
    .expect("load consume-latest fixture cdylib");
    let (forwarded, fired) = run_gap_forwarder("latgapdyl", Box::new(entry));
    assert_eq!(forwarded, vec![2, 3]);
    assert_eq!(
        fired, 2,
        "every fire reads a frame: forwarded {forwarded:?}"
    );
}

/// The cdylib fixture's node, compiled into this test binary: an in-process
/// `#[cerulion_node]` entry declares the probe through the macro.
#[cerulion_node]
#[derive(Default)]
struct InProcessLatestNode {
    #[input(trigger, consume = latest)]
    trigger_in: Vector3,
    #[output]
    cmd: Vector3,
}

#[cerulion_node_impl]
impl InProcessLatestNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.cmd.x = self.trigger_in.x;
        Ok(())
    }
}

/// An in-process `#[cerulion_node]` node under forced Separate reads a gap
/// frame once, and the build names nothing.
#[test]
#[serial]
#[tracing_test::traced_test]
fn an_in_process_macro_node_reads_a_gap_frame_once_on_the_separate_drain() {
    let _guard = EnvVarGuard::set("CERULION_DRAIN_DISCIPLINE", "separate");
    let entry = InProcessLatestNodeEntry::new();
    assert!(entry.probes_trigger_input(), "the macro declares the probe");
    let (forwarded, fired) = run_gap_forwarder("latgapmac", Box::new(entry));
    assert!(!logs_contain(
        "consume = latest inputs on a hand-written NodeEntry lack the body probe"
    ));
    assert_eq!(forwarded, vec![2, 3]);
    assert_eq!(
        fired, 2,
        "every fire reads a frame: forwarded {forwarded:?}"
    );
}

/// The `block` arm's source: an absolute topic with an in-graph producer (a
/// `block` input needs one) that `multi_publisher_topics` opens to a writer
/// outside the graph, the test's publisher.
const BLK: &str = "/latest/blk";

/// The `block` arm's graph: `pace` (period 1 ms) produces [`BLK`] and never
/// writes it, and writes one frame per tick on its `tick` output; `inj` fires
/// on each `tick` frame (a `block` input, so it is in `sink`'s level and in its
/// fused block group, which decides and ticks in graph order) and publishes
/// `gap` (0 = nothing) onto [`BLK`] through the test's publisher; `sink` reads
/// [`BLK`] through `entry`. `inj` ticks after the level's trigger drain and
/// before `sink`'s read, the window between the two pops.
fn block_gap_runtime(
    prefix: &str,
    sink: Box<dyn NodeEntry>,
) -> (GraphRuntime, SharedPublisher, Arc<AtomicU64>) {
    let ext: SharedPublisher = Arc::new(Mutex::new(None));
    let gap = Arc::new(AtomicU64::new(0));
    let pace_info = NodeInfo::from_names(vec![], vec!["out".to_string(), "tick".to_string()])
        .with_policy(MacroPolicy::Period { period_ms: 1 });
    let pace = ClosureNodeEntry::new(pace_info, |ctx| {
        if let Some(p) = ctx.publisher_mut("tick") {
            let mut proxy = p.loan_proxy::<Vector3>()?;
            proxy.x = 1.0;
        }
        Ok(())
    })
    .with_label("latest_block_pace");
    let inj_info = NodeInfo::with_meta(
        vec![InputMeta {
            name: "tick".to_string(),
            schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
            trigger: true,
            depth: DEFAULT_CONSUMER_DEPTH,
            backpressure: BackpressurePolicy::Block,
            expect_within_ms: None,
            consume: InputConsume::Latest,
            accumulate: false,
        }],
        vec![],
    )
    .with_policy(MacroPolicy::DataTrigger {
        input_name: "tick".to_string(),
    });
    let (inj_ext, inj_gap) = (Arc::clone(&ext), Arc::clone(&gap));
    let inj = ClosureNodeEntry::new(inj_info, move |ctx| {
        if let Some(s) = ctx.subscriber_mut("tick") {
            let _ = s.try_view::<Vector3, _>(|view| view.x);
        }
        let v = inj_gap.swap(0, Ordering::Relaxed);
        if v != 0 {
            if let Some(p) = inj_ext.lock().unwrap().as_mut() {
                publish_all(p, [v]);
            }
        }
        Ok(())
    })
    .with_label("latest_block_injector");
    let output = |name: &str, topic: Option<&str>| OutputDef {
        name: name.to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: topic.map(str::to_string),
    };
    let nodes = vec![
        NodeDef {
            isolate: None,
            params: Default::default(),
            ros2: None,
            id: "pace".to_string(),
            node_type: "latest_block_pace".to_string(),
            inputs: vec![],
            outputs: vec![output("out", Some(BLK)), output("tick", None)],
        },
        NodeDef {
            isolate: None,
            params: Default::default(),
            ros2: None,
            id: "inj".to_string(),
            node_type: "latest_block_injector".to_string(),
            inputs: vec![InputDef {
                name: "tick".to_string(),
                source: "pace/tick".to_string(),
                ..Default::default()
            }],
            outputs: vec![],
        },
        sink_node(BLK),
    ];
    let mut config = graph(prefix, nodes);
    config.multi_publisher_topics = vec![BLK.to_string()];
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("pace".to_string(), Box::new(pace));
    factories.insert("inj".to_string(), Box::new(inj));
    factories.insert("sink".to_string(), sink);
    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build block gap graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    *ext.lock().unwrap() = Some(
        mgr.create_publisher(BLK, MaxSliceLen::const_new(64), 0)
            .expect("a second publisher attaches to the multi-publisher topic"),
    );
    (runtime, ext, gap)
}

/// `block` keeps the separate discipline under the default build, and a
/// frame landing between its trigger drain and its read is served by that
/// read and fires nothing more. The queue never fills, so nothing is paced.
#[test]
#[serial]
fn a_blocking_latest_input_reads_a_gap_frame_once() {
    let c = Consumer {
        backpressure: BackpressurePolicy::Block,
        ..Consumer::latest()
    };
    let seen = Arc::new(Mutex::new(Vec::new()));
    let (mut runtime, ext, gap) =
        block_gap_runtime("latgapblk", Box::new(consumer_entry(c, Arc::clone(&seen))));
    let fired = drive_gap(&mut runtime, &ext, &gap, "sink", 4);
    let observed = seen.lock().unwrap().clone();
    assert_eq!(observed, vec![2, 3]);
    assert_eq!(fired, 2, "every fire reads a frame: reads {observed:?}");
}

/// A consumer whose body probe answers from `script` first and from its own
/// subscriber after. A scripted `Empty` while the body queue holds the frame
/// is what a frame the trigger subscriber received before the body queue did
/// looks like from the step: iceoryx2 delivers to one subscriber queue at a
/// time. The entry declares no unified drain, so its input is read through
/// two subscribers.
struct ScriptedProbe {
    inner: ClosureNodeEntry,
    script: std::collections::VecDeque<TriggerProbe>,
}

impl NodeEntry for ScriptedProbe {
    fn info(&self) -> TransportResult<NodeInfo> {
        self.inner.info()
    }
    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.inner.init(context)
    }
    fn tick(&mut self) -> TransportResult<()> {
        self.inner.tick()
    }
    fn probes_trigger_input(&self) -> bool {
        true
    }
    fn probe_trigger_input(&mut self, input_name: &str) -> TriggerProbe {
        match self.script.pop_front() {
            Some(answer) => answer,
            None => self.inner.probe_trigger_input(input_name),
        }
    }
}

/// One frame, probe answers `script` then the truth: returns (reads, fires)
/// after each of 4 steps.
fn run_scripted(prefix: &str, script: &[TriggerProbe]) -> Vec<(Vec<u64>, u64)> {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let entry = ScriptedProbe {
        inner: consumer_entry(Consumer::latest(), Arc::clone(&seen)),
        script: script.iter().copied().collect(),
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("sink".to_string(), Box::new(entry));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(graph(prefix, vec![sink_node(EXT)]), factories, clock, 16)
            .expect("build scripted graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher(EXT, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");
    publish_all(&mut ext, [1]);
    let mut after = Vec::new();
    for _ in 0..4 {
        runtime.step(Duration::from_millis(1));
        after.push((seen.lock().unwrap().clone(), fires(&runtime)));
    }
    after
}

/// The trigger subscriber counts frame 1 while the body shows nothing, so the
/// step fires nothing. The arrival is carried: the next step pops nothing,
/// probes again, finds the frame and fires once on it.
#[test]
#[serial]
fn a_frame_the_body_has_not_received_yet_fires_on_a_later_step() {
    let after = run_scripted("latcarry", &[TriggerProbe::Empty]);
    assert_eq!(after[0], (vec![], 0), "step 1: the body showed no frame");
    assert_eq!(after[1], (vec![1], 1), "step 2: the carried arrival fires");
    assert_eq!(after[3], (vec![1], 1), "nothing more");
}

/// A probe that cannot answer fires the node as the arrival count says.
#[test]
#[serial]
fn a_probe_that_cannot_answer_fires_on_the_arrival() {
    let after = run_scripted("latunknown", &[TriggerProbe::Unknown]);
    assert_eq!(after[0], (vec![1], 1));
    assert_eq!(after[3], (vec![1], 1));
}

/// An entry that answers no probe keeps firing on every counted arrival, and
/// the build names it once.
struct NoProbe(ClosureNodeEntry);

impl NodeEntry for NoProbe {
    fn info(&self) -> TransportResult<NodeInfo> {
        self.0.info()
    }
    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.0.init(context)
    }
    fn tick(&mut self) -> TransportResult<()> {
        self.0.tick()
    }
}

#[test]
#[serial]
#[tracing_test::traced_test]
fn the_build_names_a_latest_input_whose_body_answers_no_probe() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "sink".to_string(),
        Box::new(NoProbe(consumer_entry(Consumer::latest(), seen))),
    );
    let clock = Arc::new(VirtualClock::new());
    let _runtime = GraphRuntime::build_for_test(
        graph("latnoprobe", vec![sink_node(EXT)]),
        factories,
        clock,
        16,
    )
    .expect("build no-probe graph");
    assert!(logs_contain("sink.inp"), "the warn names node and input");
    assert!(logs_contain(
        "consume = latest inputs on a hand-written NodeEntry lack the body probe"
    ));
}

/// An entry that implements the per-set Sync head ops for its Sync inputs
/// alone and answers `Nothing` for any other input. It declares no body probe, and
/// declares no unified drain, so its input is read through two subscribers.
struct SyncOnlyHeadOps(ClosureNodeEntry);

impl NodeEntry for SyncOnlyHeadOps {
    fn info(&self) -> TransportResult<NodeInfo> {
        self.0.info()
    }
    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.0.init(context)
    }
    fn tick(&mut self) -> TransportResult<()> {
        self.0.tick()
    }
    fn supports_sync_head_ops(&self) -> bool {
        true
    }
    fn sync_head_op(&mut self, _input_name: &str, _op: SyncHeadOp) -> SyncOpAnswer {
        SyncOpAnswer::Nothing
    }
}

/// Head ops alone do not make the graph probe a latest input: the build names
/// the input, and its counted arrival fires the node, which reads the frame.
/// A probe through these head ops would answer `Empty` and never fire it.
#[test]
#[serial]
#[tracing_test::traced_test]
fn head_ops_for_sync_inputs_alone_leave_a_latest_input_unprobed_and_firing() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let entry = SyncOnlyHeadOps(consumer_entry(Consumer::latest(), Arc::clone(&seen)));
    assert!(!entry.probes_trigger_input());
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("sink".to_string(), Box::new(entry));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(
        graph("latsynconly", vec![sink_node(EXT)]),
        factories,
        clock,
        16,
    )
    .expect("build sync-only head-op graph");
    assert!(logs_contain("sink.inp"), "the warn names node and input");
    assert!(logs_contain(
        "consume = latest inputs on a hand-written NodeEntry lack the body probe"
    ));
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher(EXT, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");
    publish_all(&mut ext, [1]);
    for _ in 0..3 {
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(*seen.lock().unwrap(), vec![1]);
    assert_eq!(fires(&runtime), 1, "one fire on the one arrival");
}

/// The host body probe observes the actual subscriber, without a node export.
#[test]
#[serial]
fn every_cdylib_gets_the_host_body_probe_without_an_export() {
    use cerulion_core::graph::node::{AnySubscriber, DylibNodeEntry};
    use cerulion_core::graph::node_host_abi::with_context;
    use cerulion_core::testing::TestTransport;
    use cerulion_core::transport::plugin_lease::LeaseDirection;
    for _ in 0..2 {
        let tt = TestTransport::with_buffer_size(4);
        let topic = "test/consume_latest/host_probe/b";
        let mut publisher = tt.publisher(topic, MaxSliceLen::const_new(64), 0);
        let mut subscribers = IndexMap::new();
        subscribers.insert("b".into(), AnySubscriber::Ipc(tt.subscriber(topic)));
        let mut entry = DylibNodeEntry::load(&cerulion_core::testing::find_fixture_cdylib(
            "test_node_sync_op_fail_cdylib",
        ))
        .expect("load stable head-operation fixture");
        entry
            .init(NodeContext::for_tests(IndexMap::new(), subscribers))
            .unwrap();
        let context = entry.__test_host_context_handle().unwrap();
        assert_eq!(entry.probe_trigger_input("b"), TriggerProbe::Empty);
        publish_all(&mut publisher, [42]);
        assert_eq!(entry.probe_trigger_input("b"), TriggerProbe::Unread);
        with_context(context, |host| {
            host.begin_node_call()?;
            host.begin_fire()?;
            let port = host.bind_port(context, 1, Vector3::SCHEMA_HASH, LeaseDirection::Read)?;
            let frame = host
                .receive_input(context, port)?
                .expect("published frame is read");
            assert_eq!(frame.size, 64);
            host.release_input(context, port, frame.lease)?;
            host.end_node_call()
        })
        .unwrap();
        assert_eq!(entry.probe_trigger_input("b"), TriggerProbe::Empty);
        entry.shutdown().unwrap();
    }
}

/// A failed body probe on a cdylib is reported in its own words, on its own
/// latch. Holding the real host context lock refuses both the body probe
/// and the per-set Sync operations with INVALID_STATE, without touching
/// the node. Three failed probes give one loud line naming the probe, and the
/// Sync op failing after them still gets its own loud line.
#[test]
#[serial]
#[tracing_test::traced_test]
fn a_failed_cdylib_body_probe_says_so_on_its_own_latch() {
    use cerulion_core::graph::node::DylibNodeEntry;

    let mut entry = DylibNodeEntry::load(&cerulion_core::testing::find_fixture_cdylib(
        "test_node_sync_op_fail_cdylib",
    ))
    .expect("load stable head-operation fixture cdylib");
    entry
        .init(NodeContext::for_tests(IndexMap::new(), IndexMap::new()))
        .expect("init the stable head-operation fixture");
    let context = entry
        .__test_host_context_handle()
        .expect("initialized host context");
    cerulion_core::graph::node_host_abi::with_context(context, |_| {
        for _ in 0..3 {
            assert_eq!(entry.probe_trigger_input("b"), TriggerProbe::Unknown);
        }
        for _ in 0..3 {
            assert_eq!(
                entry.sync_head_op("b", SyncHeadOp::ProbeNext),
                SyncOpAnswer::Failed
            );
        }
        Ok(())
    })
    .expect("hold the real host context lock");
    logs_assert(|lines: &[&str]| {
        let probe_heads: Vec<&str> = lines
            .iter()
            .filter(|l| l.contains("ERROR") && l.contains("consume = latest body probe"))
            .copied()
            .collect();
        if probe_heads.len() != 1 {
            return Err(format!(
                "one loud line for the failing probe, got {}: {probe_heads:?}",
                probe_heads.len()
            ));
        }
        if probe_heads[0].contains("alignment") {
            return Err(format!(
                "the probe line names the probe, not the Sync pass: {}",
                probe_heads[0]
            ));
        }
        if !probe_heads[0].contains("stable consume = latest body probe got no answer (host head op PROBE_NEXT failed or answered an unsupported kind); this input fires on every arrival its trigger subscriber counts, including a frame an earlier read already took, until the probe answers (suppressing further to debug until it recovers)") {
            return Err("the probe must name the stable host operation exactly".into());
        }
        if !lines.iter().any(|line| line.contains("stable host Sync head op failed; the per-set Sync alignment pass ends here — no set member fabricated (suppressing further to debug until it recovers)")) { // dash-ok: native alignment message separator retained
            return Err("the Sync failure must name the stable host operation exactly".into());
        }
        if !probe_heads[0].contains("status=9")
            || !probe_heads[0].contains("kind=4")
            || !probe_heads[0].contains("error=host head operation refused with status 9")
        {
            return Err(
                "the probe must retain the actual host status, answer kind and refusal detail"
                    .into(),
            );
        }
        if !lines.iter().any(|line| {
            line.contains("stable host Sync head op failed")
                && line.contains("status=9")
                && line.contains("error=host head operation refused with status 9")
        }) {
            return Err("the Sync failure must retain its host status and refusal detail".into());
        }
        let sync_heads = lines
            .iter()
            .filter(|l| l.contains("ERROR") && l.contains("per-set Sync alignment pass ends here"))
            .count();
        if sync_heads != 1 {
            return Err(format!(
                "the Sync op's first failure is loud on its own latch, got {sync_heads} lines"
            ));
        }
        Ok(())
    });
}

/// A cdylib's probe latch is per input. With host admission refused for input `b`,
/// alternating probes of a failing `b` and an answering `a` give one loud
/// line for `b` and no recovery line: `a`'s answer leaves `b`'s regime
/// latched, as a shared latch would not.
#[test]
#[serial]
#[tracing_test::traced_test]
fn a_cdylib_body_probe_failing_on_one_input_stays_latched_while_another_answers() {
    use cerulion_core::graph::node::{AnySubscriber, DylibNodeEntry};
    use cerulion_core::testing::TestTransport;

    let tt = TestTransport::with_buffer_size(4);
    let mut subscribers: IndexMap<String, AnySubscriber> = IndexMap::new();
    subscribers.insert(
        "a".to_string(),
        AnySubscriber::Ipc(tt.subscriber("test/consume_latest/probe_latch/a")),
    );
    let mut entry = DylibNodeEntry::load(&cerulion_core::testing::find_fixture_cdylib(
        "test_node_sync_op_fail_cdylib",
    ))
    .expect("load stable head-operation fixture cdylib");
    entry
        .init(NodeContext::for_tests(IndexMap::new(), subscribers))
        .expect("init the stable head-operation fixture");
    let context = entry
        .__test_host_context_handle()
        .expect("initialized host context");
    for _ in 0..3 {
        cerulion_core::graph::node_host_abi::with_context(context, |_| {
            assert_eq!(entry.probe_trigger_input("b"), TriggerProbe::Unknown);
            Ok(())
        })
        .expect("refuse the probe through the real host admission lock");
        assert_eq!(entry.probe_trigger_input("a"), TriggerProbe::Empty);
    }
    logs_assert(|lines: &[&str]| {
        let heads = lines
            .iter()
            .filter(|l| l.contains("ERROR") && l.contains("consume = latest body probe"))
            .count();
        if heads != 1 {
            return Err(format!("one loud line for the failing probe, got {heads}"));
        }
        let recoveries = lines
            .iter()
            .filter(|l| l.contains("body probe recovered"))
            .count();
        if recoveries != 0 {
            return Err(format!(
                "the answering input recovers no other input's latch, got {recoveries} lines"
            ));
        }
        Ok(())
    });
}

/// Wraps an entry and times each [`NodeEntry::probe_trigger_input`] call.
struct TimedProbe<E> {
    inner: E,
    probe_ns: Arc<Mutex<Vec<u64>>>,
}

impl<E: NodeEntry> NodeEntry for TimedProbe<E> {
    fn info(&self) -> TransportResult<NodeInfo> {
        self.inner.info()
    }
    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.inner.init(context)
    }
    fn tick(&mut self) -> TransportResult<()> {
        self.inner.tick()
    }
    fn pump_history(&mut self) {
        self.inner.pump_history()
    }
    fn shutdown(&mut self) -> TransportResult<()> {
        self.inner.shutdown()
    }
    fn supports_sync_head_ops(&self) -> bool {
        self.inner.supports_sync_head_ops()
    }
    fn sync_head_op(&mut self, input_name: &str, op: SyncHeadOp) -> SyncOpAnswer {
        self.inner.sync_head_op(input_name, op)
    }
    fn probes_trigger_input(&self) -> bool {
        self.inner.probes_trigger_input()
    }
    fn probe_trigger_input(&mut self, input_name: &str) -> TriggerProbe {
        let t0 = std::time::Instant::now();
        let answer = self.inner.probe_trigger_input(input_name);
        let ns = t0.elapsed().as_nanos() as u64;
        self.probe_ns.lock().unwrap().push(ns);
        answer
    }
}

/// Idle steps timed per phase.
const MEASURE_STEPS: usize = 2000;
/// Carried and cleared phases per run.
const MEASURE_ROUNDS: usize = 10;

/// Per-step wall times (ns) of a gap runtime's idle steps, carried phase and
/// cleared phase, `MEASURE_ROUNDS` of each.
///
/// Each round arms the gap schedule's first two steps (frame 2 is read by the
/// step-1 tick and counted by the step-2 trigger drain, whose probe answers
/// `Empty` and carries the arrival), times `MEASURE_STEPS` idle steps, clears
/// the carry with a fresh frame (its probe answers `Unread` and the node fires
/// once), and times `MEASURE_STEPS` more idle steps.
fn measure_phases(
    prefix: &str,
    entry: Box<dyn NodeEntry>,
    def: NodeDef,
    node: &str,
) -> (Vec<u64>, Vec<u64>, u64) {
    fn timed_steps(runtime: &mut GraphRuntime, out: &mut Vec<u64>) {
        for _ in 0..MEASURE_STEPS {
            let t0 = std::time::Instant::now();
            runtime.step(Duration::from_millis(1));
            out.push(t0.elapsed().as_nanos() as u64);
        }
    }
    let (mut runtime, ext, gap) = gap_runtime(prefix, entry, def);
    let mut carried = Vec::with_capacity(MEASURE_STEPS * MEASURE_ROUNDS);
    let mut cleared = Vec::with_capacity(MEASURE_STEPS * MEASURE_ROUNDS);
    for round in 0..MEASURE_ROUNDS as u64 {
        gap.store(2 + 10 * round, Ordering::Relaxed);
        publish_all(
            ext.lock().unwrap().as_mut().expect("publisher"),
            [1 + 10 * round],
        );
        runtime.step(Duration::from_millis(1));
        runtime.step(Duration::from_millis(1));
        timed_steps(&mut runtime, &mut carried);
        publish_all(
            ext.lock().unwrap().as_mut().expect("publisher"),
            [3 + 10 * round],
        );
        runtime.step(Duration::from_millis(1));
        timed_steps(&mut runtime, &mut cleared);
    }
    let fired = runtime.node_handle(node).unwrap().fire_count();
    (carried, cleared, fired)
}

fn percentile(sorted: &[u64], p: f64) -> u64 {
    sorted[((sorted.len() - 1) as f64 * p).round() as usize]
}

fn report(label: &str, samples: &mut [u64]) -> (f64, u64) {
    samples.sort_unstable();
    let mean = samples.iter().sum::<u64>() as f64 / samples.len() as f64;
    let p50 = percentile(samples, 0.50);
    println!(
        "{label}: n={} mean={mean:.0} ns p10={} p50={p50} p90={} p99={} ns",
        samples.len(),
        percentile(samples, 0.10),
        percentile(samples, 0.90),
        percentile(samples, 0.99),
    );
    (mean, p50)
}

fn closure_consumer() -> (ClosureNodeEntry, NodeDef) {
    let seen = Arc::new(Mutex::new(Vec::new()));
    (
        consumer_entry(Consumer::latest(), seen).with_unified_drain(false),
        sink_node(EXT),
    )
}

fn cdylib_forwarder() -> (cerulion_core::graph::node::DylibNodeEntry, NodeDef) {
    let entry = cerulion_core::graph::node::DylibNodeEntry::load(
        &cerulion_core::testing::find_fixture_cdylib("test_node_macro_consume_latest_cdylib"),
    )
    .expect("load consume-latest fixture cdylib");
    (entry, forwarder_node())
}

/// The cost of a carried arrival: one body probe per step while the arrival
/// is carried, on a closure and on a dlopen'd `#[cerulion_node]` cdylib,
/// both on the separate drain.
///
/// Per entry it prints the idle-step wall time with the arrival carried and
/// with it cleared (their difference is the probe's per-step cost, node lock
/// and FFI included), and, through [`TimedProbe`], the probe call alone.
///
/// ```bash
/// cargo build --release -p test_node_macro_consume_latest_cdylib
/// cargo test --release -p cerulion_core --test consume_latest_iox2_test \
///     -- --ignored --nocapture measure_the_carried_probe_cost
/// ```
#[test]
#[serial]
#[ignore = "measurement: prints the carried probe's per-step cost; run with --ignored --nocapture"]
fn measure_the_carried_probe_cost() {
    let _guard = EnvVarGuard::set("CERULION_DRAIN_DISCIPLINE", "separate");
    let rounds = MEASURE_ROUNDS as u64;
    for kind in ["closure", "cdylib"] {
        let (entry, def, node): (Box<dyn NodeEntry>, NodeDef, &str) = match kind {
            "closure" => {
                let (e, d) = closure_consumer();
                (Box::new(e), d, "sink")
            }
            _ => {
                let (e, d) = cdylib_forwarder();
                (Box::new(e), d, "node")
            }
        };
        let (mut carried, mut cleared, fired) =
            measure_phases(&format!("latmeas{kind}"), entry, def, node);
        assert_eq!(fired, 2 * rounds, "{kind}: two fires per round");
        let (carried_mean, carried_p50) =
            report(&format!("{kind} idle step, carried"), &mut carried);
        let (cleared_mean, cleared_p50) =
            report(&format!("{kind} idle step, cleared"), &mut cleared);
        println!(
            "{kind} carried minus cleared: mean {:+.0} ns, p50 {:+} ns",
            carried_mean - cleared_mean,
            carried_p50 as i64 - cleared_p50 as i64
        );

        let probe_ns = Arc::new(Mutex::new(Vec::new()));
        let (inner, def, node): (Box<dyn NodeEntry>, NodeDef, &str) = match kind {
            "closure" => {
                let (e, d) = closure_consumer();
                (
                    Box::new(TimedProbe {
                        inner: e,
                        probe_ns: Arc::clone(&probe_ns),
                    }),
                    d,
                    "sink",
                )
            }
            _ => {
                let (e, d) = cdylib_forwarder();
                (
                    Box::new(TimedProbe {
                        inner: e,
                        probe_ns: Arc::clone(&probe_ns),
                    }),
                    d,
                    "node",
                )
            }
        };
        let (_, _, fired) = measure_phases(&format!("latmeasw{kind}"), inner, def, node);
        assert_eq!(fired, 2 * rounds, "{kind} wrapped: two fires per round");
        let mut probes = probe_ns.lock().unwrap().clone();
        assert_eq!(
            probes.len() as u64,
            rounds * (MEASURE_STEPS as u64 + 3),
            "{kind}: per round, one probe per carried step plus the three counted \
             arrivals' probes"
        );
        report(&format!("{kind} probe call"), &mut probes);
    }
}
