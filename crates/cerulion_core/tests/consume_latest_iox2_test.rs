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
//! | `block` | the read empties the queue every step, so the producer is never paced; superseded frames are not served |
//! | `sample(N)` | the gate admits the drain's newest frame at most once per N ms; a decimated one is counted |
//!
//! `#[serial]`: the forced-Separate arms mutate the process-global
//! `CERULION_DRAIN_DISCIPLINE` env.
//!
//! # Running
//!
//! ```bash
//! cargo build -p test_node_macro_consume_latest_cdylib
//! cargo nextest run -p cerulion_core --test consume_latest_iox2_test
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{
    BackpressurePolicy, ClosureNodeEntry, InputConsume, InputMeta, NodeEntry, NodeInfo,
};
use cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::MacroPolicy;
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
        fuse: None,
        params: Default::default(),
        ros2: None,
        id: "sink".to_string(),
        node_type: "latest_consumer".to_string(),
        inputs: vec![InputDef {
            name: "inp".to_string(),
            source: source.to_string(),
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
    assert_eq!(observed, vec![5], "one fire, on the newest frame of the burst");
    assert_eq!(n, 1, "exactly one fire: the idle steps owe nothing");
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
    assert_eq!(n, 2, "the per-arrival signals collapse to one fire per step");
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

/// `block` composition. A `period_ms = 1` producer publishes one frame per
/// step into a depth-4 `block` edge whose consumer is throttled to one fire
/// per 3 ms. Every step's read drains the queue, so it never fills and the
/// producer is never deferred, and each fire reads the newest frame:
/// 1, then every third frame. As `fifo` the same graph paces the producer and
/// serves 1, 2, 3, ... contiguously (`fifo_consume_iox2_test`).
#[test]
#[serial]
fn block_with_latest_never_paces_the_producer_and_serves_the_newest() {
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
        throttle_ms: Some(3),
        ..Consumer::latest()
    };
    let producer_node = NodeDef {
        fuse: None,
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
        graph("latblock", vec![producer_node, sink_node("producer/out")]),
        factories,
        clock,
        16,
    )
    .expect("build block graph");
    for _ in 0..30 {
        runtime.step(Duration::from_millis(1));
    }
    let observed = seen.lock().unwrap().clone();
    let expected: Vec<u64> = (0..observed.len() as u64).map(|k| 1 + 3 * k).collect();
    assert!(observed.len() >= 9, "a real stream: {observed:?}");
    assert_eq!(observed, expected);
    assert_eq!(
        runtime
            .node_handle("producer")
            .unwrap()
            .backpressure_block_fires_deferred_count("inp")
            + runtime
                .node_handle("sink")
                .unwrap()
                .backpressure_block_fires_deferred_count("inp"),
        0,
        "the queue never fills, so the block gate never defers"
    );
    assert_eq!(counter.load(Ordering::Relaxed), 30, "one publish per step");
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
    assert_eq!(meta[0].consume, InputConsume::Latest, "the key crossed the FFI");

    let node = NodeDef {
        fuse: None,
        params: Default::default(),
        ros2: None,
        id: "node".to_string(),
        node_type: "consume_latest_node".to_string(),
        inputs: vec![InputDef {
            name: "trigger_in".to_string(),
            source: IN_TOPIC.to_string(),
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
