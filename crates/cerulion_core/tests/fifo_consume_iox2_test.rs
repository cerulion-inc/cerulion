// SPDX-License-Identifier: AGPL-3.0-only
//! Per-message FIFO consumption for data-trigger inputs — the delivery
//! contract "fire on each message arriving" made TRUE.
//!
//! Without per-message consumption a burst of N frames between fires collapses to ONE
//! fire observing only the NEWEST frame (drain-to-latest + the scheduler's
//! pending-count reset): N−1 frames are silently discarded, uncounted. A
//! data-trigger consumer fires once per queued frame, in FIFO arrival
//! order, each tick observing exactly that frame — the scheduler consumes
//! one pending arrival per step and carries the remainder, and the
//! trigger-input read pops the FIFO head instead of draining to the latest.
//!
//! Every oracle here is HAND-WRITTEN (never a self-compare), and the
//! headline arms FAIL on the pre-FIFO code (one fire seeing only the burst's
//! last value). The consumer reads via `try_view` — the frozen-slot-served
//! read path, the same code path the `#[cerulion_node]` macro's generated
//! tick uses — so these pins cover the production read semantics on BOTH
//! drain disciplines (Unified, and forced-Separate via the
//! `CERULION_DRAIN_DISCIPLINE` seam).
//!
//! Scope boundary pinned by the negative control: latest-value CONTEXT
//! inputs (non-trigger) KEEP drain-to-latest + cross-step hold — the FIFO
//! flip applies to Data-policy trigger inputs only.
//!
//! `#[serial]`: the forced-Separate arm mutates the process-global
//! `CERULION_DRAIN_DISCIPLINE` env; the whole file runs serially for
//! simplicity (it is fast).
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test fifo_consume_iox2_test -- --test-threads=1
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{
    BackpressurePolicy, ClosureNodeEntry, InputMeta, NodeEntry, NodeInfo,
};
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

/// Build a graph with ONE data-trigger closure consumer on the absolute
/// external source `/fifo/ext`, recording every `try_view`-observed value in
/// arrival order into `seen`.
fn burst_consumer_graph(
    prefix: &str,
    depth: usize,
    backpressure: BackpressurePolicy,
    seen: Arc<Mutex<Vec<u64>>>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let consumer_info = NodeInfo::with_meta(
        vec![InputMeta {
            name: "inp".to_string(),
            schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
            trigger: true,
            depth,
            backpressure,
            expect_within_ms: None,
        }],
        vec![],
    )
    .with_policy(MacroPolicy::DataTrigger {
        input_name: "inp".to_string(),
    });
    let consumer = ClosureNodeEntry::new(consumer_info, move |ctx| {
        if let Some(s) = ctx.subscriber_mut("inp") {
            if let Ok(Some(v)) = s.try_view::<Vector3, _>(|view| view.x as u64) {
                seen.lock().unwrap().push(v);
            }
        }
        Ok(())
    })
    .with_label("fifo_burst_consumer");

    let config = GraphConfig {
        execution: None,
        name: None,
        identity: "fifo_consume".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "sink".to_string(),
            node_type: "fifo_burst_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: "/fifo/ext".to_string(),
            }],
            outputs: vec![],
        }],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("sink".to_string(), Box::new(consumer));
    (config, factories)
}

/// Publish `values` as Vector3.x frames from an external publisher.
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

/// Drive one burst-then-drain run and return the observed sequence.
fn run_burst(prefix: &str, burst: &[u64], drain_steps: u32) -> Vec<u64> {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = burst_consumer_graph(
        prefix,
        cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
        BackpressurePolicy::DropOldest,
        Arc::clone(&seen),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build fifo graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher("/fifo/ext", MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");

    // The whole burst lands BEFORE any consumer fire — the exact shape the
    // pre-FIFO collapse silently truncated to its last value.
    publish_all(&mut ext, burst.iter().copied());
    for _ in 0..drain_steps {
        runtime.step(Duration::from_millis(1));
    }
    let out = seen.lock().unwrap().clone();
    out
}

/// HEADLINE: a burst of 5 frames yields 5 fires, each observing the next
/// frame in FIFO arrival order — the hand oracle is the full sequence.
/// Pre-FIFO code fires ONCE and observes only `[5]`.
#[test]
#[serial]
fn burst_is_delivered_per_message_in_fifo_order() {
    let observed = run_burst("fifoburst", &[1, 2, 3, 4, 5], 8);
    assert_eq!(
        observed,
        vec![1, 2, 3, 4, 5],
        "every burst frame must be delivered to its own fire, in arrival \
         order (pre-FIFO drain-to-latest observed only [5])"
    );
}

/// The forced-Separate discipline delivers the IDENTICAL sequence — the
/// drain-discipline seam changes HOW frames are drained, never WHAT the
/// node observes (the unified-drain invariant, now per-message).
#[test]
#[serial]
fn forced_separate_discipline_delivers_the_same_fifo_sequence() {
    let _guard = EnvVarGuard::set("CERULION_DRAIN_DISCIPLINE", "separate");
    let observed = run_burst("fifosep", &[1, 2, 3, 4, 5], 8);
    assert_eq!(
        observed,
        vec![1, 2, 3, 4, 5],
        "Separate-discipline delivery must match the Unified FIFO sequence"
    );
}

/// The burst is served by the step that SEES it — ONE step, not one
/// step per frame. The arms above allow 8 steps for 5 frames, so they hold on
/// a consumer whose throughput is capped at one frame per step; this one does
/// not, and that cap is what took the CLI e2e graph-latency gate from ~25 us
/// p50 to 8.75 ms (a data-trigger consumer behind a 1 kHz producer falls a
/// queue-depth behind and can never work it off).
#[test]
#[serial]
fn a_queued_burst_is_served_within_one_step() {
    let observed = run_burst("fifoonestep", &[1, 2, 3, 4, 5], 1);
    assert_eq!(
        observed,
        vec![1, 2, 3, 4, 5],
        "one step must serve the whole queued burst; a one-fire-per-step \
         consumer observes only [1] and carries the rest"
    );
}

/// The Separate discipline reaches the same place by a different route — its
/// boundary drain signals one arrival per queued frame, so the fire count is
/// already the burst and no refill is involved. Pinned because the two
/// disciplines must agree on WHAT the node observes AND on WHEN, and because a
/// refill that were the only mechanism would leave this arm passing on a
/// regression that broke the Unified default.
#[test]
#[serial]
fn the_separate_discipline_also_serves_a_queued_burst_within_one_step() {
    let _guard = EnvVarGuard::set("CERULION_DRAIN_DISCIPLINE", "separate");
    let observed = run_burst("fifoonestepsep", &[1, 2, 3, 4, 5], 1);
    assert_eq!(
        observed,
        vec![1, 2, 3, 4, 5],
        "Separate must serve the burst within one step too"
    );
}

/// Determinism (Principle #7): two identical runs produce byte-identical
/// sequences, both equal to the hand oracle (never a bare self-compare).
#[test]
#[serial]
fn fifo_delivery_is_deterministic_across_runs() {
    let a = run_burst("fifodet_a", &[7, 8, 9, 10], 6);
    let b = run_burst("fifodet_b", &[7, 8, 9, 10], 6);
    assert_eq!(a, vec![7, 8, 9, 10], "run A == hand oracle");
    assert_eq!(b, a, "run B byte-identical to run A");
}

/// drop_oldest composition: a burst deeper than the queue evicts the oldest
/// (counted EXACTLY by the eviction probe) and the SURVIVORS are delivered
/// per-message in FIFO order. Depth 4, burst 8 after a served baseline
/// frame ⇒ retained newest 4, evicted 4.
#[test]
#[serial]
fn overflow_evicts_counted_and_survivors_flow_fifo() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = burst_consumer_graph(
        "fifoevict",
        4,
        BackpressurePolicy::DropOldest,
        Arc::clone(&seen),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build evict graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher("/fifo/ext", MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");

    // Baseline: the stream's first observed frame establishes the eviction
    // baseline (uncounted) — it must be SERVED for the baseline to exist.
    publish_all(&mut ext, [100]);
    runtime.step(Duration::from_millis(1));

    // Burst 8 into the depth-4 queue: iceoryx2 retains the newest 4
    // (values 5..=8), evicting 4 — which the probe counts exactly when the
    // first retained frame is served (wire-sequence gap vs the baseline).
    publish_all(&mut ext, [1, 2, 3, 4, 5, 6, 7, 8]);
    for _ in 0..6 {
        runtime.step(Duration::from_millis(1));
    }

    assert_eq!(
        *seen.lock().unwrap(),
        vec![100, 5, 6, 7, 8],
        "baseline + the 4 retained frames, in FIFO order"
    );
    assert_eq!(
        runtime
            .node_handle("sink")
            .unwrap()
            .backpressure_drop_oldest_count("inp"),
        4,
        "the 4 evicted frames are counted exactly (loss is observable, \
         never silent)"
    );
}

/// `sample(N)` composition: the read-gate decimates (counted) and the
/// ACCEPTED frames flow per-message in order. Frames published in one
/// same-timestamp burst: the first is accepted, the rest decimated; a later
/// frame (fresh timestamp beyond the interval) is accepted again.
#[test]
#[serial]
fn sample_gate_decimates_counted_and_accepted_frames_flow_fifo() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = burst_consumer_graph(
        "fifosamp",
        cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
        BackpressurePolicy::Sample(5),
        Arc::clone(&seen),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build sample graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher("/fifo/ext", MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");

    // Three frames stamped at the same virtual instant: the first opens the
    // gate (accepted), the next two are inside the 5 ms window (decimated).
    publish_all(&mut ext, [1, 2, 3]);
    for _ in 0..4 {
        runtime.step(Duration::from_millis(1));
    }
    // 4 steps advanced the clock 4 ms; two more put the next frame's stamp
    // past the 5 ms interval — accepted.
    runtime.step(Duration::from_millis(1));
    runtime.step(Duration::from_millis(1));
    publish_all(&mut ext, [4]);
    for _ in 0..3 {
        runtime.step(Duration::from_millis(1));
    }

    assert_eq!(
        *seen.lock().unwrap(),
        vec![1, 4],
        "accepted frames flow in order; decimated frames are dropped by the \
         gate, not silently superseded"
    );
    assert_eq!(
        runtime
            .node_handle("sink")
            .unwrap()
            .backpressure_sampled_count("inp"),
        2,
        "both decimated frames are counted"
    );
}

/// Negative control — the scope boundary: a latest-value CONTEXT input
/// (non-trigger, on a Period node) KEEPS drain-to-latest. A burst between
/// fires yields ONE read observing only the NEWEST value: state samples
/// supersede, per the latest-value contract (cross-step hold semantics).
#[test]
#[serial]
fn latest_value_context_inputs_keep_drain_to_latest() {
    let last = Arc::new(AtomicU64::new(0));
    let last_c = Arc::clone(&last);
    let info = NodeInfo::with_meta(
        vec![InputMeta {
            name: "ctx_in".to_string(),
            schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
            trigger: false,
            depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
            backpressure: BackpressurePolicy::DropOldest,
            expect_within_ms: None,
        }],
        vec![],
    )
    .with_policy(MacroPolicy::Period { period_ms: 1 });
    let reader = ClosureNodeEntry::new(info, move |ctx| {
        if let Some(s) = ctx.subscriber_mut("ctx_in") {
            if let Ok(Some(v)) = s.try_view::<Vector3, _>(|view| view.x as u64) {
                last_c.store(v, Ordering::Relaxed);
            }
        }
        Ok(())
    })
    .with_label("latest_ctx_reader");

    let config = GraphConfig {
        execution: None,
        name: None,
        identity: "fifo_ctx_control".to_string(),
        prefix: "fifoctx".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "reader".to_string(),
            node_type: "latest_ctx_reader".to_string(),
            inputs: vec![InputDef {
                name: "ctx_in".to_string(),
                source: "/fifo/ext".to_string(),
            }],
            outputs: vec![],
        }],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("reader".to_string(), Box::new(reader));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build ctx graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher("/fifo/ext", MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");

    publish_all(&mut ext, [1, 2, 3, 4, 5]);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        last.load(Ordering::Relaxed),
        5,
        "a non-trigger latest-value context read serves the NEWEST sample \
         (drain-to-latest is the CORRECT semantic for state samples — the \
         FIFO flip is scoped to data-trigger inputs)"
    );
}

/// `block` + FIFO on the macro-equivalent read path: a producer throttled
/// only by the block gate feeds a consumer that drains slower than the
/// producer publishes (consumer `throttle_ms = 3`). The block pre-fire
/// defer paces the producer so the queue NEVER overflows, and per-message
/// FIFO consumption serves EVERY frame in order — end-to-end lossless flow
/// control, observed at the tick (not merely at the queue). The pre-FIFO
/// read path discarded all but the newest queued frame at every fire, so
/// this CONTIGUITY oracle fails on it.
#[test]
#[serial]
fn block_with_fifo_is_lossless_end_to_end_on_the_tick_path() {
    const DEPTH: usize = 4;
    const STEPS: u32 = 60;

    let seen = Arc::new(Mutex::new(Vec::new()));

    // In-graph producer (block requires one): publishes exactly ONE frame
    // per tick, counter-valued, every step (Period 1 ms).
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
    .with_label("block_paced_producer");

    // Data-trigger block consumer, throttled to at most one fire per 3 ms —
    // strictly slower than the producer, so the queue fills and the block
    // gate must defer the producer (lossless pacing).
    let seen_c = Arc::clone(&seen);
    let consumer_info = NodeInfo::with_meta(
        vec![InputMeta {
            name: "inp".to_string(),
            schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
            trigger: true,
            depth: DEPTH,
            backpressure: BackpressurePolicy::Block,
            expect_within_ms: None,
        }],
        vec![],
    )
    .with_policy(MacroPolicy::DataTrigger {
        input_name: "inp".to_string(),
    })
    .with_throttle_ms(3);
    let consumer = ClosureNodeEntry::new(consumer_info, move |ctx| {
        if let Some(s) = ctx.subscriber_mut("inp") {
            if let Ok(Some(v)) = s.try_view::<Vector3, _>(|view| view.x as u64) {
                seen_c.lock().unwrap().push(v);
            }
        }
        Ok(())
    })
    .with_label("block_fifo_consumer");

    let config = GraphConfig {
        execution: None,
        name: None,
        identity: "fifo_block_lossless".to_string(),
        prefix: "fifoblk".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "block_paced_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    topic: None,
                    history_size: 0,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "sink".to_string(),
                node_type: "block_fifo_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(producer));
    factories.insert("sink".to_string(), Box::new(consumer));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build block graph");

    for _ in 0..STEPS {
        runtime.step(Duration::from_millis(1));
    }
    // Let the throttled consumer drain the tail after the producer stalls
    // at the gate.
    for _ in 0..(DEPTH as u32 * 4) {
        runtime.step(Duration::from_millis(1));
    }

    let observed = seen.lock().unwrap().clone();
    assert!(
        observed.len() >= 4,
        "the consumer must have drained a real stream (got {observed:?})"
    );
    let expected: Vec<u64> = (1..=observed.len() as u64).collect();
    assert_eq!(
        observed, expected,
        "CONTIGUOUS 1..=N on the tick read path — block pacing + FIFO \
         consumption loses nothing end to end (pre-FIFO the tick observed \
         only the newest queued frame per fire, leaving gaps)"
    );
    let handle = runtime.node_handle("producer").unwrap();
    assert!(
        handle.backpressure_block_fires_deferred_count("inp") > 0
            || runtime
                .node_handle("sink")
                .unwrap()
                .backpressure_block_fires_deferred_count("inp")
                > 0,
        "the block gate must actually have deferred the producer at least \
         once (else this arm proves nothing about block pacing)"
    );
    assert_eq!(
        runtime
            .node_handle("sink")
            .unwrap()
            .backpressure_drop_oldest_count("inp"),
        0,
        "no eviction ever — the queue never overflowed"
    );
}

/// PRODUCTION-PATH PARITY (the inert-shipping rule): the SAME per-message
/// FIFO contract through a REAL dlopen'd `#[cerulion_node]` cdylib
/// (`test_node_macro_data_trigger_cdylib`, which mirrors `trigger_in.x`
/// into `cmd.x` each fire) — the exact `cerulion graph run` node shape.
/// A burst of 5 frames before any fire must produce 5 downstream `cmd`
/// frames carrying 1..=5 in order: each fire's macro-generated `try_view`
/// observed its own frame. Pre-FIFO the cdylib fired once and forwarded
/// only `[5]`.
#[test]
#[serial]
fn cdylib_data_trigger_forwards_a_burst_per_message_in_order() {
    use cerulion_core::graph::node::DylibNodeEntry;

    const IN_TOPIC: &str = "/fifodyl/in";
    let config = GraphConfig {
        execution: None,
        name: None,
        identity: "fifo_cdylib_parity".to_string(),
        prefix: "fifodyl".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "node".to_string(),
            node_type: "data_trigger_node".to_string(),
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
        }],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "node".to_string(),
        Box::new(
            DylibNodeEntry::load(&cerulion_core::testing::find_fixture_cdylib(
                "test_node_macro_data_trigger_cdylib",
            ))
            .expect("load data-trigger fixture cdylib"),
        ),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build cdylib graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher(IN_TOPIC, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");
    // Downstream capture on the cdylib's OUTPUT topic: every fire publishes
    // its observed value, so the cmd stream IS the observation record.
    let cmd_sub = mgr
        .create_subscriber_open_only("/fifodyl/node/cmd")
        .expect("open the cdylib's cmd topic");

    publish_all(&mut ext, [1, 2, 3, 4, 5]);
    for _ in 0..8 {
        runtime.step(Duration::from_millis(1));
    }

    let mut forwarded: Vec<u64> = Vec::new();
    cmd_sub
        .try_receive(|msg| {
            let x = f64::from_le_bytes(msg.payload()[0..8].try_into().unwrap());
            forwarded.push(x as u64);
        })
        .expect("drain cmd stream");
    assert_eq!(
        forwarded,
        vec![1, 2, 3, 4, 5],
        "the dlopen'd macro node must forward EVERY burst frame, in order — \
         the per-message FIFO contract on the production FFI path (pre-FIFO \
         it forwarded only [5])"
    );
}

/// A deferred fire must not lose the
/// frozen FIFO head. The unified boundary drain pops+freezes one frame per
/// step; when the scheduler DEFERS the fire (here: `throttle_ms` on the
/// data-trigger consumer), the next boundary must NOT pop the next frame
/// over the unserved head — every frame still reaches its own fire, in
/// order. Consumer throttled to one fire per 4 ms against a burst of 6:
/// contiguity is the oracle.
#[test]
#[serial]
fn a_throttled_data_trigger_consumer_loses_no_frames() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let seen_c = Arc::clone(&seen);
    let consumer_info = NodeInfo::with_meta(
        vec![InputMeta {
            name: "inp".to_string(),
            schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
            trigger: true,
            depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
            backpressure: BackpressurePolicy::DropOldest,
            expect_within_ms: None,
        }],
        vec![],
    )
    .with_policy(MacroPolicy::DataTrigger {
        input_name: "inp".to_string(),
    })
    .with_throttle_ms(4);
    let consumer = ClosureNodeEntry::new(consumer_info, move |ctx| {
        if let Some(s) = ctx.subscriber_mut("inp") {
            if let Ok(Some(v)) = s.try_view::<Vector3, _>(|view| view.x as u64) {
                seen_c.lock().unwrap().push(v);
            }
        }
        Ok(())
    })
    .with_label("throttled_fifo_consumer");

    let config = GraphConfig {
        execution: None,
        name: None,
        identity: "fifo_throttle".to_string(),
        prefix: "fifothr".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "sink".to_string(),
            node_type: "throttled_fifo_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: "/fifo/ext".to_string(),
            }],
            outputs: vec![],
        }],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("sink".to_string(), Box::new(consumer));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build throttle graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher("/fifo/ext", MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");

    publish_all(&mut ext, [1, 2, 3, 4, 5, 6]);
    // 6 frames x one fire per 4 ms = at least 24 ms of stepping + slack.
    for _ in 0..40 {
        runtime.step(Duration::from_millis(1));
    }

    assert_eq!(
        *seen.lock().unwrap(),
        vec![1, 2, 3, 4, 5, 6],
        "a deferred fire must not overwrite the unserved FIFO head — every \
         frame reaches its own (throttled) fire, in order"
    );
}

/// A fire that runs but never reaches this
/// input's `try_view` must not wedge the node. The macro's generated tick
/// nests one `try_view` per input in DECLARATION order, so a non-trigger
/// context input declared BEFORE the trigger collapses the chain while the
/// context has no delivery yet (the pre-first-delivery WAIT) — the
/// trigger's read never runs, and a guard that held the head WITHOUT
/// re-offering it wedged the node permanently (no signal ⇒ no fire ⇒ no
/// read ⇒ no signal; reproduced: 0 frames ever served). Modeled with a
/// closure whose tick returns BEFORE the trigger read until the context
/// flag flips — the exact collapsed-chain shape. Once the context arrives,
/// EVERY queued trigger frame must still be served, in order.
#[test]
#[serial]
fn a_collapsed_tick_does_not_wedge_the_input_and_the_backlog_survives() {
    use std::sync::atomic::AtomicBool;

    let seen = Arc::new(Mutex::new(Vec::new()));
    let seen_c = Arc::clone(&seen);
    let context_ready = Arc::new(AtomicBool::new(false));
    let context_ready_c = Arc::clone(&context_ready);
    let consumer_info = NodeInfo::with_meta(
        vec![InputMeta {
            name: "inp".to_string(),
            schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
            trigger: true,
            depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
            backpressure: BackpressurePolicy::DropOldest,
            expect_within_ms: None,
        }],
        vec![],
    )
    .with_policy(MacroPolicy::DataTrigger {
        input_name: "inp".to_string(),
    });
    let consumer = ClosureNodeEntry::new(consumer_info, move |ctx| {
        // The collapsed-chain shape: while the "context" is absent, the
        // tick returns WITHOUT reading the trigger input — exactly what
        // the macro's nested try_view chain does when an earlier
        // context input yields Ok(None).
        if !context_ready_c.load(Ordering::Relaxed) {
            return Ok(());
        }
        if let Some(s) = ctx.subscriber_mut("inp") {
            if let Ok(Some(v)) = s.try_view::<Vector3, _>(|view| view.x as u64) {
                seen_c.lock().unwrap().push(v);
            }
        }
        Ok(())
    })
    .with_label("collapsing_fifo_consumer");

    let config = GraphConfig {
        execution: None,
        name: None,
        identity: "fifo_collapse".to_string(),
        prefix: "fifocol".to_string(),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "sink".to_string(),
            node_type: "collapsing_fifo_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: "/fifo/ext".to_string(),
            }],
            outputs: vec![],
        }],
        multi_publisher_topics: Vec::new(),
        process_groups: Default::default(),
        process_group_order: Default::default(),
        level_assignments: None,
        network: None,
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("sink".to_string(), Box::new(consumer));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build collapse graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher("/fifo/ext", MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");

    // Frames arrive while the context is ABSENT: fires run, ticks collapse,
    // nothing is served — and nothing may be lost or wedged.
    publish_all(&mut ext, [1, 2, 3]);
    for _ in 0..6 {
        runtime.step(Duration::from_millis(1));
    }
    assert!(
        seen.lock().unwrap().is_empty(),
        "while the context is absent the collapsed tick serves nothing"
    );

    // Context arrives; more frames follow. EVERYTHING queued must now flow,
    // in order — a wedged input would serve nothing forever.
    context_ready.store(true, Ordering::Relaxed);
    publish_all(&mut ext, [4, 5]);
    for _ in 0..12 {
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(
        *seen.lock().unwrap(),
        vec![1, 2, 3, 4, 5],
        "after the context arrives, the held head and the queued backlog \
         are all served in FIFO order — the collapsed-tick stretch loses \
         nothing and wedges nothing"
    );
}

/// On the Separate discipline the
/// trigger-drain queue must not retain — and signal — more frames than the
/// declared input depth can serve. A `.max(sub_buf)` floor
/// ("observability generosity — drain depth never gates fires") would mint
/// phantom fires once signals become fires: a depth-1 input under a 5-frame
/// burst would signal 5 fires for 1 servable frame. The drain queue is
/// sized to the declared depth, so the fire count tracks servable frames:
/// depth-1 + burst-5 ⇒ exactly ONE fire serving the newest frame, with the
/// body queue's 4 evictions counted (the loss is the user's own depth-1
/// declaration — identical pre-FIFO — never a FIFO regression).
#[test]
#[serial]
fn a_shallow_separate_input_mints_no_phantom_fires_for_evicted_frames() {
    let _guard = EnvVarGuard::set("CERULION_DRAIN_DISCIPLINE", "separate");
    let seen = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = burst_consumer_graph(
        "fifoshal",
        1,
        BackpressurePolicy::DropOldest,
        Arc::clone(&seen),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 16).expect("build shallow graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher("/fifo/ext", MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");

    // Baseline first: the eviction detector counts wire-sequence gaps per
    // stream, and a stream's FIRST observation baseline-establishes
    // uncounted (prior history is unknowable — the documented under-report
    // direction). Serve one frame so the burst's evictions are countable.
    publish_all(&mut ext, [100]);
    runtime.step(Duration::from_millis(1));

    publish_all(&mut ext, [1, 2, 3, 4, 5]);
    for _ in 0..10 {
        runtime.step(Duration::from_millis(1));
    }

    assert_eq!(
        *seen.lock().unwrap(),
        vec![100, 5],
        "a depth-1 queue retains only the newest burst frame; baseline + \
         that frame are served"
    );
    let handle = runtime.node_handle("sink").unwrap();
    assert_eq!(
        handle.fire_count(),
        2,
        "two servable frames ⇒ exactly two fires — a drain queue deeper \
         than the body queue minted one fire per BURST frame here (4 \
         phantoms reading nothing)"
    );
    assert_eq!(
        handle.backpressure_drop_oldest_count("inp"),
        4,
        "the four evicted burst frames are counted — the loss is declared \
         (depth = 1), never silent"
    );
}
