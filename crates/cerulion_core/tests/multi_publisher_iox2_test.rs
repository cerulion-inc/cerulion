// SPDX-License-Identifier: AGPL-3.0-only
//! `multi_publisher_topics` opt-in, end-to-end over real
//! iceoryx2.
//!
//! A listed ABSOLUTE topic with in-graph producer(s) provisions the shared
//! loose publisher cap (`MULTI_PUBLISHER_LOOSE_MAX`) instead of
//! single-writer: cross-graph publishers attach by requirement equality,
//! `block` defers EVERY in-graph producer ("block all"), the
//! per-stream eviction counting sizes its baselines
//! from the live loose cap, and the live-truth event-cap formula
//! tracks it with no further changes (verified here with pins).
//!
//! Parallel-safe: each test generates its own isolated iceoryx2 config.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{ClosureNodeEntry, InputMeta, MacroPolicy, NodeEntry, NodeInfo};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::message::ShmMessage;
use cerulion_core::prelude::*;
use cerulion_core::transport::{
    TransportConfig, TransportManager, MULTI_PUBLISHER_LOOSE_MAX, MULTI_SUBSCRIBER_LOOSE_MAX,
};
use cerulion_core::wire::MaxSliceLen;
use iceoryx2::service::port_factory::PortFactory as _;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// Period producer publishing a fixed marker value every tick.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct MarkerProducer {
    #[output]
    out: Vector3,
    marker: f64,
}

#[cerulion_node_impl]
impl MarkerProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.marker;
        Ok(())
    }
}

/// One "graph process": its own manager (iceoryx2 node) over the shared root.
fn manager(name: &str, ix: iceoryx2::config::Config) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: name.into(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 8,
            network: None,
        },
        ix,
    )
    .expect("init isolated transport manager")
}

fn producer_only_graph(
    prefix: &str,
    node_id: &str,
    marker: f64,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: vec!["/tf".to_string()],
        name: None,
        identity: format!("{prefix}_{node_id}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: node_id.to_string(),
            node_type: "marker_producer".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: Some("/tf".to_string()),
            }],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        node_id.to_string(),
        Box::new(MarkerProducerEntry::with_state(MarkerProducer {
            marker,
            ..Default::default()
        })),
    );
    (config, factories)
}

#[test]
fn cross_graph_listed_topic_both_publish_and_flow() {
    // THE headline /tf use case: two graphs each own
    // a /tf broadcaster, both list /tf — both must BUILD (the
    // single-writer enforcement is off for listed topics: Multi
    // provisioning skips the active-publisher pre-check AND provisions
    // the loose cap, so B's open requires the same constant A created
    // with), and data from BOTH reaches a consumer. (Mutation oracles:
    // Multi → SingleWriter in the provisioning decision makes graph B's
    // build die at the pre-check; Multi mapping to Some(1) makes B's
    // open verification fail.)
    //
    // `/tf` IS AN ACCUMULATE-ALL TOPIC. Like ROS2 tf2 — which inserts
    // EVERY transform from EVERY broadcaster into a time-indexed buffer —
    // every frame matters, not just the latest. So the listener here is a
    // DRAIN-ALL consumer: each tick it drains ALL queued /tf frames via
    // `ctx.subscriber("inp").try_receive(...)` (which removes every sample
    // from the body queue) and tallies pos (x>0) / neg (x<0) for EACH
    // drained frame — the correct tf model, vs a latest-wins macro
    // consumer (`try_view`/`#[input(trigger)]` field reads keep only the
    // newest frame and would structurally hide a same-step sibling).
    //
    // The listener is a data-TRIGGER consumer of /tf (`MacroPolicy::
    // DataTrigger { input_name: "inp" }`), so the trigger-aware DAG levels
    // place it at LEVEL 1 — DOWNSTREAM of its in-graph /tf
    // producer `bc_b` (level 0). The level executor therefore CORRECTLY
    // fires bc_b BEFORE the listener within a step (the same-tick
    // collapse). A latest-wins listener could only catch graph A's
    // cross-graph frame by being inserted BEFORE bc_b, so insertion-order
    // firing read it before bc_b's same-step frame shadowed it — a
    // trick that is incompatible with the level executor.
    // With a DRAIN-ALL listener no ordering trick
    // is needed: bc_b firing first is fine, because the listener still
    // sees A's cross-graph frame ALONGSIDE bc_b's same-step frame when it
    // drains every queued sample.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr_a = manager("multi_a", ix.clone());
    let mgr_b = manager("multi_b", ix.clone());

    let (cfg_a, fac_a) = producer_only_graph("ga", "bc_a", 1.0);
    let mut rt_a = GraphRuntime::build(cfg_a, fac_a, &mgr_a, Arc::new(VirtualClock::new()))
        .expect("graph A (listed /tf) must build");

    // Graph B: its own /tf broadcaster (negative marker) + a drain-all
    // /tf consumer. Insertion order does not matter (the level executor
    // orders bc_b before its level-1 consumer regardless); list bc_b first
    // for clarity.
    let pos = Arc::new(AtomicU64::new(0));
    let neg = Arc::new(AtomicU64::new(0));
    let cfg_b = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: vec!["/tf".to_string()],
        name: None,
        identity: "gb".to_string(),
        prefix: "gb".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "bc_b".to_string(),
                node_type: "marker_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: Some("/tf".to_string()),
                }],
            },
            NodeDef {
                ros2: None,
                id: "listener".to_string(),
                node_type: "drain_all_tf_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "/tf".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut fac_b: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    fac_b.insert(
        "bc_b".to_string(),
        Box::new(MarkerProducerEntry::with_state(MarkerProducer {
            marker: -1.0,
            ..Default::default()
        })),
    );
    // The drain-all /tf listener. A data-TRIGGER consumer of /tf (so it
    // fires when /tf data arrives and lands at level 1, after bc_b), with
    // a default DropOldest depth-`DEFAULT_CONSUMER_DEPTH` body queue (the
    // same shape a `#[input(trigger)]` macro field generates) so a tick
    // can drain multiple frames at once. Its tick drains EVERY queued
    // sample via
    // `try_receive` and tallies each frame's marker sign — accumulate-all,
    // not latest-wins.
    let pos_c = Arc::clone(&pos);
    let neg_c = Arc::clone(&neg);
    let listener_info = NodeInfo::with_meta(
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
    let listener = ClosureNodeEntry::new(listener_info, move |ctx| {
        let sub = ctx
            .subscriber("inp")
            .expect("listener subscriber 'inp' must be wired");
        // Drain ALL queued /tf frames this tick (accumulate-all tf model),
        // tallying each frame's marker sign. `payload()` is the Vector3
        // fixed section (x,y,z f64); x = marker at [0..8], little-endian.
        let _drained = sub.try_receive(|msg| {
            let x = f64::from_le_bytes(msg.payload()[0..8].try_into().unwrap());
            if x > 0.0 {
                pos_c.fetch_add(1, Ordering::Relaxed);
            } else if x < 0.0 {
                neg_c.fetch_add(1, Ordering::Relaxed);
            }
        })?;
        Ok(())
    })
    .with_label("drain_all_tf_consumer")
    // OPT OUT of the unified trigger drain. This tick reads
    // accumulate-all via `try_receive`, which the unified drain's frozen slot
    // does NOT serve (frozen serves `try_view` only, latest-wins) — unified,
    // every /tf frame would be silently lost. Keep `DrainSource::Separate`.
    .with_unified_drain(false);
    fac_b.insert("listener".to_string(), Box::new(listener));
    let mut rt_b = GraphRuntime::build(cfg_b, fac_b, &mgr_b, Arc::new(VirtualClock::new()))
        .expect("graph B (listed /tf, second publisher) must build");

    // Interleave: both producers publish, B's drain-all consumer sees BOTH
    // markers. bc_b (level 0) fires before the listener (level 1) within
    // rt_b.step, and the listener drains A's cross-graph frame ALONGSIDE
    // bc_b's same-step frame — so both tallies fire.
    for _ in 0..6 {
        rt_a.step(Duration::from_millis(1));
        rt_b.step(Duration::from_millis(1));
    }
    assert!(
        pos.load(Ordering::Relaxed) >= 1,
        "graph A's marker must reach the consumer (pos={})",
        pos.load(Ordering::Relaxed)
    );
    assert!(
        neg.load(Ordering::Relaxed) >= 1,
        "graph B's own marker must reach the consumer (neg={})",
        neg.load(Ordering::Relaxed)
    );
}

// ===========================================================================
// ALL-BLOCK `multi_publisher_topics` over-publish at outstanding ==
// depth-1.
//
// Two same-level in-graph producers of one ALL-`block` listed topic share ONE
// `outstanding` mirror (one shared counter per consumer edge). A
// level executor that ran DECIDE-all then TICK-all would have BOTH
// producers' `decide_node` read `outstanding` at the LEVEL BOUNDARY — before
// either published (publishes happen in tick). So at `outstanding == depth-1`
// both producers would decide to fire, both publish, and `outstanding` overshoot
// to `depth+1` — violating block's "never publish past the threshold"
// contract (and risking iceoryx2 eviction → data loss on a
// topic the user declared lossless).
//
// The executor routes the block-involved subset through the FUSED decide+tick seam
// (`Scheduler::evaluate_nodes_fused`) in graph order: producer A ticks
// (PUBLISHES → `outstanding` bumps) BEFORE producer B's `decide_node` reads
// the mirror, so B defers correctly — `outstanding` never exceeds `depth`.
//
// This test ENGINEERS the `outstanding == depth-1` state (which the existing
// `block_defers_all_in_graph_producers_on_listed_topic` test does NOT — it
// starts at outstanding=0, where both producers fire to exactly depth=2 with no
// bug) using EXTERNAL producers driven by hand:
//   - depth=2, two external producers A/B of the listed /tf, one full-drain
//     block consumer (also external; only ticks when triggered).
//   - "Prime" step: trigger ONLY A (consumer NOT triggered) → A publishes,
//     outstanding 0→1 == depth-1.
//   - "Race" step: trigger BOTH A and B (consumer NOT triggered).
//       DECIDE-all then TICK-all: both decide at outstanding=1<2 → both publish → outstanding
//                 → 3 == depth+1 (OVER-PUBLISH past the block threshold).
//       FUSED decide+tick: A ticks (1→2), B sees 2 == depth → defers (B's external
//                 trigger is NOT consumed — the block pre-fire check returns
//                 before the External arm — so B retries on a later step).
//   - "Drain" step: trigger ONLY the consumer → full-drain, which reveals how
//     many frames were IN FLIGHT (the shared `outstanding`) at the end of the
//     race step: that batch size IS the high-water `outstanding`.
//   - Repeat the prime/race/drain cycle several times.
//
// PRIMARY ORACLE (the contract — "no over-publish past the
// block threshold"): the consumer's per-cycle full-drain batch — the number of
// frames in flight at the race-step boundary — must NEVER exceed `depth`.
// WITHOUT the fused routing the depth-1 race over-publishes to depth+1, so a post-race
// drain pulls `depth+1` frames → the assert FAILS. WITH the fused routing the second
// producer defers, so at most `depth` are ever in flight.
//
// NOTE on why eviction itself is NOT the oracle: iceoryx2 queues are
// per-(publisher, subscriber) CONNECTION, so a 2-publisher edge has `depth`
// slots PER connection (the shared `outstanding` total is gated CONSERVATIVELY
// at the declared depth — see the build wiring comment in graph/runtime.rs).
// A single +1 overshoot of the shared total therefore lands within the
// combined per-connection headroom and does NOT necessarily evict — so the
// robust, deterministic oracle is the over-publish of the shared mirror
// itself (drain batch > depth), exactly that contract. A
// secondary per-stream contiguity check guards the no-loss invariant too.
// ===========================================================================

/// External-triggered producer that writes its STREAM id into `out.x` and a
/// per-stream incrementing sequence (1, 2, 3, ...) into `out.y`. External so
/// the test fires it exactly when it wants (a deferred fire is NOT lost — the
/// block pre-fire check returns before the External arm consumes the trigger,
/// so the trigger persists and the producer retries on a later step).
#[cerulion_node(external)]
#[derive(Default)]
struct SeqProducer {
    #[output]
    out: Vector3,
    stream_id: f64,
    seq: f64,
}

#[cerulion_node_impl]
impl SeqProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seq += 1.0;
        self.out.x = self.stream_id;
        self.out.y = self.seq;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// Decode (stream_id, seq) from a drained Vector3 body. `try_receive` strips
/// the 32-byte `WireHeader`, so `payload` is the Vector3 fixed section
/// (x,y,z f64); x = stream id at [0..8], y = seq at [8..16], little-endian.
fn decode_stream_seq(payload: &[u8]) -> (u32, u32) {
    let x = f64::from_le_bytes(payload[0..8].try_into().unwrap());
    let y = f64::from_le_bytes(payload[8..16].try_into().unwrap());
    (x as u32, y as u32)
}

/// Build the block `InputMeta` for the full-drain consumer's `inp` port on the
/// listed /tf, depth 2.
fn block_input_meta() -> InputMeta {
    InputMeta {
        name: "inp".to_string(),
        schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
        trigger: false,
        depth: 2,
        backpressure: BackpressurePolicy::Block,
        expect_within_ms: None,
    }
}

/// Per-run observations the consumer records: the flat arrival sequence
/// (`seen`, as (stream_id, seq)) for the contiguity oracle, and the size of
/// EACH non-empty full-drain batch (`batch_sizes`) for the over-publish oracle
/// (a batch is the shared `outstanding` high-water at that drain).
#[derive(Clone, Default)]
struct BlockAllObs {
    seen: Arc<Mutex<Vec<(u32, u32)>>>,
    batch_sizes: Arc<Mutex<Vec<usize>>>,
}

/// Build the ALL-block listed-/tf graph: two external `SeqProducer`s (streams
/// 1 and 2) + one external full-drain block consumer. The consumer is a
/// hand-rolled `ClosureNodeEntry` keeping its subscriber in the ctx so it can
/// full-drain (`try_receive` delivers EVERY queued sample, decrementing the
/// block mirror per removal) — the macro input path auto-drains latest-wins and
/// takes the subscriber, which would mask a gap.
fn block_all_graph(obs: BlockAllObs) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: vec!["/tf".to_string()],
        name: None,
        identity: "block_all".to_string(),
        prefix: "dc".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "prod_a".to_string(),
                node_type: "seq_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: Some("/tf".to_string()),
                }],
            },
            NodeDef {
                ros2: None,
                id: "prod_b".to_string(),
                node_type: "seq_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: Some("/tf".to_string()),
                }],
            },
            NodeDef {
                ros2: None,
                id: "consumer".to_string(),
                node_type: "draining_block_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "/tf".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "prod_a".to_string(),
        Box::new(SeqProducerEntry::with_state(SeqProducer {
            stream_id: 1.0,
            ..Default::default()
        })),
    );
    factories.insert(
        "prod_b".to_string(),
        Box::new(SeqProducerEntry::with_state(SeqProducer {
            stream_id: 2.0,
            ..Default::default()
        })),
    );
    // External full-drain consumer: only ticks when the test triggers it.
    let info =
        NodeInfo::with_meta(vec![block_input_meta()], vec![]).with_policy(MacroPolicy::External);
    let consumer = ClosureNodeEntry::new(info, move |ctx| {
        let sub = ctx
            .subscriber("inp")
            .expect("consumer subscriber 'inp' must be wired");
        let mut batch: Vec<(u32, u32)> = Vec::new();
        let _drained = sub.try_receive(|msg| {
            batch.push(decode_stream_seq(msg.payload()));
        })?;
        if !batch.is_empty() {
            // Record the batch SIZE (= shared `outstanding` high-water at this
            // drain) for the over-publish oracle, then the values for the
            // contiguity oracle.
            obs.batch_sizes.lock().unwrap().push(batch.len());
            obs.seen.lock().unwrap().extend(batch);
        }
        Ok(())
    })
    .with_label("draining_block_consumer");
    factories.insert("consumer".to_string(), Box::new(consumer));
    (config, factories)
}

/// The block input's declared depth — both the iceoryx2 queue and the shared
/// `outstanding` defer threshold.
const BLOCK_DEPTH: usize = 2;

/// Drive the prime/race/drain cycle and return the consumer's observations
/// (flat arrival sequence + per-drain batch sizes).
fn run_block_all() -> BlockAllObs {
    let obs = BlockAllObs::default();
    let (config, factories) = block_all_graph(obs.clone());
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build block-all graph");

    // Several prime/race/drain cycles. Each cycle:
    //   prime: trigger A only (outstanding 0→1 == depth-1).
    //   race:  trigger A AND B (the depth-1 window). Consumer NOT triggered,
    //          so `outstanding` carries the prime value into the race decide.
    //   drain: trigger consumer only (full-drain). The drain batch size IS the
    //          shared `outstanding` high-water at the end of the race step.
    for _ in 0..6 {
        // prime
        runtime.trigger_external("prod_a").unwrap();
        runtime.step(Duration::from_millis(1));
        // race — BOTH producers eligible at outstanding == depth-1
        runtime.trigger_external("prod_a").unwrap();
        runtime.trigger_external("prod_b").unwrap();
        runtime.step(Duration::from_millis(1));
        // drain
        runtime.trigger_external("consumer").unwrap();
        runtime.step(Duration::from_millis(1));
    }
    // Final flush: drain any deferred producer retries so the contiguity oracle
    // sees a complete prefix of each stream. (These drains start from
    // outstanding ≤ depth, so they do not perturb the over-publish oracle.)
    for _ in 0..4 {
        runtime.trigger_external("prod_a").unwrap();
        runtime.trigger_external("prod_b").unwrap();
        runtime.step(Duration::from_millis(1));
        runtime.trigger_external("consumer").unwrap();
        runtime.step(Duration::from_millis(1));
    }
    drop(runtime);
    obs
}

#[test]
#[serial]
fn block_all_no_over_publish_at_depth_minus_one() {
    let obs = run_block_all();
    let seen = obs.seen.lock().unwrap().clone();
    let batch_sizes = obs.batch_sizes.lock().unwrap().clone();
    assert!(!seen.is_empty(), "consumer must have received data");
    assert!(
        !batch_sizes.is_empty(),
        "consumer must have drained at least once"
    );

    // PRIMARY ORACLE (no over-publish past the block threshold): the shared `outstanding`
    // mirror never exceeds `depth`. Each non-empty full-drain pulls every
    // in-flight frame, so the batch size IS `outstanding` at that drain. The
    // post-race drains land at the depth-1 race boundary, so without the fused
    // routing the second producer over-publishes and a drain pulls
    // `depth + 1` (3) frames → this FAILS. With the fused routing the second producer
    // defers, so no drain ever exceeds `depth`.
    let max_batch = *batch_sizes.iter().max().unwrap();
    assert!(
        max_batch <= BLOCK_DEPTH,
        "block over-publish: a drain pulled {max_batch} in-flight frames but the declared \
         depth is {BLOCK_DEPTH} — the shared `outstanding` mirror exceeded the block \
         threshold (the depth-1 race). batches={batch_sizes:?}"
    );

    // SECONDARY ORACLE: per-stream contiguity (no data loss). Split the
    // interleaved arrival stream by stream_id and assert each producer's seq is
    // 1,2,3,... with NO gap. A gap would mean a lost frame on this lossless
    // block topic.
    for stream in [1u32, 2u32] {
        let seqs: Vec<u32> = seen
            .iter()
            .filter(|(s, _)| *s == stream)
            .map(|(_, seq)| *seq)
            .collect();
        assert!(
            !seqs.is_empty(),
            "stream {stream} must deliver at least one value (got {seen:?})"
        );
        assert_eq!(
            seqs[0], 1,
            "stream {stream}'s first delivered value must be seq 1 — a missing head means an \
             early loss. Got {seqs:?} (full: {seen:?})"
        );
        for w in seqs.windows(2) {
            assert_eq!(
                w[1],
                w[0] + 1,
                "stream {stream} values must be CONTIGUOUS (no gap) — a gap means a frame was \
                 lost on this lossless block topic. Got {seqs:?} (full: {seen:?})"
            );
        }
    }
}

#[test]
#[serial]
fn block_all_no_over_publish_is_deterministic() {
    // Determinism: two identical hand-driven runs produce the SAME observed
    // per-stream arrival sequence AND the same per-drain batch sizes
    // (VirtualClock + deterministic trigger order + fused graph-order routing).
    let obs1 = run_block_all();
    let obs2 = run_block_all();
    assert_eq!(
        *obs1.seen.lock().unwrap(),
        *obs2.seen.lock().unwrap(),
        "the block-all arrival sequence must be deterministic across runs"
    );
    assert_eq!(
        *obs1.batch_sizes.lock().unwrap(),
        *obs2.batch_sizes.lock().unwrap(),
        "the block-all drain batch sizes must be deterministic across runs"
    );
}

/// Stalled block consumer (external-triggered, never fired): its queue
/// fills and the producers must defer.
#[cerulion_node(external)]
#[derive(Default)]
struct StalledBlockConsumer {
    #[input(backpressure = block, depth = 2)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl StalledBlockConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn block_defers_all_in_graph_producers_on_listed_topic() {
    // "Block all": ONE graph, TWO producers of
    // the listed /tf, one stalled all-block consumer (depth 2). The
    // shared outstanding counter gates BOTH producers: A fires once
    // (outstanding 1), B fires once (outstanding 2 = threshold), then
    // EVERY subsequent fire of EITHER producer is deferred — total
    // publishes across both producers == depth, no data loss. (Mutation
    // oracle: fanning the defer edge out to only the FIRST producer
    // leaves B firing every step — fires_b grows past 1 and the
    // equality fails.)
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: vec!["/tf".to_string()],
        name: None,
        identity: "block_all".to_string(),
        prefix: "ba".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "prod_a".to_string(),
                node_type: "marker_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: Some("/tf".to_string()),
                }],
            },
            NodeDef {
                ros2: None,
                id: "prod_b".to_string(),
                node_type: "marker_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: Some("/tf".to_string()),
                }],
            },
            NodeDef {
                ros2: None,
                id: "stalled".to_string(),
                node_type: "stalled_block_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "/tf".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "prod_a".to_string(),
        Box::new(MarkerProducerEntry::with_state(MarkerProducer {
            marker: 1.0,
            ..Default::default()
        })),
    );
    factories.insert(
        "prod_b".to_string(),
        Box::new(MarkerProducerEntry::with_state(MarkerProducer {
            marker: 2.0,
            ..Default::default()
        })),
    );
    factories.insert(
        "stalled".to_string(),
        Box::new(StalledBlockConsumerEntry::new()),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build block-all graph");
    for _ in 0..10 {
        runtime.step(Duration::from_millis(1));
    }
    let fires_a = runtime.node_handle("prod_a").unwrap().fire_count();
    let fires_b = runtime.node_handle("prod_b").unwrap().fire_count();
    let deferred = runtime
        .node_handle("stalled")
        .unwrap()
        .backpressure_block_fires_deferred_count("inp");
    assert_eq!(
        fires_a + fires_b,
        2,
        "total publishes must equal the consumer depth (2): the shared \
         outstanding counter gates BOTH producers (a={fires_a}, b={fires_b})"
    );
    assert_eq!(
        fires_b, 1,
        "producer B must be deferred too (block-all, not block-first)"
    );
    assert!(
        deferred >= 2,
        "defers must be counted for both producers (got {deferred})"
    );
}

/// External-triggered producer: never fired, exists so the listed topic
/// has an in-graph producer (Multi provisioning) while staying quiet.
#[cerulion_node(external)]
#[derive(Default)]
struct QuietProducer {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl QuietProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 0.0;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// Data-trigger drop_oldest consumer with a tiny queue, for the
/// per-stream eviction pin.
#[cerulion_node]
#[derive(Default)]
struct TinyQueueConsumer {
    #[input(trigger, backpressure = drop_oldest, depth = 2)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl TinyQueueConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

#[test]
fn per_stream_eviction_counting_tracks_loose_cap() {
    // Verification pin: the per-id eviction baselines
    // size from the topic's LIVE max_publishers — here the loose cap (16)
    // — so THREE concurrent streams (the quiet in-graph producer + two
    // external bursters) are all tracked and evictions count EXACTLY per
    // stream. (A single-writer graph topic carries max_publishers = 1:
    // one baseline slot — the second external stream would displace the
    // first and under-report. Mutation oracle: Multi mapping to Some(1)
    // also kills the external attaches outright.)
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: vec!["/tf".to_string()],
        name: None,
        identity: "evict".to_string(),
        prefix: "ev".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "quiet".to_string(),
                node_type: "quiet_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: Some("/tf".to_string()),
                }],
            },
            NodeDef {
                ros2: None,
                id: "sink".to_string(),
                node_type: "tiny_queue_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "/tf".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("quiet".to_string(), Box::new(QuietProducerEntry::new()));
    factories.insert("sink".to_string(), Box::new(TinyQueueConsumerEntry::new()));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build eviction graph");
    let mgr = runtime.test_transport().expect("test transport parked");

    // Two EXTERNAL publishers attach to the listed topic (loose cap
    // admits them; the graph's quiet producer holds slot 1).
    let mut pub_b = mgr
        .create_publisher("/tf", MaxSliceLen::const_new(256), 0)
        .expect("external publisher B attaches to the listed topic");
    let mut pub_c = mgr
        .create_publisher("/tf", MaxSliceLen::const_new(256), 0)
        .expect("external publisher C attaches to the listed topic");

    let publish = |p: &mut cerulion_core::transport::publisher::CerulionPublisher, v: f64| {
        let mut proxy = p.loan_proxy::<Vector3>().expect("loan");
        proxy.x = v;
        drop(proxy);
    };

    // Warm-up: one frame per stream so the consumer BASELINES both
    // streams (a stream's first-seen frame establishes uncounted —
    // evictions are only exact AFTER the baseline). Per-message FIFO
    // consumption pops ONE frame per step, so TWO steps pop both warm-up
    // frames (one per stream) — each stream's baseline needs its frame
    // actually served.
    publish(&mut pub_b, 1.0);
    publish(&mut pub_c, 2.0);
    runtime.step(Duration::from_millis(1));
    runtime.step(Duration::from_millis(1));

    // Burst without draining: B publishes 6 (depth-2 connection queue
    // keeps the newest 2 → 4 evicted), C publishes 4 (→ 2 evicted).
    for i in 0..6 {
        publish(&mut pub_b, 10.0 + f64::from(i));
    }
    for i in 0..4 {
        publish(&mut pub_c, 20.0 + f64::from(i));
    }
    // FIFO pop-one serves the 4 retained frames (2 per stream) across 4
    // steps; each stream's FIRST served burst frame carries that stream's
    // wire-sequence gap vs its baseline, so the exact per-stream eviction
    // totals accrue as the retained frames are consumed.
    for _ in 0..4 {
        runtime.step(Duration::from_millis(1));
    }

    let evicted = runtime
        .node_handle("sink")
        .unwrap()
        .backpressure_drop_oldest_count("inp");
    assert_eq!(
        evicted, 6,
        "per-stream EXACT counting across both external streams: \
         (6-2) + (4-2) = 6 evictions"
    );
}

#[test]
fn event_caps_track_loose_cap_via_live_truth_formula() {
    // Verification pin: the event-cap formula reads
    // the LIVE data service's subscriber + publisher terms — with Multi
    // provisioning the publisher term becomes the loose cap, with no
    // formula change. Read both services raw and assert the tracking.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mgr = manager("event_caps_multi", ix.clone());
    let (cfg, fac) = producer_only_graph("ec", "bc", 1.0);
    let _rt = GraphRuntime::build(cfg, fac, &mgr, Arc::new(VirtualClock::new()))
        .expect("build listed graph");

    let raw_node = iceoryx2::node::NodeBuilder::new()
        .config(&ix)
        .create::<iceoryx2::service::ipc::Service>()
        .expect("raw iceoryx2 node");
    let data_name: iceoryx2::service::service_name::ServiceName =
        "/tf/data".try_into().expect("name");
    let data = raw_node
        .service_builder(&data_name)
        .publish_subscribe::<[u8]>()
        .open()
        .expect("open the data service raw");
    let event_name: iceoryx2::service::service_name::ServiceName =
        "/tf/event".try_into().expect("name");
    let event = raw_node
        .service_builder(&event_name)
        .event()
        .open()
        .expect("open the event service raw");

    assert_eq!(
        data.static_config().max_publishers(),
        MULTI_PUBLISHER_LOOSE_MAX,
        "the listed topic must be provisioned at the loose cap"
    );
    let expected_event_ports =
        data.static_config().max_subscribers() + data.static_config().max_publishers();
    assert_eq!(
        event.static_config().max_listeners(),
        expected_event_ports,
        "event listener caps must track live subs + live pubs (loose cap included)"
    );
    assert_eq!(
        event.static_config().max_notifiers(),
        expected_event_ports,
        "event notifier caps must track live subs + live pubs (loose cap included)"
    );
}

/// Deep consumer (depth 32 > the shared Multi ceiling 16).
#[cerulion_node]
#[derive(Default)]
struct DeepConsumer {
    #[input(trigger, backpressure = drop_oldest, depth = 32)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl DeepConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

#[test]
fn listed_topic_rejects_depth_above_shared_ceiling_at_build() {
    // Fail-fast (buffer axis): the Multi ceiling is a shared
    // graph-independent constant, so a deeper declared queue can never
    // be honored on a listed topic — the build must die with the
    // trade-off named, not at a confusing open-requirement error.
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: vec!["/tf".to_string()],
        name: None,
        identity: "deep".to_string(),
        prefix: "dp".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "quiet".to_string(),
                node_type: "quiet_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: Some("/tf".to_string()),
                }],
            },
            NodeDef {
                ros2: None,
                id: "deep".to_string(),
                node_type: "deep_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "/tf".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("quiet".to_string(), Box::new(QuietProducerEntry::new()));
    factories.insert("deep".to_string(), Box::new(DeepConsumerEntry::new()));
    let clock = Arc::new(VirtualClock::new());
    let msg = match GraphRuntime::build_for_test(config, factories, clock, 8) {
        Ok(_) => panic!("depth 32 on a listed topic must NOT build"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("buffer ceiling is 16") && msg.contains("multi-publisher"),
        "the rejection must name the shared ceiling + the trade-off: {msg}"
    );
}

#[test]
fn listed_topic_rejects_single_graph_subscriber_overflow_at_build() {
    // Fail-fast (subscriber axis): one graph alone needing more
    // subscriber slots than the shared cap must die at build with the
    // arithmetic named, not at port creation mid-build.
    //
    // These `tiny_queue_consumer`s are MACRO `drop_oldest`
    // data-trigger nodes, so they UNIFY onto their body subscriber — each
    // contributes ONE subscriber (body only), NOT the legacy two. So 13
    // consumers = 13 bodies = 13 in-graph subscribers, + the introspection
    // headroom = 17 > 16 must die at build (the counts below are DERIVED from
    // INTROSPECTION_SUBSCRIBER_HEADROOM, so this decomposition tracks it).
    // (Under a dual-subscriber count each consumer would take 2 slots and
    // fewer consumers would reach the same cap.) The standalone trigger
    // LISTENERs are provisioned on the EVENT service (`extra_event_listeners`),
    // not as subscriber slots, so they don't enter this subscriber-axis sum.
    let mut nodes = vec![NodeDef {
        ros2: None,
        id: "quiet".to_string(),
        node_type: "quiet_producer".to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: Some("/tf".to_string()),
        }],
    }];
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("quiet".to_string(), Box::new(QuietProducerEntry::new()));
    // Each unified consumer is 1 subscriber (body only), so
    // the smallest consumer count that overflows is (cap - headroom) + 1.
    let over_cap = (MULTI_SUBSCRIBER_LOOSE_MAX
        - cerulion_core::transport::INTROSPECTION_SUBSCRIBER_HEADROOM)
        + 1;
    for i in 0..over_cap {
        let id = format!("sink_{i}");
        nodes.push(NodeDef {
            ros2: None,
            id: id.clone(),
            node_type: "tiny_queue_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: "/tf".to_string(),
            }],
            outputs: vec![],
        });
        factories.insert(id, Box::new(TinyQueueConsumerEntry::new()));
    }
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: vec!["/tf".to_string()],
        name: None,
        identity: "fanout".to_string(),
        prefix: "fo".to_string(),
        nodes,
    };
    let clock = Arc::new(VirtualClock::new());
    // Every count derives from the two constants: a hard-coded literal here
    // would mis-state, as soon as the headroom changes, the arithmetic
    // the rejection is supposed to prove it got right.
    let needed = over_cap + cerulion_core::transport::INTROSPECTION_SUBSCRIBER_HEADROOM;
    let msg = match GraphRuntime::build_for_test(config, factories, clock, 8) {
        Ok(_) => panic!(
            "{needed} needed slots on a {MULTI_SUBSCRIBER_LOOSE_MAX}-cap listed \
             topic must NOT build"
        ),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains(&format!("needs {needed} subscriber slots"))
            && msg.contains(&format!("subscriber cap is {MULTI_SUBSCRIBER_LOOSE_MAX}"))
            && msg.contains(&format!("{over_cap} in-graph subscribers")),
        "the rejection must carry the exact arithmetic: {msg}"
    );
}

/// Data-trigger consumer at EXACTLY the shared Multi ceiling (depth 16).
#[cerulion_node]
#[derive(Default)]
struct BoundaryConsumer {
    #[input(trigger, backpressure = drop_oldest, depth = 16)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl BoundaryConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// A data-trigger consumer with `sample(N)` backpressure.
/// `sample(N)`'s read-decimation gates the FIRE rate, so the unification (the
/// body-subscriber collapse) is INELIGIBLE — this consumer keeps the
/// legacy dual-subscriber path and so still creates a separate trigger-drain
/// subscriber. It exists for `listed_topic_builds_when_transport_default_
/// exceeds_ceiling`, whose oracle is the trigger-DRAIN buffer clamp breadcrumb
/// (emitted only on the ineligible/dual-subscriber arm).
#[cerulion_node]
#[derive(Default)]
struct SampleTriggerConsumer {
    #[input(trigger, backpressure = sample(2), depth = 2)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl SampleTriggerConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

fn quiet_listed_graph_with(
    consumers: &[(&str, &str)],
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let mut nodes = vec![NodeDef {
        ros2: None,
        id: "quiet".to_string(),
        node_type: "quiet_producer".to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: Some("/tf".to_string()),
        }],
    }];
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("quiet".to_string(), Box::new(QuietProducerEntry::new()));
    for (id, node_type) in consumers {
        nodes.push(NodeDef {
            ros2: None,
            id: (*id).to_string(),
            node_type: (*node_type).to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: "/tf".to_string(),
            }],
            outputs: vec![],
        });
        let entry: Box<dyn NodeEntry> = match *node_type {
            "tiny_queue_consumer" => Box::new(TinyQueueConsumerEntry::new()),
            "boundary_consumer" => Box::new(BoundaryConsumerEntry::new()),
            "sample_trigger_consumer" => Box::new(SampleTriggerConsumerEntry::new()),
            other => panic!("unknown test node type {other}"),
        };
        factories.insert((*id).to_string(), entry);
    }
    (
        GraphConfig {
            level_assignments: None,
            network: None,
            process_groups: Default::default(),
            process_group_order: Default::default(),
            multi_publisher_topics: vec!["/tf".to_string()],
            name: None,
            identity: "boundary".to_string(),
            prefix: "bd".to_string(),
            nodes,
        },
        factories,
    )
}

#[test]
fn listed_topic_builds_at_exact_subscriber_cap() {
    // Boundary arm (catches a `>` where `>=` belongs in the subscriber
    // fail-fast, which a reject-side-only suite cannot see):
    // `at_cap` data-trigger consumers = that many provisioned subscriber
    // slots + INTROSPECTION_SUBSCRIBER_HEADROOM = exactly
    // MULTI_SUBSCRIBER_LOOSE_MAX (16) — must BUILD.
    // Derived from the constants (a hardcoded count silently stops
    // pinning the boundary if the headroom shrinks).
    // These `tiny_queue_consumer`s are MACRO `drop_oldest`
    // data-trigger nodes, so they UNIFY onto their body subscriber — each
    // PROVISIONS exactly ONE subscriber slot (body only). The standalone
    // trigger listener is provisioned on the EVENT service
    // (`extra_event_listeners`), not as a subscriber slot. So the boundary is
    // (cap - headroom) consumers, not (cap - headroom)/2 as in the legacy
    // dual-subscriber count.
    let at_cap =
        MULTI_SUBSCRIBER_LOOSE_MAX - cerulion_core::transport::INTROSPECTION_SUBSCRIBER_HEADROOM;
    let consumers: Vec<(String, &str)> = (0..at_cap)
        .map(|i| (format!("sink_{i}"), "tiny_queue_consumer"))
        .collect();
    let refs: Vec<(&str, &str)> = consumers.iter().map(|(id, t)| (id.as_str(), *t)).collect();
    let (config, factories) = quiet_listed_graph_with(&refs);
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("exactly-at-cap (16 == 16) must build");
}

#[test]
fn listed_topic_builds_at_exact_buffer_ceiling() {
    // The buffer-axis boundary twin: one consumer declaring depth 16
    // (== MULTI_TOPIC_BUFFER_CEILING) must BUILD.
    let (config, factories) = quiet_listed_graph_with(&[("deep16", "boundary_consumer")]);
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("depth == ceiling (16 == 16) must build");
}

#[test]
#[tracing_test::traced_test]
fn listed_topic_builds_when_transport_default_exceeds_ceiling() {
    // Regression pin: a trigger-drain buffer sized to
    // max(depth, transport default) would, with subscriber_buffer_size
    // 32 > the shared Multi ceiling 16, die mid-build at the create-site
    // guard with a generic at-a-distance error. Per-message FIFO carries
    // no `.max(sub_buf)` term at all (a drain queue deeper than the
    // body queue would mint phantom fires once signals became fires), so the
    // hazard's INPUT — a drain request derived from the transport default
    // — does not exist: the drain sizes to the declared depth, which
    // every build path already caps at the topic ceiling. The build must
    // simply succeed, with NO over-ceiling request and hence no clamp.
    //
    // The consumer MUST be INELIGIBLE for the
    // body-subscriber unification — only the ineligible (dual-subscriber)
    // arm creates a trigger-DRAIN subscriber at all.
    // `sample_trigger_consumer` (`sample(N)` backpressure) stays on the
    // dual-subscriber path.
    let (config, factories) = quiet_listed_graph_with(&[("sink", "sample_trigger_consumer")]);
    let clock = Arc::new(VirtualClock::new());
    let _rt = GraphRuntime::build_for_test(config, factories, clock, 32)
        .expect("a transport default above the Multi ceiling must not kill the build");
    // No clamp breadcrumb is expected: nothing requests above the ceiling,
    // so a clamp line here would mean a transport-default term crept in.
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("trigger-drain buffer clamped"))
            .count();
        if n == 0 {
            Ok(())
        } else {
            Err(format!(
                "expected NO clamp breadcrumb (the transport-default term is \
                 gone), got {n}"
            ))
        }
    });
}

#[test]
#[tracing_test::traced_test]
fn listed_provisioning_warns_and_infos_are_pinned() {
    // The two loud-inference surfaces at provisioning.
    // (a) consumed-only listed topic → exactly one "listing has no
    // effect" warn (External already admits multiple publishers);
    // (b) a produced listed topic → the "opted in" info with the cap
    // fields. Both in one traced test, distinct graphs.
    let consumed_only = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: vec!["/ext/imu".to_string()],
        name: None,
        identity: "consumed".to_string(),
        prefix: "co".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "sink".to_string(),
            node_type: "tiny_queue_consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: "/ext/imu".to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("sink".to_string(), Box::new(TinyQueueConsumerEntry::new()));
    let clock = Arc::new(VirtualClock::new());
    let _rt = GraphRuntime::build_for_test(consumed_only, factories, clock, 8)
        .expect("consumed-only listed topic builds (External)");

    let (config, factories) = quiet_listed_graph_with(&[("sink", "tiny_queue_consumer")]);
    let clock = Arc::new(VirtualClock::new());
    let _rt2 = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("produced listed topic builds (Multi)");

    logs_assert(|lines: &[&str]| {
        // Topic-scoped (a cross-branch message swap would survive
        // combined totals — each message must ride with ITS graph's topic).
        let no_effect = lines
            .iter()
            .filter(|l| l.contains("listing has no effect") && l.contains("/ext/imu"))
            .count();
        let opted_in = lines
            .iter()
            .filter(|l| l.contains("multi-publisher topic (opted in)") && l.contains("/tf"))
            .count();
        if no_effect == 1 && opted_in == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected exactly 1 no-effect warn + 1 opted-in info, got {no_effect}/{opted_in}"
            ))
        }
    });
}

#[test]
fn foreign_default_service_locks_out_listed_graph_with_multi_hint() {
    // Pin: a foreign creator at iceoryx2 defaults (2 publisher
    // slots) locks a listed graph out at open (requires the loose 16) —
    // the failure must name the multi-publisher cause + the
    // foreign-creator remedy, not the single-writer narrative.
    let ix = cerulion_core::testing::iceoryx_test_config();
    let raw_node = iceoryx2::node::NodeBuilder::new()
        .config(&ix)
        .create::<iceoryx2::service::ipc::Service>()
        .expect("raw iceoryx2 node");
    let data_name: iceoryx2::service::service_name::ServiceName =
        "/tf/data".try_into().expect("name");
    let _foreign = raw_node
        .service_builder(&data_name)
        .publish_subscribe::<[u8]>()
        .create()
        .expect("foreign service at iceoryx2 defaults");

    let mgr = manager("foreign_lockout", ix);
    let (cfg, fac) = producer_only_graph("fl", "bc", 1.0);
    let msg = match GraphRuntime::build(cfg, fac, &mgr, Arc::new(VirtualClock::new())) {
        Ok(_) => panic!("a listed graph must NOT open a foreign default-caps service"),
        Err(e) => e.to_string(),
    };
    assert!(
        msg.contains("multi_publisher_topics topic")
            && msg.contains("every participant must provision the shared loose caps"),
        "the failure must carry the Multi-aware cause + remedy: {msg}"
    );
}
