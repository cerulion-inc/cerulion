// SPDX-License-Identifier: AGPL-3.0-only
//! The DIRECT STREAM: a runtime armed without fork
//! (`GraphRuntime::attach_state_arm_no_fork`) captures the nodes the inline walk
//! defers on its own thread, driven through a REAL `GraphRuntime` over REAL iceoryx2,
//! a REAL mapped arm word and a REAL POSIX-SHM state ring.
//!
//! An executor unit that shares its process with other units cannot fork: the child
//! copies one thread, and a lock a sibling unit's thread holds stays held in the copy.
//! So this arm must capture a lock-carrying node and a node over the 64 KiB arena with
//! no child process, refuse a stream the ring cannot hold without waiting, and survive
//! an encoder that errors or panics.
//!
//! Oracles: the hand-written `Pattern` node's blob is built byte by byte here; the
//! macro nodes' blobs go through the production capture helper over a hand-constructed
//! value, as `state_anchor_fork_iox2_test` does. "No child" is read from the kernel:
//! `waitpid(-1, WNOHANG)` fails with `ECHILD` only when this process has no child,
//! alive or a zombie, which is why this is its own binary and no test here forks.
//!
//! ```bash
//! cargo test -p cerulion_core --test state_anchor_direct_iox2_test -- --test-threads=1
//! ```

#![cfg(unix)]

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::error::{TransportError, TransportResult};
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{MacroPolicy, NodeContext, NodeEntry, NodeInfo};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::state::{SkipCause, StateSink, VecSink};
use cerulion_core::state_arm::MappedStateArm;
use cerulion_core::state_carrier::DirectStreamStats;
use cerulion_core::state_restore::{
    capture_anchor_blob_with_framework, NodeFrameworkState, ANCHOR_BLOB_MAGIC_V2,
};
use cerulion_core::state_ring::{
    StateAnchorEvent, StateAssembler, StateRingConsumer, StateRingOwner,
};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// Plain data: inline-eligible. The control in every test.
#[cerulion_node(period_ms = 10)]
struct Counter {
    #[output]
    out: Vector3,
    count: u64,
}

#[cerulion_node_impl]
impl Counter {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.count += 1;
        Ok(())
    }
}

/// Carries a lock, so `INLINE_SAFE = false` and the walk defers it.
#[cerulion_node(period_ms = 10)]
struct Guarded {
    #[output]
    out: Vector3,
    shared: Arc<Mutex<u64>>,
    total: u64,
}

#[cerulion_node_impl]
impl Guarded {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.total += 2;
        Ok(())
    }
}

const PATTERN_SHAPE: u64 = 0xD1EC_0000_0000_1064;

#[derive(Clone, Copy)]
enum Encode {
    /// The tick count, then `.0` blocks of `.1` bytes, block `b` filled with `b`.
    Blocks(usize, usize),
    /// `.0` bytes of `0xAB`, then an error.
    ErrorAfter(usize),
    Panic,
}

/// A HAND-WRITTEN `NodeEntry` whose encoder the test spells out, with counters the
/// test reads while the runtime owns the node.
struct Pattern {
    inline_safe: bool,
    encode: Encode,
    ticks: Arc<AtomicU64>,
    entered: Arc<AtomicU32>,
    context: Option<NodeContext>,
}

/// `(ticks, encoder entries)` of a [`Pattern`] the runtime owns.
type Probe = (Arc<AtomicU64>, Arc<AtomicU32>);

fn pattern(inline_safe: bool, encode: Encode) -> (Box<dyn NodeEntry>, Probe) {
    let probe: Probe = Default::default();
    let node = Pattern {
        inline_safe,
        encode,
        ticks: Arc::clone(&probe.0),
        entered: Arc::clone(&probe.1),
        context: None,
    };
    (Box::new(node), probe)
}

fn entered(probe: &Probe) -> u32 {
    probe.1.load(Ordering::Relaxed)
}

fn graph_error(reason: &str) -> TransportError {
    TransportError::GraphError {
        reason: reason.to_string(),
    }
}

impl NodeEntry for Pattern {
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(NodeInfo::from_names(Vec::new(), Vec::new())
            .with_policy(MacroPolicy::Period { period_ms: 10 }))
    }

    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.context = Some(context);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        self.ticks.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }

    fn state_shape(&self) -> Option<u64> {
        Some(PATTERN_SHAPE)
    }

    fn inline_safe(&self) -> bool {
        self.inline_safe
    }

    fn capture_state(&self, out: &mut dyn StateSink) -> TransportResult<()> {
        self.entered.fetch_add(1, Ordering::Relaxed);
        let refused = |_| graph_error("the sink refused a write");
        match self.encode {
            Encode::Blocks(blocks, block) => {
                out.write(&self.ticks.load(Ordering::Relaxed).to_le_bytes())
                    .map_err(refused)?;
                for b in 0..blocks {
                    out.write(&vec![b as u8; block]).map_err(refused)?;
                }
                Ok(())
            }
            Encode::ErrorAfter(len) => {
                let _ = out.write(&vec![0xAB; len]);
                Err(graph_error("this fixture's encoder always refuses"))
            }
            Encode::Panic => panic!("this fixture's encoder always panics"),
        }
    }
}

const RING_RECORDS: u32 = 4096;
const RUN_ID: u64 = 0xC0FF_EE10_1064;

fn unique(tag: &str) -> String {
    static SEQ: AtomicU64 = AtomicU64::new(0);
    let n = SEQ.fetch_add(1, Ordering::Relaxed);
    format!("cd{tag}{}{n}", std::process::id() % 1000)
}

fn graph(prefix: &str, ids: &[&str]) -> GraphConfig {
    let out = OutputDef {
        name: "out".to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    };
    let node = |id: &str| NodeDef {
        isolate: None,
        params: Default::default(),
        ros2: None,
        id: id.to_string(),
        node_type: id.to_string(),
        inputs: vec![],
        outputs: vec![out.clone()],
    };
    GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "direct".to_string(),
        prefix: prefix.to_string(),
        nodes: ids.iter().map(|id| node(id)).collect(),
    }
}

/// `counter` first, then `others`, in manifest order.
fn with_counter(others: Vec<(&str, Box<dyn NodeEntry>)>) -> IndexMap<String, Box<dyn NodeEntry>> {
    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("counter".to_string(), Box::new(CounterEntry::new()));
    for (id, node) in others {
        nodes.insert(id.to_string(), node);
    }
    nodes
}

struct Harness {
    runtime: GraphRuntime,
    _owner: StateRingOwner,
    arm: Arc<MappedStateArm>,
    /// One consumer for the harness's life: a consumer opens at the ring's START, so
    /// a second one would read every earlier anchor again.
    consumer: StateRingConsumer,
}

impl Harness {
    fn new(tag: &str, nodes: IndexMap<String, Box<dyn NodeEntry>>, ring: u32) -> Self {
        let node_ids: Vec<String> = nodes.keys().cloned().collect();
        let ids: Vec<&str> = node_ids.iter().map(|s| s.as_str()).collect();
        let clock = Arc::new(VirtualClock::new());
        let mut runtime = GraphRuntime::build_for_test(graph(&unique(tag), &ids), nodes, clock, 16)
            .expect("graph builds");
        let mut owner = StateRingOwner::create(&unique(&format!("{tag}r")), ring, 0, RUN_ID, &ids)
            .expect("state ring");
        let consumer = StateRingConsumer::open(owner.name()).expect("open ring");
        runtime.set_state_ring_producer(owner.producer().expect("producer"), &node_ids);
        let arm = MappedStateArm::create_owned(&unique(&format!("{tag}a"))).expect("arm word");
        let arm = Arc::new(arm);
        arm.arm(4, 4);
        Harness {
            runtime,
            _owner: owner,
            arm,
            consumer,
        }
    }

    /// Armed WITHOUT fork, at cadence 4 from step 4.
    fn armed(tag: &str, nodes: IndexMap<String, Box<dyn NodeEntry>>, ring: u32) -> Self {
        let mut h = Self::new(tag, nodes, ring);
        h.runtime.attach_state_arm_no_fork(Arc::clone(&h.arm));
        h
    }

    fn step(&mut self, n: usize) {
        for _ in 0..n {
            self.runtime.step(Duration::from_millis(10));
        }
    }

    fn drain(&mut self) -> Vec<StateAnchorEvent> {
        let mut assembler = StateAssembler::passthrough();
        let mut events = Vec::new();
        self.consumer
            .drain(&mut assembler, &mut events)
            .expect("drain");
        events.extend(assembler.finish());
        events
    }
}

fn completes(events: &[StateAnchorEvent]) -> Vec<(u64, u32, Vec<u8>)> {
    events
        .iter()
        .filter_map(|e| match e {
            StateAnchorEvent::Complete {
                step,
                node_idx,
                bytes,
                ..
            } => Some((*step, *node_idx, bytes.clone())),
            _ => None,
        })
        .collect()
}

fn skips(events: &[StateAnchorEvent]) -> Vec<(u64, u32, SkipCause, String)> {
    events
        .iter()
        .filter_map(|e| match e {
            StateAnchorEvent::Skipped {
                step,
                node_idx,
                cause,
                detail,
                ..
            } => Some((*step, *node_idx, *cause, detail.clone())),
            _ => None,
        })
        .collect()
}

fn causes(events: &[StateAnchorEvent]) -> Vec<(u64, u32, SkipCause)> {
    skips(events)
        .into_iter()
        .map(|(s, i, c, _)| (s, i, c))
        .collect()
}

/// The kernel's answer to "does this process have a child?", alive or a zombie.
fn assert_no_child_process() {
    // SAFETY: a null status pointer is allowed, and `WNOHANG` never blocks.
    let rc = unsafe { libc::waitpid(-1, std::ptr::null_mut(), libc::WNOHANG) };
    let errno = std::io::Error::last_os_error().raw_os_error();
    assert_eq!(
        (rc, errno),
        (-1, Some(libc::ECHILD)),
        "this process has a child"
    );
}

/// A 10 ms `Period` node's framework section at `step`: after step `S` the clock is
/// `(S + 1) * 10 ms`, and the next fire is one period later.
fn framework_oracle(step: u64) -> NodeFrameworkState {
    NodeFrameworkState {
        next_fire_ns: Some((step + 2) * 10_000_000),
        ..NodeFrameworkState::default()
    }
}

/// An anchor "at S" is the state AFTER step S, and steps are 0-based, so a node that
/// fires once per step has fired `S + 1` times.
fn counter_at(step: u64) -> Vec<u8> {
    let node = Counter {
        count: step + 1,
        ..Default::default()
    };
    let mut sink = VecSink::new();
    capture_anchor_blob_with_framework(&node, &framework_oracle(step), &mut sink).expect("fits");
    sink.into_inner()
}

/// `Guarded` adds 2 per fire.
fn guarded_at(step: u64) -> Vec<u8> {
    let node = Guarded {
        total: 2 * (step + 1),
        ..Default::default()
    };
    let mut sink = VecSink::new();
    capture_anchor_blob_with_framework(&node, &framework_oracle(step), &mut sink).expect("fits");
    sink.into_inner()
}

/// A `Pattern` blob at `step`, byte by byte: the v2 magic and the shape, the section
/// length (25), the section (version 1, a present `next_fire_ns`, no pending data, no
/// Sync inputs), then the tick count and the blocks.
fn pattern_at(step: u64, blocks: usize, block: usize) -> Vec<u8> {
    let mut b = Vec::new();
    b.extend_from_slice(&ANCHOR_BLOB_MAGIC_V2.to_le_bytes());
    b.extend_from_slice(&PATTERN_SHAPE.to_le_bytes());
    b.extend_from_slice(&25u32.to_le_bytes());
    b.extend_from_slice(&1u32.to_le_bytes());
    b.push(1);
    b.extend_from_slice(&((step + 2) * 10_000_000).to_le_bytes());
    b.extend_from_slice(&[0; 12]);
    b.extend_from_slice(&(step + 1).to_le_bytes());
    for i in 0..blocks {
        b.extend(std::iter::repeat_n(i as u8, block));
    }
    b
}

/// 140 blocks of 512 bytes: 71 688 payload bytes, over the 64 KiB arena.
const BIG: Encode = Encode::Blocks(140, 512);

fn three_node_oracle() -> Vec<(u64, u32, Vec<u8>)> {
    vec![
        (4, 0, counter_at(4)),
        (4, 1, guarded_at(4)),
        (4, 2, pattern_at(4, 140, 512)),
    ]
}

/// `counter` inline, `guarded` gated, `big` overflowing the arena, to step 4's anchor.
fn run_three_node_graph(tag: &str) -> (Vec<StateAnchorEvent>, GraphRuntime, Probe) {
    let (big, probe) = pattern(true, BIG);
    let guarded: Box<dyn NodeEntry> = Box::new(GuardedEntry::new());
    let mut h = Harness::armed(
        tag,
        with_counter(vec![("guarded", guarded), ("big", big)]),
        RING_RECORDS,
    );
    h.step(5);
    (h.drain(), h.runtime, probe)
}

#[test]
#[serial]
fn a_lock_carrying_node_and_a_node_over_the_arena_are_captured_on_the_unit_thread() {
    let (events, runtime, big) = run_three_node_graph("hl");

    assert_eq!(completes(&events), three_node_oracle());
    assert!(skips(&events).is_empty(), "{:?}", skips(&events));
    assert_no_child_process();
    assert!(!runtime.capture_child_in_flight());
    assert!(runtime.state_arm_fork_sweep().is_none());

    let split = runtime.last_anchor_carriers().expect("an anchor at step 4");
    assert_eq!(
        (split.inline_nodes, split.deferred_nodes, split.gated_nodes),
        (1, 2, 1)
    );
    let records = |len: usize| len.div_ceil(472) as u64;
    assert_eq!(
        runtime.last_anchor_direct(),
        Some(DirectStreamStats {
            streamed_nodes: 2,
            records: records(guarded_at(4).len()) + records(pattern_at(4, 140, 512).len()),
            ..DirectStreamStats::default()
        })
    );
    assert_eq!(
        entered(&big),
        2,
        "once inline (overflow), once in the direct stream"
    );
}

#[test]
#[serial]
fn two_runs_on_two_threads_record_identical_events_in_their_own_rings() {
    // Two units of one process, each with its own arm word and ring, stepped at once.
    let a = std::thread::spawn(|| run_three_node_graph("tha").0);
    let b = std::thread::spawn(|| run_three_node_graph("thb").0);
    let (a, b) = (a.join().expect("thread a"), b.join().expect("thread b"));
    assert_eq!(completes(&a), three_node_oracle());
    assert!(skips(&a).is_empty(), "{:?}", skips(&a));
    assert_eq!(a, b, "two runs of one graph record the same events");
    assert_no_child_process();
}

#[test]
#[serial]
fn a_stream_past_the_ring_budget_is_refused_and_its_encoder_is_not_re_entered() {
    // 300 blocks of 1 KiB, about 651 records, into a 512-record ring.
    let (huge, probe) = pattern(false, Encode::Blocks(300, 1024));
    let mut h = Harness::armed("budget", with_counter(vec![("huge", huge)]), 512);

    let began = std::time::Instant::now();
    h.step(5);
    assert!(
        began.elapsed() < Duration::from_secs(2),
        "refused, not waited out"
    );

    // After the counter's one record 511 are free; one is kept per deferred node and
    // 64 are headroom, so the stream may use 446.
    let stats = h.runtime.last_anchor_direct().expect("a direct stream ran");
    assert_eq!((stats.refused_nodes, stats.not_entered_nodes), (1, 0));
    assert!(stats.records > 0 && stats.records <= 446, "{stats:?}");
    let events = h.drain();
    assert_eq!(completes(&events), vec![(4, 0, counter_at(4))]);
    let detail = "the recorder has not drained enough of the state ring to hold this anchor \
                  without blocking the node thread";
    assert_eq!(
        skips(&events),
        vec![(4, 1, SkipCause::RecorderBehind, detail.to_string())]
    );
    assert_eq!(
        events.len(),
        2,
        "the SKIP voids the truncated prefix: {events:?}"
    );
    assert_eq!(h.runtime.anchor_skips_unpublished(), 0);
    assert_eq!(entered(&probe), 1);

    // Same budget at the next cadence: refused WITHOUT entering the encoder.
    h.step(4);
    assert_eq!(entered(&probe), 1);
    let stats = h.runtime.last_anchor_direct().expect("a direct stream ran");
    assert_eq!(
        (stats.refused_nodes, stats.not_entered_nodes, stats.records),
        (1, 1, 0)
    );
    let events = h.drain();
    assert_eq!(completes(&events), vec![(8, 0, counter_at(8))]);
    assert_eq!(causes(&events), vec![(8, 1, SkipCause::RecorderBehind)]);
    assert_no_child_process();
}

#[test]
#[serial]
fn a_panicking_deferred_encoder_is_caught_and_its_node_keeps_ticking() {
    let (panicky, probe) = pattern(false, Encode::Panic);
    let mut h = Harness::armed(
        "panic",
        with_counter(vec![("panicky", panicky)]),
        RING_RECORDS,
    );

    h.step(5);
    let events = h.drain();
    assert_eq!(completes(&events), vec![(4, 0, counter_at(4))]);
    let detail = "the node's encoder panicked during direct capture";
    assert_eq!(
        skips(&events),
        vec![(4, 1, SkipCause::CaptureFailed, detail.to_string())]
    );

    // A poisoned node mutex would fail the next boundary's `try_lock` and skip the
    // WHOLE anchor as contended, so the counter's anchor at step 8 is the proof.
    h.step(4);
    assert_eq!(probe.0.load(Ordering::Relaxed), 9, "the node kept ticking");
    let events = h.drain();
    assert_eq!(completes(&events), vec![(8, 0, counter_at(8))]);
    assert_eq!(causes(&events), vec![(8, 1, SkipCause::CaptureFailed)]);
    assert_eq!(entered(&probe), 2);
}

#[test]
#[serial]
fn an_erroring_deferred_encoder_is_capture_failed_and_its_prefix_is_voided() {
    let (refusing, probe) = pattern(false, Encode::ErrorAfter(2000));
    let mut h = Harness::armed(
        "err",
        with_counter(vec![("refusing", refusing)]),
        RING_RECORDS,
    );

    h.step(5);
    let stats = h.runtime.last_anchor_direct().expect("a direct stream ran");
    assert_eq!((stats.failed_nodes, stats.not_entered_nodes), (1, 0));
    assert_eq!(
        stats.records, 4,
        "45 header bytes + 2000 = 2045: four whole records"
    );
    let events = h.drain();
    assert_eq!(completes(&events), vec![(4, 0, counter_at(4))]);
    let detail = "the node's encoder returned an error during direct capture";
    assert_eq!(
        skips(&events),
        vec![(4, 1, SkipCause::CaptureFailed, detail.to_string())]
    );
    assert_eq!(
        events.len(),
        2,
        "the SKIP voids the four records: {events:?}"
    );
    assert_eq!(entered(&probe), 1);
}

#[test]
#[serial]
fn a_claim_held_on_the_arm_word_does_not_stop_this_unit_anchoring() {
    // A fork-armed worker skips a cadence while any claim is held, as `StillEncoding`;
    // a unit armed without fork anchors, since the recorder picks one checkpoint per rank.
    let guarded: Box<dyn NodeEntry> = Box::new(GuardedEntry::new());
    let mut h = Harness::armed(
        "claim",
        with_counter(vec![("guarded", guarded)]),
        RING_RECORDS,
    );
    let pid = std::process::id() as i32;
    let slot = h.arm.claim(pid, 0).expect("a free claim slot");
    assert_ne!(h.arm.busy_workers(), 0);

    h.step(5);
    h.arm.release(slot, pid);
    let events = h.drain();
    assert_eq!(
        completes(&events),
        vec![(4, 0, counter_at(4)), (4, 1, guarded_at(4))]
    );
    assert!(skips(&events).is_empty(), "{:?}", skips(&events));
}

#[test]
#[serial]
fn a_foreign_holder_of_a_declared_lock_skips_every_node_as_contended() {
    // The claim gate is skipped in this arm; the contention probe after it is not.
    let shared = Arc::new(Mutex::new(7u64));
    let guarded = GuardedEntry::with_state(Guarded {
        shared: Arc::clone(&shared),
        ..Default::default()
    });
    let nodes = with_counter(vec![("guarded", Box::new(guarded) as Box<dyn NodeEntry>)]);
    let mut h = Harness::armed("cont", nodes, RING_RECORDS);

    let held = shared.lock().expect("uncontended");
    h.step(5);
    drop(held);
    let events = h.drain();
    assert!(completes(&events).is_empty(), "{events:?}");
    assert_eq!(
        causes(&events),
        vec![(4, 0, SkipCause::Contended), (4, 1, SkipCause::Contended)]
    );
    assert_eq!(
        h.runtime.last_anchor_direct(),
        None,
        "no stream was attempted"
    );
}

#[test]
#[serial]
fn arming_without_fork_after_a_fork_arm_tears_the_fork_carrier_down() {
    let guarded: Box<dyn NodeEntry> = Box::new(GuardedEntry::new());
    let mut h = Harness::new(
        "rearm",
        with_counter(vec![("guarded", guarded)]),
        RING_RECORDS,
    );
    h.runtime.attach_state_arm(Arc::clone(&h.arm));
    assert!(h.runtime.state_arm_fork_sweep().is_some());
    h.runtime.attach_state_arm_no_fork(Arc::clone(&h.arm));
    assert!(h.runtime.state_arm_fork_sweep().is_none());

    h.step(5);
    assert_no_child_process();
    let events = h.drain();
    assert_eq!(
        completes(&events),
        vec![(4, 0, counter_at(4)), (4, 1, guarded_at(4))]
    );
    let streamed = h.runtime.last_anchor_direct().map(|s| s.streamed_nodes);
    assert_eq!(
        streamed,
        Some(1),
        "the direct stream captured it, not a fork child"
    );

    // And a fork arm afterwards drops the direct stream.
    h.runtime.attach_state_arm(Arc::clone(&h.arm));
    assert_eq!(h.runtime.last_anchor_direct(), None);
}
