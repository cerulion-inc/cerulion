// SPDX-License-Identifier: AGPL-3.0-only
//! `block` backpressure with per-consumer credit slots, over real iceoryx2 in
//! one process, on a virtual clock (one 10 ms step per producer period).
//!
//! Rows, each with a count floor:
//! - a slow `block` consumer defers its producer (fire-rate drop);
//! - a fast `drop_oldest` consumer of the same topic keeps up with every
//!   publish in the step it was made, and records no defer or drop;
//! - `block_timeout_ms` releases the stalled consumer's slot and the producer
//!   resumes, and the release touches that slot only;
//! - after a timeout release, a FIFO consumer that resumes between two
//!   publishes receives every frame published after the release;
//! - after a timeout release, a send that reaches no queue takes its count
//!   back through the real publisher and restores the forgiven backlog that
//!   count clamped, and the next frame is delivered; the drains of forgiven
//!   frames report no clip, one frame at a time or in one read;
//! - a consumer that keeps draining after its slot is released reports no
//!   clip and logs no line of the clip report, on a latest and a FIFO input,
//!   and a local word reports the drain of a frame it never counted when that
//!   drain finds the queue empty: one loud head, then a counted repeat;
//! - after a timeout release, a frame the release forgave that the per-set
//!   Sync matcher moves into a slot (the trigger fill or refill, the
//!   descent's peek) stays forgiven, a skipped forgiven head refilled with a
//!   frame published after the release leaves that frame counted, an errored
//!   refill holds nothing, and each walk equals its hand oracle row for row,
//!   twice; on a cross-process word a refilled head counts as held;
//! - a skipped head the queue refills rings the wake word once, at the
//!   refill's pop, on a local and a cross-process word, and the refill's
//!   drain does not reopen the consumer's `block` regime; a skipped head
//!   whose refill fails leaves its slot;
//! - a multi-process worker refuses `block_timeout_ms` on a consumer input
//!   its plan binds to a credit word shared with a producer in another
//!   process, and builds the same input without the key; a worker whose
//!   plan binds another consumer of the topic builds a local input with the
//!   key, releases that slot once and never forgives the shared word;
//! - a single-producer edge with `block_timeout_ms` loads, is released once,
//!   and its consumer then receives every frame published after the release
//!   with the slot's count equal to the frames still queued;
//! - on a word fed by two publishers, a FIFO read over real queues that finds
//!   the queue empty clears the count a release left high;
//! - `block_timeout_ms` on a topic with several producers is refused at load
//!   with one text, however the input is made `block`, on the
//!   single-process build, the planning build and the free-run live build of
//!   the whole graph; without the key such a
//!   consumer that stops for good still has its slot released;
//! - a consumer node that stops with a fatal error has its slot released;
//! - the live loop's wait sizing holds a deferred producer until its slot's
//!   `block_timeout_ms` deadline, or until the drain that frees it, instead of
//!   re-evaluating it at the 1 ms floor;
//! - a held `external` producer whose event its doorbell already delivered
//!   is released at its `block_timeout_ms` deadline on the live loop, not at
//!   the 250 ms sweep;
//! - the graph's `backpressure:` key turns a `drop_oldest` input into `block`;
//! - a ring of `block` edges, `block` on a `network.ingress` topic, and the
//!   graph's `backpressure: block` or `depth:` on an `#[input(accumulate)]`
//!   input, are refused at build;
//! - a topic with `block` and `drop_oldest` or `sample(N)` consumers logs one
//!   load-time warning naming who paces whom, and no other topic logs it; a
//!   multi-process worker logs its plan's warnings in place of its own view.
//!
//! Mutants (`--features oracle-mutants`, `CERULION_ORACLE_MUTANT=<name>`):
//! `block_no_timeout_release`, `block_shared_release`, `block_cycle_off`,
//! `block_accumulate_refusal_off`, `block_forgiven_drain_spends_credit`,
//! `block_empty_resync_off`, `block_shared_timeout_refusal_off`,
//! `block_split_timeout_refusal_off`, `block_forgiven_hold_counted` and
//! `block_replaced_head_unpaid` each make a row here fail.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::credit::{credit_edge_id, CreditShared, CreditWord, MappedCredit};
use cerulion_core::graph::config::{
    GraphConfig, InputBackpressure, InputDef, NetworkBlock, NetworkMode, NodeDef, OutputDef,
};
use cerulion_core::graph::node::{ClosureNodeEntry, InputMeta, MacroPolicy, NodeEntry, NodeInfo};
use cerulion_core::graph::{BuildPurpose, CrossProcessWiring, GraphRuntime};
use cerulion_core::message::ShmMessage;
use cerulion_core::prelude::*;
use cerulion_core::scheduler::BackpressureCounters;
use cerulion_core::transport::publisher::BlockSendHook;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use tracing_test::traced_test;

const STEPS: u64 = 100;

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct Producer {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl Producer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

/// `block`, depth 2, ticks only when the test triggers it.
#[cerulion_node(external)]
#[derive(Default)]
struct HostBlock {
    #[input(backpressure = block, depth = 2)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl HostBlock {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// `block`, depth 2, stops with a fatal error on its first tick.
#[cerulion_node(external)]
#[derive(Default)]
struct FatalBlock {
    #[input(backpressure = block, depth = 2)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl FatalBlock {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Err(NodeError::Fatal("consumer stops".to_string()))
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// `drop_oldest`, depth 2, ticks only when the test triggers it.
#[cerulion_node(external)]
#[derive(Default)]
struct HostDropOldest {
    #[input(backpressure = drop_oldest, depth = 2)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl HostDropOldest {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// `sample(15)`, depth 2, ticks only when the test triggers it.
#[cerulion_node(external)]
#[derive(Default)]
struct HostSample {
    #[input(backpressure = sample(15), depth = 2)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl HostSample {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// `drop_oldest`, fires on every frame.
#[cerulion_node]
#[derive(Default)]
struct FastDropOldest {
    #[input(trigger, backpressure = drop_oldest, depth = 8)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl FastDropOldest {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// `#[input(accumulate)]`, ticks only when the test triggers it.
#[cerulion_node(external)]
#[derive(Default)]
struct HostAccumulate {
    #[input(accumulate)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl HostAccumulate {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.iter().count();
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// `block` input, one output: a ring member. Period-driven, so the ring is
/// not a ring of triggering edges (which the level derivation refuses on its
/// own).
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct Relay {
    #[input(backpressure = block, depth = 2)]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl Relay {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.inp.x;
        Ok(())
    }
}

/// An `external` producer fed by a blocking read: each `()` the test sends is
/// one event, which the runtime's helper thread rings on the node's doorbell.
#[cerulion_node(external)]
#[derive(Default)]
struct BlockingProducer {
    #[output]
    out: Vector3,
    events: Option<mpsc::Receiver<()>>,
    n: u32,
}

#[cerulion_node_impl]
impl BlockingProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        let events = self
            .events
            .take()
            .expect("the runtime queries the external source once");
        ExternalSource::Blocking(Box::new(move || {
            events.recv_timeout(Duration::from_millis(20)).is_ok()
        }))
    }
}

/// `block`, depth 2, on a period far past any test, so it never drains.
#[cerulion_node(period_ms = 600000)]
#[derive(Default)]
struct IdleBlock {
    #[input(backpressure = block, depth = 2)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl IdleBlock {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// An `external` producer the test never triggers: the second producer of a
/// shared topic, whose connection into the consumer stays empty.
#[cerulion_node(external)]
#[derive(Default)]
struct QuietProducer {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl QuietProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

fn output(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    }
}

/// An output that publishes on the absolute `topic`.
fn output_on(name: &str, topic: &str) -> OutputDef {
    OutputDef {
        topic: Some(topic.to_string()),
        ..output(name)
    }
}

fn input(source: &str) -> InputDef {
    InputDef {
        name: "inp".to_string(),
        source: source.to_string(),
        ..Default::default()
    }
}

fn node(id: &str, ty: &str, inputs: Vec<InputDef>, outputs: Vec<OutputDef>) -> NodeDef {
    NodeDef {
        isolate: None,
        params: Default::default(),
        ros2: None,
        id: id.to_string(),
        node_type: ty.to_string(),
        inputs,
        outputs,
    }
}

fn config(prefix: &str, nodes: Vec<NodeDef>) -> GraphConfig {
    GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("{prefix}_graph"),
        prefix: prefix.to_string(),
        nodes,
    }
}

fn entry(ty: &str) -> Box<dyn NodeEntry> {
    match ty {
        "producer" => Box::new(ProducerEntry::new()),
        "host_block" => Box::new(HostBlockEntry::new()),
        "fatal_block" => Box::new(FatalBlockEntry::new()),
        "host_drop_oldest" => Box::new(HostDropOldestEntry::new()),
        "host_sample" => Box::new(HostSampleEntry::new()),
        "fast_drop_oldest" => Box::new(FastDropOldestEntry::new()),
        "relay" => Box::new(RelayEntry::new()),
        "host_accumulate" => Box::new(HostAccumulateEntry::new()),
        "quiet_producer" => Box::new(QuietProducerEntry::new()),
        other => panic!("no entry for {other}"),
    }
}

fn factories_for(cfg: &GraphConfig) -> IndexMap<String, Box<dyn NodeEntry>> {
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for n in &cfg.nodes {
        factories.insert(n.id.clone(), entry(&n.node_type));
    }
    factories
}

fn build(cfg: GraphConfig) -> Result<GraphRuntime, String> {
    let factories = factories_for(&cfg);
    GraphRuntime::build_for_test(cfg, factories, Arc::new(VirtualClock::new()), 64)
        .map_err(|e| e.to_string())
}

/// Build `cfg` as the multi-process supervisor's planning build does, before
/// it spawns a worker. `None` when it builds.
fn planning_refusal(cfg: GraphConfig) -> Option<String> {
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("{}_plan", cfg.prefix),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 64,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("per-test SHM transport");
    let factories = factories_for(&cfg);
    match GraphRuntime::build_with_schema_hashes_provisioning_and_recorded(
        cfg,
        factories,
        &mgr,
        Arc::new(VirtualClock::new()),
        None,
        None,
        None,
        BuildPurpose::PlanningOnly,
    ) {
        Ok(_planning) => None,
        Err(e) => Some(e.to_string()),
    }
}

/// Build the whole of `cfg` on the live free-run build path, with no
/// cross-process wiring.
fn build_live(cfg: GraphConfig) -> Result<GraphRuntime, String> {
    let factories = factories_for(&cfg);
    GraphRuntime::build_for_test_with_policy_and_wiring(
        cfg,
        factories,
        Arc::new(VirtualClock::new()),
        64,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::none(),
    )
    .map_err(|e| e.to_string())
}

fn fires(rt: &GraphRuntime, id: &str) -> u64 {
    rt.node_handle(id).unwrap().fire_count()
}

fn releases(rt: &GraphRuntime, id: &str) -> u64 {
    rt.node_handle(id)
        .unwrap()
        .backpressure_block_slot_releases_count("inp")
}

fn outstanding(rt: &GraphRuntime, consumer: &str) -> u64 {
    rt.block_credits()
        .find(|c| c.consumer_node == consumer)
        .unwrap_or_else(|| panic!("no block credit for {consumer}"))
        .outstanding
}

/// Run `STEPS` steps, triggering `slow` every `every` steps. Returns the
/// producer's fire count after each step.
fn run(rt: &mut GraphRuntime, slow: Option<(&str, u64)>, mut each: impl FnMut(&GraphRuntime)) {
    for step in 0..STEPS {
        if let Some((id, every)) = slow {
            if step % every == every - 1 {
                rt.trigger_external(id).expect("trigger");
            }
        }
        rt.step(Duration::from_millis(10));
        each(rt);
    }
}

#[test]
fn a_slow_block_consumer_defers_its_producer() {
    let mut rt = build(config(
        "bsr_slow",
        vec![
            node("producer", "producer", vec![], vec![output("out")]),
            node("slow", "host_block", vec![input("producer/out")], vec![]),
        ],
    ))
    .expect("build");
    run(&mut rt, Some(("slow", 5)), |_| {});
    let produced = fires(&rt, "producer");
    let deferred = rt
        .node_handle("slow")
        .unwrap()
        .backpressure_block_fires_deferred_count("inp");
    // Unpressured the producer fires once per step (100). The slow consumer
    // drains its depth-2 queue every 5th step, so the producer gets about 2
    // fires per 5 steps.
    assert!(
        (30..=45).contains(&produced),
        "producer fires {produced}: expected about 2 per 5 steps (an unpressured producer fires {STEPS})"
    );
    assert!(
        deferred >= 50,
        "deferred evaluations {deferred} below the floor of 50"
    );
}

fn mixed(prefix: &str, with_slow: bool) -> GraphConfig {
    let mut nodes = vec![
        node("producer", "producer", vec![], vec![output("out")]),
        node(
            "fast",
            "fast_drop_oldest",
            vec![input("producer/out")],
            vec![],
        ),
    ];
    if with_slow {
        nodes.push(node(
            "slow",
            "host_block",
            vec![input("producer/out")],
            vec![],
        ));
    }
    config(prefix, nodes)
}

#[test]
fn a_fast_drop_oldest_consumer_on_the_same_topic_is_unaffected() {
    // Control: no block consumer.
    let mut control = build(mixed("bsr_ctl", false)).expect("build control");
    let mut lag_control = 0u64;
    run(&mut control, None, |rt| {
        lag_control = lag_control.max(fires(rt, "producer") - fires(rt, "fast"));
    });
    assert_eq!(fires(&control, "producer"), STEPS);
    assert_eq!(fires(&control, "fast"), STEPS);

    let mut rt = build(mixed("bsr_mix", true)).expect("build mixed");
    let mut steps_checked = 0u64;
    let mut max_lag = 0u64;
    run(&mut rt, Some(("slow", 5)), |rt| {
        // After every step the fast consumer has consumed every frame the
        // producer published: it never lags, even by one step.
        max_lag = max_lag.max(fires(rt, "producer") - fires(rt, "fast"));
        steps_checked += 1;
    });
    assert_eq!(steps_checked, STEPS);
    let produced = fires(&rt, "producer");
    assert!(
        (30..=45).contains(&produced),
        "the block consumer must still defer the producer on a mixed topic (fires {produced})"
    );
    assert_eq!(
        fires(&rt, "fast"),
        produced,
        "the drop_oldest consumer fires once per publish"
    );
    assert_eq!(
        max_lag, lag_control,
        "the drop_oldest consumer lags no more than with no block consumer"
    );
    let fast = rt.node_handle("fast").unwrap();
    assert_eq!(
        fast.backpressure_drop_oldest_count("inp"),
        0,
        "nothing evicted"
    );
    assert_eq!(fast.backpressure_block_fires_deferred_count("inp"), 0);
    assert!(
        rt.block_credits().all(|c| c.consumer_node != "fast"),
        "a drop_oldest consumer holds no credit slot"
    );
}

fn with_timeout(mut i: InputDef, ms: u64) -> InputDef {
    i.block_timeout_ms = Some(ms);
    i
}

#[test]
fn block_timeout_releases_the_stalled_slot_and_the_producer_resumes() {
    let mut rt = build(config(
        "bsr_to",
        vec![
            node("producer", "producer", vec![], vec![output("out")]),
            node(
                "stuck",
                "host_block",
                vec![with_timeout(input("producer/out"), 50)],
                vec![],
            ),
        ],
    ))
    .expect("build");
    run(&mut rt, None, |_| {});
    let produced = fires(&rt, "producer");
    let released = releases(&rt, "stuck");
    // Each cycle: 2 fires fill the slot, about 5 deferred steps reach 50 ms,
    // then the slot is forgiven.
    assert!(released >= 5, "releases {released} below the floor of 5");
    assert!(
        rt.block_credits()
            .all(|c| c.block_timeout && !c.conservation_broken()),
        "a timeout edge is never judged by the conservation assert"
    );
    assert!(
        produced >= 2 * released && produced < STEPS,
        "producer fires {produced} with {released} releases: it must resume after each, and still be held between them"
    );
}

#[test]
fn a_timeout_release_touches_only_the_timed_out_slot() {
    let mut rt = build(config(
        "bsr_one",
        vec![
            node("producer", "producer", vec![], vec![output("out")]),
            node(
                "stuck",
                "host_block",
                vec![with_timeout(input("producer/out"), 50)],
                vec![],
            ),
            node("held", "host_block", vec![input("producer/out")], vec![]),
        ],
    ))
    .expect("build");
    run(&mut rt, None, |_| {});
    assert!(
        releases(&rt, "stuck") >= 1,
        "the timed-out slot was released"
    );
    assert_eq!(releases(&rt, "held"), 0, "the other slot was not");
    assert_eq!(
        outstanding(&rt, "held"),
        2,
        "the other consumer's credit is untouched: still full"
    );
    assert_eq!(
        outstanding(&rt, "stuck"),
        0,
        "the released slot was forgiven"
    );
    assert_eq!(
        fires(&rt, "producer"),
        2,
        "the held consumer still defers the producer after the release"
    );
}

/// A 10 ms consumer whose `inp` is `block`, depth 2. On every `drain`-th tick
/// it drains its whole queue with `try_receive` and appends each frame's `x` to
/// `seen`; `drain` 0 never drains.
fn recording_block_consumer(
    drain: Arc<AtomicU64>,
    seen: Arc<Mutex<Vec<u32>>>,
) -> Box<dyn NodeEntry> {
    let meta = InputMeta {
        name: "inp".to_string(),
        schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
        trigger: false,
        depth: 2,
        backpressure: BackpressurePolicy::Block,
        expect_within_ms: None,
        accumulate: false,
        consume: Default::default(),
    };
    let info =
        NodeInfo::with_meta(vec![meta], vec![]).with_policy(MacroPolicy::Period { period_ms: 10 });
    let mut ticks = 0u64;
    Box::new(
        ClosureNodeEntry::new(info, move |ctx| {
            ticks += 1;
            let every = drain.load(Ordering::Relaxed);
            if every == 0 || !ticks.is_multiple_of(every) {
                return Ok(());
            }
            let sub = ctx.subscriber("inp").expect("the block input is wired");
            let mut got = Vec::new();
            sub.try_receive(|msg| {
                let x = f64::from_le_bytes(msg.payload()[0..8].try_into().expect("8 bytes"));
                got.push(x as u32);
            })?;
            seen.lock().expect("seen").extend(got);
            Ok(())
        })
        .with_label("recording_block_consumer"),
    )
}

/// A single-producer `block` edge (depth 2) with `block_timeout_ms: 50`
/// loads, and its timeout release is exact. The consumer stalls until one
/// release; it then drains every third tick, slower than the producer and well
/// inside the timeout. Over 60 steps no second release follows, every frame
/// published after the release reaches the consumer once and in order, so none
/// was evicted, and the slot's count equals the frames still queued.
#[test]
fn a_single_producer_timeout_release_is_exact() {
    const WINDOW: u64 = 60;
    let drain = Arc::new(AtomicU64::new(0));
    let seen: Arc<Mutex<Vec<u32>>> = Arc::new(Mutex::new(Vec::new()));
    let cfg = config(
        "bsr_exact",
        vec![
            node("producer", "producer", vec![], vec![output("out")]),
            node(
                "consumer",
                "recording_block",
                vec![with_timeout(input("producer/out"), 50)],
                vec![],
            ),
        ],
    );
    let mut nodes: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    nodes.insert("producer".to_string(), entry("producer"));
    nodes.insert(
        "consumer".to_string(),
        recording_block_consumer(Arc::clone(&drain), Arc::clone(&seen)),
    );
    let mut rt = GraphRuntime::build_for_test(cfg, nodes, Arc::new(VirtualClock::new()), 64)
        .expect("a single-producer block edge with block_timeout_ms loads");
    // The producer's fires before the step that releases the slot.
    let mut held_at = 0u64;
    let mut stalled_steps = 0u64;
    while releases(&rt, "consumer") == 0 {
        assert!(stalled_steps < 30, "no timeout release within 300 ms");
        held_at = fires(&rt, "producer");
        rt.step(Duration::from_millis(10));
        stalled_steps += 1;
    }
    assert_eq!(
        held_at, 2,
        "the stalled consumer held the producer at depth until the release"
    );
    assert!(
        stalled_steps >= 6,
        "released at step {stalled_steps}: the slot filled at step 2, so 50 ms of stall ends no \
         earlier than step 6"
    );
    assert!(
        seen.lock().expect("seen").is_empty(),
        "precondition: the consumer drained nothing before the release"
    );
    drain.store(3, Ordering::Relaxed);
    for _ in 0..WINDOW {
        rt.step(Duration::from_millis(10));
    }
    let published = fires(&rt, "producer");
    let seen = seen.lock().expect("seen").clone();
    assert_eq!(
        releases(&rt, "consumer"),
        1,
        "a consumer that drains inside the timeout is never released again (received {seen:?})"
    );
    let after: Vec<u32> = seen
        .iter()
        .copied()
        .filter(|&x| u64::from(x) > held_at)
        .collect();
    let last = after.last().copied().map_or(held_at, u64::from);
    let want: Vec<u32> = (held_at + 1..=last).map(|x| x as u32).collect();
    assert_eq!(
        after, want,
        "every frame published after the release is received once, in order"
    );
    assert!(
        seen.windows(2).all(|w| w[0] < w[1]),
        "no frame is received twice or out of order: {seen:?}"
    );
    assert!(
        published - held_at >= 30,
        "the producer resumes: {} fires after the release in {WINDOW} steps",
        published - held_at
    );
    let queued = published - last;
    assert!(queued <= 2, "{queued} frames queued past depth 2");
    assert_eq!(
        outstanding(&rt, "consumer"),
        queued,
        "the slot counts exactly the frames still queued ({published} published, {last} received)"
    );
}

/// Publish one `Vector3` carrying `x` on `publisher` and count it on `word`,
/// as a `block` producer's publish does.
fn publish_counted(publisher: &mut CerulionPublisher, word: &CreditShared, x: u32) {
    let mut p = publisher.loan_proxy::<Vector3>().expect("loan");
    p.x = f64::from(x);
    drop(p);
    word.record_published();
}

/// One `block` FIFO input over a real depth-2 iceoryx2 queue, released by a
/// timeout while it holds two frames, then resumed between two publishes.
///
/// `word` is the local word the input's probe drains through, the producer's
/// gate reads and its publish counts on. Frames carry `x = 1, 2, ...` in
/// publish order. The consumer drains one frame after the
/// first publish that follows the release, then the producer publishes while
/// its gate allows (at most 4 more), then the consumer drains what is queued.
/// Returns the frames published after the release and the frames the consumer
/// received after the release.
fn resume_after_a_timeout_release(tag: &str, word: CreditWord) -> (Vec<u32>, Vec<u32>) {
    let producer: &CreditShared = &word;
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("bsr_{tag}"),
            clock: Arc::new(cerulion_core::clock::RealClock),
            subscriber_buffer_size: 2,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("per-test SHM transport");
    let topic = format!("bsr_forgive_{tag}");
    let mut publisher = mgr
        .create_publisher(&topic, MaxSliceLen::try_new(1024).expect("len"), 0)
        .expect("publisher");
    let mut sub = mgr.create_subscriber(&topic).expect("subscriber");
    sub.register_block_probe_for_test(
        word.clone(),
        2,
        Arc::new(BackpressureCounters::new()),
        Arc::from("inp"),
        2,
    );
    sub.mark_fifo_consume_for_test();
    let read = |sub: &mut CerulionSubscriber| {
        sub.try_view::<Vector3, u32>(|v| v.x as u32)
            .expect("the FIFO read does not fail")
    };

    for x in 1..=2 {
        assert!(producer.try_claim(), "frame {x} fits the depth-2 slot");
        publish_counted(&mut publisher, producer, x);
    }
    assert!(!producer.try_claim(), "precondition: the slot is full");
    // The `block_timeout_ms` release: the runtime's pre-fire calls this on
    // the slot that stayed full with nothing drained.
    assert_eq!(
        producer.forgive_outstanding(),
        2,
        "both frames are forgiven"
    );

    let mut published = Vec::new();
    assert!(
        producer.try_claim(),
        "the release lets the producer publish"
    );
    publish_counted(&mut publisher, producer, 3);
    published.push(3);
    // The queue holds 2: frame 1 was evicted by frame 3, which the release
    // event announces. The consumer resumes and drains one forgiven frame.
    assert_eq!(read(&mut sub), Some(2), "the oldest queued frame is 2");
    for x in 4..8 {
        if !producer.try_claim() {
            break;
        }
        publish_counted(&mut publisher, producer, x);
        published.push(x);
    }
    let mut received = Vec::new();
    while let Some(x) = read(&mut sub) {
        received.push(x);
    }
    (published, received)
}

/// After a `block_timeout_ms` release, the consumer's drains of the frames
/// queued at the release do not free credit that belongs to frames published
/// after it: the producer is held again once 2 frames published after the
/// release are queued, and every one of them reaches the consumer. A drain of
/// frame 2 that spent frame 3's credit would let the producer publish 4 and 5,
/// and the queue would evict 3 with no release event and no count.
#[test]
fn frames_published_after_a_timeout_release_are_never_evicted() {
    let local = CreditWord::local(2);
    let (published, received) = resume_after_a_timeout_release("forgive_local", local.clone());
    assert_eq!(
        published,
        vec![3, 4],
        "the producer is held once frames 3 and 4 fill the slot (received {received:?})"
    );
    assert_eq!(
        received, published,
        "every frame published after the release is received"
    );
    assert_eq!(local.outstanding(), 0, "the drains emptied the slot");
}

/// One `block` input over a real depth-2 iceoryx2 queue on its own SHM root,
/// whose probe drains through `word`, and a publisher on its topic that counts
/// every frame on `word` before the send, as the publisher of a `block`
/// producer does. With `fifo` each read pops one frame, as a FIFO input's
/// does; without it a read drains the queue and keeps the newest frame.
fn counting_edge(
    tag: &str,
    word: &CreditWord,
    fifo: bool,
) -> (Arc<TransportManager>, CerulionPublisher, CerulionSubscriber) {
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("bsr_counted_{tag}"),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 2,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("per-test SHM transport");
    let topic = format!("bsr_counted_{tag}");
    let mut publisher = mgr
        .create_publisher(&topic, MaxSliceLen::try_new(1024).expect("len"), 0)
        .expect("publisher");
    publisher.register_block_outstanding_for_test(word.clone());
    let mut sub = mgr.create_subscriber(&topic).expect("subscriber");
    sub.register_block_probe_for_test(
        word.clone(),
        2,
        Arc::new(BackpressureCounters::new()),
        Arc::from("inp"),
        2,
    );
    if fifo {
        sub.mark_fifo_consume_for_test();
    }
    (mgr, publisher, sub)
}

/// Publish one `Vector3` carrying `x` through `publisher`'s typed publish.
fn publish_x(publisher: &mut CerulionPublisher, x: u32) {
    let mut p = publisher.loan_proxy::<Vector3>().expect("loan");
    p.x = f64::from(x);
    drop(p);
}

/// One read of a `Vector3` input: the `x` it served, if any.
fn read_x(sub: &mut CerulionSubscriber) -> Option<u32> {
    sub.try_view::<Vector3, u32>(|v| v.x as u32)
        .expect("the read does not fail")
}

/// `(outstanding, forgiven)`: the two counts a slot's gate and its forgiven
/// backlog read.
fn counts(word: &CreditWord) -> (u64, u64) {
    (word.outstanding(), word.forgiven())
}

/// A send that reaches no queue on a forgiven LOCAL word takes its count back
/// and restores the forgiven backlog that count clamped, through the real
/// publisher and a real depth-2 queue, and the next frame is delivered and
/// stays counted until it is read.
///
/// Oracle, by hand, as `(outstanding, forgiven)`, frames `x = 1, 2, ...` in
/// publish order: frames 1 and 2 are published (2, 0) and the timeout release
/// forgives them (0, 2). Frame 3's send is refused after its count
/// (`BlockSendHook::FailNextSend`) and the word reads (0, 2) again. The
/// consumer reads frame 1 (0, 1), frame 4 is published (1, 1), the consumer
/// reads frame 2, the last forgiven frame (1, 0), and then frame 4 (0, 0); no
/// drain clipped. A take-back that only subtracted the queued count would
/// leave (0, 1) after frame 3, the read of frame 2 would take frame 4's count,
/// and the read of frame 4 would clip.
#[test]
fn a_failed_send_on_a_forgiven_local_word_restores_its_backlog() {
    let word = CreditWord::local(2);
    let (_mgr, mut publisher, mut sub) = counting_edge("undo_forgiven", &word, true);
    publish_x(&mut publisher, 1);
    publish_x(&mut publisher, 2);
    assert_eq!(
        counts(&word),
        (2, 0),
        "the publisher counted frames 1 and 2"
    );
    assert_eq!(
        word.forgive_outstanding(),
        2,
        "the timeout release forgives both"
    );
    assert_eq!(counts(&word), (0, 2));

    publisher.set_block_send_hook_for_test(BlockSendHook::FailNextSend);
    publish_x(&mut publisher, 3);
    assert_eq!(
        counts(&word),
        (0, 2),
        "frame 3 reached no queue: its take-back restores the backlog its count clamped"
    );

    assert_eq!(read_x(&mut sub), Some(1), "frame 1");
    assert_eq!(counts(&word), (0, 1));
    publish_x(&mut publisher, 4);
    assert_eq!(counts(&word), (1, 1), "frame 4 is counted");
    assert_eq!(
        read_x(&mut sub),
        Some(2),
        "frame 2, the last forgiven frame"
    );
    assert_eq!(counts(&word), (1, 0), "frame 4 is still counted");
    assert_eq!(read_x(&mut sub), Some(4), "frame 4 is delivered");
    assert_eq!(counts(&word), (0, 0));
    assert_eq!(read_x(&mut sub), None, "and nothing else is queued");
    assert_eq!(sub.block_drains_clipped(), 0, "no drain clipped");
}

/// After a timeout release, the consumer's drains of the frames the release
/// forgave pay the forgiven backlog and report no clip: each forgiven frame was
/// counted when it was sent, so its drain removes no frame its word never
/// counted.
///
/// Oracle, by hand, on depth-2 local words over real depth-2 queues, as
/// `(outstanding, forgiven)`. A FIFO input: frames 1 and 2 are published and
/// released (0, 2), and the reads of frame 1 and frame 2 leave (0, 1) and
/// (0, 0). An input whose read drains the queue: frames 1 and 2 are published
/// and released (0, 2), frame 3 is published and evicts frame 1 (1, 1), and
/// one read removes frames 2 and 3, one forgiven and one counted, and serves
/// frame 3 (0, 0). `block_drains_clipped()` reads 0 after every drain. A clip
/// reckoned against the counted frames alone would count each of those
/// drains as a clip.
#[test]
fn a_drain_of_forgiven_frames_reports_no_clip() {
    let fifo = CreditWord::local(2);
    let (_fifo_mgr, mut publisher, mut sub) = counting_edge("noclip_fifo", &fifo, true);
    publish_x(&mut publisher, 1);
    publish_x(&mut publisher, 2);
    assert_eq!(
        fifo.forgive_outstanding(),
        2,
        "the release forgives frames 1 and 2"
    );
    assert_eq!(read_x(&mut sub), Some(1));
    assert_eq!(counts(&fifo), (0, 1));
    assert_eq!(
        sub.block_drains_clipped(),
        0,
        "the drain of forgiven frame 1 pays the backlog and clips nothing"
    );
    assert_eq!(read_x(&mut sub), Some(2));
    assert_eq!(counts(&fifo), (0, 0));
    assert_eq!(
        sub.block_drains_clipped(),
        0,
        "and so does the drain of forgiven frame 2"
    );

    let latest = CreditWord::local(2);
    let (_latest_mgr, mut publisher, mut sub) = counting_edge("noclip_latest", &latest, false);
    publish_x(&mut publisher, 1);
    publish_x(&mut publisher, 2);
    assert_eq!(latest.forgive_outstanding(), 2);
    publish_x(&mut publisher, 3);
    assert_eq!(counts(&latest), (1, 1), "frame 3 evicted forgiven frame 1");
    assert_eq!(
        read_x(&mut sub),
        Some(3),
        "the read drains frames 2 and 3 and serves the newest"
    );
    assert_eq!(counts(&latest), (0, 0));
    assert_eq!(
        sub.block_drains_clipped(),
        0,
        "a drain of a forgiven frame and a counted one clips nothing"
    );
}

/// The words every line of a `block` input's clip report starts with: the
/// loud head, the decade line, the counted repeat and the recovery line.
const CLIP_REPORT: &str = "block credit drain";
/// The text of the loud line a clipped drain logs.
const CLIP_LOUD: &str = "block credit drain removed frames its word never counted";
/// The text of a counted repeat, logged at `debug`.
const CLIP_REPEAT: &str = "block credit drain clipped at zero again";
/// The text of the recovery line.
const CLIP_RECOVERED: &str = "block credit drains match the word's count again";

/// A consumer that keeps draining after its slot is released reports no clip:
/// the release zeroed the slot's counts and the slot counts nothing after it,
/// so a drain of a frame queued before the release or sent after it removes a
/// frame the slot holds no count for by design, not one a writer outside the
/// graph sent.
///
/// Oracle, by hand, on depth-2 local words over real depth-2 queues, as
/// `(outstanding, forgiven)`. An input whose read drains the queue: frames 1
/// and 2 are published (2, 0) and the slot is released (0, 0); frame 3 counts
/// nothing and evicts frame 1, one read removes frames 2 and 3 and serves
/// frame 3 (0, 0), and frame 4 is published and read the same way. A FIFO
/// input: frame 1 is published (1, 0), the slot is released (0, 0), frame 2
/// is published, and the reads serve frames 1 and 2 and then find the queue
/// empty. `block_drains_clipped()` reads 0 after every read, and no line of
/// the clip report is logged at any level. A clip reckoned without the
/// release would count every one of those drains and log the loud head.
#[test]
#[traced_test]
fn a_drain_of_a_released_slot_reports_no_clip() {
    // How many captured lines carry `text`, at any level.
    let lines_with = |text: &str| {
        let n = std::cell::Cell::new(0usize);
        logs_assert(|lines: &[&str]| {
            n.set(lines.iter().filter(|l| l.contains(text)).count());
            Ok(())
        });
        n.get()
    };
    let latest = CreditWord::local(2);
    let (_latest_mgr, mut publisher, mut sub) = counting_edge("released_latest", &latest, false);
    publish_x(&mut publisher, 1);
    publish_x(&mut publisher, 2);
    assert_eq!(counts(&latest), (2, 0), "frames 1 and 2 are counted");
    assert!(latest.release_slot(), "the release takes the slot");
    assert_eq!(counts(&latest), (0, 0), "and zeroes its counts");
    publish_x(&mut publisher, 3);
    assert_eq!(counts(&latest), (0, 0), "the released slot counts nothing");
    assert_eq!(
        read_x(&mut sub),
        Some(3),
        "the read drains frames 2 and 3 and serves the newest"
    );
    assert_eq!(counts(&latest), (0, 0));
    assert_eq!(
        sub.block_drains_clipped(),
        0,
        "a drain of the released slot is no clip"
    );
    publish_x(&mut publisher, 4);
    assert_eq!(read_x(&mut sub), Some(4), "frame 4 is delivered");
    assert_eq!(counts(&latest), (0, 0));
    assert_eq!(
        sub.block_drains_clipped(),
        0,
        "and neither is the drain of a frame sent after the release"
    );
    assert!(latest.is_released(), "the slot stays released");

    let fifo = CreditWord::local(2);
    let (_fifo_mgr, mut publisher, mut sub) = counting_edge("released_fifo", &fifo, true);
    publish_x(&mut publisher, 1);
    assert_eq!(counts(&fifo), (1, 0), "frame 1 is counted");
    assert!(fifo.release_slot());
    publish_x(&mut publisher, 2);
    assert_eq!(read_x(&mut sub), Some(1), "frame 1");
    assert_eq!(read_x(&mut sub), Some(2), "frame 2");
    assert_eq!(read_x(&mut sub), None, "and the queue is empty");
    assert_eq!(counts(&fifo), (0, 0));
    assert_eq!(
        sub.block_drains_clipped(),
        0,
        "no FIFO drain of the released slot clips"
    );

    assert_eq!(
        lines_with(CLIP_REPORT),
        0,
        "no line of the clip report, at any level"
    );
}

/// A frame a LOCAL word never counted is reported by the drain that finds the
/// queue empty, the read a latest input makes. Its word's queued count and
/// backlog are already 0 there, so the drain and the empty-queue resync leave
/// the word as it was and its RMW stores nothing; the clip is reported all the
/// same.
///
/// A writer the word does not count (a second writer on a
/// `multi_publisher_topics` topic is one) sends frames 1 and 2 to a `block`
/// input on a depth-2 local word over a real depth-2 queue, and each is read
/// on its own. Oracle, by hand: the read of frame 1 clips (1) and is the loud
/// head of the input's first regime; the read of frame 2 clips again (2)
/// before any clean drain closed that regime, so it is a counted repeat. One
/// loud line, one repeat at `debug`, no recovery line, and the word reads (0, 0)
/// after each read.
#[test]
#[traced_test]
fn a_local_word_reports_the_drain_of_a_frame_it_never_counted() {
    // How many captured lines carry `text`, at any level.
    let lines_with = |text: &str| {
        let n = std::cell::Cell::new(0usize);
        logs_assert(|lines: &[&str]| {
            n.set(lines.iter().filter(|l| l.contains(text)).count());
            Ok(())
        });
        n.get()
    };
    let word = CreditWord::local(2);
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "bsr_uncounted".to_string(),
            clock: Arc::new(VirtualClock::new()),
            subscriber_buffer_size: 2,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("per-test SHM transport");
    let topic = "bsr_uncounted_topic";
    let mut outside = mgr
        .create_publisher(topic, MaxSliceLen::try_new(1024).expect("len"), 0)
        .expect("the outside writer's publisher");
    let mut sub = mgr.create_subscriber(topic).expect("subscriber");
    sub.register_block_probe_for_test(
        word.clone(),
        2,
        Arc::new(BackpressureCounters::new()),
        Arc::from("inp"),
        2,
    );

    publish_x(&mut outside, 1);
    assert_eq!(counts(&word), (0, 0), "the outside writer counts nothing");
    assert_eq!(read_x(&mut sub), Some(1), "frame 1");
    assert_eq!(counts(&word), (0, 0));
    assert_eq!(
        sub.block_drains_clipped(),
        1,
        "the drain of frame 1 clipped"
    );
    publish_x(&mut outside, 2);
    assert_eq!(read_x(&mut sub), Some(2), "frame 2");
    assert_eq!(counts(&word), (0, 0));
    assert_eq!(sub.block_drains_clipped(), 2, "and the drain of frame 2");

    assert_eq!(
        (
            lines_with(CLIP_LOUD),
            lines_with(CLIP_RECOVERED),
            lines_with(CLIP_REPEAT)
        ),
        (1, 0, cerulion_core::testing::debug_lines_expected(1)),
        "one loud head, no recovery line, and the second clip a counted repeat (a `debug!` \
         line, so a release build counts none)"
    );
    // The level-free twin: in every build, no repeat line was emitted loud.
    logs_assert(|lines: &[&str]| cerulion_core::testing::never_loud(lines, CLIP_REPEAT));
}

/// One operation of a hand-driven walk over the two slots of a per-set Sync
/// `block` trigger input, as the matcher's ops and the node's read move it.
#[derive(Clone, Copy)]
enum SlotOp {
    /// Publish the next frame when the producer's gate has room.
    Publish,
    /// The `block_timeout_ms` release of the slot.
    Release,
    /// The boundary fill of the head slot (`FillBoundary`).
    Fill,
    /// The refill of the head slot after a fire (`FillRefill`).
    Refill,
    /// The descent's pop of the next frame into the second slot (`NeedStamp`).
    Peek,
    /// `Advance` or `DiscardTie`: the head is skipped and refilled.
    Discard,
    /// The node's read.
    Serve,
    /// The next read of the queue fails.
    FailNextReceive,
}

/// One row of a slot walk, read after its operation: what the operation did,
/// the frames counted against depth, the forgiven frames the consumer still
/// holds, the frames the input's slots hold, and whether the producer may
/// publish.
type SlotRow<'a> = (&'a str, u64, u64, u8, bool);

/// A [`SlotRow`] as [`slot_walk`] records it.
type WalkRow = (String, u64, u64, u8, bool);

/// What one operation of a slot walk signalled: how many times it rang the
/// word's wake epoch, and whether the input queued its `block` event.
type Signals = (u32, bool);

/// Run `ops` on a fresh `block` FIFO input over a real depth-2 iceoryx2
/// queue whose probe counts on `word`, the word the producer's gate reads and
/// its publish counts on. Frames carry `x = 1, 2, ...` in publish order.
fn slot_walk(tag: &str, word: CreditWord, ops: &[SlotOp]) -> Vec<WalkRow> {
    slot_walk_signalled(tag, word, ops).0
}

/// [`slot_walk`], with what each operation signalled. The input's threshold
/// is the depth, 2, so its `block` event opens a regime at a drain that
/// arrives with 2 frames counted and rearms at one that arrives with fewer.
fn slot_walk_signalled(
    tag: &str,
    word: CreditWord,
    ops: &[SlotOp],
) -> (Vec<WalkRow>, Vec<Signals>) {
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("bsr_slot_{tag}"),
            clock: Arc::new(cerulion_core::clock::RealClock),
            subscriber_buffer_size: 2,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("per-test SHM transport");
    let topic = format!("bsr_slot_{tag}");
    let mut publisher = mgr
        .create_publisher(&topic, MaxSliceLen::try_new(1024).expect("len"), 0)
        .expect("publisher");
    let mut sub = mgr.create_subscriber(&topic).expect("subscriber");
    sub.register_block_probe_for_test(
        word.clone(),
        2,
        Arc::new(BackpressureCounters::new()),
        Arc::from("inp"),
        2,
    );
    sub.mark_fifo_consume_for_test();
    let site = cerulion_core::SyncAlignSite::Boundary;
    let mut next = 1u32;
    let mut rows = Vec::new();
    let mut signals = Vec::new();
    for op in ops {
        let rings_before = word.wake_seq_snapshot();
        let what = match op {
            SlotOp::Publish if word.try_claim() => {
                publish_counted(&mut publisher, &word, next);
                next += 1;
                format!("publish {}", next - 1)
            }
            SlotOp::Publish => format!("held at {next}"),
            SlotOp::Release => format!("release -> {}", word.forgive_outstanding()),
            SlotOp::Fill => format!("fill -> {}", sub.snapshot_latest_for_trigger().0),
            SlotOp::Refill => format!("refill -> {}", sub.refill_for_trigger().0),
            SlotOp::Peek => format!(
                "peek -> {}",
                sub.sync_peek_next_stamp_for_test(site)
                    .expect("the peek does not fail")
                    .is_some()
            ),
            SlotOp::Discard => format!(
                "discard -> {}",
                sub.sync_discard_head_for_test(site)
                    .expect("a discard never fails")
                    .is_some()
            ),
            SlotOp::Serve => format!(
                "serve -> {:?}",
                sub.try_view::<Vector3, u32>(|v| v.x as u32)
                    .expect("the read does not fail")
            ),
            SlotOp::FailNextReceive => {
                sub.fault_inject_receive_after_for_test(0);
                "fail next receive".to_string()
            }
        };
        let event = sub.try_take_backpressure_event();
        if let Some(e) = &event {
            assert_eq!(
                (e.policy, e.dropped),
                (BackpressurePolicy::Block, 0),
                "{what}: a `block` input queues only its lossless event"
            );
        }
        signals.push((
            word.wake_seq_snapshot().wrapping_sub(rings_before),
            event.is_some(),
        ));
        rows.push((
            what,
            word.outstanding(),
            word.forgiven(),
            sub.block_slot_debt_for_test(),
            word.try_claim(),
        ));
    }
    (rows, signals)
}

/// Every operation of a slot walk signalled what its hand-written oracle
/// says, row for row.
fn assert_walk_signals(rows: &[WalkRow], signals: &[Signals], oracle: &[Signals]) {
    for (i, (got, want)) in signals.iter().zip(oracle).enumerate() {
        assert_eq!(
            got, want,
            "row {i} ({}): (wake rings, block event queued)",
            rows[i].0
        );
    }
    assert_eq!(signals.len(), oracle.len(), "one signal row per operation");
}

/// Every row of a slot walk equals its hand-written oracle row.
fn assert_slot_walk(rows: &[WalkRow], oracle: &[SlotRow<'_>]) {
    for (i, (row, want)) in rows.iter().zip(oracle).enumerate() {
        let got: SlotRow<'_> = (row.0.as_str(), row.1, row.2, row.3, row.4);
        assert_eq!(
            got, *want,
            "row {i}: (operation, counted, forgiven, held slots, producer room)"
        );
    }
    assert_eq!(rows.len(), oracle.len(), "one row per operation");
}

/// The trigger refill after a timeout release holds frame 2, which the
/// release forgave, and frames 3 and 4 are the two published after it.
const A_FORGIVEN_REFILL: [SlotOp; 13] = [
    SlotOp::Publish,
    SlotOp::Fill,
    SlotOp::Publish,
    SlotOp::Release,
    SlotOp::Publish,
    SlotOp::Serve,
    SlotOp::Refill,
    SlotOp::Publish,
    SlotOp::Publish,
    SlotOp::Serve,
    SlotOp::Serve,
    SlotOp::Serve,
    SlotOp::Serve,
];

/// The hand oracle of [`A_FORGIVEN_REFILL`] at depth 2. The release forgives
/// frame 1 (held) and frame 2 (queued). The refill pops frame 2 and pays the
/// forgiven backlog, so frame 2 sits in the slot forgiven: the producer has
/// room for frame 4 and is held at frame 5, when frames 3 and 4 are unread.
const A_FORGIVEN_REFILL_ORACLE: [SlotRow<'static>; 13] = [
    ("publish 1", 1, 0, 0, true),
    ("fill -> 1", 1, 0, 1, true),
    ("publish 2", 2, 0, 1, false),
    ("release -> 2", 0, 2, 1, true),
    ("publish 3", 1, 2, 1, true),
    ("serve -> Some(1)", 1, 1, 0, true),
    ("refill -> 1", 1, 1, 1, true),
    ("publish 4", 2, 1, 1, false),
    ("held at 5", 2, 1, 1, false),
    ("serve -> Some(2)", 2, 0, 0, false),
    ("serve -> Some(3)", 1, 0, 0, true),
    ("serve -> Some(4)", 0, 0, 0, true),
    ("serve -> None", 0, 0, 0, true),
];

/// After a timeout release, a frame the release forgave stays forgiven when
/// the trigger refill moves it from the queue into the head slot: the
/// producer may publish `depth` frames after the release before it is held,
/// and the consumer reads every one.
#[test]
fn a_forgiven_frame_the_trigger_refill_holds_leaves_the_producer_its_depth() {
    let rows = slot_walk("refill", CreditWord::local(2), &A_FORGIVEN_REFILL);
    assert_slot_walk(&rows, &A_FORGIVEN_REFILL_ORACLE);
}

/// Both slots hold frames the release forgave: the fill holds frame 1 and
/// the descent's peek frame 2, each forgiven, so the producer publishes
/// frames 3 and 4 and is held at 5. Skipping the head frees one forgiven
/// frame and leaves frames 3 and 4 counted.
#[test]
fn two_forgiven_frames_the_matcher_holds_do_not_hold_the_producer() {
    let rows = slot_walk(
        "both",
        CreditWord::local(2),
        &[
            SlotOp::Publish,
            SlotOp::Publish,
            SlotOp::Release,
            SlotOp::Fill,
            SlotOp::Peek,
            SlotOp::Publish,
            SlotOp::Publish,
            SlotOp::Publish,
            SlotOp::Discard,
            SlotOp::Serve,
            SlotOp::Serve,
            SlotOp::Serve,
            SlotOp::Serve,
        ],
    );
    assert_slot_walk(
        &rows,
        &[
            ("publish 1", 1, 0, 0, true),
            ("publish 2", 2, 0, 0, false),
            ("release -> 2", 0, 2, 0, true),
            ("fill -> 1", 0, 2, 1, true),
            ("peek -> true", 0, 2, 2, true),
            ("publish 3", 1, 2, 2, true),
            ("publish 4", 2, 2, 2, false),
            ("held at 5", 2, 2, 2, false),
            ("discard -> true", 2, 1, 1, false),
            ("serve -> Some(2)", 2, 0, 0, false),
            ("serve -> Some(3)", 1, 0, 0, true),
            ("serve -> Some(4)", 0, 0, 0, true),
            ("serve -> None", 0, 0, 0, true),
        ],
    );
}

/// The matcher skips a head the release forgave and refills it from the
/// queue with frame 3, published after the release (frame 4 evicted the
/// forgiven frame 2): the skipped frame leaves forgiven and frame 3 enters
/// counted, so frames 3 and 4 hold the producer and both are read. The skip
/// rings the wake word once, at the refill's pop: the head's exit and the
/// refill's entry are one update that raises the count and frees no credit.
/// The refill's drain arrives with frames 3 and 4 counted, so it opens the
/// input's `block` regime.
#[test]
fn a_skipped_forgiven_head_refilled_with_a_new_frame_keeps_that_frame_counted() {
    let (rows, signals) = slot_walk_signalled(
        "skip",
        CreditWord::local(2),
        &[
            SlotOp::Publish,
            SlotOp::Fill,
            SlotOp::Publish,
            SlotOp::Release,
            SlotOp::Publish,
            SlotOp::Publish,
            SlotOp::Discard,
            SlotOp::Publish,
            SlotOp::Serve,
            SlotOp::Serve,
            SlotOp::Serve,
        ],
    );
    assert_slot_walk(
        &rows,
        &[
            ("publish 1", 1, 0, 0, true),
            ("fill -> 1", 1, 0, 1, true),
            ("publish 2", 2, 0, 1, false),
            ("release -> 2", 0, 2, 1, true),
            ("publish 3", 1, 2, 1, true),
            ("publish 4", 2, 1, 1, false),
            ("discard -> true", 2, 0, 1, false),
            ("held at 5", 2, 0, 1, false),
            ("serve -> Some(3)", 1, 0, 0, true),
            ("serve -> Some(4)", 0, 0, 0, true),
            ("serve -> None", 0, 0, 0, true),
        ],
    );
    assert_walk_signals(
        &rows,
        &signals,
        &[
            (0, false),
            (1, false),
            (0, false),
            (1, false),
            (0, false),
            (0, false),
            (1, true),
            (0, false),
            (1, false),
            (1, false),
            (0, false),
        ],
    );
}

/// After a release, a refill whose queue read fails holds nothing and
/// changes no count; the next boundary fill pops frame 2, which the release
/// forgave, and holds it forgiven.
#[test]
fn an_errored_refill_after_a_release_holds_nothing_and_the_next_fill_holds_its_frame_forgiven() {
    let rows = slot_walk(
        "err",
        CreditWord::local(2),
        &[
            SlotOp::Publish,
            SlotOp::Fill,
            SlotOp::Publish,
            SlotOp::Release,
            SlotOp::Publish,
            SlotOp::Serve,
            SlotOp::FailNextReceive,
            SlotOp::Refill,
            SlotOp::Fill,
            SlotOp::Publish,
            SlotOp::Publish,
            SlotOp::Serve,
            SlotOp::Serve,
            SlotOp::Serve,
            SlotOp::Serve,
        ],
    );
    assert_slot_walk(
        &rows,
        &[
            ("publish 1", 1, 0, 0, true),
            ("fill -> 1", 1, 0, 1, true),
            ("publish 2", 2, 0, 1, false),
            ("release -> 2", 0, 2, 1, true),
            ("publish 3", 1, 2, 1, true),
            ("serve -> Some(1)", 1, 1, 0, true),
            ("fail next receive", 1, 1, 0, true),
            ("refill -> 0", 1, 1, 0, true),
            ("fill -> 1", 1, 1, 1, true),
            ("publish 4", 2, 1, 1, false),
            ("held at 5", 2, 1, 1, false),
            ("serve -> Some(2)", 2, 0, 0, false),
            ("serve -> Some(3)", 1, 0, 0, true),
            ("serve -> Some(4)", 0, 0, 0, true),
            ("serve -> None", 0, 0, 0, true),
        ],
    );
}

/// Two runs of the forgiven-refill walk on fresh transports each equal the
/// hand oracle, so they equal each other row for row.
#[test]
fn a_forgiven_hold_walk_matches_its_oracle_on_two_runs() {
    let first = slot_walk("det_a", CreditWord::local(2), &A_FORGIVEN_REFILL);
    let second = slot_walk("det_b", CreditWord::local(2), &A_FORGIVEN_REFILL);
    assert_slot_walk(&first, &A_FORGIVEN_REFILL_ORACLE);
    assert_slot_walk(&second, &A_FORGIVEN_REFILL_ORACLE);
    assert_eq!(first, second, "the two runs are row-for-row identical");
}

/// On a cross-process word, which no release forgives, a skipped head the
/// matcher refills from the queue keeps the counts a held frame always had:
/// frame 2 replaces frame 1 in the head slot and the word counts it alone.
/// The skip rings the wake word once, at the refill's pop: the head's exit
/// and the refill's entry, a counted frame for a counted one, change no
/// count. Its drain arrives with frames 1 and 2 counted, so it opens the
/// input's `block` regime.
#[test]
fn a_mapped_word_counts_a_refilled_head_as_held() {
    let ns = format!("bsr_{}_slot_mapped", std::process::id());
    let id = credit_edge_id("/bsr/slot/mapped", "consumer", "inp");
    let _owner = MappedCredit::create_owned(&ns, &id, 2).expect("the supervisor's word");
    let word = CreditWord::mapped(Arc::new(
        MappedCredit::open_unowned(&ns, &id).expect("the consumer's mapping"),
    ));
    let (rows, signals) = slot_walk_signalled(
        "mapped",
        word,
        &[
            SlotOp::Publish,
            SlotOp::Fill,
            SlotOp::Publish,
            SlotOp::Publish,
            SlotOp::Discard,
            SlotOp::Publish,
            SlotOp::Serve,
            SlotOp::Serve,
            SlotOp::Serve,
        ],
    );
    assert_slot_walk(
        &rows,
        &[
            ("publish 1", 1, 0, 0, true),
            ("fill -> 1", 1, 0, 1, true),
            ("publish 2", 2, 0, 1, false),
            ("held at 3", 2, 0, 1, false),
            ("discard -> true", 1, 0, 1, true),
            ("publish 3", 2, 0, 1, false),
            ("serve -> Some(2)", 1, 0, 0, true),
            ("serve -> Some(3)", 0, 0, 0, true),
            ("serve -> None", 0, 0, 0, true),
        ],
    );
    assert_walk_signals(
        &rows,
        &signals,
        &[
            (0, false),
            (1, false),
            (0, false),
            (0, false),
            (1, true),
            (0, false),
            (1, false),
            (1, false),
            (0, false),
        ],
    );
}

/// The consumer's `block` regime walk: a full slot, a skipped head, the
/// descent's peek, then a drain below the slot and a full one.
const A_SKIP_IN_A_REGIME: [SlotOp; 16] = [
    SlotOp::Publish,
    SlotOp::Publish,
    SlotOp::Fill,
    SlotOp::Discard,
    SlotOp::Publish,
    SlotOp::Peek,
    SlotOp::Serve,
    SlotOp::Fill,
    SlotOp::Serve,
    SlotOp::Publish,
    SlotOp::Fill,
    SlotOp::Publish,
    SlotOp::Peek,
    SlotOp::Serve,
    SlotOp::Serve,
    SlotOp::Serve,
];

/// The hand oracle of [`A_SKIP_IN_A_REGIME`]'s counts at depth 2.
const A_SKIP_IN_A_REGIME_ORACLE: [SlotRow<'static>; 16] = [
    ("publish 1", 1, 0, 0, true),
    ("publish 2", 2, 0, 0, false),
    ("fill -> 1", 2, 0, 1, false),
    ("discard -> true", 1, 0, 1, true),
    ("publish 3", 2, 0, 1, false),
    ("peek -> true", 2, 0, 2, false),
    ("serve -> Some(2)", 1, 0, 1, true),
    ("fill -> 1", 1, 0, 1, true),
    ("serve -> Some(3)", 0, 0, 0, true),
    ("publish 4", 1, 0, 0, true),
    ("fill -> 1", 1, 0, 1, true),
    ("publish 5", 2, 0, 1, false),
    ("peek -> true", 2, 0, 2, false),
    ("serve -> Some(4)", 1, 0, 1, true),
    ("serve -> Some(5)", 0, 0, 0, true),
    ("serve -> None", 0, 0, 0, true),
];

/// The hand oracle of what [`A_SKIP_IN_A_REGIME`]'s operations signal. The
/// fill opens the regime (its drain arrives with 2 frames counted). The
/// skip's refill drain arrives with the skipped head still counted, so it
/// stays in that regime and rings once, at its pop, and the peek of frame 3
/// queues no second event. The fill of frame 4 arrives with 1 frame counted
/// and rearms, and the peek of frame 5 opens a fresh regime. The fill that
/// promotes frame 3 drains nothing and rings nothing.
const A_SKIP_IN_A_REGIME_SIGNALS: [Signals; 16] = [
    (0, false),
    (0, false),
    (1, true),
    (1, false),
    (0, false),
    (1, false),
    (1, false),
    (0, false),
    (1, false),
    (0, false),
    (1, false),
    (0, false),
    (1, true),
    (1, false),
    (1, false),
    (0, false),
];

/// A skipped head's refill does not reopen the consumer's `block` regime: the
/// refill's drain reads the slot with the skipped head still counted, so a
/// skip inside a full stretch queues no second event for it, and the head's
/// exit and the refill's entry ring the wake word only at the refill's pop.
/// Twice on fresh transports, each against the hand oracle.
#[test]
fn a_skipped_head_refill_does_not_reopen_the_consumers_block_regime() {
    for run in ["regime_a", "regime_b"] {
        let (rows, signals) = slot_walk_signalled(run, CreditWord::local(2), &A_SKIP_IN_A_REGIME);
        assert_slot_walk(&rows, &A_SKIP_IN_A_REGIME_ORACLE);
        assert_walk_signals(&rows, &signals, &A_SKIP_IN_A_REGIME_SIGNALS);
    }
}

/// A skipped head whose refill read fails leaves its slot: the word pays it
/// and rings once, the input holds nothing, and the next fill pops the
/// queued frame.
#[test]
fn a_skipped_head_whose_refill_fails_leaves_its_slot_and_rings_once() {
    let (rows, signals) = slot_walk_signalled(
        "skip_err",
        CreditWord::local(2),
        &[
            SlotOp::Publish,
            SlotOp::Fill,
            SlotOp::Publish,
            SlotOp::FailNextReceive,
            SlotOp::Discard,
            SlotOp::Fill,
            SlotOp::Serve,
            SlotOp::Serve,
        ],
    );
    assert_slot_walk(
        &rows,
        &[
            ("publish 1", 1, 0, 0, true),
            ("fill -> 1", 1, 0, 1, true),
            ("publish 2", 2, 0, 1, false),
            ("fail next receive", 2, 0, 1, false),
            ("discard -> false", 1, 0, 0, true),
            ("fill -> 1", 1, 0, 1, true),
            ("serve -> Some(2)", 0, 0, 0, true),
            ("serve -> None", 0, 0, 0, true),
        ],
    );
    assert_walk_signals(
        &rows,
        &signals,
        &[
            (0, false),
            (1, false),
            (0, false),
            (0, false),
            (1, false),
            (1, false),
            (1, false),
            (0, false),
        ],
    );
}

/// The refusal of `block_timeout_ms` on `consumer.inp` of `topic` that a
/// multi-process worker's build gives when its plan binds the input to a
/// credit word whose producer `producer` runs in another process.
fn split_timeout_refusal(topic: &str, producer: &str) -> String {
    format!(
        "Graph validation error: input 'consumer.inp' on topic '{topic}' sets \
         `block_timeout_ms`, but the input runs in this process and its producer \
         '{producer}' in another process, so the two fire on different threads and count the \
         edge on one shared credit word. A timeout release keeps that count exact only when \
         the producer and the consumer fire one at a time on one thread: here a read and a \
         publish can interleave after a release, so the slot can count a frame that is gone \
         and hold the producer with nothing queued, or miss one and let a frame published \
         after the release be evicted. Put both nodes in ONE `process_groups:` group, or run \
         `--single-process`, to keep the timeout; or remove `block_timeout_ms`, and the slot \
         is still released when the consumer stops for good (a fatal error, a tick panic, or \
         its process dying)"
    )
}

/// Build the graph of a multi-process worker that runs only `consumer`, whose
/// `block` input `inp` reads `/{prefix}/producer/out` with `timeout` as its
/// `block_timeout_ms`, with the plan's binding of that input to a mapped
/// credit word whose producer `producer` runs in another process.
fn build_split_consumer(prefix: &str, timeout: Option<u64>) -> Result<GraphRuntime, String> {
    let topic = format!("/{prefix}/producer/out");
    let mut inp = input(&topic);
    inp.block_timeout_ms = timeout;
    let cfg = config(
        prefix,
        vec![node("consumer", "host_block", vec![inp], vec![])],
    );
    let ns = format!("bsr_{}_{prefix}", std::process::id());
    let id = credit_edge_id(&topic, "consumer", "inp");
    let _owner = MappedCredit::create_owned(&ns, &id, 2).expect("the supervisor's word");
    let word = Arc::new(MappedCredit::open_unowned(&ns, &id).expect("the consumer's mapping"));
    let bind = [cerulion_core::graph::CreditBinding {
        topic: topic.clone(),
        consumer_node: "consumer".to_string(),
        consumer_input: "inp".to_string(),
        depth: 2,
        role: cerulion_core::graph::CreditRole::Consumer,
        word: CreditWord::mapped(word),
        producer_slot: None,
        producer_nodes: vec!["producer".to_string()],
    }];
    let factories = factories_for(&cfg);
    GraphRuntime::build_for_test_with_policy_and_wiring(
        cfg,
        factories,
        Arc::new(VirtualClock::new()),
        64,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::with_credit(None, &bind),
    )
    .map_err(|e| e.to_string())
}

/// A multi-process worker refuses `block_timeout_ms` on a `block` input its
/// plan binds to a mapped credit word, the word whose producer runs in
/// another process, naming the edge, both sides and the remedies. The same
/// input without the key builds on the same binding.
#[test]
fn block_timeout_on_a_split_edge_is_refused_at_the_consumers_build() {
    let err = build_split_consumer("bsr_split_to", Some(50))
        .err()
        .expect("the consumer's worker must refuse a timeout on a mapped word");
    assert_eq!(
        err,
        split_timeout_refusal("/bsr_split_to/producer/out", "producer")
    );
    let rt = build_split_consumer("bsr_split_plain", None)
        .expect("the same input without block_timeout_ms builds on the mapped word");
    let credit = rt
        .block_credits()
        .find(|c| c.consumer_node == "consumer")
        .expect("the bound input reports its credit");
    assert!(
        credit.mapped && !credit.block_timeout,
        "the input counts on the mapped word and carries no timeout: {credit:?}"
    );
}

/// A multi-process worker refuses `block_timeout_ms` per edge, not per topic
/// or per plan. This worker runs the producer `src` and its `block` consumer
/// `near` (depth 2, `block_timeout_ms: 50`), and its plan binds the same
/// topic's consumer `far` in another process to a mapped word (depth 2), as
/// in an executor unit that holds a producer and one of its consumers. The
/// worker builds. `src` fires twice and fills both slots. `near` counts on a
/// local word with its timeout, and once it has been full for 50 ms its slot
/// is released exactly once: the mapped word still holds `src`, so `near`'s
/// slot never fills again. The mapped word keeps its 2 frames, never forgiven or
/// released. One drain on the far side frees `src` to fire a third frame,
/// which `near`'s slot counts alone.
#[test]
fn a_timeout_beside_a_mapped_edge_of_its_topic_builds_and_is_released_once() {
    let prefix = "bsr_split_near";
    let topic = format!("/{prefix}/src/out");
    let cfg = config(
        prefix,
        vec![
            node("src", "producer", vec![], vec![output("out")]),
            node(
                "near",
                "host_block",
                vec![with_timeout(input("src/out"), 50)],
                vec![],
            ),
        ],
    );
    let ns = format!("bsr_{}_{prefix}", std::process::id());
    let id = credit_edge_id(&topic, "far", "inp");
    let _owner = MappedCredit::create_owned(&ns, &id, 2).expect("the supervisor's word");
    let producer_side =
        Arc::new(MappedCredit::open_unowned(&ns, &id).expect("the producer's mapping"));
    let far = MappedCredit::open_unowned(&ns, &id).expect("the far consumer's mapping");
    let bind = [cerulion_core::graph::CreditBinding {
        topic: topic.clone(),
        consumer_node: "far".to_string(),
        consumer_input: "inp".to_string(),
        depth: 2,
        role: cerulion_core::graph::CreditRole::Producer,
        word: CreditWord::mapped(producer_side),
        producer_slot: Some(cerulion_core::credit::ProducerSlot::new(0)),
        producer_nodes: vec!["src".to_string()],
    }];
    let clock = Arc::new(VirtualClock::new());
    let factories = factories_for(&cfg);
    let mut rt = GraphRuntime::build_for_test_with_policy_and_wiring(
        cfg,
        factories,
        Arc::clone(&clock),
        64,
        cerulion_core::MonitorWaitPolicy::off(),
        CrossProcessWiring::with_credit(None, &bind),
    )
    .expect("a local timeout edge beside a mapped edge of its topic builds");
    let near = rt
        .block_credits()
        .find(|c| c.consumer_node == "near")
        .expect("the local input reports its credit");
    assert!(
        !near.mapped && near.block_timeout,
        "the local input counts on a local word and keeps its timeout: {near:?}"
    );
    // The build is a free-run one: it reads the clock and never advances it,
    // so each step sets the clock one producer period on.
    fn step(rt: &mut GraphRuntime, clock: &VirtualClock, k: u64) {
        clock.set((k + 1) * 10_000_000);
        rt.step(Duration::from_millis(10));
    }
    for k in 0..STEPS {
        step(&mut rt, &clock, k);
    }
    assert_eq!(fires(&rt, "src"), 2, "both depth-2 slots filled, then held");
    assert_eq!(releases(&rt, "near"), 1, "the timed-out local slot, once");
    assert_eq!(outstanding(&rt, "near"), 0, "its 2 frames were forgiven");
    assert_eq!(
        (far.outstanding(), far.forgiven(), far.is_released()),
        (2, 0, false),
        "the mapped word still counts its 2 frames: nothing forgave or released it"
    );
    far.record_drained(1);
    step(&mut rt, &clock, STEPS);
    assert_eq!(
        fires(&rt, "src"),
        3,
        "the far drain freed one frame and the released local slot held nothing"
    );
    assert_eq!(
        outstanding(&rt, "near"),
        1,
        "the local slot counts the one frame published after its release"
    );
    assert_eq!(releases(&rt, "near"), 1, "and is not released again");
}

/// One `block` FIFO input fed by two publishers over real depth-2 iceoryx2
/// queues, one per connection, so the word is stamped two connections and
/// four queue places. Only the first publisher publishes. Its frames 1 and 2
/// fill the slot and a timeout release forgives them; its frames 3 and 4 then
/// evict 1 and 2 from its own queue, which the backlog bound (four places)
/// cannot see. The consumer reads 3 and 4, whose drains pay the stale
/// backlog, and a third read finds the queue empty: that read must leave the
/// slot with room, and the producer and the consumer then keep pace.
#[test]
fn a_fifo_read_that_finds_the_queue_empty_clears_a_count_a_release_left_high() {
    let word = CreditWord::local(2);
    word.set_connections(2);
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "bsr_resync_fifo".to_string(),
            clock: Arc::new(cerulion_core::clock::RealClock),
            subscriber_buffer_size: 2,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("per-test SHM transport");
    let topic = "bsr_resync_fifo_topic";
    let len = MaxSliceLen::try_new(1024).expect("len");
    let mut first = mgr
        .create_publisher(topic, len, 0)
        .expect("first publisher");
    let _second = mgr
        .create_publisher(topic, len, 0)
        .expect("second publisher");
    let mut sub = mgr.create_subscriber(topic).expect("subscriber");
    sub.register_block_probe_for_test(
        word.clone(),
        2,
        Arc::new(BackpressureCounters::new()),
        Arc::from("inp"),
        2,
    );
    sub.mark_fifo_consume_for_test();
    let read = |sub: &mut CerulionSubscriber| {
        sub.try_view::<Vector3, u32>(|v| v.x as u32)
            .expect("the FIFO read does not fail")
    };

    for x in 1..=2 {
        publish_counted(&mut first, &word, x);
    }
    assert_eq!(word.forgive_outstanding(), 2, "both frames are forgiven");
    for x in 3..=4 {
        assert!(word.try_claim(), "frame {x} fits after the release");
        publish_counted(&mut first, &word, x);
    }
    assert!(!word.try_claim(), "frames 3 and 4 hold the producer");
    assert_eq!(read(&mut sub), Some(3), "frames 1 and 2 were evicted");
    assert_eq!(read(&mut sub), Some(4));
    assert_eq!(read(&mut sub), None, "the queue is empty");
    assert_eq!(
        (word.outstanding(), word.forgiven()),
        (0, 0),
        "the read that found the queue empty leaves no count behind"
    );
    for x in 5..=10 {
        assert!(
            word.try_claim(),
            "frame {x}: a consumer that keeps up never leaves the slot full"
        );
        publish_counted(&mut first, &word, x);
        assert_eq!(read(&mut sub), Some(x), "frame {x} is received");
        assert_eq!(read(&mut sub), None);
    }
    assert_eq!(word.outstanding(), 0);
}

/// A topic `/{prefix}/shared` listed in `multi_publisher_topics:`: `first`
/// (a 10 ms producer) and, when `two`, `quiet` (an `external` producer the
/// test never triggers) publish it, and `consumer`, of type `consumer_ty`,
/// reads it through the input `inp` builds from the topic name.
fn shared_topic(
    prefix: &str,
    two: bool,
    consumer_ty: &str,
    inp: impl FnOnce(&str) -> InputDef,
) -> GraphConfig {
    let topic = format!("/{prefix}/shared");
    let mut nodes = vec![node(
        "first",
        "producer",
        vec![],
        vec![output_on("out", &topic)],
    )];
    if two {
        nodes.push(node(
            "quiet",
            "quiet_producer",
            vec![],
            vec![output_on("out", &topic)],
        ));
    }
    nodes.push(node("consumer", consumer_ty, vec![inp(&topic)], vec![]));
    let mut cfg = config(prefix, nodes);
    cfg.multi_publisher_topics = vec![topic];
    cfg
}

/// The load refusal of `block_timeout_ms` on `consumer.inp` of the listed
/// `topic`, whose in-graph producers `producers` names.
fn shared_timeout_refusal(topic: &str, producers: &str) -> String {
    format!(
        "Graph validation error: input 'consumer.inp' on topic '{topic}' sets \
         `block_timeout_ms`, but the topic {producers} and is listed in \
         `multi_publisher_topics:`, which lets publishers outside the graph write it. A \
         timeout release keeps an exact count only on a topic with one producer: each \
         producer's connection is its own queue, so on this topic a release can leave the \
         slot's count high and release a consumer that keeps up again and again. Keep one \
         producer on the topic and take it out of `multi_publisher_topics:`; or remove \
         `block_timeout_ms`, and the slot is still released when the consumer stops for good \
         (a fatal error, a tick panic, or its process dying)"
    )
}

/// `block_timeout_ms` on a `block` input of a topic with several producers is
/// refused at load with one text: with the input `block` by the node's
/// attribute or by the graph's `backpressure:` key, with two in-graph
/// producers or with one on a listed topic, on the single-process build, the
/// multi-process supervisor's planning build and the free-run live build of
/// the whole graph. The same graphs without the key build.
#[test]
fn block_timeout_on_a_topic_with_several_producers_is_refused_at_load() {
    let timed = |topic: &str| with_timeout(input(topic), 50);
    let keyed = |topic: &str| {
        let mut i = with_timeout(input(topic), 50);
        i.backpressure = Some(InputBackpressure::Block);
        i
    };
    let two = "has 2 in-graph producers ('first', 'quiet')";
    let one = "has 1 in-graph producer ('first')";
    let forms = [
        (
            "the attribute, two producers",
            shared_topic("bsr_mp_attr", true, "host_block", timed),
            two,
        ),
        (
            "the graph key, two producers",
            shared_topic("bsr_mp_key", true, "host_drop_oldest", keyed),
            two,
        ),
        (
            "the attribute, one producer on a listed topic",
            shared_topic("bsr_mp_one", false, "host_block", timed),
            one,
        ),
    ];
    for (form, cfg, producers) in forms {
        let want = shared_timeout_refusal(&format!("/{}/shared", cfg.prefix), producers);
        let err = build(cfg.clone())
            .err()
            .unwrap_or_else(|| panic!("{form}: the single-process build must refuse"));
        assert_eq!(err, want, "{form}: the single-process build");
        assert_eq!(
            planning_refusal(cfg.clone()).as_deref(),
            Some(want.as_str()),
            "{form}: the planning build"
        );
        let err = build_live(cfg)
            .err()
            .unwrap_or_else(|| panic!("{form}: the live build must refuse"));
        assert_eq!(err, want, "{form}: the live build");
    }
    let keyed_plain = |topic: &str| {
        let mut i = input(topic);
        i.backpressure = Some(InputBackpressure::Block);
        i
    };
    build(shared_topic("bsr_mp_plain", true, "host_block", input))
        .expect("the attribute form without block_timeout_ms builds");
    build(shared_topic(
        "bsr_mp_kplain",
        true,
        "host_drop_oldest",
        keyed_plain,
    ))
    .expect("the graph key form without block_timeout_ms builds");
    assert_eq!(
        planning_refusal(shared_topic("bsr_mp_pplain", true, "host_block", input)),
        None,
        "the planning build of the topic without block_timeout_ms builds"
    );
}

/// A `block` consumer of a topic two producers share, with no
/// `block_timeout_ms`, still has its slot released when it stops for good.
#[test]
fn a_consumer_of_a_shared_topic_that_stops_releases_its_slot() {
    let mut rt = build(shared_topic("bsr_mp_fatal", true, "fatal_block", input)).expect("build");
    let mut held_at = 0;
    for step in 0..STEPS {
        if step == 9 {
            held_at = fires(&rt, "first");
            rt.trigger_external("consumer").expect("trigger");
        }
        rt.step(Duration::from_millis(10));
    }
    assert_eq!(held_at, 2, "precondition: the consumer held first at depth");
    assert_eq!(releases(&rt, "consumer"), 1, "exactly one release");
    let produced = fires(&rt, "first");
    assert!(
        produced >= 85,
        "first resumes after the consumer stops (fires {produced}; held it would stay at 2)"
    );
    assert_eq!(fires(&rt, "quiet"), 0, "the second producer never fires");
}

#[test]
fn a_consumer_that_stops_with_a_fatal_error_releases_its_slot() {
    let mut rt = build(config(
        "bsr_fatal",
        vec![
            node("producer", "producer", vec![], vec![output("out")]),
            node("dies", "fatal_block", vec![input("producer/out")], vec![]),
        ],
    ))
    .expect("build");
    for step in 0..STEPS {
        if step == 9 {
            rt.trigger_external("dies").expect("trigger");
        }
        rt.step(Duration::from_millis(10));
    }
    assert_eq!(releases(&rt, "dies"), 1, "exactly one release");
    let produced = fires(&rt, "producer");
    assert!(
        produced >= 85,
        "the producer resumes after the consumer stops (fires {produced}; held it would stay near 4)"
    );
}

/// The live loop's shortest wait.
const FLOOR: Duration = Duration::from_millis(1);
/// The live loop's longest wait (its liveliness sweep).
const SWEEP: Duration = Duration::from_millis(250);

fn deferred(rt: &GraphRuntime, consumer: &str) -> u64 {
    rt.node_handle(consumer)
        .unwrap()
        .backpressure_block_fires_deferred_count("inp")
}

/// Step until `consumer`'s slot first defers the producer; returns the step
/// count.
fn step_until_deferred(rt: &mut GraphRuntime, consumer: &str) -> u64 {
    for step in 1..=10 {
        rt.step(Duration::from_millis(10));
        if deferred(rt, consumer) > 0 {
            return step;
        }
    }
    panic!("the producer was never deferred on {consumer}");
}

#[test]
fn a_held_producer_waits_for_its_release_deadline_not_the_floor() {
    let mut rt = build(config(
        "bsr_wait_to",
        vec![
            node("producer", "producer", vec![], vec![output("out")]),
            node(
                "stuck",
                "host_block",
                vec![with_timeout(input("producer/out"), 50)],
                vec![],
            ),
        ],
    ))
    .expect("build");
    step_until_deferred(&mut rt, "stuck");
    // The producer's period has passed, so it is due; its slot is full with
    // nothing drained, so it cannot fire before the 50 ms release.
    let first = rt.live_timeout_for_test();
    assert!(
        first > Duration::from_millis(40) && first <= Duration::from_millis(50),
        "the wait runs to the release deadline 50 ms out, not the {FLOOR:?} floor: {first:?}"
    );
    rt.step(Duration::from_millis(10));
    rt.step(Duration::from_millis(10));
    let later = rt.live_timeout_for_test();
    assert!(
        later > Duration::from_millis(20) && later <= Duration::from_millis(30),
        "20 ms later the wait is 20 ms shorter: {later:?}"
    );
    rt.step(Duration::from_millis(10));
    rt.step(Duration::from_millis(10));
    assert_eq!(releases(&rt, "stuck"), 0, "40 ms held is short of 50");
    rt.step(Duration::from_millis(10));
    assert_eq!(
        releases(&rt, "stuck"),
        1,
        "the release lands at the step at the deadline"
    );
}

#[test]
fn a_held_producer_with_no_timeout_waits_for_the_drain_that_frees_it() {
    let mut rt = build(config(
        "bsr_wait_drain",
        vec![
            node("producer", "producer", vec![], vec![output("out")]),
            node("held", "host_block", vec![input("producer/out")], vec![]),
        ],
    ))
    .expect("build");
    step_until_deferred(&mut rt, "held");
    assert_eq!(
        rt.live_timeout_for_test(),
        SWEEP,
        "only the consumer's drain on this level loop frees the slot, so the held producer \
         adds no deadline and the loop does not wake every {FLOOR:?} to defer it again"
    );
    rt.trigger_external("held").expect("trigger");
    rt.step(Duration::from_millis(10));
    assert!(
        outstanding(&rt, "held") < 2,
        "anti-vacuity: the consumer drained"
    );
    let freed = rt.live_timeout_for_test();
    assert!(
        freed <= Duration::from_millis(10),
        "once the slot has room the producer's own period stands: {freed:?}"
    );
}

/// One live-loop iteration, waiting what the loop itself would wait.
fn live_iteration(rt: &mut GraphRuntime) {
    let wait = rt.live_timeout_for_test();
    rt.run_live_step_once_for_test(wait);
}

/// Hand `events` one event and run the live loop until `done` holds.
fn deliver_and_run_until(
    rt: &mut GraphRuntime,
    events: &mpsc::SyncSender<()>,
    what: &str,
    mut done: impl FnMut(&GraphRuntime) -> bool,
) {
    // A rendezvous channel: `send` returns once the helper thread has taken
    // the event, just before it rings the doorbell.
    events.send(()).expect("the helper thread takes the event");
    let start = Instant::now();
    while !done(rt) {
        assert!(
            start.elapsed() < Duration::from_secs(5),
            "the live loop never reached {what}"
        );
        live_iteration(rt);
    }
}

#[test]
fn a_held_external_producer_is_released_at_its_deadline_not_the_sweep() {
    const TIMEOUT_MS: u64 = 10;
    let (events, rx) = mpsc::sync_channel::<()>(0);
    let cfg = config(
        "bsr_ext_wait",
        vec![
            node("producer", "blocking_producer", vec![], vec![output("out")]),
            node(
                "stuck",
                "idle_block",
                vec![with_timeout(input("producer/out"), TIMEOUT_MS)],
                vec![],
            ),
        ],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(BlockingProducerEntry::with_state(BlockingProducer {
            events: Some(rx),
            ..Default::default()
        })),
    );
    factories.insert("stuck".to_string(), Box::new(IdleBlockEntry::new()));
    let mut rt = GraphRuntime::build_for_test(cfg, factories, Arc::new(VirtualClock::new()), 64)
        .expect("build");
    rt.collect_external_sources_for_test();
    assert_eq!(
        rt.live_timeout_for_test(),
        SWEEP,
        "an external producer with no event pending adds no deadline"
    );

    // Two events fill the depth-2 slot, one fire each.
    for k in 1..=2 {
        deliver_and_run_until(&mut rt, &events, "the producer's fire", |rt| {
            fires(rt, "producer") >= k
        });
    }
    assert_eq!(
        outstanding(&rt, "stuck"),
        2,
        "anti-vacuity: the slot is full"
    );
    assert_eq!(
        deferred(&rt, "stuck"),
        0,
        "anti-vacuity: no evaluation has seen the slot full, so its stall clock starts at the next"
    );

    // The third event: the sweep drains the doorbell and marks the node, and
    // the step that would fire it finds the slot full. Nothing else will ring.
    deliver_and_run_until(&mut rt, &events, "the deferral", |rt| {
        deferred(rt, "stuck") > 0
    });
    let held_at = Instant::now();
    assert_eq!(fires(&rt, "producer"), 2, "the third event was deferred");
    assert_eq!(releases(&rt, "stuck"), 0, "and the slot is still held");

    let first_wait = rt.live_timeout_for_test();
    let mut iterations = 0u32;
    while releases(&rt, "stuck") == 0 {
        assert!(
            held_at.elapsed() < Duration::from_secs(5),
            "the held slot was never released"
        );
        live_iteration(&mut rt);
        iterations += 1;
    }
    let delay = held_at.elapsed();
    println!(
        "held external producer, block_timeout_ms = {TIMEOUT_MS}: first wait {first_wait:?}, \
         released after {delay:?} in {iterations} live iteration(s); the sweep is {SWEEP:?}"
    );
    assert!(
        first_wait <= Duration::from_millis(TIMEOUT_MS) && first_wait > FLOOR,
        "the held event bounds the wait at its {TIMEOUT_MS} ms release deadline, not the {SWEEP:?} sweep: {first_wait:?}"
    );
    // Waiting out the sweep instead, with nothing else to wake the loop, the
    // release lands no sooner than 250 ms after the deferral. The bound leaves
    // a loaded host room for late wakes.
    assert!(
        delay < Duration::from_millis(200),
        "the release lands near its {TIMEOUT_MS} ms deadline, before the {SWEEP:?} sweep: {delay:?}"
    );
    assert_eq!(
        fires(&rt, "producer"),
        3,
        "the deferred event fires once, in the step that releases the slot"
    );
    assert_eq!(
        rt.live_timeout_for_test(),
        SWEEP,
        "served, the producer adds no deadline again"
    );
}

#[test]
fn the_graph_key_makes_a_drop_oldest_input_block() {
    let mut i = input("producer/out");
    i.backpressure = Some(InputBackpressure::Block);
    i.depth = Some(3);
    let mut rt = build(config(
        "bsr_yaml",
        vec![
            node("producer", "producer", vec![], vec![output("out")]),
            node("never", "host_drop_oldest", vec![i], vec![]),
        ],
    ))
    .expect("build");
    let produced = Arc::new(AtomicU64::new(0));
    let p = Arc::clone(&produced);
    run(&mut rt, None, |rt| {
        p.store(fires(rt, "producer"), Ordering::Relaxed)
    });
    assert_eq!(
        produced.load(Ordering::Relaxed),
        3,
        "the producer plateaus at the YAML depth: the input is block"
    );
}

#[test]
fn a_ring_of_block_edges_is_refused_at_build() {
    let err = build(config(
        "bsr_ring",
        vec![
            node("a", "relay", vec![input("b/out")], vec![output("out")]),
            node("b", "relay", vec![input("a/out")], vec![output("out")]),
        ],
    ))
    .err()
    .expect("a ring of block edges must be refused");
    assert!(
        err.contains("a ring of `block` edges") && err.contains("a.inp") && err.contains("b.inp"),
        "the refusal names the ring: {err}"
    );
}

#[test]
fn a_ring_broken_by_a_drop_oldest_edge_builds() {
    let mut i = input("a/out");
    i.backpressure = Some(InputBackpressure::DropOldest);
    build(config(
        "bsr_noring",
        vec![
            node("a", "relay", vec![input("b/out")], vec![output("out")]),
            node("b", "relay", vec![i], vec![output("out")]),
        ],
    ))
    .expect("a ring with one drop_oldest edge holds nobody forever");
}

#[test]
fn block_on_a_network_ingress_topic_is_refused() {
    let mut cfg = config(
        "bsr_net",
        vec![node(
            "sink",
            "host_block",
            vec![input("/remote/cmd")],
            vec![],
        )],
    );
    cfg.network = Some(NetworkBlock {
        mode: NetworkMode::Peer,
        ingress: vec!["/remote/cmd".to_string()],
        ..Default::default()
    });
    let err = build(cfg).err().expect("block on ingress must be refused");
    assert!(
        err.contains("network.ingress") && err.contains("sink.inp"),
        "the refusal names the network edge: {err}"
    );
}

#[test]
fn block_timeout_without_block_is_refused() {
    let err = build(config(
        "bsr_badto",
        vec![
            node("producer", "producer", vec![], vec![output("out")]),
            node(
                "d",
                "host_drop_oldest",
                vec![with_timeout(input("producer/out"), 10)],
                vec![],
            ),
        ],
    ))
    .err()
    .expect("block_timeout_ms on a drop_oldest input must be refused");
    assert!(err.contains("block_timeout_ms"), "{err}");
}

#[test]
fn the_graph_cannot_make_an_accumulate_input_block() {
    let accumulate_graph = |i: InputDef| {
        config(
            "bsr_acc",
            vec![
                node("producer", "producer", vec![], vec![output("out")]),
                node("acc", "host_accumulate", vec![i], vec![]),
            ],
        )
    };
    let mut block = input("producer/out");
    block.backpressure = Some(InputBackpressure::Block);
    let err = build(accumulate_graph(block))
        .err()
        .expect("block on an accumulate input must be refused");
    assert!(
        err.contains("acc.inp")
            && err
                .contains("`#[input(accumulate)]` cannot be combined with `backpressure = block`"),
        "the refusal names the input and carries the macro's wording: {err}"
    );
    let mut depth = input("producer/out");
    depth.depth = Some(4);
    let err = build(accumulate_graph(depth))
        .err()
        .expect("depth on an accumulate input must be refused");
    assert!(
        err.contains("acc.inp") && err.contains("takes `cap = N`, not `depth`"),
        "{err}"
    );
    // Control: the same graph without the keys builds.
    build(accumulate_graph(input("producer/out"))).expect("a plain accumulate input builds");
}

/// The message of the load-time warning a mixed `block` topic logs.
const PACED_MESSAGE: &str = "mixed `block` topic: its `drop_oldest` and `sample(N)` consumers \
     receive frames at the pace its `block` consumers allow, because the producer node defers \
     while any `block` consumer's queue is full and a deferred node publishes on none of its \
     outputs. `block_consumers` lists the slowest declared first";

fn with_depth(mut i: InputDef, depth: usize) -> InputDef {
    i.depth = Some(depth);
    i
}

/// The captured lines that carry the paced-topic warning.
fn paced_lines<'a>(lines: &[&'a str]) -> Vec<&'a str> {
    lines
        .iter()
        .copied()
        .filter(|l| l.contains(PACED_MESSAGE))
        .collect()
}

/// Exactly one paced-topic warning line, carrying `want` verbatim.
fn one_paced_line(lines: &[&str], want: &str) -> Result<(), String> {
    match paced_lines(lines).as_slice() {
        [line] if line.contains(want) => Ok(()),
        [line] => Err(format!(
            "the warning line\n  {line}\ndoes not carry\n  {want}"
        )),
        other => Err(format!(
            "expected exactly 1 paced-topic warning line, got {}: {other:#?}",
            other.len()
        )),
    }
}

#[test]
#[traced_test]
fn a_mixed_topic_warns_once_at_load_naming_who_paces_whom() {
    let topic = || input("producer/out");
    let mut rt = build(config(
        "bsr_warn",
        vec![
            node("producer", "producer", vec![], vec![output("out")]),
            node("slow", "host_block", vec![topic()], vec![]),
            node("fast", "fast_drop_oldest", vec![topic()], vec![]),
            node(
                "timed",
                "host_block",
                vec![with_timeout(topic(), 50)],
                vec![],
            ),
            node("sampled", "host_sample", vec![topic()], vec![]),
            node(
                "tight",
                "host_block",
                vec![with_timeout(with_depth(topic(), 1), 200)],
                vec![],
            ),
            node(
                "long",
                "host_block",
                vec![with_timeout(topic(), 500)],
                vec![],
            ),
        ],
    ))
    .expect("build");
    // Smallest depth first; at one depth, no timeout (never released) first,
    // then the longest timeout. The paced consumers keep graph order.
    let want = format!(
        "{PACED_MESSAGE} graph=bsr_warn_graph topic=/bsr_warn/producer/out \
         producer_nodes=producer block_consumers=tight.inp (depth 1, block_timeout_ms 200), \
         slow.inp (depth 2), long.inp (depth 2, block_timeout_ms 500), \
         timed.inp (depth 2, block_timeout_ms 50) \
         paced_consumers=fast.inp (drop_oldest), sampled.inp (sample(15))"
    );
    logs_assert(|lines: &[&str]| one_paced_line(lines, &want));
    run(&mut rt, None, |_| {});
    assert!(
        deferred(&rt, "tight") > 0,
        "the run defers its producer, so the steps below are ones a per-frame or per-defer \
         warning would have logged in"
    );
    // Still one line after a hundred steps of publishes, defers and releases.
    logs_assert(|lines: &[&str]| one_paced_line(lines, &want));
}

#[test]
#[traced_test]
fn only_a_topic_with_block_and_paced_consumers_warns() {
    let mut rt = build(config(
        "bsr_nowarn",
        vec![
            // All `block`.
            node("all_block", "producer", vec![], vec![output("out")]),
            node("b1", "host_block", vec![input("all_block/out")], vec![]),
            node("b2", "host_block", vec![input("all_block/out")], vec![]),
            // No `block`.
            node("no_block", "producer", vec![], vec![output("out")]),
            node(
                "d1",
                "fast_drop_oldest",
                vec![input("no_block/out")],
                vec![],
            ),
            node("s1", "host_sample", vec![input("no_block/out")], vec![]),
            // Mixed.
            node("mixed", "producer", vec![], vec![output("out")]),
            node("b3", "host_block", vec![input("mixed/out")], vec![]),
            node("d2", "fast_drop_oldest", vec![input("mixed/out")], vec![]),
        ],
    ))
    .expect("build");
    run(&mut rt, None, |_| {});
    logs_assert(|lines: &[&str]| {
        one_paced_line(
            lines,
            "topic=/bsr_nowarn/mixed/out producer_nodes=mixed block_consumers=b3.inp (depth 2) \
             paced_consumers=d2.inp (drop_oldest)",
        )
    });
    assert!(
        !logs_contain("topic=/bsr_nowarn/all_block/out producer_nodes")
            && !logs_contain("topic=/bsr_nowarn/no_block/out producer_nodes"),
        "an all-`block` topic and a topic with no `block` consumer log no paced-topic warning"
    );
}

/// The local graph of a multi-process worker that builds the producer of a
/// mixed topic: `block` and `drop_oldest` consumers on the producer's topic.
fn worker_graph(prefix: &str) -> GraphConfig {
    config(
        prefix,
        vec![
            node("producer", "producer", vec![], vec![output("out")]),
            node("slow", "host_block", vec![input("producer/out")], vec![]),
            node(
                "fast",
                "fast_drop_oldest",
                vec![input("producer/out")],
                vec![],
            ),
        ],
    )
}

fn build_worker(
    cfg: GraphConfig,
    planned: &[cerulion_core::graph::PacedTopicWarning],
) -> GraphRuntime {
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for n in &cfg.nodes {
        factories.insert(n.id.clone(), entry(&n.node_type));
    }
    GraphRuntime::build_for_test_with_policy_and_wiring(
        cfg,
        factories,
        Arc::new(VirtualClock::new()),
        64,
        cerulion_core::MonitorWaitPolicy::off(),
        cerulion_core::graph::CrossProcessWiring::none().with_paced_topic_warnings(planned),
    )
    .expect("build")
}

#[test]
#[traced_test]
fn a_worker_logs_its_plans_warning_in_place_of_its_own_view() {
    // The supervisor's whole-graph warning names a consumer in another process
    // group, which this worker's own topology cannot see.
    let planned = vec![cerulion_core::graph::PacedTopicWarning {
        topic: "/bsr_wplan/producer/out".to_string(),
        producer_nodes: vec!["producer".to_string()],
        block_consumers: vec!["slow.inp (depth 2)".to_string()],
        paced_consumers: vec![
            "fast.inp (drop_oldest)".to_string(),
            "remote.inp (sample(40))".to_string(),
        ],
    }];
    let _rt = build_worker(worker_graph("bsr_wplan"), &planned);
    logs_assert(|lines: &[&str]| {
        one_paced_line(
            lines,
            "graph=bsr_wplan_graph topic=/bsr_wplan/producer/out producer_nodes=producer \
             block_consumers=slow.inp (depth 2) \
             paced_consumers=fast.inp (drop_oldest), remote.inp (sample(40))",
        )
    });
}

#[test]
#[traced_test]
fn a_worker_whose_plan_names_no_paced_topic_logs_none() {
    let _rt = build_worker(worker_graph("bsr_wnone"), &[]);
    logs_assert(|lines: &[&str]| match paced_lines(lines).as_slice() {
        [] => Ok(()),
        other => Err(format!(
            "a worker logs its plan's list, not its own view: {other:#?}"
        )),
    });
}
