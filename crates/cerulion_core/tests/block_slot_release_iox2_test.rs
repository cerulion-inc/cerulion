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
//!   publishes receives every frame published after the release, on a local
//!   and on a split word;
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
//! `block_accumulate_refusal_off` and `block_forgiven_drain_spends_credit`
//! each make a row here fail.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{mpsc, Arc};
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::credit::{credit_edge_id, CreditShared, CreditWord, MappedCredit};
use cerulion_core::graph::config::{
    GraphConfig, InputBackpressure, InputDef, NetworkBlock, NetworkMode, NodeDef, OutputDef,
};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::scheduler::BackpressureCounters;
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

fn output(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
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
        other => panic!("no entry for {other}"),
    }
}

fn build(cfg: GraphConfig) -> Result<GraphRuntime, String> {
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for n in &cfg.nodes {
        factories.insert(n.id.clone(), entry(&n.node_type));
    }
    GraphRuntime::build_for_test(cfg, factories, Arc::new(VirtualClock::new()), 64)
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
/// `consumer` is the word the input's probe drains through and `producer` the
/// word the producer's gate reads and its publish counts on: one word on a
/// local edge, two mappings of one page on a split edge. Frames carry
/// `x = 1, 2, ...` in publish order. The consumer drains one frame after the
/// first publish that follows the release, then the producer publishes while
/// its gate allows (at most 4 more), then the consumer drains what is queued.
/// Returns the frames published after the release and the frames the consumer
/// received after the release.
fn resume_after_a_timeout_release(
    tag: &str,
    consumer: CreditWord,
    producer: &CreditShared,
) -> (Vec<u32>, Vec<u32>) {
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
        consumer,
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
    let (published, received) =
        resume_after_a_timeout_release("forgive_local", local.clone(), &local);
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

/// The same release on a split edge: the consumer's process drains through
/// its own mapping of the page and the producer's gate reads another.
#[test]
fn frames_published_after_a_timeout_release_on_a_split_edge_are_never_evicted() {
    let ns = format!("bsr_{}_forgive", std::process::id());
    let id = credit_edge_id("/bsr/forgive", "consumer", "inp");
    let _owner = MappedCredit::create_owned(&ns, &id, 2).expect("the supervisor's word");
    let consumer = Arc::new(MappedCredit::open_unowned(&ns, &id).expect("consumer mapping"));
    let producer = MappedCredit::open_unowned(&ns, &id).expect("producer mapping");
    let (published, received) = resume_after_a_timeout_release(
        "forgive_mapped",
        CreditWord::mapped(Arc::clone(&consumer)),
        &producer,
    );
    assert_eq!(
        published,
        vec![3, 4],
        "the producer is held once frames 3 and 4 fill the slot (received {received:?})"
    );
    assert_eq!(
        received, published,
        "every frame published after the release is received"
    );
    assert_eq!(producer.outstanding(), 0, "the drains emptied the slot");
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
