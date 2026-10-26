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
//! - a consumer node that stops with a fatal error has its slot released;
//! - the graph's `backpressure:` key turns a `drop_oldest` input into `block`;
//! - a ring of `block` edges, and `block` on a `network.ingress` topic, are
//!   refused at build.
//!
//! Mutants (`--features oracle-mutants`, `CERULION_ORACLE_MUTANT=<name>`):
//! `block_no_timeout_release`, `block_shared_release` and `block_cycle_off`
//! each make a row here fail.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{
    GraphConfig, InputBackpressure, InputDef, NetworkBlock, NetworkMode, NodeDef, OutputDef,
};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

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
        fuse: None,
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
        "fast_drop_oldest" => Box::new(FastDropOldestEntry::new()),
        "relay" => Box::new(RelayEntry::new()),
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
