// SPDX-License-Identifier: AGPL-3.0-only
//! `NodeError::Fatal` stops the node that returns it.
//!
//! Through `GraphRuntime::build_for_test` (per-test SHM root, parallel-safe):
//! a producer returns `Fatal` from a chosen tick. After that step the
//! scheduler does not fire it again, so it publishes nothing more; the rest
//! of the graph keeps running; a downstream input's `expect_within_ms`
//! watchdog reports the silence exactly as for any silent upstream; and
//! `node_fatal_errors()` names the node and the message. The stop holds for a
//! Period source, a data-triggered relay, a wide level fired on the rayon
//! pool, and a fire inside a Period catch-up burst. Every other `NodeError` kind keeps today's
//! behaviour: logged, and the node keeps firing.
//!
//! The expected fire counts and delivered values are hand-written from the
//! graph's shape (a 5 ms period producer on 5 ms steps fires once per step);
//! nothing is derived from a second run of the code under test, except the
//! determinism test, which compares two runs AND pins the hand oracle.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::scheduler::TraceEntry;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

/// The error a producer returns, by word.
fn error_for(kind: &str) -> NodeError {
    match kind {
        "fatal" => NodeError::Fatal("lidar lost".to_string()),
        "logic" => NodeError::Logic("bad state".to_string()),
        "invalid_input" => NodeError::InvalidInput {
            input: "x".to_string(),
            reason: "stale".to_string(),
        },
        "custom" => NodeError::Custom(Box::new(std::io::Error::other("custom"))),
        "transport" => NodeError::Transport(TransportError::NotInitialized),
        "io" => NodeError::Io(std::io::Error::other("disk")),
        other => panic!("unknown kind {other}"),
    }
}

/// Publishes its tick number in `out.x`; from tick `fail_at` on it returns
/// the `kind` error instead (0 = never fails).
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct Producer {
    #[output]
    out: Vector3,
    n: u32,
    fail_at: u32,
    kind: String,
    init_kind: String,
    shutdown_kind: String,
}
#[cerulion_node_impl]
impl Producer {
    fn init(&mut self, _ctx: &mut NodeContext) -> Result<(), NodeError> {
        if !self.init_kind.is_empty() {
            return Err(error_for(&self.init_kind));
        }
        Ok(())
    }

    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        if self.fail_at != 0 && self.n >= self.fail_at {
            return Err(error_for(&self.kind));
        }
        self.out.x = self.n as f64;
        Ok(())
    }

    fn shutdown(&mut self) -> Result<(), NodeError> {
        if !self.shutdown_kind.is_empty() {
            return Err(error_for(&self.shutdown_kind));
        }
        Ok(())
    }
}

/// Data-triggered relay: forwards `inp.x` to `out.x`; from its fire
/// `fail_at` on it returns the `kind` error instead (0 = never fails).
#[cerulion_node]
#[derive(Default)]
struct Relay {
    #[input(trigger)]
    inp: Vector3,
    #[output]
    out: Vector3,
    n: u32,
    fail_at: u32,
    kind: String,
}
#[cerulion_node_impl]
impl Relay {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        if self.fail_at != 0 && self.n >= self.fail_at {
            return Err(error_for(&self.kind));
        }
        self.out.x = self.inp.x;
        Ok(())
    }
}

/// Sync node over two trigger inputs: from its fire `fail_at` on it returns
/// the `kind` error (0 = never fails). Counts its fires.
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct Fuse {
    #[input(trigger)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    n: u32,
    fail_at: u32,
    kind: String,
}
#[cerulion_node_impl]
impl Fuse {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        if self.fail_at != 0 && self.n >= self.fail_at {
            return Err(error_for(&self.kind));
        }
        Ok(())
    }
}

/// A 5 ms period node watching a latest-value input with a 30 ms
/// `expect_within_ms`: it keeps firing when the input goes silent, so its
/// `#[on_event]` handler runs (events dispatch on the consumer's own tick).
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct Watcher {
    #[input(expect_within_ms = 30)]
    inp: Vector3,
    stale: Arc<AtomicU64>,
}
#[cerulion_node_impl]
impl Watcher {
    fn tick(&mut self) -> Result<(), NodeError> {
        Ok(())
    }

    #[on_event(input = "inp")]
    fn on_stale(&mut self, _event: ExpectWithinEvent) {
        self.stale.fetch_add(1, Ordering::Relaxed);
    }
}

/// Records every value it receives and counts `ExpectWithinEvent`s.
#[cerulion_node]
#[derive(Default)]
struct Consumer {
    #[input(trigger, expect_within_ms = 30)]
    inp: Vector3,
    seen: Arc<Mutex<Vec<f64>>>,
    stale: Arc<AtomicU64>,
}
#[cerulion_node_impl]
impl Consumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.seen.lock().unwrap().push(self.inp.x);
        Ok(())
    }

    #[on_event(input = "inp")]
    fn on_stale(&mut self, _event: ExpectWithinEvent) {
        self.stale.fetch_add(1, Ordering::Relaxed);
    }
}

/// One `prod<i> -> cons<i>` pair: the producer fails with `kind` from tick
/// `fail_at` (0 = never), with `init_kind` from init, or with `shutdown_kind`
/// from shutdown (empty = never). `watch` adds `watch<i>`, a [`Watcher`] on
/// the same producer.
#[derive(Default)]
struct Pair {
    fail_at: u32,
    kind: &'static str,
    init_kind: &'static str,
    shutdown_kind: &'static str,
    watch: bool,
}

struct Observed {
    seen: Vec<Vec<f64>>,
    stale: Vec<u64>,
    trace: Vec<TraceEntry>,
    fatal: Vec<(String, String)>,
    /// Per consumer: the input watchdog's missed-window count.
    missed: Vec<u64>,
    /// Per pair with `watch`: the watcher's handler runs, else 0.
    watch_stale: Vec<u64>,
    /// Per pair with `watch`: the watcher's missed-window count, else 0.
    watch_missed: Vec<u64>,
}

fn producer_fires(trace: &[TraceEntry], id: &str) -> usize {
    trace.iter().filter(|e| &*e.node_id == id).count()
}

/// Fires of `id` at each step `0..steps`.
fn fires_per_step(trace: &[TraceEntry], id: &str, steps: u64) -> Vec<usize> {
    (0..steps)
        .map(|step| {
            trace
                .iter()
                .filter(|e| &*e.node_id == id && e.step == step)
                .count()
        })
        .collect()
}

fn node(id: &str, node_type: &str, source: Option<String>, output: bool) -> NodeDef {
    NodeDef {
        params: Default::default(),
        fuse: None,
        ros2: None,
        id: id.to_string(),
        node_type: node_type.to_string(),
        inputs: source
            .map(|source| {
                vec![InputDef {
                    name: "inp".to_string(),
                    source,
                }]
            })
            .unwrap_or_default(),
        outputs: if output {
            vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }]
        } else {
            vec![]
        },
    }
}

fn finish(
    nodes: Vec<NodeDef>,
    factories: IndexMap<String, Box<dyn NodeEntry>>,
) -> TransportResult<GraphRuntime> {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "node_fatal_stop_test".to_string(),
        prefix: "nfs".to_string(),
        nodes,
    };
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 4096)
}

/// The runtime, then each consumer's received values and event count, then
/// each pair's watcher event count (a fresh unread counter without `watch`).
type Built = (
    GraphRuntime,
    Vec<Arc<Mutex<Vec<f64>>>>,
    Vec<Arc<AtomicU64>>,
    Vec<Arc<AtomicU64>>,
);

fn build(pairs: &[Pair]) -> Built {
    try_build(pairs).expect("build fatal-stop graph")
}

fn try_build(pairs: &[Pair]) -> TransportResult<Built> {
    let mut nodes = Vec::new();
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let mut seen = Vec::new();
    let mut stale = Vec::new();
    let mut watch_stale = Vec::new();
    for (i, p) in pairs.iter().enumerate() {
        let prod = format!("prod{i}");
        let cons = format!("cons{i}");
        nodes.push(node(&prod, "producer", None, true));
        nodes.push(node(&cons, "consumer", Some(format!("{prod}/out")), false));
        factories.insert(
            prod.clone(),
            Box::new(ProducerEntry::with_state(Producer {
                fail_at: p.fail_at,
                kind: p.kind.to_string(),
                init_kind: p.init_kind.to_string(),
                shutdown_kind: p.shutdown_kind.to_string(),
                ..Default::default()
            })),
        );
        let s = Arc::new(Mutex::new(Vec::new()));
        let st = Arc::new(AtomicU64::new(0));
        factories.insert(
            cons,
            Box::new(ConsumerEntry::with_state(Consumer {
                seen: Arc::clone(&s),
                stale: Arc::clone(&st),
                ..Default::default()
            })),
        );
        seen.push(s);
        stale.push(st);
        let ws = Arc::new(AtomicU64::new(0));
        if p.watch {
            let watch = format!("watch{i}");
            nodes.push(node(&watch, "watcher", Some(format!("{prod}/out")), false));
            factories.insert(
                watch,
                Box::new(WatcherEntry::with_state(Watcher {
                    stale: Arc::clone(&ws),
                    ..Default::default()
                })),
            );
        }
        watch_stale.push(ws);
    }
    let rt = finish(nodes, factories)?;
    Ok((rt, seen, stale, watch_stale))
}

fn missed_of(rt: &GraphRuntime, id: &str) -> u64 {
    rt.node_handle(id)
        .unwrap_or_else(|| panic!("{id} handle"))
        .expect_within_missed_count()
}

fn run(pairs: &[Pair], steps: usize) -> Observed {
    run_with_step(pairs, steps, Duration::from_millis(5))
}

fn run_with_step(pairs: &[Pair], steps: usize, step: Duration) -> Observed {
    let (mut rt, seen, stale, watch_stale) = build(pairs);
    for _ in 0..steps {
        rt.step(step);
    }
    let missed = (0..pairs.len())
        .map(|i| missed_of(&rt, &format!("cons{i}")))
        .collect();
    let watch_missed = pairs
        .iter()
        .enumerate()
        .map(|(i, p)| {
            if p.watch {
                missed_of(&rt, &format!("watch{i}"))
            } else {
                0
            }
        })
        .collect();
    Observed {
        missed,
        watch_missed,
        watch_stale: watch_stale
            .iter()
            .map(|s| s.load(Ordering::Relaxed))
            .collect(),
        seen: seen.iter().map(|s| s.lock().unwrap().clone()).collect(),
        stale: stale.iter().map(|s| s.load(Ordering::Relaxed)).collect(),
        trace: rt.trace().to_vec(),
        fatal: rt.node_fatal_errors(),
    }
}

const FATAL_MSG: &str = "Fatal error: lidar lost";
/// The refused build's whole error text for a fatal `init` (hand-written).
const INIT_FATAL_TEXT: &str = "Node 'user_init' error: Fatal error: lidar lost";

fn healthy() -> Pair {
    Pair::default()
}

fn fatal_at(n: u32) -> Pair {
    Pair {
        fail_at: n,
        kind: "fatal",
        ..Default::default()
    }
}

#[test]
fn a_fatal_node_stops_firing_and_publishing_while_the_rest_of_the_graph_runs() {
    let o = run(&[fatal_at(3), healthy()], 20);
    // prod0 ran ticks 1, 2 (published 1.0, 2.0) and 3 (fatal), then never again.
    assert_eq!(
        producer_fires(&o.trace, "prod0"),
        3,
        "fires up to and including the fatal step"
    );
    assert_eq!(
        o.seen[0],
        vec![1.0, 2.0],
        "nothing is published from the fatal step on"
    );
    // The healthy pair keeps running: 20 steps, 20 publishes.
    assert_eq!(producer_fires(&o.trace, "prod1"), 20);
    assert_eq!(
        o.seen[1].len(),
        20,
        "the rest of the graph keeps delivering"
    );
    assert_eq!(o.seen[1].first(), Some(&1.0));
    assert_eq!(o.seen[1].last(), Some(&20.0));
    // Reported: the node and its message.
    assert_eq!(o.fatal.len(), 1, "{:?}", o.fatal);
    assert_eq!(o.fatal[0].0, "prod0");
    assert_eq!(o.fatal[0].1, FATAL_MSG, "{:?}", o.fatal);
}

fn watched(p: Pair) -> Pair {
    Pair { watch: true, ..p }
}

#[test]
fn a_downstream_watchdog_reports_the_silence_of_a_stopped_node_as_normal() {
    let fatal = run(&[watched(fatal_at(3)), watched(healthy())], 20);
    // The control: the same producer goes silent from the same tick WITHOUT a
    // fatal stop (a `Logic` error keeps it firing but publishes nothing).
    let silent = run(
        &[watched(Pair {
            fail_at: 3,
            kind: "logic",
            ..Default::default()
        })],
        20,
    );
    // Hand oracle. Step k (0-based) runs at t = 5(k+1) ms; the last frame is
    // tick 2's, stamped t = 10 ms. A window is missed when `now - start > 30
    // ms` (strict), and a miss restarts the window at `now`: misses at t = 45
    // (step 8) and t = 80 (step 15); the next would need t > 110, past the
    // last step at t = 100. So exactly 2, on both watched inputs.
    assert_eq!(fatal.missed[0], 2, "the data-triggered consumer's misses");
    assert_eq!(fatal.watch_missed[0], 2, "the period watcher's misses");
    // `ExpectWithinEvent` is edge-triggered, once per silence regime, and
    // dispatches on the consumer's own tick: the period watcher keeps firing,
    // so its handler runs once; the data-triggered consumer never fires again
    // after its input goes silent, so its handler never runs (the documented
    // dispatch limit; its missed counter above is the signal).
    assert_eq!(fatal.watch_stale[0], 1, "the watcher's handler runs once");
    assert_eq!(
        fatal.stale[0], 0,
        "a data-triggered consumer sees only the counter"
    );
    // "As normal": the same observations as for an upstream that is silent
    // without being stopped.
    assert_eq!(
        (
            fatal.missed[0],
            fatal.watch_missed[0],
            fatal.watch_stale[0],
            fatal.stale[0]
        ),
        (
            silent.missed[0],
            silent.watch_missed[0],
            silent.watch_stale[0],
            silent.stale[0]
        ),
        "a stopped upstream looks like any silent one"
    );
    // The healthy pair misses no window and raises no event.
    assert_eq!(
        (
            fatal.missed[1],
            fatal.watch_missed[1],
            fatal.watch_stale[1],
            fatal.stale[1]
        ),
        (0, 0, 0, 0)
    );
    // Liveliness: a fatal stop disables the node but keeps its publisher, so
    // the input's publisher is still alive and no `LivelinessEvent` is due;
    // `expect_within_ms` is the signal for a stopped upstream.
}

#[test]
fn every_non_fatal_kind_keeps_the_node_firing() {
    for kind in ["logic", "invalid_input", "custom", "transport", "io"] {
        let o = run(
            &[Pair {
                fail_at: 3,
                kind,
                ..Default::default()
            }],
            20,
        );
        assert_eq!(
            producer_fires(&o.trace, "prod0"),
            20,
            "{kind}: keeps firing"
        );
        assert_eq!(
            o.seen[0],
            vec![1.0, 2.0],
            "{kind}: an erroring tick publishes nothing"
        );
        assert!(o.fatal.is_empty(), "{kind}: not fatal, got {:?}", o.fatal);
    }
}

#[test]
fn fatal_on_the_first_tick_publishes_nothing_and_fires_once() {
    let o = run(&[fatal_at(1)], 10);
    assert_eq!(producer_fires(&o.trace, "prod0"), 1);
    assert!(o.seen[0].is_empty());
    assert_eq!(o.fatal.len(), 1);
}

#[test]
fn two_nodes_fatal_in_the_same_step_are_both_stopped_and_reported_in_graph_order() {
    let o = run(&[fatal_at(4), healthy(), fatal_at(4)], 12);
    assert_eq!(producer_fires(&o.trace, "prod0"), 4);
    assert_eq!(producer_fires(&o.trace, "prod2"), 4);
    assert_eq!(producer_fires(&o.trace, "prod1"), 12);
    let ids: Vec<&str> = o.fatal.iter().map(|(id, _)| id.as_str()).collect();
    assert_eq!(ids, vec!["prod0", "prod2"]);
}

#[test]
fn a_fatal_stop_is_bit_identical_across_two_runs() {
    let a = run(&[fatal_at(5), healthy()], 16);
    let b = run(&[fatal_at(5), healthy()], 16);
    assert_eq!(a.trace, b.trace, "scheduler trace up to and after the stop");
    assert_eq!(a.seen, b.seen);
    assert_eq!(a.fatal, b.fatal);
    // And the hand oracle, so two equally wrong runs cannot pass.
    assert_eq!(producer_fires(&a.trace, "prod0"), 5);
    assert_eq!(a.seen[0], vec![1.0, 2.0, 3.0, 4.0]);
    // The last prod0 fire is at the 5th step (0-based step 4).
    let last = a.trace.iter().rfind(|e| &*e.node_id == "prod0").unwrap();
    assert_eq!(last.step, 4);
}

#[test]
fn fatal_from_init_refuses_the_build_with_the_typed_kind() {
    let err = match try_build(&[Pair {
        init_kind: "fatal",
        ..Default::default()
    }]) {
        Ok(_) => panic!("a fatal init must refuse the build"),
        Err(e) => e,
    };
    assert_eq!(err.node_error_kind(), Some(NodeErrorKind::Fatal), "{err:?}");
    assert_eq!(err.to_string(), INIT_FATAL_TEXT, "{err}");
}

/// `prod -> relay -> cons`: the relay returns `Fatal` from its 3rd fire.
#[test]
fn a_fatal_data_triggered_node_stops_while_its_upstream_keeps_publishing() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "prod".to_string(),
        Box::new(ProducerEntry::with_state(Producer::default())),
    );
    factories.insert(
        "relay".to_string(),
        Box::new(RelayEntry::with_state(Relay {
            fail_at: 3,
            kind: "fatal".to_string(),
            ..Default::default()
        })),
    );
    factories.insert(
        "cons".to_string(),
        Box::new(ConsumerEntry::with_state(Consumer {
            seen: Arc::clone(&seen),
            ..Default::default()
        })),
    );
    let nodes = vec![
        node("prod", "producer", None, true),
        node("relay", "relay", Some("prod/out".to_string()), true),
        node("cons", "consumer", Some("relay/out".to_string()), false),
    ];
    let mut rt = finish(nodes, factories).expect("build relay chain");
    for _ in 0..20 {
        rt.step(Duration::from_millis(5));
    }
    let trace = rt.trace().to_vec();
    // The producer publishes on every one of the 20 steps; the relay fires on
    // its first three arrivals (forwarding 1.0 and 2.0, the third is fatal),
    // and is never fired again although its input keeps arriving.
    assert_eq!(
        producer_fires(&trace, "prod"),
        20,
        "the upstream keeps running"
    );
    assert_eq!(
        producer_fires(&trace, "relay"),
        3,
        "fires up to the fatal one"
    );
    assert_eq!(
        *seen.lock().unwrap(),
        vec![1.0, 2.0],
        "the downstream goes silent"
    );
    let fatal = rt.node_fatal_errors();
    assert_eq!(fatal.len(), 1, "{fatal:?}");
    assert_eq!(fatal[0].0, "relay");
    assert_eq!(fatal[0].1, FATAL_MSG, "{fatal:?}");
}

/// Eight producers in one level: the level fires on the rayon pool
/// (`PARALLEL_FIRE_THRESHOLD` is 8), where `fire_node_into` runs on a worker
/// thread. The fatal stop and every healthy count are the serial ones.
#[test]
fn a_fatal_node_in_a_wide_parallel_level_stops_and_the_rest_keep_firing() {
    let mut pairs = vec![fatal_at(3)];
    pairs.extend((0..7).map(|_| healthy()));
    let o = run(&pairs, 20);
    assert_eq!(producer_fires(&o.trace, "prod0"), 3);
    assert_eq!(o.seen[0], vec![1.0, 2.0]);
    for i in 1..8 {
        assert_eq!(producer_fires(&o.trace, &format!("prod{i}")), 20, "prod{i}");
        assert_eq!(o.seen[i].len(), 20, "cons{i}");
    }
    let ids: Vec<&str> = o.fatal.iter().map(|(id, _)| id.as_str()).collect();
    assert_eq!(ids, vec!["prod0"]);
}

/// A 20 ms step on a 5 ms period is a catch-up burst of four fires in one
/// step. A fatal return stops the burst at that fire: the fatal fire is the
/// last one, in the same step, and no later step fires the node.
#[test]
fn a_fatal_fire_inside_a_period_catch_up_burst_ends_the_burst() {
    let o = run_with_step(
        &[fatal_at(1), fatal_at(2), healthy()],
        5,
        Duration::from_millis(20),
    );
    assert_eq!(
        fires_per_step(&o.trace, "prod2", 5),
        vec![4, 4, 4, 4, 4],
        "healthy burst"
    );
    assert_eq!(
        fires_per_step(&o.trace, "prod0", 5),
        vec![1, 0, 0, 0, 0],
        "fatal on the first fire of the burst"
    );
    assert_eq!(
        fires_per_step(&o.trace, "prod1", 5),
        vec![2, 0, 0, 0, 0],
        "fatal on the second fire of the burst"
    );
    assert!(o.seen[0].is_empty());
    assert_eq!(o.seen[1], vec![1.0]);
}

/// A fatal error from `shutdown` is logged like any shutdown error: the
/// node's ticks were healthy, so it never stopped, `node_fatal_errors()` (read
/// before shutdown, as `graph run` reads it) is empty, and the shutdown
/// completes. (`shutdown` consumes the runtime, so nothing can read it after.)
#[test]
fn a_fatal_error_from_shutdown_is_not_a_fatal_stop() {
    let (mut rt, seen, _, _) = build(&[Pair {
        shutdown_kind: "fatal",
        ..Default::default()
    }]);
    for _ in 0..5 {
        rt.step(Duration::from_millis(5));
    }
    assert_eq!(
        producer_fires(rt.trace(), "prod0"),
        5,
        "the ticks were healthy"
    );
    assert!(
        rt.node_fatal_errors().is_empty(),
        "{:?}",
        rt.node_fatal_errors()
    );
    rt.shutdown();
    assert_eq!(seen[0].lock().unwrap().len(), 5);
}

/// `pa`, `pb -> fuse` (Sync): the fuse returns `Fatal` from its 3rd fire.
#[test]
fn a_fatal_sync_node_stops_while_both_upstreams_keep_publishing() {
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for id in ["pa", "pb"] {
        factories.insert(
            id.to_string(),
            Box::new(ProducerEntry::with_state(Producer::default())),
        );
    }
    factories.insert(
        "fuse".to_string(),
        Box::new(FuseEntry::with_state(Fuse {
            fail_at: 3,
            kind: "fatal".to_string(),
            ..Default::default()
        })),
    );
    let mut fuse = node("fuse", "fuse", None, false);
    fuse.inputs = ["a", "b"]
        .iter()
        .map(|name| InputDef {
            name: (*name).to_string(),
            source: format!("p{name}/out"),
        })
        .collect();
    let nodes = vec![
        node("pa", "producer", None, true),
        node("pb", "producer", None, true),
        fuse,
    ];
    let mut rt = finish(nodes, factories).expect("build sync graph");
    for _ in 0..20 {
        rt.step(Duration::from_millis(5));
    }
    let trace = rt.trace().to_vec();
    assert_eq!(producer_fires(&trace, "pa"), 20);
    assert_eq!(producer_fires(&trace, "pb"), 20);
    // Both sources publish every step, so a set forms on every step: the
    // fuse fires for sets 1 and 2, returns Fatal on set 3, then never again.
    assert_eq!(
        producer_fires(&trace, "fuse"),
        3,
        "fires up to the fatal one"
    );
    let ids: Vec<String> = rt
        .node_fatal_errors()
        .into_iter()
        .map(|(id, _)| id)
        .collect();
    assert_eq!(ids, vec!["fuse".to_string()]);
}
