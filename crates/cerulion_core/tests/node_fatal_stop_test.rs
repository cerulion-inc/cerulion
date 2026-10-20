// SPDX-License-Identifier: AGPL-3.0-only
//! `NodeError::Fatal` stops the node that returns it.
//!
//! Through `GraphRuntime::build_for_test` (per-test SHM root, parallel-safe):
//! a producer returns `Fatal` from a chosen tick. After that step the
//! scheduler does not fire it again, so it publishes nothing more; the rest
//! of the graph keeps running; a downstream input's `expect_within_ms`
//! watchdog reports the silence as normal; and `node_fatal_errors()` names
//! the node and the message. Every other `NodeError` kind keeps today's
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
/// `fail_at` (0 = never), or with `init_kind` from init (empty = never).
struct Pair {
    fail_at: u32,
    kind: &'static str,
    init_kind: &'static str,
}

struct Observed {
    seen: Vec<Vec<f64>>,
    stale: Vec<u64>,
    trace: Vec<TraceEntry>,
    fatal: Vec<(String, String)>,
    /// Per consumer: the input watchdog's missed-window count.
    missed: Vec<u64>,
}

fn producer_fires(trace: &[TraceEntry], id: &str) -> usize {
    trace.iter().filter(|e| &*e.node_id == id).count()
}

/// The runtime, then each consumer's received values and event count.
type Built = (GraphRuntime, Vec<Arc<Mutex<Vec<f64>>>>, Vec<Arc<AtomicU64>>);

fn build(pairs: &[Pair]) -> Built {
    try_build(pairs).expect("build fatal-stop graph")
}

fn try_build(pairs: &[Pair]) -> TransportResult<Built> {
    let mut nodes = Vec::new();
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let mut seen = Vec::new();
    let mut stale = Vec::new();
    for (i, p) in pairs.iter().enumerate() {
        let prod = format!("prod{i}");
        let cons = format!("cons{i}");
        nodes.push(NodeDef {
            fuse: None,
            ros2: None,
            id: prod.clone(),
            node_type: "producer".to_string(),
            inputs: vec![],
            outputs: vec![OutputDef {
                name: "out".to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            }],
        });
        nodes.push(NodeDef {
            fuse: None,
            ros2: None,
            id: cons.clone(),
            node_type: "consumer".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: format!("{prod}/out"),
            }],
            outputs: vec![],
        });
        factories.insert(
            prod,
            Box::new(ProducerEntry::with_state(Producer {
                fail_at: p.fail_at,
                kind: p.kind.to_string(),
                init_kind: p.init_kind.to_string(),
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
    }
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
    let rt = GraphRuntime::build_for_test(config, factories, clock, 4096)?;
    Ok((rt, seen, stale))
}

fn run(pairs: &[Pair], steps: usize) -> Observed {
    let (mut rt, seen, stale) = build(pairs);
    for _ in 0..steps {
        rt.step(Duration::from_millis(5));
    }
    let missed = (0..pairs.len())
        .map(|i| {
            rt.node_handle(&format!("cons{i}"))
                .expect("consumer handle")
                .expect_within_missed_count()
        })
        .collect();
    Observed {
        missed,
        seen: seen.iter().map(|s| s.lock().unwrap().clone()).collect(),
        stale: stale.iter().map(|s| s.load(Ordering::Relaxed)).collect(),
        trace: rt.trace().to_vec(),
        fatal: rt.node_fatal_errors(),
    }
}

const FATAL_MSG: &str = "Fatal error: lidar lost";

fn healthy() -> Pair {
    Pair {
        fail_at: 0,
        kind: "",
        init_kind: "",
    }
}

fn fatal_at(n: u32) -> Pair {
    Pair {
        fail_at: n,
        kind: "fatal",
        init_kind: "",
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
    assert!(o.fatal[0].1.ends_with(FATAL_MSG), "{:?}", o.fatal);
}

#[test]
fn a_downstream_watchdog_reports_the_silence_of_a_stopped_node() {
    // 30 ms window, 5 ms steps: after the stop at step 3 the consumer's input
    // is silent for 85 ms, so its `expect_within_ms` watchdog misses windows,
    // exactly as for any silent upstream; the healthy pair's never does. (The
    // `#[on_event]` handler runs when the consumer next fires, which a
    // data-triggered consumer of a stopped node never does.)
    let o = run(&[fatal_at(3), healthy()], 20);
    assert!(
        o.missed[0] >= 1,
        "missed windows after the stop, got {}",
        o.missed[0]
    );
    assert_eq!(o.missed[1], 0, "a fresh input misses no window");
    assert_eq!(o.stale[1], 0, "and raises no event");
}

#[test]
fn every_non_fatal_kind_keeps_the_node_firing() {
    for kind in ["logic", "invalid_input", "custom", "transport", "io"] {
        let o = run(
            &[Pair {
                fail_at: 3,
                kind,
                init_kind: "",
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
        fail_at: 0,
        kind: "",
        init_kind: "fatal",
    }]) {
        Ok(_) => panic!("a fatal init must refuse the build"),
        Err(e) => e,
    };
    assert_eq!(err.node_error_kind(), Some(NodeErrorKind::Fatal), "{err:?}");
    assert!(err.to_string().ends_with(FATAL_MSG), "{err}");
}
