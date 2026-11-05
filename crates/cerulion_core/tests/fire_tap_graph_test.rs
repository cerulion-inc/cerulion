// SPDX-License-Identifier: AGPL-3.0-only
//! A built graph records one fire per node tick into its fire tap while a
//! reader holds the lease, and nothing while no reader does.
//!
//! Every graph here runs on `GraphRuntime::build_for_test` (per-test SHM
//! root) under a `VirtualClock`, so the binary is parallel-safe:
//!
//! ```bash
//! cargo test -p cerulion_core --test fire_tap_graph_test
//! ```
//!
//! The overhead test prints the armed and disarmed cost per step. Those
//! numbers compare only when nothing else runs beside it, so read them from
//! a run of that test alone:
//!
//! ```bash
//! cargo test -p cerulion_core --test fire_tap_graph_test overhead -- --nocapture
//! ```

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::fire_tap::{
    Cursor, FireBatch, FireCause, FireFlags, FireRecord, FireTap, FireTapConfig, FIRE_SERVE_MAX,
};
use cerulion_core::graph::node::{NodeContext, NodeEntry, OutputMeta};
use cerulion_core::graph::{parse_graph, GraphRuntime, TopologyTrigger};
use cerulion_core::prelude::*;
use cerulion_core::{ClosureNodeEntry, MacroPolicy, NodeInfo};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

const STEP: Duration = Duration::from_millis(1);

/// A period source with two outputs feeding a relay triggered by the first.
const ALTERNATING: &str = r#"
prefix: fires
nodes:
  - id: source
    type: alternating_source
    outputs:
      - {name: a, schema: geometry_msgs/Vector3}
      - {name: b, schema: geometry_msgs/Vector3}
  - id: relay
    type: relay
    inputs:
      - {name: a, source: source/a}
    outputs:
      - {name: out, schema: geometry_msgs/Vector3}
"#;

const NONE: u8 = 0;
const SOURCE: u16 = 0;
const RELAY: u16 = 1;
const PERIOD: FireCause = FireCause::Period;
const DATA: FireCause = FireCause::Data;

fn config(lease: Duration) -> FireTapConfig {
    FireTapConfig {
        capacity: 64,
        lease,
    }
}

fn trigger(name: &str) -> InputMeta {
    InputMeta {
        name: name.into(),
        schema_hash: 0,
        trigger: true,
        depth: 4,
        backpressure: BackpressurePolicy::DropOldest,
        expect_within_ms: None,
    }
}

fn period_info(outputs: &[&str]) -> NodeInfo {
    NodeInfo::from_names(vec![], outputs.iter().map(|o| o.to_string()).collect())
        .with_policy(MacroPolicy::Period { period_ms: 1 })
}

fn data_info(input: &str, outputs: &[&str]) -> NodeInfo {
    NodeInfo::with_meta(
        vec![trigger(input)],
        outputs
            .iter()
            .map(|o| OutputMeta::new(o.to_string(), 0, None))
            .collect(),
    )
    .with_policy(MacroPolicy::DataTrigger {
        input_name: input.into(),
    })
}

fn publish(ctx: &mut NodeContext, output: &str, x: f64) -> TransportResult<()> {
    let mut proxy = ctx
        .publisher_mut(output)
        .expect("output is wired")
        .loan_proxy::<Vector3>()?;
    proxy.x = x;
    Ok(())
}

/// Tick n (from 1) publishes `a` when n is odd and `b` when n is even.
fn alternating_source() -> ClosureNodeEntry {
    let mut n = 0_u64;
    ClosureNodeEntry::new(period_info(&["a", "b"]), move |ctx| {
        n += 1;
        publish(ctx, if n % 2 == 1 { "a" } else { "b" }, n as f64)
    })
}

/// Republishes each `a` frame it reads on `out`, counting its ticks.
fn relay(ticks: Arc<AtomicU64>) -> ClosureNodeEntry {
    ClosureNodeEntry::new(data_info("a", &["out"]), move |ctx| {
        ticks.fetch_add(1, Ordering::Relaxed);
        let x = ctx
            .subscriber_mut("a")
            .expect("input is wired")
            .try_view::<Vector3, _>(|view| view.x)?;
        match x {
            Some(x) => publish(ctx, "out", x),
            None => Ok(()),
        }
    })
}

fn build(yaml: &str, entries: Vec<(&str, Box<dyn NodeEntry>)>) -> GraphRuntime {
    let factories: IndexMap<String, Box<dyn NodeEntry>> = entries
        .into_iter()
        .map(|(id, entry)| (id.to_string(), entry))
        .collect();
    match GraphRuntime::build_for_test(
        parse_graph(yaml).expect("graph parses"),
        factories,
        Arc::new(VirtualClock::new()),
        8,
    ) {
        Ok(runtime) => runtime,
        Err(error) => panic!("graph builds: {error}"),
    }
}

fn alternating_graph() -> GraphRuntime {
    build(
        ALTERNATING,
        vec![
            ("source", Box::new(alternating_source())),
            ("relay", Box::new(relay(Arc::default()))),
        ],
    )
}

fn steps(runtime: &mut GraphRuntime, n: usize) {
    for _ in 0..n {
        runtime.step(STEP);
    }
}

/// Arm the lease without consuming any sequence.
fn arm(tap: &FireTap) {
    let batch = tap.serve(Cursor::After(0), 0);
    assert!(batch.records.is_empty());
}

fn read_all(tap: &FireTap) -> FireBatch {
    let batch = tap.serve(Cursor::After(0), FIRE_SERVE_MAX);
    assert_eq!(batch.lost, 0, "a 64-slot ring holds every record here");
    assert_eq!(batch.next, batch.records.len() as u64);
    for (i, record) in batch.records.iter().enumerate() {
        assert_eq!(record.seq, i as u64 + 1, "sequences are gap-free from 1");
    }
    batch
}

/// The replay-stable columns of a record: everything but `dur_ns`.
type Shape = (u64, u16, FireCause, u8, u64);

fn shape(record: &FireRecord) -> Shape {
    (
        record.at_ns,
        record.node,
        record.cause,
        record.flags.0,
        record.wrote,
    )
}

fn shapes(batch: &FireBatch) -> Vec<Shape> {
    batch.records.iter().map(shape).collect()
}

fn ms(n: u64) -> u64 {
    n * 1_000_000
}

#[test]
fn armed_ticks_record_node_cause_writes_and_duration() {
    let mut runtime = alternating_graph();
    let tap = runtime.install_fire_tap(config(Duration::from_secs(5)));
    let topology = runtime.run_topology();
    assert_eq!(topology.nodes[usize::from(SOURCE)].id, "source");
    assert_eq!(topology.nodes[usize::from(RELAY)].id, "relay");
    assert!(topology.nodes.iter().all(|node| node.reports_writes));

    arm(&tap);
    steps(&mut runtime, 6);
    let batch = read_all(&tap);

    assert_eq!(
        shapes(&batch),
        vec![
            (ms(1), SOURCE, PERIOD, NONE, 0b01),
            (ms(1), RELAY, DATA, NONE, 0b1),
            (ms(2), SOURCE, PERIOD, NONE, 0b10),
            (ms(3), SOURCE, PERIOD, NONE, 0b01),
            (ms(3), RELAY, DATA, NONE, 0b1),
            (ms(4), SOURCE, PERIOD, NONE, 0b10),
            (ms(5), SOURCE, PERIOD, NONE, 0b01),
            (ms(5), RELAY, DATA, NONE, 0b1),
            (ms(6), SOURCE, PERIOD, NONE, 0b10),
        ]
    );
    assert!(
        batch.records.iter().all(|record| record.dur_ns > 0),
        "every tick that published took measurable wall time"
    );
    let stats = tap.stats();
    assert!(stats.armed);
    assert_eq!(stats.recorded, 9);
}

#[test]
fn disarmed_ticks_record_nothing() {
    let mut runtime = alternating_graph();
    let tap = runtime.install_fire_tap(config(Duration::from_secs(5)));
    steps(&mut runtime, 6);

    let stats = tap.stats();
    assert!(!stats.armed);
    assert_eq!(stats.recorded, 0);
    let batch = tap.serve(Cursor::After(0), FIRE_SERVE_MAX);
    assert!(batch.records.is_empty());
    assert_eq!((batch.next, batch.lost), (0, 0));
}

#[test]
fn ticks_after_the_lease_expires_record_nothing() {
    let mut runtime = alternating_graph();
    let tap = runtime.install_fire_tap(config(Duration::from_micros(2_500)));
    arm(&tap);
    steps(&mut runtime, 6);
    let batch = read_all(&tap);

    assert_eq!(
        shapes(&batch),
        vec![
            (ms(1), SOURCE, PERIOD, NONE, 0b01),
            (ms(1), RELAY, DATA, NONE, 0b1),
            (ms(2), SOURCE, PERIOD, NONE, 0b10),
        ]
    );
    assert_eq!(tap.stats().recorded, 3, "expired ticks claim no sequence");
}

#[test]
fn install_fire_tap_is_idempotent() {
    let mut runtime = alternating_graph();
    let first = runtime.install_fire_tap(config(Duration::from_secs(5)));
    let second = runtime.install_fire_tap(FireTapConfig {
        capacity: 128,
        lease: Duration::from_secs(1),
    });
    assert!(Arc::ptr_eq(&first, &second));
    assert_eq!(second.stats().capacity, 64);

    arm(&second);
    steps(&mut runtime, 1);
    assert_eq!(first.stats().recorded, 2, "one tap, one record per tick");
}

const BOMB: &str = r#"
prefix: fires
nodes:
  - id: source
    type: source
    outputs:
      - {name: a, schema: geometry_msgs/Vector3}
  - id: bomb
    type: bomb
    inputs:
      - {name: a, source: source/a}
"#;

#[test]
fn panicking_tick_records_panicked_and_the_graph_keeps_running() {
    let mut runtime = build(
        BOMB,
        vec![
            (
                "source",
                Box::new(ClosureNodeEntry::new(period_info(&["a"]), |ctx| {
                    publish(ctx, "a", 1.0)
                })),
            ),
            (
                "bomb",
                Box::new(ClosureNodeEntry::new(data_info("a", &[]), |_| {
                    panic!("bomb tick panics by design")
                })),
            ),
        ],
    );
    let tap = runtime.install_fire_tap(config(Duration::from_secs(5)));
    arm(&tap);
    steps(&mut runtime, 3);
    let batch = read_all(&tap);

    assert_eq!(
        shapes(&batch),
        vec![
            (ms(1), 0, PERIOD, NONE, 0b1),
            (ms(1), 1, DATA, FireFlags::PANICKED, 0),
            (ms(2), 0, PERIOD, NONE, 0b1),
            (ms(3), 0, PERIOD, NONE, 0b1),
        ],
        "the dead node records no later fires; the source keeps recording"
    );
}

#[test]
fn tick_error_records_tick_error_and_panic_class_errors_record_panicked() {
    let mut runtime = build(
        BOMB,
        vec![
            (
                "source",
                Box::new(ClosureNodeEntry::new(period_info(&["a"]), |ctx| {
                    publish(ctx, "a", 1.0)?;
                    Err(TransportError::NodeError {
                        node_id: "source".into(),
                        reason: "source fails after publishing".into(),
                    })
                })),
            ),
            (
                "bomb",
                Box::new(ClosureNodeEntry::new(data_info("a", &[]), |_| {
                    Err(TransportError::NodeTickPanicked {
                        node_id: "bomb".into(),
                        reason: "a node library caught its own panic".into(),
                    })
                })),
            ),
        ],
    );
    let tap = runtime.install_fire_tap(config(Duration::from_secs(5)));
    arm(&tap);
    steps(&mut runtime, 2);
    let batch = read_all(&tap);

    assert_eq!(
        shapes(&batch),
        vec![
            (ms(1), 0, PERIOD, FireFlags::TICK_ERROR, 0b1),
            (ms(1), 1, DATA, FireFlags::PANICKED, 0),
            (ms(2), 0, PERIOD, FireFlags::TICK_ERROR, 0b1),
            (ms(2), 1, DATA, FireFlags::PANICKED, 0),
        ]
    );
}

#[test]
fn fatal_tick_error_records_its_fire_before_the_node_stops() {
    let mut runtime = build(
        SOURCE_ONLY,
        vec![(
            "source",
            Box::new(ClosureNodeEntry::new(period_info(&["a"]), |ctx| {
                publish(ctx, "a", 1.0)?;
                Err(NodeError::Fatal("source cannot continue".into()).into_transport("source"))
            })),
        )],
    );
    let tap = runtime.install_fire_tap(config(Duration::from_secs(5)));
    arm(&tap);
    steps(&mut runtime, 3);
    assert_eq!(
        shapes(&read_all(&tap)),
        vec![(ms(1), 0, PERIOD, FireFlags::TICK_ERROR, 0b1)],
        "a stopped node does not fire again"
    );
}

/// One output more than the write mask holds.
const WIDE_OUTPUTS: usize = 65;

#[test]
fn outputs_beyond_the_mask_record_wrote_truncated() {
    let outputs: Vec<String> = (0..WIDE_OUTPUTS).map(|k| format!("o{k}")).collect();
    let yaml = format!(
        "prefix: fires\nnodes:\n  - id: wide\n    type: wide\n    outputs:\n{}",
        outputs
            .iter()
            .map(|name| format!("      - {{name: {name}, schema: geometry_msgs/Vector3}}\n"))
            .collect::<String>()
    );
    let names: Vec<&str> = outputs.iter().map(String::as_str).collect();
    let wide = ClosureNodeEntry::new(period_info(&names), |ctx| {
        publish(ctx, "o1", 1.0)?;
        publish(ctx, "o64", 1.0)
    });
    let mut runtime = build(&yaml, vec![("wide", Box::new(wide))]);
    let tap = runtime.install_fire_tap(config(Duration::from_secs(5)));
    assert_eq!(runtime.run_topology().nodes[0].outputs.len(), WIDE_OUTPUTS);
    arm(&tap);
    steps(&mut runtime, 1);
    assert_eq!(
        shapes(&read_all(&tap)),
        vec![(ms(1), 0, PERIOD, FireFlags::WROTE_TRUNCATED, 0b10)],
        "output 64 has no bit; output 1 keeps its own"
    );
}

/// A node entry that keeps every `NodeEntry` default, so it cannot report
/// which outputs a tick committed.
struct OpaqueSource {
    context: Option<NodeContext>,
}

impl NodeEntry for OpaqueSource {
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(period_info(&["a"]))
    }

    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.context = Some(context);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        publish(self.context.as_mut().expect("initialized"), "a", 1.0)
    }
}

const SOURCE_ONLY: &str = r#"
prefix: fires
nodes:
  - id: source
    type: source
    outputs:
      - {name: a, schema: geometry_msgs/Vector3}
"#;

#[test]
fn entry_without_commit_counts_records_wrote_unknown() {
    let mut runtime = build(
        SOURCE_ONLY,
        vec![("source", Box::new(OpaqueSource { context: None }))],
    );
    let tap = runtime.install_fire_tap(config(Duration::from_secs(5)));
    assert!(!runtime.run_topology().nodes[0].reports_writes);

    arm(&tap);
    steps(&mut runtime, 2);
    assert_eq!(
        shapes(&read_all(&tap)),
        vec![
            (ms(1), 0, PERIOD, FireFlags::WROTE_UNKNOWN, 0),
            (ms(2), 0, PERIOD, FireFlags::WROTE_UNKNOWN, 0),
        ]
    );
}

/// The text every `ProbedSource` probe panic carries.
const PROBE_PANIC: &str = "the commit-count probe panics by design";

/// A period source that owns its context and counts each commit-count
/// probe. Probe number `panic_on` (from 1) panics instead.
struct ProbedSource {
    context: Option<NodeContext>,
    probes: Arc<AtomicU64>,
    panic_on: Option<u64>,
}

impl ProbedSource {
    fn new(probes: Arc<AtomicU64>, panic_on: Option<u64>) -> Self {
        Self {
            context: None,
            probes,
            panic_on,
        }
    }
}

impl NodeEntry for ProbedSource {
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(period_info(&["a"]))
    }

    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.context = Some(context);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        publish(self.context.as_mut().expect("initialized"), "a", 1.0)
    }

    fn output_commit_counts(&self, out: &mut [u32]) -> bool {
        let probe = self.probes.fetch_add(1, Ordering::Relaxed) + 1;
        if Some(probe) == self.panic_on {
            panic!("{PROBE_PANIC}");
        }
        self.context
            .as_ref()
            .is_some_and(|ctx| ctx.output_commit_counts(out))
    }
}

#[test]
fn disarmed_ticks_never_ask_the_entry_for_commit_counts() {
    let probes = Arc::new(AtomicU64::new(0));
    let mut runtime = build(
        SOURCE_ONLY,
        vec![(
            "source",
            Box::new(ProbedSource::new(Arc::clone(&probes), None)),
        )],
    );
    let tap = runtime.install_fire_tap(config(Duration::from_secs(5)));
    assert!(runtime.run_topology().nodes[0].reports_writes);
    assert_eq!(probes.load(Ordering::Relaxed), 1, "install probes once");

    steps(&mut runtime, 5);
    assert_eq!(
        probes.load(Ordering::Relaxed),
        1,
        "a disarmed tick asks the entry nothing"
    );
    assert_eq!(tap.stats().recorded, 0);

    arm(&tap);
    steps(&mut runtime, 2);
    assert_eq!(
        probes.load(Ordering::Relaxed),
        5,
        "an armed tick asks once before and once after"
    );
    assert_eq!(
        shapes(&read_all(&tap)),
        vec![(ms(6), 0, PERIOD, NONE, 0b1), (ms(7), 0, PERIOD, NONE, 0b1)]
    );
}

/// Two period sources on one level. They can fire on two threads, so the
/// tests below compare records sorted by time and node.
const STEADY_AND_FRAGILE: &str = r#"
prefix: fires
nodes:
  - id: steady
    type: source
    outputs:
      - {name: a, schema: geometry_msgs/Vector3}
  - id: fragile
    type: probed_source
    outputs:
      - {name: a, schema: geometry_msgs/Vector3}
"#;

const STEADY: u16 = 0;
const FRAGILE: u16 = 1;

/// Install the tap (probe 1), arm it, and step 3 times. `probes` counts the
/// fragile node's probes.
fn run_with_probe_panic(panic_on: u64, probes: Arc<AtomicU64>) -> (GraphRuntime, Vec<Shape>) {
    let steady = ClosureNodeEntry::new(period_info(&["a"]), |ctx| publish(ctx, "a", 1.0));
    let fragile = ProbedSource::new(probes, Some(panic_on));
    let mut runtime = build(
        STEADY_AND_FRAGILE,
        vec![("steady", Box::new(steady)), ("fragile", Box::new(fragile))],
    );
    let tap = runtime.install_fire_tap(config(Duration::from_secs(5)));
    arm(&tap);
    steps(&mut runtime, 3);
    let mut fires = shapes(&read_all(&tap));
    fires.sort_by_key(|&(at_ns, node, ..)| (at_ns, node));
    (runtime, fires)
}

#[test]
fn a_panic_in_the_count_probe_before_a_tick_is_that_node_s_tick_panic() {
    let (runtime, fires) = run_with_probe_panic(2, Arc::default());
    assert_eq!(
        runtime.node_tick_panics(),
        vec![("fragile".to_string(), PROBE_PANIC.to_string())]
    );
    assert_eq!(
        fires,
        vec![
            (ms(1), STEADY, PERIOD, NONE, 0b1),
            (ms(2), STEADY, PERIOD, NONE, 0b1),
            (ms(3), STEADY, PERIOD, NONE, 0b1),
        ],
        "the fragile node never ticked and is dead; the steady node keeps recording"
    );
}

#[test]
fn a_panic_in_the_count_probe_after_a_tick_records_a_panicked_fire() {
    let (runtime, fires) = run_with_probe_panic(3, Arc::default());
    assert_eq!(
        runtime.node_tick_panics(),
        vec![("fragile".to_string(), PROBE_PANIC.to_string())]
    );
    assert_eq!(
        fires,
        vec![
            (ms(1), STEADY, PERIOD, NONE, 0b1),
            (
                ms(1),
                FRAGILE,
                PERIOD,
                FireFlags::PANICKED | FireFlags::WROTE_UNKNOWN,
                0
            ),
            (ms(2), STEADY, PERIOD, NONE, 0b1),
            (ms(3), STEADY, PERIOD, NONE, 0b1),
        ],
        "the fragile node's one fire is a panic; it does not fire again"
    );
}

#[test]
fn a_panic_in_the_count_probe_at_install_reports_no_writes_and_the_node_keeps_ticking() {
    let probes = Arc::new(AtomicU64::new(0));
    let (runtime, fires) = run_with_probe_panic(1, Arc::clone(&probes));
    let reports: Vec<(String, bool)> = runtime
        .run_topology()
        .nodes
        .iter()
        .map(|node| (node.id.clone(), node.reports_writes))
        .collect();
    assert_eq!(
        reports,
        vec![("steady".to_string(), true), ("fragile".to_string(), false)]
    );
    assert_eq!(runtime.node_tick_panics(), vec![]);
    assert_eq!(
        fires,
        vec![
            (ms(1), STEADY, PERIOD, NONE, 0b1),
            (ms(1), FRAGILE, PERIOD, FireFlags::WROTE_UNKNOWN, 0),
            (ms(2), STEADY, PERIOD, NONE, 0b1),
            (ms(2), FRAGILE, PERIOD, FireFlags::WROTE_UNKNOWN, 0),
            (ms(3), STEADY, PERIOD, NONE, 0b1),
            (ms(3), FRAGILE, PERIOD, FireFlags::WROTE_UNKNOWN, 0),
        ],
        "the install probe's panic poisons nothing: both nodes keep ticking"
    );
    assert_eq!(
        probes.load(Ordering::Relaxed),
        1,
        "an entry that reports no writes at install is not asked again"
    );
}

#[test]
fn run_topology_reports_writes_only_after_the_tap_probes_entries() {
    let mut runtime = alternating_graph();
    assert!(runtime
        .run_topology()
        .nodes
        .iter()
        .all(|node| !node.reports_writes));
    let _tap = runtime.install_fire_tap(config(Duration::from_secs(5)));
    assert!(runtime
        .run_topology()
        .nodes
        .iter()
        .all(|node| node.reports_writes));
}

#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct Beat {
    #[output]
    count: Vector3,
    #[output]
    spare: Vector3,
    n: u64,
}

#[cerulion_node_impl]
impl Beat {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.count.x = self.n as f64;
        self.spare.x = 0.0;
        Ok(())
    }
}

const BEAT: &str = r#"
prefix: fires
nodes:
  - id: beat
    type: beat
    outputs:
      - {name: count, schema: geometry_msgs/Vector3}
      - {name: spare, schema: geometry_msgs/Vector3}
"#;

#[test]
fn macro_node_reports_its_writes() {
    let mut runtime = build(BEAT, vec![("beat", Box::new(BeatEntry::new()))]);
    let tap = runtime.install_fire_tap(config(Duration::from_secs(5)));
    assert!(runtime.run_topology().nodes[0].reports_writes);

    arm(&tap);
    steps(&mut runtime, 2);
    assert_eq!(
        shapes(&read_all(&tap)),
        vec![
            (ms(1), 0, PERIOD, NONE, 0b11),
            (ms(2), 0, PERIOD, NONE, 0b11),
        ]
    );
}

const DISCARDING: &str = r#"
prefix: fires
nodes:
  - id: source
    type: discarding_source
    outputs:
      - {name: kept, schema: geometry_msgs/Vector3}
      - {name: dropped, schema: geometry_msgs/Vector3}
"#;

#[test]
fn discarded_loan_sets_no_write_bit() {
    let source = ClosureNodeEntry::new(period_info(&["kept", "dropped"]), |ctx| {
        publish(ctx, "kept", 1.0)?;
        let mut proxy = ctx
            .publisher_mut("dropped")
            .expect("output is wired")
            .loan_proxy::<Vector3>()?;
        proxy.x = 2.0;
        proxy.__cer_defer_publish();
        Ok(())
    });
    let mut runtime = build(DISCARDING, vec![("source", Box::new(source))]);
    let tap = runtime.install_fire_tap(config(Duration::from_secs(5)));
    arm(&tap);
    steps(&mut runtime, 2);
    assert_eq!(
        shapes(&read_all(&tap)),
        vec![
            (ms(1), 0, PERIOD, NONE, 0b01),
            (ms(2), 0, PERIOD, NONE, 0b01),
        ]
    );
}

const JOIN: &str = r#"
prefix: fires
nodes:
  - id: source
    type: alternating_source
    outputs:
      - {name: a, schema: geometry_msgs/Vector3}
      - {name: b, schema: geometry_msgs/Vector3}
  - id: join
    type: join
    inputs:
      - {name: a, source: source/a}
      - {name: b, source: source/b}
"#;

#[test]
fn two_input_default_data_node_records_trigger_ambiguous() {
    let join = ClosureNodeEntry::new(
        NodeInfo::from_names(vec!["a".into(), "b".into()], vec![]),
        |_| Ok(()),
    );
    let mut runtime = build(
        JOIN,
        vec![
            ("source", Box::new(alternating_source())),
            ("join", Box::new(join)),
        ],
    );
    let tap = runtime.install_fire_tap(config(Duration::from_secs(5)));
    assert!(matches!(
        runtime.run_topology().nodes[1].trigger,
        TopologyTrigger::Data {
            ambiguous: true,
            ..
        }
    ));

    arm(&tap);
    // A node without a declared trigger input has no drain binding: it fires
    // when its host signals data, and either input could be the cause.
    for _ in 0..2 {
        runtime.signal_data("join").expect("join is a node");
        runtime.step(STEP);
    }
    let join_fires: Vec<_> = read_all(&tap)
        .records
        .iter()
        .filter(|record| record.node == 1)
        .map(shape)
        .collect();
    assert_eq!(
        join_fires,
        vec![
            (ms(1), 1, DATA, FireFlags::TRIGGER_AMBIGUOUS, 0),
            (ms(2), 1, DATA, FireFlags::TRIGGER_AMBIGUOUS, 0),
        ]
    );
}

/// One read's `in_tick`: `(node, tick start)` pairs.
type InTick = Vec<(u16, u64)>;

#[test]
fn a_read_inside_a_tick_lists_that_node_in_tick() {
    let shared: Arc<OnceLock<Arc<FireTap>>> = Arc::new(OnceLock::new());
    let seen: Arc<Mutex<Vec<InTick>>> = Arc::default();
    let (tap_in_tick, seen_in_tick) = (Arc::clone(&shared), Arc::clone(&seen));
    let source = ClosureNodeEntry::new(period_info(&["a"]), move |ctx| {
        if let Some(tap) = tap_in_tick.get() {
            let batch = tap.serve(Cursor::Now, 0);
            seen_in_tick.lock().unwrap().push(batch.in_tick);
        }
        publish(ctx, "a", 1.0)
    });
    let mut runtime = build(
        BOMB,
        vec![
            ("source", Box::new(source)),
            (
                "bomb",
                Box::new(ClosureNodeEntry::new(data_info("a", &[]), |_| Ok(()))),
            ),
        ],
    );
    let tap = runtime.install_fire_tap(config(Duration::from_secs(5)));
    shared.set(Arc::clone(&tap)).unwrap();
    steps(&mut runtime, 2);

    assert_eq!(
        *seen.lock().unwrap(),
        vec![vec![(0, ms(1))], vec![(0, ms(2))]]
    );
    let batch = tap.serve(Cursor::Now, 0);
    assert!(
        batch.in_tick.is_empty(),
        "no node is inside a tick between steps"
    );
}

#[test]
fn arming_changes_no_fire_and_two_armed_runs_record_the_same_fires() {
    let run = |armed: bool| {
        let mut runtime = alternating_graph();
        runtime.set_trace_limit(64);
        let tap = runtime.install_fire_tap(config(Duration::from_secs(5)));
        if armed {
            arm(&tap);
        }
        steps(&mut runtime, 8);
        let trace = runtime.trace().to_vec();
        (trace, shapes(&read_all(&tap)))
    };
    let (armed_trace, armed_fires) = run(true);
    let (again_trace, again_fires) = run(true);
    let (disarmed_trace, disarmed_fires) = run(false);

    assert_eq!(armed_trace.len(), 12);
    assert_eq!(
        armed_trace, disarmed_trace,
        "recording never changes a fire"
    );
    assert_eq!(armed_trace, again_trace);
    assert_eq!(armed_fires.len(), 12);
    assert_eq!(armed_fires, again_fires);
    assert!(disarmed_fires.is_empty());
}

const CHAIN: &str = r#"
prefix: fires
nodes:
  - id: source
    type: source
    outputs:
      - {name: a, schema: geometry_msgs/Vector3}
  - id: relay
    type: relay
    inputs:
      - {name: a, source: source/a}
    outputs:
      - {name: out, schema: geometry_msgs/Vector3}
"#;

#[test]
fn overhead_of_100k_relay_ticks_armed_versus_disarmed_is_printed() {
    const TICKS: usize = 100_000;
    const ROUNDS: usize = 3;
    let relay_ticks = Arc::new(AtomicU64::new(0));
    let time_run = |armed: bool, ticks: usize| {
        let relay = relay(Arc::clone(&relay_ticks));
        let source = ClosureNodeEntry::new(period_info(&["a"]), |ctx| publish(ctx, "a", 1.0));
        let mut runtime = build(
            CHAIN,
            vec![("source", Box::new(source)), ("relay", Box::new(relay))],
        );
        let tap = runtime.install_fire_tap(FireTapConfig {
            capacity: 4096,
            lease: Duration::from_secs(3600),
        });
        if armed {
            arm(&tap);
        }
        let before = relay_ticks.load(Ordering::Relaxed);
        let started = Instant::now();
        steps(&mut runtime, ticks);
        let elapsed = started.elapsed();
        assert_eq!(
            relay_ticks.load(Ordering::Relaxed) - before,
            ticks as u64,
            "one relay tick per step"
        );
        assert_eq!(
            tap.stats().recorded,
            if armed { 2 * ticks as u64 } else { 0 }
        );
        elapsed
    };
    // A first run pays one-time costs, so each posture warms once. The
    // postures then alternate which goes first, and each keeps its fastest
    // round, so neither one owns the quiet or the noisy part of the run.
    time_run(false, TICKS / 10);
    time_run(true, TICKS / 10);
    let (mut disarmed, mut armed) = (Duration::MAX, Duration::MAX);
    for round in 0..ROUNDS {
        let order = if round % 2 == 0 {
            [false, true]
        } else {
            [true, false]
        };
        for posture in order {
            let elapsed = time_run(posture, TICKS);
            let best = if posture { &mut armed } else { &mut disarmed };
            *best = (*best).min(elapsed);
        }
    }
    eprintln!(
        "fire tap overhead over {TICKS} steps (one source and one relay tick each), \
         fastest of {ROUNDS} alternating rounds: disarmed {:?} per step, armed {:?} per step",
        disarmed / TICKS as u32,
        armed / TICKS as u32
    );
}
