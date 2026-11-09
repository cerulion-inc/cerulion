// SPDX-License-Identifier: AGPL-3.0-only
//! A chain member called straight after its chain head's level fires exactly
//! as it fires through the level loop.
//!
//! Under `CERULION_FUSE_CHAINS=1` every member of a census chain is called
//! from the step thread right after the level of its chain head ends: the
//! member drains its own trigger input, decides, freezes its latest-value
//! inputs and ticks, and its trace entries are held in its own buffer until
//! the level loop reaches the member's level, where they join the trace at
//! the member's graph position. The publish of every frame is unchanged.
//!
//! The graph below is built twice, once queued and once with chain calls,
//! and stepped through the same clock deltas, bursts included. Three logs
//! must match exactly:
//!
//! - the fire trace: node, step, fire time, level and the discard bit of
//!   every entry, in order (a skipped frame writes no output, so the skip shows
//!   as fewer fires downstream);
//! - the read log: every staged read outcome the level-end merge emits, with
//!   each producer annotation's token resolved to the publisher it names;
//! - the frames: every value every node read, in order.
//!
//! Every log row has a count floor, and the chain-call counter must read the
//! expected number of calls on the fused run and zero on the queued run, so
//! a run in which nothing was called cannot pass.
//!
//! The graph puts members on levels that also hold non-member nodes, before
//! and after them in graph order, so an entry written at call time rather
//! than at the member's position is out of order. It has three chains with
//! three different head levels, a member that reads a latest value written
//! before its head's level, a node that reads a member's second output as a
//! latest value after the member's level, members that skip a frame, and
//! steps that carry several period fires.
//!
//! A second graph pins the unit boundary: `src -> mid -> far` is one chain in
//! one level loop, and with `far` declaring `isolate: true` the executor-unit
//! derivation puts it in a unit of its own. Each unit runs its own runtime over
//! its own subgraph, where `far` reads the absolute topic `mid` publishes, so
//! the chain call stays inside the first unit and the hop between the units is
//! a queued one that carries every frame, with every node firing at the steps
//! and times the queued run of the whole graph fires it.
//!
//! A third graph gives a chain member a `consume = latest` trigger input.
//! Called, it fires once per step on the newest frame, as its queued twin
//! does. On the separate drain discipline, where the step asks the member's
//! body subscriber whether a read finds a frame before it fires the member,
//! the member's binding is no unified one, so the chain call is cut there and
//! the member runs at its own level.
//!
//! The build's log is checked as well. With the seam on, a chain is cut at
//! the first member a call cannot drain, and the build logs one warning per
//! cut naming the chain head, the member and the reason, then one line naming
//! the members it calls, or saying it calls none and why. In
//! `src -> one -> bare -> end`, where `bare` is a hand-written entry with no
//! trigger policy and so no data-trigger binding, the build calls `one` alone;
//! the `consume = latest` graph on the separate drain calls none. With the seam
//! off or unset the build logs none of these lines.
//!
//! ```bash
//! cargo test -p cerulion_core --test chain_call_identity_test -- --test-threads=1
//! ```

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::chain::{census_chains, ChainBar, Colocation};
use cerulion_core::graph::node::{NodeContext, NodeEntry, NodeInfo};
use cerulion_core::graph::topology::GraphTopology;
use cerulion_core::graph::{
    build_trigger_edges, derive_units, parse_graph, resolve_levels, resolve_source, validate_graph,
    Boundary, GraphConfig, GraphRuntime, UnitGraph, FUSE_CHAINS_ENV,
};
use cerulion_core::prelude::*;
use cerulion_core::scheduler::{MergedReadOutcome, TraceEntry};
use cerulion_core::transport::{TransportConfig, TransportManager};
use indexmap::IndexMap;
use native_ros2_messages::std_msgs::Int32;
use serial_test::serial;
use tracing_test::traced_test;

/// Every value every node read, tagged with the node and input it came
/// through, in tick order. The comparison groups it per tag: a called member
/// ticks earlier in wall time than its queued twin, so only each stream's own
/// order is the same program.
static FRAMES: Mutex<Vec<(&'static str, i32)>> = Mutex::new(Vec::new());

fn note(node: &'static str, value: i32) {
    FRAMES.lock().expect("not poisoned").push((node, value));
}

/// Steps per run, and the clock delta of each, in milliseconds. A delta above
/// the 1 ms period makes every period node fire that many times in one step,
/// so its consumers see several frames in one step.
const STEPS: usize = 600;
const DELTAS_MS: [u64; 7] = [1, 1, 3, 1, 2, 1, 4];

/// Chain calls a fused run makes per step at most: head -> a -> b -> sink is
/// three, p2 -> q2 -> r2 two, x3 -> late one.
const MEMBERS: usize = 6;

/// Every node that reads, and how many inputs it reads through: each must
/// log read outcomes on every input.
const READ_INPUTS: &[(&str, u16)] = &[
    ("fan", 1),
    ("q2", 1),
    ("z", 3),
    ("head", 1),
    ("x2", 1),
    ("r2", 1),
    ("a", 2),
    ("x3", 1),
    ("b", 1),
    ("late", 2),
    ("sink", 1),
];

// ---------------------------------------------------------------------------
// The nodes.
// ---------------------------------------------------------------------------

#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct Count {
    #[output]
    out: Int32,
    n: i32,
}

#[cerulion_node_impl]
impl Count {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.data = self.n;
        Ok(())
    }
}

/// One relay type per relay node, so each notes its frames under its own id:
/// forwards its input plus one.
macro_rules! relay {
    ($ty:ident, $id:literal) => {
        #[cerulion_node]
        #[derive(Default)]
        struct $ty {
            #[input(trigger)]
            inp: Int32,
            #[output]
            out: Int32,
        }

        #[cerulion_node_impl]
        impl $ty {
            fn tick(&mut self) -> Result<(), NodeError> {
                let v = self.inp.data;
                note($id, v);
                self.out.data = v + 1;
                Ok(())
            }
        }
    };
}

relay!(FanRelay, "fan");
relay!(HeadRelay, "head");
relay!(X2Relay, "x2");
relay!(X3Relay, "x3");
relay!(BRelay, "b");

/// The chain member with context: forwards `inp * 100 + ctx` on `out` and
/// `inp` on `aux`, and skips every seventh frame (neither output written).
#[cerulion_node]
#[derive(Default)]
struct Mixer {
    #[input(trigger)]
    inp: Int32,
    #[input]
    ctx: Int32,
    #[output]
    out: Int32,
    #[output]
    aux: Int32,
}

#[cerulion_node_impl]
impl Mixer {
    fn tick(&mut self) -> Result<(), NodeError> {
        let (v, c) = (self.inp.data, self.ctx.data);
        note("a.inp", v);
        note("a.ctx", c);
        if v % 7 == 0 {
            return Ok(());
        }
        self.out.data = v.wrapping_mul(100).wrapping_add(c);
        self.aux.data = v;
        Ok(())
    }
}

/// A relay that skips every fifth frame.
#[cerulion_node]
#[derive(Default)]
struct Skipper {
    #[input(trigger)]
    inp: Int32,
    #[output]
    out: Int32,
}

#[cerulion_node_impl]
impl Skipper {
    fn tick(&mut self) -> Result<(), NodeError> {
        let v = self.inp.data;
        note("q2", v);
        if v % 5 == 0 {
            return Ok(());
        }
        self.out.data = v + 1;
        Ok(())
    }
}

/// Fired by one input, reads two more as latest values.
#[cerulion_node]
#[derive(Default)]
struct Watcher {
    #[input(trigger)]
    inp: Int32,
    #[input]
    first: Int32,
    #[input]
    second: Int32,
}

#[cerulion_node_impl]
impl Watcher {
    fn tick(&mut self) -> Result<(), NodeError> {
        note("z.inp", self.inp.data);
        note("z.first", self.first.data);
        note("z.second", self.second.data);
        Ok(())
    }
}

/// Fired by one input, reads one more as a latest value, publishes nothing.
#[cerulion_node]
#[derive(Default)]
struct Tail {
    #[input(trigger)]
    inp: Int32,
    #[input]
    side: Int32,
}

#[cerulion_node_impl]
impl Tail {
    fn tick(&mut self) -> Result<(), NodeError> {
        note("late.inp", self.inp.data);
        note("late.side", self.side.data);
        Ok(())
    }
}

/// One sink type per sink node: fired by one input, publishes nothing.
macro_rules! sink {
    ($ty:ident, $id:literal) => {
        #[cerulion_node]
        #[derive(Default)]
        struct $ty {
            #[input(trigger)]
            inp: Int32,
        }

        #[cerulion_node_impl]
        impl $ty {
            fn tick(&mut self) -> Result<(), NodeError> {
                note($id, self.inp.data);
                Ok(())
            }
        }
    };
}

sink!(R2Sink, "r2");
sink!(TailSink, "sink");

// ---------------------------------------------------------------------------
// The graph.
//
// Levels (derived): L0 ctx, tick, p2; L1 fan, q2, z, head; L2 x2, r2, a;
// L3 x3, b; L4 late, sink.
// Chains: head -> a -> b -> sink (head L1), p2 -> q2 -> r2 (head L0),
// x3 -> late (head L3). Everything else keeps the queued path: tick/out,
// fan/out and x2/out each have two readers, ctx/out is read by a latest
// value. `a` reads ctx (L0) as a latest value, before its head's level;
// `late` reads a/aux (a is at L2) at L4, after a's level.
// ---------------------------------------------------------------------------

const GRAPH: &str = r#"
prefix: test/chain_call_identity
nodes:
  - id: ctx
    type: count
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: tick
    type: count
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: p2
    type: count
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: fan
    type: relay
    inputs:
      - name: inp
        source: tick/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: q2
    type: skipper
    inputs:
      - name: inp
        source: p2/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: z
    type: watcher
    inputs:
      - name: inp
        source: ctx/out
      - name: first
        source: fan/out
      - name: second
        source: x2/out
  - id: head
    type: relay
    inputs:
      - name: inp
        source: tick/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: x2
    type: relay
    inputs:
      - name: inp
        source: fan/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: r2
    type: sink
    inputs:
      - name: inp
        source: q2/out
  - id: a
    type: mixer
    inputs:
      - name: inp
        source: head/out
      - name: ctx
        source: ctx/out
    outputs:
      - name: out
        schema: std_msgs/Int32
      - name: aux
        schema: std_msgs/Int32
  - id: x3
    type: relay
    inputs:
      - name: inp
        source: x2/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: b
    type: relay
    inputs:
      - name: inp
        source: a/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: late
    type: tail
    inputs:
      - name: inp
        source: x3/out
      - name: side
        source: a/aux
  - id: sink
    type: sink
    inputs:
      - name: inp
        source: b/out
"#;

fn factories() -> IndexMap<String, Box<dyn NodeEntry>> {
    let mut f: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for id in ["ctx", "tick", "p2"] {
        f.insert(id.into(), Box::new(CountEntry::new()));
    }
    f.insert("fan".into(), Box::new(FanRelayEntry::new()));
    f.insert("head".into(), Box::new(HeadRelayEntry::new()));
    f.insert("x2".into(), Box::new(X2RelayEntry::new()));
    f.insert("x3".into(), Box::new(X3RelayEntry::new()));
    f.insert("b".into(), Box::new(BRelayEntry::new()));
    f.insert("q2".into(), Box::new(SkipperEntry::new()));
    f.insert("z".into(), Box::new(WatcherEntry::new()));
    f.insert("a".into(), Box::new(MixerEntry::new()));
    f.insert("late".into(), Box::new(TailEntry::new()));
    f.insert("r2".into(), Box::new(R2SinkEntry::new()));
    f.insert("sink".into(), Box::new(TailSinkEntry::new()));
    f
}

/// Removes the seam's variable on drop, panic included.
struct FuseEnv;

impl FuseEnv {
    fn set(value: &str) -> Self {
        std::env::set_var(FUSE_CHAINS_ENV, value);
        FuseEnv
    }
}

impl Drop for FuseEnv {
    fn drop(&mut self) {
        std::env::remove_var(FUSE_CHAINS_ENV);
    }
}

fn build(fused: bool) -> GraphRuntime {
    let _env = FuseEnv::set(if fused { "1" } else { "0" });
    let config = parse_graph(GRAPH).expect("the fixture graph must parse");
    validate_graph(&config).expect("the fixture graph must validate");
    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories(), clock, 8).expect("the fixture must build")
}

/// One trace entry, every field a replay reproduces plus the discard bit.
type TraceRow = (String, u64, u64, usize, bool);

fn trace_rows(trace: &[TraceEntry]) -> Vec<TraceRow> {
    trace
        .iter()
        .map(|e| {
            (
                e.node_id.to_string(),
                e.step,
                e.fire_time_ns,
                e.global_level,
                e.discarded,
            )
        })
        .collect()
}

/// What one run produced.
struct Run {
    trace: Vec<TraceRow>,
    reads: Vec<MergedReadOutcome>,
    frames: Vec<(&'static str, i32)>,
    calls: u64,
}

fn run(fused: bool) -> Run {
    step_through(build(fused))
}

/// Step `runtime` through the clock deltas and collect what it produced.
fn step_through(mut runtime: GraphRuntime) -> Run {
    FRAMES.lock().expect("not poisoned").clear();
    runtime.set_trace_limit(usize::MAX);
    runtime.enable_read_outcome_memory_sink();
    let mut reads = Vec::new();
    for i in 0..STEPS {
        runtime.step(Duration::from_millis(DELTAS_MS[i % DELTAS_MS.len()]));
        reads.extend(runtime.take_read_outcomes());
    }
    let reads = resolve_producer_tokens(&runtime, reads);
    let trace = trace_rows(runtime.trace());
    let calls = runtime.chain_calls_made();
    let frames = std::mem::take(&mut *FRAMES.lock().expect("not poisoned"));
    Run {
        trace,
        reads,
        frames,
        calls,
    }
}

/// The first index at which two sequences differ, for a readable failure.
fn first_difference<T: PartialEq + std::fmt::Debug>(a: &[T], b: &[T]) -> Option<String> {
    let n = a.len().min(b.len());
    for i in 0..n {
        if a[i] != b[i] {
            return Some(format!(
                "first difference at {i}: queued {:?}, fused {:?}",
                a[i], b[i]
            ));
        }
    }
    (a.len() != b.len()).then(|| format!("lengths differ: queued {}, fused {}", a.len(), b.len()))
}

/// `reads` with each producer annotation's token replaced by the place of the
/// publisher it names in `runtime`'s publisher table: the node's position in
/// the high 32 bits, the output's position in the low 32.
///
/// A token hashes its publisher's id, which the transport mints afresh in
/// every build, so two builds of one graph annotate the same publisher with
/// different tokens. Each run resolves its own tokens through its own table.
/// A token the table does not hold, two publishers whose ids hash to one
/// token, and an input that logs reads but no annotation fail the run.
fn resolve_producer_tokens(
    runtime: &GraphRuntime,
    reads: Vec<MergedReadOutcome>,
) -> Vec<MergedReadOutcome> {
    let mut place_of: std::collections::HashMap<u64, u64> = Default::default();
    for (n, (node, outputs)) in runtime.read_log_publisher_ids().iter().enumerate() {
        for (o, (output, id)) in outputs.iter().enumerate() {
            let token = cerulion_core::wire::fnv1a_hash(&id.to_le_bytes());
            let place = ((n as u64) << 32) | o as u64;
            if let Some(other) = place_of.insert(token, place) {
                panic!("{node}/{output} hashes to token {token:#x}, as place {other:#x} does");
            }
        }
    }
    let mut annotated: std::collections::BTreeMap<(&str, u16), usize> = Default::default();
    for r in &reads {
        *annotated
            .entry((r.node_id.as_str(), r.input_idx))
            .or_default() += usize::from(r.outcome.token.is_some());
    }
    for ((node, input), n) in &annotated {
        assert!(
            *n > 0,
            "{node} input {input} logs reads but no producer annotation"
        );
    }
    reads
        .into_iter()
        .map(|mut r| {
            if let Some(token) = r.outcome.token {
                let place = place_of.get(&token).unwrap_or_else(|| {
                    panic!(
                        "{} input {} at step {}: token {token:#x} names no publisher of this build",
                        r.node_id, r.input_idx, r.step
                    )
                });
                r.outcome.token = Some(*place);
            }
            r
        })
        .collect()
}

/// Every node fires at least this many times per run; the slowest stream is
/// a member that skips one frame in five behind a relay that skips one in
/// seven.
const FIRE_FLOOR_PER_NODE: usize = STEPS / 2;

fn assert_floors(run: &Run, label: &str) {
    for node in [
        "ctx", "tick", "p2", "fan", "q2", "z", "head", "x2", "r2", "a", "x3", "b", "late", "sink",
    ] {
        let fires = run.trace.iter().filter(|r| r.0 == node).count();
        assert!(
            fires >= FIRE_FLOOR_PER_NODE,
            "{label}: {node} fired {fires} times, under the floor {FIRE_FLOOR_PER_NODE}"
        );
    }
    // A member that skips a frame writes no output, so the next member fires
    // fewer times than it does.
    let fires = |node: &str| run.trace.iter().filter(|r| r.0 == node).count();
    assert!(
        fires("b") + STEPS / 10 <= fires("a"),
        "{label}: a fired {} times and b {}; the skipped frames must not reach b",
        fires("a"),
        fires("b")
    );
    assert!(
        fires("r2") + STEPS / 10 <= fires("q2"),
        "{label}: q2 fired {} times and r2 {}; the skipped frames must not reach r2",
        fires("q2"),
        fires("r2")
    );
    let bursts = run
        .trace
        .windows(2)
        .filter(|w| w[0].0 == "sink" && w[1].0 == "sink" && w[0].1 == w[1].1)
        .count();
    assert!(
        bursts >= STEPS / 20,
        "{label}: {bursts} steps where the chain tail fired twice; the bursts must reach it"
    );
    let mut per_input: std::collections::BTreeMap<(String, u16), usize> = Default::default();
    for r in &run.reads {
        *per_input
            .entry((r.node_id.clone(), r.input_idx))
            .or_default() += 1;
    }
    eprintln!("{label}: read outcomes per input {per_input:?}");
    for (node, inputs) in READ_INPUTS {
        for idx in 0..*inputs {
            let n = per_input
                .get(&(node.to_string(), idx))
                .copied()
                .unwrap_or(0);
            assert!(
                n >= STEPS / 4,
                "{label}: {node} input {idx} logged {n} read outcomes, under the floor {}",
                STEPS / 4
            );
        }
    }
    let mut per_tag: std::collections::BTreeMap<&str, usize> = Default::default();
    for (tag, _) in &run.frames {
        *per_tag.entry(tag).or_default() += 1;
    }
    assert_eq!(
        per_tag.len(),
        15,
        "{label}: frame streams {:?}",
        per_tag.keys()
    );
    for (tag, n) in &per_tag {
        assert!(
            *n >= STEPS / 4,
            "{label}: stream {tag} noted {n} frames, under the floor {}",
            STEPS / 4
        );
    }
}

fn compare(queued: &Run, fused: &Run) -> Result<(), String> {
    if let Some(d) = first_difference(&queued.trace, &fused.trace) {
        return Err(format!("fire trace: {d}"));
    }
    if let Some(d) = first_difference(&queued.reads, &fused.reads) {
        return Err(format!("read log: {d}"));
    }
    let streams = |frames: &[(&'static str, i32)]| {
        let mut by: std::collections::BTreeMap<&'static str, Vec<i32>> = Default::default();
        for (tag, v) in frames {
            by.entry(tag).or_default().push(*v);
        }
        by
    };
    let (q, f) = (streams(&queued.frames), streams(&fused.frames));
    if q.keys().ne(f.keys()) {
        return Err(format!(
            "frames: streams differ: queued {:?}, fused {:?}",
            q.keys().collect::<Vec<_>>(),
            f.keys().collect::<Vec<_>>()
        ));
    }
    for (tag, qv) in &q {
        if let Some(d) = first_difference(qv, &f[tag]) {
            return Err(format!("frames of {tag}: {d}"));
        }
    }
    Ok(())
}

#[test]
#[serial]
fn a_chain_called_run_matches_the_queued_run_entry_for_entry() {
    let queued = run(false);
    let fused = run(true);
    assert_eq!(queued.calls, 0, "the queued run must make no chain call");
    // One call per member per step, due or not: a chain the census lost
    // would show here.
    assert_eq!(
        fused.calls,
        (STEPS * MEMBERS) as u64,
        "the fused run must call every member of the three chains every step"
    );
    assert_floors(&queued, "queued");
    assert_floors(&fused, "fused");
    if let Err(e) = compare(&queued, &fused) {
        panic!("{e}");
    }
}

/// The seam's off value and an unset seam both run the queued path, so the
/// identity above is against the path every graph runs today.
#[test]
#[serial]
fn an_unset_seam_makes_no_chain_call() {
    std::env::remove_var(FUSE_CHAINS_ENV);
    let config = parse_graph(GRAPH).expect("the fixture graph must parse");
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories(), clock, 8)
        .expect("the fixture must build");
    runtime.set_trace_limit(usize::MAX);
    for _ in 0..50 {
        runtime.step(Duration::from_millis(1));
    }
    let n = runtime.trace().len();
    assert!(n >= 50 * 10, "the graph must run; {n} fires in 50 steps");
    assert_eq!(runtime.chain_calls_made(), 0);
}

// ---------------------------------------------------------------------------
// A chain member whose trigger input is `consume = latest`.
//
// `pace -> pick -> last`: `pace` fires every 1 ms, so a step of d ms puts d
// frames on `pick`'s input. `pick` fires once per step on the newest of them,
// and `last` reads every frame `pick` publishes. The census chains all three,
// and on the unified drain `pick` and `last` are called; on the separate
// drain, where the step asks `pick`'s body subscriber whether a read finds a
// frame before it fires `pick`, the chain call is cut at `pick` and both run
// at their own levels.
// ---------------------------------------------------------------------------

/// Forwards the newest frame its `consume = latest` input holds.
#[cerulion_node]
#[derive(Default)]
struct Picker {
    #[input(trigger, consume = latest)]
    inp: Int32,
    #[output]
    out: Int32,
}

#[cerulion_node_impl]
impl Picker {
    fn tick(&mut self) -> Result<(), NodeError> {
        let v = self.inp.data;
        note("pick", v);
        self.out.data = v;
        Ok(())
    }
}

sink!(LastSink, "last");

const LATEST_GRAPH: &str = r#"
prefix: test/chain_call_latest
nodes:
  - id: pace
    type: count
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: pick
    type: picker
    inputs:
      - name: inp
        source: pace/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: last
    type: sink
    inputs:
      - name: inp
        source: pick/out
"#;

/// Removes the drain-discipline seam's variable on drop, panic included.
struct DrainEnv;

impl Drop for DrainEnv {
    fn drop(&mut self) {
        std::env::remove_var("CERULION_DRAIN_DISCIPLINE");
    }
}

fn latest_factories() -> IndexMap<String, Box<dyn NodeEntry>> {
    let mut f: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    f.insert("pace".into(), Box::new(CountEntry::new()));
    f.insert("pick".into(), Box::new(PickerEntry::new()));
    f.insert("last".into(), Box::new(LastSinkEntry::new()));
    f
}

fn latest_run(fused: bool, separate: bool) -> Run {
    let _drain = separate.then(|| {
        std::env::set_var("CERULION_DRAIN_DISCIPLINE", "separate");
        DrainEnv
    });
    let runtime = {
        let _env = FuseEnv::set(if fused { "1" } else { "0" });
        let config = parse_graph(LATEST_GRAPH).expect("the latest graph must parse");
        validate_graph(&config).expect("the latest graph must validate");
        GraphRuntime::build_for_test(config, latest_factories(), Arc::new(VirtualClock::new()), 8)
            .expect("the latest graph must build")
    };
    step_through(runtime)
}

/// `pick` read the newest frame of every step: `pace` publishes its fire
/// count, so the frame `pick` reads at step `s` is the number of `pace` fires
/// at or before `s`. `last` reads every frame `pick` publishes, in order.
fn assert_latest_oracle(run: &Run, label: &str) {
    let pace: Vec<u64> = run
        .trace
        .iter()
        .filter(|r| r.0 == "pace")
        .map(|r| r.1)
        .collect();
    let pick_steps: Vec<u64> = run
        .trace
        .iter()
        .filter(|r| r.0 == "pick")
        .map(|r| r.1)
        .collect();
    let picked = stream(&run.frames, "pick");
    assert_eq!(
        pick_steps.len(),
        STEPS,
        "{label}: pick fires once in every step"
    );
    assert!(
        pace.len() >= STEPS + STEPS / 2,
        "{label}: pace fired {} times in {STEPS} steps; the multi-fire steps must reach pick",
        pace.len()
    );
    assert_eq!(
        picked.len(),
        pick_steps.len(),
        "{label}: one frame per fire"
    );
    for (s, v) in pick_steps.iter().zip(&picked) {
        let newest = pace.iter().filter(|p| *p <= s).count();
        assert_eq!(
            usize::try_from(*v).expect("a fire count"),
            newest,
            "{label}: pick read {v} at step {s}, the newest frame then is {newest}"
        );
    }
    assert_eq!(
        stream(&run.frames, "last"),
        picked,
        "{label}: last reads every frame pick publishes"
    );
    assert!(
        run.reads.iter().any(|r| r.node_id == "pick"),
        "{label}: pick logs its reads"
    );
}

#[test]
#[serial]
fn a_chain_member_with_a_latest_trigger_input_is_called_as_it_fires_queued() {
    for separate in [false, true] {
        let label = if separate { "separate" } else { "unified" };
        let queued = latest_run(false, separate);
        let fused = latest_run(true, separate);
        assert_eq!(
            queued.calls, 0,
            "{label}: the queued run makes no chain call"
        );
        let want = if separate { 0 } else { (STEPS * 2) as u64 };
        assert_eq!(
            fused.calls, want,
            "{label}: pick and last are called every step on the unified drain, and \
             neither on the separate one"
        );
        assert_latest_oracle(&queued, &format!("{label}, queued"));
        assert_latest_oracle(&fused, &format!("{label}, fused"));
        if let Err(e) = compare(&queued, &fused) {
            panic!("{label}: {e}");
        }
    }
}

// ---------------------------------------------------------------------------
// A chain split across two executor units.
//
// `src -> mid -> far`, with no profile: the nodes of the process form one unit
// apart from each node that declares `isolate: true`. `far` declares it, so it
// runs in a unit of its own and `src` and `mid` share the other. In one level
// loop the census chains all three.
// ---------------------------------------------------------------------------

relay!(MidRelay, "mid");
sink!(FarSink, "far");

const SPLIT_GRAPH: &str = r#"
prefix: test/chain_unit_split
nodes:
  - id: src
    type: count
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: mid
    type: relay
    inputs:
      - name: inp
        source: src/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: far
    type: sink
    isolate: true
    inputs:
      - name: inp
        source: mid/out
"#;

fn split_factories(members: &[String]) -> IndexMap<String, Box<dyn NodeEntry>> {
    let mut f: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for id in members {
        let entry: Box<dyn NodeEntry> = match id.as_str() {
            "src" => Box::new(CountEntry::new()),
            "mid" => Box::new(MidRelayEntry::new()),
            "far" => Box::new(FarSinkEntry::new()),
            other => panic!("no node {other} in the split graph"),
        };
        f.insert(id.clone(), entry);
    }
    f
}

fn split_graph() -> GraphConfig {
    let config = parse_graph(SPLIT_GRAPH).expect("the split graph must parse");
    validate_graph(&config).expect("the split graph must validate");
    config
}

fn node_ids(config: &GraphConfig) -> Vec<String> {
    config.native_nodes().map(|n| n.id.clone()).collect()
}

fn split_infos(config: &GraphConfig) -> IndexMap<String, NodeInfo> {
    split_factories(&node_ids(config))
        .iter()
        .map(|(id, e)| (id.clone(), e.info().expect("a macro node has its info")))
        .collect()
}

/// `members` of `config` as one unit's runtime builds them: the other nodes
/// dropped, and an input whose producer is in another unit rewritten to the
/// absolute topic that producer publishes, as the CLI's worker subgraph
/// (`subgraph_for`) rewrites a cross-unit edge.
fn unit_subgraph(config: &GraphConfig, members: &[String]) -> GraphConfig {
    let mut sub = config.clone();
    sub.nodes.retain(|n| members.contains(&n.id));
    let produced: Vec<String> = sub
        .nodes
        .iter()
        .flat_map(|n| {
            n.outputs
                .iter()
                .map(|o| resolve_source(&config.prefix, &format!("{}/{}", n.id, o.name)))
        })
        .collect();
    for node in &mut sub.nodes {
        for input in &mut node.inputs {
            let resolved = resolve_source(&config.prefix, &input.source);
            if !produced.contains(&resolved) {
                input.source = resolved;
            }
        }
    }
    sub
}

/// One unit's transport over the shared isolated root `ix`.
///
/// The first unit owns `mid/out` and holds no consumer of it, so it gives the
/// service the buffer ceiling of this transport default; `far`'s unit opens
/// that service as a subscriber of depth `DEFAULT_CONSUMER_DEPTH` (10), which
/// 16 covers and 8 would not.
fn split_manager(
    name: String,
    ix: iceoryx2::config::Config,
    clock: Arc<VirtualClock>,
) -> Arc<TransportManager> {
    TransportManager::init_for_test(
        TransportConfig {
            node_name: name,
            clock,
            subscriber_buffer_size: 16,
            network: None,
        },
        ix,
    )
    .expect("an isolated transport must open")
}

/// One graph's runtimes stepped together: each runtime's chain-call count,
/// every frame each node read, and every runtime's fire trace.
struct SplitRun {
    calls: Vec<u64>,
    frames: Vec<(&'static str, i32)>,
    trace: Vec<TraceRow>,
}

/// Build one runtime per unit, in unit order, over one isolated shared-memory
/// root, and step each through the same clock deltas, the units of one step in
/// unit order.
fn run_units(units: &[Vec<String>], fused: bool) -> SplitRun {
    FRAMES.lock().expect("not poisoned").clear();
    let config = split_graph();
    let ix = cerulion_core::testing::iceoryx_test_config();
    let mut runtimes: Vec<(GraphRuntime, Arc<TransportManager>)> = Vec::new();
    for (i, members) in units.iter().enumerate() {
        let _env = FuseEnv::set(if fused { "1" } else { "0" });
        let clock = Arc::new(VirtualClock::new());
        let mgr = split_manager(
            format!("chain_unit_split_{i}"),
            ix.clone(),
            Arc::clone(&clock),
        );
        let mut runtime = GraphRuntime::build(
            unit_subgraph(&config, members),
            split_factories(members),
            &mgr,
            clock,
        )
        .expect("a unit's runtime must build");
        runtime.set_trace_limit(usize::MAX);
        runtimes.push((runtime, mgr));
    }
    for i in 0..STEPS {
        let delta = Duration::from_millis(DELTAS_MS[i % DELTAS_MS.len()]);
        for (runtime, _) in &mut runtimes {
            runtime.step(delta);
        }
    }
    let calls = runtimes.iter().map(|(r, _)| r.chain_calls_made()).collect();
    let trace = runtimes
        .iter_mut()
        .flat_map(|(r, _)| trace_rows(r.trace()))
        .collect();
    for (runtime, mgr) in runtimes {
        runtime.shutdown();
        drop(mgr);
    }
    let frames = std::mem::take(&mut *FRAMES.lock().expect("not poisoned"));
    SplitRun {
        calls,
        frames,
        trace,
    }
}

/// One node's fires: step, fire time and the discard bit of each, in order.
/// The level is left out: a unit's runtime levels its own subgraph, where
/// `far` sits on the unit's first level.
fn fires_of(trace: &[TraceRow], node: &str) -> Vec<(u64, u64, bool)> {
    trace
        .iter()
        .filter(|r| r.0 == node)
        .map(|r| (r.1, r.2, r.4))
        .collect()
}

fn stream(frames: &[(&'static str, i32)], tag: &str) -> Vec<i32> {
    frames
        .iter()
        .filter(|(t, _)| *t == tag)
        .map(|(_, v)| *v)
        .collect()
}

#[test]
#[serial]
fn a_chain_split_across_two_executor_units_keeps_the_queued_hop_between_them() {
    let config = split_graph();
    let infos = split_infos(&config);
    let topology = GraphTopology::build(&config, &infos).expect("the split graph's topology");
    let trigger_edges = build_trigger_edges(&config, &infos);
    let all = node_ids(&config);

    // The derivation the CLI plans units with, without a profile: `src` and
    // `mid` form one unit, and `far` starts a unit of its own for its
    // `isolate: true`.
    let graph = UnitGraph::from_graph(&config, &infos, &topology, &trigger_edges);
    let units = derive_units("host", &all, &graph, None);
    let members: Vec<Vec<String>> = units.iter().map(|u| u.nodes.clone()).collect();
    assert_eq!(
        members,
        vec![
            vec!["src".to_string(), "mid".to_string()],
            vec!["far".to_string()]
        ],
        "the units of the split graph"
    );
    assert_eq!(
        units[0].boundaries,
        vec![("src".to_string(), Boundary::NoProfile)],
        "src starts the unit of the nodes that do not declare `isolate: true`"
    );
    assert_eq!(
        units[1].boundaries,
        vec![("far".to_string(), Boundary::Isolated)],
        "far starts its unit for its `isolate: true`"
    );

    // In one level loop the census chains all three nodes.
    let levels = resolve_levels(&config, &topology, &trigger_edges).expect("levels");
    let census = census_chains(
        &config,
        &infos,
        &topology,
        &trigger_edges,
        &levels,
        Colocation::SingleProcess,
    );
    let chains: Vec<Vec<String>> = census.chains().iter().map(|c| c.nodes.clone()).collect();
    assert_eq!(chains, vec![all.clone()], "one chain in one level loop");

    // In far's own unit the edge from mid has no producer in the build, so it
    // is no chain link.
    let far_unit = unit_subgraph(&config, &members[1]);
    assert_eq!(
        far_unit.nodes[0].inputs[0].source,
        "/test/chain_unit_split/mid/out"
    );
    let far_infos = infos_of(&far_unit, &infos);
    let far_topology = GraphTopology::build(&far_unit, &far_infos).expect("far's topology");
    let far_triggers = build_trigger_edges(&far_unit, &far_infos);
    let far_levels = resolve_levels(&far_unit, &far_topology, &far_triggers).expect("far's levels");
    let far_census = census_chains(
        &far_unit,
        &far_infos,
        &far_topology,
        &far_triggers,
        &far_levels,
        Colocation::SingleProcess,
    );
    assert!(far_census.chains().is_empty(), "far's unit holds no chain");
    let far_edge = far_census
        .edges()
        .iter()
        .find(|e| e.consumer == "far")
        .expect("far's census judges its trigger edge");
    assert_eq!(far_edge.bar, Some(ChainBar::NoInGraphProducer));

    let queued = run_units(std::slice::from_ref(&all), false);
    let one_loop = run_units(std::slice::from_ref(&all), true);
    let split = run_units(&members, true);
    assert_eq!(queued.calls, vec![0], "the queued run makes no chain call");
    assert_eq!(
        one_loop.calls,
        vec![(STEPS * 2) as u64],
        "in one level loop mid and far are both called every step"
    );
    assert_eq!(
        split.calls,
        vec![STEPS as u64, 0],
        "split, the first unit calls mid every step and the second unit calls nothing"
    );

    // The queued hop between the units carries every frame mid published, in
    // order, as the queued run of the whole graph reads it.
    let reference = stream(&queued.frames, "far");
    assert!(
        reference.len() >= STEPS,
        "far read {} frames in {STEPS} steps",
        reference.len()
    );
    for tag in ["mid", "far"] {
        let (q, l, s) = (
            stream(&queued.frames, tag),
            stream(&one_loop.frames, tag),
            stream(&split.frames, tag),
        );
        if let Some(d) = first_difference(&q, &l) {
            panic!("{tag}, one level loop: {d}");
        }
        if let Some(d) = first_difference(&q, &s) {
            panic!("{tag}, split across units: {d}");
        }
    }
    // Every node fires at the same steps and times, with the same discard
    // bits, in the queued run, in one level loop and split across the units.
    for node in ["src", "mid", "far"] {
        let (q, l, s) = (
            fires_of(&queued.trace, node),
            fires_of(&one_loop.trace, node),
            fires_of(&split.trace, node),
        );
        assert!(
            q.len() >= STEPS,
            "{node} fired {} times in {STEPS} steps",
            q.len()
        );
        if let Some(d) = first_difference(&q, &l) {
            panic!("{node}'s fires, one level loop: {d}");
        }
        if let Some(d) = first_difference(&q, &s) {
            panic!("{node}'s fires, split across units: {d}");
        }
    }
}

fn infos_of(config: &GraphConfig, all: &IndexMap<String, NodeInfo>) -> IndexMap<String, NodeInfo> {
    node_ids(config)
        .into_iter()
        .map(|id| {
            let info = all[&id].clone();
            (id, info)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// What the build says about the chain calls it forms.
//
// `src -> one -> bare -> end`: the census chains all four. `one` is called.
// `bare` is a hand-written entry that declares no trigger policy, so the
// runtime fires it on any input arrival and gives it no data-trigger binding
// for a call to drain: the chain is cut there, and `bare` and `end` fire at
// their own levels. In `pace -> pick -> last` on the separate drain the chain
// is cut at its first member, so that build calls no member.
// ---------------------------------------------------------------------------

relay!(OneRelay, "one");
sink!(EndSink, "end");

const CUT_GRAPH: &str = r#"
prefix: test/chain_call_cut
nodes:
  - id: src
    type: count
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: one
    type: relay
    inputs:
      - name: inp
        source: src/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: bare
    type: bare
    inputs:
      - name: inp
        source: one/out
    outputs:
      - name: out
        schema: std_msgs/Int32
  - id: end
    type: sink
    inputs:
      - name: inp
        source: bare/out
"#;

fn cut_factories() -> IndexMap<String, Box<dyn NodeEntry>> {
    let mut f: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    f.insert("src".into(), Box::new(CountEntry::new()));
    f.insert("one".into(), Box::new(OneRelayEntry::new()));
    f.insert(
        "bare".into(),
        Box::new(ClosureNodeEntry::new(
            NodeInfo::from_names(vec!["inp".into()], vec!["out".into()]),
            |_ctx: &mut NodeContext| Ok(()),
        )),
    );
    f.insert("end".into(), Box::new(EndSinkEntry::new()));
    f
}

/// Build `yaml` with the seam at `seam` (`None` leaves it unset) and the
/// drain forced separate when `separate`, and return the census chains the
/// build judged.
fn build_with_seam(
    yaml: &str,
    factories: IndexMap<String, Box<dyn NodeEntry>>,
    seam: Option<&str>,
    separate: bool,
) -> Vec<Vec<String>> {
    let _drain = separate.then(|| {
        std::env::set_var("CERULION_DRAIN_DISCIPLINE", "separate");
        DrainEnv
    });
    let _env = match seam {
        Some(v) => Some(FuseEnv::set(v)),
        None => {
            std::env::remove_var(FUSE_CHAINS_ENV);
            None
        }
    };
    let config = parse_graph(yaml).expect("the graph must parse");
    validate_graph(&config).expect("the graph must validate");
    let infos: IndexMap<String, NodeInfo> = factories
        .iter()
        .map(|(id, e)| (id.clone(), e.info().expect("a macro node has its info")))
        .collect();
    let topology = GraphTopology::build(&config, &infos).expect("the topology");
    let trigger_edges = build_trigger_edges(&config, &infos);
    let levels = resolve_levels(&config, &topology, &trigger_edges).expect("levels");
    let census = census_chains(
        &config,
        &infos,
        &topology,
        &trigger_edges,
        &levels,
        Colocation::SingleProcess,
    );
    GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 8)
        .expect("the graph must build");
    census.chains().iter().map(|c| c.nodes.clone()).collect()
}

fn names(ids: &[&str]) -> Vec<String> {
    ids.iter().map(|s| s.to_string()).collect()
}

/// The lines of the captured log that carry `needle`.
fn lines_with<'a>(lines: &[&'a str], needle: &str) -> Vec<&'a str> {
    lines
        .iter()
        .copied()
        .filter(|l| l.contains(needle))
        .collect()
}

const CUT: &str = "chain fusion: a census chain is cut at this member";
const NONE_CALLED: &str = "chain fusion: this build calls no chain member";
const CALLED: &str = "chain fusion: the chain members this build calls";
const DECISION: &str = "chain fusion: the decision is resolved";
const CENSUS: &str = "chain fusion: the chains this graph's declarations allow";

/// One `warn` per cut, naming the chain head, the member and the reason, and
/// one line naming the members the build calls.
#[test]
#[serial]
#[traced_test]
fn the_build_names_the_member_a_chain_is_cut_at_and_the_members_it_calls() {
    let chains = build_with_seam(CUT_GRAPH, cut_factories(), Some("1"), false);
    assert_eq!(chains, vec![names(&["src", "one", "bare", "end"])]);
    logs_assert(|lines: &[&str]| {
        let cuts = lines_with(lines, CUT);
        let [cut] = cuts.as_slice() else {
            return Err(format!("one cut, one cut line: {cuts:#?}"));
        };
        for want in [
            "WARN",
            "chain_head=src",
            "member=bare",
            "reason=no data-trigger binding",
        ] {
            if !cut.contains(want) {
                return Err(format!("the cut line lacks {want:?}: {cut}"));
            }
        }
        let called = lines_with(lines, CALLED);
        let [line] = called.as_slice() else {
            return Err(format!("one line names the members called: {called:#?}"));
        };
        if !line.contains("members=1") || !line.contains(r#"called=["one"]"#) {
            return Err(format!("the called line names `one` alone: {line}"));
        }
        if !lines_with(lines, NONE_CALLED).is_empty() {
            return Err("a build that calls a member says it calls none".into());
        }
        let decision = lines_with(lines, DECISION);
        match decision.as_slice() {
            [d] if d.contains("every callable chain member is called") => Ok(()),
            _ => Err(format!(
                "the decision line says every callable member: {decision:#?}"
            )),
        }
    });
}

/// A chain cut at its first member: the build calls no member and says why.
#[test]
#[serial]
#[traced_test]
fn a_build_whose_every_chain_is_cut_at_its_first_member_says_it_calls_none() {
    let chains = build_with_seam(LATEST_GRAPH, latest_factories(), Some("1"), true);
    assert_eq!(chains, vec![names(&["pace", "pick", "last"])]);
    logs_assert(|lines: &[&str]| {
        let cuts = lines_with(lines, CUT);
        let [cut] = cuts.as_slice() else {
            return Err(format!("one cut, one cut line: {cuts:#?}"));
        };
        for want in [
            "WARN",
            "chain_head=pace",
            "member=pick",
            "reason=its trigger input drains on the separate discipline",
        ] {
            if !cut.contains(want) {
                return Err(format!("the cut line lacks {want:?}: {cut}"));
            }
        }
        let none = lines_with(lines, NONE_CALLED);
        match none.as_slice() {
            [line] if line.contains("reason=every census chain is cut at its first member") => {}
            _ => {
                return Err(format!(
                    "one line says no member is called, and why: {none:#?}"
                ))
            }
        }
        if lines_with(lines, CALLED).is_empty() {
            Ok(())
        } else {
            Err("a build that calls no member names members it calls".into())
        }
    });
}

/// With the seam off or unset the build forms no chain call and says nothing
/// about one; the census lines it always logs are there.
#[test]
#[serial]
#[traced_test]
fn with_the_seam_off_the_build_says_nothing_about_chain_calls() {
    let mut builds = 0;
    for seam in [Some("0"), None] {
        for separate in [false, true] {
            build_with_seam(CUT_GRAPH, cut_factories(), seam, separate);
            build_with_seam(LATEST_GRAPH, latest_factories(), seam, separate);
            builds += 2;
        }
    }
    logs_assert(|lines: &[&str]| {
        let census = lines_with(lines, CENSUS).len();
        if census != builds {
            return Err(format!("{census} census lines for {builds} builds"));
        }
        for phrase in [CUT, NONE_CALLED, CALLED] {
            let hits = lines_with(lines, phrase);
            if !hits.is_empty() {
                return Err(format!(
                    "the seam is off and the build logs {phrase:?}: {hits:#?}"
                ));
            }
        }
        Ok(())
    });
}

/// The mutants: each breaks one obligation of the chain call, and the
/// identity comparison above must see it. A mutant that slips through means
/// the oracle does not check what it claims to.
#[cfg(feature = "oracle-mutants")]
mod mutants {
    use super::*;
    use cerulion_core::graph::{
        CHAIN_CALL_INLINE_TRACE_MUTANT_ENV, CHAIN_CALL_SKIP_DEQUEUE_MUTANT_ENV,
    };

    struct MutantEnv(&'static str);

    impl Drop for MutantEnv {
        fn drop(&mut self) {
            std::env::remove_var(self.0);
        }
    }

    fn mutated_run(var: &'static str) -> (Run, Run) {
        let queued = run(false);
        std::env::set_var(var, "1");
        let _guard = MutantEnv(var);
        let fused = run(true);
        (queued, fused)
    }

    /// A chain call that writes its trace entry into the shared trace at call
    /// time puts it ahead of the non-member nodes of the member's level.
    #[test]
    #[serial]
    fn the_inline_trace_mutant_is_caught() {
        let (queued, fused) = mutated_run(CHAIN_CALL_INLINE_TRACE_MUTANT_ENV);
        assert!(fused.calls > 0, "the mutant run must make chain calls");
        let err =
            compare(&queued, &fused).expect_err("the oracle must catch an inline trace write");
        assert!(
            err.starts_with("fire trace:"),
            "caught at the wrong log: {err}"
        );
    }

    /// A chain call that skips the member's own dequeue fires on a frame it
    /// never took.
    #[test]
    #[serial]
    fn the_skipped_dequeue_mutant_is_caught() {
        let (queued, fused) = mutated_run(CHAIN_CALL_SKIP_DEQUEUE_MUTANT_ENV);
        assert!(fused.calls > 0, "the mutant run must make chain calls");
        compare(&queued, &fused).expect_err("the oracle must catch a skipped dequeue");
    }
}
