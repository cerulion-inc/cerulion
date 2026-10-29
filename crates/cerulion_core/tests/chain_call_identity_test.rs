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
//! ```bash
//! cargo test -p cerulion_core --test chain_call_identity_test -- --test-threads=1
//! ```

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::{parse_graph, validate_graph, GraphRuntime, FUSE_CHAINS_ENV};
use cerulion_core::prelude::*;
use cerulion_core::scheduler::{MergedReadOutcome, TraceEntry};
use indexmap::IndexMap;
use native_ros2_messages::std_msgs::Int32;
use serial_test::serial;

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
    FRAMES.lock().expect("not poisoned").clear();
    let mut runtime = build(fused);
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
