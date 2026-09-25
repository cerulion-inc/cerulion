// SPDX-License-Identifier: AGPL-3.0-only
//! Per-set Sync cdylib PARITY: `#[input(backpressure = block | sample(N))]` on a
//! PER-SET Sync TRIGGER input, through the REAL `DylibNodeEntry` FFI.
//!
//! # Why this file exists
//!
//! Both policies are REAL on a Sync trigger input and their semantics are
//! written down, but only in process. `sync_per_set_backpressure_iox2_test.rs` drives
//! `block` and `sample(N)` through the per-set matcher against hand oracles, and
//! every one of its nodes is an in-process `#[cerulion_node]`.
//!
//! The surface every `graph run` node actually deploys on is `DylibNodeEntry`,
//! and on that surface neither policy had a single driver: no fixture in the
//! repo declared `backpressure` on a Sync trigger input at all. The declaration
//! crosses the FFI as info JSON (`"backpressure":"block"` /
//! `{"sample":N}` plus `"depth"`), the host re-reads it into `InputMeta`, and
//! the runtime wires the gate from there. That is a hand-written round trip, and
//! a round trip nothing drives is a round trip nobody knows is wrong — the
//! no-inert-shipping rule. This file drives it.
//!
//! # What is pinned, and against what
//!
//! Each arm is an EXACT stimulus mirror of its in-process twin, so its oracle is
//! the SAME hand-derived oracle, re-asserted on the deployment path:
//!
//!   0. `block` stops its producer at EXACTLY the declared depth (mirrors B1,
//!      `a_block_producer_stops_at_depth_counting_the_member_the_matcher_holds`)
//!      — the one exact-valued oracle in the block half, and the only arm that
//!      pins the gate to the number the FFI carried rather than to "some
//!      throttling happened".
//!   1. `block` THROTTLES ITS PRODUCER and loses nothing (mirrors B2,
//!      `block_on_a_per_set_sync_trigger_loses_nothing`). Read
//!      [`assert_block_oracle`] before trusting the second half of that
//!      sentence: MEASURED, only the throttle discriminates on this stimulus.
//!   2. `sample(N)` decimates BEFORE matching, and that CHANGES WHICH FRAMES
//!      ALIGN (mirrors S1,
//!      `a_sample_gate_decides_which_frames_are_eligible_to_align`). The oracle
//!      is exact MEMBERSHIP — `[(0,0), (5,4), (10,10)]` — which no counter
//!      assertion can see, backed by an UNGATED discriminator run.
//!   3. PARITY, both policies (the in-process vs dylib divergence class): an in-process twin
//!      with the identical declaration under the identical stimulus observes the
//!      identical sequence and the identical counters, and BOTH sides are
//!      asserted against the hand oracle. For `sample(N)` that oracle is exact
//!      membership, so a regression common to both surfaces cannot pass; for
//!      `block` it is a family of BOUNDS, so parity is necessary but not
//!      sufficient there — the exact-depth arm is what makes it sufficient.
//!   4. DETERMINISM (Principle #7): two full dylib runs are byte-identical.
//!   5. The DECLARATION half: `backpressure` + `depth` + `trigger` survive the
//!      info-JSON FFI into `InputMeta`. A behavioural arm says the gate did not
//!      act; this one says whether the declaration ever arrived.
//!
//! # Reading a cdylib's behaviour
//!
//! A cdylib's Rust state is opaque across the FFI, so the in-process suites'
//! `with_state`-injected sink is unavailable. Each fixture's tick publishes what
//! it OBSERVED into its `Vector3` output — `fused.x` = the `a` member, `fused.y`
//! = the `b` member, `fused.z` = its own `#[on_event]` handler's running count —
//! and a host closure sink reads it back off SHM. The values crossing SHM ARE
//! the proof (Principle #13).
//!
//! The sink declares its input through `NodeInfo::with_meta` rather than
//! `from_names` for TWO reasons, neither of which is the one-frame-per-fire
//! read: `unifies_data_trigger` (`graph/node.rs`) requires an `input_meta` entry
//! carrying `DropOldest`, so a names-only sink binds the legacy Separate way,
//! and the meta is also what sizes the sink's own depth at 16. The FIFO read is
//! NOT downstream of that choice — `mark_fifo_consume` is called from the
//! `MacroPolicy::DataTrigger` arm in `graph/runtime.rs`, keyed on the POLICY and
//! applied on BOTH drain disciplines, so a `from_names` sink with the same
//! policy would read the same per-message sequence. (Drain-to-latest is the
//! latest-value CONTEXT path, which a data-trigger input never takes.)
//!
//! `fused.z` LAGS by one fire — the macro dispatches queued events after the
//! body on the tick-Ok path — so the arms take the MAX, which is monotone-safe,
//! and assert it as a LIVENESS floor (`>= 1`) rather than an exact count.
//!
//! # Required fixture builds
//!
//! ```bash
//! cargo build -p test_node_macro_sync_block_cdylib \
//!             -p test_node_macro_sync_sample_cdylib
//! cargo test -p cerulion_core --test cdylib_sync_backpressure_parity_test \
//!            -- --test-threads=1
//! ```
//!
//! `#[serial]` for ONE reason: the cdylib `NODES` registry is process-global.
//! It is NOT an iceoryx2 constraint — `build_for_test` mints an isolated SHM
//! namespace per call, which is why this binary touches no DEFAULT-namespace
//! service and is deliberately absent from the `.config/nextest.toml` fence
//! (`serial_discipline_test` excludes `init_for_test` from its singleton doors
//! for exactly that reason). Same classification as its sibling
//! `cdylib_sync_nontrigger_test.rs`. Under nextest each test is its own process,
//! so the attribute is inert there and matters only to `cargo test`.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{
    BackpressurePolicy, ClosureNodeEntry, DylibNodeEntry, InputMeta, MacroPolicy, NodeEntry,
    NodeInfo,
};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::message::ShmMessage;
use cerulion_core::prelude::*;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

const MS_NS: u64 = 1_000_000;

/// B2's stimulus length. Also the number of frames a `period_ms = 1` producer
/// publishes when NOTHING holds it back — which is what makes the throttle
/// assertion in [`assert_block_oracle`] exact rather than a vague "fewer".
const BLOCK_STEPS: u64 = 60;

/// The depth `test_node_macro_sync_block_cdylib` (and its in-process twin)
/// DECLARE on input `a`. Every use of the number goes through this const —
/// the starved-producer oracle, the depth box, and the `InputMeta` carry — so
/// the test cannot drift from the fixture by editing one and not the others.
const BLOCK_DEPTH: u64 = 4;

/// Mirror of the block fixture's own `LOSSY_BLOCK_SENTINEL`.
///
/// `block` is lossless, so a `Block` backpressure event that reports a DROP is
/// the contract broken. The fixture encodes that in the counter's VALUE rather
/// than ignoring it, because ignoring it maps a broken contract onto the same
/// `0` as "the gate never engaged" — opposite diagnoses that would otherwise
/// fail with the same, wrong message.
const LOSSY_BLOCK_SENTINEL: u64 = 1_000_000;

/// The absolute external topics the SAMPLE arms publish onto by hand. `block`
/// cannot use this shape at all — `GraphTopology::validate` requires an
/// in-graph producer for a block input — which is half of why the two policies
/// need separate fixtures.
const TOPIC_A: &str = "/csbp/a";
const TOPIC_B: &str = "/csbp/b";

/// Every `(a, b, events)` triple a fire published, in fire order.
type Fires = Arc<Mutex<Vec<(u64, u64, u64)>>>;

fn recorded(fires: &Fires) -> Vec<(u64, u64, u64)> {
    fires.lock().expect("fires sink poisoned").clone()
}

/// The `(a, b)` pairs alone — the SET SEQUENCE, with the lagging event counter
/// projected out so an oracle can name exactly which frames were served.
fn sets(fires: &[(u64, u64, u64)]) -> Vec<(u64, u64)> {
    fires.iter().map(|(a, b, _)| (*a, *b)).collect()
}

/// The highest `fused.z` any fire published — the node's own `#[on_event]`
/// handler count, read back across SHM.
fn max_events(fires: &[(u64, u64, u64)]) -> u64 {
    fires.iter().map(|(_, _, e)| *e).max().unwrap_or(0)
}

fn load(stem: &str) -> Box<dyn NodeEntry> {
    Box::new(
        DylibNodeEntry::load(&cerulion_core::testing::find_fixture_cdylib(stem)).unwrap_or_else(
            |e| {
                panic!(
                    "load {stem}: {e}. Build it first: cargo build -p {stem} (under the SAME \
                     profile as this test)"
                )
            },
        ),
    )
}

// ===========================================================================
// The in-process TWINS — identical declarations to the two fixtures.
//
// Each is asserted against the hand oracle in its own right, so the parity
// arms are a three-way agreement (dylib == twin == oracle) rather than a
// self-compare between two things that could be wrong together.
// ===========================================================================

/// Twin of `test_node_macro_sync_block_cdylib`.
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct InProcessSyncBlockTwin {
    #[input(trigger, backpressure = block, depth = 4)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    #[output]
    fused: Vector3,
    block_events: u32,
}

#[cerulion_node_impl]
impl InProcessSyncBlockTwin {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.fused.x = self.a.x;
        self.fused.y = self.b.x;
        self.fused.z = f64::from(self.block_events);
        Ok(())
    }

    /// Byte-for-byte the block fixture's handler, sentinel included — the
    /// parity arms compare BEHAVIOUR, so a twin that treated a lossy Block
    /// event differently would make them compare two different nodes.
    #[on_event(input = "a")]
    fn on_a_pressure(&mut self, event: BackpressureEvent) {
        if matches!(event.policy, BackpressurePolicy::Block) {
            self.block_events += if event.dropped == 0 {
                1
            } else {
                LOSSY_BLOCK_SENTINEL as u32
            };
        }
    }
}

/// Twin of `test_node_macro_sync_sample_cdylib`.
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct InProcessSyncSampleTwin {
    #[input(trigger, backpressure = sample(5), depth = 16)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    #[output]
    fused: Vector3,
    sample_events: u32,
}

#[cerulion_node_impl]
impl InProcessSyncSampleTwin {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.fused.x = self.a.x;
        self.fused.y = self.b.x;
        self.fused.z = f64::from(self.sample_events);
        Ok(())
    }

    #[on_event(input = "a")]
    fn on_a_pressure(&mut self, event: BackpressureEvent) {
        if matches!(event.policy, BackpressurePolicy::Sample(5)) {
            self.sample_events += 1;
        }
    }
}

/// The SAMPLE arm's DISCRIMINATOR: the same node with NO gate on `a`. Nothing
/// else differs, so a membership difference between it and the cdylib is the
/// gate's doing and only the gate's.
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct UngatedTwin {
    #[input(trigger, depth = 16)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    #[output]
    fused: Vector3,
}

#[cerulion_node_impl]
impl UngatedTwin {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.fused.x = self.a.x;
        self.fused.y = self.b.x;
        self.fused.z = 0.0;
        Ok(())
    }
}

// ===========================================================================
// The BLOCK stimulus: two in-graph feeders, mirroring B2's `block_pair_graph`.
// ===========================================================================

/// The fast in-graph producer of the block topic. One frame per fire, value =
/// fire ordinal, so the consumer's members name exactly which frames it saw.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct BlockFeeder {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl BlockFeeder {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = f64::from(self.n);
        Ok(())
    }
}

/// The slow partner, on its own `drop_oldest` topic.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct SlowFeeder {
    #[output]
    out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl SlowFeeder {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = f64::from(self.n);
        Ok(())
    }
}

fn node(id: &str, ty: &str, inputs: Vec<(&str, &str)>, outputs: Vec<&str>) -> NodeDef {
    NodeDef {
        fuse: None,
        ros2: None,
        id: id.to_string(),
        node_type: ty.to_string(),
        inputs: inputs
            .into_iter()
            .map(|(name, source)| InputDef {
                name: name.to_string(),
                source: source.to_string(),
            })
            .collect(),
        outputs: outputs
            .into_iter()
            .map(|name| OutputDef {
                name: name.to_string(),
                schema: "Vector3".to_string(),
                max_slice_len: None,
                history_size: 0,
                topic: None,
            })
            .collect(),
    }
}

fn graph(prefix: &str, nodes: Vec<NodeDef>) -> GraphConfig {
    GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cdylib_sync_backpressure".to_string(),
        prefix: prefix.to_string(),
        nodes,
    }
}

/// The host-side reader of a fixture's output. Declared through
/// `NodeInfo::with_meta` so the binding is Unified per-message FIFO — one fire
/// reads one frame, which is what makes the sequence oracles meaningful.
fn recording_sink(fires: Fires) -> ClosureNodeEntry {
    let meta = InputMeta {
        name: "in".to_string(),
        schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
        trigger: true,
        depth: 16,
        backpressure: BackpressurePolicy::DropOldest,
        expect_within_ms: None,
    };
    ClosureNodeEntry::new(
        NodeInfo::with_meta(vec![meta], vec![]).with_policy(MacroPolicy::DataTrigger {
            input_name: "in".to_string(),
        }),
        move |ctx| {
            // Neither a missing subscriber (a wiring desync) nor a `try_view`
            // Err may be swallowed: both would surface as FEWER recorded sets,
            // and every oracle here would then blame the stimulus for a read
            // failure. Only a genuinely EMPTY read is silent.
            let sub = ctx
                .subscriber_mut("in")
                .expect("sink input `in` has no subscriber — inputs/binding desync");
            let read = sub
                .try_view::<Vector3, _>(|view| {
                    // `as u64` saturates a NaN or a negative to 0, and (0, 0) is
                    // the sample oracle's own first element — so a garbage read
                    // would be indistinguishable from a correct one.
                    assert!(
                        view.x.is_finite() && view.y.is_finite() && view.z.is_finite(),
                        "sink read a non-finite member: ({}, {}, {})",
                        view.x,
                        view.y,
                        view.z
                    );
                    (view.x as u64, view.y as u64, view.z as u64)
                })
                .expect("sink try_view failed on `in`");
            if let Some(v) = read {
                fires.lock().expect("fires sink poisoned").push(v);
            }
            Ok(())
        },
    )
    .with_label("cdylib_sync_backpressure_sink")
}

/// What one block-stimulus run observed.
#[derive(Debug, PartialEq, Eq)]
struct BlockRun {
    sets: Vec<(u64, u64)>,
    max_events: u64,
    skips: u64,
    unmatched: u64,
    evictions: u64,
    deferred: u64,
    published: u64,
    /// The fuse's OWN fire count. The in-process reference records members from
    /// inside the node, so its `sets.len()` IS the number of `a` frames served;
    /// here they are the frames the SINK read back, which is one edge further
    /// downstream. Carrying both lets the oracle assert they agree BEFORE the
    /// loss block, so a sink-side loss fails as a sink-side loss instead of
    /// being folded into `accounted` and reported against the producer edge.
    fires: u64,
}

/// Build the block graph. `partner` is `fuse.b`'s source: `"slow/out"` for the
/// B2 pair stimulus, or an ABSOLUTE topic nobody publishes for the B1 starved
/// stimulus (`fuse` can then never fire and never consumes a member).
fn block_graph(
    prefix: &str,
    partner: &str,
    fuse: Box<dyn NodeEntry>,
    fires: Fires,
) -> GraphRuntime {
    let config = graph(
        prefix,
        vec![
            node("fast", "block_feeder", vec![], vec!["out"]),
            node("slow", "slow_feeder", vec![], vec!["out"]),
            node(
                "fuse",
                "sync_block_pair_node",
                vec![("a", "fast/out"), ("b", partner)],
                vec!["fused"],
            ),
            node("sink", "recording_sink", vec![("in", "fuse/fused")], vec![]),
        ],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fast".to_string(), Box::new(BlockFeederEntry::new()));
    factories.insert("slow".to_string(), Box::new(SlowFeederEntry::new()));
    factories.insert("fuse".to_string(), fuse);
    factories.insert("sink".to_string(), Box::new(recording_sink(fires)));

    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8).expect("build block sync graph")
}

/// `fast` (1 ms) → `fuse.a` (block, depth 4); `slow` (5 ms) → `fuse.b`;
/// `fuse.fused` → a recording sink. 60 steps of 1 ms — B2's stimulus exactly.
fn run_block(prefix: &str, fuse: Box<dyn NodeEntry>) -> BlockRun {
    let fires: Fires = Arc::new(Mutex::new(Vec::new()));
    let mut runtime = block_graph(prefix, "slow/out", fuse, Arc::clone(&fires));
    for _ in 0..BLOCK_STEPS {
        runtime.step(Duration::from_millis(1));
    }

    let handle = runtime.node_handle("fuse").expect("fuse handle");
    // The sync counters are keyed by RESOLVED TOPIC (leading slash — see
    // `resolve_source`) while the backpressure ones are keyed by INPUT NAME.
    // Get either wrong and the counter silently reads 0.
    let topic_a = format!("/{prefix}/fast/out");
    let observed = recorded(&fires);
    BlockRun {
        sets: sets(&observed),
        max_events: max_events(&observed),
        skips: handle.sync_closer_skip_count(&topic_a),
        unmatched: handle.sync_unmatched_discard_count(&topic_a),
        evictions: handle.backpressure_drop_oldest_count("a"),
        deferred: handle.backpressure_block_fires_deferred_count("a"),
        published: runtime
            .node_handle("fast")
            .expect("fast handle")
            .fire_count(),
        fires: handle.fire_count(),
    }
}

/// The HAND oracle for [`run_block`], asserted against whichever side ran it.
///
/// # What actually discriminates `block` here, and what does not
///
/// `block`'s declared contract has two halves — the producer is DEFERRED, and
/// nothing is LOST — and only the first half is discriminating on this
/// stimulus. That is worth stating plainly rather than letting a passing
/// assertion imply more than it proves:
///
/// * THE THROTTLE (discriminating). A `period_ms = 1` producer over
///   [`BLOCK_STEPS`] 1 ms steps publishes exactly one frame per step unless
///   something holds it back. `block` holds it back the moment `depth` frames
///   are unserved, so a real block edge publishes STRICTLY FEWER. This is the
///   mechanism: with the declaration stripped, the producer free-runs to
///   exactly `BLOCK_STEPS`.
/// * NO LOSS (necessary, not sufficient). `a`'s frames carry their publish
///   ordinal and leave the queue in FIFO order, so each is either SERVED to a
///   set or PASSED OVER by the descent, and the highest ordinal observed cannot
///   exceed `served + skipped`. That invariant must hold — but MEASURED, it
///   also holds under `drop_oldest` on this stimulus, because the per-set
///   descent drains the whole backlog at every boundary and the depth-4 queue
///   never actually overflows. So it is a genuine invariant that this stimulus
///   cannot attribute to `block`; the throttle assertions are what attribute.
///
/// Asserted throttle-first so a failure names the mechanism rather than the
/// symptom.
fn assert_block_oracle(run: &BlockRun, label: &str) {
    assert!(
        run.sets.len() >= 3,
        "{label}: the partner ticks every 5 ms over {BLOCK_STEPS} steps, so \
         several sets must fire — got {:?}",
        run.sets
    );

    // --- THE THROTTLE: the half that attributes this run to `block`. ---
    assert!(
        run.published < BLOCK_STEPS,
        "{label}: the producer was NOT held back. `fast` is `period_ms = 1` \
         driven for {BLOCK_STEPS} 1 ms steps, so an unthrottled producer \
         publishes exactly {BLOCK_STEPS} frames — and it published {}. `block` \
         defers the producer the moment `depth` = {BLOCK_DEPTH} frames are \
         unserved, so a \
         real block edge publishes strictly fewer. A free-running producer here \
         means the edge is not `block`: either the declaration never reached the \
         host (the info-JSON round trip) or the gate was never wired from it",
        run.published
    );
    assert!(
        run.deferred > 0,
        "{label}: and the scheduler's own counter agrees — the pre-fire block \
         gate really did defer `fast` at least once"
    );
    assert!(
        run.max_events >= 1,
        "{label}: the node's OWN `#[on_event]` handler saw a Block event, so the \
         defers above are THIS consumer's flow control rather than an unrelated \
         stall. The handler guards on `BackpressurePolicy::Block`, so a degraded \
         `DropOldest` edge can never move it. For the cdylib the count crossed \
         SHM out of the fixture, which is what makes it proof the handler ran \
         INSIDE the loaded library"
    );
    assert!(
        run.max_events < LOSSY_BLOCK_SENTINEL,
        "{label}: a Block event REPORTED A DROP ({} carries the sentinel). \
         `block` is lossless by definition, so this is the contract broken at \
         the event source — a distinct failure from the gate never engaging, \
         which is why the fixture encodes it in the value rather than ignoring it",
        run.max_events
    );

    // --- NO LOSS: invariants that must hold (see the note above on scope). ---
    assert_eq!(
        run.sets.len() as u64,
        run.fires,
        "{label}: the sink read back one frame per fuse fire. The loss oracles \
         below treat `sets.len()` as the number of `a` frames SERVED, which is \
         only true if nothing was lost on the fuse→sink edge — so this is \
         checked FIRST, or a sink-side loss would be folded into `accounted` and \
         reported against the producer edge instead"
    );
    let a_values: Vec<u64> = run.sets.iter().map(|(a, _)| *a).collect();
    assert!(
        a_values.windows(2).all(|w| w[0] < w[1]),
        "{label}: each frame is consumed by at most one set, in arrival order, \
         so the members are strictly increasing — got {a_values:?}"
    );
    let accounted = run.sets.len() as u64 + run.skips;
    let highest = a_values.last().copied().unwrap_or(0);
    assert!(
        highest <= accounted,
        "{label}: LOSSLESS — every `a` frame the node consumed was either served \
         to a set ({} of them) or passed over by the descent ({}), so the \
         highest ordinal observed ({highest}) cannot exceed {accounted}. A \
         larger value means frames vanished between the producer and the matcher",
        run.sets.len(),
        run.skips
    );
    assert!(
        accounted + BLOCK_DEPTH >= run.published,
        "{label}: the DEPTH BOX — at most `depth` = {BLOCK_DEPTH} frames can be \
         unconsumed at any instant, so the producer's {} publishes cannot run \
         more than {BLOCK_DEPTH} ahead of the {accounted} frames the node \
         accounted for",
        run.published
    );
    assert_eq!(run.evictions, 0, "{label}: block never evicts");
    assert_eq!(
        run.unmatched, 0,
        "{label}: every frame here has a partner within the 50 ms window, so \
         nothing is UNMATCHABLE — under `block` an under-provisioned depth costs \
         the PRODUCER's rate, never the slow frame's life"
    );
    assert!(
        run.published >= highest,
        "{label}: sanity — the producer published at least every frame the node \
         observed"
    );
}

// ===========================================================================
// The SAMPLE stimulus: hand-stamped publishes onto absolute topics (S1).
// ===========================================================================

/// What one S1-stimulus run observed.
#[derive(Debug, PartialEq, Eq)]
struct SampleRun {
    sets: Vec<(u64, u64)>,
    max_events: u64,
    sampled: u64,
    skips: u64,
    unmatched: u64,
}

/// Publish `x` onto `publisher` stamped at `at_ns` on the shared virtual clock.
/// The payload carries the STAMP, so a recorded set is its own oracle.
fn publish_at(publisher: &mut CerulionPublisher, clock: &VirtualClock, at_ns: u64, x: f64) {
    clock.set(at_ns);
    let mut p = publisher.loan_proxy::<Vector3>().expect("loan");
    p.x = x;
}

/// The S1 stimulus, run against whichever fixture is handed in.
///
/// `a` = frames stamped 0..=10 ms (value = stamp); `b` = frames stamped 0, 4 and
/// 10. Everything is queued before the first step, and the steps then drain it.
fn run_sample(prefix: &str, fuse: Box<dyn NodeEntry>) -> SampleRun {
    let fires: Fires = Arc::new(Mutex::new(Vec::new()));
    let config = graph(
        prefix,
        vec![
            node(
                "fuse",
                "sync_sample_pair_node",
                vec![("a", TOPIC_A), ("b", TOPIC_B)],
                vec!["fused"],
            ),
            node("sink", "recording_sink", vec![("in", "fuse/fused")], vec![]),
        ],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), fuse);
    factories.insert(
        "sink".to_string(),
        Box::new(recording_sink(Arc::clone(&fires))),
    );

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 32)
        .expect("build sampled sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on b");
    for ts in 0..=10u64 {
        publish_at(&mut pub_a, &clock, ts * MS_NS, ts as f64);
    }
    for ts in [0u64, 4, 10] {
        publish_at(&mut pub_b, &clock, ts * MS_NS, ts as f64);
    }
    clock.set(0);
    // One step per boundary; a gated input admits at most one `a` frame per
    // boundary, so the backlog drains over several.
    for _ in 0..14 {
        runtime.step(Duration::from_millis(1));
    }

    let handle = runtime.node_handle("fuse").expect("fuse handle");
    let observed = recorded(&fires);
    SampleRun {
        sets: sets(&observed),
        max_events: max_events(&observed),
        sampled: handle.backpressure_sampled_count("a"),
        skips: handle.sync_closer_skip_count(TOPIC_A),
        unmatched: handle.sync_unmatched_discard_count(TOPIC_A),
    }
}

/// The HAND oracle for [`run_sample`], walked by hand from the design.
///
/// The gate admits `a`@0 (a first frame is always admitted), then refuses 1..4
/// and admits `a`@5 (5 - 0 >= 5), then refuses 6..9 and admits `a`@10. The
/// matcher sees only those three, so the sets are (0,0), (5,4), (10,10).
///
/// UNGATED, same stimulus: the matcher sees every `a` frame and pairs (0,0),
/// then (1,4) — `a`@1 is simply the next frame in the queue — then descends to
/// (10,10) once `b` is scarce. The middle set is the discriminator, `(5,4)`
/// versus `(1,4)`, and no counter assertion can see it.
///
/// `(5,4)` is also the decision working as written: `a`@4 arrived, was in window,
/// and would have given a tighter span (0) than `a`@5 — an unconstrained matcher
/// would have preferred it, and the gate made it ineligible. Only frames the
/// read-gate ACCEPTS can join a set, which is what "decimates before matching"
/// means.
fn assert_sample_oracle(run: &SampleRun, label: &str) {
    assert_eq!(
        run.sets,
        vec![(0, 0), (5, 4), (10, 10)],
        "{label}: only ADMITTED frames are eligible to align — the gate refuses \
         a@1..4, so the set that pairs with b@4 reads a@5, even though a@4 \
         arrived, was in window, and would have given a tighter span"
    );
    assert_eq!(
        run.sampled, 8,
        "{label}: eight of a's eleven frames were decimated (1,2,3,4 and \
         6,7,8,9); the gate runs on EVERY per-set pop, so each is counted once"
    );
    assert_eq!(
        run.skips, 0,
        "{label}: no frame was PASSED OVER — the descent only ever ran at the \
         last set, where nothing was queued behind a@10. A gate drop is not a \
         skip; the two accountings are disjoint"
    );
    assert_eq!(
        run.unmatched, 0,
        "{label}: every admitted frame found a partner inside the window"
    );
    assert!(
        run.max_events >= 1,
        "{label}: the decimation regime reached the node's OWN `#[on_event]` \
         handler. For the cdylib this counter crossed SHM out of the fixture, so \
         it is the proof the handler ran INSIDE the loaded library"
    );
}

// ===========================================================================
// 0. THE EXACT-DEPTH ORACLE (mirrors B1) — the block half's only exact number.
// ===========================================================================

/// Run the B1 starved-partner stimulus and return `fast`'s publish count.
///
/// `fuse.b` reads an absolute topic nobody publishes, so the node can NEVER fire
/// and never consumes a member: the head fills on the first boundary and sits
/// there forever, and the producer stops as soon as the edge is full.
fn run_starved_block(prefix: &str, fuse: Box<dyn NodeEntry>) -> u64 {
    let fires: Fires = Arc::new(Mutex::new(Vec::new()));
    let partner = format!("/{prefix}/silent");
    let mut runtime = block_graph(prefix, &partner, fuse, Arc::clone(&fires));
    for _ in 0..20 {
        runtime.step(Duration::from_millis(1));
    }
    assert!(
        recorded(&fires).is_empty(),
        "{prefix}: the partner never publishes, so the node can never fire — \
         any recorded set means the stimulus is not the one this oracle assumes"
    );
    runtime
        .node_handle("fast")
        .expect("fast handle")
        .fire_count()
}

/// B1 — the producer stops at EXACTLY the declared depth, HELD MEMBER INCLUDED.
///
/// This is the arm that ties the gate to the NUMBER the FFI carried, and the
/// block half needs it because every other block assertion is one-sided.
/// [`assert_block_oracle`] would pass just as happily if the host wired the gate
/// from the wrong depth: an effective depth of 1 still throttles, still defers,
/// still mints Block events, and still satisfies every bound — so "some
/// throttling happened" is all those arms can prove on their own.
///
/// HAND ORACLE: exactly [`BLOCK_DEPTH`] publishes. `block`'s contract is "at
/// most `depth` UNSERVED frames per edge"; here `depth - 1` sit in the iceoryx2
/// queue and the last is the head the matcher holds — unserved by every reading
/// of the word, since no tick has run. So the producer must stop at 4 and stay
/// there for the remaining steps.
///
/// Under a pure pop-time decrement the boundary pop looks like a delivery, one
/// slot frees, and the producer publishes a FIFTH frame — `depth + 1` unserved
/// against a declared depth of 4. Under a stripped declaration it free-runs to
/// 20. Both are caught by one equality.
#[test]
#[serial]
fn a_block_producer_stops_at_the_declared_depth_through_the_ffi() {
    let dylib = run_starved_block("csbb0d", load("test_node_macro_sync_block_cdylib"));
    assert_eq!(
        dylib, BLOCK_DEPTH,
        "the cdylib's block edge must hold the producer at EXACTLY its declared \
         depth of {BLOCK_DEPTH} — {BLOCK_DEPTH} - 1 queued plus the member the \
         matcher is holding. A larger value means the gate is sized from \
         something other than the declared depth (or from nothing at all); a \
         smaller one means it is sized from less than the node asked for"
    );

    // PARITY on the exact number, not just on the bounds.
    let twin = run_starved_block("csbb0t", Box::new(InProcessSyncBlockTwinEntry::new()));
    assert_eq!(
        twin, BLOCK_DEPTH,
        "the in-process twin must hold at the same declared depth"
    );
}

// ===========================================================================
// 1. `block` on a per-set Sync trigger is LOSSLESS — through the real FFI.
// ===========================================================================
#[test]
#[serial]
fn block_on_a_per_set_sync_trigger_loses_nothing_through_the_ffi() {
    let run = run_block("csbb1", load("test_node_macro_sync_block_cdylib"));
    assert_block_oracle(&run, "cdylib");
}

// ===========================================================================
// 2. `sample(N)` decimates BEFORE matching — through the real FFI, with an
//    UNGATED discriminator so the membership oracle is attributed to the gate.
// ===========================================================================
#[test]
#[serial]
fn a_sample_gate_decides_which_frames_are_eligible_to_align_through_the_ffi() {
    let gated = run_sample("csbs1", load("test_node_macro_sync_sample_cdylib"));
    assert_sample_oracle(&gated, "cdylib");

    // THE DISCRIMINATOR — the identical stimulus with the gate removed. If the
    // FFI dropped the declaration on the floor, the cdylib WOULD be this run,
    // and every assertion above would be describing the matcher rather than the
    // gate. Its oracle is EXACT, not merely "different": two sides that both
    // regressed to different-but-wrong sequences would satisfy an `assert_ne!`.
    let ungated = run_sample("csbs1u", Box::new(UngatedTwinEntry::new()));
    assert_eq!(
        ungated.sets,
        vec![(0, 0), (1, 4), (10, 10)],
        "UNGATED, same stimulus: the matcher sees every `a` frame, pairs (0,0), \
         then takes `a`@1 as simply the next frame in the queue, then descends \
         to (10,10) once `b` is scarce"
    );
    assert_ne!(
        ungated.sets, gated.sets,
        "and the middle set is the discriminator: (5,4) gated vs (1,4) ungated"
    );
    assert_eq!(
        ungated.sampled, 0,
        "and the control really is ungated: nothing was decimated"
    );
}

// ===========================================================================
// 3. PARITY (the in-process vs dylib divergence class), one arm per policy: the identical
//    declaration must behave identically in-process and through the FFI, and
//    BOTH sides must equal the hand oracle.
// ===========================================================================
#[test]
#[serial]
fn the_block_contract_is_identical_in_process_and_through_the_ffi() {
    let dylib = run_block("csbb3d", load("test_node_macro_sync_block_cdylib"));
    let twin = run_block("csbb3t", Box::new(InProcessSyncBlockTwinEntry::new()));
    assert_block_oracle(&dylib, "cdylib");
    assert_block_oracle(&twin, "in-process twin");
    assert_eq!(
        dylib, twin,
        "the identical `block, depth = 4` declaration on a per-set Sync trigger \
         must produce the identical set sequence AND the identical accounting \
         on both surfaces — the whole state the info-JSON round trip is \
         supposed to carry"
    );
}

#[test]
#[serial]
fn the_sample_contract_is_identical_in_process_and_through_the_ffi() {
    let dylib = run_sample("csbs3d", load("test_node_macro_sync_sample_cdylib"));
    let twin = run_sample("csbs3t", Box::new(InProcessSyncSampleTwinEntry::new()));
    assert_sample_oracle(&dylib, "cdylib");
    assert_sample_oracle(&twin, "in-process twin");
    assert_eq!(
        dylib, twin,
        "the identical `sample(5), depth = 16` declaration must decide the same \
         MEMBERSHIP on both surfaces"
    );
}

// ===========================================================================
// 4. DETERMINISM (Principle #7): two full dylib runs are byte-identical, and
//    both equal the oracle — so this is not a self-compare.
// ===========================================================================
#[test]
#[serial]
fn two_dylib_runs_are_byte_identical() {
    let block_a = run_block("csbb4a", load("test_node_macro_sync_block_cdylib"));
    let block_b = run_block("csbb4b", load("test_node_macro_sync_block_cdylib"));
    assert_eq!(
        block_a, block_b,
        "two dylib runs of the block stimulus must agree on the set sequence AND \
         all five counters — the slot-debt credit is a shared-atomic write on \
         the READ path, so a run that depended on interleaving shows up here"
    );
    assert_block_oracle(&block_a, "cdylib run 1");

    let sample_a = run_sample("csbs4a", load("test_node_macro_sync_sample_cdylib"));
    let sample_b = run_sample("csbs4b", load("test_node_macro_sync_sample_cdylib"));
    assert_eq!(sample_a, sample_b, "and the same for the gated stimulus");
    assert_sample_oracle(&sample_a, "cdylib run 1");
}

// ===========================================================================
// 5. The DECLARATION half: `backpressure` + `depth` + `trigger` survive the
//    info-JSON FFI into `InputMeta`. This is what the runtime wires the gate
//    from, so it is the assertion that says WHY a behavioural arm failed.
// ===========================================================================
#[test]
#[serial]
fn the_declared_backpressure_and_depth_cross_the_info_json_ffi() {
    for (stem, expected, depth) in [
        (
            "test_node_macro_sync_block_cdylib",
            BackpressurePolicy::Block,
            BLOCK_DEPTH as usize,
        ),
        (
            "test_node_macro_sync_sample_cdylib",
            BackpressurePolicy::Sample(5),
            16usize,
        ),
    ] {
        let entry = load(stem);
        let info = entry.info().expect("fixture info parses");
        assert_eq!(
            info.policy(),
            Some(MacroPolicy::Sync { window_ms: 50 }),
            "{stem}: the bounded-sync policy must round-trip the FFI"
        );
        let meta = info.input_meta();
        assert_eq!(meta.len(), 2, "{stem}: two declared inputs");
        assert_eq!(meta[0].name, "a");
        assert!(
            meta[0].trigger,
            "{stem}: `a` is `#[input(trigger)]` — a Sync node aligns only its \
             trigger-marked inputs"
        );
        assert_eq!(
            meta[0].backpressure, expected,
            "{stem}: the DECLARED policy must cross the info-JSON FFI (ABI item \
             A2). A dropped declaration silently degrades the input to the \
             host-side `DropOldest` default — for `block` that is a producer \
             free-running past the depth it was promised, for `sample(N)` a \
             matcher aligning frames the gate should have refused. Neither is a \
             load error; both are silent"
        );
        assert_eq!(
            meta[0].depth, depth,
            "{stem}: and its declared depth — the number the block defer gate \
             and the gated queue are both sized from"
        );
        assert_eq!(meta[1].name, "b");
        assert!(meta[1].trigger, "{stem}: `b` is a trigger too");
        assert_eq!(
            meta[1].backpressure,
            BackpressurePolicy::DropOldest,
            "{stem}: the partner is deliberately on the DEFAULT policy, so these \
             arms exercise the per-INPUT rule rather than a whole-node one"
        );
    }
}
