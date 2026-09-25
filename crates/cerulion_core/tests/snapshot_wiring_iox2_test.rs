// SPDX-License-Identifier: AGPL-3.0-only
//! Fire-gated step-boundary snapshot of a firing node's
//! NON-trigger (latest-value) inputs, over real iceoryx2.
//!
//! The runtime FREEZES a firing node's non-trigger inputs at the DAG-level
//! boundary — after deciding the level's fire-set and draining its trigger
//! inputs, BEFORE running the firing nodes' ticks. So a same-level producer's
//! same-step publish is NOT observed by a same-level latest-value reader; the
//! reader sees the STEP-BOUNDARY (prior) value. This is the rayon
//! within-level determinism prerequisite (Principle #7: replay = live).
//!
//! What this file pins:
//! - **Test 1 (headline, non-tautological oracle):** two same-level `Period`
//!   nodes — producer `P` ticks BEFORE consumer `C` in graph order, yet `C`'s
//!   plain (`DropOldest`, snapshotted) `#[input]` reads the value `P` published
//!   on the PRIOR step (`v - 1`), NOT `P`'s same-step value (`v`). The buggy
//!   (no-snapshot) behavior would be `v` — a live read of `P`'s same-step
//!   publish (C ticks after P), so asserting `v - 1` PROVES the freeze.
//! - **Test 2 (determinism):** the Test-1 graph run twice produces a
//!   byte-identical recorded read sequence (the decide/snapshot/tick split is
//!   reproducible).
//! - **Test 3 (fire-gating):** an `External` consumer's non-trigger input is
//!   snapshotted/drained ONLY on steps where it fires — on a non-fire step the
//!   snapshot pass is skipped, so frames pile up and the next fire sees a stale
//!   (pre-pileup) frozen value, not a step-rate-drained latest one.
//! - **Test 4:** `NodeContext::snapshot_inputs` with an UNWIRED
//!   name fires the loud `debug_assert!` (classifier/wiring desync is loud, not
//!   silent).
//! - **Test 5 (block-exclusion):** one `Period` consumer with
//!   a plain `#[input]` AND a `#[input(backpressure = block)]` from two distinct
//!   same-level producers — the plain sibling is FROZEN (`v - 1`) while the block
//!   input is EXCLUDED from the snapshot set and drains live (`v`). Pins that
//!   `build_snapshot_input_names` drops block while snapshotting the sibling.
//! - **Test 6 (DataTrigger asymmetric arm):** a `DataTrigger`
//!   consumer with a `#[input(trigger)]` and a plain `#[input]` sibling, arranged
//!   so the plain producer shares the consumer's level — the TRIGGER input reads
//!   THIS step's data (live) while the PLAIN sibling reads the PRIOR step's value
//!   (frozen). Pins the `DataTrigger { input_name }` arm (snapshot all-but-the-
//!   trigger). Plus a determinism companion.
//! - **Test 7 (sample-inclusion):** a same-level `Period`
//!   consumer with a `#[input(backpressure = sample(N))]` non-trigger input reads
//!   the FROZEN prior-step value (`v - 1`), proving `sample(N)` IS snapshotted
//!   (only `block` is excluded). A mutation excluding sample would read live `v`.
//!   Plus a determinism companion.
//! - **Test 8 (flat-vs-level byte-identity):** an all-`Period`
//!   multi-node graph run through `GraphRuntime::step` (level path) produces a
//!   byte-identical `TraceEntry` stream to a bare `Scheduler::step` (flat path)
//!   over the same ids/order/policy — the headline byte-identity claim, pinned
//!   directly via the two paths' public `trace()` accessors.
//! - **Test 9 (Sync trigger-scoped snapshot arm):** the Test-6 shape
//!   with a `sync_window_ms` consumer — TWO `#[input(trigger)]` ports read THIS
//!   step's data (live) while the PLAIN sibling (fed by a same-level producer)
//!   reads the PRIOR step's value (frozen). Pins `build_snapshot_input_names`'s
//!   Sync complement arm e2e. The level-sharing is itself load-bearing:
//!   the plain edge is non-triggering so it does NOT levelize the consumer below
//!   its same-level producer — earlier Sync classified EVERY input as
//!   triggering (the consumer sat a level lower AND nothing was snapshotted).
//!
//! CRITICAL: the step-boundary snapshot is ONLY active for REAL
//! `#[cerulion_node]` MACRO nodes — `ClosureNodeEntry` / `DylibNodeEntry`
//! inherit the no-op `NodeEntry::snapshot_inputs` default (deferral #16). So the
//! frozen-value tests use macro nodes; a closure consumer would never freeze
//! (it would read live). No fake data (Principle #13): every recorded value
//! comes from a real iceoryx2 publish read inside a real tick.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{AnySubscriber, NodeContext, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::scheduler::{NodeConfig, Scheduler, TraceEntry, TriggerPolicy};
use cerulion_core::testing::TestTransport;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

// A sentinel the producer never publishes (it publishes 1, 2, 3, ...). If a
// measured-step read records this, the consumer's tick body did NOT run (a
// `None`/Empty frozen slot collapses the macro's nested `try_view` to a no-op),
// which the measured-step assert REJECTS — this keeps the test from silently
// degrading to a tautology.
const MISSING: u64 = u64::MAX;

// ===========================================================================
// Shared producer: a Period node publishing an incrementing counter (1, 2, ...)
// into a fixed Vector3 field. `self.out.x = ...` writes straight into the
// loaned SHM slot (Vector3 is a fixed schema; fields via Deref).
// ===========================================================================

#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct SnapProducer {
    #[output]
    out: Vector3,
    n: u64,
}

#[cerulion_node_impl]
impl SnapProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // Increment FIRST so the first publish carries 1, not 0 — keeps the
        // recorded values strictly positive and the MISSING sentinel distinct.
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

// ===========================================================================
// Test 1 + 2: same-level Period consumer with a plain (snapshotted) input.
// Records EVERY tick's read into a shared atomic (the macro input-read idiom
// is `self.inp.x` — the same field read FloodProducer/SlowDropConsumer use in
// backpressure_event_iox2_test.rs).
// ===========================================================================

/// Same-level (`Period`) consumer. Its plain `#[input]` defaults to
/// `DropOldest`, so the runtime SNAPSHOTS it at the level boundary. Each tick
/// it records what it read into `last_read` (initialized to MISSING by the
/// harness before each measured step, so a tick that does NOT run leaves the
/// sentinel and the assert fails loud).
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct SnapConsumer {
    #[input]
    inp: Vector3,
    /// Shared with the harness — set to the read value EVERY tick.
    last_read: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl SnapConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        // `self.inp.x` serves the FROZEN slot (snapshot_latest) — the value as
        // of the level boundary, BEFORE the same-level producer's same-step
        // tick published this step's value.
        self.last_read.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }
}

/// Build the Test-1 graph: producer `P` wired BEFORE consumer `C` in the
/// NodeDef vector (graph order → P ticks before C in tick_decided), C reads
/// `P/out`. `last_read` is the consumer's shared record atomic.
fn snap_graph(
    prefix: &str,
    last_read: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "snap_frozen".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "snap_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "snap_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(SnapProducerEntry::new()));
    let consumer = SnapConsumer {
        last_read,
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(SnapConsumerEntry::with_state(consumer)),
    );
    (config, factories)
}

/// Run the Test-1 graph `warmup + measured` steps; return the consumer's
/// recorded read on each MEASURED step, paired with the producer counter `v`
/// it published THAT step. Both nodes are Period(10ms), so each 10ms step fires
/// both exactly once: on step `k` (1-indexed over the whole run) the producer
/// publishes `k`, so on the i-th measured step the producer value is
/// `warmup + i + 1`.
fn run_snap(prefix: &str, warmup: u32, measured: u32) -> Vec<(u64, u64)> {
    let last_read = Arc::new(AtomicU64::new(MISSING));
    let (config, factories) = snap_graph(prefix, Arc::clone(&last_read));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build snap graph");

    // WARMUP: let the producer publish a few frames and the consumer's
    // snapshot establish a non-Empty frozen slot, so the measured window is
    // free of the cold-start Empty read (where the macro tick body no-ops).
    for _ in 0..warmup {
        runtime.step(Duration::from_millis(10));
    }

    let mut out = Vec::with_capacity(measured as usize);
    for i in 0..measured {
        // Producer value published on this step: the (warmup + i + 1)-th tick.
        let v = (warmup + i + 1) as u64;
        // Reset to MISSING so a non-running consumer tick is detectable.
        last_read.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(10));
        out.push((v, last_read.load(Ordering::Relaxed)));
    }
    out
}

#[test]
fn snapshot_freezes_same_level_input_to_prior_step_value() {
    // THE headline oracle (non-tautological): producer `P` and consumer `C`
    // are BOTH `Period` → both level 0 (no DAG edge between them). In graph
    // order P ticks before C, so a LIVE read (no snapshot — the bug) would see
    // P's same-step value `v`. The step-boundary snapshot freezes C's `inp`
    // BEFORE P's same-step tick, so C reads the value P published on the PRIOR
    // step: exactly `v - 1`.
    //
    // Reading `v` (the same-step value) would mean the snapshot FAILED to
    // freeze before P's tick — the bug this test guards.
    let reads = run_snap("snapf", 4, 5);
    for (v, read) in &reads {
        assert_ne!(
            *read, MISSING,
            "consumer tick did NOT run on a measured step (frozen slot Empty — \
             warmup should make this unreachable): the macro try_view chain \
             collapsed to a no-op. producer value was {v}"
        );
        assert_eq!(
            *read,
            v - 1,
            "snapshot must freeze C's input to the PRIOR step's value: at the \
             step where P published {v}, C must read {} (the step-boundary / \
             frozen value), got {read}",
            v - 1
        );
        // Decisive non-tautology pin: the buggy live-read would record `v`.
        assert_ne!(
            *read, *v,
            "C read P's SAME-STEP value {v} — the snapshot FAILED to freeze \
             before P's tick (the bug this test guards)"
        );
    }
}

#[test]
fn snapshot_frozen_reads_are_deterministic() {
    // The one legitimate cross-run equality: same steps, fresh prefix each run
    // (build_for_test uses a per-test SHM root), assert the full recorded
    // (producer_value, frozen_read) sequence is byte-identical. Pins that the
    // decide/snapshot/tick split is reproducible (Principle #7).
    let a = run_snap("snapd1", 4, 6);
    let b = run_snap("snapd2", 4, 6);
    assert_eq!(
        a, b,
        "the frozen-read sequence must be bit-identical across runs \
         (decide/snapshot/tick split is reproducible — Principle #7). a={a:?} b={b:?}"
    );
    // Non-vacuous: the run actually recorded measured reads, and they froze.
    assert_eq!(a.len(), 6, "the measured window actually ran");
    for (v, read) in &a {
        assert_eq!(*read, v - 1, "and the recorded reads are the frozen values");
    }
}

// ===========================================================================
// Test 3 (fire-gating): an External consumer's non-trigger input is
// snapshotted/drained ONLY on steps where the consumer FIRES. On a non-fire
// step the snapshot pass is skipped (fire-gated), so the input is NOT drained
// at step-rate — frames pile up between triggers.
//
// Observable contract: the snapshot freezes the LATEST queued frame at the
// moment of a fire. With fire-gating, between two triggers many producer
// frames accumulate undrained; the next fire's snapshot then captures the
// latest of that pile. If the snapshot were NOT fire-gated (ran every step),
// the input would be drained every step and the consumer's read would advance
// by exactly one producer value per step. We trigger sparsely and assert the
// consumer's recorded read JUMPS by more than one producer value between
// consecutive fires — proof the input advanced at FIRE rate, not step rate.
// ===========================================================================

/// External consumer: fires only when the host calls `trigger_external`. Its
/// plain `#[input]` is a non-trigger latest-value input (External → ALL inputs
/// are snapshotted per build_snapshot_input_names). Records the value read on
/// each fire.
#[cerulion_node(external)]
#[derive(Default)]
struct ExtSnapConsumer {
    #[input]
    inp: Vector3,
    last_read: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl ExtSnapConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last_read.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
fn fire_gating_skips_snapshot_on_non_fire_steps() {
    let last_read = Arc::new(AtomicU64::new(MISSING));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "snap_firegate".to_string(),
        prefix: "snapfg".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "snap_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "ext_snap_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(SnapProducerEntry::new()));
    let consumer = ExtSnapConsumer {
        last_read: Arc::clone(&last_read),
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(ExtSnapConsumerEntry::with_state(consumer)),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build fire-gate graph");

    // Warmup: a few steps with a trigger each so the consumer's snapshot
    // establishes a non-Empty frozen slot (cold-start Empty → no-op tick).
    for _ in 0..3 {
        runtime.trigger_external("consumer").expect("trigger");
        runtime.step(Duration::from_millis(10));
    }

    // Fire the consumer, capture its read.
    runtime.trigger_external("consumer").expect("trigger");
    runtime.step(Duration::from_millis(10));
    let first = last_read.load(Ordering::Relaxed);
    assert_ne!(
        first, MISSING,
        "consumer tick must have run on its first measured fire"
    );

    // Now step MANY times WITHOUT triggering the consumer. If the snapshot were
    // NOT fire-gated, each step would drain/advance the input. Fire-gating
    // skips the snapshot pass on these non-fire steps, so the input is NOT
    // drained — frames pile up.
    const SILENT_STEPS: u64 = 20;
    for _ in 0..SILENT_STEPS {
        runtime.step(Duration::from_millis(10));
    }
    // The producer kept publishing during the silent window, so its counter
    // advanced by SILENT_STEPS. (Reading `last_read` here would still show
    // `first` — the consumer never ticked — but that only proves the tick is
    // gated, which the scheduler already guarantees. The snapshot-specific pin
    // is the JUMP below.)
    assert_eq!(
        last_read.load(Ordering::Relaxed),
        first,
        "the consumer must not have ticked during the silent window (no record \
         change) — its tick is fire-gated"
    );

    // Fire again. The snapshot now captures the LATEST queued producer frame.
    runtime.trigger_external("consumer").expect("trigger");
    runtime.step(Duration::from_millis(10));
    let second = last_read.load(Ordering::Relaxed);
    assert_ne!(
        second, MISSING,
        "consumer tick must have run on its next fire"
    );

    // THE fire-gating oracle: the read JUMPED by MORE than one producer value.
    // A step-rate-drained input (snapshot NOT fire-gated) would advance the
    // frozen value by exactly one per step regardless of fires, so the next
    // fire would read `first + 1`-ish. Because the snapshot is fire-gated, the
    // input was NOT drained during the silent window — the next fire snapshots
    // the latest of ~SILENT_STEPS+1 accumulated frames, so the read jumps far
    // past `first + 1`.
    let jump = second - first;
    assert!(
        jump > 1,
        "fire-gated snapshot: the read must jump by MORE than one producer \
         value across a {SILENT_STEPS}-step silent window (first={first}, \
         second={second}, jump={jump}) — a jump of 1 would mean the input was \
         drained at step rate (snapshot NOT fire-gated)"
    );
}

// ===========================================================================
// Test 4 (deferral #17): NodeContext::snapshot_inputs with an UNWIRED input
// name must fire the loud debug_assert! (a classifier/wiring desync is loud,
// not silent). NodeContext::for_tests is cheaply constructible with a real
// subscriber, so we pin the debug_assert directly (debug build).
// ===========================================================================

#[test]
#[should_panic(expected = "not a wired subscriber")]
fn snapshot_inputs_unknown_name_fires_debug_assert() {
    let tt = TestTransport::with_buffer_size(8);
    // A real subscriber wired under the name "inp".
    let sub = tt.subscriber("snap_assert/out");
    let mut subscribers: IndexMap<String, AnySubscriber> = IndexMap::new();
    subscribers.insert("inp".to_string(), AnySubscriber::Ipc(sub));
    let mut ctx = NodeContext::for_tests(IndexMap::new(), subscribers);

    // "bogus_name" is NOT a wired subscriber — the classifier/wiring desync
    // path must debug_assert!(false) (debug build) rather than silently skip.
    ctx.snapshot_inputs(&["bogus_name".to_string()]);
}

// ===========================================================================
// Test 5: the block-exclusion e2e pin.
//
// `build_snapshot_input_names` EXCLUDES `block` inputs from the snapshot set
// (their producer-pacing `outstanding` mirror must drain at the consumer's tick,
// not the level boundary) while STILL snapshotting a sibling plain (DropOldest)
// input. This test pins both halves at once on ONE consumer:
//
//   - a PLAIN `#[input]` (DropOldest → snapshotted): reads the FROZEN prior-step
//     value `v - 1`, exactly like the headline test.
//   - a `#[input(backpressure = block, depth = N)]` (EXCLUDED from snapshot):
//     NOT frozen — drains at the consumer's tick (graph order: its producer
//     ticks first, then the consumer), so it reads the producer's SAME-step
//     value `v` (a live read).
//
// Topology (all three `Period(10)` → all level 0, no DAG edges — non-trigger
// reads do not levelize): `producer_plain`, `producer_block`, then `consumer`
// (consumer declared LAST → ticks AFTER both producers within level 0). Two
// SEPARATE topics, each a single-producer→single-consumer edge (the block edge
// is its own all-block topic — valid). Block depth is large (16) so the block
// producer never defers across the measured window (it drains one frame per
// consumer tick, publishes one per its own tick — lockstep, never backs up).
//
// THE oracle (non-tautological): plain == v-1 AND block == v on the SAME tick.
// A regression that snapshotted the block input too would make block == v-1
// (caught); one that snapshotted neither would make plain == v (caught); the
// MISSING sentinel catches a no-op tick.
// ===========================================================================

/// Consumer with TWO inputs from two distinct producers: one plain (snapshotted)
/// and one `block` (excluded). Records BOTH reads every tick.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct DualInputConsumer {
    /// Plain → DropOldest → SNAPSHOTTED (frozen to prior step's value).
    #[input]
    inp_plain: Vector3,
    /// Block → EXCLUDED from snapshot → drains live at this tick.
    #[input(backpressure = block, depth = 16)]
    inp_block: Vector3,
    /// Shared with the harness: the plain input's read value, set every tick.
    plain_read: Arc<AtomicU64>,
    /// Shared with the harness: the block input's read value, set every tick.
    block_read: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl DualInputConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.plain_read
            .store(self.inp_plain.x as u64, Ordering::Relaxed);
        self.block_read
            .store(self.inp_block.x as u64, Ordering::Relaxed);
        Ok(())
    }
}

/// Run the dual-input graph `warmup + measured` steps; return per measured step
/// `(producer_value_v, plain_read, block_read)`. Both producers are `Period(10)`
/// publishing the SAME counter cadence, so on the i-th measured step BOTH
/// published `warmup + i + 1`.
fn run_dual(prefix: &str, warmup: u32, measured: u32) -> Vec<(u64, u64, u64)> {
    let plain_read = Arc::new(AtomicU64::new(MISSING));
    let block_read = Arc::new(AtomicU64::new(MISSING));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "snap_dual".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer_plain".to_string(),
                node_type: "snap_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer_block".to_string(),
                node_type: "snap_producer".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            // Consumer declared LAST → ticks after both producers within level 0.
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "dual_input_consumer".to_string(),
                inputs: vec![
                    InputDef {
                        name: "inp_plain".to_string(),
                        source: "producer_plain/out".to_string(),
                    },
                    InputDef {
                        name: "inp_block".to_string(),
                        source: "producer_block/out".to_string(),
                    },
                ],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer_plain".to_string(),
        Box::new(SnapProducerEntry::new()),
    );
    factories.insert(
        "producer_block".to_string(),
        Box::new(SnapProducerEntry::new()),
    );
    let consumer = DualInputConsumer {
        plain_read: Arc::clone(&plain_read),
        block_read: Arc::clone(&block_read),
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(DualInputConsumerEntry::with_state(consumer)),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build dual graph");

    for _ in 0..warmup {
        runtime.step(Duration::from_millis(10));
    }
    let mut out = Vec::with_capacity(measured as usize);
    for i in 0..measured {
        let v = (warmup + i + 1) as u64;
        plain_read.store(MISSING, Ordering::Relaxed);
        block_read.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(10));
        out.push((
            v,
            plain_read.load(Ordering::Relaxed),
            block_read.load(Ordering::Relaxed),
        ));
    }
    out
}

#[test]
fn block_input_excluded_from_snapshot_plain_sibling_frozen() {
    let reads = run_dual("snapdual", 4, 5);
    for (v, plain, block) in &reads {
        assert_ne!(*plain, MISSING, "consumer tick did not run (plain): v={v}");
        assert_ne!(*block, MISSING, "consumer tick did not run (block): v={v}");
        // Plain input is SNAPSHOTTED → frozen at the level boundary → prior step.
        assert_eq!(
            *plain,
            v - 1,
            "plain (DropOldest) input must be FROZEN to the prior step's value \
             {} at the step where its producer published {v}, got {plain}",
            v - 1
        );
        // Block input is EXCLUDED from snapshot → drains live → same-step value.
        assert_eq!(
            *block, *v,
            "block input must NOT be frozen — it drains at the consumer's tick \
             (after its producer's same-step publish), so it reads the LIVE \
             same-step value {v}, got {block}"
        );
        // Decisive asymmetry pin: the two siblings read DIFFERENT values on the
        // same tick. A regression that snapshotted the block input too would
        // collapse this to plain == block == v-1.
        assert_ne!(
            *plain, *block,
            "the snapshotted plain input ({plain}) and the excluded block input \
             ({block}) must read DIFFERENT values on the same tick — equal values \
             mean the block input was wrongly snapshotted (== v-1) or the plain \
             input was wrongly left live (== v)"
        );
    }
    // The producer must keep flowing — the block edge never permanently defers
    // (depth 16, drained every consumer tick). The last block read is the live v.
    let (last_v, _, last_block) = reads.last().unwrap();
    assert_eq!(
        *last_block, *last_v,
        "the block producer kept publishing (no permanent defer); last block read \
         is the live value"
    );
}

// ===========================================================================
// Test 6: the DataTrigger asymmetric snapshot arm.
//
// `build_snapshot_input_names`'s `DataTrigger { input_name }` arm snapshots
// EVERY input EXCEPT the trigger input. This test pins that asymmetry e2e: on a
// fire, the TRIGGER input reads THIS step's data (live — it is the input that
// fired the node), while a PLAIN (non-trigger) sibling input reads the PRIOR
// step's value (FROZEN at the level boundary).
//
// Level structure (the load-bearing part — the plain producer must be SAME-LEVEL
// as the consumer for the freeze to be VALUE-observable):
//
//   level 0: `src`     Period(10) source → `src/out`
//   level 1: `trig_a`  DataTrigger on `src/out` → republishes counter to `a`
//            `trig_b`  DataTrigger on `src/out` → republishes counter to `b`
//   level 2: `qsib`    DataTrigger on `trig_b/b` → republishes counter to `q`
//            `cons`    DataTrigger: TRIGGER input from `trig_a/a`;
//                                   PLAIN  input  from `qsib/q`
//
// `cons` is level 2 (one above its trigger producer `trig_a` at level 1). `qsib`
// is ALSO level 2 (one above ITS trigger producer `trig_b` at level 1). So `cons`
// and `qsib` SHARE level 2 — and `qsib` feeds `cons`'s PLAIN input (a non-trigger
// read, which does NOT levelize, so it does not push `cons` to level 3). Within
// level 2, `qsib` is declared BEFORE `cons` → `qsib` ticks first (publishes its
// same-step value) but the snapshot froze `cons`'s plain input at the level-2
// BOUNDARY (before any level-2 tick), so `cons` reads `qsib`'s PRIOR value.
//
// Every node forwards a single incrementing chain so values are comparable. The
// counter that reaches `cons` is the same `n` everywhere: `src` publishes `n`
// each step, `trig_a`/`trig_b` forward `n`, `qsib` forwards `n`. So on the step
// where the chain carries value `v`:
//   - TRIGGER input (from trig_a, live)  == v   (this step's data fired cons)
//   - PLAIN   input (from qsib, frozen)  == v-1 (the level-2-boundary value)
//
// THE oracle (non-tautological): trigger == v AND plain == v-1. A regression
// that snapshotted the trigger input too would make trigger == v-1 (caught); one
// that snapshotted NONE would make plain == v (caught).
// ===========================================================================

/// A forwarding DataTrigger node: fires on its trigger input, copies `x` to its
/// output. Used for `trig_a`, `trig_b`, `qsib` (same code, distinct instances).
#[cerulion_node]
#[derive(Default)]
struct ForwardNode {
    #[input(trigger)]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl ForwardNode {
    fn tick(&mut self) -> Result<(), NodeError> {
        let v = self.inp.x;
        self.out.x = v;
        Ok(())
    }
}

/// The consumer under test: a DataTrigger node with a TRIGGER input and a PLAIN
/// (non-trigger) sibling input. Records BOTH reads on each fire.
#[cerulion_node]
#[derive(Default)]
struct TrigPlainConsumer {
    /// The trigger → NOT snapshotted (it fires the node, read live).
    #[input(trigger)]
    trig: Vector3,
    /// The non-trigger sibling → SNAPSHOTTED (frozen at the level boundary).
    #[input]
    plain: Vector3,
    trig_read: Arc<AtomicU64>,
    plain_read: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl TrigPlainConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.trig_read.store(self.trig.x as u64, Ordering::Relaxed);
        self.plain_read
            .store(self.plain.x as u64, Ordering::Relaxed);
        Ok(())
    }
}

fn vec3_out(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    }
}

/// Run the DataTrigger asymmetric-arm graph; return per measured step
/// `(chain_value_v, trig_read, plain_read)`.
fn run_trig_plain(prefix: &str, warmup: u32, measured: u32) -> Vec<(u64, u64, u64)> {
    let trig_read = Arc::new(AtomicU64::new(MISSING));
    let plain_read = Arc::new(AtomicU64::new(MISSING));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "snap_trig_plain".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            // level 0
            NodeDef {
                fuse: None,
                ros2: None,
                id: "src".to_string(),
                node_type: "snap_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            // level 1
            NodeDef {
                fuse: None,
                ros2: None,
                id: "trig_a".to_string(),
                node_type: "forward_node".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "src/out".to_string(),
                }],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "trig_b".to_string(),
                node_type: "forward_node".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "src/out".to_string(),
                }],
                outputs: vec![vec3_out("out")],
            },
            // level 2: qsib declared BEFORE cons → qsib ticks first within level 2
            NodeDef {
                fuse: None,
                ros2: None,
                id: "qsib".to_string(),
                node_type: "forward_node".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "trig_b/out".to_string(),
                }],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "cons".to_string(),
                node_type: "trig_plain_consumer".to_string(),
                inputs: vec![
                    InputDef {
                        name: "trig".to_string(),
                        source: "trig_a/out".to_string(),
                    },
                    InputDef {
                        name: "plain".to_string(),
                        source: "qsib/out".to_string(),
                    },
                ],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("src".to_string(), Box::new(SnapProducerEntry::new()));
    factories.insert("trig_a".to_string(), Box::new(ForwardNodeEntry::new()));
    factories.insert("trig_b".to_string(), Box::new(ForwardNodeEntry::new()));
    factories.insert("qsib".to_string(), Box::new(ForwardNodeEntry::new()));
    let cons = TrigPlainConsumer {
        trig_read: Arc::clone(&trig_read),
        plain_read: Arc::clone(&plain_read),
        ..Default::default()
    };
    factories.insert(
        "cons".to_string(),
        Box::new(TrigPlainConsumerEntry::with_state(cons)),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build trig/plain graph");

    for _ in 0..warmup {
        runtime.step(Duration::from_millis(10));
    }
    let mut out = Vec::with_capacity(measured as usize);
    for _ in 0..measured {
        trig_read.store(MISSING, Ordering::Relaxed);
        plain_read.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(10));
        out.push((
            // The chain value reaching `cons` THIS step is observed via the
            // live trigger read (it fired the node); the oracle below ties the
            // frozen plain read to it as `trig - 1`, so no separate `v` math is
            // needed (the multi-hop chain has its own warmup latency).
            trig_read.load(Ordering::Relaxed),
            trig_read.load(Ordering::Relaxed),
            plain_read.load(Ordering::Relaxed),
        ));
    }
    out
}

#[test]
fn datatrigger_snapshots_all_but_trigger_input() {
    // Warmup generously: the chain is `src -> trig_{a,b} -> qsib -> cons`, a
    // 3-hop trigger chain, so values take several steps to propagate steadily.
    let reads = run_trig_plain("snaptp", 8, 5);
    for (_v, trig, plain) in &reads {
        assert_ne!(
            *trig, MISSING,
            "cons tick did not run (trigger read missing)"
        );
        assert_ne!(
            *plain, MISSING,
            "cons tick did not run (plain read missing)"
        );
        // The TRIGGER input is read LIVE (it fired the node) → this step's value.
        // The PLAIN sibling is SNAPSHOTTED at the level-2 boundary → prior value.
        // `qsib` and `cons` share level 2 and carry the SAME chain counter, so
        // the frozen plain read is exactly one chain-step behind the live trigger.
        assert_eq!(
            *plain,
            trig - 1,
            "DataTrigger arm: the PLAIN sibling input must be FROZEN to the prior \
             step's chain value (trig={trig} live, plain must be {}), got {plain}",
            trig - 1
        );
        // Decisive asymmetry pin: trigger live, plain frozen — they differ.
        assert_ne!(
            *trig, *plain,
            "trigger ({trig}, live) and plain ({plain}, frozen) must differ — equal \
             means the trigger was wrongly snapshotted (both v-1) or the plain was \
             wrongly left live (both v)"
        );
    }
}

#[test]
fn datatrigger_asymmetric_snapshot_is_deterministic() {
    let a = run_trig_plain("snaptpd1", 8, 6);
    let b = run_trig_plain("snaptpd2", 8, 6);
    assert_eq!(
        a, b,
        "the DataTrigger (trigger live, plain frozen) read sequence must be \
         bit-identical across runs (Principle #7). a={a:?} b={b:?}"
    );
    assert_eq!(a.len(), 6, "the measured window actually ran");
    for (_v, trig, plain) in &a {
        assert_eq!(
            *plain,
            trig - 1,
            "and the recorded reads hold the asymmetry"
        );
    }
}

// ===========================================================================
// Test 7: `sample(N)` is snapshotted (freeze pin).
//
// `build_snapshot_input_names` EXCLUDES `block` but INCLUDES `sample(N)` —
// sample is consumer-side decimation (no producer pacing), so the step-boundary
// freeze is its CORRECT latest-value semantics. Tests 1/5 pin plain (DropOldest)
// is frozen and block is excluded; nothing pinned that a `sample(N)` non-trigger
// input is ACTUALLY FROZEN. A mutation that wrongly treated sample like block
// (excluding it) would leave the input live and pass everything else.
//
// This is the headline test (Test 1) with a `sample(N)` sibling: same-level
// `Period(10)` producer + `Period(10)` consumer whose `#[input(backpressure =
// sample(N))]` is a non-trigger latest-value input. Assert the consumer reads
// the FROZEN prior-step value (`v - 1`), NOT the same-step value (`v`).
//
// SAMPLE-GATE TIMING (load-bearing): `sample(N)` accepts at most one message per
// N ms keyed off the WIRE timestamp. To make the FREEZE (not the decimation
// gate) produce `v - 1`, the producer must publish a fresh-enough frame each
// step that the gate ACCEPTS it. With both nodes `Period(10)` and `sample(5)`,
// consecutive frames are 10 ms apart (> the 5 ms gate), so EVERY frame is
// accepted — the gate never decimates at steady state, and the only thing
// shifting the read off `v` is the level-boundary freeze. (If the gate ever
// decimated here it would freeze an even-older value, so `< v` still holds; the
// assert below pins the exact `v - 1` because the chosen timing accepts every
// frame.) A mutation excluding sample → the consumer reads live `v` → fails.
// ===========================================================================

/// Same-level `Period` consumer whose input uses `sample(5)`. Because both
/// producer and consumer are `Period(10)` (10 ms inter-arrival > the 5 ms gate),
/// every frame is accepted — so the read shifts off the live value ONLY via the
/// step-boundary snapshot, which this test pins.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct SampleConsumer {
    #[input(backpressure = sample(5))]
    inp: Vector3,
    last_read: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl SampleConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last_read.store(self.inp.x as u64, Ordering::Relaxed);
        Ok(())
    }
}

/// Build + run the sample-sibling graph; return per measured step
/// `(producer_value_v, sample_read)`. Identical shape to `run_snap` (Test 1)
/// but the consumer's input is `sample(5)` instead of plain.
fn run_sample(prefix: &str, warmup: u32, measured: u32) -> Vec<(u64, u64)> {
    let last_read = Arc::new(AtomicU64::new(MISSING));
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "snap_sample".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "snap_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "sample_consumer".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(SnapProducerEntry::new()));
    let consumer = SampleConsumer {
        last_read: Arc::clone(&last_read),
        ..Default::default()
    };
    factories.insert(
        "consumer".to_string(),
        Box::new(SampleConsumerEntry::with_state(consumer)),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build sample graph");

    for _ in 0..warmup {
        runtime.step(Duration::from_millis(10));
    }
    let mut out = Vec::with_capacity(measured as usize);
    for i in 0..measured {
        let v = (warmup + i + 1) as u64;
        last_read.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(10));
        out.push((v, last_read.load(Ordering::Relaxed)));
    }
    out
}

#[test]
fn sample_input_is_snapshotted_frozen_to_prior_step() {
    let reads = run_sample("snapsmp", 4, 5);
    for (v, read) in &reads {
        assert_ne!(
            *read, MISSING,
            "consumer tick did not run on a measured step (v={v})"
        );
        // sample(N) is INCLUDED in the snapshot set → frozen at the level
        // boundary → reads the producer's PRIOR-step value, exactly like a plain
        // (DropOldest) input.
        assert_eq!(
            *read,
            v - 1,
            "sample(N) input must be FROZEN to the prior step's value {} at the \
             step where the producer published {v}, got {read} — if it read {v} \
             the sample input was wrongly EXCLUDED from the snapshot (treated like \
             block) and read live",
            v - 1
        );
        // Decisive mutation pin: a wrongly-excluded sample input reads live `v`.
        assert_ne!(
            *read, *v,
            "sample input read the SAME-STEP value {v} — it was wrongly excluded \
             from the snapshot set (sample must be snapshotted, only block is \
             excluded)"
        );
    }
}

#[test]
fn sample_snapshot_is_deterministic() {
    let a = run_sample("snapsmpd1", 4, 6);
    let b = run_sample("snapsmpd2", 4, 6);
    assert_eq!(
        a, b,
        "the sample-input frozen-read sequence must be bit-identical across runs \
         (Principle #7). a={a:?} b={b:?}"
    );
    assert_eq!(a.len(), 6, "the measured window actually ran");
    for (v, read) in &a {
        assert_eq!(*read, v - 1, "and the recorded reads are the frozen values");
    }
}

// ===========================================================================
// Test 8: flat-vs-level byte-identity.
//
// The scheduler documents that the FLAT `Scheduler::step` path stays
// byte-identical to the pre-split fused form, while the LEVEL path
// (`GraphRuntime::step` → `decide_fires` → snapshot → `tick_decided`) is the new
// executor. `decide_fires`/`tick_decided` are `pub(crate)` (unreachable from an
// integration test), but BOTH executors surface their fire history via the SAME
// public `TraceEntry` stream — the LEVEL path through `GraphRuntime::trace()`,
// the FLAT path through `Scheduler::trace()`. This test cross-compares those two
// traces directly on an EQUIVALENT all-`Period` (all-level-0) multi-node graph.
//
// Equivalence: the level-path runtime uses three macro `Period(10)` nodes
// (`SnapProducer` instances p0/p1/p2, level 0, no edges); the flat-path bare
// `Scheduler` registers three `Period(10)` nodes with the SAME ids in the SAME
// insertion order (closure callbacks — the trace records only `(node_id,
// fire_time_ns)`, independent of callback bodies). Stepping both the same number
// of identical 10 ms steps must yield byte-identical trace vectors — the fire
// ORDER (graph/insertion order within the level) and fire TIMES must match.
//
// A regression where the level executor sorted within a level, dropped/duplicated
// a fire, or shifted a Period fire time would diverge the two traces here. This
// is a NICE-TO-HAVE (flat path covered by scheduler_test's 49 tests, level path
// by graph_test + Tests 1-7); it adds a DIRECT cross-path byte-identity pin.
// ===========================================================================

#[test]
fn flat_and_level_executors_produce_byte_identical_traces() {
    const IDS: [&str; 3] = ["p0", "p1", "p2"];
    const STEPS: u32 = 8;

    // LEVEL path: a GraphRuntime of three macro Period(10) sources, all level 0
    // (no inputs → no DAG edges → one wide level). Capture its scheduler trace.
    let level_trace: Vec<TraceEntry> = {
        let nodes: Vec<NodeDef> = IDS
            .iter()
            .map(|id| NodeDef {
                fuse: None,
                ros2: None,
                id: id.to_string(),
                node_type: "snap_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            })
            .collect();
        let config = GraphConfig {
            execution: None,
            level_assignments: None,
            network: None,
            process_groups: Default::default(),
            process_group_order: Default::default(),
            multi_publisher_topics: Vec::new(),
            name: None,
            identity: "byte_id_level".to_string(),
            prefix: "byteid".to_string(),
            nodes,
        };
        let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        for id in IDS {
            factories.insert(id.to_string(), Box::new(SnapProducerEntry::new()));
        }
        let clock = Arc::new(VirtualClock::new());
        let mut runtime =
            GraphRuntime::build_for_test(config, factories, clock, 8).expect("build level graph");
        for _ in 0..STEPS {
            runtime.step(Duration::from_millis(10));
        }
        runtime.trace().to_vec()
    };

    // FLAT path: a bare Scheduler with the SAME ids/order/policy. Callback bodies
    // are irrelevant to the trace (it records only node_id + fire_time_ns).
    let flat_trace: Vec<TraceEntry> = {
        let clock = Arc::new(VirtualClock::new());
        let mut scheduler = Scheduler::with_virtual_clock(clock);
        for id in IDS {
            scheduler
                .add_node(NodeConfig {
                    id: id.to_string(),
                    policy: TriggerPolicy::Period {
                        interval: Duration::from_millis(10),
                        max_catchup: None,
                    },
                    callback: Box::new(|| {}),
                })
                .expect("add flat node");
        }
        for _ in 0..STEPS {
            scheduler.step_ms(10);
        }
        scheduler.trace().to_vec()
    };

    // Non-vacuous: both ran (3 nodes × 8 steps = 24 fires each).
    assert_eq!(
        level_trace.len(),
        (IDS.len() as u32 * STEPS) as usize,
        "level trace must record every fire (3 nodes x {STEPS} steps)"
    );
    // THE byte-identity pin: the level executor's trace equals the flat
    // scheduler's trace, entry-for-entry (same fire order + same fire times).
    assert_eq!(
        level_trace, flat_trace,
        "the LEVEL executor (GraphRuntime::step -> decide_fires/tick_decided) must \
         produce a byte-identical fire trace to the FLAT Scheduler::step path for \
         the same node set + policies (the byte-identity claim). \
         level={level_trace:?} flat={flat_trace:?}"
    );
}

// ===========================================================================
// Test 9: the Sync TRIGGER-SCOPED snapshot arm.
//
// A `Sync`/`UnboundedSync` node now aligns ONLY its
// `#[input(trigger)]`-marked inputs; a plain `#[input]` sibling is a
// non-trigger latest-value read that `build_snapshot_input_names` now emits
// (previously the Sync arm snapshotted NONE). This is the Test-6 (DataTrigger
// asymmetric) shape with the consumer flipped to bounded Sync:
//
//   level 0: `src`     Period(10) source → `src/out`
//   level 1: `trig_a`  DataTrigger on `src/out` → forwards the counter
//            `trig_b`  DataTrigger on `src/out` → forwards the counter
//   level 2: `qsib`    DataTrigger on `trig_b/out` → forwards the counter
//            `scons`   Sync(50): TRIGGER inputs from `trig_a/out` + `trig_b/out`;
//                                PLAIN input from `qsib/out`
//
// `scons`'s DAG edges are its two triggers (both level-1 producers) → level 2,
// SHARED with `qsib` — possible ONLY because the plain `qsib/out` edge is
// non-triggering now (pre-flip it was an edge and pushed `scons` to
// level 3, where `qsib`'s same-step publish would be a PRIOR-level read and the
// freeze unobservable). Within level 2, `qsib` is declared BEFORE `scons` →
// `qsib` ticks first, but the snapshot froze `scons`'s plain input at the
// level-2 BOUNDARY.
//
// THE oracle (non-tautological, the Test-6 form): on a steady-state fire
// carrying chain value `v`, trigger reads == v (live — this step's data fired
// the alignment) AND plain read == v-1 (frozen). A regression snapshotting the
// triggers too → trig == v-1 (caught); one snapshotting none (the earlier
// Sync arm) → plain == v (caught).
// ===========================================================================

/// The Sync consumer under test: TWO `#[input(trigger)]` ports + a PLAIN
/// (non-trigger) sibling. Records one trigger read + the plain read per fire.
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct SyncTrigPlainConsumer {
    /// Trigger — aligned by check_sync, NOT snapshotted (read live).
    #[input(trigger)]
    trig_a: Vector3,
    /// Trigger — aligned by check_sync, NOT snapshotted (unread; its body
    /// data still arrives every fire step).
    #[input(trigger)]
    trig_b: Vector3,
    /// The non-trigger sibling → SNAPSHOTTED (frozen at the level boundary).
    #[input]
    plain: Vector3,
    trig_read: Arc<AtomicU64>,
    plain_read: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl SyncTrigPlainConsumer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.trig_read
            .store(self.trig_a.x as u64, Ordering::Relaxed);
        self.plain_read
            .store(self.plain.x as u64, Ordering::Relaxed);
        Ok(())
    }
}

/// Run the Sync trigger-scoped arm graph; return per measured step
/// `(trig_read, plain_read)`.
fn run_sync_trig_plain(prefix: &str, warmup: u32, measured: u32) -> Vec<(u64, u64)> {
    let trig_read = Arc::new(AtomicU64::new(MISSING));
    let plain_read = Arc::new(AtomicU64::new(MISSING));
    let config = GraphConfig {
        execution: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        level_assignments: None,
        network: None,
        name: None,
        identity: "snap_sync_trig_plain".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            // level 0
            NodeDef {
                fuse: None,
                ros2: None,
                id: "src".to_string(),
                node_type: "snap_producer".to_string(),
                inputs: vec![],
                outputs: vec![vec3_out("out")],
            },
            // level 1
            NodeDef {
                fuse: None,
                ros2: None,
                id: "trig_a".to_string(),
                node_type: "forward_node".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "src/out".to_string(),
                }],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "trig_b".to_string(),
                node_type: "forward_node".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "src/out".to_string(),
                }],
                outputs: vec![vec3_out("out")],
            },
            // level 2: qsib declared BEFORE scons → qsib ticks first within level 2
            NodeDef {
                fuse: None,
                ros2: None,
                id: "qsib".to_string(),
                node_type: "forward_node".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "trig_b/out".to_string(),
                }],
                outputs: vec![vec3_out("out")],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "scons".to_string(),
                node_type: "sync_trig_plain_consumer".to_string(),
                inputs: vec![
                    InputDef {
                        name: "trig_a".to_string(),
                        source: "trig_a/out".to_string(),
                    },
                    InputDef {
                        name: "trig_b".to_string(),
                        source: "trig_b/out".to_string(),
                    },
                    InputDef {
                        name: "plain".to_string(),
                        source: "qsib/out".to_string(),
                    },
                ],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("src".to_string(), Box::new(SnapProducerEntry::new()));
    factories.insert("trig_a".to_string(), Box::new(ForwardNodeEntry::new()));
    factories.insert("trig_b".to_string(), Box::new(ForwardNodeEntry::new()));
    factories.insert("qsib".to_string(), Box::new(ForwardNodeEntry::new()));
    let scons = SyncTrigPlainConsumer {
        trig_read: Arc::clone(&trig_read),
        plain_read: Arc::clone(&plain_read),
        ..Default::default()
    };
    factories.insert(
        "scons".to_string(),
        Box::new(SyncTrigPlainConsumerEntry::with_state(scons)),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build sync trig/plain graph");

    for _ in 0..warmup {
        runtime.step(Duration::from_millis(10));
    }
    let mut out = Vec::with_capacity(measured as usize);
    for _ in 0..measured {
        trig_read.store(MISSING, Ordering::Relaxed);
        plain_read.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(10));
        out.push((
            trig_read.load(Ordering::Relaxed),
            plain_read.load(Ordering::Relaxed),
        ));
    }
    out
}

#[test]
fn sync_snapshots_only_non_trigger_inputs() {
    // Warmup generously (the Test-6 rationale): the chain is a 3-hop trigger
    // chain, so values take several steps to propagate steadily.
    let reads = run_sync_trig_plain("snapsy", 8, 5);
    for (trig, plain) in &reads {
        assert_ne!(
            *trig, MISSING,
            "scons tick did not run (trigger read missing)"
        );
        assert_ne!(
            *plain, MISSING,
            "scons tick did not run (plain read missing)"
        );
        // TRIGGER inputs are read LIVE (this step's aligned data fired the
        // node). The PLAIN sibling is SNAPSHOTTED at the level-2 boundary →
        // the prior step's chain value. Earlier the Sync arm snapshotted
        // NOTHING (plain would read v, live) — this assert is the flip pin.
        assert_eq!(
            *plain,
            trig - 1,
            "Sync arm: the PLAIN sibling must be FROZEN to the prior step's \
             chain value (trig={trig} live, plain must be {}), got {plain}",
            trig - 1
        );
        // Decisive asymmetry pin: triggers live, plain frozen — they differ.
        assert_ne!(
            *trig, *plain,
            "trigger ({trig}, live) and plain ({plain}, frozen) must differ — \
             equal means the triggers were wrongly snapshotted (both v-1) or \
             the plain was wrongly left live (both v, the earlier Sync arm)"
        );
    }
}
