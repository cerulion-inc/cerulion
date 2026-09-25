// SPDX-License-Identifier: AGPL-3.0-only
//! Per-edge READ-OUTCOME capture over real iceoryx2.
//!
//! Drives small `#[cerulion_node]` graphs through `GraphRuntime::build_for_test`
//! (per-test SHM root — parallel-safe transport; the file is still `#[serial]`
//! because one arm mutates process env and the suite runs single-threaded in CI
//! anyway) with a REAL trace ring installed, then reads the ring back and
//! asserts the kind-6 records against HAND-WRITTEN oracles (step / node_idx /
//! input_idx / outcome kind / served wire sequence / popped / read-site role)
//! — never a self-compare. Wire sequences are the producers' commit counters (gap-free,
//! numbered at commit), so every expected seq is hand-computable.
//!
//! Level shapes matter to the oracles and are stated per test:
//! - a TRIGGER edge levelizes the consumer BELOW the producer, so the trigger
//!   drain sees the producer's SAME-step publishes (the within-step level collapse);
//! - a PLAIN (non-trigger) edge does NOT levelize, so producer and consumer
//!   share a level and the consumer's step-boundary snapshot reads the PRIOR
//!   step's publish (the snapshot runs before any same-level tick).
//!
//! Per-message FIFO (52125241e): a
//! Data-policy trigger input's reads now pop ONE frame per read in arrival
//! order — the UNIFIED boundary drain freezes the FIFO head (`DrainedBatch`
//! with popped = 1, served-seq = the head), the Separate tick body pops the
//! head live (`Served`, popped = 1), and a DEFERRED fire's held head makes
//! the boundary RE-OFFER without draining (NO kind-6 record — one record per
//! consumed frame, never per re-offer). Sync alignment, latest-value context
//! inputs and the Separate trigger-DRAIN subscriber keep drain-to-latest /
//! drain-everything, so those arms' oracles are FIFO-invariant.
//!
//! WITHIN-STEP BURST: per-message FIFO says how many frames ONE
//! read pops; the burst says how many READS a step performs. A Data node now
//! fires k times WITHIN the step it sees k queued arrivals, instead of firing
//! once and CARRYING the rest — so a step's record count is a function of that
//! step's QUEUE DEPTH, never a constant. It reaches the two disciplines
//! differently, which is why arms (a) and (e) record different shapes for the
//! same 2-frames-per-step stimulus: on `Separate` the boundary drain already
//! signals one arrival per queued frame, so the signalled count IS the burst;
//! on `Unified` the drain pops and freezes exactly ONE frame (there is one
//! frozen slot), so it can only ever signal one, and the fire loop REFILLS
//! between fires through the runtime's per-node hook — the SAME
//! `NodeEntry::drain_trigger_input`, hence one further `DrainedBatch` record
//! per refilled frame. A refill that pops NOTHING records nothing, so the
//! count stays exactly one record per CONSUMED frame on both paths. Arms
//! whose producer commits at most one frame per step (`StepSrc`, `SlowSrc`,
//! `CountSrc`) are burst-invariant: their bursts are always length 1.
//!
//! SERVE-MANY, the NON-TRIGGER half: the step-boundary snapshot is
//! the only read a plain `#[input]` performs per step, however many times its
//! node fires. Every fire serves that one frozen slot and a serve records
//! nothing, so a plain edge contributes EXACTLY ONE record per step — a
//! function of the step's READS, never of its FIRES. Arms (g) and (n) are the
//! two that turn on it, and both previously encoded serve-ONCE mechanics (fire
//! 1 consumed the slot; fires 2..k fell to a LIVE body drain and recorded).
//!
//! Every kind-6 record carries a READ-SITE ROLE, and the
//! `role_view` arms below are what read it back off a real run. Its own
//! coverage is tracked separately from the list below: sweep R1-R8 (the
//! stamp/steer/threshold regressions) is caught by the arms in
//! `replay_engine_test.rs`; the mint sites reached from HERE are the
//! `try_receive_timestamps`-calls-the-body-entry-point headline case (caught
//! by the Separate-discipline arm) and the six-flip of the per-set Sync
//! matcher's mints, which no capture suite in the first change set catches
//! directly, only indirectly, via the format-5 byte-exact
//! corpus arm — the direct capture-side oracles live in the `role_view` arms.
//!
//! Mutation targets (1 and 2 were RUN):
//! 1. Recording the seq of a DROPPED frame rather than the kept one (e.g.
//!    first-drained-wins in `drain_samples`) fails the Separate-discipline
//!    drained_batch oracle (arm (e)'s drain batches are 2 wide so first ≠
//!    last); on the FIFO pop-one paths the same class shows as ORDER — a
//!    latest-wins variant serves the NEWEST queued frame where the head oracle
//!    demands the oldest, so a burst walks its seqs out of order (or repeats
//!    one) instead of 2k then 2k+1, failing arms (a)/(e)/(j).
//! 2. Skipping the held-replay record (`snapshot_latest`'s Empty→Held arm)
//!    fails `non_trigger_snapshot_records_served_then_held...`'s Held rows.
//! 3. Double-recording via snapshot+try_view on one fire (re-draining instead
//!    of serving the frozen slot) fails the exactly-once count pins — every
//!    oracle here is an EXACT full-sequence compare, so an extra record per
//!    fire is a length mismatch (accounting-once is the house invariant).

#![cfg(unix)]

use std::collections::BTreeSet;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{
    BackpressurePolicy, ClosureNodeEntry, InputMeta, MacroPolicy, NodeEntry, NodeInfo, OutputMeta,
};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::message::ShmMessage;
use cerulion_core::prelude::*;
use cerulion_core::read_outcome::ReadSiteRole;
use cerulion_core::trace_ring::{
    default_capacity_records, read_site_role, unpack_read_outcome_meta, unpack_read_outcome_popped,
    TraceRingConsumer, TraceRingOwner, TraceRingRecord, READ_OUTCOME_DECIMATED,
    READ_OUTCOME_DRAINED_BATCH, READ_OUTCOME_HELD, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME,
    READ_OUTCOME_PRODUCER, READ_OUTCOME_SERVED, READ_OUTCOME_TRUNCATED, RECORD_TYPE_READ_OUTCOME,
};
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing_test::traced_test;

// ===========================================================================
// Node types
// ===========================================================================

/// Fast producer: `Period(5)` on a 10 ms step cadence ⇒ a 2-fire catch-up
/// burst per step (seqs 2k, 2k+1 on step k) — batches are ≥ 2 wide so a
/// first-vs-last (dropped-vs-kept) seq mutation is detectable.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct FastSrc {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl FastSrc {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        Ok(())
    }
}

/// Slow producer: `Period(30)` on a 10 ms cadence ⇒ fires on steps 2, 5, 8, …
/// (seq f on its f-th fire).
#[cerulion_node(period_ms = 30)]
#[derive(Default)]
struct SlowSrc {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl SlowSrc {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        Ok(())
    }
}

/// Per-step producer: `Period(10)` ⇒ one publish per step (seq k on step k).
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct StepSrc {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl StepSrc {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        Ok(())
    }
}

/// Silent producer: its output is NEVER written, so lazy loan never
/// publishes — the topic exists (graph-owned service) but carries no frame.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct SilentSrc {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl SilentSrc {
    fn tick(&mut self) -> Result<(), NodeError> {
        Ok(())
    }
}

/// Data-trigger consumer (the `#[input(trigger)]` field infers the policy).
/// Macro node ⇒ `unifies_trigger_drain` ⇒ `DrainSource::Unified` by default.
#[cerulion_node]
#[derive(Default)]
struct TrigSink {
    #[input(trigger)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl TrigSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// A data-trigger consumer whose input is provisioned
/// at the DEEPEST retainable queue (`MAX_CONSUMER_DEPTH` = 64), so one step
/// can hand it a FULL-DEPTH backlog — the within-step burst's
/// worst case, which is what the stage capacity is now sized against.
#[cerulion_node]
#[derive(Default)]
struct DeepTrigSink {
    #[input(trigger, depth = 64)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl DeepTrigSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// A `Period(1)` producer — one publish per catch-up fire, so a single
/// `step(N ms)` queues N frames for a downstream consumer in ONE step (the
/// backlog builder for the saturation arm).
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct BacklogSrc {
    #[output]
    out: Vector3,
    count: u32,
}

#[cerulion_node_impl]
impl BacklogSrc {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.count += 1;
        self.out.x = self.count as f64;
        Ok(())
    }
}

/// Period consumer with a plain (non-trigger, snapshotted) input.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct PollSink {
    #[input]
    inp: Vector3,
}

#[cerulion_node_impl]
impl PollSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// Period consumer with a `sample(25)` non-trigger input — the decimation
/// shape (accept iff the wire ts is ≥ 25 ms past the last accept).
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct SampleSink {
    #[input(backpressure = sample(25))]
    inp: Vector3,
}

#[cerulion_node_impl]
impl SampleSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// Period(5) consumer with a plain snapshotted input, driven at 10 ms steps ⇒
/// a 2-fire CATCH-UP burst per step — the arm-(g) shape. Under
/// SERVE-MANY both fires read the ONE frozen slot the step's snapshot captured,
/// and a serve records nothing, so the step carries exactly one record.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct CatchupSink {
    #[input]
    inp: Vector3,
}

#[cerulion_node_impl]
impl CatchupSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// `Period(1)` consumer with a plain snapshotted input — the run-exit
/// drop-report shape: one 700 ms step catch-up-fires it 700
/// times while staging exactly ONE read (the boundary snapshot), because every
/// fire serves the same frozen slot. Under serve-once the same burst
/// staged 1 snapshot + 699 live body reads and overflowed the 64-record stage.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct BurstSink {
    #[input]
    inp: Vector3,
}

#[cerulion_node_impl]
impl BurstSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// Counting producer: `Period(10)` ⇒ one publish per step carrying the fire
/// ordinal (1.0, 2.0, …) — the VALUE oracle for the record-only arms (seq k
/// carries value k+1, so the delivered stream and the kind-6 stream derive
/// from ONE counter).
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct CountSrc {
    #[output]
    out: Vector3,
    count: u32,
}

#[cerulion_node_impl]
impl CountSrc {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.count += 1;
        self.out.x = f64::from(self.count);
        Ok(())
    }
}

/// A producer with a PLAIN (non-trigger)
/// context input — the MACRO control twin of the `ClosureNodeEntry` in
/// `a_serial_gated_producer_does_not_count_toward_the_shared_pair`.
/// Same ports, same wiring; the one build-time difference is that a macro node
/// performs its own input snapshot (`performs_input_snapshot() == true`), so it
/// is NOT class-(A) serial-gated and DOES reach the level's REST.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct CtxSrc {
    #[input]
    ctx: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl CtxSrc {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.ctx.x;
        Ok(())
    }
}

/// Data-trigger RELAY: republishes its trigger input's value — the
/// record-only arm's observable (the DELIVERED stream a host tap captures).
#[cerulion_node]
#[derive(Default)]
struct TrigRelay {
    #[input(trigger)]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl TrigRelay {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.inp.x;
        Ok(())
    }
}

/// Period consumer with TWO plain inputs in a KNOWN declaration order
/// (`first`, then `second`) — the input_idx > 0 pin.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct DualPollSink {
    #[input]
    first: Vector3,
    #[input]
    second: Vector3,
}

#[cerulion_node_impl]
impl DualPollSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.first.x;
        let _ = self.second.x;
        Ok(())
    }
}

/// Period consumer with a `block`-policy plain input — the node (and its
/// producer) land in the level's BLOCK partition (`LevelPlan::block_ids`), so its
/// kind-6 records reach the ring only through the merge's
/// `.chain(block_ids.iter())` half.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct BlockSink {
    #[input(backpressure = block)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl BlockSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// Data-trigger consumer rate-capped by `throttle_ms = 25`: the trigger drain
/// pops EVERY step in `drain_level` while the FIRE is deferred on most steps —
/// the level-end-merge (vs fire-scoped) rationale's shape.
#[cerulion_node(throttle_ms = 25)]
#[derive(Default)]
struct ThrottleTrigSink {
    #[input(trigger)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl ThrottleTrigSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// Data-trigger consumer whose TRIGGER input is `sample(25)`-gated — the
/// Decimated rows via `snapshot_latest_for_trigger` (the
/// decimation arm (c) covers the NON-trigger snapshot path).
#[cerulion_node]
#[derive(Default)]
struct SampleTrigSink {
    #[input(trigger, backpressure = sample(25))]
    inp: Vector3,
}

#[cerulion_node_impl]
impl SampleTrigSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// Bounded-sync consumer (two trigger inputs) — the PER-SET Sync drain path
/// (the align pass drains each trigger member through the node's OWN
/// body subscriber, and the body's `try_view` then serves that frozen slot).
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct SyncSink {
    #[input(trigger)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
}

#[cerulion_node_impl]
impl SyncSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.a.x;
        let _ = self.b.x;
        Ok(())
    }
}

/// The P1-4 / P1-5 consumer: it carries TWO edges of DIFFERENT
/// PRODUCER-ANNOTATION classes at once — `shared` (a `#[input(trigger)]` on
/// whichever topic the arm puts under test) and `solo` (a plain `#[input]` on
/// an ordinary single-writer, graph-owned topic).
///
/// `solo` is the in-body ANTI-TAUTOLOGY control for both annotation arms: ONE
/// runtime, ONE recording, ONE build-time property apart
/// (`edge_needs_producer_annotation`). Without it, "annotations appear on the
/// edge under test" is equally satisfied by a runtime that annotates EVERY
/// edge — which would be the same defect from the other side (a per-read
/// `sample.origin()` call on every single-publisher edge in every shipped
/// graph, the cost `capture_producer_token`'s second conjunct exists to
/// avoid).
#[cerulion_node]
#[derive(Default)]
struct AnnotDualSink {
    #[input(trigger)]
    shared: Vector3,
    #[input]
    solo: Vector3,
}

#[cerulion_node_impl]
impl AnnotDualSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.shared.x;
        let _ = self.solo.x;
        Ok(())
    }
}

/// The same two-edge shape as
/// [`AnnotDualSink`], with `shared` a PLAIN `sample(25)`-gated input instead of
/// a trigger.
///
/// That one change routes the edge onto `snapshot_latest`, which is the ONLY
/// read path in the system that SERVES a frame while recording a kind that is
/// not `Served`/`Held`/`DrainedBatch`: when the gate drops the fresh frame the
/// drain yields Empty, `snapshot_latest` records `Decimated`, and the frozen
/// slot is left at `Held` — so the body's `try_view` reads the held
/// frame that step. `try_view`'s own live arm and the trigger boundary drain
/// both record `Decimated` too, but neither serves anything (Empty is
/// `Ok(None)` / no fire), which is why the annotation belongs here and only
/// here.
///
/// `solo` is the in-body ANTI-TAUTOLOGY control, exactly as on
/// [`AnnotDualSink`]: ONE runtime, ONE recording, ONE build-time property
/// apart.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct AnnotSampledSink {
    #[input(backpressure = sample(25))]
    shared: Vector3,
    #[input]
    solo: Vector3,
}

#[cerulion_node_impl]
impl AnnotSampledSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.shared.x;
        let _ = self.solo.x;
        Ok(())
    }
}

// ===========================================================================
// Harness
// ===========================================================================

/// A decoded kind-6 record, in oracle-comparable form.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
struct ReadRec {
    step: u64,
    node_idx: u32,
    input_idx: u16,
    kind: u16,
    seq: u64,
    popped: u32,
}

/// Build the two-node graph `producer → consumer` (ids fixed: "producer",
/// "consumer"; input "inp" wired to "producer/out").
fn two_node_graph(
    prefix: &str,
    producer: Box<dyn NodeEntry>,
    consumer: Box<dyn NodeEntry>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "read_outcome_capture".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "ro_src".to_string(),
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
                node_type: "ro_sink".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), producer);
    factories.insert("consumer".to_string(), consumer);
    (config, factories)
}

/// Build the graph + a REAL trace ring (manifest carrying the input table),
/// install the producer, step `steps` × 10 ms, then drain the ring and return
/// (kind-6 records decoded, all records, the consumer's manifest input table).
fn run_recorded(
    tag: &str,
    producer: Box<dyn NodeEntry>,
    consumer: Box<dyn NodeEntry>,
    steps: u64,
) -> (Vec<ReadRec>, Vec<TraceRingRecord>, Vec<Vec<String>>) {
    let (config, factories) = two_node_graph(&format!("ro{tag}"), producer, consumer);
    run_recorded_custom(
        tag,
        config,
        factories,
        &["producer", "consumer"],
        &[&[], &["inp"]],
        steps,
    )
}

/// The generalized recorded-run harness: any graph shape, any manifest
/// node/input tables (parallel slices — the same contract as
/// `TraceRingOwner::create_with_inputs`). Steps `steps` × 10 ms.
fn run_recorded_custom(
    tag: &str,
    config: GraphConfig,
    factories: IndexMap<String, Box<dyn NodeEntry>>,
    node_ids: &[&str],
    node_inputs: &[&[&str]],
    steps: u64,
) -> (Vec<ReadRec>, Vec<TraceRingRecord>, Vec<Vec<String>>) {
    run_recorded_driven(
        tag,
        config,
        factories,
        node_ids,
        node_inputs,
        |runtime, _clock| {
            for _ in 0..steps {
                runtime.step(Duration::from_millis(10));
            }
        },
    )
}

/// [`run_recorded_custom`] with the STEP LOOP handed to the caller.
///
/// Every arm above drives its graph the same way — N × 10 ms, stimulus supplied
/// by in-graph producer nodes — and that shape cannot reach a read path whose
/// entry condition is a QUEUE SHAPE (a backlog on one trigger input beside a
/// scarce partner): an in-graph producer publishes on its own tick, so a step's
/// frames all carry the SAME wire stamp and the per-set matcher's descent, whose
/// first act is a strict `lo_count == 1` test, is unreachable.
///
/// The closure gets the runtime AND the graph's clock, so an arm can publish
/// externally at hand-chosen stamps (the `sync_per_set_iox2_test` pattern) and
/// read `NodeHandle` counters before the ring is drained. The ring, the manifest
/// and the drain are unchanged, so a driven arm's records are captured by
/// exactly the machinery every other arm's are.
fn run_recorded_driven(
    tag: &str,
    config: GraphConfig,
    factories: IndexMap<String, Box<dyn NodeEntry>>,
    node_ids: &[&str],
    node_inputs: &[&[&str]],
    drive: impl FnOnce(&mut GraphRuntime, &VirtualClock),
) -> (Vec<ReadRec>, Vec<TraceRingRecord>, Vec<Vec<String>>) {
    let node_id_strings: Vec<String> = node_ids.iter().map(|s| s.to_string()).collect();
    let ring_tag = format!("{tag}_{}", std::process::id());
    let mut owner = TraceRingOwner::create_with_inputs(
        &ring_tag,
        default_capacity_records(),
        0,
        node_ids,
        node_inputs,
    )
    .expect("create ring");
    let ring_name = owner.name().to_string();
    let producer_handle = owner.producer().expect("mint producer");

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 8)
        .expect("build graph");
    runtime.set_trace_ring_producer(producer_handle, &node_id_strings);
    assert!(
        runtime.read_outcome_stages_armed_for_test() > 0,
        "installing the ring must ARM the wired read-outcome stages \
         (anti-tautology for the recording-off arm)"
    );

    drive(&mut runtime, &clock);

    let mut ring_consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
    let input_names = ring_consumer.input_names().to_vec();
    let mut records = Vec::new();
    ring_consumer.drain(&mut records).expect("drain ring");
    drop(ring_consumer);
    runtime.shutdown();
    drop(owner);

    let kind6 = records
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_READ_OUTCOME)
        .map(|r| {
            let (input_idx, kind) = unpack_read_outcome_meta(r.global_level);
            ReadRec {
                step: r.step,
                node_idx: r.node_idx,
                input_idx,
                kind,
                seq: r.fire_time_ns,
                popped: unpack_read_outcome_popped(r.duration_ns),
            }
        })
        .collect();
    (kind6, records, input_names)
}

/// Shorthand for the consumer's expected record (node_idx 1, input_idx 0 —
/// the manifest order every test here uses).
fn rec(step: u64, kind: u16, seq: u64, popped: u32) -> ReadRec {
    ReadRec {
        step,
        node_idx: 1,
        input_idx: 0,
        kind,
        seq,
        popped,
    }
}

/// The N8 ANTI-INERT view: every kind-6 record of a recorded
/// run as `(step, input_idx, outcome kind, CALL-SITE role)`.
///
/// Deliberately a SEPARATE projection rather than a field on [`ReadRec`]: the
/// role is what this pair of arms is about, and folding it into the struct
/// every other arm's hand oracle is written against would have re-blessed ~30
/// expected vectors in one edit — the class where a wrong role rides in on a
/// mass rewrite nobody re-derived.
///
/// It reads the role through the PRODUCTION unpacker
/// ([`read_site_role`]), off the SAME `global_level` word the replay engine
/// reads, so a packing change that moved the bits fails here rather than
/// passing against a test-local re-implementation.
///
/// **The projection carries `node_idx` too.** Without
/// it the tuple addressed a record by `(step, input_idx)` alone, which is not a
/// key: the fixtures here run several nodes, `input_idx` restarts at 0 on each
/// of them, and a mint that recorded onto the WRONG NODE's stage would land a
/// row that reads identically. The addressing is now the same triple the replay
/// engine demuxes on.
fn role_view(records: &[TraceRingRecord]) -> Vec<(u64, u32, u16, u16, ReadSiteRole)> {
    records
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_READ_OUTCOME)
        .map(|r| {
            let (input_idx, kind) = unpack_read_outcome_meta(r.global_level);
            (
                r.step,
                r.node_idx,
                input_idx,
                kind,
                read_site_role(r.global_level),
            )
        })
        .collect()
}

/// A source `NodeDef` (one `Vector3` output "out", no inputs).
fn src_def(id: &str) -> NodeDef {
    NodeDef {
        fuse: None,
        ros2: None,
        id: id.to_string(),
        node_type: "ro_src".to_string(),
        inputs: vec![],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    }
}

/// A sink `NodeDef` with the given ordered `(input name, source)` wiring.
fn sink_def(id: &str, inputs: &[(&str, &str)]) -> NodeDef {
    NodeDef {
        fuse: None,
        ros2: None,
        id: id.to_string(),
        node_type: "ro_sink".to_string(),
        inputs: inputs
            .iter()
            .map(|(name, source)| InputDef {
                name: name.to_string(),
                source: source.to_string(),
            })
            .collect(),
        outputs: vec![],
    }
}

/// A bare `GraphConfig` around `nodes` under `prefix`.
fn graph_of(prefix: &str, nodes: Vec<NodeDef>) -> GraphConfig {
    GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "read_outcome_capture".to_string(),
        prefix: prefix.to_string(),
        nodes,
    }
}

// ===========================================================================
// Tests
// ===========================================================================

/// (a) UNIFIED trigger drain under the WITHIN-STEP BURST: the
/// trigger edge levelizes the consumer below the `Period(5)` producer, which
/// commits TWO frames per step (seqs 2k, 2k+1) — and the consumer now serves
/// BOTH inside that step. One record per FIRE, in arrival order.
///
/// The Unified boundary drain pops and freezes exactly ONE frame (the FIFO
/// head, seq 2k) and therefore signals ONE arrival however many are queued,
/// so `fire_count` is 1 and the burst comes from the fire loop REFILLING
/// between fires through the runtime's per-node hook — the SAME
/// `NodeEntry::drain_trigger_input`, through the SAME node lock. That refill
/// is an ordinary drain, so it stages its own `DrainedBatch(2k+1, popped 1)`
/// exactly as the boundary pop staged `DrainedBatch(2k, popped 1)`; the
/// step's THIRD refill finds the queue empty, pops nothing, and records
/// nothing (a silent drain is not an outcome), which is what holds the count
/// at one record per CONSUMED frame. No backlog accumulates, so seq 2k+1
/// really is the newest frame committed in step k.
///
/// Before SERVE-MANY this arm read ONE record per step (seq = k): the scheduler
/// consumed one signalled arrival per step and CARRIED the rest, capping a
/// data-trigger consumer's throughput at one frame per step. Either way the
/// seq sequence is the FIFO-order pin (a latest-wins regression serves 2k+1
/// before 2k), and the EXACT full-sequence compare stays the exactly-once
/// pin: the tick's `try_view` serves the frozen slot, so a re-draining variant
/// (snapshot + try_view both recording) adds records and fails the length
/// compare. (STEPS = 6 keeps each step's burst well under the depth-10 body
/// queue, so no eviction perturbs the sequence.)
#[test]
#[serial]
fn unified_trigger_drain_records_one_drained_batch_per_fire_in_arrival_order() {
    const STEPS: u64 = 6;
    let (kind6, _all, input_names) = run_recorded(
        "uni",
        Box::new(FastSrcEntry::new()),
        Box::new(TrigSinkEntry::new()),
        STEPS,
    );
    // The manifest input table round-trips through the real ring.
    assert_eq!(
        input_names,
        vec![Vec::<String>::new(), vec!["inp".to_string()]],
        "the ring manifest carries the consumer's ordered input table"
    );
    let expected: Vec<ReadRec> = (0..STEPS)
        .flat_map(|k| {
            [
                rec(k, READ_OUTCOME_DRAINED_BATCH, 2 * k, 1),
                rec(k, READ_OUTCOME_DRAINED_BATCH, 2 * k + 1, 1),
            ]
        })
        .collect();
    assert_eq!(
        kind6, expected,
        "one DrainedBatch(head, popped 1) per FIRE — the step's whole burst \
         (boundary pop 2k, then the refill's 2k+1), never one frame per step"
    );
}

/// (b) NON-trigger (snapshotted) input across silent steps: producer and
/// consumer share a level (a plain edge does not levelize), so the step-k
/// snapshot sees publishes through step k-1. `SlowSrc` (Period 30) fires on
/// steps 2, 5, 8 ⇒ the consumer reads NoFrame ×3, then Served(0) on step 3,
/// Held(0) ×2 (the SAME seq — the hold), Served(1) on step 6, …
#[test]
#[serial]
fn non_trigger_snapshot_records_served_then_held_with_the_same_seq() {
    const STEPS: u64 = 10;
    let (kind6, _all, _inputs) = run_recorded(
        "hold",
        Box::new(SlowSrcEntry::new()),
        Box::new(PollSinkEntry::new()),
        STEPS,
    );
    let expected = vec![
        rec(0, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME, 0),
        rec(1, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME, 0),
        rec(2, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME, 0),
        rec(3, READ_OUTCOME_SERVED, 0, 1),
        rec(4, READ_OUTCOME_HELD, 0, 0),
        rec(5, READ_OUTCOME_HELD, 0, 0),
        rec(6, READ_OUTCOME_SERVED, 1, 1),
        rec(7, READ_OUTCOME_HELD, 1, 0),
        rec(8, READ_OUTCOME_HELD, 1, 0),
        rec(9, READ_OUTCOME_SERVED, 2, 1),
    ];
    assert_eq!(
        kind6, expected,
        "served → held → held replays the SAME seq; pre-delivery is NoFrame"
    );
}

/// (c) `sample(25)` decimation: the per-step producer's frames arrive with
/// wire ts (k+1)·10 ms; the step-k snapshot drains the step-(k-1) frame. The
/// gate accepts at ts 10 (seq 0), 40 (seq 3), 70 (seq 6) and DECIMATES the
/// rest — each Decimated record reports popped > 0 and the LAST-ACCEPTED seq.
#[test]
#[serial]
fn sample_gate_decimation_records_decimated_with_the_last_accepted_seq() {
    const STEPS: u64 = 10;
    let (kind6, _all, _inputs) = run_recorded(
        "samp",
        Box::new(StepSrcEntry::new()),
        Box::new(SampleSinkEntry::new()),
        STEPS,
    );
    let expected = vec![
        rec(0, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME, 0),
        rec(1, READ_OUTCOME_SERVED, 0, 1),
        rec(2, READ_OUTCOME_DECIMATED, 0, 1),
        rec(3, READ_OUTCOME_DECIMATED, 0, 1),
        rec(4, READ_OUTCOME_SERVED, 3, 1),
        rec(5, READ_OUTCOME_DECIMATED, 3, 1),
        rec(6, READ_OUTCOME_DECIMATED, 3, 1),
        rec(7, READ_OUTCOME_SERVED, 6, 1),
        rec(8, READ_OUTCOME_DECIMATED, 6, 1),
        rec(9, READ_OUTCOME_DECIMATED, 6, 1),
    ];
    assert_eq!(
        kind6, expected,
        "decimated frames report the last ACCEPTED seq, never their own"
    );
}

/// (d) A never-delivered input (silent producer — lazy loan never publishes):
/// every fire's snapshot records NoFrame with popped 0 — "no frame ever
/// delivered" is a first-class outcome, not an absence of records.
#[test]
#[serial]
fn never_delivered_input_records_no_frame_every_fire() {
    const STEPS: u64 = 5;
    let (kind6, _all, _inputs) = run_recorded(
        "none",
        Box::new(SilentSrcEntry::new()),
        Box::new(PollSinkEntry::new()),
        STEPS,
    );
    let expected: Vec<ReadRec> = (0..STEPS)
        .map(|k| rec(k, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME, 0))
        .collect();
    assert_eq!(kind6, expected, "silence is recorded, not omitted");
}

/// (e) The LEGACY Separate discipline (forced via the drain-discipline measurement
/// seam): the SAME graph as (a) still reads TWICE per fire — the
/// runtime-owned trigger-drain subscriber's batch drain (`drain_samples` →
/// DrainedBatch) AND the tick body's live `try_view` (Served) — both
/// attributed to input 0, in stage-REGISTRATION order (body first, then the
/// trigger-drain stage). Under per-message FIFO (52125241e) the two reads
/// genuinely DIVERGE in shape: the drain subscriber is NOT FIFO-marked (it is
/// the fire-signal source and drains EVERYTHING — DrainedBatch(newest =
/// 2k+1, popped = 2)), while the EachFifo-marked body pops ONE head per fire
/// (Served, popped = 1).
///
/// Under the WITHIN-STEP BURST that drain's TWO signals are also
/// SERVED within the step. This is the path where the signalled count IS the
/// burst — the boundary drain already minted one arrival per queued frame —
/// so `fire_count` is 2 and the fire loop consults no refill hook at all
/// (unlike arm (a), whose one-frozen-slot drain can only ever signal one).
/// The body therefore pops BOTH heads and step k yields
/// [Served(2k, 1), Served(2k+1, 1), DrainedBatch(2k+1, 2)]. Before SERVE-MANY the
/// scheduler consumed one signal per step and CARRIED the rest, so the body
/// recorded a single Served(k, 1) and the consumer fell progressively further
/// behind its own drain. The two Served records land BEFORE the DrainedBatch
/// although the drain ran FIRST chronologically — the stage-REGISTRATION
/// order pin — and the 2-wide batch is the file's surviving newest-seq pin
/// (a first-drained-wins variant records 2k there).
#[test]
#[serial]
fn separate_discipline_records_both_the_batch_drain_and_the_body_read() {
    let _guard = WaveEnvGuard::set("CERULION_DRAIN_DISCIPLINE", "separate");

    const STEPS: u64 = 4;
    let (kind6, _all, _inputs) = run_recorded(
        "sep",
        Box::new(FastSrcEntry::new()),
        Box::new(TrigSinkEntry::new()),
        STEPS,
    );
    let mut expected = Vec::new();
    for k in 0..STEPS {
        expected.push(rec(k, READ_OUTCOME_SERVED, 2 * k, 1));
        expected.push(rec(k, READ_OUTCOME_SERVED, 2 * k + 1, 1));
        expected.push(rec(k, READ_OUTCOME_DRAINED_BATCH, 2 * k + 1, 2));
    }
    assert_eq!(
        kind6, expected,
        "the Separate path records BOTH reads: one FIFO body pop per FIRE \
         (2k then 2k+1, popped 1) and ONE drain-everything signal batch \
         (newest 2k+1, popped 2)"
    );
}

/// N8, the NO-INERT-SHIPPING proof for the read-site role bits:
/// a REAL graph over REAL iceoryx2 carries **Drain** on the trigger drain's
/// records and **Body** on the node body's, in ONE run.
///
/// Every other role assertion in the repo is against a CRAFTED record — a
/// value a test wrote and then read back, which proves the packing and nothing
/// about the production mint. This arm is the only place the roles are read
/// off a stream the runtime actually produced, so it is what stands between
/// "the bits are stamped" and "the bits are stamped CORRECTLY".
///
/// The `Separate` discipline is the fixture because it is the ONE shape whose
/// step carries both sites at once: the runtime-owned trigger-drain subscriber
/// batch-drains (`try_receive_timestamps` → `try_receive_for_drain` →
/// `ReadSiteRole::Drain`) while the node body FIFO-pops its own subscriber
/// (`try_view` → `ReadSiteRole::Body`). Under `Unified` there is only one
/// subscriber, so a run there cannot separate the two by construction.
///
/// **Reverting `graph/runtime.rs`'s `try_receive_timestamps` from
/// `try_receive_for_drain` to the plain `try_receive` a node body
/// reaches** is the headline case: the drain then stamps `Body`, every
/// record in the run reads `Body`, and the whole drain-vs-body distinction is
/// silently inert while every crafted-record oracle in the repo stays green.
/// This arm fails on it at the `DrainedBatch` row.
///
/// The kind/seq/popped oracle is deliberately re-asserted here beside the role
/// rather than delegated to the sibling arm: a role assertion over a stream
/// whose SHAPE nobody checked in the same body could pass against a run that
/// recorded the wrong records with the right roles.
#[test]
#[serial]
fn the_separate_discipline_stamps_drain_on_the_drain_and_body_on_the_body() {
    let _guard = WaveEnvGuard::set("CERULION_DRAIN_DISCIPLINE", "separate");

    const STEPS: u64 = 4;
    let (kind6, all, _inputs) = run_recorded(
        "roles_sep",
        Box::new(FastSrcEntry::new()),
        Box::new(TrigSinkEntry::new()),
        STEPS,
    );

    // (1) The SHAPE oracle first — the sibling arm's, verbatim — so the role
    // assertion below cannot be satisfied by a run recording the wrong stream.
    let mut shape = Vec::new();
    for k in 0..STEPS {
        shape.push(rec(k, READ_OUTCOME_SERVED, 2 * k, 1));
        shape.push(rec(k, READ_OUTCOME_SERVED, 2 * k + 1, 1));
        shape.push(rec(k, READ_OUTCOME_DRAINED_BATCH, 2 * k + 1, 2));
    }
    assert_eq!(
        kind6, shape,
        "PRECONDITION: the Separate run records the two body pops and the one \
         drain batch this arm reads roles off"
    );

    // (2) The ROLE oracle, hand-written per record.
    let mut expected = Vec::new();
    for k in 0..STEPS {
        // The node body's own FIFO pops, inside its tick.
        expected.push((k, 1u32, 0u16, READ_OUTCOME_SERVED, ReadSiteRole::Body));
        expected.push((k, 1u32, 0u16, READ_OUTCOME_SERVED, ReadSiteRole::Body));
        // The scheduler's trigger drain, on the runtime-owned subscriber.
        expected.push((
            k,
            1u32,
            0u16,
            READ_OUTCOME_DRAINED_BATCH,
            ReadSiteRole::Drain,
        ));
    }
    assert_eq!(
        role_view(&all),
        expected,
        "the drain's batch must carry Drain and the body's pops Body — a run \
         where every record reads Body is `try_receive_timestamps` calling the \
         BODY entry point (the headline mutant), and one where every record \
         reads Unstamped is a mint that never threaded a role at all"
    );
}

/// N8's second half: a per-set **Sync** node's set-member pops
/// are DRAIN-role reads.
///
/// These records are what the replay side leans on hardest: they are exactly
/// the reads whose stamps reached `sync_input_timestamps`, which is what
/// `replay_rederive::verify_sync` folds its alignment map from. A mint that
/// stamped `Body` would leave a stamped bag's fold EMPTY while the unstamped
/// arm kept working — a regression visible nowhere else.
///
/// It shares the sibling arm's fixture (two sources into one Sync sink) rather
/// than inventing one, so the record SHAPE is already pinned there and this
/// arm adds only the role column. Note what the shape says: the align pass pops
/// each member through the node's OWN (unified) body SUBSCRIBER, so these
/// records land in a `Body`-role STAGE — and still carry the `Drain` CALL-SITE
/// role. That divergence is the whole reason the wire role is the call site's
/// (`ReadSiteRole`'s own docs), and this is the run that demonstrates it.
///
/// **SCOPE, MEASURED rather than asserted.**
/// This arm does NOT cover `sync_peek_next_stamp`
/// and `sync_discard_head`. Under the per-set discipline the align
/// pass reaches `drain_for_trigger`, so THAT is the mint this arm pins:
/// flipping `drain_for_trigger`'s three `ReadSiteRole::Drain` mints to `Body`
/// FAILS it (measured, ten records, every one reading `Body`), while flipping
/// the six mints inside `sync_peek_next_stamp` / `sync_discard_head` does NOT —
/// and passes this whole file, the cdylib capture file, and
/// `sync_per_set_iox2_test` as well. Those six used to be covered only
/// INDIRECTLY, and only off this crate: `replay_engine_test`'s
/// `a_format_5_corpus_bag_replays_byte_exact_including_its_report`
/// fails on them through `read_log_divergence`, because a body-role set-member
/// read steers to the body queue and the injection plan moves with it — a real
/// kill, and an unattributable one.
///
/// **That gap is now closed by
/// [`the_sync_matchers_descent_pops_are_drain_site_reads`]**, which
/// builds the queue shape the two functions need (a backlog on one trigger input
/// beside a scarce partner at a DIFFERENT stamp) and reads their records
/// directly. This arm keeps its own scope: `drain_for_trigger`'s mints, on the
/// stimulus every other arm here uses.
#[test]
#[serial]
fn a_per_set_sync_nodes_member_pops_are_drain_site_reads() {
    const STEPS: u64 = 5;
    let config = graph_of(
        "rolesync",
        vec![
            src_def("pa"),
            src_def("pb"),
            sink_def("consumer", &[("a", "pa/out"), ("b", "pb/out")]),
        ],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("pa".to_string(), Box::new(StepSrcEntry::new()));
    factories.insert("pb".to_string(), Box::new(StepSrcEntry::new()));
    factories.insert("consumer".to_string(), Box::new(SyncSinkEntry::new()));
    let (_kind6, all, _inputs) = run_recorded_custom(
        "roles_sync",
        config,
        factories,
        &["pa", "pb", "consumer"],
        &[&[], &[], &["a", "b"]],
        STEPS,
    );

    let mut expected = Vec::new();
    for k in 0..STEPS {
        expected.push((
            k,
            2u32,
            0u16,
            READ_OUTCOME_DRAINED_BATCH,
            ReadSiteRole::Drain,
        ));
        expected.push((
            k,
            2u32,
            1u16,
            READ_OUTCOME_DRAINED_BATCH,
            ReadSiteRole::Drain,
        ));
    }
    assert_eq!(
        role_view(&all),
        expected,
        "every per-set Sync matcher pop is a DRAIN-site read — one per trigger \
         member per set-fire, in registration order, and nothing else"
    );
}

/// TIGHT-window per-set Sync sink (15 ms) — the descent fixture.
///
/// The window is 15 ms against a 10 ms step cadence for ONE reason: it makes
/// a two-step-old head PROVABLY unmatchable, which is the only verdict that
/// reaches `sync_discard_head`'s REFILL arm (an `Advance` always has a staged
/// next to promote, and a promotion records nothing).
#[cerulion_node(sync_window_ms = 15)]
#[derive(Default)]
struct TightSyncSink {
    #[input(trigger)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
}

#[cerulion_node_impl]
impl TightSyncSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.a.x;
        let _ = self.b.x;
        Ok(())
    }
}

/// The absolute topics the ANNOTATED leg's external publishers own — an
/// absolute `source:` with no in-graph producer, which is what makes the edge
/// `PublisherProvisioning::External` and therefore producer-ANNOTATED
/// (`edge_needs_producer_annotation`; the `/tf`-under-`ros2 attach` shape).
const DESCENT_TOPIC_A: &str = "/roledscx/a";
const DESCENT_TOPIC_B: &str = "/roledscx/b";

/// The per-set Sync MATCHER's OWN pops (`sync_peek_next_stamp`
/// and `sync_discard_head`) are DRAIN-site reads.
///
/// # Why this arm exists
///
/// Its sibling above pins the mints inside `drain_for_trigger`, and says so
/// explicitly: flipping the SIX `ReadSiteRole::Drain` mints inside
/// `sync_peek_next_stamp` / `sync_discard_head` to `Body` failed NOTHING in this
/// file, the cdylib capture file, or `sync_per_set_iox2_test` — those six were
/// covered only INDIRECTLY, off this crate, through `replay_engine_test`'s
/// byte-exactness gate (a body-role set-member read steers to the body queue and
/// the injection plan moves with it). That is a real kill and an unattributable
/// one: the failure names a `read_log_divergence`, not a role.
///
/// The two functions are unreachable from every OTHER arm in this file for a
/// structural reason, not a coverage oversight: an in-graph producer publishes
/// inside its own tick, so every frame a step commits carries the SAME wire
/// stamp, and the descent's first act is a strict `lo_count == 1` test
/// (`sync_match::next_sync_step` returns `Fire` on a tie). Reaching it needs a
/// BACKLOG on one trigger input whose head is at a DIFFERENT stamp from its
/// partner's — which is what [`run_recorded_driven`] exists to let an arm build.
///
/// # The two legs, and what each covers
///
/// Both drive the SAME matcher through the SAME two verdicts; they differ in
/// which STAGING HELPER the mints reach, which is a build-time property of the
/// edge and therefore not something one graph can exercise both ways:
///
/// 1. **PLAIN** — graph-owned single-writer topics (every shipped graph), so
///    `capture_producer_token()` is false and both sites take
///    `stage_read_outcome(DrainedBatch, .., Drain)`.
/// 2. **ANNOTATED** — absolute `source:` topics fed by EXTERNAL publishers, so
///    the edge is `PublisherProvisioning::External`, the annotation is on, and
///    both sites take `stage_read_outcome_with_producer(.., Drain)` — which
///    stamps the role onto the `Producer` annotation as well as the read.
///    (`multi_publisher_topics` cannot serve here: the graph build REFUSES a
///    derived name in that list, so the annotated shape is reachable only
///    through an absolute topic, which is external by construction.)
///
/// **SCOPE: four of the six mints, MEASURED.** The remaining two are the
/// `Decimated` arms of the same two functions, which need a `sample(N)` gate on
/// a Sync TRIGGER input so the matcher's own pop comes back decimated. They are
/// not covered here, and a variant that flips all six is caught by the four
/// that are.
///
/// # Attribution — why these records are the matcher's and not the drain's
///
/// Every record in both legs is a `DrainedBatch`, so `(kind, role)` alone cannot
/// say which SITE minted one. What separates them is the matcher's OWN counters,
/// asserted in the same body: `sync_unmatched_discard_count == 1` is the window
/// DEATH that drove `sync_discard_head`'s refill, and
/// `sync_closer_skip_count == 1` is the `Advance` that followed the `NeedStamp`
/// pop `sync_peek_next_stamp` performed. Both are 0 in the CONTROL leg, whose
/// record stream is exactly the boundary drains — so the extra records exist
/// only where the matcher descended.
#[test]
#[serial]
fn the_sync_matchers_descent_pops_are_drain_site_reads() {
    /// A `ReadRec` at an explicit node index (the file's `rec` helper hardcodes
    /// the two-node graphs' consumer at 1).
    ///
    /// `popped` is EXPLICIT since the peek/head mark: a promotion record
    /// carries `popped: 0` (the frame left the queue at its PEEK, which is
    /// where that pop is accounted), so a helper hardcoding `1` could not
    /// express the stream any more — and hardcoding it would have hidden a
    /// double-counted pop, which is exactly what the FIFO and `block` sums read.
    fn rec_at(step: u64, node_idx: u32, input_idx: u16, seq: u64, popped: u32) -> ReadRec {
        ReadRec {
            step,
            node_idx,
            input_idx,
            kind: READ_OUTCOME_DRAINED_BATCH,
            seq,
            popped,
        }
    }

    // =======================================================================
    // LEG 1 — PLAIN mints, on graph-owned single-writer topics.
    //
    // `pa` publishes every step; `pb` every third (steps 2, 5, 8). While `b` is
    // absent the matcher WAITS, so `a`'s head is HELD and the boundary re-offers
    // it without draining (no record) while `a`'s later frames QUEUE — which is
    // the backlog, built by the graph itself.
    //
    // Step 2 is the whole arm: heads are `a`@0 ms (step 0's frame) and `b`@20 ms,
    // a 20 ms span against a 15 ms window ⇒ DEATH on `a`, whose discard has
    // NOTHING staged and therefore refills FROM THE QUEUE — `sync_discard_head`'s
    // own mint (seq 1). The refilled head is 10 ms from `b`, `b` provably has no
    // second frame (the gate), so the descent probes and POPS — `sync_peek_next_stamp`'s
    // mint (seq 2) — and the resulting span of 0 is a strict improvement, so the
    // `Advance` promotes the staged frame and records NOTHING.
    // =======================================================================
    const PLAIN_STEPS: u64 = 4;
    let config = graph_of(
        "roledsc",
        vec![
            src_def("pa"),
            src_def("pb"),
            sink_def("fuse", &[("a", "pa/out"), ("b", "pb/out")]),
        ],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("pa".to_string(), Box::new(StepSrcEntry::new()));
    factories.insert("pb".to_string(), Box::new(SlowSrcEntry::new()));
    factories.insert("fuse".to_string(), Box::new(TightSyncSinkEntry::new()));
    let mut plain_counters = None;
    let (kind6, all, _inputs) = run_recorded_driven(
        "roledsc",
        config,
        factories,
        &["pa", "pb", "fuse"],
        &[&[], &[], &["a", "b"]],
        |runtime, _clock| {
            for _ in 0..PLAIN_STEPS {
                runtime.step(Duration::from_millis(10));
            }
            let h = runtime.node_handle("fuse").expect("node handle");
            plain_counters = Some((
                h.sync_unmatched_discard_count("/roledsc/pa/out"),
                h.sync_closer_skip_count("/roledsc/pa/out"),
                h.sync_unmatched_discard_count("/roledsc/pb/out"),
                h.sync_closer_skip_count("/roledsc/pb/out"),
            ));
        },
    );

    assert_eq!(
        plain_counters,
        Some((1, 1, 0, 0)),
        "ATTRIBUTION: exactly one window DEATH (which drives `sync_discard_head`'s \
         refill) and exactly one `Advance` (which follows `sync_peek_next_stamp`'s \
         pop), both on `a`, and NOTHING on the scarce partner"
    );
    assert_eq!(
        kind6,
        vec![
            // Step 0: `a`'s boundary drain freezes its head. `b` is silent, so
            // the matcher waits and `a`'s head is held from here on.
            rec_at(0, 2, 0, 0, 1),
            // Step 2, the descent, in the order the matcher runs it: the
            // DEATH's refill (seq 1 becomes the head), the NeedStamp pop
            // (seq 2 is PARKED), then the `Advance`'s PROMOTION of that same
            // frame — which now mints its own record at seq 2 with
            // `popped: 0`, because the pop was accounted at the peek.
            // `a`'s BOUNDARY drain records nothing this step — a held head is
            // re-offered, never re-drained.
            rec_at(2, 2, 0, 1, 1),
            rec_at(2, 2, 0, 2, 1),
            rec_at(2, 2, 0, 2, 0),
            // `b`'s boundary drain. Records are merged per INPUT, so this
            // follows `a`'s whatever order the two ran in.
            rec_at(2, 2, 1, 0, 1),
            // Step 3: the set fired, so `a`'s head is EMITTED and the boundary
            // drains a fresh frame. `b` is silent again ⇒ wait.
            rec_at(3, 2, 0, 3, 1),
        ],
        "the matcher's descent rides the read log exactly as the boundary \
         drain's pops do — one record per CONSUMED frame, PLUS one for the \
         hand-off the `Advance` performs, whose `popped: 0` says the head \
         advanced while nothing left the queue"
    );
    assert_eq!(
        role_view(&all),
        vec![
            (0, 2, 0, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Drain),
            (2, 2, 0, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Drain),
            (2, 2, 0, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Peek),
            (2, 2, 0, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Drain),
            (2, 2, 1, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Drain),
            (3, 2, 0, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Drain),
        ],
        "THE PIN: the matcher reads on the node's OWN body subscriber, but the \
         CALL SITE is the scheduler — so `sync_discard_head`'s refill (step 2, \
         seq 1) and its PROMOTION (step 2, seq 2, popped 0) are DRAIN-site \
         records while `sync_peek_next_stamp`'s pop (step 2, seq 2, popped 1) \
         is a PEEK. Flipping the refill or the promotion to `Body` moves its \
         row; flipping the peek to `Drain` re-creates the false `ScheduleFinding` \
         an earlier change removed, because `verify_sync` would then fold its stamp"
    );

    // =======================================================================
    // CONTROL — the SAME fixture with a per-step partner, so no head is ever
    // stale and the matcher never descends. Without it, "the descent's records
    // are Drain" is satisfied by a run whose extra records came from somewhere
    // else entirely.
    // =======================================================================
    const CTRL_STEPS: u64 = 3;
    let config = graph_of(
        "roledscc",
        vec![
            src_def("pa"),
            src_def("pb"),
            sink_def("fuse", &[("a", "pa/out"), ("b", "pb/out")]),
        ],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("pa".to_string(), Box::new(StepSrcEntry::new()));
    factories.insert("pb".to_string(), Box::new(StepSrcEntry::new()));
    factories.insert("fuse".to_string(), Box::new(TightSyncSinkEntry::new()));
    let mut ctrl_counters = None;
    let (ctrl_kind6, _ctrl_all, _inputs) = run_recorded_driven(
        "roledscc",
        config,
        factories,
        &["pa", "pb", "fuse"],
        &[&[], &[], &["a", "b"]],
        |runtime, _clock| {
            for _ in 0..CTRL_STEPS {
                runtime.step(Duration::from_millis(10));
            }
            let h = runtime.node_handle("fuse").expect("node handle");
            ctrl_counters = Some((
                h.sync_unmatched_discard_count("/roledscc/pa/out"),
                h.sync_closer_skip_count("/roledscc/pa/out"),
            ));
        },
    );
    assert_eq!(
        ctrl_counters,
        Some((0, 0)),
        "CONTROL: both heads land at the same stamp every step, so the descent's \
         own `lo_count == 1` test refuses before any probe"
    );
    assert_eq!(
        ctrl_kind6,
        (0..CTRL_STEPS)
            .flat_map(|k| [rec_at(k, 2, 0, k, 1), rec_at(k, 2, 1, k, 1)])
            .collect::<Vec<_>>(),
        "CONTROL: with no descent the stream is exactly the two boundary drains \
         per step — the extra records above exist only where the matcher popped"
    );

    // =======================================================================
    // LEG 2 — ANNOTATED mints, on absolute topics fed by EXTERNAL publishers.
    //
    // The stimulus is hand-stamped rather than graph-driven (that is the point
    // of an external publisher): `a` = [0, 20, 30] ms and `b` = [55] ms against
    // a 50 ms window. The 55 ms span kills `a`@0, whose refill takes
    // `sync_discard_head`'s mint; the descent from `a`@20 then takes
    // `sync_peek_next_stamp`'s. One step, so every record is step 0.
    // =======================================================================
    let config = graph_of(
        "roledscx",
        vec![sink_def(
            "fuse",
            &[("a", DESCENT_TOPIC_A), ("b", DESCENT_TOPIC_B)],
        )],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), Box::new(SyncSinkEntry::new()));
    let mut annot_counters = None;
    let (annot_kind6, annot_all, _inputs) = run_recorded_driven(
        "roledscx",
        config,
        factories,
        &["fuse"],
        &[&["a", "b"]],
        |runtime, clock| {
            let mgr = Arc::clone(runtime.test_transport().expect("test transport parked"));
            let mut pub_a = mgr
                .create_publisher(DESCENT_TOPIC_A, MaxSliceLen::const_new(64), 0)
                .expect("external publisher on a");
            let mut pub_b = mgr
                .create_publisher(DESCENT_TOPIC_B, MaxSliceLen::const_new(64), 0)
                .expect("external publisher on b");
            for at_ms in [0u64, 20, 30] {
                clock.set(at_ms * 1_000_000);
                let mut p = pub_a.loan_proxy::<Vector3>().expect("loan a");
                p.x = at_ms as f64;
            }
            clock.set(55 * 1_000_000);
            {
                let mut p = pub_b.loan_proxy::<Vector3>().expect("loan b");
                p.x = 55.0;
            }
            clock.set(0);
            runtime.step(Duration::from_millis(1));
            let h = runtime.node_handle("fuse").expect("node handle");
            annot_counters = Some((
                h.sync_unmatched_discard_count(DESCENT_TOPIC_A),
                h.sync_closer_skip_count(DESCENT_TOPIC_A),
                h.sync_unmatched_discard_count(DESCENT_TOPIC_B),
                h.sync_closer_skip_count(DESCENT_TOPIC_B),
            ));
        },
    );
    assert_eq!(
        annot_counters,
        Some((1, 1, 0, 0)),
        "ATTRIBUTION, same as leg 1: one DEATH and one `Advance` on `a`, nothing \
         on the scarce partner"
    );
    assert_eq!(
        annot_kind6
            .iter()
            .filter(|r| r.kind == READ_OUTCOME_DRAINED_BATCH)
            .copied()
            .collect::<Vec<_>>(),
        vec![
            // `a`: the boundary drain, then the DEATH's refill, then the
            // NeedStamp pop — three CONSUMED frames, three records — and then
            // the `Advance`'s PROMOTION of the peeked frame, whose `popped: 0`
            // adds no fourth pop.
            rec_at(0, 0, 0, 0, 1),
            rec_at(0, 0, 0, 1, 1),
            rec_at(0, 0, 0, 2, 1),
            rec_at(0, 0, 0, 2, 0),
            // `b`: its boundary drain, and nothing else.
            rec_at(0, 0, 1, 0, 1),
        ],
        "the annotated edge consumes the same frames as the plain one — the \
         annotation adds a record, it does not change what a read pops"
    );
    assert_eq!(
        role_view(&annot_all),
        vec![
            (0, 0, 0, READ_OUTCOME_PRODUCER, ReadSiteRole::Drain),
            (0, 0, 0, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Drain),
            (0, 0, 0, READ_OUTCOME_PRODUCER, ReadSiteRole::Drain),
            (0, 0, 0, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Drain),
            (0, 0, 0, READ_OUTCOME_PRODUCER, ReadSiteRole::Peek),
            (0, 0, 0, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Peek),
            (0, 0, 0, READ_OUTCOME_PRODUCER, ReadSiteRole::Drain),
            (0, 0, 0, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Drain),
            (0, 0, 1, READ_OUTCOME_PRODUCER, ReadSiteRole::Drain),
            (0, 0, 1, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Drain),
        ],
        "THE SECOND PIN: on an EXTERNAL (absolute-source) edge every matcher site \
         takes `stage_read_outcome_with_producer`, which stamps the call-site role \
         onto the `Producer` annotation as well as the read — so the peek's PAIR \
         carries `Peek` on BOTH rows, and `Peek` on a `Producer` is therefore a \
         PRODUCIBLE shape (which is why `unproducible_read_shape` exempts it)"
    );
}

/// The absolute topics the NON-ADVANCING-PEEK arm's external publishers own.
const PEEK_TOPIC_A: &str = "/rolepk/a";
const PEEK_TOPIC_B: &str = "/rolepk/b";

/// **THE NON-ADVANCING PEEK: the shape that produced a
/// FALSE `ScheduleFinding`, and the one the `Peek` role exists for.**
///
/// # Why this arm and not the descent arm above
///
/// Its sibling drives a descent that DISCARDS — a window death, a refill, a
/// peek, an `Advance`, a promotion — so its peeked frame becomes the head one
/// op later and the fold would have landed on it anyway. The shape that BREAKS
/// the fold is the ORDINARY one: the matcher looks ahead, the look does not
/// improve the set, and it fires on the head it already had. The peeked frame
/// stays PARKED in `next_head`, never aligned on, while its record is the LAST
/// one of the step — so `verify_sync`'s last-write-wins fold held a stamp the
/// scheduler never had, and on a stamp far enough out it re-derived "0 fires"
/// against a recorded 1. That is a `ScheduleFinding`, which becomes a trace
/// divergence and owns an exit code; it is the one place in the read-log
/// machinery where being wrong costs a verdict rather than a note.
///
/// # The stimulus, and why every number in it is load-bearing
///
/// External publishers on absolute topics, so the stamps are HAND-SET rather
/// than whatever an in-graph producer's tick happened to mint (an in-graph
/// producer stamps every frame of a step identically, which the matcher's
/// `lo_count == 1` test refuses before any probe — the structural reason the
/// descent is unreachable from an ordinary graph fixture).
///
/// * `a` = [0 ms, 100 ms], `b` = [40 ms], window 50 ms.
/// * Heads land at `a`@0 / `b`@40: span 40, INSIDE the window, so the pass is
///   alive and no `DiscardTie` runs (that is what the death check would do at
///   51+, and it is why the sibling arm's 55 ms is not reused here).
/// * `a` is the unique argmin, `a` HAS a second arrived frame and `b` has NONE,
///   so the gate passes and the matcher asks for the next stamp — the ONLY way
///   `sync_peek_next_stamp` is reached.
/// * The peek pops `a`@100. The descent test is STRICT: advancing would leave a
///   span of `|100 - 40|` = 60, WORSE than the 40 it has, so the verdict is
///   `Fire` and the frame stays parked. 100 ms is chosen for that inequality
///   and nothing else — anything under 80 ms would improve the span and turn
///   this into the sibling arm.
///
/// # What is asserted, and how each claim is attributed
///
/// The record stream IS the property: `a` carries a `Drain` record at seq 0 —
/// the head, and the LAST head-naming record of the step, which is exactly what
/// `verify_sync` folds — followed by a `Peek` record at seq 1, the frame that
/// was only looked at. The matcher's own counters are the attribution that no
/// OTHER op ran: `sync_unmatched_discard_count == 0` (no window death, so the
/// seq-0 record is the boundary drain's and not a refill's) and
/// `sync_closer_skip_count == 0` (no `Advance`, so seq 1 was never promoted and
/// the `Peek` record is the whole of what the descent left behind).
#[test]
#[serial]
fn a_non_improving_peek_is_marked_a_peek_not_the_head() {
    fn rec_at(step: u64, node_idx: u32, input_idx: u16, seq: u64, popped: u32) -> ReadRec {
        ReadRec {
            step,
            node_idx,
            input_idx,
            kind: READ_OUTCOME_DRAINED_BATCH,
            seq,
            popped,
        }
    }

    let config = graph_of(
        "rolepk",
        vec![sink_def(
            "fuse",
            &[("a", PEEK_TOPIC_A), ("b", PEEK_TOPIC_B)],
        )],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), Box::new(SyncSinkEntry::new()));
    let mut counters = None;
    let (kind6, all, _inputs) = run_recorded_driven(
        "rolepk",
        config,
        factories,
        &["fuse"],
        &[&["a", "b"]],
        |runtime, clock| {
            let mgr = Arc::clone(runtime.test_transport().expect("test transport parked"));
            let mut pub_a = mgr
                .create_publisher(PEEK_TOPIC_A, MaxSliceLen::const_new(64), 0)
                .expect("external publisher on a");
            let mut pub_b = mgr
                .create_publisher(PEEK_TOPIC_B, MaxSliceLen::const_new(64), 0)
                .expect("external publisher on b");
            for at_ms in [0u64, 100] {
                clock.set(at_ms * 1_000_000);
                let mut p = pub_a.loan_proxy::<Vector3>().expect("loan a");
                p.x = at_ms as f64;
            }
            clock.set(40 * 1_000_000);
            {
                let mut p = pub_b.loan_proxy::<Vector3>().expect("loan b");
                p.x = 40.0;
            }
            clock.set(0);
            runtime.step(Duration::from_millis(1));
            let h = runtime.node_handle("fuse").expect("node handle");
            counters = Some((
                h.sync_unmatched_discard_count(PEEK_TOPIC_A),
                h.sync_closer_skip_count(PEEK_TOPIC_A),
            ));
        },
    );

    assert_eq!(
        counters,
        Some((0, 0)),
        "ATTRIBUTION: NEITHER a window death NOR an `Advance` ran, so the seq-0 \
         record is the boundary drain's own and seq 1 was never promoted — the \
         peek is the whole of what the descent left behind"
    );
    assert_eq!(
        kind6
            .iter()
            .filter(|r| r.kind == READ_OUTCOME_DRAINED_BATCH)
            .copied()
            .collect::<Vec<_>>(),
        vec![
            // `a`: the boundary drain freezes seq 0 as the head, then the
            // descent POPS seq 1 to read its stamp and parks it.
            rec_at(0, 0, 0, 0, 1),
            rec_at(0, 0, 0, 1, 1),
            // `b`: its boundary drain, and nothing else — it had no second
            // frame, which is what let the gate pass in the first place.
            rec_at(0, 0, 1, 0, 1),
        ],
        "both frames really left `a`'s queue, so both are recorded — the peek is \
         not a silent read"
    );
    assert_eq!(
        role_view(&all),
        vec![
            (0, 0, 0, READ_OUTCOME_PRODUCER, ReadSiteRole::Drain),
            (0, 0, 0, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Drain),
            (0, 0, 0, READ_OUTCOME_PRODUCER, ReadSiteRole::Peek),
            (0, 0, 0, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Peek),
            (0, 0, 1, READ_OUTCOME_PRODUCER, ReadSiteRole::Drain),
            (0, 0, 1, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Drain),
        ],
        "THE PIN: the LAST `Drain`-role record on `a` is seq 0 — the head the \
         matcher fired on — and seq 1 is a `Peek`. Mint the peek as `Drain` \
         and the last head-naming record becomes seq 1, which is the frame the \
         matcher looked at and refused: that is the false `ScheduleFinding`, \
         reproduced at the wire"
    );
}

/// (f) Recording OFF: the same graph built WITHOUT a trace ring never arms a
/// stage — the capture plane is structurally inert (stages record nothing
/// while disarmed; pinned at the unit level by
/// `read_outcome::tests::disarmed_stage_records_nothing`). The armed count
/// here is the observable; its `> 0` twin lives in `run_recorded` (every ON
/// arm asserts it), so this cannot pass vacuously.
#[test]
#[serial]
fn recording_off_arms_no_stage() {
    let (config, factories) = two_node_graph(
        "rooff",
        Box::new(FastSrcEntry::new()),
        Box::new(TrigSinkEntry::new()),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build graph");
    assert_eq!(
        runtime.read_outcome_stages_armed_for_test(),
        0,
        "no recording ⇒ no stage armed at build"
    );
    for _ in 0..4 {
        runtime.step(Duration::from_millis(10));
    }
    assert_eq!(
        runtime.read_outcome_stages_armed_for_test(),
        0,
        "stepping never arms capture — only set_trace_ring_producer does"
    );
    runtime.shutdown();
}

/// (g) The Period CATCH-UP contract under SERVE-MANY. A `Period(5)`
/// consumer driven at 10 ms steps fires TWICE per step; per step k the read log
/// carries EXACTLY ONE record for its one plain input — the step-boundary
/// SNAPSHOT — because BOTH catch-up fires serve that one frozen slot and a
/// serve records nothing:
///
/// * step 0 — the same-level producer has not ticked yet when the snapshot
///   runs, and nothing has ever been delivered, so the snapshot correctly
///   records NoFrame(popped 0);
/// * step k ≥ 1 — the snapshot pops the producer's step-(k-1) publish and
///   records Served(seq k-1, popped 1). Both fires of step k then read those
///   same bytes off the frozen slot.
///
/// # This SUPERSEDES the earlier serve-once catch-up contract
///
/// That contract read `NoFrame(0) + Served(seq k, popped 1)` per step, and it
/// was downstream of serve-ONCE mechanics rather than of anything about Period
/// catch-up: fire 1 consumed the frozen slot, fire 2 found it empty and FELL TO
/// THE LIVE DRAIN, which on this shape really did pop — the same-level
/// producer's SAME-step publish (declaration order runs it first on the serial
/// narrow-level path). Every step's queue was therefore empty by the next
/// boundary, which is why the snapshot recorded NoFrame every time. SERVE-MANY
/// removes that mid-step live read, so the frame stays queued for the next
/// boundary and the roles swap: the snapshot pops, the fires serve.
///
/// Period catch-up consequently now honours the PRIOR-VALUE rule that already
/// governed every other same-level non-trigger edge (`snapshot_wiring_iox2_test`):
/// one step of added context latency, against a fire-2 read that used to see a
/// same-step publish its own fire 1 could not — the accepted trade.
///
/// # What is UNCHANGED, and still pinned here
///
/// * ACCOUNTING-ONCE — one record per CONSUMED frame, never one per fire. Two
///   fires read the slot and exactly one record exists for them, so a serve
///   that re-drained (or that recorded) would add a second record per step and
///   fail the length compare.
/// * NO DOUBLE-RECORDING SNAPSHOT — a snapshot that staged twice would likewise
///   fail the exact-sequence compare.
///
/// # Frame conservation (the accounting cross-check)
///
/// `StepSrc` is `Period(10)` on 10 ms steps ⇒ exactly `STEPS` frames published,
/// sequences gap-free at commit. The record stream accounts for every
/// one of them: seqs `0..=STEPS-2` are each served exactly once with popped 1,
/// and the LAST frame (published during step `STEPS-1`) is still queued when the
/// run ends — the one in-flight frame the next boundary would pop, which is the
/// one-step latency above, not a loss. Nothing is evicted (a `popped` above 1
/// would mean the drain discarded a frame; every record here reads 1).
#[test]
#[serial]
fn period_catchup_every_fire_serves_the_one_frozen_snapshot_of_its_step() {
    const STEPS: u64 = 6;
    let (kind6, _all, _inputs) = run_recorded(
        "catch",
        Box::new(StepSrcEntry::new()),
        Box::new(CatchupSinkEntry::new()),
        STEPS,
    );
    let mut expected = vec![rec(0, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME, 0)];
    for k in 1..STEPS {
        expected.push(rec(k, READ_OUTCOME_SERVED, k - 1, 1));
    }
    assert_eq!(
        kind6, expected,
        "catch-up: ONE snapshot record per step, serving the PRIOR step's frame \
         to both fires"
    );
    // Frame conservation, read off the same stream: STEPS frames published,
    // STEPS-1 popped-and-served, 1 still queued at run end, 0 discarded.
    let popped_total: u64 = kind6.iter().map(|r| u64::from(r.popped)).sum();
    assert_eq!(
        popped_total,
        STEPS - 1,
        "every published frame but the last in-flight one is popped exactly \
         once; a drain that discarded frames would report popped > 1 somewhere"
    );
}

/// (h) the input_idx > 0 pin: a consumer with TWO plain
/// inputs declared `first` then `second` stages DISTINCT input indices (0/1,
/// the wiring order), and each resolves through the manifest input table to
/// its own name. The two producers run at DIFFERENT rates (per-step vs
/// Period-30) so the two edges' record streams are value-distinct — a
/// `ReadOutcomeStage::new(0)` hardcode (a position-lookup variant) collapses
/// both onto idx 0 and fails the full-sequence oracle.
#[test]
#[serial]
fn two_plain_inputs_carry_distinct_input_idx_resolving_to_their_names() {
    const STEPS: u64 = 6;
    let config = graph_of(
        "rodual",
        vec![
            src_def("pfast"),
            src_def("pslow"),
            sink_def(
                "consumer",
                &[("first", "pfast/out"), ("second", "pslow/out")],
            ),
        ],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("pfast".to_string(), Box::new(StepSrcEntry::new()));
    factories.insert("pslow".to_string(), Box::new(SlowSrcEntry::new()));
    factories.insert("consumer".to_string(), Box::new(DualPollSinkEntry::new()));
    let (kind6, _all, input_names) = run_recorded_custom(
        "dual",
        config,
        factories,
        &["pfast", "pslow", "consumer"],
        &[&[], &[], &["first", "second"]],
        STEPS,
    );

    // The manifest resolves each staged index to ITS OWN name.
    assert_eq!(
        input_names,
        vec![
            vec![],
            vec![],
            vec!["first".to_string(), "second".to_string()]
        ],
        "manifest input table (index == wiring order)"
    );
    // Both plain edges snapshot per step: `first` (per-step producer) reads
    // the PRIOR step's frame from step 1 on; `second` (Period-30 producer,
    // fires steps 2/5) reads NoFrame ×3 → Served(0) → Held(0) ×2 — the
    // value-distinct streams that make a crossed index fail. Merge order
    // within the node is stage-registration (= wiring) order.
    let c = |step: u64, input_idx: u16, kind: u16, seq: u64, popped: u32| ReadRec {
        step,
        node_idx: 2,
        input_idx,
        kind,
        seq,
        popped,
    };
    let expected = vec![
        c(0, 0, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME, 0),
        c(0, 1, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME, 0),
        c(1, 0, READ_OUTCOME_SERVED, 0, 1),
        c(1, 1, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME, 0),
        c(2, 0, READ_OUTCOME_SERVED, 1, 1),
        c(2, 1, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME, 0),
        c(3, 0, READ_OUTCOME_SERVED, 2, 1),
        c(3, 1, READ_OUTCOME_SERVED, 0, 1),
        c(4, 0, READ_OUTCOME_SERVED, 3, 1),
        c(4, 1, READ_OUTCOME_HELD, 0, 0),
        c(5, 0, READ_OUTCOME_SERVED, 4, 1),
        c(5, 1, READ_OUTCOME_HELD, 0, 0),
    ];
    assert_eq!(
        kind6, expected,
        "two inputs stage DISTINCT wiring-order indices with their own streams"
    );
    // Belt-and-braces: idx 1 really occurs (the exact case a position-lookup
    // regression would miss),
    // and the manifest resolves it to "second".
    assert!(
        kind6.iter().any(|r| r.input_idx == 1),
        "input_idx 1 must appear"
    );
    assert_eq!(input_names[2][1], "second");
}

/// (i) the BLOCK-partition merge pin: a `block`-policy
/// consumer (and its producer) land in `LevelPlan::block_ids`, so its kind-6
/// records reach the ring ONLY through `merge_read_outcomes`' block half.
/// Dropping `.chain(block_ids.iter())` from the merge loses
/// every record here (the non-block arms all stay green).
///
/// Oracle: a `block` input is EXCLUDED from the step-boundary snapshot (the
/// snapshot-wiring contract — it reads LIVE in the body), and the plain edge
/// shares a level with the producer, whose declaration-order tick runs FIRST —
/// so the body's live drain at step k serves THAT step's frame: Served(k, 1)
/// every step, step 0 included.
#[test]
#[serial]
fn block_partition_consumer_records_reach_the_ring_through_the_block_half() {
    const STEPS: u64 = 6;
    let (kind6, _all, _inputs) = run_recorded(
        "blk",
        Box::new(StepSrcEntry::new()),
        Box::new(BlockSinkEntry::new()),
        STEPS,
    );
    let expected: Vec<ReadRec> = (0..STEPS)
        .map(|k| rec(k, READ_OUTCOME_SERVED, k, 1))
        .collect();
    assert_eq!(
        kind6, expected,
        "a block-partition node's reads must reach the ring (the merge's block half)"
    );
}

/// (j) REWRITTEN for per-message FIFO (52125241e) —
/// DEFERRED-FIRE boundaries: a `throttle_ms = 25` data-trigger consumer on
/// 10 ms steps. Under FIFO the boundary drain pops a head ONLY when no
/// unserved head is frozen; while a deferred fire holds the head, the
/// boundary RE-OFFERS it (re-minting the fire signal) and records NOTHING —
/// one kind-6 record per CONSUMED frame, never one per re-offer (a throttled
/// defer must not mint duplicate records for one frame). Hand trace
/// (`StepSrc` seq k at step k, t = (k+1)·10 ms; first fire unthrottled):
///
///   step 0 (t=10): pop seq 0 → DB(0,1); FIRE (serves head 0).
///   step 1 (t=20): pop seq 1 → DB(1,1); fire DEFERRED (10 < 25) — a drain
///                  record with NO same-step fire (the level-end-merge
///                  rationale, still pinned).
///   step 2 (t=30): head 1 held → re-offer, NO record; deferred (20 < 25).
///   step 3 (t=40): head 1 held → re-offer, NO record; FIRE (30 ≥ 25,
///                  serves head 1).
///   step 4 (t=50): pop seq 2 → DB(2,1); deferred (10 < 25).
///   step 5 (t=60): head 2 held → re-offer, NO record; deferred (20 < 25).
///
/// The exact kind-6 stream + the exact fire-step set are the pins: a
/// pre-52125241e boundary that drained every step would add records at steps
/// 2/3/5 (and lose the unserved head — the [1, 5]-of-6 overwrite bug), and a
/// fire-scoped merge would strand step 1/4's records or restamp them.
#[test]
#[serial]
fn throttled_consumer_drain_records_at_the_drain_step_with_no_same_step_fire() {
    const STEPS: u64 = 6;
    let (kind6, all, _inputs) = run_recorded(
        "thr",
        Box::new(StepSrcEntry::new()),
        Box::new(ThrottleTrigSinkEntry::new()),
        STEPS,
    );
    // Seq 2 is popped at step 3 by the burst loop's REFILL,
    // not at step 4 by the boundary — and that moved when the re-offer mint gate
    // landed, not when this arm was written.
    //
    // `tick_data_burst` only reaches its refill when the SIGNALLED remainder is
    // exhausted (`if node.pending_data_count > 0 { break; }`). Before the gate,
    // every boundary re-offer of the held head minted a fresh arrival, so on a
    // throttled node the count never returned to 0 and the refill was
    // unreachable: the queue advanced one frame per BOUNDARY. With the gate a
    // re-offer mints nothing, the count reaches 0 after the fire, and the refill
    // pops the next frame and freezes it for the following fire — which is
    // exactly the `refilled_unfired` / `data_backlog_hint` state built and
    // documented, previously masked on this path by the inflation.
    //
    // The record is truthful either way (it dates the pop), and DELIVERY is
    // byte-identical: the frozen frame is read by the step-4 fire in both
    // designs, and the fire set below is unchanged. What moves is one pop step.
    //
    // The arm's own rationale — a DrainedBatch on a step with NO same-step fire,
    // i.e. the level-end-merge rather than fire-scoped staging — is still pinned,
    // by step 1 (fires are {0, 3}). Step 3's record is now a refill pop DURING a
    // fire, so it no longer contributes to that half.
    let expected = vec![
        rec(0, READ_OUTCOME_DRAINED_BATCH, 0, 1),
        rec(1, READ_OUTCOME_DRAINED_BATCH, 1, 1),
        rec(3, READ_OUTCOME_DRAINED_BATCH, 2, 1),
    ];
    assert_eq!(
        kind6, expected,
        "one record per CONSUMED head at its pop step; held-head re-offer \
         steps (2, 4, 5) record NOTHING"
    );
    // The deterministic fire set: steps 0 and 3 (VirtualClock — hand trace
    // above), UNCHANGED by the mint gate. Step 1 carries a DrainedBatch record
    // with NO same-step fire, which is the level-end-merge (vs fire-scoped)
    // rationale's pin — and asserting the fire set here is what makes that
    // claim checkable rather than assumed.
    let fire_steps: std::collections::BTreeSet<u64> = all
        .iter()
        .filter(|r| r.record_type == cerulion_core::trace_ring::RECORD_TYPE_FIRE && r.node_idx == 1)
        .map(|r| r.step)
        .collect();
    assert_eq!(
        fire_steps,
        std::collections::BTreeSet::from([0, 3]),
        "throttle defers per the hand trace"
    );
}

/// (k) the `sample(N)` TRIGGER input (arm (c) covers the
/// NON-trigger snapshot path). MEASURED routing: a backpressure-gated trigger
/// input is NOT unified (`unifies_data_trigger` excludes it), so it rides the
/// SEPARATE discipline — TWO stages per step, in registration order:
///
/// 1. the BODY subscriber's live `try_view` read, where the `sample(25)` gate
///    actually decimates: the trigger edge levelizes the consumer below the
///    per-step producer, the step-k read sees the step-k frame (wire ts
///    (k+1)·10 ms), the gate ACCEPTS ts 10/40/70 (seqs 0/3/6 → Served) and
///    DECIMATES the rest (Decimated, reporting the LAST-ACCEPTED seq);
/// 2. the runtime-owned trigger-DRAIN subscriber's batch drain, which is not
///    sample-gated (it is the fire signal): DrainedBatch(seq k, popped 1)
///    every step — which is also why the node FIRES (and body-reads) every
///    step.
#[test]
#[serial]
fn sample_gated_trigger_records_decimated_via_the_separate_body_read() {
    const STEPS: u64 = 9;
    let (kind6, _all, _inputs) = run_recorded(
        "strg",
        Box::new(StepSrcEntry::new()),
        Box::new(SampleTrigSinkEntry::new()),
        STEPS,
    );
    // Per step: [body Served/Decimated, drain DrainedBatch(k, 1)].
    let mut expected = Vec::new();
    for k in 0..STEPS {
        match k % 3 {
            0 => expected.push(rec(k, READ_OUTCOME_SERVED, k, 1)), // accepted (ts 10+30·(k/3))
            _ => expected.push(rec(k, READ_OUTCOME_DECIMATED, k - (k % 3), 1)),
            // ^ decimated: reports the last ACCEPTED seq (k rounded down to
            //   the accept), never its own dropped frame's.
        }
        expected.push(rec(k, READ_OUTCOME_DRAINED_BATCH, k, 1));
    }
    assert_eq!(
        kind6, expected,
        "a sample-gated TRIGGER rides the Separate discipline: gated body reads \
         (Served/Decimated with the last-accepted seq) + ungated drain batches"
    );
}

/// (l) as per-set delivery left it — the PER-SET Sync drain path:
/// ONE `DrainedBatch` per trigger MEMBER per set-fire, and the body serve
/// records NOTHING.
///
/// Before per-set delivery this node ran the SEPARATE discipline and recorded FOUR
/// records per step: the runtime-owned Sync drain subscribers' `drain_samples`
/// (→ DrainedBatch) AND a live body `try_view` (→ Served). Per-set delivery
/// deleted that dedicated drain subscriber's DATA receiver and UNIFIED each
/// trigger input onto the node's own body subscriber (exactly as the
/// data-trigger one was unified), so the align pass's pop is the read, and
/// `try_view` then serves the FROZEN slot, which by the accounting-once
/// contract records nothing (`subscriber.rs`: "serving a FROZEN slot records
/// NOTHING — the read outcome was already staged when the snapshot/trigger
/// drain ran"). The shape is now byte-for-byte the one
/// `unified_trigger_drain_records_one_drained_batch_per_fire_in_arrival_order`
/// already asserts for a unified data-trigger input, which is the point: a
/// Sync trigger input IS per-message FIFO now.
///
/// Nothing observable was lost — the OPPOSITE. The two old records carried the
/// same `seq` only while the two subscribers happened to agree; the whole
/// defect per-set fixes is that the drain sub "read a DIFFERENT frame than the
/// tick did", so the pair could legitimately disagree about which frame the
/// node saw. One record per CONSUMED frame names the set's member exactly.
///
/// Two records per step here, in stage-REGISTRATION order (input wiring order),
/// every one carrying that step's seq (the trigger edges levelize the consumer
/// below both producers) — one frame per input per step is one COMPLETE SET per
/// step, so per-set fires once per step and consumes exactly one member each.
#[test]
#[serial]
fn sync_per_set_records_one_drained_batch_per_member_and_the_frozen_serve_records_nothing() {
    const STEPS: u64 = 5;
    let config = graph_of(
        "rosync",
        vec![
            src_def("pa"),
            src_def("pb"),
            sink_def("consumer", &[("a", "pa/out"), ("b", "pb/out")]),
        ],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("pa".to_string(), Box::new(StepSrcEntry::new()));
    factories.insert("pb".to_string(), Box::new(StepSrcEntry::new()));
    factories.insert("consumer".to_string(), Box::new(SyncSinkEntry::new()));
    let (kind6, _all, _inputs) = run_recorded_custom(
        "sync",
        config,
        factories,
        &["pa", "pb", "consumer"],
        &[&[], &[], &["a", "b"]],
        STEPS,
    );
    let c = |step: u64, input_idx: u16, kind: u16, seq: u64, popped: u32| ReadRec {
        step,
        node_idx: 2,
        input_idx,
        kind,
        seq,
        popped,
    };
    let mut expected = Vec::new();
    for k in 0..STEPS {
        // The align pass pops each member through the node's OWN (unified) body
        // subscriber; the tick's `try_view` then serves that frozen slot and
        // stages nothing.
        expected.push(c(k, 0, READ_OUTCOME_DRAINED_BATCH, k, 1)); // set member a
        expected.push(c(k, 1, READ_OUTCOME_DRAINED_BATCH, k, 1)); // set member b
    }
    // `kind6` is the WHOLE kind-6 stream (only the record type is filtered),
    // so this exact-vector compare is ALSO the no-second-read guard: a `Served`
    // record — the body read popping the QUEUE again instead of serving the
    // member the matcher chose, i.e. the dual-subscriber double-read and the
    // frame it could disagree on — would appear as an extra element and fail
    // here. The `all(kind == DrainedBatch)` assertion that used to follow was
    // deleted rather than kept: every element of `expected` carries that kind,
    // so it was implied by this line and could not fail on its own — a branch
    // no test can kill.
    assert_eq!(
        kind6, expected,
        "per-set Sync: ONE DrainedBatch per trigger member per set-fire, in \
         registration order, and NOTHING else — the frozen serve records \
         nothing, so any extra record here is a second read of a member"
    );
}

/// The record-only-arm observables: (kind-6 records, delivered relay frames,
/// fire tuples `(step, node_id, fire_time_ns)`).
type RelayCapture = (Vec<ReadRec>, Vec<Vec<u8>>, Vec<(u64, String, u64)>);

/// The relay-graph builder for the record-only arms: CountSrc "producer" →
/// TrigRelay "consumer" (output `out`), plus a host DataOnly tap on the
/// relay's out topic. Returns (kind-6 records, delivered relay frames, the
/// consumer-relevant fire tuples) for `steps` × 10 ms; `record` toggles the
/// trace ring (recording ON/OFF — the A/B seam).
fn run_relay_capture(tag: &str, record: bool, steps: u64) -> RelayCapture {
    let relay_out = OutputDef {
        name: "out".to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    };
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "read_outcome_capture".to_string(),
        prefix: format!("rorly{tag}"),
        nodes: vec![
            src_def("producer"),
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "ro_relay".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![relay_out.clone()],
            },
        ],
    };
    let relay_topic =
        cerulion_core::graph::resolve_output_topic(&config.prefix, "consumer", &relay_out);
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(CountSrcEntry::new()));
    factories.insert("consumer".to_string(), Box::new(TrigRelayEntry::new()));

    let node_ids: Vec<String> = vec!["producer".to_string(), "consumer".to_string()];
    let mut owner_and_name: Option<(TraceRingOwner, String)> = None;
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build graph");
    if record {
        let ring_tag = format!("{tag}_{}", std::process::id());
        let mut owner = TraceRingOwner::create_with_inputs(
            &ring_tag,
            default_capacity_records(),
            0,
            &["producer", "consumer"],
            &[&[], &["inp"]],
        )
        .expect("create ring");
        let ring_name = owner.name().to_string();
        let producer_handle = owner.producer().expect("mint producer");
        runtime.set_trace_ring_producer(producer_handle, &node_ids);
        assert!(runtime.read_outcome_stages_armed_for_test() > 0);
        owner_and_name = Some((owner, ring_name));
    } else {
        assert_eq!(
            runtime.read_outcome_stages_armed_for_test(),
            0,
            "recording OFF must arm nothing"
        );
    }

    // Consumer-first: the host tap opens BEFORE step 0 so frame 0 is captured.
    let mgr = runtime
        .test_transport()
        .expect("test transport parked")
        .clone();
    let mut tap = mgr
        .create_data_only_subscriber(&relay_topic)
        .expect("open relay tap");
    let mut delivered: Vec<Vec<u8>> = Vec::new();
    for _ in 0..steps {
        runtime.step(Duration::from_millis(10));
        // Drain the tap each step (bounded queue — nothing may overflow).
        let budget = tap.max_borrowed_samples().max(1);
        let mut scratch = Vec::new();
        loop {
            scratch.clear();
            let n = tap.drain_owned(budget, &mut scratch).expect("tap drain");
            for s in scratch.drain(..) {
                delivered.push(s.payload().to_vec());
            }
            if n < budget {
                break;
            }
        }
    }
    let fires: Vec<(u64, String, u64)> = runtime
        .trace()
        .iter()
        .map(|e| (e.step, e.node_id.to_string(), e.fire_time_ns))
        .collect();

    let kind6: Vec<ReadRec> = if let Some((owner, ring_name)) = owner_and_name {
        let mut ring_consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
        let mut records = Vec::new();
        ring_consumer.drain(&mut records).expect("drain ring");
        drop(ring_consumer);
        runtime.shutdown();
        drop(owner);
        records
            .iter()
            .filter(|r| r.record_type == RECORD_TYPE_READ_OUTCOME)
            .map(|r| {
                let (input_idx, kind) = unpack_read_outcome_meta(r.global_level);
                ReadRec {
                    step: r.step,
                    node_idx: r.node_idx,
                    input_idx,
                    kind,
                    seq: r.fire_time_ns,
                    popped: unpack_read_outcome_popped(r.duration_ns),
                }
            })
            .collect()
    } else {
        runtime.shutdown();
        Vec::new()
    };
    (kind6, delivered, fires)
}

/// The `x` field of a `Vector3` wire frame (32-byte header + fixed section).
fn frame_x(frame: &[u8]) -> f64 {
    f64::from_le_bytes(frame[32..40].try_into().expect("frame carries x"))
}

/// (m) ARMED CAPTURE IS RECORD-ONLY, the delivery half: with
/// recording ON, the relay's DELIVERED value stream matches the SAME hand
/// oracle as the kind-6 stream (one counter drives both: seq k carries value
/// k+1), so the armed capture provably did not perturb what the graph
/// delivered.
#[test]
#[serial]
fn armed_capture_is_record_only_delivered_values_match_the_kind6_stream() {
    const STEPS: u64 = 8;
    let (kind6, delivered, _fires) = run_relay_capture("recon", true, STEPS);
    // Hand oracle: one DrainedBatch per step (unified trigger drain), seq k.
    let expected: Vec<ReadRec> = (0..STEPS)
        .map(|k| rec(k, READ_OUTCOME_DRAINED_BATCH, k, 1))
        .collect();
    assert_eq!(kind6, expected, "the armed kind-6 stream");
    // Hand oracle: the delivered relay values are 1.0..=STEPS (CountSrc's
    // counter), in order — the SAME counter the seqs derive from.
    let values: Vec<f64> = delivered.iter().map(|f| frame_x(f)).collect();
    let expected_values: Vec<f64> = (1..=STEPS).map(|k| k as f64).collect();
    assert_eq!(values, expected_values, "delivered value stream");
    // The cross-check that ties the two streams to ONE counter: value at
    // position k == served seq at position k, +1.
    for (r, v) in kind6.iter().zip(values.iter()) {
        assert_eq!(
            *v,
            (r.seq + 1) as f64,
            "value/seq pairing at step {}",
            r.step
        );
    }
}

/// (n) The run-exit DROP REPORT against a hand oracle, under
/// SERVE-MANY. A `Period(1)` plain-input consumer driven by ONE
/// `step(700ms)` catch-up-fires 700 times inside one level execution and stages
/// EXACTLY ONE record: the step-boundary snapshot. All 700 fires then serve that
/// one frozen slot, and a serve records nothing — so a burst's record count is a
/// function of the step's READS, not of its FIRES, and 700 fires cannot overflow
/// a 64-record stage.
///
/// # This SUPERSEDES the earlier oracle, and the 636 were never FRAMES
///
/// The arm used to expect `(636, [("consumer", "inp", "body", 636)])`: under
/// serve-once, fire 1 consumed the frozen slot and fires 2..700 each fell to the
/// LIVE body drain, staging 699 records + 1 snapshot = 700 against the 64-record
/// capacity of the day, so the stage stored 64 and dropped 636. Those
/// 636 were CAPTURE-PLANE RECORDS, never frames, and each described a read that
/// popped NOTHING — `SilentSrc` never writes its output, so lazy loan
/// never publishes and this edge carries ZERO frames for the whole run. There is
/// consequently no frame accounting to preserve here: removing 699 reads removed
/// 699 records describing an empty queue.
///
/// The assertions below state that as the full picture rather than as a bare
/// empty report, so an implementation that dropped records SILENTLY (or one that
/// stopped bursting at all) cannot pass:
///
/// * the burst is REAL — 700 consumer fires from one step (anti-vacuity: without
///   it, a graph whose consumer never catch-up-fires reports `(0, [])` too);
/// * exactly ONE record is staged, and it is NoFrame with popped 0 — the edge
///   moved no frame in either direction;
/// * `backpressure_drop_oldest_count` is 0 — nothing was evicted either, so the
///   699 vanished reads took no frame with them.
///
/// Healthy control in the same body: the same graph stepped at the normal
/// cadence also reports `(0, [])`.
#[test]
#[serial]
fn run_exit_drop_report_is_empty_because_a_burst_stages_one_read_not_one_per_fire() {
    struct Outcome {
        report: (u64, Vec<(String, String, &'static str, u64)>),
        consumer_fires: u64,
        drop_oldest: u64,
        kind6: Vec<ReadRec>,
        /// The consumer stage's DERIVED capacity — the number that
        /// used to be one global constant a test could name.
        capacity: u32,
    }

    let run = |tag: &str, deltas_ms: &[u64]| -> Outcome {
        let (config, factories) = two_node_graph(
            tag,
            Box::new(SilentSrcEntry::new()),
            Box::new(BurstSinkEntry::new()),
        );
        let ring_tag = format!("{tag}_{}", std::process::id());
        let mut owner = TraceRingOwner::create_with_inputs(
            &ring_tag,
            default_capacity_records(),
            0,
            &["producer", "consumer"],
            &[&[], &["inp"]],
        )
        .expect("create ring");
        let ring_name = owner.name().to_string();
        let producer_handle = owner.producer().expect("mint producer");
        let clock = Arc::new(VirtualClock::new());
        let mut runtime =
            GraphRuntime::build_for_test(config, factories, clock, 8).expect("build graph");
        runtime.set_trace_ring_producer(
            producer_handle,
            &["producer".to_string(), "consumer".to_string()],
        );
        assert!(runtime.read_outcome_stages_armed_for_test() > 0);
        let capacity = {
            // "exactly one stage" is the claim — assert it rather than
            // reading stage 0 of an unknown number.
            let stages = runtime.read_outcome_stage_capacities("consumer");
            assert_eq!(
                stages.len(),
                1,
                "the consumer owns exactly one read-outcome stage: {stages:?}"
            );
            stages[0].capacity
        };
        for delta in deltas_ms {
            runtime.step(Duration::from_millis(*delta));
        }
        let report = runtime.read_outcome_dropped_report();
        let handle = runtime
            .node_handle("consumer")
            .expect("the consumer is a scheduler node");
        let consumer_fires = handle.fire_count();
        let drop_oldest = handle.backpressure_drop_oldest_count("inp");
        let mut ring_consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
        let mut records = Vec::new();
        ring_consumer.drain(&mut records).expect("drain ring");
        drop(ring_consumer);
        runtime.shutdown();
        drop(owner);
        let kind6 = records
            .iter()
            .filter(|r| r.record_type == RECORD_TYPE_READ_OUTCOME)
            .map(|r| {
                let (input_idx, kind) = unpack_read_outcome_meta(r.global_level);
                ReadRec {
                    step: r.step,
                    node_idx: r.node_idx,
                    input_idx,
                    kind,
                    seq: r.fire_time_ns,
                    popped: unpack_read_outcome_popped(r.duration_ns),
                }
            })
            .collect();
        Outcome {
            report,
            consumer_fires,
            drop_oldest,
            kind6,
            capacity,
        }
    };

    // The burst shape: ONE 700 ms step ⇒ 700 catch-up fires, ONE staged read.
    let burst = run("burst", &[700]);
    assert_eq!(
        burst.consumer_fires, 700,
        "anti-vacuity: the 700-fire catch-up burst really happened (a graph \
         that never bursts would report zero drops for an unrelated reason)"
    );
    assert_eq!(
        burst.report,
        (0, Vec::new()),
        "700 fires stage ONE read, so nothing overflows the {}-record stage",
        burst.capacity
    );
    // Anti-vacuity: the one staged read is far INSIDE the derived
    // capacity, so "nothing overflowed" is a claim about the read count and not
    // about a stage that happens to be enormous.
    assert!(
        burst.capacity >= 4,
        "even the smallest derived stage holds the floor: got {}",
        burst.capacity
    );
    assert_eq!(
        burst.kind6,
        vec![rec(0, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME, 0)],
        "the one staged record is the boundary snapshot, and it popped NOTHING \
         — the silent producer never published, so this edge carried zero frames"
    );
    assert_eq!(
        burst.drop_oldest, 0,
        "nothing was evicted either: the reads that no longer happen took no \
         frame with them"
    );

    // Healthy control: normal cadence ⇒ nothing dropped, empty breakdown.
    let healthy = run("burstok", &[10, 10, 10, 10, 10, 10]);
    assert_eq!(
        healthy.report,
        (0, Vec::new()),
        "a healthy run reports zero drops and an empty breakdown"
    );
}

/// (m2) the A/B arm: the SAME graph run recording-ON vs
/// recording-OFF yields BYTE-IDENTICAL delivered frames and an identical fire
/// trace — the armed capture changes the BAG, never the execution
/// (Principle #7's record-only contract, pinned against the OFF control
/// rather than a self-compare: both sides also equal the hand oracle above).
#[test]
#[serial]
fn recording_on_vs_off_fires_and_delivered_frames_are_byte_identical() {
    const STEPS: u64 = 8;
    let (kind6_on, delivered_on, fires_on) = run_relay_capture("abon", true, STEPS);
    let (kind6_off, delivered_off, fires_off) = run_relay_capture("aboff", false, STEPS);
    assert!(
        !kind6_on.is_empty(),
        "the ON leg must really capture (anti-vacuity)"
    );
    assert!(kind6_off.is_empty(), "the OFF leg captures nothing");
    assert_eq!(
        delivered_on, delivered_off,
        "delivered frames must be BYTE-identical with capture armed vs not"
    );
    assert_eq!(
        fires_on, fires_off,
        "the fire trace must be identical with capture armed vs not"
    );
    // Anti-vacuity: the compared streams are non-trivial.
    assert_eq!(delivered_on.len(), STEPS as usize);
    assert!(fires_on.len() >= 2 * STEPS as usize);
}

// ===========================================================================
// The capacity ↔ clamp SATURATION arithmetic,
// run on the union tree. Neither tree alone exhibits it (the clamp is the
// FIFO branch's, the stage is `read_outcome.rs`'s), so this is the
// first place the merged reality can be measured rather than reasoned about.
// ===========================================================================

/// A FULL-DEPTH backlog burst — the steady state of a backlogged
/// consumer under free-run, not a stall artefact — stays INSIDE the stage's
/// capacity: nothing is dropped, and no OVERFLOW MARKER is emitted.
///
/// The burst is built the way the executor really sees one: a `Period(1)`
/// producer catch-up-fires past `MAX_CONSUMER_DEPTH` inside ONE
/// `step(65 ms)`, saturating a `depth = 64` trigger input, which the SERVE-MANY
/// within-step burst then serves inside that same step (one kind-6 record per
/// CONSUMED frame). The anti-vacuity half is asserted first: the burst really
/// did reach the full depth, so "no marker" is a property of the CAPACITY
/// rather than of a burst that never happened.
///
/// # The stage is drained PER MERGE WINDOW, so the count is asserted per step
///
/// The helper takes TWO steps — a warm-up and the burst — and both feed the
/// SAME ring, so the run total is not the quantity the capacity bounds. It is
/// the LARGEST WINDOW that has to fit, and the two are asserted separately
/// rather than summed, because a sum lets a shortfall in the burst window be
/// paid for by an unrelated record from the warm-up one. MEASURED across
/// `burst_ms` ∈ {64, 65, 66, 70, 130}: the warm-up window records exactly 1
/// and the burst window exactly `MAX_CONSUMER_DEPTH`, unchanged by the burst's
/// width — the queue caps at its declared depth and the producer's catch-up
/// fires all land at its own level before the consumer's, so no window can
/// ever hold more than the depth.
#[test]
#[serial]
fn a_full_depth_backlog_burst_stays_inside_capacity_and_marks_nothing() {
    use cerulion_core::graph::topology::MAX_CONSUMER_DEPTH;

    // One step deep enough to SATURATE the queue. `+1` on the producer's
    // catch-up count over-fills it by one, so the depth is reached by eviction
    // rather than by landing exactly on it — which is why the burst window's
    // first served seq is 2 (the warm-up committed seq 0, and seq 1 was
    // evicted as the 65th frame arrived at a 64-deep queue).
    let burst_ms = MAX_CONSUMER_DEPTH as u64 + 1;
    let (kind6, records, capacity) = run_deep_backlog_burst("sat64", burst_ms);
    let consumed = |step: u64| -> Vec<&ReadRec> {
        kind6
            .iter()
            .filter(|r| r.step == step && r.node_idx == 1 && r.kind == READ_OUTCOME_DRAINED_BATCH)
            .collect()
    };
    // Step 0 is the connection warm-up: the producer's single publish is
    // consumed there, which is why it contributes a record of its own.
    assert_eq!(
        consumed(0).len(),
        1,
        "the warm-up step consumes the one frame it produced — stated so the \
         burst window's own count below cannot be topped up by it"
    );
    // THE ANTI-VACUITY HALF: the burst window really did serve a full-depth
    // backlog, so "no marker" below is a property of the CAPACITY rather than
    // of a burst that never happened.
    assert_eq!(
        consumed(1).len(),
        MAX_CONSUMER_DEPTH,
        "the within-step burst must serve the input's FULL declared depth in \
         ONE merge window — the case this arm is named for"
    );
    // THE PIN: that many records fit in one stage, so nothing was dropped and
    // no marker was emitted.
    assert!(
        consumed(1).len() as u32 <= capacity,
        "a full-depth burst must fit in the stage ({} records vs this edge's \
         DERIVED capacity {capacity})",
        consumed(1).len()
    );
    assert!(
        !kind6.iter().any(|r| r.kind == READ_OUTCOME_TRUNCATED),
        "no OVERFLOW MARKER may be emitted for a burst that fit: {kind6:?}"
    );
    // And the ring carries no other kind-6 shape than reads (the marker is
    // the only annotation this shape could produce).
    assert!(
        records
            .iter()
            .filter(|r| r.record_type == RECORD_TYPE_READ_OUTCOME)
            .count()
            >= MAX_CONSUMER_DEPTH,
        "sanity: the ring really carries the burst's records"
    );
}

/// THE ANTI-TAUTOLOGY HALF — an OVER-capacity window really does
/// emit a marker, carrying THIS WINDOW's exact drop count, and the marker is
/// the window's LAST kind-6 record on that edge.
///
/// This is also `truncated_consumed_frame_reads_as_a_marked_hole_not_silence`:
/// what replay reads back at the position of a dropped record is a MARKED HOLE
/// (`truncated`, with a count), never silence — which is exactly what a
/// verdicting verifier needs in order not to call a truncated edge
/// "compared clean".
///
/// # The stimulus is STAGED, not fired — and that is FORCED, not a shortcut
///
/// A 700-fire `Period(1)` catch-up burst on a plain `#[input]` cannot reach the
/// rim: under serve-ONCE mechanics it would stage 1 boundary
/// snapshot + 699 live body reads, but the step-frozen context
/// slot serves EVERY fire of a burst, so a plain edge
/// stages exactly ONE read per step however many times its node fires — which
/// is why a fire-count stimulus never overflows anything.
/// With the consumer `depth`
/// cap and the within-step burst clamp both at `MAX_CONSUMER_DEPTH`, against a
/// capacity const-asserted at `2 * (MAX_CONSUMER_DEPTH + HEADROOM)`, **no
/// `GraphConfig` can fill a stage any more** — which is precisely what that
/// sizing is for. Reaching the rim therefore means staging records directly
/// (`GraphRuntime::stage_read_outcomes_for_test`).
///
/// Everything downstream of the staging is the PRODUCTION path — the level-end
/// merge, the marker emission, the trace-ring hand-over — and the graph's own
/// contribution is pinned at ZERO in the same body (a data-trigger consumer
/// whose producer never publishes never fires), so the staged count is the
/// whole population and the oracle below stays a hand-written one.
#[test]
#[serial]
fn truncated_consumed_frame_reads_as_a_marked_hole_not_silence() {
    // `SilentSrc` never writes its output, so lazy loan never
    // publishes and the data-trigger consumer never fires: the graph stages
    // NOTHING, on any step, under any read-accounting rule.
    const STAGED: u64 = 700;
    let (config, factories) = two_node_graph(
        "hole",
        Box::new(SilentSrcEntry::new()),
        Box::new(TrigSinkEntry::new()),
    );
    let ring_tag = format!("hole_{}", std::process::id());
    let mut owner = TraceRingOwner::create_with_inputs(
        &ring_tag,
        default_capacity_records(),
        0,
        &["producer", "consumer"],
        &[&[], &["inp"]],
    )
    .expect("create ring");
    let ring_name = owner.name().to_string();
    let producer_handle = owner.producer().expect("mint producer");
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build graph");
    runtime.set_trace_ring_producer(
        producer_handle,
        &["producer".to_string(), "consumer".to_string()],
    );
    assert_eq!(
        runtime.read_outcome_stages_armed_for_test(),
        1,
        "the recording armed the consumer's ONE stage (the Unified trigger \
         edge carries a single stage)"
    );
    // The rim is this edge's own DERIVED capacity, not a global
    // constant — so the hand oracle below asks for it rather than naming it.
    let capacity = {
        // "exactly one stage" is the claim, so ASSERT it: `.first()` would
        // silently read stage 0 of several and the oracle below would be
        // computed against a rim that is not the only one in play.
        let stages = runtime.read_outcome_stage_capacities("consumer");
        assert_eq!(
            stages.len(),
            1,
            "the consumer owns exactly one read-outcome stage: {stages:?}"
        );
        stages[0].capacity
    };
    assert!(
        (capacity as u64) < STAGED,
        "the stimulus must really outrun the derived capacity: staged {STAGED} \
         against {capacity}"
    );

    // (1) The CONTROL window — a step with nothing staged. Every kind-6 record
    // below is therefore attributable to the injection, never to the graph.
    runtime.step(Duration::from_millis(10));

    // (2) The OVERFLOW window — stage past the rim, then merge it.
    assert_eq!(
        runtime.stage_read_outcomes_for_test("consumer", STAGED as usize),
        1,
        "the reads were staged on exactly the one stage this edge owns"
    );
    runtime.step(Duration::from_millis(10));
    let (dropped_total, _breakdown) = runtime.read_outcome_dropped_report();

    let mut ring_consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
    let mut records = Vec::new();
    ring_consumer.drain(&mut records).expect("drain ring");
    drop(ring_consumer);
    runtime.shutdown();
    drop(owner);

    let kind6: Vec<ReadRec> = records
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_READ_OUTCOME)
        .map(|r| {
            let (input_idx, kind) = unpack_read_outcome_meta(r.global_level);
            ReadRec {
                step: r.step,
                node_idx: r.node_idx,
                input_idx,
                kind,
                seq: r.fire_time_ns,
                popped: unpack_read_outcome_popped(r.duration_ns),
            }
        })
        .collect();

    // The CONTROL claim: the graph itself staged NOTHING, so the whole stream
    // belongs to the overflow window (step 1) and the oracle below counts only
    // records this test staged. Without it, "700 staged" is an assumption.
    assert!(
        kind6.iter().all(|r| r.step == 1),
        "the control window (step 0) must carry no kind-6 record at all — the \
         graph stages nothing on this edge: {kind6:?}"
    );

    // Hand oracle: 700 reads staged against capacity ⇒ CAPACITY survive and
    // the rest are dropped, and the drop report agrees with the wire.
    let expected_dropped = STAGED - capacity as u64;
    assert_eq!(
        dropped_total, expected_dropped,
        "the run-exit report's total is the hand oracle"
    );
    let survivors = kind6
        .iter()
        .filter(|r| r.kind != READ_OUTCOME_TRUNCATED)
        .count();
    assert_eq!(
        survivors as u32, capacity,
        "the stage retained exactly its capacity in READ records"
    );
    // THE PIN: the hole is MARKED, and the marker is LAST in the window.
    let marker = kind6
        .last()
        .copied()
        .expect("the window's records are non-empty");
    assert_eq!(
        (marker.kind, marker.popped as u64, marker.seq),
        (
            READ_OUTCOME_TRUNCATED,
            expected_dropped,
            READ_OUTCOME_NO_FRAME
        ),
        "the window ENDS with a marker carrying its own drop count and no served \
         frame: {kind6:?}"
    );
    assert_eq!(
        kind6
            .iter()
            .filter(|r| r.kind == READ_OUTCOME_TRUNCATED)
            .count(),
        1,
        "ONE marker per window, never one per dropped record"
    );
    // A review addendum: the marker also carries a ROLE, which nothing here
    // pinned. The marker is minted BY THE STAGE (`drain_into`), so the only
    // role it can carry is the STAGE's own — `ReadSiteRole::from(ReadStageRole)`
    // — and this Unified trigger edge's stage is a `Body` one. Read through the
    // PRODUCTION decoder, off the same word the replay engine reads. It is
    // DIAGNOSTIC and no consumer steers on it (a `Truncated` marker goes to
    // BOTH injection queues under every role value, and raises
    // `TruncatedReadLog` under every one), but a marker that carried
    // `Unstamped` here would mean the stage's role never reached the wire at
    // all.
    let marker_roles: Vec<ReadSiteRole> = records
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_READ_OUTCOME)
        .filter(|r| unpack_read_outcome_meta(r.global_level).1 == READ_OUTCOME_TRUNCATED)
        .map(|r| read_site_role(r.global_level))
        .collect();
    assert_eq!(
        marker_roles,
        vec![ReadSiteRole::Body],
        "the overflow marker carries the OVERFLOWED STAGE's role — a mint that \
         never threaded one reads `Unstamped` here"
    );
    // Nothing is silent: survivors + the marker's count == every read the
    // recording really performed.
    assert_eq!(
        survivors as u64 + marker.popped as u64,
        STAGED,
        "the read log accounts for EVERY staged read — retained or marked"
    );
}

/// The ADOPTED rim is OBSERVABLE: this is the arm that catches a variant
/// where the replay adopts nothing.
///
/// That variant was the only one in the roster with no killing test:
/// adoption is the C2 change of record, and it was revertible with the
/// whole suite green. The residual said the arm belonged here, "where the
/// stage AND its capacity are directly observable rather than only the
/// verdict", and it does.
///
/// The stimulus is `stage_read_outcomes_for_test`, whose synthetic reads carry
/// DISTINCT sequences and therefore never fold — a burst of identical reads
/// would collapse to a handful of records and overflow nothing, which is the
/// fold working and would make this arm vacuous.
#[test]
#[serial]
fn a_recorded_rim_is_adopted_and_the_truncation_is_the_recorded_shape() {
    use cerulion_core::read_outcome::{AdoptedRim, StageKey};
    // Above BOTH rims (the derived one for this edge is 75), so the drop count
    // is a discriminator rather than a coincidence.
    const STAGED: usize = 100;
    const RECORDED_RIM: u32 = 8;
    let (config, factories) = two_node_graph(
        "adoptrim",
        Box::new(SilentSrcEntry::new()),
        Box::new(TrigSinkEntry::new()),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build graph");

    // The DERIVED rim — what this binary would use if it adopted nothing. The
    // arm is only meaningful if the two differ, so that is asserted, not hoped.
    let derived = {
        let stages = runtime.read_outcome_stage_capacities("consumer");
        assert_eq!(stages.len(), 1, "one stage on this edge: {stages:?}");
        stages[0].capacity
    };
    assert_ne!(
        derived, RECORDED_RIM,
        "PRECONDITION: the recorded rim must differ from the derived one, or \
         adopting and not adopting are the same experiment"
    );
    assert!(
        (STAGED as u32) > derived && (STAGED as u32) > RECORDED_RIM,
        "PRECONDITION: the stimulus must outrun BOTH rims, so the drop count \
         says WHICH rim was armed (staged {STAGED}, derived {derived}, recorded \
         {RECORDED_RIM})"
    );

    // ADOPT the recorded rim, exactly as the replay engine does.
    let mut recorded: std::collections::HashMap<StageKey, AdoptedRim> =
        std::collections::HashMap::new();
    for key in runtime.read_outcome_stage_keys() {
        recorded.insert(
            key,
            AdoptedRim::from_recorded(RECORDED_RIM).expect("8 is inside the window"),
        );
    }
    runtime.enable_read_outcome_memory_sink_with_recorded_capacities(&recorded);
    assert_eq!(
        runtime.read_outcome_stage_capacities("consumer")[0].capacity,
        RECORDED_RIM,
        "the stage really armed at the RECORDED rim"
    );

    assert_eq!(
        runtime.stage_read_outcomes_for_test("consumer", STAGED),
        1,
        "the reads landed on the one stage this edge owns"
    );
    let (dropped, _breakdown) = runtime.read_outcome_dropped_report();
    runtime.shutdown();

    // THE PIN: the truncation is the RECORDED shape, not the derived one. A
    // replay that adopted nothing drops `STAGED - derived`; one that adopted
    // drops `STAGED - RECORDED_RIM`. Hand arithmetic, both spelled out.
    assert_eq!(
        dropped,
        STAGED as u64 - RECORDED_RIM as u64,
        "adopting the recorded rim means truncating where the RECORDING did \
         (would be {} if the derived rim {derived} had been armed)",
        STAGED as u64 - derived as u64
    );
}

/// Drive a full-depth backlog burst: a `Period(1)` producer + a `depth = 64`
/// data-trigger consumer, ONE `step(burst_ms)` so the producer catch-up-fires
/// `burst_ms` times and the consumer serves the queued backlog inside the
/// same step. Returns the decoded kind-6 stream + every ring record.
fn run_deep_backlog_burst(tag: &str, burst_ms: u64) -> (Vec<ReadRec>, Vec<TraceRingRecord>, u32) {
    let (config, factories) = two_node_graph(
        tag,
        Box::new(BacklogSrcEntry::new()),
        Box::new(DeepTrigSinkEntry::new()),
    );
    let ring_tag = format!("{tag}_{}", std::process::id());
    let mut owner = TraceRingOwner::create_with_inputs(
        &ring_tag,
        default_capacity_records(),
        0,
        &["producer", "consumer"],
        &[&[], &["inp"]],
    )
    .expect("create ring");
    let ring_name = owner.name().to_string();
    let producer_handle = owner.producer().expect("mint producer");
    let clock = Arc::new(VirtualClock::new());
    // The input is provisioned at MAX_CONSUMER_DEPTH, so the transport must
    // be able to hold that many frames.
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 128).expect("build graph");
    runtime.set_trace_ring_producer(
        producer_handle,
        &["producer".to_string(), "consumer".to_string()],
    );
    // The consumer's stage is sized from its OWN edge, so the arm
    // that asks "did the burst fit?" has to read the capacity rather than name
    // a constant.
    let capacity = {
        // "exactly one stage" is the claim, so ASSERT it: `.first()` would
        // silently read stage 0 of several and the oracle below would be
        // computed against a rim that is not the only one in play.
        let stages = runtime.read_outcome_stage_capacities("consumer");
        assert_eq!(
            stages.len(),
            1,
            "the consumer owns exactly one read-outcome stage: {stages:?}"
        );
        stages[0].capacity
    };
    // Warm the connection, then ONE long step that builds AND serves the
    // backlog inside a single merge window.
    runtime.step(Duration::from_millis(1));
    runtime.step(Duration::from_millis(burst_ms));
    let mut ring_consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
    let mut records = Vec::new();
    ring_consumer.drain(&mut records).expect("drain ring");
    drop(ring_consumer);
    runtime.shutdown();
    drop(owner);
    let kind6 = records
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_READ_OUTCOME)
        .map(|r| {
            let (input_idx, kind) = unpack_read_outcome_meta(r.global_level);
            ReadRec {
                step: r.step,
                node_idx: r.node_idx,
                input_idx,
                kind,
                seq: r.fire_time_ns,
                popped: unpack_read_outcome_popped(r.duration_ns),
            }
        })
        .collect();
    (kind6, records, capacity)
}

// ===========================================================================
// The PRODUCER ANNOTATION on the PRODUCTION wiring path
// (P1-4 / P1-5), and the OVERFLOW MARKER's hand-over debt (P2-8)
// ===========================================================================
//
// `edge_needs_producer_annotation` (graph/runtime.rs) decides at WIRING time
// whether a consumer edge's reads carry a `READ_OUTCOME_PRODUCER` annotation,
// and marks the subscriber (`mark_multi_publisher_edge`) at THREE call sites
// — the body input, the Separate trigger drain and the Sync drain. Until this
// change nothing drove that decision end to end: the annotation's own unit
// arms live in `read_outcome.rs` (they call `record_with_producer` directly,
// so they are blind to whether any production site ever marks an edge) and
// the trace-ring arms build the record by hand (blind to whether the token a
// subscriber computes is the one the manifest table can resolve).
//
// What these arms add is the RESOLUTION SEAM: the publisher id the BUILD
// recorded into `GraphRuntime::read_log_publisher_ids` — the table bagd hands
// `TraceRingOwner::create_with_inputs_and_publishers_or_degrade`, i.e. the
// offline resolver's whole input — must hash to EXACTLY the token the
// subscriber stamped onto the wire from `Sample::origin()`. Those are two
// independent expressions of one rule (`producer_token`), computed from two
// independently obtained ids, so agreeing is a cross-check rather than a
// self-compare. A token that resolves to nothing is a `foreign(...)` edge the
// replay verifier stands down loudly — which is the same silence, offline, as
// having no annotation at all.

/// A decoded kind-6 record that keeps the aux word RAW.
///
/// [`ReadRec`] projects the aux word through `unpack_read_outcome_popped` —
/// the READ packing. On a [`READ_OUTCOME_PRODUCER`] annotation that word is
/// the FULL 64-bit producer token (`trace_ring` documents
/// `unpack_read_outcome_popped` as never being called on this kind), so the
/// projection would shred it. These arms therefore decode their own stream.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
struct AnnotRec {
    node_idx: u32,
    input_idx: u16,
    kind: u16,
    /// The served wire sequence, or `READ_OUTCOME_NO_FRAME`. A `Producer`
    /// annotation REPEATS the annotated read's sequence — the redundant join
    /// key, so a reader that lost the ordering can still pair them.
    seq: u64,
    /// Raw aux: the producer TOKEN on a `Producer` record, the read packing
    /// on every other kind.
    aux: u64,
}

/// Decode every kind-6 record in ring order, aux word intact.
fn annot_stream(records: &[TraceRingRecord]) -> Vec<AnnotRec> {
    records
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_READ_OUTCOME)
        .map(|r| {
            let (input_idx, kind) = unpack_read_outcome_meta(r.global_level);
            AnnotRec {
                node_idx: r.node_idx,
                input_idx,
                kind,
                seq: r.fire_time_ns,
                aux: r.duration_ns,
            }
        })
        .collect()
}

/// The producer TOKEN a wiring-time publisher id resolves to.
///
/// `transport::subscriber::producer_token` is private, so this is the rule
/// spelled out BY HAND: FNV-1a 64 over the `UniquePublisherId::value()`
/// LITTLE-ENDIAN bytes. Hand-writing it is what makes the comparison an
/// oracle — the test derives the token from the id the BUILD recorded, the
/// runtime derives it from the `Sample::origin()` it read off the wire, and
/// the two must meet. A byte-order or hash-input change on either side breaks
/// the meet.
fn expected_token(publisher_id: u128) -> u64 {
    cerulion_core::wire::fnv1a_hash(&publisher_id.to_le_bytes())
}

/// One consumed frame on an ANNOTATED edge, as the pairing walk resolves it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct AnnotatedRead {
    token: u64,
    seq: u64,
    kind: u16,
}

/// Walk one edge's kind-6 stream and enforce the BINDING RULE the annotation
/// is defined by: a `Producer` record names the publisher whose frame the
/// NEXT read record on the same `(node, input)` served.
///
/// Returns the resolved `(token, seq)` reads. Panics — with the offending
/// stream printed — on any shape that would make an offline reader attribute
/// a frame to the wrong publisher: an unannotated read on an annotated edge,
/// two annotations in a row (a WIDOWED token, which is exactly what
/// `record_with_producer`'s both-or-neither rim admission exists to prevent),
/// a join-key disagreement between the annotation and its read, or a trailing
/// annotation with nothing to bind to.
fn pair_annotated_reads(edge: &[AnnotRec]) -> Vec<AnnotatedRead> {
    let mut out = Vec::new();
    let mut pending: Option<AnnotRec> = None;
    for rec in edge {
        if rec.kind == READ_OUTCOME_PRODUCER {
            assert!(
                pending.is_none(),
                "two PRODUCER annotations in a row — the first names a read that \
                 does not exist (a widowed token binds the NEXT window's first \
                 read to the wrong publisher): {edge:?}"
            );
            pending = Some(*rec);
            continue;
        }
        let annot = pending.take().unwrap_or_else(|| {
            panic!(
                "an UNANNOTATED read on an annotated edge — its served sequence \
                 names no producer offline: {edge:?}"
            )
        });
        assert_eq!(
            annot.seq, rec.seq,
            "the annotation must repeat its read's served sequence (the redundant \
             join key): {edge:?}"
        );
        out.push(AnnotatedRead {
            token: annot.aux,
            seq: rec.seq,
            kind: rec.kind,
        });
    }
    assert!(
        pending.is_none(),
        "the window ends on a PRODUCER annotation with no read to bind it to: {edge:?}"
    );
    out
}

/// The recorded-run harness for the annotation arms: identical to
/// [`run_recorded_custom`] except that it also hands back the BUILD's
/// `read_log_publisher_ids` table — the offline resolver's whole input, and
/// the half of the resolution seam these arms exist to cross-check.
/// The WIRING-time publisher table: node id -> its `(output, publisher id)`
/// pairs, exactly as `TraceRingOwner::create_with_inputs_and_publishers*`
/// receives it. Named because `clippy::type_complexity` refuses the inline
/// spelling in a return position — and a name is the better answer anyway:
/// this IS the table whose ids must hash to the tokens the drained `Producer`
/// records carry.
type WiringPublisherTable = IndexMap<String, Vec<(String, u128)>>;

fn run_annot_recorded(
    tag: &str,
    config: GraphConfig,
    factories: IndexMap<String, Box<dyn NodeEntry>>,
    node_ids: &[&str],
    node_inputs: &[&[&str]],
    steps: u64,
) -> (Vec<AnnotRec>, WiringPublisherTable) {
    run_annot_recorded_inspecting(tag, config, factories, node_ids, node_inputs, steps, |_| {})
}

/// [`run_annot_recorded`] with a hook that sees the BUILT, STEPPED runtime
/// before it is shut down — the wide arms read the fire-pool width
/// and the forced-walk counter off it, which the ring cannot carry.
fn run_annot_recorded_inspecting(
    tag: &str,
    config: GraphConfig,
    factories: IndexMap<String, Box<dyn NodeEntry>>,
    node_ids: &[&str],
    node_inputs: &[&[&str]],
    steps: u64,
    inspect: impl FnOnce(&GraphRuntime),
) -> (Vec<AnnotRec>, WiringPublisherTable) {
    let node_id_strings: Vec<String> = node_ids.iter().map(|s| s.to_string()).collect();
    let ring_tag = format!("{tag}_{}", std::process::id());
    let mut owner = TraceRingOwner::create_with_inputs(
        &ring_tag,
        default_capacity_records(),
        0,
        node_ids,
        node_inputs,
    )
    .expect("create ring");
    let ring_name = owner.name().to_string();
    let producer_handle = owner.producer().expect("mint producer");

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 32).expect("build graph");
    runtime.set_trace_ring_producer(producer_handle, &node_id_strings);
    assert!(
        runtime.read_outcome_stages_armed_for_test() > 0,
        "installing the ring must ARM the wired read-outcome stages"
    );

    for _ in 0..steps {
        runtime.step(Duration::from_millis(10));
    }
    inspect(&runtime);

    // Read the publisher table off the BUILT runtime — the ids belong to the
    // ports this build wired, so the table cannot describe a port that never
    // existed.
    let publishers = runtime.read_log_publisher_ids().clone();

    let mut ring_consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
    let mut records = Vec::new();
    ring_consumer.drain(&mut records).expect("drain ring");
    drop(ring_consumer);
    runtime.shutdown();
    drop(owner);
    (annot_stream(&records), publishers)
}

/// The ONE publisher id a node's output table records, by node id.
fn publisher_id_of(publishers: &WiringPublisherTable, node: &str, output: &str) -> u128 {
    let outputs = publishers
        .get(node)
        .unwrap_or_else(|| panic!("node '{node}' missing from the publisher table"));
    outputs
        .iter()
        .find(|(name, _)| name == output)
        .unwrap_or_else(|| panic!("node '{node}' has no output '{output}': {outputs:?}"))
        .1
}

/// P1-4 — THE PRODUCTION ANNOTATION PATH, AND THE TOKEN THAT RESOLVES, and
/// the NARROW-shape routing pins — hence the name: this graph is the
/// wide arms' narrow twin, so it is the only place the size half of the routing
/// ranking is observable.
///
/// A `multi_publisher_topics:`-listed topic with TWO REAL in-graph publishers
/// (the `/tf` broadcaster shape the opt-in exists for), consumed by a
/// data-trigger input while read capture is ARMED. On such a topic the wire
/// `sequence` is a PER-PUBLISHER counter, so a served sequence alone names no
/// producer — which is the entire condition the annotation exists for.
///
/// The arm pins BOTH halves of the seam, because either alone is satisfiable
/// by a broken implementation:
///
/// (a) the ANNOTATION really is emitted by the production wiring path — every
///     read on that edge is immediately preceded by a `Producer` record
///     carrying the same served sequence (`pair_annotated_reads`);
/// (b) the TOKEN RESOLVES — the set of tokens on the wire is EXACTLY the set
///     the BUILD's `read_log_publisher_ids` table hashes to. Set EQUALITY, in
///     both directions: no token on the wire fails to resolve (which offline
///     is a `foreign(...)` stand-down), and BOTH table entries are exercised
///     (so the annotation is genuinely discriminating between two publishers
///     rather than stamping a constant).
///
/// Plus the ATTRIBUTION pin: the two producers run at DIFFERENT rates
/// (`StepSrc` = `Period(10)` on 10 ms steps ⇒ one frame per step; `SlowSrc` =
/// `Period(30)` ⇒ one per three steps), so a table swap — resolving each
/// token to the OTHER publisher — is caught by cardinality: the fast
/// producer's token must carry strictly more reads, by a margin the 3:1
/// publish ratio makes unambiguous.
///
/// # MULTIPLICITY, which neither (b) nor the ratio can see
///
/// A `BTreeSet` of tokens is blind to how MANY times each was recorded, and
/// the ratio is blind to any factor applied to BOTH sides — so a runtime that
/// staged every `(Producer, read)` PAIR twice satisfies both: the token set is
/// unchanged, `pair_annotated_reads` still walks a well-formed `P,R,P,R`
/// stream, and `fast_reads`/`slow_reads` both double. That is the exact class
/// the module header names as mutation target 3 (accounting-once), which every
/// OTHER arm in this file catches because its oracle is a full-sequence
/// compare. So this arm carries one too: the ordered `(token, seq)` sequence,
/// DERIVED from the two declared periods rather than measured — at 10 ms
/// steps `fast` commits on every step and `slow` on every third, and the
/// shared topic is FIFO, so the run is
/// `f0 f1 f2 s0 f3 f4 f5 s1 …` and every element is hand-computable.
/// A duplicated pair, a dropped read, a re-recorded re-offer and a swapped
/// arrival order all break it.
///
/// ANTI-TAUTOLOGY, in the same body and the same recording: the consumer's
/// SECOND edge is a plain input on an ordinary single-writer graph-owned
/// topic. It reads (records exist) and carries ZERO annotations — so "the
/// runtime annotates" cannot pass by annotating everything.
#[test]
#[serial]
fn a_narrow_shared_level_is_stamped_but_not_credited() {
    const SHARED: &str = "/romp/tf";
    const STEPS: u64 = 12;

    let mut fast = src_def("fast");
    fast.outputs[0].topic = Some(SHARED.to_string());
    let mut slow = src_def("slow");
    slow.outputs[0].topic = Some(SHARED.to_string());
    let solo_src = src_def("solo_src");
    let consumer = sink_def("consumer", &[("shared", SHARED), ("solo", "solo_src/out")]);
    let mut config = graph_of("romp", vec![fast, slow, solo_src, consumer]);
    config.multi_publisher_topics = vec![SHARED.to_string()];

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fast".to_string(), Box::new(StepSrcEntry::new()));
    factories.insert("slow".to_string(), Box::new(SlowSrcEntry::new()));
    factories.insert("solo_src".to_string(), Box::new(StepSrcEntry::new()));
    factories.insert("consumer".to_string(), Box::new(AnnotDualSinkEntry::new()));

    let (stream, publishers) = run_annot_recorded_inspecting(
        "mpannot",
        config,
        factories,
        &["fast", "slow", "solo_src", "consumer"],
        &[&[], &[], &[], &["shared", "solo"]],
        STEPS,
        |runtime| {
            // ROUTING, on the NARROW shape (this arm's graph is the
            // wide arms' narrow twin, so it is where the size/pool half of the
            // ranking is observable). The build STAMPS level 0 — it holds two
            // non-serial-gated producers of one listed topic — while level 1
            // (the lone consumer) is unconstrained; but the level is only 3
            // REST fires, so `rest_driver` ranks it `SerialBySize` and the
            // constraint is NOT credited. A counter that moved here would mean
            // the "alone" in `forced_serial_rest_walks` had stopped holding.
            assert_eq!(
                runtime.level_rest_walk_for_test(),
                vec![true, false],
                "level 0 carries the shared pair and is stamped; the consumer's \
                 level carries no producer at all"
            );
            assert_eq!(
                runtime.forced_serial_rest_walks(),
                0,
                "a 3-fire REST is serial by SIZE — the constraint changed no \
                 routing here and must not be credited"
            );
        },
    );

    // node_idx 3 = "consumer" (manifest order); input_idx 0 = "shared",
    // 1 = "solo" (wiring order).
    let shared_edge: Vec<AnnotRec> = stream
        .iter()
        .copied()
        .filter(|r| r.node_idx == 3 && r.input_idx == 0)
        .collect();
    let solo_edge: Vec<AnnotRec> = stream
        .iter()
        .copied()
        .filter(|r| r.node_idx == 3 && r.input_idx == 1)
        .collect();

    // Nothing overflowed, so the pairing walk sees only annotations + reads.
    assert!(
        !shared_edge.iter().any(|r| r.kind == READ_OUTCOME_TRUNCATED),
        "this run must not overflow its stage (an overflow marker is a \
         non-annotated record and would confound the pairing): {shared_edge:?}"
    );

    // (a) the annotation is EMITTED by the production wiring path, and binds.
    let reads = pair_annotated_reads(&shared_edge);
    assert!(
        !reads.is_empty(),
        "the multi-publisher edge must have consumed frames: {shared_edge:?}"
    );
    assert!(
        reads.iter().all(|r| r.kind == READ_OUTCOME_DRAINED_BATCH),
        "a Unified data-trigger edge's reads are drained batches: {reads:?}"
    );

    // (b) the token RESOLVES — set equality against the BUILD's own table.
    let fast_token = expected_token(publisher_id_of(&publishers, "fast", "out"));
    let slow_token = expected_token(publisher_id_of(&publishers, "slow", "out"));
    assert_ne!(
        fast_token, slow_token,
        "the two publishers must mint distinct ids (the token is the offline \
         discriminator; equal ids would make every assertion below vacuous)"
    );
    let observed: BTreeSet<u64> = reads.iter().map(|r| r.token).collect();
    let resolvable: BTreeSet<u64> = [fast_token, slow_token].into_iter().collect();
    assert_eq!(
        observed,
        resolvable,
        "every token on the wire must resolve through the manifest publisher \
         table, and BOTH table entries must be exercised — the id the BUILD \
         recorded ({:#x} / {:#x}) hashed by the SAME rule the subscriber \
         stamps from `Sample::origin()`",
        publisher_id_of(&publishers, "fast", "out"),
        publisher_id_of(&publishers, "slow", "out"),
    );

    // ATTRIBUTION: a swapped table is caught by cardinality (3:1 publish rate).
    let fast_reads = reads.iter().filter(|r| r.token == fast_token).count();
    let slow_reads = reads.iter().filter(|r| r.token == slow_token).count();
    assert!(
        fast_reads >= 2 * slow_reads && slow_reads > 0,
        "the Period(10) producer publishes ~3x the Period(30) one, so its token \
         must carry the larger share (fast {fast_reads}, slow {slow_reads}): \
         {reads:?}"
    );

    // MULTIPLICITY + ORDER: the hand oracle, derived from the declared
    // periods. `fast` is `Period(10)` on 10 ms steps ⇒ it commits once per
    // step, seq k on step k+1; `slow` is `Period(30)` ⇒ once every third step,
    // seq j on step 3(j+1). Both write ONE FIFO topic and level 0 fires them
    // in declaration order, so a step carrying both reads fast FIRST. Nothing
    // here is read off the run: a set cannot see a doubled pair, and a ratio
    // cannot see a factor applied to both sides.
    let mut expected: Vec<(u64, u64)> = Vec::new();
    for step in 1..=STEPS {
        expected.push((fast_token, step - 1));
        if step % 3 == 0 {
            expected.push((slow_token, step / 3 - 1));
        }
    }
    let observed_pairs: Vec<(u64, u64)> = reads.iter().map(|r| (r.token, r.seq)).collect();
    assert_eq!(
        observed_pairs, expected,
        "one record per CONSUMED frame, in arrival order, each naming its own \
         producer — a duplicated (Producer, read) pair, a dropped read or a \
         re-recorded re-offer all break this while leaving the token SET and \
         the fast:slow RATIO intact (fast token {fast_token:#x}, slow \
         {slow_token:#x})"
    );

    // ANTI-TAUTOLOGY: the single-writer sibling edge reads, and is NOT annotated.
    assert!(
        !solo_edge.is_empty(),
        "the control edge must really read (an empty control proves nothing)"
    );
    assert_eq!(
        solo_edge
            .iter()
            .filter(|r| r.kind == READ_OUTCOME_PRODUCER)
            .count(),
        0,
        "a single-writer graph-owned edge carries NO producer annotation — its \
         served sequence already names its one publisher: {solo_edge:?}"
    );
}

// ===========================================================================
// The WIDE-path twin of the arm above.
//
// On the rayon WIDE executor path (>= PARALLEL_FIRE_THRESHOLD non-serial-gated
// fires on one level, threads > 1) node ticks — publishes included — run on
// rayon workers, so two same-level producers of ONE `multi_publisher_topics`
// topic used to publish CONCURRENTLY with nothing ordering them: the shared
// FIFO's interleave was a scheduling artifact. The fire trace is blind to it by
// construction (PASS 3 merges per-node fragments in decision order), so the
// read log and the bag inherited an order replay could not reproduce on
// healthy code (Principle #7). The fix stamps such a level
// `RestWalk::InsertionOrder` at build and the scheduler walks its REST
// serially in graph order. The arm above is the NARROW shape (a 4-node graph
// is 3 REST fires); these arms are the same oracle on a level wide enough to
// engage rayon, plus the routing observable that makes the pin deterministic.
//
// WHAT A CONSUMER CAN AND CANNOT SEE (measured, and it shaped the arms). An
// in-graph consumer of a two-publisher topic drains in CONNECTION order, not
// commit order: iceoryx2's `Receiver::receive` walks its connection storage in
// slot order and returns the FIRST connection holding data (0.9.1
// `port/details/receiver.rs`), and a level-1 consumer only drains after level
// 0 has joined, so both frames are already queued and connection 0 — the
// publisher that connected first, i.e. the first-declared one — is served
// first on EVERY run. The `(producer, seq)` read-order oracle is therefore
// satisfied by an unconstrained wide path too (the read-order half alone survives the
// disjunct-deleted variant 10/10), which is exactly the tautology class this file
// refuses. So the tick order is made DATA: both producers draw
// a process-wide ticket inside their tick and stamp it INTO the frame (`x` =
// ticket, `y` = producer label), and the hand oracle is `(producer, seq,
// ticket == its position in the stream)`. The read order stays the
// WHAT-unchanged pin; the ticket is the discriminating half. What the
// constraint protects is the drainer that CAN land between the two commits —
// a bagd tap, `topic echo`, any out-of-process reader — whose recorded order
// is the publish order.
//
// TICKET ORDER vs ROUTING (why there are TWO observables). The ticket is a
// race outcome: under an unconstrained wide path the two ticks overlap on
// different rayon workers, and which one reaches its stamp first depends on
// the workers' scheduling. It is biased hard (see `FAST_BIAS_ITERS`) but not
// guaranteed, so on a starved runner a counted-but-not-enforced constraint
// could draw the tickets in order anyway. The ROUTING observable has no such
// hole: the serial REST walk runs every tick on the thread that called
// `step()`, while the wide path runs each tick on a `cerulion-fire-N` worker
// and the injecting caller never participates in the `for_each`. So each
// producer also records `std::thread::current().id()` per tick
// (`PUBLISH_TIDS`), and the arms assert it equals the stepping thread's — the
// same discipline the scheduler's
// `armed_intra_step_pauses_force_the_serial_rest_walk_for_deterministic_hook_order`
// already uses. Ticket = the ORDER half; thread id = the ROUTING half.
// ===========================================================================

/// RAII guard restoring (removing) `CERULION_FIRE_THREADS` on drop, even on a
/// panic. `build_for_test` reads it ONCE at build, so a guard is taken BEFORE
/// the harness builds and held for the whole body. Env is process-global;
/// every test in this binary is `#[serial]` (mirrors `rayon_fire_iox2_test`).
struct FireThreadsGuard;

impl FireThreadsGuard {
    fn set(value: &str) -> Self {
        std::env::set_var("CERULION_FIRE_THREADS", value);
        Self
    }
}

impl Drop for FireThreadsGuard {
    fn drop(&mut self) {
        std::env::remove_var("CERULION_FIRE_THREADS");
    }
}

/// The TICK TICKET: one process-wide counter both shared-topic producers draw
/// from inside their tick and stamp into the frame, so the data itself carries
/// an order the consumer's connection-order drain cannot see.
///
/// It measures TICK order, not commit order — deliberately, and the two are
/// the same thing under the walk this file pins. The write that stamps the
/// ticket is what LOANS the port (the lazy loan); the COMMIT happens later,
/// when `OutputProxy::Drop` runs at the end of the tick. Under the serial REST
/// walk a tick and its Drop both complete before the next tick begins, so tick
/// order IS commit order and the ticket names the publish order exactly. Under
/// the wide path the two ticks overlap and the ticket records which tick
/// reached its stamp first — still the right signal (a mis-ordered tick is a
/// mis-ordered publish), but a RACE outcome, which is why the deterministic
/// half of the pin is [`PUBLISH_TIDS`], not this. Reset per run under
/// `#[serial]`.
static TICK_TICKET: AtomicU64 = AtomicU64::new(0);
/// Each shared-topic producer tick's `(ticket, thread id)` — the ROUTING half
/// of the pin, and the deterministic one (see the section header). The serial
/// REST walk fires every tick on the thread that called `step()`; the wide path
/// fires each on a rayon worker. A constraint that is COUNTED but not ENFORCED
/// therefore shows up here on EVERY run, where the ticket order shows it only
/// when a worker wins the race. Reset per run under `#[serial]`.
static PUBLISH_TIDS: Mutex<Vec<(u64, std::thread::ThreadId)>> = Mutex::new(Vec::new());

/// Record the calling thread against `ticket` — see [`PUBLISH_TIDS`].
fn record_publish_thread(ticket: u64) {
    PUBLISH_TIDS
        .lock()
        .expect("PUBLISH_TIDS poisoned")
        .push((ticket, std::thread::current().id()));
}
/// Every `(producer label, ticket)` the wide consumer read off `shared`, in
/// READ order (one entry per consumer fire = per consumed frame).
static WIDE_READS: Mutex<Vec<(u8, u64)>> = Mutex::new(Vec::new());
const LABEL_FAST: u8 = 1;
const LABEL_SLOW: u8 = 2;
/// Bounded, wall-clock-free work the FIRST-declared producer does before it
/// draws its ticket — a real broadcaster composes transforms before it
/// publishes. It BIASES an unconstrained wide path toward an observable mis-order: under
/// rayon the second producer's tick runs on a thief while the first is still
/// burning, so the second tends to draw the lower ticket; under the constraint
/// the first always draws first because the walk is serial.
///
/// A bias, NOT a guarantee — nothing forces rayon to schedule the thief inside
/// the burn, and on a starved runner it may not. That is exactly why the
/// deterministic discriminator is the routing observable (`PUBLISH_TIDS` and
/// `forced_serial_rest_walks`) rather than this burn. Iterations, never time,
/// so it changes WHEN and never WHAT (Principle #7).
const FAST_BIAS_ITERS: u64 = 100_000;

fn burn(iters: u64) -> u64 {
    let mut acc = 0u64;
    for i in 0..iters {
        acc = std::hint::black_box(acc.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(i));
    }
    acc
}

/// The first-declared shared-topic producer: `Period(10)` ⇒ one publish per
/// step; burns [`FAST_BIAS_ITERS`] then draws its ticket.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct TicketedFastSrc {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl TicketedFastSrc {
    fn tick(&mut self) -> Result<(), NodeError> {
        std::hint::black_box(burn(FAST_BIAS_ITERS));
        let ticket = TICK_TICKET.fetch_add(1, Ordering::SeqCst);
        self.out.x = ticket as f64;
        self.out.y = f64::from(LABEL_FAST);
        record_publish_thread(ticket);
        Ok(())
    }
}

/// The second-declared shared-topic producer: `Period(30)` ⇒ every third
/// step; draws its ticket at once.
#[cerulion_node(period_ms = 30)]
#[derive(Default)]
struct TicketedSlowSrc {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl TicketedSlowSrc {
    fn tick(&mut self) -> Result<(), NodeError> {
        let ticket = TICK_TICKET.fetch_add(1, Ordering::SeqCst);
        self.out.x = ticket as f64;
        self.out.y = f64::from(LABEL_SLOW);
        record_publish_thread(ticket);
        Ok(())
    }
}

/// [`AnnotDualSink`]'s shape (trigger `shared` + plain `solo` control) that
/// also RECORDS each consumed shared frame's `(label, ticket)`.
#[cerulion_node]
#[derive(Default)]
struct TicketDualSink {
    #[input(trigger)]
    shared: Vector3,
    #[input]
    solo: Vector3,
}

#[cerulion_node_impl]
impl TicketDualSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let label = self.shared.y as u8;
        let ticket = self.shared.x as u64;
        WIDE_READS
            .lock()
            .expect("WIDE_READS poisoned")
            .push((label, ticket));
        let _ = self.solo.x;
        Ok(())
    }
}

/// The forced fire-pool width for the wide arms: real parallelism on any
/// runner with >= 2 cores (the env override bypasses the
/// `available_parallelism` auto-size, so a 1-core runner still gets 4 workers).
const WIDE_THREADS: &str = "4";
/// Unconsumed `StepSrc` fillers declared BEFORE the shared pair, and AFTER it.
const FILLERS_BEFORE: usize = 7;
const FILLERS_AFTER: usize = 6;
/// The consumer's manifest input table (wiring order: `shared` = 0, `solo` = 1).
const CONSUMER_INPUTS: &[&str] = &["shared", "solo"];
const NO_INPUTS: &[&str] = &[];

/// The wide `/tf` broadcaster shape. Level 0, in declaration order:
/// `FILLERS_BEFORE` fillers, `fast` and `slow` (the two producers of `shared`),
/// `FILLERS_AFTER` fillers, `solo_src` — 16 input-less macro `Period` nodes,
/// so the REST is 15 on an ordinary step and 16 on a `slow` step, both
/// `>= PARALLEL_FIRE_THRESHOLD`. Level 1 is the data-trigger consumer (its
/// `shared` trigger levelizes it below the pair).
///
/// Why the pair sits in the MIDDLE rather than at the front: rayon's `for_each`
/// splits the scheduler's node range in halves, runs the LEFT half on the
/// installing worker and pushes the RIGHT half for another worker to steal.
/// With 17 scheduler nodes the first split lands at index 8, so `fast` (index
/// 7) is the LAST tick of the installing worker's chain while `slow` (index 8)
/// is the FIRST tick of the stolen chain. Under the unconstrained wide path
/// that puts `slow`'s publish AHEAD of `fast`'s on nearly every step both fire
/// — the interleave the constraint forbids — where a pair at indices 0/1 would
/// let rayon's left-first tendency mask the same defect most of the time
/// (measured on this shape). The layout is what BIASES a regression toward
/// a visible mis-order; it is not what makes the kill deterministic — the
/// routing observables (`forced_serial_rest_walks` and the per-tick thread id)
/// are, and neither depends on the layout. The fix is layout-independent too.
fn wide_shared_graph(
    prefix: &str,
    shared: &str,
) -> (
    GraphConfig,
    IndexMap<String, Box<dyn NodeEntry>>,
    Vec<String>,
) {
    let mut defs: Vec<NodeDef> = Vec::new();
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for i in 0..FILLERS_BEFORE {
        let def = src_def(&format!("f{i:02}"));
        factories.insert(def.id.clone(), Box::new(StepSrcEntry::new()));
        defs.push(def);
    }
    let mut fast = src_def("fast");
    fast.outputs[0].topic = Some(shared.to_string());
    factories.insert("fast".to_string(), Box::new(TicketedFastSrcEntry::new()));
    defs.push(fast);
    let mut slow = src_def("slow");
    slow.outputs[0].topic = Some(shared.to_string());
    factories.insert("slow".to_string(), Box::new(TicketedSlowSrcEntry::new()));
    defs.push(slow);
    for i in FILLERS_BEFORE..FILLERS_BEFORE + FILLERS_AFTER {
        let def = src_def(&format!("f{i:02}"));
        factories.insert(def.id.clone(), Box::new(StepSrcEntry::new()));
        defs.push(def);
    }
    let solo_src = src_def("solo_src");
    factories.insert("solo_src".to_string(), Box::new(StepSrcEntry::new()));
    defs.push(solo_src);
    let consumer = sink_def("consumer", &[("shared", shared), ("solo", "solo_src/out")]);
    factories.insert("consumer".to_string(), Box::new(TicketDualSinkEntry::new()));
    defs.push(consumer);
    let ids: Vec<String> = defs.iter().map(|d| d.id.clone()).collect();
    let mut config = graph_of(prefix, defs);
    config.multi_publisher_topics = vec![shared.to_string()];
    (config, factories, ids)
}

/// The hand oracle of the arm above, in run-independent form, plus the tick
/// ticket: `fast` is `Period(10)` on 10 ms steps ⇒ seq k on step k+1; `slow`
/// is `Period(30)` ⇒ seq j on step 3(j+1); a step carrying both reads `fast`
/// FIRST (connection order == graph order); and because the serial walk ticks
/// in exactly this order from a counter reset to 0, each frame's ticket is its
/// POSITION in the stream. Nothing here is read off a run.
fn wide_shared_oracle(steps: u64) -> Vec<(&'static str, u64, u64)> {
    let mut expected = Vec::new();
    for step in 1..=steps {
        let ticket = expected.len() as u64;
        expected.push(("fast", step - 1, ticket));
        if step % 3 == 0 {
            let ticket = expected.len() as u64;
            expected.push(("slow", step / 3 - 1, ticket));
        }
    }
    expected
}

/// Drive the wide shape under a 4-worker fire pool for `steps` steps and return
/// the consumer's shared-edge reads as `("fast" | "slow", served seq, tick
/// ticket)` — the run-INDEPENDENT projection of the kind-6 `(token, seq)`
/// stream (publisher ids, and so the tokens, are minted fresh per run) joined
/// with the `(label, ticket)` the consumer read out of each frame — plus the
/// solo control edge.
///
/// The join is itself a cross-check: read `i`'s token (attributed through the
/// BUILD's publisher table) must name the same producer the frame's own label
/// says wrote it. THREE routing pins live here so BOTH arms carry them — one
/// vacuity guard and two deterministic regression kills:
///
/// - VACUITY GUARD: the pool really is multi-threaded and the level really is
///   wide. It catches no regression; without it every assertion below would pass on a
///   serial-by-SIZE routing and prove nothing about the constraint;
/// - KILL: `forced_serial_rest_walks` equals the step count — level 0 is the only
///   multi-fire level, its REST is `>= PARALLEL_FIRE_THRESHOLD` on every step,
///   and the constraint alone kept it off rayon each time (level 1 is a
///   one-fire fast path and never counts). A disjunct-DELETED variant fails
///   here, at a counter of 0;
/// - KILL: every producer tick ran on the STEPPING thread (`PUBLISH_TIDS`). The
///   COUNTED-BUT-NOT-ENFORCED variant (the counter bumps, PASS 2 still fans out)
///   fails here on every run, because rayon fires each tick on a
///   `cerulion-fire-N` worker and the installing caller never participates in
///   the `for_each`.
///
/// The ticket half of the oracle is the ORDER evidence — what the mis-routing
/// actually costs a drainer — but it is a race outcome (see [`TICK_TICKET`]),
/// so it is deliberately not the half the kills rest on.
fn run_wide_shared(
    tag: &str,
    prefix: &str,
    shared: &str,
    steps: u64,
) -> (Vec<(&'static str, u64, u64)>, Vec<AnnotRec>) {
    let _threads = FireThreadsGuard::set(WIDE_THREADS);
    TICK_TICKET.store(0, Ordering::SeqCst);
    WIDE_READS.lock().expect("WIDE_READS poisoned").clear();
    PUBLISH_TIDS.lock().expect("PUBLISH_TIDS poisoned").clear();
    // `run_annot_recorded_inspecting` drives `step()` on THIS thread, so this
    // is the thread the serial REST walk must fire every tick on.
    let step_tid = std::thread::current().id();
    let (config, factories, ids) = wide_shared_graph(prefix, shared);
    let consumer_idx = (ids.len() - 1) as u32;
    let level0_width = ids.len() - 1;
    let id_refs: Vec<&str> = ids.iter().map(String::as_str).collect();
    let inputs: Vec<&[&str]> = ids
        .iter()
        .map(|id| {
            if id == "consumer" {
                CONSUMER_INPUTS
            } else {
                NO_INPUTS
            }
        })
        .collect();

    let (stream, publishers) = run_annot_recorded_inspecting(
        tag,
        config,
        factories,
        &id_refs,
        &inputs,
        steps,
        |runtime| {
            assert!(
                runtime.fire_pool_thread_count() > 1,
                "the wide arm must route WIDE: a {}-thread pool forces the serial REST \
                 regardless of width (is CERULION_FIRE_THREADS forced >= 2?)",
                runtime.fire_pool_thread_count()
            );
            // On an ordinary step `slow` does not fire: the REST is the level
            // minus one, and that must still be wide.
            let threshold = GraphRuntime::parallel_fire_threshold();
            assert!(
                level0_width > threshold,
                "the wide arm must route WIDE: level 0's smallest REST ({}) must be >= \
                 PARALLEL_FIRE_THRESHOLD ({threshold}); widen FILLERS_* if the threshold rose",
                level0_width - 1
            );
            assert_eq!(
                runtime.forced_serial_rest_walks(),
                steps,
                "the shared-topic constraint must have taken level 0's REST off the rayon \
                 path on EVERY step (0 = the constraint is inert and the pair published \
                 concurrently; more = a one-fire level was counted)"
            );
        },
    );

    let shared_edge: Vec<AnnotRec> = stream
        .iter()
        .copied()
        .filter(|r| r.node_idx == consumer_idx && r.input_idx == 0)
        .collect();
    let solo_edge: Vec<AnnotRec> = stream
        .iter()
        .copied()
        .filter(|r| r.node_idx == consumer_idx && r.input_idx == 1)
        .collect();
    assert!(
        !shared_edge.iter().any(|r| r.kind == READ_OUTCOME_TRUNCATED),
        "this run must not overflow its stage (an overflow marker is a \
         non-annotated record and would confound the pairing): {shared_edge:?}"
    );

    let reads = pair_annotated_reads(&shared_edge);
    let fast_token = expected_token(publisher_id_of(&publishers, "fast", "out"));
    let slow_token = expected_token(publisher_id_of(&publishers, "slow", "out"));
    assert_ne!(
        fast_token, slow_token,
        "the two publishers must mint distinct ids (equal ids would make the \
         labelling below vacuous)"
    );
    let ticketed: Vec<(u8, u64)> =
        std::mem::take(&mut *WIDE_READS.lock().expect("WIDE_READS poisoned"));
    assert_eq!(
        ticketed.len(),
        reads.len(),
        "one consumer fire per consumed frame: the frames the body read \
         ({ticketed:?}) must be exactly the reads the ring recorded ({reads:?})"
    );

    // ROUTING (the deterministic half — see this fn's doc): every shared-topic
    // producer tick ran on the STEPPING thread, which only the serial REST walk
    // does. Under the wide path each tick runs on a `cerulion-fire-N` worker
    // and the installing caller never participates in the `for_each`, so a
    // constraint that is counted but not enforced fails here on EVERY run
    // rather than when a worker happens to win the ticket race.
    let publish_tids: Vec<(u64, std::thread::ThreadId)> =
        std::mem::take(&mut *PUBLISH_TIDS.lock().expect("PUBLISH_TIDS poisoned"));
    assert_eq!(
        publish_tids.len(),
        reads.len(),
        "one producer tick per published frame, and every published frame is \
         consumed on this shape: ticks {publish_tids:?} vs reads {reads:?}"
    );
    let foreign: Vec<(u64, std::thread::ThreadId)> = publish_tids
        .iter()
        .copied()
        .filter(|(_, tid)| *tid != step_tid)
        .collect();
    assert!(
        foreign.is_empty(),
        "every shared-topic producer tick must run on the stepping thread \
         ({step_tid:?}) — the serial REST walk is the ONLY driver that fires \
         on the caller; these ticks ran on a rayon worker, so the level's REST \
         fanned out and the two publishes were unordered: {foreign:?}"
    );
    let labelled = reads
        .iter()
        .zip(&ticketed)
        .map(|(r, &(label, ticket))| {
            let who = if r.token == fast_token {
                "fast"
            } else if r.token == slow_token {
                "slow"
            } else {
                panic!(
                    "a token on the wire ({:#x}) resolves to neither publisher \
                     ({fast_token:#x} / {slow_token:#x}): {reads:?}",
                    r.token
                )
            };
            let self_declared = match label {
                LABEL_FAST => "fast",
                LABEL_SLOW => "slow",
                other => panic!("a frame carries an unknown producer label {other}: {ticketed:?}"),
            };
            assert_eq!(
                who, self_declared,
                "the token the ring attributed read {} to must be the producer the \
                 frame itself says wrote it (seq {}, ticket {ticket})",
                r.seq, r.seq
            );
            (who, r.seq, ticket)
        })
        .collect();
    (labelled, solo_edge)
}

/// THE WIDE-PATH PIN: two same-level producers of one shared topic,
/// on a level wide enough to fire under rayon, still publish in graph order.
///
/// Same read-order oracle as the narrow arm above (`f0 f1 f2 s0 f3 f4 f5 s1 …`),
/// now on a 16-wide level 0 under a 4-worker pool, extended with the TICK
/// TICKET each frame carries: because the level's REST walks serially in
/// insertion order, `fast` ticks (and so publishes) before `slow` on every step
/// both fire and every frame's ticket is its position in the stream. Under an
/// unconstrained wide path `slow` (the first tick of the stolen half) runs while
/// `fast` (the last tick of the installing worker's half) is still burning, so
/// `slow` tends to draw the lower ticket — the read ORDER stays `fast` first
/// regardless (connection order, see the section header), which is why the
/// ticket, not the read order, is the ORDER evidence. The routing pins inside
/// [`run_wide_shared`] (the walk counter and the per-tick thread id) are what
/// make the kills deterministic.
///
/// ANTI-TAUTOLOGY, same recording: the consumer's `solo` edge on an ordinary
/// single-writer topic reads (records exist) and carries ZERO annotations.
#[test]
#[serial]
fn a_wide_level_publishes_a_shared_topic_in_insertion_order() {
    const SHARED: &str = "/rowide/tf";
    const STEPS: u64 = 12;

    let (labelled, solo_edge) = run_wide_shared("wideannot", "rowide", SHARED, STEPS);

    assert_eq!(
        labelled,
        wide_shared_oracle(STEPS),
        "one record per CONSUMED frame, in arrival order, each naming its own \
         producer, and each carrying the tick ticket of its POSITION — `fast` \
         ticks (and so publishes) ahead of `slow` on every step both publish \
         because the level walks in graph order, not in the rayon workers' \
         scheduling order"
    );

    assert!(
        !solo_edge.is_empty(),
        "the control edge must really read (an empty control proves nothing)"
    );
    assert_eq!(
        solo_edge
            .iter()
            .filter(|r| r.kind == READ_OUTCOME_PRODUCER)
            .count(),
        0,
        "a single-writer graph-owned edge carries NO producer annotation — its \
         served sequence already names its one publisher: {solo_edge:?}"
    );
}

/// DETERMINISM on the wide shape: two fresh runtimes (fresh SHM
/// roots, fresh publisher ids, fresh 4-worker pools) record BIT-IDENTICAL
/// `(producer, seq, tick ticket)` streams, and both equal the hand oracle —
/// so the equality is a cross-check, not a self-compare. The projection is by
/// producer LABEL because the raw tokens are minted per run; everything else
/// the ring carries for the edge (kind, seq, order, multiplicity) and the
/// ticket each frame carries are compared verbatim.
#[test]
#[serial]
fn the_wide_shared_topic_interleave_is_bit_identical_across_runs() {
    const STEPS: u64 = 12;

    let (first, _) = run_wide_shared("widedet_a", "rowdeta", "/rowdeta/tf", STEPS);
    let (second, _) = run_wide_shared("widedet_b", "rowdetb", "/rowdetb/tf", STEPS);
    let oracle = wide_shared_oracle(STEPS);

    assert_eq!(first, oracle, "run 1 must match the hand oracle");
    assert_eq!(second, oracle, "run 2 must match the hand oracle");
    assert_eq!(
        first, second,
        "two runs of the wide shared-topic shape must record the same interleave \
         (a rayon-ordered pair would differ run to run)"
    );
}

// ===========================================================================
// The BUILD-TIME verdict pins.
//
// The arms above drive the SCHEDULER: they prove that a level the build
// STAMPED really walks its REST serially. They say nothing about WHICH levels
// the build stamps, and the derivation — the `rest_walk` half of each level's
// `LevelPlan` — has three narrowing conditions whose negative branches nothing
// exercised: the producer must be on THIS level, must be OUTSIDE
// `serial_fire_node_ids` (class-A snapshot-gated or block-involved), and the
// topic must carry `>= 2` such producers. (A fourth, "the verdict is read at
// the level it was derived for", is no longer a condition anyone can get
// wrong: partition and verdict are ONE `LevelPlan` at ONE index.)
//
// These pins read the verdict straight off the built runtime
// (`level_rest_walk_for_test`) — no 4-worker pool, no stepping unless a
// COUNTER claim needs it — so each one names the level it is talking about and
// a wrong index, a dropped filter or a widened count comparison fails with the
// whole Vec printed. Where a counter appears it is driven under the wide pool,
// so a derivation regression that over-stamps shows up as routing, not just as a
// verdict.
// ===========================================================================

/// A relay `NodeDef`: one input wired to `source`, one `Vector3` output "out",
/// optionally overridden to the absolute `topic`.
fn relay_def(id: &str, input: &str, source: &str, topic: Option<&str>) -> NodeDef {
    NodeDef {
        fuse: None,
        ros2: None,
        id: id.to_string(),
        node_type: "ro_relay".to_string(),
        inputs: vec![InputDef {
            name: input.to_string(),
            source: source.to_string(),
        }],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: topic.map(str::to_string),
        }],
    }
}

/// A source `NodeDef` publishing to the absolute `topic` instead of its
/// derived name.
fn src_def_on(id: &str, topic: &str) -> NodeDef {
    let mut def = src_def(id);
    def.outputs[0].topic = Some(topic.to_string());
    def
}

/// One unconsumed `Period(10)` source per `GraphRuntime::parallel_fire_threshold()`,
/// so a level's REST is wide enough that the size gate does not decide the
/// routing for it. Derived from the threshold, never a literal: a bump must
/// widen these pins, not silently route them narrow (the same discipline as the
/// wide arms' width guard).
fn filler_defs(factories: &mut IndexMap<String, Box<dyn NodeEntry>>) -> Vec<NodeDef> {
    (0..GraphRuntime::parallel_fire_threshold())
        .map(|i| {
            let def = src_def(&format!("v{i:02}"));
            factories.insert(def.id.clone(), Box::new(StepSrcEntry::new()));
            def
        })
        .collect()
}

/// BUILD the graph and hand back the per-level `RestWalk` verdict (`true` =
/// `InsertionOrder`). Nothing is stepped: the derivation is a build-time
/// function of the topology, so a pin on it needs no transport traffic.
fn level_walk_of(
    config: GraphConfig,
    factories: IndexMap<String, Box<dyn NodeEntry>>,
) -> Vec<bool> {
    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 32).expect("build graph");
    let walk = runtime.level_rest_walk_for_test();
    runtime.shutdown();
    walk
}

/// [`level_walk_of`] plus `steps` steps under a REAL 4-worker fire pool, so the
/// routing COUNTER is live: a level the derivation wrongly stamps is not merely
/// a wrong `bool`, it is a level taken off rayon on every step.
fn level_walk_and_counter(
    config: GraphConfig,
    factories: IndexMap<String, Box<dyn NodeEntry>>,
    steps: u64,
) -> (Vec<bool>, u64) {
    let _threads = FireThreadsGuard::set(WIDE_THREADS);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 32).expect("build graph");
    assert!(
        runtime.fire_pool_thread_count() > 1,
        "these arms must route WIDE: a {}-thread pool forces the serial REST \
         regardless of the verdict",
        runtime.fire_pool_thread_count()
    );
    let walk = runtime.level_rest_walk_for_test();
    for _ in 0..steps {
        runtime.step(Duration::from_millis(10));
    }
    let counter = runtime.forced_serial_rest_walks();
    runtime.shutdown();
    (walk, counter)
}

/// The verdict names the RIGHT level.
///
/// A 3-level graph whose shared pair sits on level 1: `head` publishes on
/// level 0, two relays trigger off it and BOTH publish the listed topic on
/// level 1, and a trigger consumer of that topic sits on level 2. The hand
/// oracle is `[false, true, false]`.
///
/// Every arm above drives a graph whose only multi-producer level is level 0,
/// so a DERIVATION that stamped the wrong position is invisible to them — a
/// reversal maps level 0 to itself. Here the stamped level is the MIDDLE one,
/// flanked by two unstamped levels, so the verdict is asymmetric in exactly the
/// way an index bug is.
///
/// Scope: this reads `level_plans` in order, so it sees a wrong
/// DERIVED position and nothing else. A wrong-index read at the DISPATCH
/// (`step` handing PASS 2 another level's `rest_walk`) is caught by the wide
/// e2e arms, whose stamped level would take an unstamped neighbour's verdict
/// and fan out. A skew between a level's PARTITION and its VERDICT is caught by
/// neither, because it is no longer representable: both are one `LevelPlan`.
#[test]
#[serial]
fn the_build_stamps_the_level_that_holds_the_shared_pair() {
    const SHARED: &str = "/rogap5/tf";
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("head".to_string(), Box::new(StepSrcEntry::new()));
    factories.insert("relay_a".to_string(), Box::new(TrigRelayEntry::new()));
    factories.insert("relay_b".to_string(), Box::new(TrigRelayEntry::new()));
    factories.insert("sink".to_string(), Box::new(TrigSinkEntry::new()));
    let mut config = graph_of(
        "rogap5",
        vec![
            src_def("head"),
            relay_def("relay_a", "inp", "head/out", Some(SHARED)),
            relay_def("relay_b", "inp", "head/out", Some(SHARED)),
            sink_def("sink", &[("inp", SHARED)]),
        ],
    );
    config.multi_publisher_topics = vec![SHARED.to_string()];

    assert_eq!(
        level_walk_of(config, factories),
        vec![false, true, false],
        "only level 1 holds two non-serial-gated producers of the listed topic \
         — level 0 holds one producer of a different topic, level 2 only a \
         consumer"
    );
}

/// THREE producers on one level still stamp.
///
/// `>= 2` is the condition, not `== 2`: the hazard is "more than one publish
/// into one FIFO from one level", and it does not stop at two. A `== 2` slip
/// would leave the WORST shape — a busy `/tf` with three broadcasters —
/// silently back on the rayon path, and every other arm in this file uses
/// exactly two producers, so nothing else can see it.
///
/// Level 0 carries the three producers, level 1 a relay off the shared topic,
/// level 2 its consumer: the oracle `[true, false, false]` also re-pins the
/// index from the other side (the stamped level is the FIRST one here, the
/// middle one in the sibling arm above).
///
/// Level 0 also carries a SECOND listed topic with a single in-graph producer,
/// which is the last-write-wins guard: the verdict is assigned inside the loop
/// over topics, so a non-qualifying topic that ASSIGNED `Unconstrained` rather
/// than skipping would unstamp the level purely by iteration order.
#[test]
#[serial]
fn three_producers_on_one_level_are_stamped_like_two() {
    const SHARED: &str = "/rogap4/tf";
    const LONELY: &str = "/rogap4/solo_tf";
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let mut defs = Vec::new();
    for id in ["p0", "p1", "p2"] {
        factories.insert(id.to_string(), Box::new(StepSrcEntry::new()));
        defs.push(src_def_on(id, SHARED));
    }
    // A SECOND listed topic on the SAME level with ONE in-graph producer. The
    // derivation loops over every topic and assigns the verdict inside that
    // loop, so a last-write-wins bug (assigning `Unconstrained` on a topic that
    // does not qualify, instead of `continue`-ing) would UNSTAMP level 0 here
    // depending only on iteration order. Without this producer the level has
    // exactly one qualifying topic and the bug is invisible.
    factories.insert("lonely".to_string(), Box::new(StepSrcEntry::new()));
    defs.push(src_def_on("lonely", LONELY));
    factories.insert("lonely_sink".to_string(), Box::new(PollSinkEntry::new()));
    defs.push(sink_def("lonely_sink", &[("inp", LONELY)]));
    factories.insert("relay".to_string(), Box::new(TrigRelayEntry::new()));
    defs.push(relay_def("relay", "inp", SHARED, None));
    factories.insert("sink".to_string(), Box::new(TrigSinkEntry::new()));
    defs.push(sink_def("sink", &[("inp", "relay/out")]));
    let mut config = graph_of("rogap4", defs);
    config.multi_publisher_topics = vec![SHARED.to_string(), LONELY.to_string()];

    assert_eq!(
        level_walk_of(config, factories),
        vec![true, false, false],
        "three same-level producers of one listed topic are the same hazard as \
         two — the count test is `>= 2`, not `== 2` — and a sibling listed topic \
         with ONE producer on that same level must neither stamp it nor UNSTAMP it"
    );
}

/// The gap-2/gap-3 fixture: `head` + fillers + `fast` (a producer of the
/// listed topic), a relay that triggers off `head` and ALSO publishes it, and a
/// trigger consumer. Under Kahn the two producers are on DIFFERENT levels
/// (`fast` on 0 with the fillers, `relay` on 1); a `level_assignments:` block
/// can co-level them.
fn gap2_graph(prefix: &str, shared: &str) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let mut defs = filler_defs(&mut factories);
    factories.insert("head".to_string(), Box::new(StepSrcEntry::new()));
    defs.push(src_def("head"));
    factories.insert("fast".to_string(), Box::new(StepSrcEntry::new()));
    defs.push(src_def_on("fast", shared));
    factories.insert("relay".to_string(), Box::new(TrigRelayEntry::new()));
    defs.push(relay_def("relay", "inp", "head/out", Some(shared)));
    factories.insert("sink".to_string(), Box::new(TrigSinkEntry::new()));
    defs.push(sink_def("sink", &[("inp", shared)]));
    let mut config = graph_of(prefix, defs);
    config.multi_publisher_topics = vec![shared.to_string()];
    (config, factories)
}

/// Two producers of one listed topic on
/// DIFFERENT levels stamp NOTHING.
///
/// The constraint is about a single level's REST: two producers the DAG already
/// separates cannot interleave, because level 0's REST is fully merged before
/// level 1 fires. Dropping `level.nodes.contains(p)` from the derivation makes
/// the test global — every level of every graph with a two-producer listed
/// topic gets stamped — and the level-0 REST here is wide, so that variant does
/// not merely flip a `bool`: it takes a genuinely wide level off rayon on every
/// step, which the counter reads back as `steps` instead of 0.
#[test]
#[serial]
fn producers_split_across_levels_stamp_no_level() {
    const STEPS: u64 = 6;
    let (config, factories) = gap2_graph("rogap2", "/rogap2/tf");

    let (walk, counter) = level_walk_and_counter(config, factories, STEPS);
    assert_eq!(
        walk,
        vec![false, false, false],
        "each level holds exactly ONE producer of the listed topic, so no level \
         can interleave two publishes"
    );
    assert_eq!(
        counter, 0,
        "no level was stamped, so the constraint took nothing off the rayon \
         path — a global (level-blind) producer test would read {STEPS}"
    );
}

/// The derivation reads the RESOLVED levels,
/// so a `level_assignments:` block that co-levels two producers is honoured.
///
/// Same nodes and same wiring as the sibling arm above; the one change is a
/// hand-written `level_assignments:` map putting `fast` on level 1 beside
/// `relay`. `GraphRuntime` levelizes through `resolve_levels`, which routes to
/// the assignment map when one is present — a derivation written against
/// `GraphTopology::derive_levels` instead would compute Kahn's levels, see the
/// producers separated, and stamp nothing while the EXECUTOR happily fires them
/// together. That derivation is invisible to every other arm in this file, because
/// no other arm carries an assignment block.
///
/// The counter is deliberately NOT the oracle here: the co-leveled pair is a
/// 2-fire REST, which `rest_driver` ranks `SerialBySize` long before the
/// constraint is consulted, so the counter correctly stays 0 and only the
/// verdict discriminates.
#[test]
#[serial]
fn a_level_assignments_block_that_co_levels_producers_is_honoured() {
    const SHARED: &str = "/rogap3/tf";
    let (mut config, factories) = gap2_graph("rogap3", SHARED);
    // EVERY node must be covered and the levels contiguous (a partial map is
    // rejected): fillers + `head` on 0, the producer pair on 1, the consumer
    // of their shared topic on 2.
    let mut assignments: indexmap::IndexMap<String, usize> = indexmap::IndexMap::new();
    for node in &config.nodes {
        let level = match node.id.as_str() {
            "fast" | "relay" => 1,
            "sink" => 2,
            _ => 0,
        };
        assignments.insert(node.id.clone(), level);
    }
    config.level_assignments = Some(assignments);

    assert_eq!(
        level_walk_of(config, factories),
        vec![false, true, false],
        "the assignment block puts both producers of the listed topic on level \
         1, so THAT level is stamped — the Kahn levelization of the identical \
         graph stamps nothing (the sibling arm)"
    );
}

/// The gap-1 fixture: fillers + two producers of a listed topic + the
/// `consumers` the caller supplies (the MIXED arm passes two). Everything is
/// level 0 — each consumer's input is plain (non-trigger), which does not
/// levelize.
fn gap1_graph(
    prefix: &str,
    shared: &str,
    consumers: Vec<(&str, Box<dyn NodeEntry>)>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let mut defs = filler_defs(&mut factories);
    for id in ["fast", "slow"] {
        factories.insert(id.to_string(), Box::new(StepSrcEntry::new()));
        defs.push(src_def_on(id, shared));
    }
    for (id, entry) in consumers {
        defs.push(sink_def(id, &[("inp", shared)]));
        factories.insert(id.to_string(), entry);
    }
    let mut config = graph_of(prefix, defs);
    config.multi_publisher_topics = vec![shared.to_string()];
    (config, factories)
}

/// A `block` consumer takes the shared topic
/// OUT of the constraint's scope, and the drop_oldest twin puts it back.
///
/// A `block` topic's producers AND consumers are `block_involved_nodes`, which
/// `serial_fire_node_ids` absorbs, so they fire in PASS 1 through the fused
/// decide+tick seam — in graph order already, and never in the REST this
/// constraint governs. Stamping such a level would be double-handling: it would
/// take a level off rayon to order publishes that were never on the rayon path.
///
/// Three graphs, identical but for the consumer's backpressure policy, each
/// with a REST wide enough to route wide on its own:
/// - `block` only ⇒ `[false]`, counter 0;
/// - `drop_oldest` only ⇒ `[true]`, counter `STEPS` (the ANTI-TAUTOLOGY half:
///   without it, a derivation that stamped nothing at all would pass);
/// - MIXED (both consumers on one topic) ⇒ `[false]`, counter 0 — `block_topics`
///   holds any topic SOME node consumes as `block`, so the producers stay
///   block-involved even though the topic also feeds a lossy reader.
#[test]
#[serial]
fn a_block_consumer_takes_the_shared_topic_out_of_the_constraint() {
    const STEPS: u64 = 6;

    let (config, factories) = gap1_graph(
        "rogap1b",
        "/rogap1b/tf",
        vec![(
            "blocker",
            Box::new(BlockSinkEntry::new()) as Box<dyn NodeEntry>,
        )],
    );
    let (walk, counter) = level_walk_and_counter(config, factories, STEPS);
    assert_eq!(
        walk,
        vec![false],
        "an all-`block` listed topic's producers fire in PASS 1, not the REST — \
         the level must NOT be stamped"
    );
    assert_eq!(counter, 0, "an unstamped level takes nothing off rayon");

    let (config, factories) = gap1_graph(
        "rogap1d",
        "/rogap1d/tf",
        vec![(
            "poller",
            Box::new(PollSinkEntry::new()) as Box<dyn NodeEntry>,
        )],
    );
    let (walk, counter) = level_walk_and_counter(config, factories, STEPS);
    assert_eq!(
        walk,
        vec![true],
        "CONTROL: the same graph with a lossy consumer leaves both producers in \
         the REST, which is exactly the shape the constraint governs"
    );
    assert_eq!(
        counter, STEPS,
        "the control's level is wide on every step and the constraint alone \
         kept it off rayon each time"
    );

    let (config, factories) = gap1_graph(
        "rogap1m",
        "/rogap1m/tf",
        vec![
            (
                "blocker",
                Box::new(BlockSinkEntry::new()) as Box<dyn NodeEntry>,
            ),
            (
                "poller",
                Box::new(PollSinkEntry::new()) as Box<dyn NodeEntry>,
            ),
        ],
    );
    let (walk, counter) = level_walk_and_counter(config, factories, STEPS);
    assert_eq!(
        walk,
        vec![false],
        "MIXED: one `block` consumer is enough to make the topic block-involved, \
         so its producers stay out of the REST and the level stays unstamped"
    );
    assert_eq!(counter, 0, "an unstamped level takes nothing off rayon");
}

/// The plan's PARTITION and its VERDICT describe
/// the SAME level.
///
/// That is the whole property the `LevelPlan` merge buys, and the verdict-only
/// accessor cannot see it: three parallel `Vec`s and one `Vec<LevelPlan>` are
/// indistinguishable if you only ever read one field. `level_partition_for_test`
/// hands back all three halves at one index, so this arm can assert they AGREE
/// about which level they are — a plan pushed with one level's partition and
/// another's verdict fails here and nowhere else.
///
/// The shape makes the two halves point in OPPOSITE directions per level, so a
/// swap cannot be masked by symmetry:
/// - level 0 = the BLOCK group (two producers of a `block`-consumed listed
///   topic + its consumer, all block-involved) ⇒ non-empty `block_ids`, verdict
///   FALSE;
/// - level 1 = a WIDE non-block shared pair (a filler-widened level of two
///   producers of a second listed topic, triggered off level 0) ⇒ EMPTY
///   `block_ids`, verdict TRUE.
#[test]
#[serial]
fn a_levels_partition_and_its_verdict_describe_the_same_level() {
    const BLOCKED: &str = "/rojoint/blocked_tf";
    const SHARED: &str = "/rojoint/tf";
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    let mut defs: Vec<NodeDef> = Vec::new();

    // Level 0: the block group. `head` also drives the level-1 relays.
    for id in ["b0", "b1"] {
        factories.insert(id.to_string(), Box::new(StepSrcEntry::new()));
        defs.push(src_def_on(id, BLOCKED));
    }
    factories.insert("blocker".to_string(), Box::new(BlockSinkEntry::new()));
    defs.push(sink_def("blocker", &[("inp", BLOCKED)]));
    factories.insert("head".to_string(), Box::new(StepSrcEntry::new()));
    defs.push(src_def("head"));

    // Level 1: two relays publishing ONE listed topic, widened past the
    // threshold by fillers that also trigger off `head` (so they levelize with
    // the pair rather than sitting on level 0).
    for id in ["r0", "r1"] {
        factories.insert(id.to_string(), Box::new(TrigRelayEntry::new()));
        defs.push(relay_def(id, "inp", "head/out", Some(SHARED)));
    }
    for i in 0..GraphRuntime::parallel_fire_threshold() {
        let id = format!("w{i:02}");
        factories.insert(id.clone(), Box::new(TrigRelayEntry::new()));
        defs.push(relay_def(&id, "inp", "head/out", None));
    }

    // Level 2: a consumer of the shared topic, so it is not a dangling listing.
    factories.insert("sink".to_string(), Box::new(TrigSinkEntry::new()));
    defs.push(sink_def("sink", &[("inp", SHARED)]));

    let mut config = graph_of("rojoint", defs);
    config.multi_publisher_topics = vec![BLOCKED.to_string(), SHARED.to_string()];

    let clock = Arc::new(VirtualClock::new());
    let runtime =
        GraphRuntime::build_for_test(config, factories, clock, 32).expect("build joint graph");
    let plans = runtime.level_partition_for_test();
    let verdicts = runtime.level_rest_walk_for_test();
    runtime.shutdown();

    assert_eq!(
        plans.len(),
        3,
        "three DAG levels: block group, relays, sink"
    );

    // Level 0 — the block group, and NOT stamped.
    let (other0, block0, walk0) = &plans[0];
    let mut block0_sorted = block0.clone();
    block0_sorted.sort();
    assert_eq!(
        block0_sorted,
        vec!["b0".to_string(), "b1".to_string(), "blocker".to_string()],
        "level 0's block partition is the producers AND the consumer of the \
         `block` topic"
    );
    assert_eq!(
        other0,
        &vec!["head".to_string()],
        "level 0's non-block partition is exactly the node off the block topic"
    );
    assert!(
        !walk0,
        "the block group's producers fire fused in PASS 1, so level 0 is not stamped"
    );

    // Level 1 — the wide non-block shared pair, and STAMPED.
    let (other1, block1, walk1) = &plans[1];
    assert!(
        block1.is_empty(),
        "level 1 touches no `block` topic, so its block partition is empty: {block1:?}"
    );
    assert!(
        other1.contains(&"r0".to_string()) && other1.contains(&"r1".to_string()),
        "level 1's non-block partition carries both shared-topic producers: {other1:?}"
    );
    assert!(
        other1.len() > GraphRuntime::parallel_fire_threshold(),
        "level 1 must be wide enough that the size gate is not what routes it \
         ({} nodes vs threshold {})",
        other1.len(),
        GraphRuntime::parallel_fire_threshold()
    );
    assert!(
        walk1,
        "level 1 holds two non-serial-gated producers of one listed topic"
    );

    // Level 2 — consumer only.
    let (_, block2, walk2) = &plans[2];
    assert!(block2.is_empty() && !walk2, "the consumer level is plain");

    // The verdict-only accessor and the joint one must agree — they read the
    // same `LevelPlan`s, and a disagreement would mean one of them indexes
    // something else.
    assert_eq!(
        verdicts,
        plans.iter().map(|(_, _, w)| *w).collect::<Vec<bool>>(),
        "both test seams read the same per-level plans"
    );
}

/// The build
/// `info!` is the PRODUCTION evidence that a level's routing changed, and it
/// tells the truth about whether the stamp COST anything.
///
/// `forced_serial_rest_walks` is reachable only through a `cfg`-gated accessor,
/// so on a shipping robot this line is the only record that exists. It was
/// unpinned; and it had ONE wording, which claimed the level "forgoes
/// within-level parallelism" even on a level whose REST can never reach the
/// threshold — a cost that does not exist there (the narrow arm is exactly that
/// shape: stamped, counter 0). So the build now decides between two wordings,
/// and this arm pins BOTH plus the silence, in ONE capture:
///
/// - the WIDE drop_oldest control graph ⇒ exactly one line saying `instead of
///   under rayon`;
/// - the NARROW 4-node graph ⇒ exactly one line saying `would have walked
///   serially anyway`;
/// - the BLOCK twin of the wide graph ⇒ ZERO lines naming its topic at all
///   (the anti-tautology half: without it, "announce on every build" passes).
///
/// Each line must carry `INFO`, its level and its topic — an operator greps by
/// those, and a line naming neither could not be acted on — plus the `rest` and
/// `threshold` the no-cost claim rests on.
#[test]
#[serial]
#[traced_test]
fn a_stamped_level_announces_its_routing_at_info() {
    // (a) WIDE + stamped: the routing really changed.
    let (wide, wide_factories) = gap1_graph(
        "roinfow",
        "/roinfow/tf",
        vec![(
            "poller",
            Box::new(PollSinkEntry::new()) as Box<dyn NodeEntry>,
        )],
    );
    assert_eq!(
        level_walk_of(wide, wide_factories),
        vec![true],
        "precondition: the drop_oldest control graph stamps its one level"
    );

    // (b) NARROW + stamped: the stamp is real, the forgone parallelism is not.
    let mut fast = src_def("fast");
    fast.outputs[0].topic = Some("/roinfon/tf".to_string());
    let mut slow = src_def("slow");
    slow.outputs[0].topic = Some("/roinfon/tf".to_string());
    let mut narrow = graph_of(
        "roinfon",
        vec![fast, slow, sink_def("sink", &[("inp", "/roinfon/tf")])],
    );
    narrow.multi_publisher_topics = vec!["/roinfon/tf".to_string()];
    let mut narrow_factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    narrow_factories.insert("fast".to_string(), Box::new(StepSrcEntry::new()));
    narrow_factories.insert("slow".to_string(), Box::new(StepSrcEntry::new()));
    narrow_factories.insert("sink".to_string(), Box::new(TrigSinkEntry::new()));
    assert_eq!(
        level_walk_of(narrow, narrow_factories),
        vec![true, false],
        "precondition: the narrow graph stamps level 0 too (counter stays 0 — see \
         `a_narrow_shared_level_is_stamped_but_not_credited`)"
    );

    // (c) The BLOCK twin of (a): same shape, one build-time property apart.
    let (blocked, blocked_factories) = gap1_graph(
        "roinfob",
        "/roinfob/tf",
        vec![(
            "blocker",
            Box::new(BlockSinkEntry::new()) as Box<dyn NodeEntry>,
        )],
    );
    assert_eq!(
        level_walk_of(blocked, blocked_factories),
        vec![false],
        "precondition: the block twin stamps nothing"
    );

    logs_assert(|lines: &[&str]| {
        // Filter on the ROUTING MARKER, not the topic: a build logs the topic
        // on several unrelated lines (the multi-publisher cap notice, every
        // `publisher created`), so a topic-only filter would count those and an
        // "exactly one" oracle would be measuring the wrong thing.
        const COST: &str = "instead of under rayon";
        const NO_COST: &str = "would have walked serially anyway";
        let with = |needle: &str| -> Vec<String> {
            lines
                .iter()
                .filter(|l| l.contains(needle))
                .map(|l| (*l).to_string())
                .collect()
        };

        let cost_lines = with(COST);
        if cost_lines.len() != 1 {
            return Err(format!(
                "exactly ONE build in this body stamps a level whose REST would have \
                 gone wide, so exactly one line may claim the constraint took it off \
                 rayon; got {}: {cost_lines:?}",
                cost_lines.len()
            ));
        }
        for want in ["INFO", "level=0", "/roinfow/tf", "rest=11", "threshold=8"] {
            if !cost_lines[0].contains(want) {
                return Err(format!(
                    "the wide line must carry {want}: {}",
                    cost_lines[0]
                ));
            }
        }

        let no_cost_lines = with(NO_COST);
        if no_cost_lines.len() != 1 {
            return Err(format!(
                "exactly ONE build in this body stamps a level the size gate had \
                 already routed serially; got {}: {no_cost_lines:?}",
                no_cost_lines.len()
            ));
        }
        for want in ["INFO", "level=0", "/roinfon/tf", "rest=2", "threshold=8"] {
            if !no_cost_lines[0].contains(want) {
                return Err(format!(
                    "the no-cost line must carry {want}: {}",
                    no_cost_lines[0]
                ));
            }
        }

        // ANTI-TAUTOLOGY: nothing stamped ⇒ nothing announced. Scoped to the
        // block twin's own topic so it cannot be satisfied by the two lines
        // above.
        let blocked: Vec<&String> = cost_lines
            .iter()
            .chain(no_cost_lines.iter())
            .filter(|l| l.contains("/roinfob/tf"))
            .collect();
        if !blocked.is_empty() {
            return Err(format!(
                "a build that stamps NO level must announce nothing: {blocked:?}"
            ));
        }
        Ok(())
    });
}

/// A class-(A) serial-gated producer does not
/// count toward the pair.
///
/// `serial_fire_node_ids` has TWO reasons; the arm above covers reason (B)
/// (block-involved). Reason (A) is a node with non-trigger inputs whose
/// `snapshot_inputs` is a no-op: a `ClosureNodeEntry` or an older cdylib.
/// Such a node fires in PASS 1, ahead of the whole REST, so its publish is
/// ordered already and the level's REST holds only ONE producer of the topic.
///
/// The two graphs differ in exactly one build-time property: the closure entry
/// (`performs_input_snapshot() == false` ⇒ serial-gated) versus a macro node
/// with the same ports and the same wiring (`true` ⇒ in the REST). Dropping
/// `!serial_fire_node_ids.contains(p)` from the derivation makes both read
/// `[true, ..]`.
#[test]
#[serial]
fn a_serial_gated_producer_does_not_count_toward_the_shared_pair() {
    const STEPS: u64 = 6;

    fn ctx_graph(
        prefix: &str,
        shared: &str,
        gated: bool,
    ) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
        let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
        let mut defs = filler_defs(&mut factories);
        factories.insert("ctx_src".to_string(), Box::new(StepSrcEntry::new()));
        defs.push(src_def("ctx_src"));
        factories.insert("plain_prod".to_string(), Box::new(StepSrcEntry::new()));
        defs.push(src_def_on("plain_prod", shared));
        // The node under test: a plain (non-trigger) context input + an output
        // on the shared topic. Same ports and same wiring either way.
        let mut under_test = relay_def("under_test", "ctx", "ctx_src/out", Some(shared));
        under_test.node_type = "ro_ctx".to_string();
        defs.push(under_test);
        if gated {
            let info = NodeInfo::with_meta(
                vec![InputMeta {
                    name: "ctx".to_string(),
                    schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
                    trigger: false,
                    depth: 8,
                    backpressure: BackpressurePolicy::DropOldest,
                    expect_within_ms: None,
                }],
                vec![OutputMeta::new(
                    "out".to_string(),
                    <Vector3 as ShmMessage>::SCHEMA_HASH,
                    <Vector3 as ShmMessage>::MAX_SLICE_LEN,
                )],
            )
            .with_policy(MacroPolicy::Period { period_ms: 10 });
            factories.insert(
                "under_test".to_string(),
                Box::new(ClosureNodeEntry::new(info, |_ctx| Ok(())).with_label("gated_producer")),
            );
        } else {
            factories.insert("under_test".to_string(), Box::new(CtxSrcEntry::new()));
        }
        factories.insert("sink".to_string(), Box::new(PollSinkEntry::new()));
        defs.push(sink_def("sink", &[("inp", shared)]));
        let mut config = graph_of(prefix, defs);
        config.multi_publisher_topics = vec![shared.to_string()];
        (config, factories)
    }

    let (config, factories) = ctx_graph("rogap8g", "/rogap8g/tf", true);
    let (walk, counter) = level_walk_and_counter(config, factories, STEPS);
    assert_eq!(
        walk,
        vec![false],
        "the closure producer is serial-gated (no-op snapshot + a non-trigger \
         input), so the level's REST holds ONE producer of the listed topic"
    );
    assert_eq!(counter, 0, "an unstamped level takes nothing off rayon");

    let (config, factories) = ctx_graph("rogap8m", "/rogap8m/tf", false);
    let (walk, counter) = level_walk_and_counter(config, factories, STEPS);
    assert_eq!(
        walk,
        vec![true],
        "CONTROL: the macro twin performs its own input snapshot, so BOTH \
         producers reach the REST and the level is stamped"
    );
    assert_eq!(
        counter, STEPS,
        "the control's level is wide on every step and the constraint alone \
         kept it off rayon each time"
    );
}

/// A DECIMATED read that SERVES
/// the held frame carries that frame's producer.
///
/// `snapshot_latest` classifies a gate-dropped fresh frame as `Decimated` and
/// leaves the frozen slot at `Held`, so the body reads the held frame that
/// step — and on a `multi_publisher_topics` edge a per-publisher sequence names
/// no publisher on its own, which is the entire reason the `Held` arm one match
/// arm below carries an annotation. Without this the ONE read path that serves
/// a frame under a non-`Held` kind was the one read an offline reader could not
/// attribute.
///
/// The oracle is [`pair_annotated_reads`], which enforces the binding rule for
/// EVERY read in the run it is given: an unannotated read on an annotated edge
/// panics with the offending stream printed. It is fed the SUFFIX from the
/// first annotation, because the reads BEFORE it are the pre-delivery
/// `NoFrame`s — those serve nothing and correctly carry no producer, which the
/// arm asserts separately as the boundary.
#[test]
#[serial]
fn a_decimated_read_that_serves_a_held_frame_is_annotated_with_its_producer() {
    const SHARED: &str = "/rosamp/tf";
    const STEPS: u64 = 12;

    let mut fast = src_def("fast");
    fast.outputs[0].topic = Some(SHARED.to_string());
    let mut slow = src_def("slow");
    slow.outputs[0].topic = Some(SHARED.to_string());
    let solo_src = src_def("solo_src");
    let consumer = sink_def("consumer", &[("shared", SHARED), ("solo", "solo_src/out")]);
    let mut config = graph_of("rosamp", vec![fast, slow, solo_src, consumer]);
    config.multi_publisher_topics = vec![SHARED.to_string()];

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fast".to_string(), Box::new(StepSrcEntry::new()));
    factories.insert("slow".to_string(), Box::new(SlowSrcEntry::new()));
    factories.insert("solo_src".to_string(), Box::new(StepSrcEntry::new()));
    factories.insert(
        "consumer".to_string(),
        Box::new(AnnotSampledSinkEntry::new()),
    );

    let (stream, publishers) = run_annot_recorded(
        "sampannot",
        config,
        factories,
        &["fast", "slow", "solo_src", "consumer"],
        &[&[], &[], &[], &["shared", "solo"]],
        STEPS,
    );

    // node_idx 3 = "consumer" (manifest order); input_idx 0 = "shared",
    // 1 = "solo" (wiring order).
    let shared_edge: Vec<AnnotRec> = stream
        .iter()
        .copied()
        .filter(|r| r.node_idx == 3 && r.input_idx == 0)
        .collect();
    let solo_edge: Vec<AnnotRec> = stream
        .iter()
        .copied()
        .filter(|r| r.node_idx == 3 && r.input_idx == 1)
        .collect();

    // Nothing overflowed, so the pairing walk sees only annotations + reads.
    assert!(
        !shared_edge.iter().any(|r| r.kind == READ_OUTCOME_TRUNCATED),
        "this run must not overflow its stage (an overflow marker is a \
         non-annotated record and would confound the pairing): {shared_edge:?}"
    );

    // THE BOUNDARY: every read BEFORE the first annotation serves nothing —
    // `held_sample` is still None, so the drain's Empty records `NoFrame` and
    // carries no producer. Once a frame is delivered `held_sample` is `Some`
    // forever, so an Empty can never classify `NoFrame` again: the unannotated
    // reads are a strict PREFIX.
    let first_annot = shared_edge
        .iter()
        .position(|r| r.kind == READ_OUTCOME_PRODUCER)
        .unwrap_or_else(|| {
            panic!("the multi-publisher edge must carry an annotation: {shared_edge:?}")
        });
    assert!(
        shared_edge[..first_annot]
            .iter()
            .all(|r| r.kind == READ_OUTCOME_NONE),
        "a read that serves nothing carries no producer — the unannotated \
         prefix is exactly the pre-delivery NoFrames: {shared_edge:?}"
    );

    // PRECONDITION: the arm is about `Decimated` reads, so the run must have
    // produced some AFTER the first delivery (an empty set would make every
    // assertion below vacuously true).
    let suffix = &shared_edge[first_annot..];
    let decimated = suffix
        .iter()
        .filter(|r| r.kind == READ_OUTCOME_DECIMATED)
        .count();
    assert!(
        decimated > 0,
        "the sample(25) gate must have decimated at least once past the first \
         delivery, or this arm proves nothing: {shared_edge:?}"
    );

    // THE PIN: the binding rule holds for EVERY read from here on, Decimated
    // included — annotation first, matching join key, then the read.
    let reads = pair_annotated_reads(suffix);
    assert_eq!(
        reads
            .iter()
            .filter(|r| r.kind == READ_OUTCOME_DECIMATED)
            .count(),
        decimated,
        "every Decimated read that serves the held frame must be annotated: \
         {suffix:?}"
    );

    // The tokens RESOLVE through the BUILD's own publisher table. A subset
    // rather than set-equality: which publisher's frame the gate happens to
    // accept is a timing fact, so requiring BOTH to appear would be a flake.
    let fast_token = expected_token(publisher_id_of(&publishers, "fast", "out"));
    let slow_token = expected_token(publisher_id_of(&publishers, "slow", "out"));
    assert_ne!(
        fast_token, slow_token,
        "the two publishers must mint distinct ids (equal ids would make the \
         resolution assertion vacuous)"
    );
    let resolvable: BTreeSet<u64> = [fast_token, slow_token].into_iter().collect();
    let observed: BTreeSet<u64> = reads.iter().map(|r| r.token).collect();
    assert!(
        !observed.is_empty() && observed.is_subset(&resolvable),
        "every token on the wire must resolve through the manifest publisher \
         table ({:#x} / {:#x}): {observed:?}",
        publisher_id_of(&publishers, "fast", "out"),
        publisher_id_of(&publishers, "slow", "out"),
    );

    // ANTI-TAUTOLOGY: the single-writer sibling edge reads, and is NOT
    // annotated — so "annotations appear" is the multi-publisher bit's doing,
    // not a runtime that annotates every read.
    assert!(
        !solo_edge.is_empty(),
        "the control edge must really read (an empty control proves nothing)"
    );
    assert_eq!(
        solo_edge
            .iter()
            .filter(|r| r.kind == READ_OUTCOME_PRODUCER)
            .count(),
        0,
        "a single-writer graph-owned edge carries NO producer annotation: \
         {solo_edge:?}"
    );
}

/// P1-5 — THE EXTERNAL ARM: an absolute `source:` with no in-graph producer.
///
/// `PublisherProvisioning::External` caps NOTHING, so N foreign writers may
/// legitimately attach — the canonical instance being `/tf` under
/// `ros2 attach`, i.e. the very route the multi-writer sequence hazard is
/// named for. Left unflagged, the shape that needs the annotation MOST got
/// none, silently: it is not `multi_publisher_topics:`-listed, so the
/// first cut of `edge_needs_producer_annotation` said no.
///
/// The topic here is deliberately NOT listed (`multi_publisher_topics` is
/// empty), so the annotation can only come from the External arm. Two REAL
/// foreign publishers attach and both stream; the oracle is the same
/// resolution seam as P1-4, with the ids read off the TEST's own publishers
/// (they are not the graph's, so they are absent from the build's table —
/// which is itself the correct shape: offline, such a token resolves to
/// `foreign(...)`, and the annotation's value there is that the reader can
/// still tell TWO producers apart).
///
/// ANTI-TAUTOLOGY, same body and same recording: the plain sibling edge on an
/// ordinary single-writer graph-owned topic reads and is NOT annotated.
#[test]
#[serial]
fn an_external_sourced_edge_carries_producer_tokens_for_its_foreign_writers() {
    const EXT: &str = "/roext/mp";
    const STEPS: u64 = 8;

    let solo_src = src_def("solo_src");
    let consumer = sink_def("consumer", &[("shared", EXT), ("solo", "solo_src/out")]);
    // NOT listed: the annotation must come from External provisioning alone.
    let config = graph_of("roext", vec![solo_src, consumer]);
    assert!(
        config.multi_publisher_topics.is_empty(),
        "the External arm is the one under test — listing the topic would make \
         this arm a second copy of the multi_publisher one"
    );

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("solo_src".to_string(), Box::new(StepSrcEntry::new()));
    factories.insert("consumer".to_string(), Box::new(AnnotDualSinkEntry::new()));

    let ring_tag = format!("extannot_{}", std::process::id());
    let mut owner = TraceRingOwner::create_with_inputs(
        &ring_tag,
        default_capacity_records(),
        0,
        &["solo_src", "consumer"],
        &[&[], &["shared", "solo"]],
    )
    .expect("create ring");
    let ring_name = owner.name().to_string();
    let producer_handle = owner.producer().expect("mint producer");

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 32).expect("build graph");
    let mgr = Arc::clone(runtime.test_transport().expect("test transport parked"));
    runtime.set_trace_ring_producer(
        producer_handle,
        &["solo_src".to_string(), "consumer".to_string()],
    );

    // TWO foreign writers on the producer-less absolute topic — legal exactly
    // because External provisioning imposes no single-writer cap.
    let mut writer_a = mgr
        .create_publisher(EXT, MaxSliceLen::const_new(256), 0)
        .expect("foreign writer A must attach to an External topic");
    let mut writer_b = mgr
        .create_publisher(EXT, MaxSliceLen::const_new(256), 0)
        .expect("foreign writer B must attach (no single-writer cap)");
    let token_a = expected_token(writer_a.publisher_id());
    let token_b = expected_token(writer_b.publisher_id());
    assert_ne!(
        token_a, token_b,
        "the two foreign writers must mint distinct ids (the token is the \
         offline discriminator)"
    );

    for i in 1..=STEPS {
        {
            let mut proxy = writer_a.loan_proxy::<Vector3>().expect("loan A");
            proxy.x = i as f64;
        }
        {
            let mut proxy = writer_b.loan_proxy::<Vector3>().expect("loan B");
            proxy.x = -(i as f64);
        }
        runtime.step(Duration::from_millis(10));
    }

    let mut ring_consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
    let mut records = Vec::new();
    ring_consumer.drain(&mut records).expect("drain ring");
    drop(ring_consumer);
    runtime.shutdown();
    drop(owner);
    let stream = annot_stream(&records);

    // node_idx 1 = "consumer"; input_idx 0 = "shared" (the External edge),
    // 1 = "solo" (the single-writer control).
    let ext_edge: Vec<AnnotRec> = stream
        .iter()
        .copied()
        .filter(|r| r.node_idx == 1 && r.input_idx == 0)
        .collect();
    let solo_edge: Vec<AnnotRec> = stream
        .iter()
        .copied()
        .filter(|r| r.node_idx == 1 && r.input_idx == 1)
        .collect();

    assert!(
        !ext_edge.iter().any(|r| r.kind == READ_OUTCOME_TRUNCATED),
        "this run must not overflow its stage: {ext_edge:?}"
    );
    let reads = pair_annotated_reads(&ext_edge);
    assert!(
        !reads.is_empty(),
        "the External edge must have consumed frames: {ext_edge:?}"
    );

    let observed: BTreeSet<u64> = reads.iter().map(|r| r.token).collect();
    let resolvable: BTreeSet<u64> = [token_a, token_b].into_iter().collect();
    assert_eq!(
        observed,
        resolvable,
        "every token on the External edge must be one of the two foreign \
         writers' (ids {:#x} / {:#x}), and BOTH must be seen — the annotation \
         is what tells two writers on one producer-less topic apart",
        writer_a.publisher_id(),
        writer_b.publisher_id(),
    );

    // THE CONDITION, DEMONSTRATED: `sequence` is a PER-PUBLISHER counter, so
    // both writers start at 0 and their streams COLLIDE — the same served
    // sequence arrives from two different producers, and the token is the
    // only thing that tells them apart. (A run that never collided would
    // make the annotation look redundant on this edge.)
    let collided = reads.iter().any(|lhs| {
        reads
            .iter()
            .any(|rhs| lhs.seq == rhs.seq && lhs.token != rhs.token)
    });
    assert!(
        collided,
        "two foreign writers on one topic must produce COLLIDING served \
         sequences (each counts from 0) — that collision is the whole reason \
         a served sequence alone names no producer: {reads:?}"
    );

    // MULTIPLICITY + ORDER — the same blind spot the P1-4 arm closes, and it
    // matters more here: a `BTreeSet` of two tokens is satisfied by ANY
    // interleaving of ANY multiplicity, so a runtime that staged every
    // `(Producer, read)` pair twice would pass every assertion above
    // (`pair_annotated_reads` still walks a well-formed `P,R,P,R` stream, and
    // the collision check only gets easier). The oracle is DERIVED from the
    // stimulus: each iteration commits A then B before stepping, both writers
    // count from 0, and the topic is FIFO — so the run is
    // `A0 B0 A1 B1 …`, one record per consumed frame.
    let mut expected: Vec<(u64, u64)> = Vec::new();
    for i in 0..STEPS {
        expected.push((token_a, i));
        expected.push((token_b, i));
    }
    let observed_pairs: Vec<(u64, u64)> = reads.iter().map(|r| (r.token, r.seq)).collect();
    assert_eq!(
        observed_pairs, expected,
        "one record per CONSUMED frame, in arrival order, each naming the \
         foreign writer that committed it (A {token_a:#x}, B {token_b:#x})"
    );

    assert!(
        !solo_edge.is_empty(),
        "the control edge must really read (an empty control proves nothing)"
    );
    assert_eq!(
        solo_edge
            .iter()
            .filter(|r| r.kind == READ_OUTCOME_PRODUCER)
            .count(),
        0,
        "a single-writer graph-owned edge carries NO producer annotation: \
         {solo_edge:?}"
    );
}

/// The P2-8 harness: a recording whose manifest OMITS the consumer node — a
/// manifest/scheduler desync, which is the one condition under which
/// `TraceRingHook::push_read_outcome` REFUSES a record and reports the
/// refusal back through `drain_into`'s `bool`.
///
/// Returns `(refusals in the FIRST window, refusals across the quiet windows,
/// lifetime drop total, kind-6 records that reached the ring)`. `stage_first`
/// opens an OVERFLOW window first by staging that many reads; `None` is the
/// healthy control.
///
/// # Why the reads are STAGED rather than fired
///
/// The rim is unreachable from a graph (see
/// `truncated_consumed_frame_reads_as_a_marked_hole_not_silence` for the
/// derivation), so the overflow is staged directly. The consumer here is a
/// data-trigger node whose producer NEVER publishes, so it never fires and the
/// graph stages nothing — which makes every offer counted below one of this
/// function's own staged records, or the re-offered marker, and nothing else.
///
/// Each QUIET window stages EXACTLY ONE read. That is load-bearing twice over:
/// a stage with nothing pending is not drained at all (`drain_into` returns at
/// its first line), so without it a quiet window would not be a merge window
/// and the re-offer could not happen either way; and one is exactly enough to
/// make the window's ordinary offer count a hand-written `1`.
fn run_unmapped_consumer(
    tag: &str,
    stage_first: Option<u64>,
    quiet_steps: u64,
) -> (u64, u64, u64, usize, u32) {
    let (config, factories) = two_node_graph(
        tag,
        Box::new(SilentSrcEntry::new()),
        Box::new(TrigSinkEntry::new()),
    );
    let ring_tag = format!("{tag}_{}", std::process::id());
    // The manifest — and therefore `set_trace_ring_producer`'s node table —
    // names ONLY the producer. "consumer" is unmapped, so every kind-6 record
    // it stages is refused at the hook.
    let mut owner = TraceRingOwner::create_with_inputs(
        &ring_tag,
        default_capacity_records(),
        0,
        &["producer"],
        &[&[]],
    )
    .expect("create ring");
    let ring_name = owner.name().to_string();
    let producer_handle = owner.producer().expect("mint producer");
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build graph");
    runtime.set_trace_ring_producer(producer_handle, &["producer".to_string()]);
    assert_eq!(
        runtime.read_outcome_stages_armed_for_test(),
        1,
        "the stage must ARM even though its node is unmapped — otherwise \
         nothing is staged and the refusal path is never reached"
    );

    // This edge's DERIVED capacity — the rim the hand oracles below
    // are computed against.
    let capacity = {
        // "exactly one stage" is the claim, so ASSERT it: `.first()` would
        // silently read stage 0 of several and the oracle below would be
        // computed against a rim that is not the only one in play.
        let stages = runtime.read_outcome_stage_capacities("consumer");
        assert_eq!(
            stages.len(),
            1,
            "the consumer owns exactly one read-outcome stage: {stages:?}"
        );
        stages[0].capacity
    };

    // Step 1: either the overflow window or a matching healthy one.
    if let Some(reads) = stage_first {
        assert_eq!(
            runtime.stage_read_outcomes_for_test("consumer", reads as usize),
            1,
            "the reads were staged on exactly the one stage this edge owns"
        );
    }
    runtime.step(Duration::from_millis(10));
    let refused_in_first = runtime.trace_ring_unmapped_read_outcomes();

    // Step 2: N windows that drop NOTHING NEW — one staged read each, so the
    // window really merges and its ordinary offer count is exactly one.
    for _ in 0..quiet_steps {
        assert_eq!(
            runtime.stage_read_outcomes_for_test("consumer", 1),
            1,
            "the quiet window's one read was staged on this edge's stage"
        );
        runtime.step(Duration::from_millis(10));
    }
    let refused_over_quiet = runtime.trace_ring_unmapped_read_outcomes() - refused_in_first;
    let (dropped_total, _) = runtime.read_outcome_dropped_report();

    let mut ring_consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
    let mut records = Vec::new();
    ring_consumer.drain(&mut records).expect("drain ring");
    drop(ring_consumer);
    runtime.shutdown();
    drop(owner);
    let kind6 = records
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_READ_OUTCOME)
        .count();
    (
        refused_in_first,
        refused_over_quiet,
        dropped_total,
        kind6,
        capacity,
    )
}

/// P2-8 — THE OVERFLOW MARKER'S DEBT IS RETIRED ONLY ON A CONFIRMED
/// HAND-OVER, so a REFUSED marker is RE-OFFERED in the next window.
///
/// `ReadOutcomeStage::drain_into` used to advance `dropped_reported` the
/// instant the marker was OFFERED. Its consumer can REFUSE it —
/// `push_read_outcome` returns without writing when the node id is missing
/// from the recording manifest — and with no in-memory sink installed that is
/// every consumer there is. Retiring on the offer therefore lost the marker
/// AND the debt in one move: the hole stayed in the bag and nothing would
/// ever describe it again, because the next window's delta is computed
/// against the count that line just advanced. Retiring on ACCEPTANCE means
/// the count accumulates into the next window instead.
///
/// # What is observable
///
/// The only refusing consumer in the system refuses records PER NODE ID, so
/// on the run that exercises the refusal NOTHING from that node reaches the
/// ring — the re-offered marker's COUNT cannot be read off the wire (both
/// runs' bags are asserted empty of kind-6 records here, which is what makes
/// that concrete rather than assumed). What IS observable is the OFFER, via
/// `GraphRuntime::trace_ring_unmapped_read_outcomes` — the hook's per-record
/// refusal counter — and re-offering IS the accumulation: the offered count
/// is `dropped - dropped_reported`, so an offer in a window that dropped
/// nothing new can only happen if the earlier debt was never retired.
///
/// # The oracle is HAND-WRITTEN on both sides
///
/// Two runs of ONE graph differing in ONE input: an overflowing first window
/// vs a healthy one. The graph itself stages nothing (a data-trigger consumer
/// whose producer never publishes never fires), and each quiet window stages
/// EXACTLY ONE read — so a quiet window's ordinary offer count is a hand-written
/// `1` on both runs rather than a number that merely cancels. The correct
/// implementation offers exactly ONE EXTRA record per quiet window (the
/// re-offered marker); a broken one offers none, and the control's own exact
/// count is asserted first, so the comparison cannot pass on two silences.
#[test]
#[serial]
fn a_refused_overflow_marker_is_re_offered_in_the_following_window() {
    const STAGED: u64 = 700;
    const QUIET: u64 = 3;

    // CONTROL: the same graph, the same quiet windows, no overflow.
    let (first_control, quiet_control, dropped_control, ring_control, _) =
        run_unmapped_consumer("nomark", None, QUIET);
    assert_eq!(
        dropped_control, 0,
        "the control must not overflow (it is the no-marker baseline)"
    );
    assert_eq!(
        first_control, 0,
        "the control's first window stages nothing, so it offers nothing — \
         which is also the pin that the GRAPH contributes no records here"
    );
    assert_eq!(
        quiet_control, QUIET,
        "each quiet window offers EXACTLY the one read it staged: the windows \
         really merge, and nothing else is offered on this edge"
    );

    // OVERFLOW: the first window is staged past the stage capacity, and its
    // marker is REFUSED because the node is unmapped.
    let (first_overflow, quiet_overflow, dropped_overflow, ring_overflow, capacity) =
        run_unmapped_consumer("remark", Some(STAGED), QUIET);
    // The rim is this edge's DERIVED capacity — asserted to really be
    // outrun, so the oracles below are not vacuous.
    assert!(
        (capacity as u64) < STAGED,
        "the stimulus must outrun the derived capacity: staged {STAGED} against {capacity}"
    );
    assert_eq!(
        dropped_overflow,
        STAGED - capacity as u64,
        "hand oracle: {STAGED} staged reads against the capacity"
    );
    assert_eq!(
        first_overflow,
        capacity as u64 + 1,
        "the overflow window offers its CAPACITY survivors plus ONE marker, \
         and every one of them is refused"
    );

    // Neither run bags a single kind-6 record — the refusal really is total,
    // which is why the OFFER counter (not the wire) is the observable.
    assert_eq!(
        (ring_control, ring_overflow),
        (0, 0),
        "an unmapped node's records never reach the ring"
    );

    // THE PIN: every quiet window after a REFUSED marker offers exactly one
    // more record than the same window on the control — the re-offered marker.
    assert_eq!(
        quiet_overflow,
        quiet_control + QUIET,
        "a refused marker's debt must survive into the next window and be \
         re-offered there (one extra offer per quiet window). Retiring the debt \
         on the OFFER makes these equal, and the hole is then described by \
         nothing: control {quiet_control}, overflow {quiet_overflow}"
    );
}

// ===========================================================================
// The per-rank BOUNDARY GUARD, pinned
// as a REPLAY-SIDE contract.
//
// The guard is `CerulionSubscriber::drain_for_trigger`'s `EachFifo` held-head
// early return: a boundary never pops over a head no tick has served, it
// RE-OFFERS it and records NOTHING. That was first pinned as a recording property
// (one kind-6 record per CONSUMED frame). The trace-driven fire plan depends on it
// as a REPLAY property: the executor consumes these records to steer injection, and the
// "next pop proves consumption" forensic lemma — the only thing that dates
// a consumption offline, since kind 6 carries no fire — is a direct corollary.
// Nothing in the repo asserted the lemma, so a guard regression would show up
// as a wrong replay verdict rather than as a red test.
// ===========================================================================

/// One popped frame, decoded from the read log: `(pop step, wire seq)`.
fn pops_of(kind6: &[ReadRec]) -> Vec<(u64, u64)> {
    kind6.iter().map(|r| (r.step, r.seq)).collect()
}

/// The consumer's FIRE steps, from the trace ring's own fire stream — an
/// INDEPENDENT oracle (a different record kind, written by a different seam),
/// which is what makes the join below a cross-check rather than a self-compare.
fn consumer_fire_steps(all: &[TraceRingRecord]) -> Vec<u64> {
    let mut steps: Vec<u64> = all
        .iter()
        .filter(|r| r.record_type == cerulion_core::trace_ring::RECORD_TYPE_FIRE && r.node_idx == 1)
        .map(|r| r.step)
        .collect();
    steps.sort_unstable();
    steps
}

/// THE per-rank boundary-guard pin: a `throttle_ms` consumer holding ONE FIFO head
/// across several boundaries records exactly ONE row per CONSUMED frame, Σ
/// popped equals the frames consumed plus the held tail, and pops and
/// consumptions strictly ALTERNATE.
///
/// Hand trace (`StepSrc` publishes seq k at step k, t = (k+1)·10 ms;
/// `throttle_ms = 25`; the first fire is unthrottled; a boundary re-offer mints
/// no fresh arrival, so after each fire the burst's refill pops the next frame
/// and freezes it):
///
///   step 0 (t=10): pop 0 → DB(0,1); FIRE (serves 0); refill finds the queue
///                  empty (nothing else published yet) and records nothing.
///   step 1 (t=20): pop 1 → DB(1,1); deferred (10 < 25).
///   step 2 (t=30): head 1 held → re-offer, NO record; deferred (20 < 25).
///   step 3 (t=40): re-offer, NO record; FIRE (30 ≥ 25, serves 1); the refill
///                  pops 2 → DB(2,1) and freezes it; deferred again (0 < 25).
///   steps 4,5:     head 2 held → re-offer, NO record; deferred.
///   step 6 (t=70): FIRE (serves 2); refill pops 3 → DB(3,1), frozen.
///   steps 7,8:     held → no record; deferred.
///   step 9 (t=100): FIRE (serves 3); refill pops 4 → DB(4,1), frozen — the
///                  TAIL: popped, never consumed inside the window.
///
/// So 5 records for 4 consumptions + 1 held tail, and the head is held across
/// TWO boundaries at a time — which is the "several boundaries" the obligation
/// names, asserted below rather than assumed.
#[test]
#[serial]
fn a_held_head_across_boundaries_records_one_row_per_consumed_frame() {
    const STEPS: u64 = 10;
    let (kind6, all, _inputs) = run_recorded(
        "guard",
        Box::new(StepSrcEntry::new()),
        Box::new(ThrottleTrigSinkEntry::new()),
        STEPS,
    );

    let expected = vec![
        rec(0, READ_OUTCOME_DRAINED_BATCH, 0, 1),
        rec(1, READ_OUTCOME_DRAINED_BATCH, 1, 1),
        rec(3, READ_OUTCOME_DRAINED_BATCH, 2, 1),
        rec(6, READ_OUTCOME_DRAINED_BATCH, 3, 1),
        rec(9, READ_OUTCOME_DRAINED_BATCH, 4, 1),
    ];
    assert_eq!(
        kind6, expected,
        "ONE record per CONSUMED frame at its pop step — every held-head \
         re-offer (steps 2, 4, 5, 7, 8) records NOTHING. Deleting the EachFifo \
         held-head early return pops a fresh frame over the unserved head at \
         every boundary: surplus rows, and the served seqs walk ahead."
    );

    let pops = pops_of(&kind6);
    let fires = consumer_fire_steps(&all);
    assert_eq!(
        fires,
        vec![0, 3, 6, 9],
        "the throttle's deterministic fire set (VirtualClock — the hand trace)"
    );

    // Σ popped == frames consumed + the held tail. Every row pops exactly one
    // frame (per-message FIFO), and the consumer's tick reads its input on
    // every fire, so "frames consumed" is the FIRE count — read off the other
    // record stream, never off the one under test.
    let total_popped: u32 = kind6.iter().map(|r| r.popped).sum();
    assert_eq!(total_popped as usize, pops.len());
    let tail = pops.len() - fires.len();
    assert_eq!(
        tail, 1,
        "the last pop (seq 4, step 9) is a refilled head frozen for a fire that \
         never came inside the window — the §15 `tail` clause"
    );

    // THE ALTERNATION, which is what the boundary guard actually buys: pops and
    // consumptions interleave 1:1, so the i-th pop is consumed by the i-th fire
    // and that fire falls inside `[pop_i, pop_{i+1}]`. This is the offline join
    // — the read log alone cannot date a consumption (kind 6 carries no fire),
    // and this is the property that lets the fire stream date it.
    for (i, fire) in fires.iter().enumerate() {
        let (pop_step, seq) = pops[i];
        assert!(
            pop_step <= *fire,
            "frame seq {seq} cannot be consumed (step {fire}) before it was popped (step {pop_step})"
        );
        if let Some((next_pop, _)) = pops.get(i + 1) {
            assert!(
                *fire <= *next_pop,
                "frame seq {seq} was consumed at step {fire}, AFTER the next pop \
                 (step {next_pop}) — the guard forbids popping over an unserved head"
            );
        }
    }

    // The window is REAL: at least one frame is held across two boundaries.
    // Without this the alternation above is satisfiable by a run in which every
    // frame is consumed at its own pop step (the control below), i.e. by a graph
    // that never exercises the guard at all.
    let widest = fires
        .iter()
        .enumerate()
        .map(|(i, fire)| fire - pops[i].0)
        .max()
        .expect("at least one fire");
    assert!(
        widest >= 2,
        "the throttled head must be held across SEVERAL boundaries (widest hold \
         was {widest} steps) — otherwise this arm is not testing the guard"
    );

    // The forensic lemma is `consumed_step = next_pop − 1`. On
    // the UNIFIED path the within-step REFILL falsifies it: the pop that
    // follows a consumption usually happens in the SAME step (the burst loop
    // refills straight after the fire), not at the following boundary. Measured
    // here, BOTH cases occur in one window — which is why the docs-pass form has
    // to be the interval `consumed ∈ [pop, next_pop]` above, not the `−1`.
    let minus_one_holds: Vec<bool> = fires
        .iter()
        .enumerate()
        .filter_map(|(i, fire)| pops.get(i + 1).map(|(next, _)| *fire + 1 == *next))
        .collect();
    assert_eq!(
        minus_one_holds,
        vec![true, false, false, false],
        "the `next_pop − 1` form holds only where the next pop was a BOUNDARY \
         pop (frame 0); after a refill pop the consumption is in the SAME step \
         as the next pop"
    );
}

/// The ANTI-TAUTOLOGY control: the same producer and the same window with the
/// throttle REMOVED. No head is ever held across a boundary, so every frame is
/// popped and consumed in its own step, nothing is left in the tail — and the
/// `−1` form holds throughout, which is the contrast that shows the
/// falsification above belongs to the HELD-HEAD shape rather than to the
/// harness.
#[test]
#[serial]
fn without_a_throttle_no_head_is_held_and_pop_equals_consume() {
    const STEPS: u64 = 10;
    let (kind6, all, _inputs) = run_recorded(
        "guardctl",
        Box::new(StepSrcEntry::new()),
        Box::new(TrigSinkEntry::new()),
        STEPS,
    );

    let expected: Vec<ReadRec> = (0..STEPS)
        .map(|k| rec(k, READ_OUTCOME_DRAINED_BATCH, k, 1))
        .collect();
    assert_eq!(
        kind6, expected,
        "one publish per step, consumed in the step it is popped"
    );

    let pops = pops_of(&kind6);
    let fires = consumer_fire_steps(&all);
    assert_eq!(fires, (0..STEPS).collect::<Vec<_>>());
    assert_eq!(
        pops.len(),
        fires.len(),
        "nothing is held: no popped-but-unconsumed tail"
    );
    for (i, fire) in fires.iter().enumerate() {
        assert_eq!(
            pops[i].0, *fire,
            "with no throttle a frame is consumed in the step it is popped"
        );
        if let Some((next_pop, _)) = pops.get(i + 1) {
            assert_eq!(
                *fire + 1,
                *next_pop,
                "and here the §15 `consumed = next_pop − 1` form holds exactly"
            );
        }
    }
}

/// The `run_recorded` harness driven by a TRACE-DRIVEN fire plan
/// — one plan per step, installed before it. `plans[k]` is step `k`'s recorded
/// fires as `(node id, first fire ns, fire count, interval ns)`.
///
/// Returns the decoded kind-6 records, the consumer's FIRE count (read off the
/// ring's own fire stream — the independent oracle for "the fire happened at
/// all"), and the runtime's replay observables (per-node refill shortfalls for
/// `producer`/`consumer`, and the unconsumed list after the last step).
#[allow(clippy::type_complexity)]
fn run_recorded_planned(
    tag: &str,
    producer: Box<dyn NodeEntry>,
    consumer: Box<dyn NodeEntry>,
    plans: &[Vec<(&str, u64, u32, u64)>],
) -> (Vec<ReadRec>, usize, (u64, u64), Vec<String>) {
    let (config, factories) = two_node_graph(&format!("ro{tag}"), producer, consumer);
    let node_ids = ["producer", "consumer"];
    let node_id_strings: Vec<String> = node_ids.iter().map(|s| s.to_string()).collect();
    let ring_tag = format!("{tag}_{}", std::process::id());
    let mut owner = TraceRingOwner::create_with_inputs(
        &ring_tag,
        default_capacity_records(),
        0,
        &node_ids,
        &[&[], &["inp"]],
    )
    .expect("create ring");
    let ring_name = owner.name().to_string();
    let producer_handle = owner.producer().expect("mint producer");

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build graph");
    runtime.set_trace_ring_producer(producer_handle, &node_id_strings);

    for (step, plan) in plans.iter().enumerate() {
        let fires: Vec<cerulion_core::scheduler::ReplayFire<'_>> = plan
            .iter()
            .map(|(node_id, first_fire_ns, fire_count, interval_ns)| {
                cerulion_core::scheduler::ReplayFire {
                    node_id,
                    first_fire_ns: *first_fire_ns,
                    fire_count: *fire_count,
                    interval_ns: *interval_ns,
                }
            })
            .collect();
        runtime
            .set_replay_fire_plan(step as u64, &fires)
            .expect("install plan");
        runtime.step(Duration::from_millis(10));
    }
    let shortfalls = (
        runtime.replay_refill_shortfalls("producer").unwrap_or(0),
        runtime.replay_refill_shortfalls("consumer").unwrap_or(0),
    );
    let unconsumed: Vec<String> = runtime
        .unconsumed_replay_fires()
        .into_iter()
        .map(str::to_string)
        .collect();
    assert_eq!(
        runtime.replay_plan_mismatches(),
        0,
        "the harness installs one plan per step, so nothing may run stale"
    );

    let mut ring_consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
    let mut records = Vec::new();
    ring_consumer.drain(&mut records).expect("drain ring");
    drop(ring_consumer);
    runtime.shutdown();
    drop(owner);

    let kind6 = records
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_READ_OUTCOME)
        .map(|r| {
            let (input_idx, kind) = unpack_read_outcome_meta(r.global_level);
            ReadRec {
                step: r.step,
                node_idx: r.node_idx,
                input_idx,
                kind,
                seq: r.fire_time_ns,
                popped: unpack_read_outcome_popped(r.duration_ns),
            }
        })
        .collect();
    let consumer_fires = records
        .iter()
        .filter(|r| r.record_type == cerulion_core::trace_ring::RECORD_TYPE_FIRE && r.node_idx == 1)
        .count();
    (kind6, consumer_fires, shortfalls, unconsumed)
}

/// A TRACE-DRIVEN burst drives the refill hook EXPLICITLY, so the
/// k-th replayed fire reads the k-th recorded frame.
///
/// A live `Data` burst discovers its length by asking the hook; a replayed one
/// is TOLD the length and drives the hook purely to pop. Without that, the
/// second fire re-reads the head — which the read log shows directly, because
/// the refill's pop stages its own record: one row instead of two, and the
/// consumer silently serves seq 0 twice.
///
/// `FastSrc` (Period 5) commits TWO frames in the step, and the plan names the
/// recorded burst for BOTH nodes — under a plan nothing fires unless the
/// recording says it did, producer included.
#[test]
#[serial]
fn a_trace_driven_burst_refills_between_fires_so_each_reads_its_own_frame() {
    let plans = vec![vec![
        // the producer's recorded 2-fire catch-up burst (5 ms apart)
        ("producer", 5_000_000, 2, 5_000_000),
        // the consumer's recorded 2-fire data burst (one stamp, the live shape)
        ("consumer", 10_000_000, 2, 0),
    ]];
    let (kind6, consumer_fires, shortfalls, unconsumed) = run_recorded_planned(
        "planrefill",
        Box::new(FastSrcEntry::new()),
        Box::new(TrigSinkEntry::new()),
        &plans,
    );

    assert_eq!(
        kind6,
        vec![
            rec(0, READ_OUTCOME_DRAINED_BATCH, 0, 1),
            rec(0, READ_OUTCOME_DRAINED_BATCH, 1, 1),
        ],
        "the boundary pops the FIFO head (seq 0) and the burst's refill pops \
         seq 1 — without the explicit refill drive the second fire re-reads the \
         head and only ONE row is staged"
    );
    assert_eq!(
        consumer_fires, 2,
        "the recording holds two fires for the consumer in this step"
    );
    assert_eq!(
        shortfalls,
        (0, 0),
        "every frame the recording says was consumed really was there"
    );
    assert!(unconsumed.is_empty(), "both planned bursts were performed");
}

/// …and a refill that finds NOTHING still fires, counting the shortfall.
///
/// The plan is authoritative for the fire SCHEDULE: skipping the fire would turn
/// an input-injection shortfall into a fabricated fire-schedule divergence,
/// blaming the candidate's control flow for the harness's missing frame. So the
/// fire happens (the node re-reads its held head, which surfaces later as a
/// frame-content divergence) and
/// `GraphRuntime::replay_refill_shortfalls` names the real cause.
#[test]
#[serial]
fn a_trace_driven_burst_whose_frame_is_missing_fires_and_counts_the_shortfall() {
    // The producer's recorded burst is 2 frames; the consumer's recording claims
    // THREE consumed frames — one more than this replay's input stream holds.
    let plans = vec![vec![
        ("producer", 5_000_000, 2, 5_000_000),
        ("consumer", 10_000_000, 3, 0),
    ]];
    let (kind6, consumer_fires, shortfalls, unconsumed) = run_recorded_planned(
        "planshort",
        Box::new(FastSrcEntry::new()),
        Box::new(TrigSinkEntry::new()),
        &plans,
    );

    assert_eq!(
        kind6,
        vec![
            rec(0, READ_OUTCOME_DRAINED_BATCH, 0, 1),
            rec(0, READ_OUTCOME_DRAINED_BATCH, 1, 1),
        ],
        "two frames existed, so two pops are staged — the third refill popped \
         nothing and a silent drain records nothing"
    );
    assert_eq!(
        consumer_fires, 3,
        "ALL THREE recorded fires happen — a burst that stopped at the empty \
         refill would fire twice and report a fire-schedule divergence the \
         candidate did not cause"
    );
    assert_eq!(
        shortfalls.1, 1,
        "the third fire's refill found no frame: counted, not swallowed"
    );
    assert!(
        unconsumed.is_empty(),
        "the fire happened anyway — the plan owns the SCHEDULE, and a missing \
         input must not masquerade as a missing fire"
    );
}

/// TIGHT-window per-set Sync sink whose FIRST trigger input is `sample(15)`-GATED
/// — the fixture that makes the matcher's own pops come back DECIMATED.
///
/// The window is 15 ms for the same reason [`TightSyncSink`]'s is (a
/// two-step-old head is provably unmatchable against a 10 ms step cadence), and
/// the GATE is 15 ms so that a decimation decision and a matcher verdict can be
/// derived from the same arithmetic: with frames stamped every 10 ms, a frame
/// one step behind the last ACCEPTED one is rejected and one two steps behind
/// is admitted.
#[cerulion_node(sync_window_ms = 15)]
#[derive(Default)]
struct GatedTightSyncSink {
    #[input(trigger, backpressure = sample(15))]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
}

#[cerulion_node_impl]
impl GatedTightSyncSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.a.x;
        let _ = self.b.x;
        Ok(())
    }
}

/// The per-set Sync matcher's two `Decimated` mint
/// arms are DRAIN-site reads, read back off a real run.
///
/// # The gap this closes
///
/// `the_sync_matchers_descent_pops_are_drain_site_reads` covers FOUR of
/// the six `ReadSiteRole::Drain` mints in `sync_peek_next_stamp` /
/// `sync_discard_head` and states the remaining two in its own scope
/// paragraph: the `Decimated` arms, which need a `sample(N)` gate on a Sync
/// TRIGGER input so the matcher's own pop comes back decimated. Nothing read
/// those two roles back — a variant that flips them would be caught only
/// INDIRECTLY, off this crate, through `replay_engine_test`'s byte-exactness
/// gate, which fails naming a `read_log_divergence` rather than a role.
///
/// It reaches the THIRD drain-side `Decimated` mint as well, inside
/// `drain_for_trigger` — the boundary drain's own decimation, whose ROLE no arm
/// in this file read either (arm (k) drives that site but asserts kinds only).
///
/// # The stimulus, derived rather than observed
///
/// `pa` publishes every step (wire stamps 0, 10, 20, … ms — the payload IS the
/// step), `pb` every third (steps 2 and 5), the window is 15 ms and the gate is
/// `sample(15)` on `a`. Step by step:
///
/// * **0** — `a`'s boundary drain pops a@0; the gate's first frame is always
///   admitted, so it is a `DrainedBatch` and becomes the frozen head. `b` is
///   silent ⇒ the matcher WAITS, so from here on `a`'s head is HELD and its
///   boundary re-offers it without draining (no record) while a@10, a@20 …
///   QUEUE. That queue is what the matcher pops from.
/// * **2** — b@20 arrives. Heads are a@0 and b@20: a 20 ms span against a 15 ms
///   window ⇒ the window DEATH that drives `sync_discard_head`'s refill. The
///   refill pops a@10, the gate rejects it (10 ms since the last admitted
///   frame, under 15) ⇒ **`sync_discard_head`'s `Decimated` mint**.
/// * **3** — the head is empty, so the boundary drain pops a@20 (20 ms since
///   a@0 ⇒ ADMITTED). Heads a@20/b@20 span 0, which is already minimal, so the
///   matcher fires the set with no descent probe. The within-step
///   burst then REFILLS between fires — through `refill_for_trigger`, i.e.
///   `drain_for_trigger` again — and that pop takes a@30, which the gate
///   rejects (10 ms since a@20) ⇒ **`drain_for_trigger`'s `Decimated` mint**.
/// * **4** — the head is empty and a@30 is GONE (the refill consumed it), so
///   the boundary drain pops a@40 — 20 ms since a@20 ⇒ ADMITTED. `b` is silent
///   ⇒ the matcher waits and a@40 is HELD.
/// * **5** — b@50 arrives; `a`'s head is held, so its boundary re-offers without
///   draining. Heads a@40/b@50 span 10 — in-window but NOT zero — and `b`
///   provably has no second frame, so the descent probes `a` for the frame
///   behind its head. That probe POPS a@50 and the gate rejects it (10 ms since
///   a@40) ⇒ **`sync_peek_next_stamp`'s `Decimated` mint**.
///
/// # Attribution
///
/// All three sites mint the same `(Decimated, Drain)` pair, so the kind and the
/// role cannot say which one wrote a row. Two things separate them. In the test
/// body, `sync_unmatched_discard_count == 1` is the step-2 window DEATH that
/// drives `sync_discard_head`'s refill, and it is 0 on the partner — so the
/// step-2 row is the discard's. The step-3 and step-5 rows are separated by
/// evidence, not argument: flipping the two MATCHER mints (and only
/// those) to `Body` moves the step-2 and step-5 rows while
/// leaving step 3 at `Drain` — which is the evidence that step 3 belongs to
/// `drain_for_trigger` (the only other `Drain`-role `Decimated` mint there is)
/// and step 5 to the peek. Whether the step-2/step-5 split is discard/peek
/// rather than the reverse rests on the counter above and the walk, not on
/// isolating each site separately: the two mints were flipped together.
#[test]
#[serial]
fn the_sync_matchers_decimated_pops_are_drain_site_reads() {
    const STEPS: u64 = 6;
    let config = graph_of(
        "roledec",
        vec![
            src_def("pa"),
            src_def("pb"),
            sink_def("fuse", &[("a", "pa/out"), ("b", "pb/out")]),
        ],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("pa".to_string(), Box::new(StepSrcEntry::new()));
    factories.insert("pb".to_string(), Box::new(SlowSrcEntry::new()));
    factories.insert("fuse".to_string(), Box::new(GatedTightSyncSinkEntry::new()));
    let mut counters = None;
    let (_kind6, all, _inputs) = run_recorded_driven(
        "roledec",
        config,
        factories,
        &["pa", "pb", "fuse"],
        &[&[], &[], &["a", "b"]],
        |runtime, _clock| {
            for _ in 0..STEPS {
                runtime.step(Duration::from_millis(10));
            }
            let h = runtime.node_handle("fuse").expect("node handle");
            counters = Some((
                h.sync_unmatched_discard_count("/roledec/pa/out"),
                h.sync_unmatched_discard_count("/roledec/pb/out"),
                h.backpressure_sampled_count("a"),
            ));
        },
    );

    let (discard_a, discard_b, sampled) = counters.expect("counters");
    assert_eq!(
        discard_a, 1,
        "ATTRIBUTION: exactly one window DEATH on `a`, which is what drives \
         `sync_discard_head`'s refill"
    );
    assert_eq!(
        discard_b, 0,
        "ATTRIBUTION: the scarce partner never dies, so every matcher pop here is `a`'s"
    );
    assert!(
        sampled >= 3,
        "ANTI-VACUITY: the gate must really have decimated (got {sampled}) — an \
         un-gated run reaches none of the three `Decimated` arms"
    );

    assert_eq!(
        role_view(&all),
        vec![
            (0, 2, 0, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Drain),
            (2, 2, 0, READ_OUTCOME_DECIMATED, ReadSiteRole::Drain),
            (2, 2, 1, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Drain),
            (3, 2, 0, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Drain),
            (3, 2, 0, READ_OUTCOME_DECIMATED, ReadSiteRole::Drain),
            (4, 2, 0, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Drain),
            (5, 2, 0, READ_OUTCOME_DECIMATED, ReadSiteRole::Drain),
            (5, 2, 1, READ_OUTCOME_DRAINED_BATCH, ReadSiteRole::Drain),
        ],
        "THE PIN: every decimated pop is a DRAIN-site read — step 2 is \
         `sync_discard_head`'s refill (the window death precedes it), step 3 is \
         `drain_for_trigger`'s between-fires refill, and step 5 is \
         `sync_peek_next_stamp`'s descent probe. Flipping any one of the three \
         `Decimated` mints to `Body` moves exactly its own row"
    );
}

/// The `(NoFrame, Body)` mint pair, read back off a
/// real run.
///
/// The six `(kind, role)` pairs a review audit found role-asserted nowhere
/// off a production stream included this one: a node whose input has never had
/// a frame delivered records `NoFrame`, and it does so at a BODY site — the
/// step-boundary snapshot, which serves the node body's latest-value read and
/// therefore mints [`ReadSiteRole::Body`] even though the SCHEDULER is what
/// runs it. That carve-out is the single most confusable thing about the role
/// vocabulary (`Drain` is defined by CONSUMING from the queue, not by who
/// called), so the pair it produces is worth reading back rather than
/// reasoning about.
///
/// The stimulus is arm (d)'s, unchanged: a lazy-loan producer that never
/// publishes, polled by a `period_ms` sink. Arm (d) pins the KINDS; this pins
/// the ROLES, and re-asserts the kind stream in the same body so a role
/// oracle cannot pass over a run that recorded something else.
///
/// Stamping the snapshot's `NoFrame` mint `Drain` moves every row
/// here, and moves nothing in any other arm of this file — the other five
/// `NoFrame` sites in the suite are asserted on kind alone.
#[test]
#[serial]
fn a_never_delivered_input_records_no_frame_at_the_body_site() {
    const STEPS: u64 = 5;
    let (kind6, all, _inputs) = run_recorded(
        "rolenone",
        Box::new(SilentSrcEntry::new()),
        Box::new(PollSinkEntry::new()),
        STEPS,
    );

    let shape: Vec<ReadRec> = (0..STEPS)
        .map(|k| rec(k, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME, 0))
        .collect();
    assert_eq!(
        kind6, shape,
        "PRECONDITION: the silent producer's run records one NoFrame per fire"
    );

    let expected: Vec<(u64, u32, u16, u16, ReadSiteRole)> = (0..STEPS)
        .map(|k| (k, 1u32, 0u16, READ_OUTCOME_NONE, ReadSiteRole::Body))
        .collect();
    assert_eq!(
        role_view(&all),
        expected,
        "THE PIN: a never-delivered input's `NoFrame` is a BODY-site read. The \
         step-boundary snapshot is performed by the scheduler but SERVES the \
         node body's latest-value read, so `Drain` here would tell an offline \
         reader the scheduler consumed a frame off a queue that has never held \
         one"
    );
}

// ===========================================================================
// The DERIVED per-stage capacity, on the PRODUCTION wiring path
// ===========================================================================

/// Every wired stage is SIZED at the capacity ITS OWN EDGE derives,
/// and three edges of one test corpus derive three DIFFERENT numbers.
///
/// This is the no-inert-shipping arm. `derive_stage_capacity`'s oracle vector
/// lives in `read_outcome.rs` and proves the ARITHMETIC; nothing there can show
/// that the three production wiring sites actually call it, or that they pass
/// each edge's own facts rather than one shape for the whole graph. Three
/// shapes are built through `GraphRuntime::build_for_test` and each stage's
/// capacity is read back through the observable
/// (`read_outcome_stage_capacities`) and compared with a HAND-COMPUTED
/// number — never with a second call to the function under test.
///
/// | consumer | policy | input | declared depth | frozen? | burst | hand oracle |
/// |---|---|---|---|---|---|---|
/// | `BurstSink` | `period_ms = 1` | plain `#[input]` | `DEFAULT_CONSUMER_DEPTH` = 10 | YES | `Fires(1)` | `max(10+1, 21) + 1` = **22** |
/// | `TrigSink` | data-trigger | `#[input(trigger)]` | 10 | no | `Fires(64)` | `max(10+64, 21) + 1` = **75** |
/// | `DeepTrigSink` | data-trigger | `#[input(trigger, depth = 64)]` | 64 | no | `Fires(64)` | `max(64+64, 129) + 1` = **130** |
///
/// The three numbers being DISTINCT is the load-bearing half: a derivation that
/// ignored the edge (or that still returned one global constant) would make
/// them equal, and every arm in this file that merely drives a rim would still
/// pass.
#[test]
#[serial]
fn each_wired_stage_is_sized_at_the_capacity_its_own_edge_derives() {
    use cerulion_core::read_outcome::ReadStageRole;

    let capacity_of = |tag: &str, sink: Box<dyn cerulion_core::graph::node::NodeEntry>| -> u32 {
        let (config, factories) = two_node_graph(tag, Box::new(SilentSrcEntry::new()), sink);
        let clock = Arc::new(VirtualClock::new());
        let runtime =
            GraphRuntime::build_for_test(config, factories, clock, 128).expect("build graph");
        let stages = runtime.read_outcome_stage_capacities("consumer");
        assert_eq!(
            stages.len(),
            1,
            "{tag}: this consumer carries exactly one stage (the unified trigger \
             edge / the plain latest-value input)"
        );
        let cerulion_core::read_outcome::ReadStageCapacity {
            input_idx,
            role,
            capacity,
        } = stages[0];
        assert_eq!(input_idx, 0, "{tag}: the first wired input");
        assert_eq!(role, ReadStageRole::Body, "{tag}: the body read path");
        capacity
    };

    let frozen_period = capacity_of("roca", Box::new(BurstSinkEntry::new()));
    let data_trigger = capacity_of("rocb", Box::new(TrigSinkEntry::new()));
    let deep_trigger = capacity_of("rocc", Box::new(DeepTrigSinkEntry::new()));

    // The hand oracles of the table above.
    assert_eq!(
        frozen_period, 22,
        "a step-FROZEN latest-value input on a Period node stages one record per \
         STEP, so its burst term is 1 and its capacity is 2*depth + 2"
    );
    assert_eq!(
        data_trigger, 75,
        "a data-trigger input is NOT frozen, so its burst term is the within-step \
         clamp. The C3 run-length encoding does NOT retire this: it folds \
         CONSECUTIVE BYTE-IDENTICAL records, and an unfrozen input can observe a \
         NEW frame on each fire — distinct served sequences, nothing to fold. RLE \
         shrinks this edge's COMMON-case population, never the bound it is sized for"
    );
    assert_eq!(
        deep_trigger, 130,
        "the SAME policy at depth 64 — only the declared depth moved"
    );

    // THE PIN: three edges, three different numbers. A derivation that ignored
    // the edge would collapse all three.
    assert_ne!(frozen_period, data_trigger);
    assert_ne!(data_trigger, deep_trigger);
    assert_ne!(frozen_period, deep_trigger);
    // (The `!= 320` sweep that used to stand here was dropped in the C1 change set:
    // the three EXACT values above already exclude 320, and a loop asserting
    // "not the old constant" adds no discrimination. The one place a `> 320`
    // claim earns its keep is the 516 row, which is pinned BY NAME in
    // `a_per_set_sync_trigger_derives_the_peek_promote_factor`.)
}

/// The file's own RAII env guard, hoisted to file scope so the
/// derived-capacity arms can share it (panic-safe removal — a failing assert must never
/// leak the discipline override into a sibling test).
struct WaveEnvGuard(&'static str);
impl WaveEnvGuard {
    fn set(key: &'static str, value: &str) -> Self {
        std::env::set_var(key, value);
        Self(key)
    }
}
impl Drop for WaveEnvGuard {
    fn drop(&mut self) {
        std::env::remove_var(self.0);
    }
}

// ===========================================================================
// The production wiring path, one fact per arm
// ===========================================================================
//
// The pure oracles in `read_outcome.rs` prove the ARITHMETIC. These arms prove
// that the three wiring sites pass each edge's OWN facts: before this change the
// production path exercised only two of the five (`depth` and the node's
// policy), so `annotated`, `sync_shape` and the freeze VALUE could each have
// been wrong at the call site with the whole suite green.

/// A `Period` consumer whose plain input declares depth 1 — the shape whose
/// FROZEN capacity is 4 and whose UNFROZEN capacity is the ceiling. Depth 1
/// makes the two answers three orders of magnitude apart.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct TinyPeriodSink {
    #[input(depth = 1)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl TinyPeriodSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// A `Period` consumer with a `block` input beside a plain one — the ONLY
/// input class `build_snapshot_input_names` excludes from the freeze by policy,
/// so `Unbounded` arises from a GRAPH here rather than from a hand oracle.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct BlockPeriodSink {
    #[input(backpressure = block)]
    blk: Vector3,
    #[input]
    ctx: Vector3,
}

#[cerulion_node_impl]
impl BlockPeriodSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.blk.x;
        let _ = self.ctx.x;
        Ok(())
    }
}

/// A data-trigger consumer whose TRIGGER and CONTEXT inputs declare DIFFERENT
/// depths — the shape that makes the Separate drain's depth read observable
/// (a drain that read the other input's depth would derive 22, not 130).
#[cerulion_node]
#[derive(Default)]
struct MixedDepthTrigSink {
    #[input(trigger, depth = 64)]
    trig: Vector3,
    #[input(depth = 10)]
    ctx: Vector3,
}

#[cerulion_node_impl]
impl MixedDepthTrigSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.trig.x;
        let _ = self.ctx.x;
        Ok(())
    }
}

/// A per-set `Sync` consumer: two trigger inputs at `MAX_CONSUMER_DEPTH`. This
/// is the ONLY shape that sets `SyncTriggerShape::PerSetSyncTrigger` on a
/// production path.
#[cerulion_node(sync_window_ms = 25)]
#[derive(Default)]
struct WideSyncSink {
    #[input(trigger, depth = 64)]
    a: Vector3,
    #[input(trigger, depth = 64)]
    b: Vector3,
}

#[cerulion_node_impl]
impl WideSyncSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.a.x;
        let _ = self.b.x;
        Ok(())
    }
}

/// The depth a `multi_publisher_topics` edge may declare. A listed topic
/// provisions a GRAPH-INDEPENDENT ceiling that commits SHM on every
/// publisher's pool, so the build refuses any consumer deeper than
/// `MULTI_TOPIC_BUFFER_CEILING` — `MAX_CONSUMER_DEPTH` (64) is four times it.
/// The two arms below therefore run their annotated/plain and per-set/ordinary
/// pairs at THIS depth on BOTH sides, so the only variable between the two
/// numbers is the factor under test.
const SHARED_EDGE_DEPTH: usize = 16;
const _: () = assert!(
    SHARED_EDGE_DEPTH <= cerulion_core::transport::MULTI_TOPIC_BUFFER_CEILING,
    "the shared-edge fixtures declare a depth the multi-publisher ceiling refuses"
);

/// A data-trigger consumer at `SHARED_EDGE_DEPTH` — the `DeepTrigSink` shape,
/// shallow enough to sit on a listed multi-publisher edge.
#[cerulion_node]
#[derive(Default)]
struct SharedDepthTrigSink {
    #[input(trigger, depth = 16)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl SharedDepthTrigSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// The `WideSyncSink` shape at `SHARED_EDGE_DEPTH`, for the same reason.
#[cerulion_node(sync_window_ms = 25)]
#[derive(Default)]
struct SharedDepthSyncSink {
    #[input(trigger, depth = 16)]
    a: Vector3,
    #[input(trigger, depth = 16)]
    b: Vector3,
}

#[cerulion_node_impl]
impl SharedDepthSyncSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.a.x;
        let _ = self.b.x;
        Ok(())
    }
}

/// A `Period` consumer with one PLAIN (frozen) input at depth 10 — the shape
/// the External arm sizes at 44 once the edge earns its producer annotation.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct ExtDepth10PeriodSink {
    #[input(depth = 10)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl ExtDepth10PeriodSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        Ok(())
    }
}

/// The derived capacities of a consumer whose inputs are wired to ABSOLUTE
/// `source:` topics with NO in-graph producer — i.e. `PublisherProvisioning::
/// External`.
///
/// This is a materially different edge from `wave_graph`'s: `External` earns
/// the producer annotation (`edge_needs_producer_annotation` = listed OR
/// External), and — the part the design got wrong — it is NOT subject to
/// `MULTI_TOPIC_BUFFER_CEILING`, because that refusal sits inside the
/// `if listed_multi` arm of the in-graph-producer branch. So an External edge
/// is annotated AND uncapped, and is the one real-graph route to the widest
/// shape.
fn external_capacities(
    prefix: &str,
    consumer: Box<dyn NodeEntry>,
    inputs: &[(&str, &str)],
) -> Vec<(u16, cerulion_core::read_outcome::ReadStageRole, u32)> {
    let config = graph_of(prefix, vec![sink_def("consumer", inputs)]);
    assert!(
        config.multi_publisher_topics.is_empty(),
        "the External arm is the one under test — listing the topic would make \
         the annotation come from the multi-publisher arm instead"
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("consumer".to_string(), consumer);
    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 128).expect("build graph");
    runtime
        .read_outcome_stage_capacities("consumer")
        .into_iter()
        .map(|c| (c.input_idx, c.role, c.capacity))
        .collect()
}

/// Build a one-producer / one-consumer graph whose consumer declares
/// `inputs`, optionally listing the producer's topic as multi-publisher.
fn wave_graph(
    prefix: &str,
    consumer: Box<dyn NodeEntry>,
    inputs: &[&str],
    multi_publisher: bool,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    // `multi_publisher_topics` only applies to ABSOLUTE topics — a DERIVED name
    // embeds the node id and cannot be shared, and the build refuses one by
    // name. So the multi-publisher arms publish under an absolute `topic:`
    // override (still graph-OWNED) and consume it as an absolute `source:`; the
    // plain arms keep the derived name, which is the shape every other arm in
    // this file already exercises.
    let shared = format!("/{prefix}/shared");
    let (out_topic, source) = if multi_publisher {
        (Some(shared.clone()), shared.clone())
    } else {
        (None, "producer/out".to_string())
    };
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: if multi_publisher {
            vec![shared]
        } else {
            Vec::new()
        },
        name: None,
        identity: "read_outcome_capture".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "ro_src".to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: out_topic,
                }],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "ro_sink".to_string(),
                inputs: inputs
                    .iter()
                    .map(|name| InputDef {
                        name: (*name).to_string(),
                        source: source.clone(),
                    })
                    .collect(),
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(SilentSrcEntry::new()) as Box<dyn NodeEntry>,
    );
    factories.insert("consumer".to_string(), consumer);
    (config, factories)
}

/// The derived capacities of `consumer`'s stages, as `(input_idx, role, capacity)`
/// in registration order — the shape a hand oracle can be written against.
fn wave_capacities(
    prefix: &str,
    consumer: Box<dyn NodeEntry>,
    inputs: &[&str],
    multi_publisher: bool,
) -> Vec<(u16, cerulion_core::read_outcome::ReadStageRole, u32)> {
    let (config, factories) = wave_graph(prefix, consumer, inputs, multi_publisher);
    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 128).expect("build graph");
    runtime
        .read_outcome_stage_capacities("consumer")
        .into_iter()
        .map(|c| (c.input_idx, c.role, c.capacity))
        .collect()
}

/// THE HEADLINE ARM — membership in the snapshot
/// NAME SET is not the freeze; the node's ENTRY has to be able to freeze.
///
/// `build_snapshot_input_names` answers a POLICY question only: "is this input
/// non-triggering and not `block`?" A `ClosureNodeEntry` and a raw-FFI cdylib
/// both inherit the default NO-OP `snapshot_inputs`, so their inputs sit in
/// that set while every read takes `try_view`'s LIVE arm — one record PER FIRE.
///
/// Sizing on membership alone therefore declared `Fires(1)` for a node that
/// really stages one record per catch-up fire. On a `Period` node with a plain
/// depth-1 input that is a derived capacity of **4** against an UNBOUNDED
/// burst — a rim the deleted global 320 comfortably cleared, so the name-only
/// predicate would have made C1 INTRODUCE a truncation it was written to
/// remove.
///
/// The discriminator is a closure entry beside a `#[cerulion_node]` twin of the
/// SAME declared shape: identical policy, identical input, identical depth,
/// and the only difference is whether the entry can hold a snapshot.
#[test]
#[serial]
fn a_closure_node_that_cannot_freeze_is_sized_for_one_record_per_fire() {
    use cerulion_core::read_outcome::{ReadStageRole, READ_OUTCOME_STAGE_MAX};

    // The macro twin: a real `#[cerulion_node(period_ms = 1)]` with one plain
    // depth-1 input. `holds_input_snapshot() == true`, so the freeze is real.
    let macro_twin = wave_capacities(
        "rocfa",
        Box::new(TinyPeriodSinkEntry::new()),
        &["inp"],
        false,
    );
    assert_eq!(
        macro_twin,
        vec![(0, ReadStageRole::Body, 4)],
        "a macro Period node CAN freeze, so its plain depth-1 input stages one \
         record per STEP: max(1 + 1, 3) + 1 = 4"
    );

    // The closure twin: the SAME declared shape — `Period`, one plain input,
    // depth 1 — on an entry whose `snapshot_inputs` is the inherited no-op.
    let closure_info = NodeInfo::with_meta(
        vec![InputMeta {
            name: "inp".to_string(),
            schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
            trigger: false,
            depth: 1,
            backpressure: BackpressurePolicy::DropOldest,
            expect_within_ms: None,
        }],
        Vec::new(),
    )
    .with_policy(MacroPolicy::Period { period_ms: 1 });
    let closure_twin = wave_capacities(
        "rocfb",
        Box::new(ClosureNodeEntry::new(closure_info, |_ctx| Ok(()))),
        &["inp"],
        false,
    );
    assert_eq!(
        closure_twin,
        vec![(0, ReadStageRole::Body, READ_OUTCOME_STAGE_MAX)],
        "a closure entry inherits the NO-OP `snapshot_inputs`, so the same \
         declared shape reads LIVE on every fire — an unclamped `Period` \
         catch-up burst, which lands on the ceiling"
    );

    // THE PIN. A predicate keyed on name membership alone makes these EQUAL
    // (both 4), which is the regression this arm exists to catch: 4 slots
    // against one record per catch-up fire is a truncation the deleted global
    // 320 did not have.
    assert_ne!(
        macro_twin[0].2, closure_twin[0].2,
        "the freeze must follow the ENTRY's capability, not the policy name set"
    );
    assert!(
        closure_twin[0].2 > 320,
        "the unfrozen shape must be sized ABOVE the deleted global, or C1 would \
         introduce a truncation on a shape that never had one"
    );
}

/// The behavioural half of the arm above: drive a closure `Period` node through
/// a REAL catch-up burst and require ZERO overflow markers.
///
/// This is the arm that would have FAILED on `de2168cb7`: a 700-fire burst
/// against a 4-record stage drops at the rim on every fire past the fourth,
/// marks the hole, and counts it. It is not a hypothetical — the stimulus is
/// the same one `run_exit_drop_report_is_empty_because_a_burst_stages_one_read_not_one_per_fire`
/// uses for the macro node.
#[test]
#[serial]
fn a_closure_period_burst_stages_no_marker_at_its_derived_capacity() {
    let closure_info = NodeInfo::with_meta(
        vec![InputMeta {
            name: "inp".to_string(),
            schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
            trigger: false,
            depth: 1,
            backpressure: BackpressurePolicy::DropOldest,
            expect_within_ms: None,
        }],
        Vec::new(),
    )
    .with_policy(MacroPolicy::Period { period_ms: 1 });
    // The closure MUST actually read its input every fire: a read-outcome record
    // is staged from a READ, never from a fire. A `|_ctx| Ok(())` body touches
    // nothing, stages nothing, and would make the zero-drop claim below hold at
    // ANY capacity — the vacuity this arm is written to avoid.
    let (config, mut factories) = wave_graph(
        "rocfc",
        Box::new(ClosureNodeEntry::new(closure_info, |ctx| {
            if let Some(sub) = ctx.subscriber_mut("inp") {
                let _ = sub.try_view::<Vector3, _>(|_v| ())?;
            }
            Ok(())
        })),
        &["inp"],
        false,
    );
    // ...and a REAL producer: `wave_graph`'s default `SilentSrc` writes no output,
    // so with lazy-loan it never publishes and every read would be a `NoFrame`.
    // `BacklogSrc` is `period_ms = 1` and writes `out.x`, so the 700 ms step below
    // drives a genuine ~700-frame burst through the stage.
    factories.insert(
        "producer".to_string(),
        Box::new(BacklogSrcEntry::new()) as Box<dyn NodeEntry>,
    );
    let ring_tag = format!("closure_{}", std::process::id());
    let mut owner = TraceRingOwner::create_with_inputs(
        &ring_tag,
        default_capacity_records(),
        0,
        &["producer", "consumer"],
        &[&[], &["inp"]],
    )
    .expect("create ring");
    let ring_name = owner.name().to_string();
    let producer_handle = owner.producer().expect("mint producer");
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 128).expect("build graph");
    runtime.set_trace_ring_producer(
        producer_handle,
        &["producer".to_string(), "consumer".to_string()],
    );
    assert!(runtime.read_outcome_stages_armed_for_test() > 0);

    // ONE 700 ms step ⇒ 700 catch-up fires on a `period_ms = 1` node.
    runtime.step(Duration::from_millis(1));
    runtime.step(Duration::from_millis(700));
    let fires = runtime
        .node_handle("consumer")
        .expect("the consumer is a scheduler node")
        .fire_count();
    let (dropped_total, breakdown) = runtime.read_outcome_dropped_report();
    // THE PREMISE, measured — not inferred from the fire count. "No overflow
    // marker" is satisfied by a stage nothing ever reaches, so the burst has to
    // be shown to have REACHED the read log, not merely to have FIRED: a fire
    // whose read stages nothing is invisible to the drop report.
    //
    // It is measured RIM-INDEPENDENTLY. The ring holds the SURVIVORS of the rim
    // this arm exists to protect, so a bare `staged >= 600` is a second copy of
    // the zero-drop assertion: shrink the rim and the premise trips first,
    // leaving the real claim unreached and the test "passing for the wrong
    // reason" in reverse. `staged + dropped` is what the burst OFFERED the
    // stage, which no capacity can change — so the premise fixes the stimulus
    // and the assertion below is left to judge the rim alone.
    let mut ring_consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
    let mut ring_records = Vec::new();
    ring_consumer.drain(&mut ring_records).expect("drain ring");
    drop(ring_consumer);
    // Scoped to THIS stage (consumer = manifest node 1, its only input 0): a
    // producer-side input added later must not be able to inflate the premise.
    // Count OCCURRENCES, not records. The run-length encoding
    // folds a maximal run of byte-identical reads into ONE record carrying a
    // count, and this burst is exactly that shape — 701 catch-up fires with no
    // new frame, so every read is an identical `NoFrame`. Counting records here
    // would measure the fold, not the stimulus, and the premise below is a
    // statement about what the burst OFFERED the stage.
    //
    // MEASURED: the 701 occurrences arrive as 3 records. That collapse is the
    // benefit C3 exists for, and it is asserted separately below so a fold that
    // silently stopped working would be caught rather than absorbed.
    let staged_records: Vec<&_> = ring_records
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_READ_OUTCOME)
        .filter(|r| {
            let (input_idx, _) = unpack_read_outcome_meta(r.global_level);
            r.node_idx == 1 && input_idx == 0
        })
        .collect();
    let staged: usize = staged_records
        .iter()
        .map(|r| cerulion_core::trace_ring::unpack_read_outcome_run(r.duration_ns) as usize)
        .sum();
    // The EXACT record count, not "fewer than the occurrences" (which a fold
    // that collapsed only the first pair also satisfies, at 700 records for 701
    // occurrences). MEASURED at 3: the burst's reads are interrupted twice by a
    // read that differs, so the run is three maximal runs rather than one.
    assert_eq!(
        staged_records.len(),
        3,
        "the fold collapses this burst to THREE records standing for {staged} \
         occurrences; a different number means the fold rule moved: {staged_records:?}"
    );
    runtime.shutdown();
    drop(owner);

    // The EXACT count, not a floor. The clock is a
    // `VirtualClock` and the node is `period_ms = 1`, so the stimulus
    // (`step(1ms)` then `step(700ms)`) fires 1 + 700 = 701 times — deterministic
    // arithmetic, not a measurement. A floor of 600 let a stimulus that
    // silently truncated to 600 pass the arm that exists to prove the burst is
    // large enough to matter.
    const EXPECTED_FIRES: u64 = 1 + 700;
    assert_eq!(
        fires, EXPECTED_FIRES,
        "anti-vacuity: the catch-up burst is EXACTLY 1 + 700 fires under the \
         VirtualClock; {fires} means the stimulus changed and every count below \
         is describing a different experiment"
    );
    let offered = staged as u64 + dropped_total;
    println!(
        "PREMISE fires={fires} staged={staged} dropped={dropped_total} \
         offered={offered}"
    );
    // The premise is EXACT too (the same finding): one record is offered per
    // fire on this edge, so the burst offers exactly what it fired.
    assert_eq!(
        offered, EXPECTED_FIRES,
        "PREMISE (rim-independent): the burst must OFFER the stage one record per \
         fire — {staged} survived + {dropped_total} dropped = {offered}, against \
         {fires} fires. Without this the assertion below is vacuous: a stage \
         nothing writes to never overflows, at ANY capacity."
    );
    assert_eq!(
        (dropped_total, breakdown),
        (0, Vec::new()),
        "a closure entry's unfrozen input is sized for one record per FIRE, so a \
         {fires}-fire burst must not overflow its stage. On `de2168cb7` this \
         edge was sized at 4."
    );
}

/// The Separate trigger-DRAIN wiring site, read back from a
/// real graph, with the vector's ORDER and the drain's DEPTH both pinned.
///
/// Two facts nothing else covers. (1) Both Drain wiring sites had ZERO
/// production coverage — every previous arm reads a single Body stage. (2) The
/// consumer declares DIFFERENT depths on its trigger and context inputs, so a
/// drain site that read the wrong input's depth derives 22 instead of 130; a
/// same-depth graph passes either way.
///
/// The registration order is `[Body per input, in `inputs` order] ++ [Separate
/// drain]`, so the drain's `input_idx` REPEATS the trigger's — which is exactly
/// why the row carries its role and why C2 keys on `(input_idx, role)`.
#[test]
#[serial]
fn two_stages_on_one_input_are_sized_independently() {
    use cerulion_core::read_outcome::ReadStageRole;

    // Default (UNIFIED) discipline: no separate drain subscriber exists.
    let unified = wave_capacities(
        "rocga",
        Box::new(MixedDepthTrigSinkEntry::new()),
        &["trig", "ctx"],
        false,
    );
    assert_eq!(
        unified,
        vec![(0, ReadStageRole::Body, 130), (1, ReadStageRole::Body, 22),],
        "unified: one Body stage per input, in `inputs` order — the depth-64 \
         TRIGGER at max(64 + 64, 129) + 1 = 130 and the depth-10 FROZEN context \
         at max(10 + 1, 21) + 1 = 22"
    );

    // FORCED SEPARATE: the trigger input grows a second, runtime-owned stage.
    let _guard = WaveEnvGuard::set("CERULION_DRAIN_DISCIPLINE", "separate");
    let separate = wave_capacities(
        "rocgb",
        Box::new(MixedDepthTrigSinkEntry::new()),
        &["trig", "ctx"],
        false,
    );
    drop(_guard);

    assert_eq!(
        separate,
        vec![
            (0, ReadStageRole::Body, 130),
            (1, ReadStageRole::Body, 22),
            (0, ReadStageRole::Drain, 130),
        ],
        "separate: the trigger input carries TWO stages sharing `input_idx` 0, \
         and the DRAIN is sized from the TRIGGER's declared depth (64 ⇒ 130), \
         not from the sibling context input's (10 ⇒ 22)"
    );
    // THE ASYMMETRY PIN, stated as its own claim: the drain and the sibling
    // context stage must NOT agree, or a site reading the wrong depth passes.
    assert_ne!(
        separate[2].2, separate[1].2,
        "a drain site that read the context input's depth would derive 22"
    );
}

/// The PAIR factor on the production path.
///
/// `edge_needs_producer_annotation` was computed at all three sites before the
/// change but nothing read its effect on a CAPACITY from a real graph — a site
/// that passed `Plain` unconditionally derived 81 where 162 is required, and
/// every existing arm agreed with it.
#[test]
#[serial]
fn a_multi_publisher_edge_derives_the_pair_factor() {
    use cerulion_core::read_outcome::ReadStageRole;

    // BOTH sides at `SHARED_EDGE_DEPTH`: a listed edge cannot declare
    // `MAX_CONSUMER_DEPTH`, and running the two sides at different depths
    // would make the pair of numbers differ for two reasons at once.
    let plain = wave_capacities(
        "rocha",
        Box::new(SharedDepthTrigSinkEntry::new()),
        &["inp"],
        false,
    );
    let annotated = wave_capacities(
        "rochb",
        Box::new(SharedDepthTrigSinkEntry::new()),
        &["inp"],
        true,
    );
    // depth 16, Data trigger: max(16 + 64, 33) + 1 = 81, doubled by the pair.
    assert_eq!(plain, vec![(0, ReadStageRole::Body, 81)]);
    assert_eq!(
        annotated,
        vec![(0, ReadStageRole::Body, 162)],
        "a `multi_publisher_topics`-listed edge stages a Producer annotation \
         with every read, so the whole bound doubles"
    );
    assert_eq!(annotated[0].2, 2 * plain[0].2);
}

/// The per-set `Sync` SITE factor on the production path.
///
/// `sync_shape` was never `PerSetSyncTrigger` in any test built from a graph,
/// so a site that passed `Ordinary` unconditionally halved every per-set Sync
/// body stage with the whole suite green. The DataTrigger control at the same
/// depth is what makes this a test of the SHAPE rather than of the depth.
#[test]
#[serial]
fn a_per_set_sync_trigger_derives_the_peek_promote_factor() {
    use cerulion_core::read_outcome::{ReadStageRole, READ_OUTCOME_STAGE_MAX};

    // The per-set shape at MAX_CONSUMER_DEPTH, on an UNLISTED edge (a listed
    // one could not carry depth 64): max(2*64 + 64, 2*2*64 + 1) + 1 = 258.
    let sync = wave_capacities(
        "rocia",
        Box::new(WideSyncSinkEntry::new()),
        &["a", "b"],
        false,
    );
    assert_eq!(
        sync,
        vec![
            (0, ReadStageRole::Body, READ_OUTCOME_STAGE_MAX),
            (1, ReadStageRole::Body, READ_OUTCOME_STAGE_MAX),
        ],
        "BOT-2: a per-set Sync TRIGGER costs a Peek record and a \
         promotion record per ADVANCE, the advance budget resets per alignment \
         pass, and the burst re-aligns after every fire — so the reachable \
         population is unbounded and the stage arms at the ceiling"
    );

    // The two factors COMPOSING, at the depth a listed edge admits: per-set
    // (site 2) AND annotated (pair 2).
    //
    // These are the two rows the refill fix MOVED, from 97/194
    // to 258/516. A per-set Body's consuming term is `site * max(depth,
    // max_sets)`, and `max_sets` (64) is `MAX_CONSUMER_DEPTH`, so it is the
    // larger term at EVERY declarable depth: a per-set trigger's capacity is
    // DEPTH-INDEPENDENT. That is the point of the fix — the matcher re-aligns
    // between fires, so a shallow queue that REFILLS is drained just as many
    // times as a deep one — and it is why these now equal their depth-64 twins
    // above. `a_per_set_trigger_is_sized_for_a_refilling_queue_not_for_its_depth` pins the
    // equality directly, so this arm is no longer the only thing that would
    // notice a depth term creeping back in.
    let sync_multi = wave_capacities(
        "rocib",
        Box::new(SharedDepthSyncSinkEntry::new()),
        &["a", "b"],
        true,
    );
    assert_eq!(
        sync_multi,
        vec![
            (0, ReadStageRole::Body, READ_OUTCOME_STAGE_MAX),
            (1, ReadStageRole::Body, READ_OUTCOME_STAGE_MAX),
        ],
        "an unbounded shape arms at the ceiling whether or not the edge is \
         listed — once the population is unbounded, neither the site factor nor \
         the pair factor can move it"
    );
    let sync_shared_plain = wave_capacities(
        "rocid",
        Box::new(SharedDepthSyncSinkEntry::new()),
        &["a", "b"],
        false,
    );
    assert_eq!(
        sync_shared_plain,
        vec![
            (0, ReadStageRole::Body, READ_OUTCOME_STAGE_MAX),
            (1, ReadStageRole::Body, READ_OUTCOME_STAGE_MAX),
        ],
        "the same per-set shape UNLISTED: also the ceiling. The pair factor is \
         no longer observable HERE — `the_pair_factor_doubles_the_whole_bound_not_only_the_depth_term` \
         pins it on a bounded (Ordinary) shape, which is where it stays visible"
    );

    // THE CONTROL: the same depth on a DataTrigger consumer derives 130. Both
    // node kinds satisfy the three CAPABILITY terms of `per_set_capable`, so a
    // predicate that read capability alone would report 258 here too.
    let data_trigger =
        wave_capacities("rocic", Box::new(DeepTrigSinkEntry::new()), &["inp"], false);
    assert_eq!(data_trigger, vec![(0, ReadStageRole::Body, 130)]);
}

/// `Unbounded` arises from a GRAPH, not only from a hand oracle.
///
/// `block` is the one input class the step freeze excludes by POLICY, so a
/// `Period` node with a `block` input is the shape that reaches the ceiling —
/// and its plain sibling on the same node, same producer, same step, is sized
/// at 22. No overflow stimulus is needed: the two capacities are the claim.
#[test]
#[serial]
fn a_period_block_input_reaches_the_ceiling_and_its_sibling_does_not() {
    use cerulion_core::read_outcome::{ReadStageRole, READ_OUTCOME_STAGE_MAX};

    let caps = wave_capacities(
        "rocj",
        Box::new(BlockPeriodSinkEntry::new()),
        &["blk", "ctx"],
        false,
    );
    assert_eq!(
        caps,
        vec![
            (0, ReadStageRole::Body, READ_OUTCOME_STAGE_MAX),
            (1, ReadStageRole::Body, 22),
        ],
        "the `block` input reads LIVE on every catch-up fire (its drain must \
         stay on the consumer's tick), so an unclamped Period burst lands on \
         the ceiling; the plain sibling on the SAME node is frozen at 22"
    );
}

/// The derived vector is DETERMINISTIC: two builds of the same
/// graph agree, and both agree with the hand oracle.
///
/// Weaker than it looks if only self-compared, which is why the oracle is
/// stated: a derivation reading an unstable map order would produce equal-but-
/// wrong vectors on both runs.
#[test]
#[serial]
fn the_derived_capacities_are_deterministic_across_builds() {
    use cerulion_core::read_outcome::ReadStageRole;

    let oracle = vec![(0, ReadStageRole::Body, 130), (1, ReadStageRole::Body, 22)];
    let first = wave_capacities(
        "rocka",
        Box::new(MixedDepthTrigSinkEntry::new()),
        &["trig", "ctx"],
        false,
    );
    let second = wave_capacities(
        "rockb",
        Box::new(MixedDepthTrigSinkEntry::new()),
        &["trig", "ctx"],
        false,
    );
    assert_eq!(first, oracle, "run 1 == the hand oracle");
    assert_eq!(second, oracle, "run 2 == the hand oracle");
    assert_eq!(first, second);
}

/// The THIRD wiring site: the legacy-`Sync` DRAIN.
///
/// There are three places a stage is minted. The Body site and the Separate
/// trigger-drain site each had graph-built coverage; this one — the `else` of
/// `per_set_capable`, which mints one drain stage PER TRIGGER for a `Sync`
/// consumer that cannot run per-set — had NONE, so its `depth`, `annotated`
/// and `sync_shape` arguments were all unpinned from a real graph.
///
/// `CERULION_DRAIN_DISCIPLINE=separate` is what forces the legacy path: it
/// takes the consumer off per-set, which ALSO drops the bodies to `Ordinary`
/// (site 1). So the shape is the one an operator really gets under that seam —
/// four stages, all at the trigger edge's own depth 16, none paying the
/// peek/promote factor.
#[test]
#[serial]
fn a_legacy_sync_drain_is_sized_from_its_own_trigger_edge() {
    use cerulion_core::read_outcome::ReadStageRole;

    let _guard = WaveEnvGuard::set("CERULION_DRAIN_DISCIPLINE", "separate");
    let stages = wave_capacities(
        "rocla",
        Box::new(SharedDepthSyncSinkEntry::new()),
        &["a", "b"],
        false,
    );
    assert_eq!(
        stages,
        vec![
            (0, ReadStageRole::Body, 81),
            (1, ReadStageRole::Body, 81),
            (0, ReadStageRole::Drain, 81),
            (1, ReadStageRole::Drain, 81),
        ],
        "forced-Separate takes the consumer off per-set, so each trigger has an \
         Ordinary BODY stage and a legacy-Sync DRAIN stage, both sized from \
         THAT trigger's own depth 16: max(16 + 64, 33) + 1 = 81"
    );
}

/// The producer annotation reaches the DRAIN site too.
///
/// `W4` mutates only the Body site, so a drain site passing `Plain`
/// unconditionally survived it. A listed edge read through a Separate drain
/// must double at BOTH stages.
#[test]
#[serial]
fn an_annotated_edge_doubles_its_drain_stage_too() {
    use cerulion_core::read_outcome::ReadStageRole;

    let _guard = WaveEnvGuard::set("CERULION_DRAIN_DISCIPLINE", "separate");
    let stages = wave_capacities(
        "rocma",
        Box::new(SharedDepthTrigSinkEntry::new()),
        &["inp"],
        true,
    );
    assert_eq!(
        stages,
        vec![
            (0, ReadStageRole::Body, 162),
            (0, ReadStageRole::Drain, 162),
        ],
        "a `multi_publisher_topics` edge stages a Producer annotation at EVERY \
         site that reads it, drain included: 81 doubled to 162 on both"
    );
}

/// External-provisioned edges are annotated AND UNCAPPED,
/// which is what makes the widest shape reachable from a real graph.
///
/// This CORRECTS an earlier note (E-shared). `edge_needs_producer_annotation`
/// is `listed OR External`, but `MULTI_TOPIC_BUFFER_CEILING` (16) is enforced
/// only inside the LISTED arm of the in-graph-producer branch — an absolute
/// `source:` with no in-graph producer never reaches it. So a per-set `Sync`
/// trigger on an External edge at `MAX_CONSUMER_DEPTH` is both doubled and
/// undepth-capped, and derives 516: not a pure-oracle bound after all, but the
/// worst shape a `ros2 attach`-style graph really builds.
#[test]
#[serial]
fn an_external_edge_is_annotated_and_uncapped() {
    use cerulion_core::read_outcome::{ReadStageRole, READ_OUTCOME_STAGE_MAX};

    // (i) the plain frozen shape: Period + a depth-10 latest-value input.
    // max(10 + 1, 21) + 1 = 22, doubled by the External annotation = 44.
    let plain = external_capacities(
        "rocna",
        Box::new(ExtDepth10PeriodSinkEntry::new()),
        &[("inp", "/rocna/ext")],
    );
    assert_eq!(
        plain,
        vec![(0, ReadStageRole::Body, 44)],
        "an External edge earns the producer annotation with no listing: 22 → 44"
    );

    // (ii) THE WORST SHIPPING SHAPE, from a graph. Depth 64 is refused on a
    // LISTED edge (ceiling 16) but is fine here, because the ceiling check
    // never runs for a producer-less absolute source.
    let widest = external_capacities(
        "rocnb",
        Box::new(WideSyncSinkEntry::new()),
        &[("a", "/rocnb/ext_a"), ("b", "/rocnb/ext_b")],
    );
    assert_eq!(
        widest,
        vec![
            (0, ReadStageRole::Body, READ_OUTCOME_STAGE_MAX),
            (1, ReadStageRole::Body, READ_OUTCOME_STAGE_MAX),
        ],
        "per-set Sync (site 2) at MAX_CONSUMER_DEPTH on an annotated External \
         edge: max(2*64 + 64, 2*2*64 + 1) + 1 = 258, doubled = 516 — the value \
         the pure oracle pins, now reached by a real build"
    );
}
