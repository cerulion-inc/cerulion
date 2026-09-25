// SPDX-License-Identifier: AGPL-3.0-only
//! — the FORK CARRIER, driven through a REAL `GraphRuntime` over REAL
//! iceoryx2, a REAL mapped arm word, a REAL POSIX-SHM state ring and a REAL `fork(2)`.
//!
//! `state_anchor_boundary_iox2_test` covers the INLINE half: what the node thread
//! writes, at which steps, and when it declines. This file covers the other carrier —
//! the one that exists so a node too big or too dangerous to encode on the node thread
//! still gets an anchor.
//!
//! # What only a real fork can show
//!
//! The pure halves are oracle-tested in `state_carrier::fork`: the memory gate, the
//! post-mortem's uncovered set, the outcome-to-cause mapping. None of them can see the
//! four things that decide whether a robot's black box actually contains a giant's
//! state:
//!
//! * that a deferred node's bytes REALLY reach the ring, and are the node's own;
//! * that the parent's producer is resynced afterwards, so the NEXT anchor does not
//!   land on top of the child's records (`fork` duplicates the producer, so
//!   the parent's local write cursor is stale by everything the child wrote);
//! * that a cadence reached while a child is still running is SKIPPED and eventually
//!   NAMED, rather than silently absent or, worse, written into a ring the child
//!   owns;
//! * that a child which dies partway leaves an exact post-mortem — the nodes it covered
//!   are in the ring, the ones it did not are recorded as skipped, and no node is both.
//!
//! # Every oracle is hand-written
//!
//! A forked node's payload is compared against a blob built by the production capture
//! helper over a HAND-CONSTRUCTED value of the same type, never against another run of
//! the same graph. So a child that captured the wrong step's state, or the wrong node's,
//! fails on the bytes.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test state_anchor_fork_iox2_test -- --test-threads=1
//! ```
//!
//! `#![cfg(unix)]`. `#[serial]` — every test here forks, and the reaper's `waitpid` is
//! targeted, but the arm word's claim table is process-wide state that two concurrent
//! anchors in one test binary would share.

#![cfg(unix)]

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use cerulion_core::clock::VirtualClock;
use cerulion_core::error::TransportResult;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::{MacroPolicy, NodeContext, NodeEntry, NodeInfo};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::state::{SkipCause, VecSink};
use cerulion_core::state_arm::MappedStateArm;
use cerulion_core::state_restore::{capture_anchor_blob_with_framework, NodeFrameworkState};
use cerulion_core::state_ring::{
    StateAnchorEvent, StateAssembler, StateRingConsumer, StateRingOwner,
};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

// ---------------------------------------------------------------------------
// fixtures
// ---------------------------------------------------------------------------

/// Plain data: inline-eligible, so this node takes the INLINE carrier. It is the
/// CONTROL in every test here — a fork that swallowed the whole boundary would take
/// this one with it.
#[cerulion_node(period_ms = 10)]
struct Counter {
    #[output]
    out: Vector3,
    count: u64,
}

#[cerulion_node_impl]
impl Counter {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.count += 1;
        self.out.x = self.count as f64;
        Ok(())
    }
}

/// Carries a lock, so `INLINE_SAFE = false` and the boundary DEFERS it to the fork
/// carrier. Its `total` is the value the child must be seen to have captured.
#[cerulion_node(period_ms = 10)]
struct Guarded {
    #[output]
    out: Vector3,
    shared: Arc<Mutex<u64>>,
    total: u64,
}

#[cerulion_node_impl]
impl Guarded {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.total += 2;
        self.out.x = self.total as f64;
        Ok(())
    }
}

/// A HAND-WRITTEN `NodeEntry` whose capture always refuses, while still declaring a
/// shape and refusing the inline carrier.
///
/// It exists because no generated encoder fails: the derive's `cer_capture` cannot
/// return `Err` for a fixture like `Guarded`. But a hand implementation can, and the child's
/// exit-code vocabulary has a code for it, so the path is production-reachable and needs a
/// production-shaped driver rather than an artificially injected fault.
///
/// `inline_safe = false` puts it in the FORK set (the default, so this is the shape a
/// hand impl gets for free); `state_shape = Some` takes it past the framing gate and
/// into the sink, which is exactly where the interesting failure is.
#[derive(Default)]
struct RefusingCapture {
    context: Option<NodeContext>,
}

impl NodeEntry for RefusingCapture {
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(NodeInfo::from_names(Vec::new(), Vec::new())
            .with_policy(MacroPolicy::Period { period_ms: 10 }))
    }

    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.context = Some(context);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        Ok(())
    }

    fn state_shape(&self) -> Option<u64> {
        // A real-looking shape: the node CLAIMS it can be anchored, which is what takes
        // it past the framing gate and into a sink.
        Some(0xDEAD_BEEF_0000_1053)
    }

    fn capture_state(&self, out: &mut dyn cerulion_core::state::StateSink) -> TransportResult<()> {
        // Writes FIRST, so the failure lands after the carrier's header AND after real
        // payload bytes — the shape that would be published as a complete-but-truncated
        // anchor if the sink were finished on this arm.
        let _ = out.write(&[0xAB; 8]);
        Err(cerulion_core::error::TransportError::GraphError {
            reason: "this fixture's encoder always refuses".to_string(),
        })
    }
}

/// How long [`SlowCapture`]'s encode takes, and how it spends it.
///
/// ~1.6 s of encode, in 8 ms steps. Long enough that the reaper polls a dozen times
/// while the child is alive, which is what makes two otherwise racy properties
/// DETERMINISTIC: that a wildcard `waitpid` has a window in which the only zombies are
/// somebody else's children, and that a watchdog counting ELAPSED time rather than
/// STALLED time would fire. It has to clear ONE SECOND to be probative at all — the
/// carrier const-asserts `REAPER_POLL_INTERVAL * 10 < STATE_STALL_TIMEOUT_NS`, so a
/// duration cap short enough to kill a faster encode does not compile.
const SLOW_CHUNKS: usize = 200;
const SLOW_CHUNK_PAUSE: Duration = Duration::from_millis(8);
const SLOW_CHUNK_BYTES: usize = 16;

/// A HAND-WRITTEN `NodeEntry` whose capture is SLOW but never stops making progress.
///
/// This is the shape the liveness watchdog exists for: a 500 MB serde
/// encode costs 2.5-10 s, and a duration cap with a 5 s floor would SIGKILL
/// it — a node-level, cost-derived refusal, which is precisely the
/// outcome the watchdog must never produce. So the encode here takes far longer than any single
/// reaper poll and bumps the progress word throughout (every `write` through the child's
/// sink does), and it must complete.
#[derive(Default)]
struct SlowCapture {
    context: Option<NodeContext>,
}

impl NodeEntry for SlowCapture {
    fn info(&self) -> TransportResult<NodeInfo> {
        Ok(NodeInfo::from_names(Vec::new(), Vec::new())
            .with_policy(MacroPolicy::Period { period_ms: 10 }))
    }

    fn init(&mut self, context: NodeContext) -> TransportResult<()> {
        self.context = Some(context);
        Ok(())
    }

    fn tick(&mut self) -> TransportResult<()> {
        Ok(())
    }

    fn state_shape(&self) -> Option<u64> {
        Some(0x5107_0000_0000_1053)
    }

    fn capture_state(&self, out: &mut dyn cerulion_core::state::StateSink) -> TransportResult<()> {
        for i in 0..SLOW_CHUNKS {
            // The pause is what makes it slow; the WRITE is what makes it progressing.
            // A duration cap cannot tell the two apart, which is the whole point.
            std::thread::sleep(SLOW_CHUNK_PAUSE);
            let _ = out.write(&[i as u8; SLOW_CHUNK_BYTES]);
        }
        Ok(())
    }
}

/// The payload `SlowCapture` must produce — hand-built from the same loop, so the
/// oracle is the specification rather than a recording of the run.
fn slow_capture_payload() -> Vec<u8> {
    let mut expected = Vec::with_capacity(SLOW_CHUNKS * SLOW_CHUNK_BYTES);
    for i in 0..SLOW_CHUNKS {
        expected.extend_from_slice(&[i as u8; SLOW_CHUNK_BYTES]);
    }
    expected
}

// ---------------------------------------------------------------------------
// harness
// ---------------------------------------------------------------------------

const RING_RECORDS: u32 = 4096;
const RUN_ID: u64 = 0xC0FF_EE10_0F0C;
/// Every wait in this file is a LIVENESS ceiling, never a measurement. A fork plus a
/// child's encode plus the reaper's 100 ms poll is milliseconds; 30 s separates
/// "did not happen" from "a loaded runner was slow", which is the class.
const CEILING: Duration = Duration::from_secs(30);

/// Unique per test AND per process, because the state ring and the arm word are named
/// POSIX SHM objects and a collision would have two runs sharing one table.
fn unique(tag: &str) -> String {
    static SEQ: AtomicU64 = AtomicU64::new(0);
    format!(
        "cf{tag}{}{}",
        std::process::id() % 1000,
        SEQ.fetch_add(1, Ordering::Relaxed)
    )
}

fn graph(prefix: &str, ids: &[&str]) -> GraphConfig {
    GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "fork".to_string(),
        prefix: prefix.to_string(),
        nodes: ids
            .iter()
            .map(|id| NodeDef {
                fuse: None,
                ros2: None,
                id: (*id).to_string(),
                node_type: (*id).to_string(),
                inputs: vec![],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            })
            .collect(),
    }
}

struct Harness {
    runtime: GraphRuntime,
    _owner: StateRingOwner,
    arm: Arc<MappedStateArm>,
    ring_name: String,
}

fn harness(
    tag: &str,
    factories: IndexMap<String, Box<dyn NodeEntry>>,
    node_ids: &[String],
    ring_records: u32,
) -> Harness {
    let prefix = unique(tag);
    let ids: Vec<&str> = node_ids.iter().map(|s| s.as_str()).collect();
    let mut runtime = GraphRuntime::build_for_test(
        graph(&prefix, &ids),
        factories,
        Arc::new(VirtualClock::new()),
        16,
    )
    .expect("graph builds");

    let mut owner =
        StateRingOwner::create(&unique(&format!("{tag}r")), ring_records, 0, RUN_ID, &ids)
            .expect("state ring");
    let ring_name = owner.name().to_string();
    let producer = owner.producer().expect("producer");
    runtime.set_state_ring_producer(producer, node_ids);

    let arm =
        Arc::new(MappedStateArm::create_owned(&unique(&format!("{tag}a"))).expect("arm word"));

    Harness {
        runtime,
        _owner: owner,
        arm,
        ring_name,
    }
}

fn drain(ring_name: &str) -> Vec<StateAnchorEvent> {
    let mut consumer = StateRingConsumer::open(ring_name).expect("open ring");
    let mut assembler = StateAssembler::passthrough();
    let mut events = Vec::new();
    consumer.drain(&mut assembler, &mut events).expect("drain");
    events.extend(assembler.finish());
    events
}

fn completes(events: &[StateAnchorEvent]) -> Vec<(u64, u32, Vec<u8>)> {
    events
        .iter()
        .filter_map(|e| match e {
            StateAnchorEvent::Complete {
                step,
                node_idx,
                bytes,
                ..
            } => Some((*step, *node_idx, bytes.clone())),
            _ => None,
        })
        .collect()
}

fn skips(events: &[StateAnchorEvent]) -> Vec<(u64, u32, SkipCause)> {
    events
        .iter()
        .filter_map(|e| match e {
            StateAnchorEvent::Skipped {
                step,
                node_idx,
                cause,
                ..
            } => Some((*step, *node_idx, *cause)),
            _ => None,
        })
        .collect()
}

/// The blob a `Guarded` holding `total` must produce, built through the SAME production
/// helper a restore reads back, over a hand-constructed value.
fn guarded_anchor_oracle(total: u64, step: u64) -> Vec<u8> {
    // Functional update rather than a plain literal: the node macro injects a hidden
    // runtime field, so every member must come from somewhere.
    let node = Guarded {
        total,
        ..Default::default()
    };
    let mut sink = VecSink::new();
    capture_anchor_blob_with_framework(&node, &framework_oracle(step), &mut sink)
        .expect("a guarded node fits any sink");
    sink.into_inner()
}

fn counter_anchor_oracle(count: u64, step: u64) -> Vec<u8> {
    let node = Counter {
        count,
        ..Default::default()
    };
    let mut sink = VecSink::new();
    capture_anchor_blob_with_framework(&node, &framework_oracle(step), &mut sink)
        .expect("a counter fits any sink");
    sink.into_inner()
}

/// The scheduler's framework section a node anchored at `step` carries.
///
/// HAND-COMPUTED: both fixtures are `period_ms = 10` and every step advances the
/// `VirtualClock` by exactly 10 ms, so after step `S` completes the clock stands at
/// `(S + 1) * 10 ms` and the next fire is due one period later. Neither fixture takes
/// data input, so the other two members are empty — which is part of the oracle.
///
/// The DEFERRED node's oracle uses this same function, and that is the point: the rule
/// "two carriers, identical bytes" is what stops a node's restorable timing state
/// depending on whether its struct happened to be inline-eligible.
fn framework_oracle(step: u64) -> NodeFrameworkState {
    NodeFrameworkState {
        next_fire_ns: Some((step + 2) * 10_000_000),
        pending_data_count: 0,
        sync_input_timestamps: Default::default(),
        // Neither fixture takes an input, so neither carries a
        // per-message FIFO service cursor and the section states no table.
        input_service: None,
    }
}

fn step_n(runtime: &mut GraphRuntime, n: usize) {
    for _ in 0..n {
        runtime.step(Duration::from_millis(10));
    }
}

/// Step until this process's capture child has been reaped, under a liveness ceiling.
///
/// The reap itself happens on the reaper thread; the RECORDS the parent owes are
/// written at the next boundary, so the loop steps rather than sleeps.
///
/// It SLEEPS rather than steps, and that is not a style choice. The reap happens on
/// the reaper thread and needs no boundary from this one, while a `VirtualClock` step
/// costs microseconds — so a spin-stepping wait runs tens of thousands of logical steps
/// through the reaper's poll interval, hits the cadence on most of them, and buries the
/// anchor under its own skip records (MEASURED: ~30 000 steps and three full memo
/// flushes for one child, which lapped a 4096-record ring). What that revealed is real
/// and by design — the memo is bounded and reports what it could not name — but it is
/// not what these tests are about.
///
/// It then steps EXACTLY once, which is the boundary where the post-mortem and the memo
/// flush run. Extra settling steps would, on a tight cadence,
/// fork AGAIN — so a test asserting the claim table was clean would read a perfectly
/// legitimate second claim as a leak.
fn wait_for_reap(runtime: &mut GraphRuntime) {
    let deadline = Instant::now() + CEILING;
    while runtime.capture_child_in_flight() {
        assert!(
            Instant::now() < deadline,
            "a capture child was still in flight after {CEILING:?}"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
    runtime.step(Duration::from_millis(10));
}

/// The `Counter` blob a complete record at step `s` must carry.
///
/// The rendezvous rule as arithmetic, and it is what makes every assertion here a
/// hand oracle rather than a self-compare: an anchor "at S" is the state AFTER step S
/// completed, steps are 0-based, so a node firing once per step has fired `S+1` times.
/// Deriving the expectation from the step the record CLAIMS is what catches a capture
/// that recorded the wrong step's state — it cannot agree with itself.
fn counter_oracle_at(step: u64) -> Vec<u8> {
    counter_anchor_oracle(step + 1, step)
}

/// The `Guarded` blob a complete record at step `s` must carry — it adds 2 per fire.
fn guarded_oracle_at(step: u64) -> Vec<u8> {
    guarded_anchor_oracle(2 * (step + 1), step)
}

// ===========================================================================
// the headline: a deferred node's own bytes reach the ring, via fork(2)
// ===========================================================================

#[test]
#[serial]
fn a_deferred_node_is_captured_by_the_fork_child_with_its_own_bytes() {
    // THE point of the whole carrier. `Guarded` carries a lock, so `INLINE_SAFE` is
    // false and the node thread refuses to run its encoder — and without the carrier that
    // is the end of the story: the node is reported as deferred and never captured.
    // With it a `fork(2)` child encodes it, and the oracle is built from a HAND-CONSTRUCTED
    // `Guarded` at the total the node must be holding at that boundary, so a child that
    // captured the wrong step's state fails on the bytes rather than on a count.
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("counter".to_string(), Box::new(CounterEntry::new()));
    factories.insert("guarded".to_string(), Box::new(GuardedEntry::new()));
    let ids = vec!["counter".to_string(), "guarded".to_string()];
    let mut h = harness("hl", factories, &ids, RING_RECORDS);

    h.arm.arm(1000, 4);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));
    step_n(&mut h.runtime, 5);

    let split = h
        .runtime
        .last_anchor_carriers()
        .expect("the boundary at step 4 took an anchor");
    assert_eq!(split.inline_nodes, 1);
    assert_eq!(split.forked_nodes, 1);

    wait_for_reap(&mut h.runtime);

    let events = drain(&h.ring_name);
    let got = completes(&events);

    // The INLINE half, written by the node thread.
    assert!(
        got.iter()
            .any(|(s, idx, bytes)| *s == 4 && *idx == 0 && *bytes == counter_oracle_at(4)),
        "the inline-eligible node's anchor must be present and correct: {:?}",
        got.iter()
            .map(|(s, i, b)| (s, i, b.len()))
            .collect::<Vec<_>>()
    );

    // The FORK half — the node the inline carrier refused, captured by the child. The rendezvous rule:
    // an anchor "at S" is the state AFTER step S completed and steps are 0-based, so a
    // node firing once per step has fired S+1 times; `Guarded` adds 2 per fire.
    let forked: Vec<&(u64, u32, Vec<u8>)> = got.iter().filter(|(_, idx, _)| *idx == 1).collect();
    assert_eq!(
        forked.len(),
        1,
        "exactly one anchor for the deferred node: {:?}",
        forked
            .iter()
            .map(|(s, i, b)| (s, i, b.len()))
            .collect::<Vec<_>>()
    );
    assert_eq!(forked[0].0, 4, "at the cadence's step, not the reap's");
    assert_eq!(
        forked[0].2,
        guarded_oracle_at(4),
        "the fork child must carry the node's state AS OF the boundary it forked at"
    );

    // And nothing was skipped: a carrier that forked AND reported the node missing
    // would satisfy the assertion above while telling a reader the opposite.
    assert!(
        skips(&events).is_empty(),
        "a captured node must not also be recorded as skipped: {:?}",
        skips(&events)
    );
}

// ===========================================================================
// The resync after a fork: without it the parent overwrites the child's records
// ===========================================================================

#[test]
#[serial]
fn the_anchor_after_a_fork_lands_past_the_childs_records_not_on_top_of_them() {
    // `fork` duplicates the producer, so the parent's LOCAL write cursor is
    // stale by everything the child wrote. Without `resync_after_fork` the parent's next
    // push re-uses the child's slots and `Release`-stores a LOWER cursor: the child's
    // anchor is overwritten mid-stream and the reader sees a torn one, or nothing.
    //
    // Several cadences, each with a fork and a reap, so the parent pushes AFTER a child
    // has advanced the shared cursor at least three times. The oracle is that EVERY
    // complete record matches the blob its own claimed step implies — a parent that
    // landed on top of a child loses records outright, and a cursor that went backwards
    // makes a later push overwrite an earlier anchor's tail, so a survivor would carry
    // a step it does not match.
    //
    // Nothing here counts steps. `wait_for_reap` steps an amount that depends on the
    // reaper's poll landing, so an assertion keyed to an absolute step number would be
    // asserting on scheduling; the oracles are derived from the step each record
    // CLAIMS, which is strictly stronger and cannot agree with itself.
    const CADENCE: u64 = 20;
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("counter".to_string(), Box::new(CounterEntry::new()));
    factories.insert("guarded".to_string(), Box::new(GuardedEntry::new()));
    let ids = vec!["counter".to_string(), "guarded".to_string()];
    let mut h = harness("rs", factories, &ids, RING_RECORDS);

    h.arm.arm(CADENCE, 4);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));
    for _ in 0..4 {
        step_n(&mut h.runtime, CADENCE as usize);
        wait_for_reap(&mut h.runtime);
    }

    let events = drain(&h.ring_name);
    let got = completes(&events);

    let forked_steps: Vec<u64> = got
        .iter()
        .filter(|(_, i, _)| *i == 1)
        .map(|(s, _, _)| *s)
        .collect();
    assert!(
        forked_steps.len() >= 3,
        "the sequence must contain at least three fork-carried anchors, or the parent \
         never pushed after a child had moved the cursor: {forked_steps:?}"
    );

    // EVERY record, both carriers, against the oracle its own step implies.
    for (step, node_idx, bytes) in &got {
        let oracle = match node_idx {
            0 => counter_oracle_at(*step),
            1 => guarded_oracle_at(*step),
            other => panic!("unexpected node index {other}"),
        };
        assert_eq!(
            bytes, &oracle,
            "the anchor recorded at S={step} for node {node_idx} does not carry that \
             step's state — an un-resynced parent overwrites a child's records and \
             Release-stores a lower cursor, which is exactly what this looks like"
        );
    }
    // The first anchor is still whole after every later one was written: the earliest
    // records are the ones an un-resynced parent re-uses first.
    let first = forked_steps.iter().copied().min().expect("a first anchor");
    assert!(
        got.iter().any(|(s, i, _)| *s == first && *i == 0),
        "the FIRST cadence's inline part must survive every later cadence: {:?}",
        got.iter().map(|(s, i, _)| (s, i)).collect::<Vec<_>>()
    );
    assert!(skips(&events).is_empty(), "{:?}", skips(&events));
}

// ===========================================================================
// One child at a time, and the skip that names it
// ===========================================================================

#[test]
#[serial]
fn a_cadence_reached_while_a_peer_is_encoding_is_skipped_and_says_so() {
    // One child at a time, driven through the arm word the way a PEER would drive it: a claim held by
    // another live process. An anchor is all-or-nothing across workers, so
    // starting one that a reader will reject as `PartialAnchor` wastes every worker's
    // boundary cost, CoW smear and barrier propagation.
    //
    // The claimant is this process's own pid, which is what a live peer looks like to
    // the gate (`kill(pid, 0)` succeeds), and it is what keeps the stale-claim sweep
    // from reclaiming it mid-test.
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("counter".to_string(), Box::new(CounterEntry::new()));
    let ids = vec!["counter".to_string()];
    let mut h = harness("busy", factories, &ids, RING_RECORDS);

    h.arm.arm(4, 4);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));

    let self_pid = unsafe { libc::getpid() };
    let slot = h.arm.claim(self_pid, 1024).expect("a free claim slot");
    assert_eq!(h.arm.busy_workers(), 1);

    step_n(&mut h.runtime, 5);

    let events = drain(&h.ring_name);
    assert!(
        completes(&events).is_empty(),
        "a busy peer must cost the WHOLE anchor, inline half included — a per-process \
         skip would have this worker fork for parts the reader then discards"
    );
    assert_eq!(
        skips(&events),
        vec![(4, 0, SkipCause::StillEncoding)],
        "and the skip must NAME the cause, so a reader can tell a busy graph from a \
         broken one"
    );
    // The walk never ran, so the split is still unset — the anti-tautology half: a
    // boundary that walked and then discarded its parts would report one here.
    assert!(
        h.runtime.last_anchor_carriers().is_none(),
        "the gate must run BEFORE the walk, or a doomed anchor still pays for it"
    );

    // Released, the very next cadence anchors normally: the skip is a deferral, never
    // a latch.
    h.arm.release(slot, self_pid);
    step_n(&mut h.runtime, 4);
    let after = completes(&drain(&h.ring_name));
    assert!(
        after.iter().any(|(s, _, _)| *s == 8),
        "the cadence after the peer released must anchor: {:?}",
        after
            .iter()
            .map(|(s, i, b)| (s, i, b.len()))
            .collect::<Vec<_>>()
    );
}

#[test]
#[serial]
fn a_cadence_skipped_while_our_own_child_ran_is_named_once_the_ring_is_ours_again() {
    // The half a peer's claim cannot show. When the busy child is OUR OWN, the ring's
    // producer role is the child's for its lifetime — so the `StillEncoding`
    // record CANNOT be pushed when the cadence is skipped, or it would interleave with
    // the child's own stream and tear whichever anchor lost the race.
    //
    // So the step is REMEMBERED and written once the child is reaped. This is the arm
    // that fails both ways: a carrier that pushed immediately corrupts the child's
    // anchor, and one that forgot the memo leaves the skipped cadence with no record at
    // all — indistinguishable from a recorder that stopped.
    //
    // The cadence is 1 step so a cadence is guaranteed to land while the child runs.
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("guarded".to_string(), Box::new(GuardedEntry::new()));
    let ids = vec!["guarded".to_string()];
    let mut h = harness("memo", factories, &ids, RING_RECORDS);

    h.arm.arm(1, 2);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));
    step_n(&mut h.runtime, 3);
    assert!(
        h.runtime.capture_child_in_flight(),
        "step 2 must have forked, or there is no window to memo anything in"
    );

    // ANTI-VACUITY: drive cadences WHILE the child holds the ring. Bounded, because the
    // point is that a few land — not how many. Without this the memo is never written
    // to and the assertion below would pass against a carrier that had no memo at all.
    let mut memoed = 0;
    while h.runtime.capture_child_in_flight() && memoed < 4 {
        h.runtime.step(Duration::from_millis(10));
        memoed += 1;
    }
    assert!(
        memoed > 0,
        "no cadence landed while the child ran, so this test proves nothing"
    );

    wait_for_reap(&mut h.runtime);
    // Two further boundaries so the flush has landed and the run has settled.
    step_n(&mut h.runtime, 2);

    let events = drain(&h.ring_name);
    let got = completes(&events);
    let skipped = skips(&events);

    assert!(
        got.iter().any(|(s, i, _)| *s == 2 && *i == 0),
        "the first cadence forked and its child's anchor must be present: {:?}",
        got.iter()
            .map(|(s, i, b)| (s, i, b.len()))
            .collect::<Vec<_>>()
    );
    assert!(
        skipped
            .iter()
            .any(|(_, _, cause)| *cause == SkipCause::StillEncoding),
        "a cadence landing while our own child ran must be NAMED, not silently absent: \
         {skipped:?}"
    );
    // And never at the step the child was capturing — that step has a real anchor, and
    // a reader holding a part AND a refusal for one node at one step has a
    // contradiction it cannot resolve.
    assert!(
        !skipped.iter().any(|(s, i, _)| *s == 2
            && *i == 0
            && got.iter().any(|(gs, gi, _)| gs == s && gi == i)),
        "no node may be both captured and skipped at one step: complete={:?} skipped={skipped:?}",
        got.iter().map(|(s, i, _)| (s, i)).collect::<Vec<_>>()
    );
}

#[test]
#[serial]
fn a_node_whose_encoder_refuses_in_the_child_is_never_published_as_a_complete_anchor() {
    // This is the worst outcome in the whole carrier.
    //
    // The child writes the carrier's 16-byte header, then calls the node's encoder. If
    // that encoder fails and the sink is FINISHED anyway, the ring carries a record
    // stream the reader reassembles into a COMPLETE anchor: `AnchorBlob::decode`
    // accepts it, a restore applies it, and the node comes back with whatever partial
    // bytes made it in — presented as its state at S. The SKIP record naming the
    // failure sits beside it, so a reader holds a valid-looking anchor AND a refusal
    // for one node at one step, with no rule for choosing.
    //
    // The sink is therefore DROPPED on the failure arm. A restore that finds nothing is
    // recoverable; one that finds a plausible lie is not.
    //
    // The fixture is a hand-written `NodeEntry`, because no GENERATED encoder can fail
    // — which is exactly why this path needs a driver of its own rather than an
    // artificially injected fault.
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("counter".to_string(), Box::new(CounterEntry::new()));
    factories.insert("refusing".to_string(), Box::new(RefusingCapture::default()));
    let ids = vec!["counter".to_string(), "refusing".to_string()];
    let mut h = harness("ref", factories, &ids, RING_RECORDS);

    h.arm.arm(1000, 4);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));
    step_n(&mut h.runtime, 5);
    let split = h
        .runtime
        .last_anchor_carriers()
        .expect("the boundary at step 4 took an anchor");
    assert_eq!(
        split.forked_nodes, 1,
        "the hand-written node must reach the FORK carrier, or this drives nothing"
    );
    wait_for_reap(&mut h.runtime);

    let events = drain(&h.ring_name);
    let got = completes(&events);

    // THE assertion: no complete anchor for the refusing node, at any step.
    assert!(
        !got.iter().any(|(_, idx, _)| *idx == 1),
        "a node whose encoder refused must NOT be published as a complete anchor: {:?}",
        got.iter()
            .map(|(s, i, b)| (s, i, b.len()))
            .collect::<Vec<_>>()
    );
    // And the refusal is NAMED, so its absence is a diagnosis rather than a gap.
    assert!(
        skips(&events)
            .iter()
            .any(|(s, i, cause)| *s == 4 && *i == 1 && *cause == SkipCause::CaptureFailed),
        "and the failure must be recorded against that node: {:?}",
        skips(&events)
    );
    // ANTI-TAUTOLOGY: the healthy sibling still anchors, so this is not a carrier that
    // simply stopped publishing.
    assert!(
        got.iter()
            .any(|(s, i, b)| *s == 4 && *i == 0 && *b == counter_oracle_at(4)),
        "one node's refusal must not cost its sibling its anchor: {:?}",
        got.iter()
            .map(|(s, i, b)| (s, i, b.len()))
            .collect::<Vec<_>>()
    );
}

// ===========================================================================
// the fork-site discipline
// ===========================================================================

#[test]
#[serial]
fn a_guard_held_across_the_fork_would_make_every_anchor_partial() {
    // THE new invariant, and the reason `take_anchor` drops its guards where it does.
    //
    // A fork child has exactly one thread, so a `MutexGuard` alive at the fork instant
    // is held forever in the child's image BY NOBODY. Every one of the child's own
    // `try_lock`s then fails, every node is recorded as refused, and the anchor is
    // silently partial on every cadence — no error, no log, no signal.
    //
    // The invariant lives in the caller's stack frame, which no signature can see, so
    // it is pinned by driving the failure directly: a lock held by THIS thread across
    // the boundary is exactly what a mis-ordered `drop(guards)` produces in the child.
    // `Guarded`'s declared lock is held here, so the child's `cer_probe`-clean node is
    // reached with its state lock taken.
    //
    // What it must NOT do is corrupt anything: the correct outcome is a NAMED refusal.
    let shared = Arc::new(Mutex::new(3u64));
    let guarded = GuardedEntry::with_state(Guarded {
        shared: Arc::clone(&shared),
        ..Default::default()
    });
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("guarded".to_string(), Box::new(guarded));
    let ids = vec!["guarded".to_string()];
    let mut h = harness("gd", factories, &ids, RING_RECORDS);

    const CADENCE: u64 = 10;
    h.arm.arm(CADENCE, 4);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));

    // The CONTROL first, in the same body: with nothing held, the child captures.
    step_n(&mut h.runtime, 5);
    wait_for_reap(&mut h.runtime);
    let healthy = completes(&drain(&h.ring_name));
    assert!(
        healthy.iter().any(|(s, i, _)| *s == 4 && *i == 0),
        "the control arm must capture, or the failure arm below proves nothing: {:?}",
        healthy
            .iter()
            .map(|(s, i, b)| (s, i, b.len()))
            .collect::<Vec<_>>()
    );

    // Now the failure. The declared-lock probe catches a held DECLARED lock BEFORE the fork —
    // which is itself the protection — so the anchor is skipped WHOLE and named. That
    // is the rule for this shape: never fork into a held lock at all.
    let held = shared.lock().expect("uncontended");
    step_n(&mut h.runtime, (CADENCE * 3) as usize);
    drop(held);

    let events = drain(&h.ring_name);
    let contended: Vec<u64> = skips(&events)
        .into_iter()
        .filter(|(_, _, cause)| *cause == SkipCause::Contended)
        .map(|(s, _, _)| s)
        .collect();
    assert!(
        !contended.is_empty(),
        "at least one cadence landed while the declared lock was held, and it must be \
         NAMED rather than silently absent: {:?}",
        skips(&events)
    );
    // And not one of those steps produced a capture. Keyed to the steps that were
    // actually refused rather than to a step number this test guessed, so it cannot
    // pass by asserting about a cadence that never happened.
    let captured: Vec<u64> = completes(&events).into_iter().map(|(s, _, _)| s).collect();
    for step in &contended {
        assert!(
            !captured.contains(step),
            "step {step} was refused for contention AND captured — a held declared lock \
             must never be forked into, because the child would hold it forever"
        );
    }
}

// ===========================================================================
// the memory gate
// ===========================================================================

#[test]
#[serial]
fn every_claim_a_fork_takes_is_given_back_with_the_bytes_it_reserved() {
    // The memory gate's ACCOUNTING, which is the half of the gate a leak destroys. The
    // reservation is claimed before the fork and released on the reap, and a claim that
    // is not given back is the stale-claim failure mode arriving through the mechanism
    // meant to prevent it: `busy_workers` never returns to zero, and from then on every
    // worker — this one and every peer — skips every cadence as `StillEncoding`, for
    // the life of the run, silently.
    //
    // SCOPE. The DECLINE arm is not reachable end to end from here, and saying
    // so is better than a test that looks like it drives it. The gate reads
    // `busy_workers` FIRST, so any reservation big enough to breach the floor has to be
    // attached to a live claim — which skips the anchor before the memory check is
    // reached. The only reservation that survives to the check is this worker's OWN,
    // which is its real anon RSS. Driving the other arm needs an injectable floor, i.e.
    // a test seam in the production path; the arithmetic is instead oracle-tested
    // directly in `state_carrier::fork::memory_verdict`, and what is pinned HERE is the
    // property that seam could not check anyway: the claim really is taken, and really
    // is given back, over a real fork.
    const CADENCE: u64 = 1000;
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("guarded".to_string(), Box::new(GuardedEntry::new()));
    let ids = vec!["guarded".to_string()];
    let mut h = harness("mem", factories, &ids, RING_RECORDS);

    h.arm.arm(CADENCE, 4);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));
    assert_eq!(h.arm.busy_workers(), 0, "nothing is claimed before the arm");

    step_n(&mut h.runtime, 5);
    // ANTI-TAUTOLOGY: a carrier that never claimed at all would satisfy every
    // assertion below. The claim is observable only while the child is in flight, so
    // it is read there — and the fork having happened is what makes that window exist.
    assert!(
        h.runtime.capture_child_in_flight(),
        "the boundary at step 4 must have forked, or the release below proves nothing"
    );
    assert_eq!(
        h.arm.busy_workers(),
        1,
        "and it must hold exactly one claim while its child runs"
    );
    assert!(
        h.arm.rss_reserved() > 0,
        "with a real projection attached to it — a zero reservation would make the \
         memory gate inert for every peer as well"
    );

    wait_for_reap(&mut h.runtime);

    assert_eq!(
        h.arm.busy_workers(),
        0,
        "every claim this worker took must be given back"
    );
    assert_eq!(
        h.arm.rss_reserved(),
        0,
        "and so must every byte it reserved"
    );
}

// ===========================================================================
// teardown
// ===========================================================================

#[test]
#[serial]
fn dropping_an_armed_runtime_joins_the_reaper_before_the_breadcrumb_goes() {
    // The reaper thread reads the breadcrumb through an `Arc` and WRITES the arm word,
    // which belongs to a recorder that may `shm_unlink` it the moment the runtime is
    // gone. `Drop` therefore stops and JOINS rather than signalling, and it clears the
    // panic hook's pointer so a panic during the rest of teardown cannot stamp a page
    // about to be unmapped.
    //
    // A thread outliving either is a use-after-free with no error path, so the arm is
    // that the drop RETURNS — under a ceiling, since the failure mode is a hang — and
    // that the claim table is clean afterwards.
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("guarded".to_string(), Box::new(GuardedEntry::new()));
    let ids = vec!["guarded".to_string()];
    let mut h = harness("drop", factories, &ids, RING_RECORDS);

    h.arm.arm(1000, 4);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));
    step_n(&mut h.runtime, 5);
    wait_for_reap(&mut h.runtime);

    let began = Instant::now();
    drop(h.runtime);
    let elapsed = began.elapsed();
    assert!(
        elapsed < CEILING,
        "dropping an armed runtime must not hang: took {elapsed:?}"
    );
    assert_eq!(
        h.arm.busy_workers(),
        0,
        "and it must leave no claim behind for the next run's stale sweep"
    );

    // The arm word outlives the runtime here, which is the shipping shape (`bagd` owns
    // it), and reading it after the drop is what proves the reaper is not still writing
    // to it.
    assert!(h.arm.is_armed());
}

#[test]
#[serial]
fn a_slow_but_progressing_encode_is_never_killed_and_lands_whole() {
    // The watchdog rule, end to end: a LIVENESS check, never a duration cap.
    //
    // A duration cap such as `max(5 s, 10x observed)` kills the wrong child: a
    // 500 MB serde encode costs 2.5-10 s — so on a Jetson at
    // the slow end the first child dies mid-encode at the floor, "observed" is undefined
    // for a child that never completed, and every subsequent attempt dies at the same
    // point. That node then never produces an anchor for the life of the robot: a
    // node-level, COST-DERIVED refusal, which is the outcome the liveness check avoids.
    //
    // The fixture takes far longer than a single reaper poll and bumps the progress word
    // throughout (every write through the child's sink does), so it must complete —
    // and the assertion is on the BYTES, hand-built from the same loop, because a
    // watchdog that killed it partway would still leave a plausible-looking prefix.
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("counter".to_string(), Box::new(CounterEntry::new()));
    factories.insert("slow".to_string(), Box::new(SlowCapture::default()));
    let ids = vec!["counter".to_string(), "slow".to_string()];
    let mut h = harness("slow", factories, &ids, RING_RECORDS);

    h.arm.arm(1000, 4);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));
    step_n(&mut h.runtime, 5);

    // ANTI-VACUITY: the encode really does outlast several reaper polls, so a duration
    // cap would have had every chance to fire. Asserted as a LOWER bound on the child's
    // life, never as a band — a loaded runner can only make it longer.
    let began = Instant::now();
    wait_for_reap(&mut h.runtime);
    let lived = began.elapsed();
    assert!(
        lived > SLOW_CHUNK_PAUSE * (SLOW_CHUNKS as u32) / 2,
        "the child must have lived long enough for a duration cap to be reachable: {lived:?}"
    );

    let events = drain(&h.ring_name);
    let got = completes(&events);
    let slow = got
        .iter()
        .find(|(s, idx, _)| *s == 4 && *idx == 1)
        .unwrap_or_else(|| {
            panic!(
                "the slow node's anchor must be present and COMPLETE — a watchdog that \
                 counted elapsed time rather than stalled time kills exactly this: {:?} \
                 skips={:?}",
                got.iter()
                    .map(|(s, i, b)| (s, i, b.len()))
                    .collect::<Vec<_>>(),
                skips(&events)
            )
        });

    // The bytes, against a hand-built oracle: header (magic + shape + the
    // framework section) then the payload the loop produces. A killed child leaves
    // a prefix that would satisfy "present".
    //
    // The section is written by hand here rather than through the production
    // encoder, so this arm pins the fork carrier's FRAMING as bytes: a deferred
    // node's anchor must be framed exactly like an inline one (two
    // carriers, identical bytes), and a fork path that quietly wrote the v1
    // header would leave every deferred node's restore inventing its next fire.
    let framework = framework_oracle(4);
    let mut oracle = Vec::new();
    oracle.extend_from_slice(&cerulion_core::state_restore::ANCHOR_BLOB_MAGIC_V2.to_le_bytes());
    oracle.extend_from_slice(&0x5107_0000_0000_1053u64.to_le_bytes());
    oracle.extend_from_slice(&(framework.encoded_len() as u32).to_le_bytes());
    {
        let mut sink = VecSink::new();
        framework.encode(&mut sink).expect("the section fits");
        oracle.extend_from_slice(&sink.into_inner());
    }
    oracle.extend_from_slice(&slow_capture_payload());
    assert_eq!(
        slow.2, oracle,
        "the slow node's anchor must carry every byte its encoder wrote"
    );
    assert!(
        skips(&events).is_empty(),
        "and nothing may be recorded as skipped: {:?}",
        skips(&events)
    );
}

#[test]
#[serial]
fn the_reaper_never_steals_a_sibling_processs_exit_status() {
    // Reaping: targeted `waitpid(pid, WNOHANG)`, NEVER `waitpid(-1)`.
    //
    // This is not hypothetical hygiene. The monolith graph process already owns
    // `std::process::Child` handles for its workers, for `bagd` and for the gateway,
    // each reaped through a targeted `try_wait()`. A wildcard wait in the capture
    // reaper steals whichever of them exits first — and an exit status stolen from
    // `bagd` is a recording that reports the wrong thing about its own finalisation.
    //
    // The sibling here stands in for exactly those: an ordinary `std::process::Child`
    // owned by this process, exiting with a DISTINCTIVE code while a capture child is
    // being reaped alongside it. Its owner must still be able to collect that code.
    // The window is made DETERMINISTIC by the capture child being SLOW: while it
    // encodes, the sibling is the ONLY zombie this process has, so a wildcard wait has
    // exactly one thing to find and must find it. With a fast capture child the two
    // become zombies together and which one a wildcard wait returns is the kernel's
    // choice — a variant that survives most runs, which is how a wildcard wait would
    // reach production. (MEASURED: with a fast child, a `waitpid(-1)` variant passed
    // this suite outright.)
    const SIBLING_EXIT: i32 = 57;
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("slow".to_string(), Box::new(SlowCapture::default()));
    let ids = vec!["slow".to_string()];
    let mut h = harness("sib", factories, &ids, RING_RECORDS);

    h.arm.arm(1000, 4);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));
    step_n(&mut h.runtime, 5);
    assert!(
        h.runtime.capture_child_in_flight(),
        "a capture child must be in flight, or a wildcard wait has no window to steal in"
    );

    // Spawned AFTER the fork so it is not inherited by the capture child, and exiting
    // at once so it is a zombie for the whole of that child's encode.
    let mut sibling = std::process::Command::new("/bin/sh")
        .arg("-c")
        .arg(format!("exit {SIBLING_EXIT}"))
        .spawn()
        .expect("spawn the sibling process");

    wait_for_reap(&mut h.runtime);

    // Its status is collected after every wait the reaper made while the slow child ran.
    let status = sibling.wait().expect(
        "the sibling's OWNER must still be able to reap it — an ECHILD here is the \
         capture reaper having stolen it with `waitpid(-1)`",
    );
    assert_eq!(
        status.code(),
        Some(SIBLING_EXIT),
        "and it must get the sibling's OWN exit status, not a capture child's"
    );

    // ANTI-TAUTOLOGY: the capture child really was reaped by this run, so the two
    // reaps genuinely overlapped rather than the carrier simply never forking.
    assert!(
        !h.runtime.capture_child_in_flight(),
        "the capture child must have been reaped by its own targeted wait"
    );
}

#[test]
#[serial]
fn an_unarmed_run_forks_nothing_and_spawns_nothing() {
    // The zero-cost property extended to the fork carrier: no arm, no breadcrumb, no
    // reaper thread, no `fork(2)`. This is the arm that fails if the carrier is ever
    // brought up at BUILD time rather than at arm time — which would put a thread and a
    // mapping into every graph on the robot, armed or not.
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("guarded".to_string(), Box::new(GuardedEntry::new()));
    let ids = vec!["guarded".to_string()];
    let mut h = harness("cold", factories, &ids, RING_RECORDS);

    // Deliberately NO `attach_state_arm`.
    step_n(&mut h.runtime, 12);

    assert!(!h.runtime.capture_child_in_flight());
    assert!(h.runtime.last_anchor_carriers().is_none());
    assert!(
        drain(&h.ring_name).is_empty(),
        "an un-armed run must write NOTHING — not an anchor, not a skip"
    );
    assert_eq!(h.arm.busy_workers(), 0, "and claim nothing");
}
