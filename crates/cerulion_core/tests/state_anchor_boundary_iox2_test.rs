// SPDX-License-Identifier: AGPL-3.0-only
//! The ANCHOR BOUNDARY, driven through a REAL `GraphRuntime`
//! over REAL iceoryx2, a REAL mapped arm word and a REAL POSIX-SHM state ring.
//!
//! `state_carrier::anchor`'s in-module tests pin the WALK against a synthetic
//! target. This file pins the thing the walk was built for: that a running graph,
//! armed by a recorder, puts anchors in a ring a recorder can read — and that it
//! does so at the steps the cadence names, with the bytes the node actually held,
//! and NOT AT ALL when nothing armed it.
//!
//! # Every oracle here is hand-written
//!
//! A node's captured payload is compared against a blob built by the production
//! capture helper over a HAND-CONSTRUCTED value of the same type, never against
//! another run of the same graph. So a capture that recorded the wrong step's
//! state, or the wrong node's, fails — where a self-compare would agree with
//! itself.
//!
//! # Why the arm is created and armed by the TEST
//!
//! Because that is what a recorder does: `bagd --state-arm` creates the word, arms
//! it with a cadence, and holds it for the recording's life. Driving the same API
//! keeps the fixture on the production seam rather than on a test-only setter, so
//! an arm that stopped taking effect would fail here rather than pass.
//!
//! # Running
//!
//! ```bash
//! cargo test -p cerulion_core --test state_anchor_boundary_iox2_test
//! ```

#![cfg(unix)]

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::state::{SkipCause, VecSink};
use cerulion_core::state_arm::MappedStateArm;
use cerulion_core::state_restore::{
    capture_anchor_blob_with_framework, AnchorBlob, InputServiceCursor, NodeFrameworkState,
};
use cerulion_core::state_ring::{
    StateAnchorEvent, StateAssembler, StateRingConsumer, StateRingOwner,
};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;

// ---------------------------------------------------------------------------
// fixtures
// ---------------------------------------------------------------------------

/// Plain data: inline-eligible, so this node takes the INLINE carrier.
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

/// Carries a lock: `INLINE_SAFE = false`, so this node is DEFERRED to the fork
/// carrier and the inline half must produce no anchor for it.
#[cerulion_node(period_ms = 10)]
struct Guarded {
    #[output]
    out: Vector3,
    shared: Arc<Mutex<u64>>,
}

#[cerulion_node_impl]
impl Guarded {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        Ok(())
    }
}

/// A data-trigger consumer on an absolute external source, so the
/// anchor's framework section must carry the input's SERVICE CURSOR — the wire
/// `sequence` of the last frame `tick()` actually read.
#[cerulion_node]
struct Tally {
    #[input(trigger)]
    inp: Vector3,
    total: u64,
}

#[cerulion_node_impl]
impl Tally {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.total += self.inp.x as u64;
        Ok(())
    }
}

/// A PER-SET Sync consumer on two absolute external sources.
///
/// Per-set delivery marks a Sync TRIGGER input per-message FIFO
/// (`mark_fifo_consume`) and mints it a SERVICE CURSOR, exactly as the FIFO switch
/// does for a data-trigger input — so this node's anchor must state a cursor
/// PER TRIGGER INPUT. That is not bookkeeping: `resolve_pre_anchor_service` is
/// the only thing that caps a resumed replay's pre-anchor injection, and a
/// section that named no cursor for these inputs would let the first replayed
/// step re-serve every frame the recorded run had already consumed — one fire
/// per re-served SET under per-set semantics, i.e. a whole-replay
/// `ByteMismatch` blamed on the candidate.
#[cerulion_node(sync_window_ms = 50)]
struct SyncTally {
    #[input(trigger)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    total: u64,
}

#[cerulion_node_impl]
impl SyncTally {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.total += (self.a.x + self.b.x) as u64;
        Ok(())
    }
}

/// A consumer that PANICS on a sentinel value, so the panic happens
/// INSIDE the tick — i.e. inside the `f` that `build_inbound_view` invokes,
/// since `cerulion_macros`' `build_nested_try_view` makes the tick body the leaf
/// of the nested `try_view` closures.
#[cerulion_node]
struct PanicTally {
    #[input(trigger)]
    inp: Vector3,
    total: u64,
}

#[cerulion_node_impl]
impl PanicTally {
    fn tick(&mut self) -> Result<(), NodeError> {
        let x = self.inp.x;
        assert!(x < 500.0, "deliberate mid-tick panic on {x}");
        self.total += x as u64;
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// harness
// ---------------------------------------------------------------------------

const RING_RECORDS: u32 = 4096;
const RUN_ID: u64 = 0xC0FF_EE10_0053;

fn unique(tag: &str) -> String {
    format!(
        "c6{tag}{}{}",
        std::process::id() % 1000,
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .expect("clock")
            .subsec_nanos()
            % 10_000
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
        identity: "anchor".to_string(),
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

/// Everything a recorder sets up, in the order a recorder sets it up.
struct Harness {
    runtime: GraphRuntime,
    /// Held so the ring's name stays live for the consumer.
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
    harness_with_config(tag, graph(&prefix, &ids), factories, node_ids, ring_records)
}

/// [`harness`] over a caller-built graph, for fixtures the uniform
/// output-only [`graph`] shape cannot express (the data-trigger arm).
fn harness_with_config(
    tag: &str,
    config: GraphConfig,
    factories: IndexMap<String, Box<dyn NodeEntry>>,
    node_ids: &[String],
    ring_records: u32,
) -> Harness {
    let ids: Vec<&str> = node_ids.iter().map(|s| s.as_str()).collect();
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, Arc::new(VirtualClock::new()), 16)
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

/// Every anchor event the ring holds, in order.
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

/// The framework section a `Counter` anchored at step `S` must carry.
///
/// HAND-COMPUTED, never read back off the scheduler: `Counter` is
/// `period_ms = 10` and every step advances the `VirtualClock` by exactly 10 ms,
/// so after step `S` completes the clock stands at `(S + 1) * 10 ms` and the next
/// fire is due one period later. A `Period` node signals no data arrivals and
/// aligns no sync inputs, so the other two members are empty — which is itself
/// part of the oracle: a section that fabricated either would fail here.
///
/// The `input_service` cursor table is `None` for the same reason and it is equally
/// part of the oracle: a service cursor is minted only for a PER-MESSAGE FIFO
/// (data-trigger) input, and `Counter` has no input at all — so a capture that
/// stated a table here would be inventing one.
fn counter_framework_oracle(step: u64) -> cerulion_core::state_restore::NodeFrameworkState {
    cerulion_core::state_restore::NodeFrameworkState {
        next_fire_ns: Some((step + 2) * 10_000_000),
        pending_data_count: 0,
        sync_input_timestamps: Default::default(),
        input_service: None,
    }
}

/// The blob a `Counter` holding `count` must produce at step `step`, built
/// through the SAME production helper a restore reads back — over a
/// hand-constructed value and a hand-computed framework section, never over
/// another run of the graph.
fn counter_anchor_oracle(count: u64, step: u64) -> Vec<u8> {
    // Functional update rather than a plain literal: the node macro injects a
    // hidden runtime field, so every member must come from somewhere.
    let node = Counter {
        count,
        ..Default::default()
    };
    let mut sink = VecSink::new();
    // The boundary writes the v2 framing whenever the scheduler
    // has something to say about the node, and a `Period` node always does (its
    // `next_fire_ns`). So the oracle is the WITH-FRAMEWORK helper — using the v1
    // one here would pin the v1 (section-less) bytes and make the section undetectable.
    capture_anchor_blob_with_framework(&node, &counter_framework_oracle(step), &mut sink)
        .expect("a counter fits any sink");
    sink.into_inner()
}

fn step_n(runtime: &mut GraphRuntime, n: usize) {
    for _ in 0..n {
        runtime.step(Duration::from_millis(10));
    }
}

// ===========================================================================
// the boundary
// ===========================================================================

#[test]
fn an_armed_run_puts_the_nodes_own_state_in_the_ring_at_the_cadences_steps() {
    // THE headline. The oracle is built from a hand-constructed `Counter` at the
    // count the node must be holding when the anchor is taken, through the same
    // production helper a restore reads back — so a capture that recorded the
    // wrong STEP's state fails on the bytes, not merely on the count of records.
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("counter".to_string(), Box::new(CounterEntry::new()));
    let ids = vec!["counter".to_string()];
    let mut h = harness("hl", factories, &ids, RING_RECORDS);

    // Arm from step 4, every 3 steps: anchors at 4, 7, 10.
    h.arm.arm(3, 4);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));

    step_n(&mut h.runtime, 11);

    let events = drain(&h.ring_name);
    let got = completes(&events);
    let steps: Vec<u64> = got.iter().map(|(s, _, _)| *s).collect();
    assert_eq!(
        steps,
        vec![4, 7, 10],
        "anchors land at first_anchor_step and every cadence after it, and nowhere else"
    );
    assert!(
        got.iter().all(|(_, idx, _)| *idx == 0),
        "the only node resolves to manifest index 0: {got:?}"
    );

    // The rendezvous rule, as arithmetic: an anchor "at S" is the state AFTER
    // step S COMPLETED, and steps are 0-based, so a node that fires once per step
    // has fired S+1 times by then. That +1 is not an adjustment to make the test
    // pass — it is the invariant a restore depends on, and getting it wrong in
    // either direction is exactly the class this oracle exists to catch (a
    // capture taken BEFORE the level loop would read S, one taken a step late
    // would read S+2).
    for (step, _, bytes) in &got {
        let oracle = counter_anchor_oracle(*step + 1, *step);
        assert_eq!(
            bytes, &oracle,
            "the anchor at S={step} must carry the node's state AS OF that boundary"
        );
    }
}

#[test]
fn nothing_is_written_while_no_recorder_has_armed_the_run() {
    // The zero-cost property, as behaviour: an un-armed runtime holds a ring and
    // still never touches it. This is the arm that fails if the boundary hook
    // stops consulting `due()` — the headline test above would still pass against
    // a hook that anchored on EVERY step, because it only asserts the steps it
    // wanted are present.
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("counter".to_string(), Box::new(CounterEntry::new()));
    let ids = vec!["counter".to_string()];
    let mut h = harness("un", factories, &ids, RING_RECORDS);

    // Deliberately NO `attach_state_arm`.
    step_n(&mut h.runtime, 12);

    assert!(
        drain(&h.ring_name).is_empty(),
        "an un-armed run must write NOTHING — not an anchor, not a skip"
    );
}

#[test]
fn an_armed_but_disarmed_word_stops_the_anchors_without_stopping_the_graph() {
    // `bagd` DISARMS the word at finalize, so a graph that outlives its recorder
    // stops anchoring into a ring nobody drains. Pinned as a transition rather
    // than a state: anchors before, none after, and the graph keeps stepping.
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("counter".to_string(), Box::new(CounterEntry::new()));
    let ids = vec!["counter".to_string()];
    let mut h = harness("dis", factories, &ids, RING_RECORDS);

    h.arm.arm(2, 2);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));
    step_n(&mut h.runtime, 5);
    let before = completes(&drain(&h.ring_name)).len();
    assert!(before > 0, "the arm must have produced anchors first");

    h.arm.disarm();
    step_n(&mut h.runtime, 8);
    let after = completes(&drain(&h.ring_name)).len();
    assert_eq!(
        after, before,
        "a disarmed word must stop anchoring — the recorder is gone and nothing drains the ring"
    );
}

#[test]
fn a_polled_replay_style_run_never_anchors() {
    // `step()` IS `step_live` with equal deltas, so the anchor hook is on the
    // replay path too — and a resim that produced anchors would write into a ring
    // its own recording never had. It cannot happen because nothing on that path
    // attaches an arm, and this pins that rather than leaving it to the argument:
    // a runtime driven exactly as `replay_engine` drives one, with a ring attached
    // and no arm, writes nothing.
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("counter".to_string(), Box::new(CounterEntry::new()));
    let ids = vec!["counter".to_string()];
    let mut h = harness("rp", factories, &ids, RING_RECORDS);

    for _ in 0..20 {
        h.runtime.step(Duration::from_millis(1));
    }

    assert!(
        drain(&h.ring_name).is_empty(),
        "a polled run with no arm must never anchor, however many steps it takes"
    );
}

// ===========================================================================
// the two filters
// ===========================================================================

#[test]
fn a_lock_carrying_node_is_deferred_while_its_plain_sibling_still_anchors() {
    // The `INLINE_SAFE` gate, end to end. `Guarded` is not `INLINE_SAFE`, so the inline
    // carrier must not capture it — and, decisively, must not let its deferral
    // cost `Counter` its anchor. A gate that refused the whole boundary on the
    // first ineligible node would fail here; so would one that captured `Guarded`
    // inline anyway.
    //
    // The assertion is on the CARRIER SPLIT rather than on what happens to be in
    // the ring, and that is not a convenience. The
    // deferred node IS captured — by the fork carrier's child — so a ring-contents assertion
    // taken right after the boundary would be asserting on a race between this
    // thread and a process that has just been forked, and it would pass or fail
    // on scheduling. `last_anchor_carriers` is the decision itself: one node
    // inline, one GATED (refused entry before its encoder was ever entered, which
    // is the gate's property) and therefore forked. The fork carrier's own end-to-end
    // arms live in `state_anchor_fork_iox2_test`.
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("counter".to_string(), Box::new(CounterEntry::new()));
    factories.insert("guarded".to_string(), Box::new(GuardedEntry::new()));
    let ids = vec!["counter".to_string(), "guarded".to_string()];
    let mut h = harness("gate", factories, &ids, RING_RECORDS);

    h.arm.arm(4, 4);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));
    step_n(&mut h.runtime, 5);

    let split = h
        .runtime
        .last_anchor_carriers()
        .expect("the boundary at step 4 took an anchor");
    assert_eq!(split.inline_nodes, 1, "`counter` is inline-eligible");
    assert_eq!(split.forked_nodes, 1, "`guarded` is not");
    assert_eq!(
        split.gated_nodes, 1,
        "and it was gated BEFORE its encoder ran — an overflow would report \
         forked_nodes without gated_nodes"
    );

    // The inline half's own part is in the ring immediately, because the inline
    // walk writes it on this thread before the fork is even considered.
    let got = completes(&drain(&h.ring_name));
    assert!(
        got.iter().any(|(s, idx, _)| *s == 4 && *idx == 0),
        "the inline-eligible node's anchor is written by the node thread, so it is \
         present without waiting for anything: {got:?}"
    );
}

#[test]
fn a_held_declared_lock_skips_the_whole_anchor_and_names_every_covered_node() {
    // A contended lock at this boundary means a FOREIGN holder, so the
    // anchor is skipped WHOLE rather than attempted — forking into a held lock
    // would leave the child holding it forever. And the skip names EVERY node the
    // anchor would have covered, because a reader reassembles per node
    // and a single record on one node would leave the rest indistinguishable from
    // parts still in flight.
    let shared = Arc::new(Mutex::new(7u64));
    // Functional update: the node macro injects a hidden runtime field, so the
    // rest must come from `Default`.
    let guarded = GuardedEntry::with_state(Guarded {
        shared: Arc::clone(&shared),
        ..Default::default()
    });
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("counter".to_string(), Box::new(CounterEntry::new()));
    factories.insert("guarded".to_string(), Box::new(guarded));
    let ids = vec!["counter".to_string(), "guarded".to_string()];
    let mut h = harness("cont", factories, &ids, RING_RECORDS);

    h.arm.arm(4, 4);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));

    let held = shared.lock().expect("uncontended");
    step_n(&mut h.runtime, 5);
    drop(held);

    let events = drain(&h.ring_name);
    assert!(
        completes(&events).is_empty(),
        "a contended probe must produce NO capture at all, not a partial one"
    );
    let got = skips(&events);
    assert_eq!(
        got,
        vec![(4, 0, SkipCause::Contended), (4, 1, SkipCause::Contended)],
        "one skip per covered node, both naming the contention"
    );

    // And the next cadence, with the lock released, anchors normally — the skip is
    // a deferral, never a latch.
    step_n(&mut h.runtime, 4);
    let after = completes(&drain(&h.ring_name));
    assert!(
        after.iter().any(|(s, _, _)| *s == 8),
        "the cadence after the contention must anchor: {after:?}"
    );
}

#[test]
fn a_ring_too_full_to_hold_an_anchor_is_declined_rather_than_blocked_on() {
    // A `BACKPRESSURE` push BLOCKS when the ring is full
    // and then LAPS — right for a fork child nobody waits for, catastrophic on the
    // node thread, where under the multi-process barrier a peer blocks behind it.
    //
    // The ring here is smaller than one anchor's precheck demands, so the boundary
    // must DECLINE. The test would still pass on a blocking implementation if the
    // ring merely had room, which is why the ring is sized below the requirement
    // rather than merely filled: there is no room for the push to find.
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("counter".to_string(), Box::new(CounterEntry::new()));
    let ids = vec!["counter".to_string()];
    // 8 records is far below the arena's worth of payload plus headroom.
    let mut h = harness("full", factories, &ids, 8);

    h.arm.arm(4, 4);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));

    let began = std::time::Instant::now();
    step_n(&mut h.runtime, 5);
    let elapsed = began.elapsed();

    let events = drain(&h.ring_name);
    assert!(
        completes(&events).is_empty(),
        "the anchor must not be written into a ring that cannot hold it"
    );
    assert_eq!(
        skips(&events),
        vec![(4, 0, SkipCause::RecorderBehind)],
        "and the operator must be told the RECORDER is behind — every other cause \
         in the vocabulary points at the encoder"
    );
    // The wall bound is deliberately loose: it separates "declined" from "waited
    // out a multi-second backpressure timeout", which is orders of magnitude, and
    // a tight bound here would be the load-inversion class (a loaded runner inverting it).
    assert!(
        elapsed < Duration::from_secs(2),
        "the boundary must DECLINE, not wait: 5 steps took {elapsed:?}"
    );
}

// ===========================================================================
// The framework section the CARRIER supplies
// ===========================================================================

/// The anchor a running graph writes CARRIES the scheduler's framework
/// section, and it decodes to the scheduler's real state.
///
/// # Why this arm exists at all
///
/// `NodeFrameworkState` is the state a REBUILT `Scheduler` cannot recover:
/// `next_fire_ns`, `pending_data_count`, `sync_input_timestamps`. Nothing in a
/// node's own `cer_capture` can supply it — the node encodes the USER's fields
/// and knows nothing about the scheduler driving it — so if the CARRIER does not
/// write it, it is absent from every anchor the product ever records, and the
/// whole framework half of restore is inert while the encode/decode pair it rides on
/// stays green in its own unit tests.
///
/// The oracle is HAND-COMPUTED from the fixture's own period and the step the
/// anchor names, so it cannot be satisfied by reading the scheduler back through
/// the same accessor the carrier used.
#[test]
fn an_anchor_carries_the_schedulers_framework_section_decoded_to_a_hand_oracle() {
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("counter".to_string(), Box::new(CounterEntry::new()));
    let ids = vec!["counter".to_string()];
    let mut h = harness("ws2c", factories, &ids, RING_RECORDS);

    h.arm.arm(3, 4);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));
    step_n(&mut h.runtime, 11);

    let events = drain(&h.ring_name);
    let got = completes(&events);
    assert_eq!(
        got.iter().map(|(s, _, _)| *s).collect::<Vec<_>>(),
        vec![4, 7, 10],
        "the cadence is unchanged by the framing — this arm is about the BYTES"
    );

    for (step, _, bytes) in &got {
        let blob = AnchorBlob::decode("counter", bytes).expect("the anchor is framed");
        let section = blob
            .framework_state("counter")
            .expect("the section decodes")
            .unwrap_or_else(|| {
                panic!(
                    "the anchor at S={step} carries NO framework section — a v1 header here \
                     means the carrier never wrote the scheduler's state, so a restore resumes \
                     a Period node with a next-fire instant it invented"
                )
            });
        assert_eq!(
            section,
            counter_framework_oracle(*step),
            "the section at S={step} must be the scheduler's state AS OF that boundary"
        );
        // …and the payload is still the node's own state, unchanged by the
        // section riding in front of it. Without this the arm could pass on a
        // carrier that wrote the framing OVER the user's bytes.
        assert_eq!(
            blob.payload,
            {
                let node = Counter {
                    count: *step + 1,
                    ..Default::default()
                };
                let mut sink = VecSink::new();
                cerulion_core::state::CerulionState::cer_capture(&node, &mut sink)
                    .expect("counter fits");
                sink.into_inner()
            },
            "the user payload must survive the framework section verbatim"
        );
    }
}

/// The PRODUCTION carrier captures a served per-message-FIFO input's
/// SERVICE CURSOR — the positive half whose `None` twin every Period/Sync
/// oracle in this file pins.
///
/// Driven over real iceoryx2 with ONE frame published per step, so the oracle
/// holds under ANY fires-per-step drain model: each step has exactly one frame
/// available, and the anchor at that step's boundary must state that frame's
/// wire `sequence` — `Served(0)` first, which is also the sentinel's
/// off-by-one kill (a carrier storing `seq` instead of `seq + 1` reads raw 0
/// = "nothing served" and this arm fails at its first served anchor).
///
/// SCOPE: the cursor states which frames were SERVED — it does NOT
/// close the pending>0 gap. A fire DECIDED at the anchor but not yet run is
/// still invisible here (`anchor_pending_fires` reports it on the replay
/// side); a burst captured mid-drain states only what the tick had read.
#[test]
fn a_served_fifo_input_writes_its_service_cursor_into_the_anchor() {
    const EXT: &str = "/svc/ext";
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("tally".to_string(), Box::new(TallyEntry::new()));
    let ids = vec!["tally".to_string()];
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "service_cursor".to_string(),
        prefix: unique("svc"),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "tally".to_string(),
            node_type: "tally".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: EXT.to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut h = harness_with_config("svc", config, factories, &ids, RING_RECORDS);
    h.arm.arm(1, 1); // every step, from the first
    h.runtime.attach_state_arm(Arc::clone(&h.arm));

    let mgr = h.runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher(EXT, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");

    // The anchor labelled S carries the state after executed step S + 1 (the
    // exact mapping the Counter arm's `count = step + 1` oracle pins), so the
    // drive is: two quiet steps, three publish+step rounds, one quiet flush —
    // anchors 1..=5 then read (quiet, Served(0), Served(1), Served(2),
    // Served(2)-again).
    //
    // Quiet phase first: with NOTHING served, an all-`None` table plus an
    // otherwise-empty section is EMPTY, so that anchor must ride the v1
    // header — the "no growth until a cursor exists" rule,
    // pinned here at the production boundary.
    step_n(&mut h.runtime, 2);
    // One frame per step, x = (k+1)*10 so the payload oracle has distinct
    // running totals. The k-th frame carries wire sequence k (fresh
    // publisher, commit-time numbering). One frame per step keeps
    // the oracle valid under ANY fires-per-step drain model.
    for k in 0u64..3 {
        let mut proxy = ext.loan_proxy::<Vector3>().expect("loan");
        proxy.x = ((k + 1) * 10) as f64;
        drop(proxy);
        step_n(&mut h.runtime, 1);
    }
    // The quiet flush step both publishes the last serve's anchor and pins
    // that a step which serves NOTHING does not advance the cursor.
    step_n(&mut h.runtime, 1);

    let events = drain(&h.ring_name);
    let got = completes(&events);
    assert_eq!(
        got.iter().map(|(s, _, _)| *s).collect::<Vec<_>>(),
        vec![1, 2, 3, 4, 5],
        "every armed step anchors"
    );
    // Per anchor label: the cursor the section must state (None = no section)
    // and the running total the payload must hold.
    let expected: [(Option<u32>, u64); 5] = [
        (None, 0),     // after executed step 2: nothing served yet
        (Some(0), 10), // after step 3: frame 0 read
        (Some(1), 30), // after step 4: frame 1 read
        (Some(2), 60), // after step 5: frame 2 read
        (Some(2), 60), // after quiet step 6: the cursor did NOT advance
    ];
    for (step, _, bytes) in &got {
        let (want_cursor, want_total) = expected[(*step - 1) as usize];
        let blob = AnchorBlob::decode("tally", bytes).expect("the anchor is framed");
        let section = blob.framework_state("tally").expect("readable");
        match want_cursor {
            None => assert_eq!(
                section, None,
                "before anything is served the section is EMPTY and the \
                 anchor stays on the v1 header — the no-growth cost statement"
            ),
            Some(seq) => {
                let section = section.unwrap_or_else(|| {
                    panic!("the anchor at S={step} must carry a section: a frame was served")
                });
                assert_eq!(
                    section.service_cursor("inp"),
                    InputServiceCursor::Served(seq),
                    "the anchor at S={step} states the last frame the tick READ"
                );
                assert_eq!(
                    section.input_service,
                    Some([("inp".to_string(), Some(seq))].into_iter().collect()),
                    "and the table names exactly the one FIFO input"
                );
                // Measured and pinned: the
                // whole tail for one input named "inp" is 16 bytes
                // on top of the 25-byte v1 prefix.
                assert_eq!(
                    section.encoded_len(),
                    41,
                    "25-byte v1 prefix + 16-byte table"
                );
            }
        }
        // The user payload survives the section in front of it.
        let expected_payload = {
            let node = Tally {
                total: want_total,
                ..Default::default()
            };
            let mut sink = VecSink::new();
            cerulion_core::state::CerulionState::cer_capture(&node, &mut sink).expect("fits");
            sink.into_inner()
        };
        assert_eq!(
            blob.payload, expected_payload,
            "the payload at S={step} is the node state as of that boundary"
        );
    }
}

/// A frame the tick NEVER SAW must not advance the service cursor —
/// the pop-vs-serve discriminator, driven at the production boundary.
///
/// A frame with a wrong `schema_hash` is POPPED off the queue like any other
/// (queue accounting is hash-blind) but `build_inbound_view` rejects it, so
/// the tick never reads it. The cursor is advanced ONLY after a successful
/// serve; a carrier that records at pop — or before the serve gate — states
/// the poisoned frame's sequence (99, chosen so no good frame can alias it)
/// and tells a resume to skip a frame nothing read, starving the first
/// resumed step. The oracle is interleave-robust (it does not pin WHICH step
/// pops the poison): no anchor may EVER state 99, and the final anchor must
/// state the last GOOD frame with the poisoned value absent from the total.
#[test]
fn a_frame_the_tick_never_saw_does_not_advance_the_cursor() {
    const EXT: &str = "/svp/ext";
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("tally".to_string(), Box::new(TallyEntry::new()));
    let ids = vec!["tally".to_string()];
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "reject".to_string(),
        prefix: unique("svp"),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "tally".to_string(),
            node_type: "tally".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: EXT.to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut h = harness_with_config("svp", config, factories, &ids, RING_RECORDS);
    h.arm.arm(1, 1);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));

    let mgr = h.runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher(EXT, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");

    // Good frame, wire sequence 0, x = 10.
    let mut proxy = ext.loan_proxy::<Vector3>().expect("loan");
    proxy.x = 10.0;
    drop(proxy);
    step_n(&mut h.runtime, 1);

    // The POISON: a well-FRAMED Vector3-sized frame whose schema_hash is
    // wrong, sequence 99. `publish_raw` writes the caller's header verbatim.
    let mut frame = vec![0u8; 56];
    let mut header =
        cerulion_core::wire::WireHeader::new(<Vector3 as ShmMessage>::SCHEMA_HASH ^ 1, 99, 0);
    header.total_size = 56;
    header.offset_table_offset = 56;
    header.write_to_buf(&mut frame[..cerulion_core::wire::WireHeader::SIZE]);
    frame[32..40].copy_from_slice(&777.0f64.to_le_bytes()); // x, must never be summed
    ext.publish_raw(&frame).expect("raw publish");
    step_n(&mut h.runtime, 1);

    // Good frame. Its wire sequence is whatever the publisher's counter says
    // (`publish_raw` may or may not burn one) — read the truth off the anchor
    // rather than assuming, requiring only that it is not 99.
    let mut proxy = ext.loan_proxy::<Vector3>().expect("loan");
    proxy.x = 20.0;
    drop(proxy);
    // Enough quiet steps to drain any interleave, plus the anchor flush.
    step_n(&mut h.runtime, 3);

    let events = drain(&h.ring_name);
    let got = completes(&events);
    assert!(!got.is_empty(), "the armed run anchors");
    let mut final_cursor = None;
    for (step, _, bytes) in &got {
        let blob = AnchorBlob::decode("tally", bytes).expect("framed");
        if let Some(section) = blob.framework_state("tally").expect("readable") {
            match section.service_cursor("inp") {
                InputServiceCursor::Served(99) => panic!(
                    "the anchor at S={step} states the POISONED frame's sequence — the \
                     cursor advanced at pop (or before the serve gate), not at serve"
                ),
                cursor => final_cursor = Some(cursor),
            }
        }
    }
    // Both good frames were eventually served; the poison was not.
    let last = got.last().expect("anchors");
    let blob = AnchorBlob::decode("tally", &last.2).expect("framed");
    let section = blob
        .framework_state("tally")
        .expect("readable")
        .expect("a served input carries a section");
    let served = match section.service_cursor("inp") {
        InputServiceCursor::Served(seq) => seq,
        other => panic!("the final anchor must state a served frame, got {other:?}"),
    };
    assert_ne!(served, 0, "the SECOND good frame was served by the end");
    assert_ne!(served, 99, "and it is not the poison");
    assert_eq!(final_cursor, Some(InputServiceCursor::Served(served)));
    // The payload proves the poison never reached the tick: 10 + 20, never 777.
    let expected_payload = {
        let node = Tally {
            total: 30,
            ..Default::default()
        };
        let mut sink = VecSink::new();
        cerulion_core::state::CerulionState::cer_capture(&node, &mut sink).expect("fits");
        sink.into_inner()
    };
    assert_eq!(
        blob.payload, expected_payload,
        "the tick summed exactly the two good frames"
    );
}

#[test]
fn a_per_set_sync_nodes_anchor_states_a_cursor_for_every_trigger_input() {
    // Per-set delivery's half of the FIFO-switch contract, and the one a resumed replay
    // rests on. The sibling arms above drive a ONE-input data-trigger node;
    // this drives the shape per-set Sync created — SEVERAL per-message FIFO
    // inputs on one node — so the table must name EVERY one of them, not the
    // first, and not none.
    //
    // Without it `resolve_pre_anchor_service` reads the node as drain-to-latest
    // and keeps the whole pre-anchor band; the band is then re-served to a
    // matcher that fires once per COMPLETE SET, so a mid-run replay of any
    // Sync graph diverges on frame 0. That is the failure this arm exists to
    // catch, at the production boundary rather than in the replay engine's
    // crafted fixtures.
    const EXT_A: &str = "/sy2/ext/a";
    const EXT_B: &str = "/sy2/ext/b";
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fusion".to_string(), Box::new(SyncTallyEntry::new()));
    let ids = vec!["fusion".to_string()];
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "trigger_cursor".to_string(),
        prefix: unique("sy2"),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "fusion".to_string(),
            node_type: "fusion".to_string(),
            inputs: vec![
                InputDef {
                    name: "a".to_string(),
                    source: EXT_A.to_string(),
                },
                InputDef {
                    name: "b".to_string(),
                    source: EXT_B.to_string(),
                },
            ],
            outputs: vec![],
        }],
    };
    let mut h = harness_with_config("sy2", config, factories, &ids, RING_RECORDS);
    h.arm.arm(1, 1); // every step, from the first
    h.runtime.attach_state_arm(Arc::clone(&h.arm));

    let mgr = h.runtime.test_transport().expect("test transport parked");
    let mut ext_a = mgr
        .create_publisher(EXT_A, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");
    let mut ext_b = mgr
        .create_publisher(EXT_B, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");

    // Two quiet steps (nothing served — the section is EMPTY and the anchor
    // stays on the v1 header), then three publish-a-SET-and-step rounds, then a
    // quiet flush. One set per step, so the k-th round serves wire sequence k on
    // BOTH inputs and the tick sums that set's two members.
    step_n(&mut h.runtime, 2);
    for k in 0u64..3 {
        let mut proxy = ext_a.loan_proxy::<Vector3>().expect("loan");
        proxy.x = ((k + 1) * 10) as f64;
        drop(proxy);
        let mut proxy = ext_b.loan_proxy::<Vector3>().expect("loan");
        proxy.x = ((k + 1) * 100) as f64;
        drop(proxy);
        step_n(&mut h.runtime, 1);
    }
    step_n(&mut h.runtime, 1);

    let events = drain(&h.ring_name);
    let got = completes(&events);
    assert_eq!(
        got.iter().map(|(s, _, _)| *s).collect::<Vec<_>>(),
        vec![1, 2, 3, 4, 5],
        "every armed step anchors"
    );
    // Per anchor label: the cursor BOTH inputs must state (None = no section)
    // and the running total the payload must hold.
    let expected: [(Option<u32>, u64); 5] = [
        (None, 0),      // after executed step 2: nothing served yet
        (Some(0), 110), // after step 3: set 0 read (10 + 100)
        (Some(1), 330), // after step 4: set 1 read (20 + 200)
        (Some(2), 660), // after step 5: set 2 read (30 + 300)
        (Some(2), 660), // after quiet step 6: neither cursor advanced
    ];
    for (step, _, bytes) in &got {
        let (want_cursor, want_total) = expected[(*step - 1) as usize];
        let blob = AnchorBlob::decode("fusion", bytes).expect("the anchor is framed");
        let section = blob.framework_state("fusion").expect("readable");
        match want_cursor {
            None => assert_eq!(
                section, None,
                "before any set is served the section is EMPTY and the anchor \
                 stays on the v1 header"
            ),
            Some(seq) => {
                let section = section.unwrap_or_else(|| {
                    panic!("the anchor at S={step} must carry a section: a set was served")
                });
                // EVERY trigger input, not just the first: the table is what a
                // resume caps each topic's band on, and one named input would
                // leave the other's whole band re-injected.
                for input in ["a", "b"] {
                    assert_eq!(
                        section.service_cursor(input),
                        InputServiceCursor::Served(seq),
                        "the anchor at S={step} states the last frame '{input}''s tick READ"
                    );
                }
                assert_eq!(
                    section.input_service,
                    Some(
                        [("a".to_string(), Some(seq)), ("b".to_string(), Some(seq))]
                            .into_iter()
                            .collect()
                    ),
                    "and the table names EXACTLY the node's two FIFO trigger inputs"
                );
            }
        }
        let expected_payload = {
            let node = SyncTally {
                total: want_total,
                ..Default::default()
            };
            let mut sink = VecSink::new();
            cerulion_core::state::CerulionState::cer_capture(&node, &mut sink).expect("fits");
            sink.into_inner()
        };
        assert_eq!(
            blob.payload, expected_payload,
            "the anchor at S={step} carries the state the node really held"
        );
    }
}

/// The two trigger cursors must be able to DISAGREE, and here they do.
///
/// The arm above drives one frame per input per round, so both cursors always
/// equal the round number — which means it passes just as happily against an
/// implementation that mints ONE cursor and hands the same `Arc` to every
/// trigger input of the node. That is not a hypothetical: the cursors are minted
/// in a loop at one wiring site, a single `Arc::clone` hoisted out of it would
/// compile, and a resume would then cap every topic's pre-anchor band at
/// whichever input happened to write last.
///
/// So this drives the shape where the values genuinely part company. `a` gets
/// TWO frames and `b` one, published last. With `b` scarce the descent gate
/// passes; the walk stages `a`'s second frame, finds that advancing onto it
/// tightens the span toward `b`, and takes it. The tick therefore READS `a`'s
/// frame 1 and `b`'s frame 0 — and the cursor records only what a tick read, so
/// the skipped frame 0 leaves no mark on `a`'s cursor and the two land one
/// apart.
///
/// The assertion that carries the weight is the INEQUALITY: `a`'s cursor is 1,
/// `b`'s is 0, and they are not the same number. Everything else here is
/// scaffolding to reach a boundary where that is true.
#[test]
fn a_skipped_member_leaves_the_two_trigger_cursors_holding_different_frames() {
    const EXT_A: &str = "/sy3/ext/a";
    const EXT_B: &str = "/sy3/ext/b";
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fusion".to_string(), Box::new(SyncTallyEntry::new()));
    let ids = vec!["fusion".to_string()];
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "trigger_cursor_split".to_string(),
        prefix: unique("sy3"),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "fusion".to_string(),
            node_type: "fusion".to_string(),
            inputs: vec![
                InputDef {
                    name: "a".to_string(),
                    source: EXT_A.to_string(),
                },
                InputDef {
                    name: "b".to_string(),
                    source: EXT_B.to_string(),
                },
            ],
            outputs: vec![],
        }],
    };
    let mut h = harness_with_config("sy3", config, factories, &ids, RING_RECORDS);
    h.arm.arm(1, 1);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));

    let mgr = h.runtime.test_transport().expect("test transport parked");
    let mut ext_a = mgr
        .create_publisher(EXT_A, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");
    let mut ext_b = mgr
        .create_publisher(EXT_B, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");

    // TWO frames on `a`, then ONE on `b` — with a STEP between the two `a`
    // publishes, so they carry different wire stamps. That separation is
    // load-bearing: a wire stamp is the transport clock at loan time, so two
    // back-to-back loans on a virtual clock that has not moved share a stamp,
    // and advancing from a stamp to the same stamp is not a strict improvement
    // — the descent correctly REFUSES (the earliest-min tie-break, by design), the
    // head never moves, and the two cursors agree after all.
    {
        let mut proxy = ext_a.loan_proxy::<Vector3>().expect("loan");
        proxy.x = 10.0;
    }
    step_n(&mut h.runtime, 1);
    {
        let mut proxy = ext_a.loan_proxy::<Vector3>().expect("loan");
        proxy.x = 20.0;
    }
    {
        let mut proxy = ext_b.loan_proxy::<Vector3>().expect("loan");
        proxy.x = 100.0;
    }
    step_n(&mut h.runtime, 2);

    // A THIRD `a` frame with NO partner. It is POPPED into the head at the next
    // boundary and never SERVED, because nothing completes a set for it — and
    // that gap is the only place a cursor that advanced at POP can be told apart
    // from one that advances at SERVE. Without it the arm's final `Served(1)` is
    // satisfied by both: a pop-advancing implementation records a@0 when popped
    // and overwrites it with a@1 when a@1 is popped, landing on the same number
    // with the same `total`.
    {
        let mut proxy = ext_a.loan_proxy::<Vector3>().expect("loan");
        proxy.x = 30.0;
    }
    step_n(&mut h.runtime, 2);

    // The node's own state proves WHICH members the tick read: `total` is the
    // sum, so 120 means (20, 100) — the descent really did advance past a@10.
    // Without this the cursor numbers below could be describing a fire that
    // never happened.
    let events = drain(&h.ring_name);
    let got = completes(&events);
    let (_, _, bytes) = got.last().expect("at least one anchor");
    let blob = AnchorBlob::decode("fusion", bytes).expect("the anchor is framed");
    let expected_payload = {
        let node = SyncTally {
            total: 120,
            ..Default::default()
        };
        let mut sink = VecSink::new();
        cerulion_core::state::CerulionState::cer_capture(&node, &mut sink).expect("fits");
        sink.into_inner()
    };
    assert_eq!(
        blob.payload, expected_payload,
        "the fire read a@20 and b@100 — the descent passed over a@10, which is \
         the whole reason the two cursors can differ"
    );

    let section = blob
        .framework_state("fusion")
        .expect("readable")
        .expect("a set was served, so the anchor carries a section");
    assert_eq!(
        section.service_cursor("a"),
        InputServiceCursor::Served(1),
        "`a`'s tick read its SECOND frame: the skipped frame 0 was never served, \
         and the cursor records reads, not pops"
    );
    assert_eq!(
        section.service_cursor("b"),
        InputServiceCursor::Served(0),
        "`b` had only one frame and the tick read it"
    );
    // THE POP-VS-SERVE DISCRIMINATOR: a@30 was popped into the head two steps
    // ago and never served, so a cursor that records READS is still at 1 while
    // one that records POPS has moved to 2. `total` is unchanged at 120, which
    // is what proves the frame really was unserved rather than quietly fused.
    assert_eq!(
        section.service_cursor("a"),
        InputServiceCursor::Served(1),
        "an UNSERVED popped frame must not advance the cursor — the section \
         states what a resumed run has already READ, and a resume that skipped \
         a@30 on the strength of this number would drop a frame the tick never \
         saw"
    );
    assert_ne!(
        section.service_cursor("a"),
        section.service_cursor("b"),
        "THE PIN: the two trigger inputs hold DIFFERENT cursors. One `Arc` \
         shared across a node's trigger inputs — a single hoisted clone at the \
         wiring site — would report the same number twice here and cap both \
         topics' resumed bands on whichever input wrote last"
    );
}

/// A node the scheduler has nothing to say about writes the v1 (section-less) bytes.
///
/// `NodeFrameworkState::is_empty` exists so a graph that carries no framework
/// state pays no framing for it, and that rule is only real if the carrier
/// consults it. Driven through the SAME production boundary as the arm above:
/// the `Guarded` fixture is deferred to the fork carrier, so this uses the
/// `Counter` with its schedule cleared… which no seam permits. Instead the claim
/// is pinned where it is decidable — on the decoder's own contract, against a
/// blob built by the production v1 helper — so the two headers stay
/// distinguishable and a v1 anchor keeps decoding to "no section".
#[test]
fn a_v1_anchor_still_decodes_to_no_section_so_older_bags_stay_readable() {
    let node = Counter {
        count: 7,
        ..Default::default()
    };
    let mut sink = VecSink::new();
    cerulion_core::state_restore::capture_anchor_blob(&node, &mut sink).expect("fits");
    let bytes = sink.into_inner();

    let blob = AnchorBlob::decode("counter", &bytes).expect("a v1 blob is framed");
    assert_eq!(
        blob.framework_state("counter").expect("decodes"),
        None,
        "a v1 anchor states NOTHING about the scheduler — it must not decode to an \
         all-zero section, which a restore would apply as a real next-fire of 0"
    );
    // The empty section and the absent one are the same claim, which is what
    // lets the carrier route both to the v1 header.
    assert!(NodeFrameworkState::default().is_empty());
}

// ===========================================================================
// The refusal path may never WAIT on the node thread
// ===========================================================================

/// The boundary's own REFUSAL records go through the non-blocking push.
///
/// A state ring is always `BACKPRESSURE`, so `push_skip` waits up to five seconds
/// per record and then laps. The sharpest caller is the one immediately below the
/// ring-room precheck: it has just MEASURED that the ring cannot hold this anchor,
/// and then publishing one blocking refusal record per node would spend exactly that
/// measured absence, N times over, on the robot's node thread. A precheck whose
/// refusal blocks is worse than no precheck.
///
/// The ring here is far too small to admit any anchor (the precheck's headroom term
/// alone exceeds its capacity), so EVERY cadence takes the `RecorderBehind` branch
/// and pushes one refusal record. Nothing drains it, so it fills and the later
/// refusals cannot be published at all.
///
/// The oracle is the COUNTER, not a wall — a wall tight enough to separate
/// "declined" from "waited" is also tight enough for a loaded runner to invert.
/// Under a blocking `push_skip` every record eventually lands by lapping,
/// so `anchor_skips_unpublished()` would read 0; only a path that DECLINES can make
/// it non-zero.
#[test]
fn a_full_state_ring_makes_the_boundary_decline_its_refusal_records_not_wait() {
    /// Small enough that the ring-room precheck can never admit an anchor (its
    /// headroom term alone is 64 records), and small enough that the per-cadence
    /// refusal records fill it well inside the script below.
    const TINY_RING: u32 = 16;
    const STEPS: usize = 24;

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("counter".to_string(), Box::new(CounterEntry::new()));
    let ids = vec!["counter".to_string()];
    let mut h = harness("skipfull", factories, &ids, TINY_RING);

    // Every step is a cadence, so every step refuses.
    h.arm.arm(1, 0);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));

    step_n(&mut h.runtime, STEPS);

    // THE ARM. More cadences than the ring holds records, and no drain, so the
    // tail of them had nowhere to go and the boundary declined rather than waited.
    let unpublished = h.runtime.anchor_skips_unpublished();
    assert!(
        unpublished > 0,
        "with {STEPS} refused cadences on a {TINY_RING}-record ring that nothing drains, \
         the boundary must have DECLINED at least one refusal record; a blocking push \
         would have landed every one of them by lapping and left this at 0"
    );
    assert!(
        unpublished <= STEPS as u64,
        "one refusal record per node per cadence bounds it: {unpublished} > {STEPS}"
    );

    // The FACT survives even though the CAUSE did not: the records that DID fit are
    // real `RecorderBehind` skips, so a reader is never told the anchor succeeded.
    let events = drain(&h.ring_name);
    let got = skips(&events);
    assert!(
        !got.is_empty() && got.iter().all(|(_, _, c)| *c == SkipCause::RecorderBehind),
        "every published record must be a RecorderBehind refusal: {got:?}"
    );
    assert!(
        completes(&events).is_empty(),
        "no anchor can have completed — the ring could never hold one"
    );
}

/// ANTI-TAUTOLOGY for the arm above: on a ring with room, the SAME script publishes
/// every refusal record and the counter stays at zero.
///
/// Without this, a counter that only ever went up — or a boundary that declined
/// unconditionally — would pass the arm above while quietly throwing away causes on
/// a perfectly healthy robot.
#[test]
fn a_ring_with_room_publishes_every_refusal_record_and_counts_none_unpublished() {
    const STEPS: usize = 24;

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("counter".to_string(), Box::new(CounterEntry::new()));
    let ids = vec!["counter".to_string()];
    // Sized BETWEEN the two: still below the ring-room precheck's need (so the same
    // `RecorderBehind` branch runs and there are refusal records to publish at all),
    // and far above the number of them this script produces. The precheck needs the
    // arena's worth of parts plus one header per node plus 64 records of headroom,
    // which is comfortably more than this — if that ever stops being true this test
    // fails LOUDLY on the record count below rather than going quietly vacuous.
    const MID_RING: u32 = 128;
    let mut h = harness("skiproom", factories, &ids, MID_RING);

    h.arm.arm(1, 0);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));
    step_n(&mut h.runtime, STEPS);

    assert_eq!(
        h.runtime.anchor_skips_unpublished(),
        0,
        "a ring with room loses no causes at all"
    );
    let events = drain(&h.ring_name);
    let got = skips(&events);
    assert_eq!(
        got.len(),
        STEPS,
        "one refusal record per node per cadence, all of them published: {got:?}"
    );
}

/// A caught tick PANIC permanently stops anchoring, so no anchor can
/// ever observe a post-panic service cursor.
///
/// # Why this is pinned: it is the load-bearing premise of a REFUTATION
///
/// One could argue that `try_view` stores
/// the cursor only AFTER `build_inbound_view` returns, and since `f` IS user
/// code (`cerulion_macros::impl_macro::build_nested_try_view` makes the tick
/// body the leaf of the nested `try_view` closures), a tick that PANICS leaves
/// its frame marked unserved — so "a later anchor re-injects an already-served
/// frame."
///
/// The ordering premise is true. The HARM is not, because there is no later
/// anchor. The tick runs under the node's `Mutex`, so a panic POISONS it;
/// `take_anchor_inner`'s `arc.try_lock()` then fails, and its `Err(_)` arm
/// calls `push_anchor_skip_all` — which skips EVERY node and returns. Nothing
/// calls `clear_poison`, so that is permanent and graph-wide.
///
/// Hence every anchor that can exist was taken at a boundary where the tick
/// COMPLETED, where the two orderings are identical. The store site is
/// observationally equivalent, and the anchor harvest is the cursor's only
/// reader (`GraphRuntime::service_cursors` — "read ONCE per anchor boundary").
///
/// This test is the TRIPWIRE on that premise. If anchoring is ever made to
/// survive a poisoned lock, a post-panic anchor becomes reachable, the ordering
/// question goes live again, and this arm fails — sending the next reader back
/// to the refutation rather than letting it rot into a silent wrong answer.
#[test]
fn a_caught_tick_panic_stops_anchoring_so_no_anchor_observes_a_post_panic_cursor() {
    const EXT: &str = "/svq/ext";
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("tally".to_string(), Box::new(PanicTallyEntry::new()));
    let ids = vec!["tally".to_string()];
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "panic".to_string(),
        prefix: unique("svq"),
        nodes: vec![NodeDef {
            fuse: None,
            ros2: None,
            id: "tally".to_string(),
            node_type: "tally".to_string(),
            inputs: vec![InputDef {
                name: "inp".to_string(),
                source: EXT.to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut h = harness_with_config("svq", config, factories, &ids, RING_RECORDS);
    // Armed from step 0, so anchors exist on BOTH sides of the panic and the
    // arm can tell "stopped" from "never started".
    h.arm.arm(1, 0);
    h.runtime.attach_state_arm(Arc::clone(&h.arm));

    let mgr = h.runtime.test_transport().expect("test transport parked");
    let mut ext = mgr
        .create_publisher(EXT, MaxSliceLen::const_new(64), 0)
        .expect("external publisher attaches");

    // A good frame and two quiet steps: anchors 0 and 1 complete normally.
    let mut proxy = ext.loan_proxy::<Vector3>().expect("loan");
    proxy.x = 10.0;
    drop(proxy);
    step_n(&mut h.runtime, 2);

    // The PANIC frame — well-framed and schema-VALID, so it passes every check
    // `build_inbound_view` makes and the only thing that can stop the cursor
    // store is the tick itself.
    let mut proxy = ext.loan_proxy::<Vector3>().expect("loan");
    proxy.x = 777.0;
    drop(proxy);
    step_n(&mut h.runtime, 1); // the panic lands here (step 2), and is CAUGHT

    // Keep stepping: a healthy carrier would anchor at every one of these.
    let mut proxy = ext.loan_proxy::<Vector3>().expect("loan");
    proxy.x = 20.0;
    drop(proxy);
    step_n(&mut h.runtime, 3);

    let events = drain(&h.ring_name);
    let got = completes(&events);
    let skipped = skips(&events);

    // The run really did anchor BEFORE the panic — without this the "stopped"
    // claim below is satisfied by a carrier that never worked at all.
    assert!(
        !got.is_empty(),
        "the armed run must anchor before the panic, or this arm is vacuous"
    );
    let last_complete_step = got.iter().map(|(s, _, _)| *s).max().expect("completes");
    assert!(
        last_complete_step < 2,
        "every completed anchor must predate the panic step (2); got {last_complete_step}"
    );

    // And it stopped, for the stated reason, on every later boundary.
    assert!(
        skipped.len() >= 3,
        "the post-panic boundaries must all report, not go silent: {skipped:?}"
    );
    for (step, _, cause) in &skipped {
        assert!(*step >= 2, "a skip before the panic step: {step}");
        assert!(
            matches!(cause, SkipCause::Contended),
            "the poisoned node lock reports Contended, got {cause:?} at step {step}"
        );
    }

    // THE consequence: the newest cursor any resume can read is the PRE-panic
    // one, taken where the tick completed — so the store-site ordering the
    // argument above names cannot be observed by any anchor.
    let last = got
        .iter()
        .max_by_key(|(s, _, _)| *s)
        .expect("a completed anchor");
    let blob = AnchorBlob::decode("tally", &last.2).expect("framed");
    let section = blob
        .framework_state("tally")
        .expect("readable")
        .expect("a served input carries a section");
    assert_eq!(
        section.service_cursor("inp"),
        InputServiceCursor::Served(0),
        "the newest readable cursor is the good frame's, from before the panic"
    );
}
