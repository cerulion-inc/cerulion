// SPDX-License-Identifier: AGPL-3.0-only
//! `#[input(backpressure = sample(N) | block)]` on a PER-SET Sync
//! TRIGGER input — real support, defined and pinned, over real iceoryx2.
//!
//! # Why this file exists
//!
//! Without it, NOTHING drives `block` on a Sync trigger input and exactly
//! one arm drives `sample(N)` (the `expect_within` decision in
//! `sync_per_set_iox2_test.rs`). Every other behaviour these policies have under
//! per-set delivery would be an ACCIDENT of shared code — real, reachable, and
//! contractually unstated. Such a node is NOT degraded
//! to legacy latest-per-set, so the semantics have to be written
//! down; these arms are where they are written down.
//!
//! # The two policies, in one sentence each
//!
//! * **`sample(N)` decimates BEFORE matching.** The gate lives inside the one
//!   pop primitive every per-set path uses, so a decimated frame can never
//!   become a head and never join a set. The gate window `N` and the alignment
//!   window `W` compose as a PIPELINE, never as a joint predicate: the gate
//!   admits a subsequence of each input's arrivals spaced `>= N` ms apart by
//!   wire stamp, and only admitted frames are eligible to align within `W`.
//!
//! * **`block` stays lossless end to end.** No eviction ever happens on a block
//!   input; the producer is deferred instead. Under per-set that needs its own
//!   occupancy rule (see `S`-vs-`B` below): a frame the matcher is HOLDING is
//!   still unserved, so it still counts against the producer's declared `depth`.
//!
//! # The oracles
//!
//! Sample arms publish by hand on a driven `VirtualClock`, so the payload IS the
//! wire stamp and a recorded set names exactly the frames it observed. Block
//! arms cannot: the all-block rule requires an IN-GRAPH producer, so their
//! stimulus is a `period_ms` node and their oracles are counts, publish totals
//! and FIFO contiguity — each derived by hand from the contract, never from a
//! previous run.
//!
//! `#[serial]` — real iceoryx2 over the process-global SHM singleton; per-test
//! SHM root via `build_for_test`.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::scheduler::BackpressureCounters;
use cerulion_core::testing::{
    count_at_exclusively, debug_lines_expected, line_level, lines_at_exclusively,
};
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing_test::traced_test;

const MS: u64 = 1_000_000;

/// Every `(a, b)` pair a fire observed, in fire order — the SET SEQUENCE.
type Sets = Arc<Mutex<Vec<(f64, f64)>>>;

fn recorded(sets: &Sets) -> Vec<(f64, f64)> {
    sets.lock().expect("sets sink poisoned").clone()
}

// ===========================================================================
// PART 1 — the `block` OCCUPANCY MODEL, at the subscriber
// ===========================================================================

/// A frame the matcher is HOLDING is still the producer's occupancy.
///
/// The `outstanding` mirror is the producer's whole view of its consumer: it
/// increments at `send` and the drain decrements it. On every read path where
/// the pop and the node's read of that frame happen inside one `try_view`, the
/// mirror IS occupancy. Per-set Sync pulls the two apart — a pop parks a frame
/// in the frozen head, and a descent parks a second behind it, where they can
/// sit unserved for many steps (a node whose partner has gone quiet holds its
/// head indefinitely). This walks ONE frame through every transition those two
/// slots have and requires occupancy to be CONSERVED at each: a frame that moved
/// from the queue into a slot leaves the mirror unchanged, and only a frame that
/// really left — served, or skipped — may lower it.
///
/// The oracle is hand-written per transition, BOTH numbers every time. A
/// debt-only assertion would pass an implementation that tracked the slots
/// perfectly and never touched the mirror, which is the entire bug.
#[test]
#[serial]
fn a_frame_held_in_a_matcher_slot_stays_the_producers_occupancy() {
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "slot_debt".into(),
            clock: Arc::new(cerulion_core::clock::RealClock),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("per-test SHM transport");

    let topic = "slot_debt";
    let mut publisher = mgr
        .create_publisher(topic, MaxSliceLen::try_new(1024).expect("len"), 0)
        .expect("publisher");
    let mut sub = mgr.create_subscriber(topic).expect("subscriber");

    // The producer's side of the contract, modelled by hand: `outstanding` is
    // the shared atomic a real publisher increments once per `send`.
    // The mirror is a `CreditWord` — LOCAL here (a same-process
    // edge modelled by hand); `record_published` is the producer's `fetch_add`
    // and `outstanding()` its `Acquire` load.
    let outstanding = cerulion_core::credit::CreditWord::local(4);
    sub.register_block_probe_for_test(
        outstanding.clone(),
        4,
        Arc::new(BackpressureCounters::new()),
        Arc::from("inp"),
        8,
    );
    // Per-message FIFO — the mode a per-set Sync trigger input is wired in.
    sub.mark_fifo_consume_for_test();

    for x in 1..=3u32 {
        let mut p = publisher.loan_proxy::<Vector3>().expect("loan");
        p.x = f64::from(x);
        drop(p);
        outstanding.record_published();
    }
    let mirror = || outstanding.outstanding();
    assert_eq!(
        (mirror(), sub.block_slot_debt_for_test()),
        (3, 0),
        "three frames queued, none held: the mirror is pure queue occupancy"
    );

    // BOUNDARY FILL — the frame MOVES from the queue into the head.
    let (popped, _) = sub.snapshot_latest_for_trigger();
    assert_eq!(popped, 1, "the boundary drain pops exactly one frame");
    assert_eq!(
        (mirror(), sub.block_slot_debt_for_test()),
        (3, 1),
        "a frame the matcher HOLDS is still unserved, so occupancy is unchanged \
         at 3 — under a pure pop-time decrement it reads 2 and the producer buys \
         itself a frame of headroom the declared depth never gave it"
    );

    // DESCENT STAGE — a second frame leaves the queue for the staged slot.
    let staged = sub
        .sync_peek_next_stamp_for_test()
        .expect("the peek pops without error");
    assert!(staged.is_some(), "a queued frame is there to stage");
    assert_eq!(
        (mirror(), sub.block_slot_debt_for_test()),
        (3, 2),
        "both slots occupied is the DEEPEST the matcher ever holds; occupancy is \
         still 3"
    );

    // ADVANCE — the head is SKIPPED (it really leaves) and the staged frame is
    // PROMOTED (slot to slot, which is not an exit).
    let promoted = sub
        .sync_discard_head_for_test()
        .expect("the advance never errors")
        .expect("the staged frame becomes the new head");
    assert!(promoted > 0, "the promoted head carries a real wire stamp");
    assert_eq!(
        (mirror(), sub.block_slot_debt_for_test()),
        (2, 1),
        "exactly ONE frame left the world here — the skipped head. A promotion \
         that also debited would release the producer twice for one frame"
    );

    // SERVE — the tick reads the head. This is the other exit.
    let served = sub
        .try_view::<Vector3, f64>(|v| v.x)
        .expect("the frozen head serves")
        .expect("a frame is there");
    assert_eq!(
        served, 2.0,
        "FIFO: frame 1 was skipped, so the promoted head is frame 2"
    );
    assert_eq!(
        (mirror(), sub.block_slot_debt_for_test()),
        (1, 0),
        "the served frame is consumed and the slots are empty; one frame is \
         still queued"
    );

    // A SECOND fill, then the restore-boundary VOID overwrites a live head.
    let (popped, _) = sub.snapshot_latest_for_trigger();
    assert_eq!(popped, 1, "the last queued frame fills the head");
    assert_eq!(
        (mirror(), sub.block_slot_debt_for_test()),
        (1, 1),
        "held again — the queue is empty but the producer is owed nothing back"
    );
    sub.sync_void_head_for_test();
    assert_eq!(
        (mirror(), sub.block_slot_debt_for_test()),
        (0, 0),
        "voiding replaces a held frame with `no frame`, which releases it"
    );

    // ANTI-TAUTOLOGY: the mirror really can reach 0 and stay there, so the
    // numbers above are not an artefact of a counter that never moves.
    assert_eq!(
        sub.snapshot_latest_for_trigger().0,
        0,
        "the queue is drained"
    );
    assert_eq!((mirror(), sub.block_slot_debt_for_test()), (0, 0));
}

/// The debt is taken ONLY where the design says, and a frame that left
/// the world keeps its pop-time decrement.
///
/// Two halves the walk above cannot see, both of which a plausible "re-credit
/// every pop" implementation gets wrong. An input with NO block probe has no
/// mirror to restate and must never accumulate a debt; and a `block` frame that
/// never lands in a slot — here a drain-to-latest drop — must lower occupancy at
/// its pop, because it is gone and nothing will ever serve it.
#[test]
#[serial]
fn the_slot_debt_is_taken_only_by_block_probed_fifo_inputs() {
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "slot_debt_scope".into(),
            clock: Arc::new(cerulion_core::clock::RealClock),
            subscriber_buffer_size: 8,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("per-test SHM transport");

    // (a) NO probe: a held head takes no debt.
    let topic = "slot_debt_unprobed";
    let mut publisher = mgr
        .create_publisher(topic, MaxSliceLen::try_new(1024).expect("len"), 0)
        .expect("publisher");
    let mut sub = mgr.create_subscriber(topic).expect("subscriber");
    sub.mark_fifo_consume_for_test();
    let mut p = publisher.loan_proxy::<Vector3>().expect("loan");
    p.x = 1.0;
    drop(p);
    assert_eq!(sub.snapshot_latest_for_trigger().0, 1, "the head fills");
    assert_eq!(
        sub.block_slot_debt_for_test(),
        0,
        "an input with no block probe has no mirror to restate"
    );

    // (b) A `block` frame DROPPED by a drain-to-latest keeps its pop-time
    // decrement: two frames enter, one survives, and BOTH lower occupancy
    // because only the survivor was ever owed to the node — and it is read here.
    let topic = "slot_debt_latest";
    let mut publisher = mgr
        .create_publisher(topic, MaxSliceLen::try_new(1024).expect("len"), 0)
        .expect("publisher");
    let mut sub = mgr.create_subscriber(topic).expect("subscriber");
    // The mirror is a `CreditWord` — LOCAL here (a same-process
    // edge modelled by hand); `record_published` is the producer's `fetch_add`
    // and `outstanding()` its `Acquire` load.
    let outstanding = cerulion_core::credit::CreditWord::local(4);
    sub.register_block_probe_for_test(
        outstanding.clone(),
        4,
        Arc::new(BackpressureCounters::new()),
        Arc::from("inp"),
        8,
    );
    // NOT `mark_fifo_consume_for_test` — this is the latest-value discipline.
    for x in 1..=2u32 {
        let mut p = publisher.loan_proxy::<Vector3>().expect("loan");
        p.x = f64::from(x);
        drop(p);
        outstanding.record_published();
    }
    let served = sub
        .try_view::<Vector3, f64>(|v| v.x)
        .expect("the live drain serves")
        .expect("a frame is there");
    assert_eq!(served, 2.0, "drain-to-latest serves the newest frame");
    assert_eq!(
        (outstanding.outstanding(), sub.block_slot_debt_for_test()),
        (0, 0),
        "both frames left the queue and the node has read what it is going to \
         read: nothing is held, so the producer is owed the full release"
    );
}

// ===========================================================================
// PART 2 — `block` on a per-set Sync TRIGGER, end to end
// ===========================================================================
//
// The all-block rule requires an IN-GRAPH producer (topology validation refuses
// a `block` edge whose topic nobody in the graph publishes — the scheduler can
// only defer producers it schedules), so these arms cannot publish by hand the
// way the sample arms do. The stimulus is a `period_ms` node writing a counter,
// and the oracles are publish totals, FIFO contiguity of the values the node
// observed, and the counters — each derived from the design.

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

/// The slow partner, on its own `drop_oldest` topic — the "other input" of the
/// pair, deliberately a DIFFERENT policy so the per-input rule is exercised.
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

/// The Sync consumer under test: trigger `a` is `block`, trigger `b` is the
/// default `drop_oldest`. It RECORDS the pair it read.
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct BlockPairFuse {
    #[input(trigger, backpressure = block, depth = 4)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    sets: Option<Sets>,
    events: Option<Arc<AtomicU64>>,
}

#[cerulion_node_impl]
impl BlockPairFuse {
    fn tick(&mut self) -> Result<(), NodeError> {
        let pair = (self.a.x, self.b.x);
        if let Some(sink) = &self.sets {
            sink.lock().expect("sets sink poisoned").push(pair);
        }
        Ok(())
    }

    /// The CONSUMER side of `block`: "your queue — including the head the
    /// matcher is holding for you — was at the defer line when this drain ran".
    #[on_event(input = "a")]
    fn on_a_pressure(&mut self, event: BackpressureEvent) {
        assert!(
            matches!(event.policy, BackpressurePolicy::Block),
            "a block input must surface a Block event, got {:?}",
            event.policy
        );
        assert_eq!(
            event.dropped, 0,
            "block is LOSSLESS — a block event may never report a drop"
        );
        if let Some(n) = &self.events {
            n.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// A plain `drop_oldest` sibling consumer, used ONLY by the mixed-topic control.
#[cerulion_node(period_ms = 1)]
#[derive(Default)]
struct PlainSibling {
    #[input]
    inp: Vector3,
}

#[cerulion_node_impl]
impl PlainSibling {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
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

fn graph(name: &str, prefix: &str, nodes: Vec<NodeDef>) -> GraphConfig {
    GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: name.to_string(),
        prefix: prefix.to_string(),
        nodes,
    }
}

/// `feeder` (1 ms, block, depth 4) → `fuse.a`; `fuse.b` reads an ABSOLUTE topic
/// nobody in this graph publishes, so the partner can be left silent by hand.
fn starved_partner_graph(
    prefix: &str,
    partner_topic: &str,
    sets: Sets,
    events: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = graph(
        "pssbp_starved",
        prefix,
        vec![
            node("feeder", "block_feeder", vec![], vec!["out"]),
            node(
                "fuse",
                "block_pair_fuse",
                vec![("a", "feeder/out"), ("b", partner_topic)],
                vec![],
            ),
        ],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("feeder".to_string(), Box::new(BlockFeederEntry::new()));
    factories.insert(
        "fuse".to_string(),
        Box::new(BlockPairFuseEntry::with_state(BlockPairFuse {
            sets: Some(sets),
            events: Some(events),
            ..Default::default()
        })),
    );
    (config, factories)
}

/// B1 — the producer runs exactly `depth` frames ahead, HELD MEMBERS INCLUDED.
///
/// `feeder` publishes 1 ms; `fuse.a` is `block, depth = 4`; `fuse.b` never
/// receives anything, so the node can never fire and never consumes a member.
/// The head fills on the first boundary and then sits there forever.
///
/// The oracle is the PUBLISH COUNT, and it is the whole occupancy model in one
/// number. `block`'s declared contract is "at most `depth` UNSERVED frames per
/// edge". Three of those frames are in the iceoryx2 queue and the fourth is the
/// head the matcher is holding — unserved by every reading of the word, since no
/// tick has run. So the producer must stop at 4.
///
/// Under a pure pop-time decrement the boundary
/// pop looks to the mirror like a delivery, one slot frees, and the producer
/// publishes a FIFTH frame — `depth + 1` unserved frames against a declared
/// depth of 4. That off-by-one is per-slot, so a node whose descent has also
/// staged a frame runs at `depth + 2`.
#[test]
#[serial]
fn a_block_producer_stops_at_depth_counting_the_member_the_matcher_holds() {
    const STEPS: usize = 20;
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let events = Arc::new(AtomicU64::new(0));
    let (config, factories) = starved_partner_graph(
        "pssb1",
        "/pssb1/silent",
        Arc::clone(&sets),
        Arc::clone(&events),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build block sync graph");
    for _ in 0..STEPS {
        runtime.step(Duration::from_millis(1));
    }

    let feeder = runtime.node_handle("feeder").expect("feeder handle");
    let fuse = runtime.node_handle("fuse").expect("fuse handle");
    assert_eq!(
        feeder.fire_count(),
        4,
        "the producer must stop at the declared depth of 4 — 3 frames queued \
         plus the ONE the matcher is holding as `a`'s head. A fifth publish means \
         the held member stopped counting as occupancy the moment it was popped"
    );
    assert_eq!(
        recorded(&sets),
        Vec::new(),
        "the partner never arrived, so no set can be complete and no fire may \
         happen — a fire here would be reading a fabricated member"
    );
    assert_eq!(
        fuse.backpressure_drop_oldest_count("a"),
        0,
        "block never evicts: that is the whole policy"
    );
    assert!(
        fuse.backpressure_block_fires_deferred_count("a") > 0,
        "the producer really was deferred, so the publish ceiling above is flow \
         control and not simply a producer that ran out of things to say"
    );
}

/// B4 — the starved-partner HOLD, and the un-wedge.
///
/// This is the real cost of `block` on a Sync trigger, stated as an arm: a
/// `block` input whose PARTNER stops arriving holds its producer at `depth`
/// unserved frames INDEFINITELY. That is not a bug to be fixed — `block`
/// promises the producer never outruns the consumer, and a Sync consumer that
/// cannot complete a set is not consuming.
///
/// The second half is what makes it a hold rather than a wedge: when the partner
/// finally arrives, the set fires on the frame the producer published FIRST (the
/// FIFO head it has been holding all along, value 1 — not the freshest), and the
/// producer resumes.
#[test]
#[serial]
fn a_starved_partner_holds_the_block_producer_and_its_arrival_releases_it() {
    const PARTNER: &str = "/pssb4/partner";
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let events = Arc::new(AtomicU64::new(0));
    let (config, factories) =
        starved_partner_graph("pssb4", PARTNER, Arc::clone(&sets), Arc::clone(&events));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 8)
        .expect("build block sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut partner = mgr
        .create_publisher(PARTNER, MaxSliceLen::const_new(64), 0)
        .expect("partner publisher");

    for _ in 0..15 {
        runtime.step(Duration::from_millis(1));
    }
    let held = runtime
        .node_handle("feeder")
        .expect("feeder handle")
        .fire_count();
    assert_eq!(held, 4, "the producer is held at its declared depth");
    assert!(recorded(&sets).is_empty(), "nothing can fire while starved");

    // The partner arrives. Its stamp lands inside the 50 ms window of every
    // frame the producer published (all four are within the first 4 ms).
    {
        let mut p = partner.loan_proxy::<Vector3>().expect("loan");
        p.x = 99.0;
    }
    for _ in 0..10 {
        runtime.step(Duration::from_millis(1));
    }

    let observed = recorded(&sets);
    assert_eq!(
        observed,
        vec![(4.0, 99.0)],
        "ONE set fires, and its member is the frame NEAREST the partner. All \
         four held frames were still there to choose from — the partner arrived \
         ~15 ms after them, so with `b` scarce the descent gate passes and the \
         walk advances `a` from frame 1 to frame 4, tightening the span at every \
         step. That is the feature working, not loss: the next assertion is the \
         one that says nothing vanished. Got {observed:?}"
    );
    let fuse_handle = runtime.node_handle("fuse").expect("fuse handle");
    let skipped = fuse_handle.sync_closer_skip_count("/pssb4/feeder/out");
    assert_eq!(
        (observed.len() as u64) + skipped,
        held,
        "LOSSLESS: every one of the {held} frames the producer published is \
         accounted for — 1 served to the set, {skipped} passed over by the \
         descent, none evicted. Under `drop_oldest` the intervening frames would \
         simply be gone and this sum would fall short"
    );
    // EXACTLY four more, and the count is the ledger read from the other end.
    // The set consumed all four held frames — one served, three passed over —
    // so four slots come free and the producer publishes into every one of them
    // before its partner goes quiet again and it stalls at the depth line.
    //
    // What this pins is the release LEDGER, not the individual exit sites —
    // measured, not assumed. Deleting the serve exit, the discard exits, or both
    // leaves this arm at 8, because the boundary fill's own re-derivation runs
    // before the producer next reads the mirror and repairs them. That is a
    // property of RE-DERIVING the debt rather than accumulating it, and the
    // subscriber walk (which observes between transitions) is where those sites
    // are pinned. What this arm does catch is the ENTRY site: on a starved input
    // no other site runs, so nothing repairs it.
    for _ in 0..15 {
        runtime.step(Duration::from_millis(1));
    }
    let resumed = runtime
        .node_handle("feeder")
        .expect("feeder handle")
        .fire_count();
    assert_eq!(
        resumed, 8,
        "consuming a set releases exactly the slots it consumed — four here, so \
         the producer resumes from {held} to 8 and then stalls again at the \
         depth line with its partner quiet once more. The hold is a hold, not a \
         wedge, and it releases exactly what was served"
    );
    assert_eq!(
        runtime
            .node_handle("fuse")
            .expect("fuse handle")
            .backpressure_drop_oldest_count("a"),
        0,
        "still no eviction anywhere on the block input"
    );
}

/// `fast` (1 ms, block, depth 4) → `fuse.a`; `slow` (5 ms) → `fuse.b`. Both
/// producers are IN-GRAPH, so every wire stamp comes off the graph clock and the
/// run is deterministic without any hand publishing.
fn block_pair_graph(
    prefix: &str,
    sets: Sets,
    events: Arc<AtomicU64>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = graph(
        "pssbp_pair",
        prefix,
        vec![
            node("fast", "block_feeder", vec![], vec!["out"]),
            node("slow", "slow_feeder", vec![], vec!["out"]),
            node(
                "fuse",
                "block_pair_fuse",
                vec![("a", "fast/out"), ("b", "slow/out")],
                vec![],
            ),
        ],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fast".to_string(), Box::new(BlockFeederEntry::new()));
    factories.insert("slow".to_string(), Box::new(SlowFeederEntry::new()));
    factories.insert(
        "fuse".to_string(),
        Box::new(BlockPairFuseEntry::with_state(BlockPairFuse {
            sets: Some(sets),
            events: Some(events),
            ..Default::default()
        })),
    );
    (config, factories)
}

/// What one `block_pair_graph` run of `steps` 1 ms steps observed.
struct PairRun {
    sets: Vec<(f64, f64)>,
    skips: u64,
    unmatched: u64,
    evictions: u64,
    deferred: u64,
    events: u64,
    published: u64,
}

fn run_block_pair(prefix: &str, steps: usize) -> PairRun {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let events = Arc::new(AtomicU64::new(0));
    let (config, factories) = block_pair_graph(prefix, Arc::clone(&sets), Arc::clone(&events));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build block pair graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(1));
    }
    let fuse = runtime.node_handle("fuse").expect("fuse handle");
    // The resolved topic carries a LEADING SLASH (`resolve_source`), and the
    // sync counters are keyed by resolved topic while the backpressure ones are
    // keyed by INPUT NAME — get either wrong and the counter silently reads 0.
    let topic_a = format!("/{prefix}/fast/out");
    PairRun {
        sets: recorded(&sets),
        skips: fuse.sync_closer_skip_count(&topic_a),
        unmatched: fuse.sync_unmatched_discard_count(&topic_a),
        evictions: fuse.backpressure_drop_oldest_count("a"),
        deferred: fuse.backpressure_block_fires_deferred_count("a"),
        events: events.load(Ordering::Relaxed),
        published: runtime
            .node_handle("fast")
            .expect("fast handle")
            .fire_count(),
    }
}

/// B2 — `block` + per-set is LOSSLESS end to end.
///
/// A 1 ms producer into a Sync node that can only fire when its 5 ms partner
/// also has a frame: the fast queue rides at its declared depth and the producer
/// is throttled to the rate the consumer can actually take.
///
/// The oracle is CONTIGUITY, which is what "lossless" means here and is the one
/// thing eviction cannot fake. `a`'s frames carry their publish ordinal, they
/// leave the queue in FIFO order, and each one is either SERVED to a set or
/// PASSED OVER by the descent — so the highest value any tick observed can never
/// exceed the number of frames accounted for (`served + skipped`). Under
/// eviction that inequality breaks immediately: iceoryx2 reclaims the oldest
/// frames silently, so the values observed JUMP while both counters stay put.
#[test]
#[serial]
fn block_on_a_per_set_sync_trigger_loses_nothing() {
    let run = run_block_pair("pssb2", 60);

    assert!(
        run.sets.len() >= 3,
        "the partner ticks every 5 ms over 60 steps, so several sets must fire — \
         got {:?}",
        run.sets
    );
    let a_values: Vec<f64> = run.sets.iter().map(|(a, _)| *a).collect();
    assert!(
        a_values.windows(2).all(|w| w[0] < w[1]),
        "each frame is consumed by at most one set, in arrival order, so the \
         members are strictly increasing — got {a_values:?}"
    );
    let accounted = run.sets.len() as u64 + run.skips;
    let highest = a_values.last().copied().unwrap_or(0.0) as u64;
    assert!(
        highest <= accounted,
        "LOSSLESS: every `a` frame the node consumed was either served to a set \
         ({} of them) or passed over by the descent ({}), so the highest ordinal \
         observed ({highest}) cannot exceed {accounted}. A larger value means \
         frames vanished between the producer and the matcher, which is exactly \
         what `block` exists to prevent",
        run.sets.len(),
        run.skips
    );
    assert!(
        accounted + 4 >= run.published,
        "and the OTHER side of the same box: at most `depth` = 4 frames can be \
         unconsumed at any instant, so the producer's {} publishes cannot run \
         more than 4 ahead of the {accounted} frames the node accounted for. \
         Under eviction the producer runs free while `accounted` stalls, and \
         this is the assertion that catches it",
        run.published
    );
    assert_eq!(run.evictions, 0, "block never evicts");
    assert_eq!(
        run.unmatched, 0,
        "every frame here has a partner within the 50 ms window, so nothing is \
         UNMATCHABLE — under `block` an under-provisioned depth costs the \
         PRODUCER's rate, never the slow frame's life"
    );
    assert!(
        run.deferred > 0,
        "the 1 ms producer really was throttled by the depth-4 edge; without \
         that this arm would be pinning an idle graph"
    );
    assert!(
        run.published >= highest,
        "sanity: the producer published at least every frame the node observed"
    );
    assert!(
        run.events >= 1,
        "the consumer's own block event fired: its queue — the held head \
         included — really did reach the defer line, which is what makes the \
         producer's defers above this consumer's flow control rather than an \
         unrelated stall"
    );
}

/// B6 — the same run twice is byte-identical, and equals the same oracle.
///
/// Determinism is Principle #7, and the slot-debt credit is a shared-atomic
/// write on the read path — exactly the sort of thing that could make a run
/// depend on interleaving. It does not: every write is driven by the polled
/// step, so two runs agree on the set sequence AND on all four counters.
#[test]
#[serial]
fn block_on_a_per_set_sync_trigger_is_deterministic() {
    let first = run_block_pair("pssb6a", 60);
    let second = run_block_pair("pssb6b", 60);
    assert_eq!(
        first.sets, second.sets,
        "two runs of one stimulus must observe the identical set sequence"
    );
    assert_eq!(
        (
            first.skips,
            first.unmatched,
            first.evictions,
            first.published
        ),
        (
            second.skips,
            second.unmatched,
            second.evictions,
            second.published
        ),
        "and the identical accounting"
    );
    assert_eq!(
        (first.deferred, first.events),
        (second.deferred, second.events),
        "including the two counters the shared `outstanding` mirror drives — \
         the slot-debt credit is a write on the READ path, so a run that \
         depended on interleaving would show up here first"
    );
    // Anti-tautology: two empty runs are trivially equal.
    assert!(!first.sets.is_empty(), "the stimulus really fired sets");
}

/// B3 — both sides of `block` report ONCE PER REGIME, driven through alignment.
///
/// The consumer's event fires off the PRE-drain outstanding, which
/// includes the member the matcher is holding — "your queue, head included, was
/// at the defer line when this drain ran". The producer's warn is the flood-latch
/// contract: loud once per regime, `debug!` for the sustained tail, and
/// the counter unconditional underneath both.
///
/// The starved-partner shape is deliberate: it opens ONE regime and never closes
/// it, so "exactly one loud line" is a statement about suppression rather than
/// about how many times the regime happened to re-arm.
#[test]
#[serial]
#[traced_test]
fn a_block_regime_on_a_sync_trigger_is_loud_once_and_counted_always() {
    const STEPS: usize = 20;
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let events = Arc::new(AtomicU64::new(0));
    let (config, factories) = starved_partner_graph(
        "pssb3",
        "/pssb3/silent",
        Arc::clone(&sets),
        Arc::clone(&events),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build block sync graph");
    for _ in 0..STEPS {
        runtime.step(Duration::from_millis(1));
    }
    let deferred = runtime
        .node_handle("fuse")
        .expect("fuse handle")
        .backpressure_block_fires_deferred_count("a");
    assert!(deferred > 1, "the regime is sustained, not a one-off");

    // The PRODUCER side. `logs_assert` sees every captured line; the level token
    // is matched as well as the message — a suppression bug that left the text
    // and the counters intact would otherwise pass — and each positive count is
    // paired with the level-free total of its own marker, so a second copy of a
    // line at another level cannot read as the one line either.
    logs_assert(|lines: &[&str]| {
        // The marker is the LOUD arm's full message: the sustained `debug!`
        // says "backpressure event (sustained; …): producer's tick deferred",
        // so the bare tail is carried at BOTH levels and is not a head marker.
        let loud = count_at_exclusively(
            lines,
            "WARN",
            &["backpressure event: producer's tick deferred"],
        )?;
        // Level-free, and FIRST: a sustained (suppressed) event must never be
        // LOUD. The exclusive DEBUG count below refuses a loud copy too, but
        // with a generic message; this arm names the condition.
        let loud_sustained = lines
            .iter()
            .filter(|l| {
                matches!(line_level(l), Some("WARN" | "INFO" | "ERROR"))
                    && l.contains("backpressure event (sustained")
                    && l.contains("policy=\"block\"")
            })
            .count();
        if loud_sustained != 0 {
            return Err(format!(
                "a sustained block event was emitted at a LOUD level ({loud_sustained} \
                 line(s)) — sustained repeats are downgraded to debug!"
            ));
        }
        let sustained = count_at_exclusively(
            lines,
            "DEBUG",
            &["backpressure event (sustained", "policy=\"block\""],
        )?;
        if loud != 1 {
            return Err(format!(
                "expected exactly 1 loud block defer WARN, got {loud}"
            ));
        }
        let want_sustained = debug_lines_expected((deferred - 1) as usize);
        if sustained != want_sustained {
            return Err(format!(
                "expected {want_sustained} sustained DEBUG lines (one per deferred pass after \
                 the regime opened), got {sustained}"
            ));
        }
        Ok(())
    });

    // The CONSUMER side, and it is a ZERO here — deliberately, and worth
    // stating. The consumer's block event fires from inside a DRAIN, off the
    // pre-drain occupancy. A held head is re-offered without draining, so once
    // the matcher is holding a member on a starved input, no further drain runs
    // on it and the consumer probe goes quiet even though the queue is full.
    // The producer's counter above is the surface that stays live in that
    // regime; the consumer event is pinned in its own shape by
    // `block_on_a_per_set_sync_trigger_loses_nothing`, where sets keep firing
    // and drains keep happening.
    assert_eq!(
        events.load(Ordering::Relaxed),
        0,
        "a held head is re-offered, never re-drained, so the consumer-side probe \
         has nothing to observe while the node is starved"
    );
}

/// B5 — the all-block rule is unchanged for Sync: a mixed topic DEGRADES.
///
/// `is_all_block` is computed over consumer EDGES with no trigger-kind term, so
/// a Sync node whose `block` trigger shares its topic with any non-block
/// consumer degrades to `drop_oldest` exactly as a Data consumer does. Deferring
/// the producer would starve the non-block sibling, and that reasoning does not
/// change because one of the consumers happens to align sets.
///
/// The control is the counters, not the log: a degraded input gets NO defer
/// edges (so the producer runs free) and DOES get the eviction detector.
#[test]
#[serial]
#[traced_test]
fn a_block_sync_trigger_sharing_its_topic_with_a_plain_consumer_degrades() {
    const STEPS: usize = 20;
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let events = Arc::new(AtomicU64::new(0));
    let config = graph(
        "pssb5_mixed",
        "pssb5",
        vec![
            node("feeder", "block_feeder", vec![], vec!["out"]),
            node(
                "fuse",
                "block_pair_fuse",
                vec![("a", "feeder/out"), ("b", "/pssb5/silent")],
                vec![],
            ),
            node(
                "sibling",
                "plain_sibling",
                vec![("inp", "feeder/out")],
                vec![],
            ),
        ],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("feeder".to_string(), Box::new(BlockFeederEntry::new()));
    factories.insert(
        "fuse".to_string(),
        Box::new(BlockPairFuseEntry::with_state(BlockPairFuse {
            sets: Some(Arc::clone(&sets)),
            events: Some(Arc::clone(&events)),
            ..Default::default()
        })),
    );
    factories.insert("sibling".to_string(), Box::new(PlainSiblingEntry::new()));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build mixed-topic graph");
    for _ in 0..STEPS {
        runtime.step(Duration::from_millis(1));
    }

    assert_eq!(
        runtime
            .node_handle("feeder")
            .expect("feeder handle")
            .fire_count(),
        STEPS as u64,
        "the producer is NEVER deferred on a mixed topic — deferring it would \
         starve the non-block sibling, so every one of its periods fires"
    );
    let fuse = runtime.node_handle("fuse").expect("fuse handle");
    assert_eq!(
        fuse.backpressure_block_fires_deferred_count("a"),
        0,
        "a degraded input has no defer edges at all"
    );
    assert_eq!(
        events.load(Ordering::Relaxed),
        0,
        "and no block event: it is not on the block policy any more"
    );
    logs_assert(|lines: &[&str]| {
        let hits = count_at_exclusively(
            lines,
            "WARN",
            &["block consumer(s) share this topic with non-block"],
        )?;
        if hits != 1 {
            return Err(format!(
                "the degrade must be announced exactly once for the topic, \
                 naming the remedy — got {hits} line(s)"
            ));
        }
        Ok(())
    });
}

// ===========================================================================
// PART 3 — `sample(N)` on a per-set Sync TRIGGER
// ===========================================================================
//
// These publish by hand on a driven `VirtualClock`, so the payload IS the wire
// stamp: a recorded set names exactly the frames it observed, and the gate's
// effect on MEMBERSHIP — not merely on a counter — is visible.

const TOPIC_A: &str = "/pssbs/a";
const TOPIC_B: &str = "/pssbs/b";

/// `a` carries a 5 ms read gate; `b` is a plain trigger.
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct SampledPairFuse {
    #[input(trigger, backpressure = sample(5), depth = 16)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    sets: Option<Sets>,
    events: Option<Arc<AtomicU64>>,
}

#[cerulion_node_impl]
impl SampledPairFuse {
    fn tick(&mut self) -> Result<(), NodeError> {
        let pair = (self.a.x, self.b.x);
        if let Some(sink) = &self.sets {
            sink.lock().expect("sets sink poisoned").push(pair);
        }
        Ok(())
    }

    #[on_event(input = "a")]
    fn on_a_pressure(&mut self, event: BackpressureEvent) {
        assert!(
            matches!(event.policy, BackpressurePolicy::Sample(5)),
            "a sample(5) input must surface a Sample(5) event, got {:?}",
            event.policy
        );
        if let Some(n) = &self.events {
            n.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// The SAME node with NO gate on `a` — the discriminator for S1. Nothing else
/// differs, so a membership difference between the two is the gate's doing and
/// only the gate's.
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct UngatedPairFuse {
    #[input(trigger, depth = 16)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    sets: Option<Sets>,
}

#[cerulion_node_impl]
impl UngatedPairFuse {
    fn tick(&mut self) -> Result<(), NodeError> {
        let pair = (self.a.x, self.b.x);
        if let Some(sink) = &self.sets {
            sink.lock().expect("sets sink poisoned").push(pair);
        }
        Ok(())
    }
}

/// A gate (100 ms) far wider than the frames behind the head — the S3 shape,
/// where every frame after the first is decimated for a long while.
#[cerulion_node(sync_window_ms = 150)]
#[derive(Default)]
struct WideGateRecordingFuse {
    #[input(trigger, backpressure = sample(100))]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    sets: Option<Sets>,
}

#[cerulion_node_impl]
impl WideGateRecordingFuse {
    fn tick(&mut self) -> Result<(), NodeError> {
        let pair = (self.a.x, self.b.x);
        if let Some(sink) = &self.sets {
            sink.lock().expect("sets sink poisoned").push(pair);
        }
        Ok(())
    }
}

fn sampled_graph(
    prefix: &str,
    node_type: &str,
    entry: Box<dyn NodeEntry>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = graph(
        "pssbs",
        prefix,
        vec![node(
            "fuse",
            node_type,
            vec![("a", TOPIC_A), ("b", TOPIC_B)],
            vec![],
        )],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), entry);
    (config, factories)
}

/// Publish `x` onto `publisher` stamped at `at_ns` on the shared virtual clock.
/// The payload carries the STAMP, so a recorded set is its own oracle.
fn publish_at(
    publisher: &mut cerulion_core::CerulionPublisher,
    clock: &VirtualClock,
    at_ns: u64,
    x: f64,
) {
    clock.set(at_ns);
    let mut p = publisher.loan_proxy::<Vector3>().expect("loan");
    p.x = x;
}

/// The S1 stimulus, run against whichever fixture is handed in.
///
/// `a` = 1 ms frames stamped 0..=10; `b` = frames stamped 0, 4, 10. Every frame
/// is queued before the first step, and the steps then drain them.
fn run_s1_stimulus(
    prefix: &str,
    node_type: &str,
    entry: Box<dyn NodeEntry>,
    sets: &Sets,
) -> GraphRuntime {
    let (config, factories) = sampled_graph(prefix, node_type, entry);
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
        publish_at(&mut pub_a, &clock, ts * MS, ts as f64);
    }
    for ts in [0u64, 4, 10] {
        publish_at(&mut pub_b, &clock, ts * MS, ts as f64);
    }
    clock.set(0);
    // One step per boundary; the gate admits at most one `a` frame per boundary,
    // so the backlog drains over several.
    for _ in 0..14 {
        runtime.step(Duration::from_millis(1));
    }
    let _ = sets;
    runtime
}

/// S1 — the gate decimates BEFORE matching, and it CHANGES WHICH FRAMES ALIGN.
///
/// This is the arm that makes "decimates before matching" a contract rather than
/// an accident of shared code, and its discriminator is MEMBERSHIP, not a
/// counter. Stimulus: `a` stamped every 1 ms from 0 to 10 under a 5 ms gate,
/// `b` stamped 0, 4 and 10.
///
/// GATED, hand-walked: the gate admits `a`@0 (first frame always), then refuses
/// 1..4 and admits `a`@5 (5 - 0 >= 5), then refuses 6..9 and admits `a`@10. The
/// matcher sees only those three, so the sets are (0,0), (5,4), (10,10).
///
/// UNGATED, same stimulus: the matcher sees every `a` frame, pairs (0,0), then
/// (1,4) — `a`@1 is simply the next frame in the queue — and then descends to
/// (10,10) once `b` is scarce. The middle set is the discriminator: `(5,4)`
/// versus `(1,4)` is the gate deciding which frames were ELIGIBLE, and no
/// counter assertion can see it.
///
/// The (5,4) set is also the decision working as written: an ungated matcher would
/// have preferred `a`@4 (span 0), and the gate dropped it. Only frames the
/// read-gate accepts are eligible to join a set.
#[test]
#[serial]
fn a_sample_gate_decides_which_frames_are_eligible_to_align() {
    let gated: Sets = Arc::new(Mutex::new(Vec::new()));
    let events = Arc::new(AtomicU64::new(0));
    let runtime = run_s1_stimulus(
        "pss1g",
        "sampled_pair_fuse",
        Box::new(SampledPairFuseEntry::with_state(SampledPairFuse {
            sets: Some(Arc::clone(&gated)),
            events: Some(Arc::clone(&events)),
            ..Default::default()
        })),
        &gated,
    );
    let observed = recorded(&gated);
    assert_eq!(
        observed,
        vec![(0.0, 0.0), (5.0, 4.0), (10.0, 10.0)],
        "only ADMITTED frames are eligible to align: the gate refuses a@1..4, so \
         the set that pairs with b@4 reads a@5 — even though a@4 arrived, was in \
         window, and would have given a tighter span"
    );

    let handle = runtime.node_handle("fuse").expect("fuse handle");
    assert_eq!(
        handle.backpressure_sampled_count("a"),
        8,
        "eight of a's eleven frames were decimated (1,2,3,4 and 6,7,8,9); the \
         gate runs on EVERY per-set pop, so each is counted once"
    );
    assert_eq!(
        handle.sync_closer_skip_count(TOPIC_A),
        0,
        "no frame was PASSED OVER: the descent only ever ran at the last set, \
         where nothing was queued behind a@10. A gate drop is not a skip — the \
         two accountings are disjoint"
    );
    assert_eq!(
        handle.sync_unmatched_discard_count(TOPIC_A),
        0,
        "every admitted frame found a partner inside the window"
    );
    assert!(
        events.load(Ordering::Relaxed) >= 1,
        "the decimation regime reached the node's `#[on_event]` handler"
    );

    // THE DISCRIMINATOR — the identical stimulus with the gate removed.
    let ungated: Sets = Arc::new(Mutex::new(Vec::new()));
    let ungated_runtime = run_s1_stimulus(
        "pss1u",
        "ungated_pair_fuse",
        Box::new(UngatedPairFuseEntry::with_state(UngatedPairFuse {
            sets: Some(Arc::clone(&ungated)),
            ..Default::default()
        })),
        &ungated,
    );
    let ungated_sets = recorded(&ungated);
    assert_ne!(
        ungated_sets, observed,
        "if the gate were consulted AFTER matching — or not at all — these two \
         runs would be identical, and every assertion above would be describing \
         the matcher rather than the gate. Ungated observed {ungated_sets:?}"
    );
    assert_eq!(
        ungated_runtime
            .node_handle("fuse")
            .expect("fuse handle")
            .backpressure_sampled_count("a"),
        0,
        "and the control really is ungated"
    );
}

/// S2 — a gated backlog drains ONE SET PER BOUNDARY, and that is the contract.
///
/// The per-set headline is that a queued backlog of k complete sets fires k
/// times within ONE step. A `sample(N)` trigger CAPS that at one: the burst
/// re-fills between fires, a refill that the gate decimates answers "nothing"
/// rather than a head, and an incomplete tuple ends the burst. The backlog is
/// still served in full and in order — it just takes a boundary per set.
///
/// Worth stating as an arm because it is the one place `sample(N)` changes a
/// per-set guarantee rather than merely thinning a stream, and because the
/// natural expectation (k fires in one step, as the ungated backlog gives) is
/// wrong.
#[test]
#[serial]
fn a_gated_backlog_serves_one_set_per_boundary_and_loses_no_set() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = sampled_graph(
        "pss2",
        "sampled_pair_fuse",
        Box::new(SampledPairFuseEntry::with_state(SampledPairFuse {
            sets: Some(Arc::clone(&sets)),
            ..Default::default()
        })),
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

    // Three sets, each pair 5 ms apart — every `a` frame here is ADMISSIBLE, so
    // the only thing that can cap the burst is the decimated-refill rule.
    for ts in [0u64, 5, 10] {
        publish_at(&mut pub_a, &clock, ts * MS, ts as f64);
        publish_at(&mut pub_b, &clock, ts * MS, ts as f64);
    }
    clock.set(0);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        recorded(&sets).len(),
        3,
        "with nothing to decimate, a gated input keeps the k-fires-per-step \
         contract in full: three queued sets, one step, three fires"
    );

    // Now the shape that DOES cap it: a 1 ms `a` stream behind the gate.
    let sets2: Sets = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = sampled_graph(
        "pss2b",
        "sampled_pair_fuse",
        Box::new(SampledPairFuseEntry::with_state(SampledPairFuse {
            sets: Some(Arc::clone(&sets2)),
            ..Default::default()
        })),
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
        publish_at(&mut pub_a, &clock, ts * MS, ts as f64);
    }
    for ts in [0u64, 5, 10] {
        publish_at(&mut pub_b, &clock, ts * MS, ts as f64);
    }
    clock.set(0);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        recorded(&sets2),
        vec![(0.0, 0.0)],
        "the first fire consumes a@0; the refill pops a@1, the gate decimates \
         it, and an unfilled head ends the burst — so the rest of the backlog \
         waits for later boundaries instead of firing now"
    );
    for _ in 0..14 {
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(
        recorded(&sets2),
        vec![(0.0, 0.0), (5.0, 5.0), (10.0, 10.0)],
        "and no set is LOST by the cap — the backlog is served in full, in \
         order, one boundary at a time"
    );
}

/// S3 — a decimated fill never becomes a head, never fabricates a stamp, and
/// self-heals in one boundary.
///
/// `a` under a gate wide enough (100 ms) that only its first frame is admitted
/// for a long while. The frames behind it are popped and dropped by the gate,
/// and each such pop must answer "nothing here" — NOT a head with a fabricated
/// stamp, which is what a `(popped >= 1, no timestamp)` answer would become if
/// it were mapped to `Head(0)`. A 0-stamp head would be 100 ms below every real
/// frame and would drag the whole window with it.
#[test]
#[serial]
fn a_decimated_fill_never_becomes_a_head() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let config = graph(
        "pss3",
        "pss3",
        vec![node(
            "fuse",
            "wide_gate_recording_fuse",
            vec![("a", TOPIC_A), ("b", TOPIC_B)],
            vec![],
        )],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "fuse".to_string(),
        Box::new(WideGateRecordingFuseEntry::with_state(
            WideGateRecordingFuse {
                sets: Some(Arc::clone(&sets)),
                ..Default::default()
            },
        )),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 32)
        .expect("build wide-gate sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on b");

    // a@0 is admitted; a@1 and a@2 are 1 ms and 2 ms behind it, far inside the
    // 100 ms gate, so both are decimated.
    for ts in 0..=2u64 {
        publish_at(&mut pub_a, &clock, ts * MS, ts as f64);
    }
    publish_at(&mut pub_b, &clock, 0, 0.0);
    clock.set(0);
    for _ in 0..4 {
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(
        recorded(&sets),
        vec![(0.0, 0.0)],
        "exactly one set: a@0 with b@0. The two decimated frames produced no \
         second fire, and — the point of the arm — no fire carrying a \
         fabricated 0-stamped member"
    );
    let handle = runtime.node_handle("fuse").expect("fuse handle");
    assert_eq!(
        handle.backpressure_sampled_count("a"),
        2,
        "both frames behind the head were popped and dropped by the gate"
    );

    // SELF-HEAL: the decimated pops leave an EMPTY head, and the next boundary
    // must pop straight through it once real frames arrive again.
    publish_at(&mut pub_a, &clock, 200 * MS, 200.0);
    publish_at(&mut pub_b, &clock, 200 * MS, 200.0);
    clock.set(200 * MS);
    for _ in 0..4 {
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(
        recorded(&sets),
        vec![(0.0, 0.0), (200.0, 200.0)],
        "an `Empty` head is not a wedge — the next drain replaces it and the \
         node fires again on the next admitted frame"
    );
}

/// S4 — the decimation regime is loud ONCE and counted ALWAYS, through the align
/// pops.
///
/// The same contract the body-read path carries, driven through
/// alignment instead: the first decimation of a regime warns, the sustained tail
/// is `debug!`, the counter is unconditional under both, and the node's
/// `#[on_event]` handler sees exactly one event per regime.
///
/// The per-set path is where this could have silently regressed: the gate now
/// runs on EVERY pop (fill, refill, peek, advance) rather than only on the
/// survivor of a drain-to-latest, so a per-frame warn here would flood at the
/// alignment rate.
#[test]
#[serial]
#[traced_test]
fn a_decimation_regime_through_the_align_pops_is_loud_once_and_counted_always() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let events = Arc::new(AtomicU64::new(0));
    let runtime = run_s1_stimulus(
        "pss4",
        "sampled_pair_fuse",
        Box::new(SampledPairFuseEntry::with_state(SampledPairFuse {
            sets: Some(Arc::clone(&sets)),
            events: Some(Arc::clone(&events)),
            ..Default::default()
        })),
        &sets,
    );
    let sampled = runtime
        .node_handle("fuse")
        .expect("fuse handle")
        .backpressure_sampled_count("a");
    assert_eq!(sampled, 8, "the stimulus's hand-walked decimation count");

    // The stimulus opens TWO regimes: a@1..4 (closed by admitting a@5) and
    // a@6..9. Each opens with one loud line and each contributes one event.
    logs_assert(|lines: &[&str]| {
        // The LOUD arm's full message (see the block twin above: the sustained
        // `debug!` carries the same tail).
        let loud = count_at_exclusively(
            lines,
            "WARN",
            &["backpressure event: dropped message arrived within sample window"],
        )?;
        // Level-free, and FIRST: a sustained (suppressed) event must never be
        // LOUD. The exclusive DEBUG count below refuses a loud copy too, but
        // with a generic message; this arm names the condition.
        let loud_sustained = lines
            .iter()
            .filter(|l| {
                matches!(line_level(l), Some("WARN" | "INFO" | "ERROR"))
                    && l.contains("backpressure event (sustained")
                    && l.contains("policy=\"sample\"")
            })
            .count();
        if loud_sustained != 0 {
            return Err(format!(
                "a sustained sample event was emitted at a LOUD level ({loud_sustained} \
                 line(s)) — sustained repeats are downgraded to debug!"
            ));
        }
        let sustained = count_at_exclusively(
            lines,
            "DEBUG",
            &["backpressure event (sustained", "policy=\"sample\""],
        )?;
        if loud != 2 {
            return Err(format!(
                "expected exactly 2 loud sample WARNs (one per regime: a@1..4 \
                 and a@6..9), got {loud}"
            ));
        }
        let want_sustained = debug_lines_expected(8 - loud);
        if sustained != want_sustained {
            return Err(format!(
                "every decimation must be logged exactly once at one level or the \
                 other: expected {want_sustained} sustained DEBUG lines beside {loud} loud \
                 (8 decimations; 0 sustained where `debug!` is compiled out), got {sustained}"
            ));
        }
        Ok(())
    });
    assert_eq!(
        events.load(Ordering::Relaxed),
        2,
        "one BackpressureEvent per regime — never one per decimated frame"
    );
}

/// S5 — a member frame is accounted EXACTLY ONCE, wherever it travels.
///
/// Under per-set a frame can take a long road: popped by the descent's stamp
/// probe into the staged slot, promoted to the head at a later boundary, and
/// only then served to a tick. Each hand-off is a place a second gate consult
/// could creep in, and a doubled `sampled_count` is what that would look like.
///
/// The stimulus drives a real descent (`b` is scarce, so the gate passes and the
/// walk stages a frame behind `a`'s head), then serves it. The oracle is the
/// hand-walked decimation count, unchanged by the staging.
#[test]
#[serial]
fn a_staged_member_is_accounted_once_across_its_whole_journey() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = sampled_graph(
        "pss5",
        "sampled_pair_fuse",
        Box::new(SampledPairFuseEntry::with_state(SampledPairFuse {
            sets: Some(Arc::clone(&sets)),
            ..Default::default()
        })),
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

    // a@0 and a@20 are both ADMITTED (20 ms apart, gate is 5 ms). b@20 alone, so
    // `b` is scarce at the boundary and the descent gate passes: the walk stages
    // a@20 behind the a@0 head, finds span 0 < 20, and advances onto it.
    publish_at(&mut pub_a, &clock, 0, 0.0);
    publish_at(&mut pub_a, &clock, 20 * MS, 20.0);
    publish_at(&mut pub_b, &clock, 20 * MS, 20.0);
    clock.set(0);
    for _ in 0..4 {
        runtime.step(Duration::from_millis(1));
    }

    assert_eq!(
        recorded(&sets),
        vec![(20.0, 20.0)],
        "the descent really ran: it passed over a@0 for the tighter a@20 — \
         without that this arm never stages anything and proves nothing"
    );
    let handle = runtime.node_handle("fuse").expect("fuse handle");
    assert_eq!(
        handle.sync_closer_skip_count(TOPIC_A),
        1,
        "a@0 was PASSED OVER, counted once"
    );
    assert_eq!(
        handle.backpressure_sampled_count("a"),
        0,
        "both frames were admitted, so the gate dropped nothing — and the staged \
         frame's journey through peek, promote and serve added no phantom \
         decimation on the way"
    );
}

/// S6 — the gated run is deterministic, and equals the same hand oracle twice.
#[test]
#[serial]
fn a_sample_gated_per_set_run_is_deterministic() {
    let oracle = vec![(0.0, 0.0), (5.0, 4.0), (10.0, 10.0)];
    for prefix in ["pss6a", "pss6b"] {
        let sets: Sets = Arc::new(Mutex::new(Vec::new()));
        let runtime = run_s1_stimulus(
            prefix,
            "sampled_pair_fuse",
            Box::new(SampledPairFuseEntry::with_state(SampledPairFuse {
                sets: Some(Arc::clone(&sets)),
                ..Default::default()
            })),
            &sets,
        );
        assert_eq!(
            recorded(&sets),
            oracle,
            "run {prefix} must equal the hand oracle, not merely its sibling"
        );
        assert_eq!(
            runtime
                .node_handle("fuse")
                .expect("fuse handle")
                .backpressure_sampled_count("a"),
            8,
            "and the accounting is identical across runs"
        );
    }
}
// ===========================================================================
// PART 4 — `sample(N)` wider than the alignment window is LOUD
// ===========================================================================

/// A `sample(80)` gate on a `sync_window_ms = 50` node — the wide-gate shape.
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct WideGatePairFuse {
    #[input(trigger, backpressure = sample(80))]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
}

#[cerulion_node_impl]
impl WideGatePairFuse {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = (self.a.x, self.b.x);
        Ok(())
    }
}

/// A gate wider than the node's own window warns ONCE at build, and is
/// never refused.
///
/// Refusing would be wrong. The gates are PER INPUT and the two windows compose
/// as a pipeline, so two gated partners still align whenever their admitted
/// frames land within `W` of each other — which a hardware-triggered in-phase
/// pair does every time. `N > W` is a graph one phase shift away from firing
/// nothing, not a graph that cannot fire.
///
/// So it is the loud-inference house rule: the runtime can see the hazard and
/// the author cannot, so it says so at the boundary where it is inferred, once
/// per build, naming both windows and what goes wrong.
#[test]
#[serial]
#[traced_test]
fn a_sample_gate_wider_than_the_sync_window_warns_once_and_still_builds() {
    let config = graph(
        "pssf2",
        "pssf2",
        vec![node(
            "fuse",
            "wide_gate_pair_fuse",
            vec![("a", TOPIC_A), ("b", TOPIC_B)],
            vec![],
        )],
    );
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), Box::new(WideGatePairFuseEntry::new()));
    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 32)
        .expect("N > W is a WARNING, never a refusal — an in-phase pair aligns at any N");
    drop(runtime);

    logs_assert(|lines: &[&str]| {
        let hits = count_at_exclusively(lines, "WARN", &["read gate is WIDER"])?;
        if hits != 1 {
            return Err(format!(
                "expected exactly one build-time WARN naming the wide gate, got {hits}"
            ));
        }
        let named = lines.iter().any(|l| {
            l.contains("read gate is WIDER")
                && l.contains("sample_interval_ms=80")
                && l.contains("sync_window_ms=50")
                && l.contains("input=a")
        });
        if !named {
            return Err(
                "the warn must name BOTH windows and the input, or an operator \
                 cannot tell which declaration to change"
                    .to_string(),
            );
        }
        Ok(())
    });
}

/// The wide-gate anti-tautology control: a gate INSIDE the window says nothing.
///
/// Without it, "the wide gate warns" is satisfied by a build that warns about
/// every `sample(N)` trigger input in the system, which would be noise on the
/// healthy declaration this feature is mostly used with.
#[test]
#[serial]
#[traced_test]
fn a_sample_gate_inside_the_sync_window_says_nothing() {
    let (config, factories) = sampled_graph(
        "pssf2c",
        "sampled_pair_fuse",
        Box::new(SampledPairFuseEntry::new()),
    );
    let clock = Arc::new(VirtualClock::new());
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 32)
        .expect("a 5 ms gate on a 50 ms window is ordinary");
    drop(runtime);
    logs_assert(|lines: &[&str]| {
        let hits = lines
            .iter()
            .filter(|l| l.contains("read gate is WIDER"))
            .count();
        if hits != 0 {
            return Err(format!(
                "sample(5) on a 50 ms window is exactly what this feature is \
                 for — warning about it would be noise. Got {hits} line(s)"
            ));
        }
        // Anti-vacuity: this graph really was built and its gated input really
        // was wired, so the silence above is a decision rather than an empty
        // capture.
        if !lines
            .iter()
            .any(|l| l.contains("external topic") && l.contains(TOPIC_A))
        {
            return Err("the capture does not show this graph wiring input `a` \
                        at all, so its silence proves nothing"
                .to_string());
        }
        Ok(())
    });
}

// ===========================================================================
// PART 5 — the wire-stamp EPOCH RESET (a publisher clock restart)
// ===========================================================================

/// A publisher clock RESTART must not wedge the node — and without the epoch
/// reset it does, permanently.
///
/// The shape is routine, which is why it matters: a robot reboots behind a netd
/// mirror (`publish_raw` re-injects the origin robot's header verbatim), a
/// `graph run-worker` restarts and its `VirtualClock` begins again at 0, a
/// `ros2 attach` bridge restarts and stamps ns-since-boot. The liveness observer's
/// epoch-reset guards exist precisely because that restart is routine.
///
/// What it does to a per-set Sync node with no reset: the two inputs' queues cross the epoch
/// boundary at DIFFERENT depths, so at some boundary `a`'s head is a new-epoch
/// stamp (~0) while `b`'s head is a retained OLD-epoch stamp (hours of ns).
/// `b`'s head is the tuple MAXIMUM, and the matcher has no exit that can evict
/// one: serving needs `span <= W`, `Advance` targets the ARGMIN, and
/// `DiscardTie` discards the LO tie-set — all three evict from the bottom. So
/// every new-epoch frame on the HEALTHY input is popped and counted
/// UNMATCHABLE forever, the node never fires again, the stalled input's own
/// `expect_within` is suppressed because its member is "held", and under
/// `block` its producer defers for good. Legacy latest-wins self-heals on
/// this identical stimulus, so a wedge here is a regression against that path.
///
/// The oracle is the SET SEQUENCE across the boundary, and the counters landing
/// on the input the operator should actually look at.
#[test]
#[serial]
#[traced_test]
fn a_publisher_clock_restart_re_bases_the_node_instead_of_wedging_it() {
    // An "old epoch" a long way above zero — a robot that had been up a while.
    const OLD: u64 = 3_600_000 * MS; // 1 h
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = sampled_graph(
        "psseR",
        "ungated_pair_fuse",
        Box::new(UngatedPairFuseEntry::with_state(UngatedPairFuse {
            sets: Some(Arc::clone(&sets)),
            ..Default::default()
        })),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 32)
        .expect("build the per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on b");

    // OLD EPOCH: one aligned pair, which fires normally.
    publish_at(&mut pub_a, &clock, OLD, 1.0);
    publish_at(&mut pub_b, &clock, OLD, 2.0);
    clock.set(0);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        recorded(&sets),
        vec![(1.0, 2.0)],
        "the old epoch aligns normally — this is the anti-vacuity half: the \
         node really was working before the reboot"
    );

    // The reboot. `b` still has ONE old-epoch frame queued behind it (the
    // different-queue-depth shape), and both producers then come back on a
    // clock that restarted near zero.
    publish_at(&mut pub_b, &clock, OLD + MS, 3.0);
    for k in 1..=4u64 {
        publish_at(&mut pub_a, &clock, k * MS, 100.0 + k as f64);
        publish_at(&mut pub_b, &clock, k * MS, 200.0 + k as f64);
    }
    clock.set(0);
    for _ in 0..14 {
        runtime.step(Duration::from_millis(1));
    }

    let observed = recorded(&sets);
    assert!(
        observed.len() > 1,
        "THE PIN: the node must fire again after the clock restart. Without the \
         epoch reset it fires ONCE (the old-epoch pair above) and then never again — a \
         matcher with no reset measured zero fires across \
         10,000 boundaries. Got {observed:?}"
    );
    let (a_after, b_after) = observed[1];
    assert!(
        a_after >= 101.0 && b_after >= 201.0,
        "and it fires on NEW-epoch members from both inputs, not on a stale \
         partner dragged across the boundary — got ({a_after}, {b_after})"
    );

    // The counter lands on the input whose frame was really thrown away — `b`,
    // which held the stale maximum — and it is its OWN counter, not the
    // UNMATCHABLE one, because the diagnosis and the remedy are different: an
    // unmatchable frame says "fix a skewed producer"; this says "a producer
    // rebooted and the node re-based itself", which needs nothing done.
    let handle = runtime.node_handle("fuse").expect("fuse handle");
    assert!(
        handle.sync_epoch_reset_discards(TOPIC_B) >= 1,
        "the stale head on `b` was discarded by the reset and counted against \
         `b` — the input whose frame it actually was"
    );

    logs_assert(|lines: &[&str]| {
        let heads = lines_at_exclusively(lines, "WARN", &["EPOCH RESET"])?;
        if heads.len() != 1 {
            return Err(format!(
                "the reset announces itself ONCE per regime, loudly — got {} lines",
                heads.len()
            ));
        }
        // And it NAMES the input, which the assertion above only claimed. The
        // input is the operator's whole starting point — a reset line that
        // names nothing (or names the wrong input) tells them a clock restarted
        // somewhere on this node and stops there.
        if !heads[0].contains("sync_input=/pssbs/a") {
            return Err(format!(
                "the loud line must name the input whose stamps regressed — \
                 got: {}",
                heads[0]
            ));
        }
        Ok(())
    });
}

/// The bounded COST of the re-base, stated as an arm: an old-epoch straggler
/// that arrives AFTER the reset is discarded and COUNTED, and the node keeps
/// firing.
///
/// A reset cannot be a one-shot purge of whatever heads happen to be filled at
/// that instant, because the partner's old-epoch frames are still QUEUED and
/// its head refills from that queue on the very next boundary. Measured, before
/// the band existed: the first straggler became the immortal maximum all over
/// again and the wedge resumed one boundary later. So a re-based input's fills
/// are tested against the new epoch's band until one lands inside it.
///
/// This is also where the accepted residual is paid. A
/// `multi_publisher_topics` trigger input mixes clocks, so a lower-clock
/// writer's frame can present the same evidence a restart does; the reset then
/// fires when nothing rebooted. That is ACCEPTED, and this arm is what makes the
/// acceptance sound — the cost is a COUNTED discard and a node that keeps
/// firing, never a wedge and never a wrong set. It mirrors its accepted
/// residual for the same class on the liveness side, for the same reason.
#[test]
#[serial]
fn an_old_epoch_straggler_after_the_reset_is_counted_and_the_node_keeps_firing() {
    const OLD: u64 = 3_600_000 * MS;
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = sampled_graph(
        "psseS",
        "ungated_pair_fuse",
        Box::new(UngatedPairFuseEntry::with_state(UngatedPairFuse {
            sets: Some(Arc::clone(&sets)),
            ..Default::default()
        })),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 32)
        .expect("build the per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on b");

    // `b`'s queue is DEEPER in the old epoch than `a`'s: two stragglers sit
    // behind its head when `a` has already crossed into the new clock.
    publish_at(&mut pub_a, &clock, OLD, 1.0);
    publish_at(&mut pub_b, &clock, OLD, 2.0);
    publish_at(&mut pub_b, &clock, OLD + MS, 3.0);
    publish_at(&mut pub_b, &clock, OLD + 2 * MS, 4.0);
    for k in 1..=4u64 {
        publish_at(&mut pub_a, &clock, k * MS, 100.0 + k as f64);
        publish_at(&mut pub_b, &clock, k * MS, 200.0 + k as f64);
    }
    clock.set(0);
    for _ in 0..16 {
        runtime.step(Duration::from_millis(1));
    }

    let observed = recorded(&sets);
    // EXACT, not a lower bound. `observed.len() > 1` proves liveness and
    // nothing else: a band that discarded the two stale frames AND most of the
    // valid new-epoch ones would still leave a pair and still pass. The whole
    // risk of an epoch band is that it eats good frames, so the oracle has to
    // be the full post-reset sequence.
    assert_eq!(
        observed,
        vec![
            (1.0, 2.0),
            (101.0, 201.0),
            (102.0, 202.0),
            (103.0, 203.0),
            (104.0, 204.0)
        ],
        "the old-epoch pair, then EVERY new-epoch pair, each served exactly \
         once — the re-base discards the epoch that ended and nothing else"
    );
    let handle = runtime.node_handle("fuse").expect("fuse handle");
    let discarded = handle.sync_epoch_reset_discards(TOPIC_B);
    assert!(
        discarded >= 2,
        "both of `b`'s queued old-epoch stragglers were discarded and COUNTED \
         against `b` — the operator can see exactly what the restart cost them \
         (got {discarded})"
    );
    assert_eq!(
        handle.sync_epoch_reset_discards(TOPIC_A),
        0,
        "and NOTHING is charged to `a`, whose frames were all fine: the counter \
         names the input whose frames were really thrown away"
    );
}

/// The reset keys ONLY on wire stamps in the popped stream, so it re-executes
/// identically — Principle #7.
///
/// Worth its own arm because a reset is a control-flow decision that DISCARDS
/// frames: if it keyed on anything ambient (a wall clock, an arrival instant, a
/// queue depth read at the wrong moment) a replay would take a different branch
/// and diverge on membership, and the divergence would be blamed on the
/// candidate.
#[test]
#[serial]
fn an_epoch_reset_replays_identically() {
    const OLD: u64 = 3_600_000 * MS;
    let run = |prefix: &str| -> Vec<(f64, f64)> {
        let sets: Sets = Arc::new(Mutex::new(Vec::new()));
        let (config, factories) = sampled_graph(
            prefix,
            "ungated_pair_fuse",
            Box::new(UngatedPairFuseEntry::with_state(UngatedPairFuse {
                sets: Some(Arc::clone(&sets)),
                ..Default::default()
            })),
        );
        let clock = Arc::new(VirtualClock::new());
        let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 32)
            .expect("build the per-set sync graph");
        let mgr = runtime.test_transport().expect("test transport parked");
        let mut pub_a = mgr
            .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
            .expect("publisher on a");
        let mut pub_b = mgr
            .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
            .expect("publisher on b");
        publish_at(&mut pub_a, &clock, OLD, 1.0);
        publish_at(&mut pub_b, &clock, OLD, 2.0);
        publish_at(&mut pub_b, &clock, OLD + MS, 3.0);
        for k in 1..=4u64 {
            publish_at(&mut pub_a, &clock, k * MS, 100.0 + k as f64);
            publish_at(&mut pub_b, &clock, k * MS, 200.0 + k as f64);
        }
        clock.set(0);
        for _ in 0..14 {
            runtime.step(Duration::from_millis(1));
        }
        recorded(&sets)
    };
    let first = run("pssD1");
    let second = run("pssD2");
    assert_eq!(
        first, second,
        "two runs of one stimulus must take the same reset decisions and serve \
         the same members"
    );
    assert!(
        first.len() > 1,
        "anti-vacuity: the stimulus really crossed the boundary and kept firing"
    );
}
