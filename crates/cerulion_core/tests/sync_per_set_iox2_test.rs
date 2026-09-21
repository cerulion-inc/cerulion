// SPDX-License-Identifier: AGPL-3.0-only
//! PER-SET Sync delivery over real iceoryx2 — one fire per COMPLETE
//! aligned set, in set order, each trigger message consumed by at most one set.
//!
//! # Why this file exists rather than more arms in `sync_fire_iox2_test.rs`
//!
//! The entire earlier Sync suite is **value-blind**: every fixture in it
//! counts fires and reads nothing, so it cannot see WHICH frames a fire
//! observed — and "which frames" is the whole of per-set delivery. A suite that
//! only counts would pass a node that fires the right number of times on the
//! wrong members, which is precisely the bug class here (the earlier code
//! fired once per alignment and read the FRESHEST frame on every trigger).
//!
//! So the fixture here READS its members into a shared sink, and every oracle
//! is a hand-written list of `(a, b)` tuples. Nothing self-compares.
//!
//! # How the stamps are made
//!
//! A frame's wire timestamp is the transport clock at LOAN time, so the tests
//! drive a `VirtualClock` by hand (`clock.set(..)` before each publish) and the
//! payload carries the same number. That makes the payload its own oracle: a
//! fire reading `(1.0, 10.0)` observed exactly the frames stamped 1 ms and
//! 10 ms, whatever the transport did in between.
//!
//! `#[serial]` — real iceoryx2 over the process-global SHM singleton;
//! per-test SHM root via `build_for_test`.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef};
use cerulion_core::graph::node::{
    BackpressurePolicy, ClosureNodeEntry, InputMeta, MacroPolicy, NodeEntry, NodeInfo,
};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::message::ShmMessage;
use cerulion_core::prelude::*;
use cerulion_core::transport::TopicServiceConfig;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;
use tracing_test::traced_test;

const TOPIC_A: &str = "/pss/a";
const TOPIC_B: &str = "/pss/b";
const TOPIC_C: &str = "/pss/c";

const MS: u64 = 1_000_000;

/// Every `(a, b)` pair a fire observed, in fire order — the SET SEQUENCE.
type Sets = Arc<Mutex<Vec<(f64, f64)>>>;
/// Every `(a, b, c)` triple a fire observed.
type Triples = Arc<Mutex<Vec<(f64, f64, f64)>>>;

// ---------------------------------------------------------------------------
// Fixtures: the CHUNK-0 requirement — a Sync node whose tick READS its members.
// ---------------------------------------------------------------------------

/// A 2-input bounded-Sync node that RECORDS the pair it read.
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct PairFuse {
    #[input(trigger)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    sets: Option<Sets>,
}

#[cerulion_node_impl]
impl PairFuse {
    fn tick(&mut self) -> Result<(), NodeError> {
        let pair = (self.a.x, self.b.x);
        if let Some(sink) = &self.sets {
            sink.lock().expect("sets sink poisoned").push(pair);
        }
        Ok(())
    }
}

/// A `PairFuse` with a producer rate cap, so a complete aligned set can be made
/// to SIT in the heads unfired — the only shape in which
/// `sync_backlog_pending()` can be observed TRUE.
#[cerulion_node(sync_window_ms = 50, throttle_ms = 50)]
#[derive(Default)]
struct ThrottledPairFuse {
    #[input(trigger)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    sets: Option<Sets>,
}

#[cerulion_node_impl]
impl ThrottledPairFuse {
    fn tick(&mut self) -> Result<(), NodeError> {
        let pair = (self.a.x, self.b.x);
        if let Some(sink) = &self.sets {
            sink.lock().expect("sets sink poisoned").push(pair);
        }
        Ok(())
    }
}

/// The UNBOUNDED twin — no window, so window death can never fire and the arms
/// isolate descent from death.
#[cerulion_node(unbounded_sync)]
#[derive(Default)]
struct PairFuseUnbounded {
    #[input(trigger)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    sets: Option<Sets>,
}

#[cerulion_node_impl]
impl PairFuseUnbounded {
    fn tick(&mut self) -> Result<(), NodeError> {
        let pair = (self.a.x, self.b.x);
        if let Some(sink) = &self.sets {
            sink.lock().expect("sets sink poisoned").push(pair);
        }
        Ok(())
    }
}

/// The 3-topic fixture for the signed-off worked example.
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct TripleFuse {
    #[input(trigger)]
    cam: Vector3,
    #[input(trigger)]
    lidar: Vector3,
    #[input(trigger)]
    imu: Vector3,
    sets: Option<Triples>,
}

#[cerulion_node_impl]
impl TripleFuse {
    fn tick(&mut self) -> Result<(), NodeError> {
        let triple = (self.cam.x, self.lidar.x, self.imu.x);
        if let Some(sink) = &self.sets {
            sink.lock().expect("sets sink poisoned").push(triple);
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

fn pair_graph(
    name: &str,
    prefix: &str,
    entry: Box<dyn NodeEntry>,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: name.to_string(),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "fuse".to_string(),
            node_type: "pair_fuse".to_string(),
            inputs: vec![
                InputDef {
                    name: "a".to_string(),
                    source: TOPIC_A.to_string(),
                },
                InputDef {
                    name: "b".to_string(),
                    source: TOPIC_B.to_string(),
                },
            ],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), entry);
    (config, factories)
}

fn triple_graph(sets: Triples) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "pss_triple".to_string(),
        prefix: "psstri".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "fuse".to_string(),
            node_type: "triple_fuse".to_string(),
            inputs: vec![
                InputDef {
                    name: "cam".to_string(),
                    source: TOPIC_A.to_string(),
                },
                InputDef {
                    name: "lidar".to_string(),
                    source: TOPIC_B.to_string(),
                },
                InputDef {
                    name: "imu".to_string(),
                    source: TOPIC_C.to_string(),
                },
            ],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "fuse".to_string(),
        Box::new(TripleFuseEntry::with_state(TripleFuse {
            sets: Some(sets),
            ..Default::default()
        })),
    );
    (config, factories)
}

/// Publish `x` onto `publisher` stamped at `at_ns` on the shared virtual clock.
///
/// The payload carries the STAMP so a recorded set is its own oracle.
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

fn recorded(sets: &Sets) -> Vec<(f64, f64)> {
    sets.lock().expect("sets sink poisoned").clone()
}

// ---------------------------------------------------------------------------
// ARM 1 — THE BACKLOG IS SERVED, NOT EATEN
// ---------------------------------------------------------------------------

/// A=[1,2,3], B=[10,20,30] all queued before ONE step. Three complete sets are
/// present among ARRIVED frames, so THREE fires must serve them in order and
/// NOTHING may be skipped.
///
/// This is the v3-flaw-(b) regression pin: a matcher that descends whenever a
/// stamp improves produces ONE fire `(3,10)` plus two PASSED-OVER skips on this
/// exact stimulus — it eats the backlog to tighten one set. The gate is what
/// forbids it: while EVERY input still holds a second arrived frame, a complete
/// LATER tuple exists and consuming an extra frame might be consuming its
/// member.
///
/// It is ALSO the k-fires-per-step pin: before per-set delivery this stimulus produced
/// exactly ONE fire reading the freshest members, `(3, 30)`.
#[test]
#[serial]
fn a_queued_backlog_of_three_sets_fires_three_times_in_order_within_one_step() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let entry = Box::new(PairFuseEntry::with_state(PairFuse {
        sets: Some(Arc::clone(&sets)),
        ..Default::default()
    }));
    let (config, factories) = pair_graph("pss_backlog", "pssbk", entry);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 8)
        .expect("build per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    for (a, b) in [(1u64, 10u64), (2, 20), (3, 30)] {
        publish_at(&mut pub_a, &clock, a * MS, a as f64);
        publish_at(&mut pub_b, &clock, b * MS, b as f64);
    }
    clock.set(0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        recorded(&sets),
        vec![(1.0, 10.0), (2.0, 20.0), (3.0, 30.0)],
        "three complete sets were queued, so ONE step must serve all three in \
         order — before per-set delivery this was a single fire reading (3, 30), and a \
         gate-less descent eats the backlog into one fire reading (3, 10)"
    );

    let handle = runtime.node_handle("fuse").expect("node handle");
    for topic in ["pssbk/a", TOPIC_A, "pssbk/b", TOPIC_B] {
        assert_eq!(
            handle.sync_closer_skip_count(topic),
            0,
            "no frame may be passed over while every input still holds a second \
             arrived frame (topic {topic})"
        );
        assert_eq!(
            handle.sync_unmatched_discard_count(topic),
            0,
            "every frame here is in a set — nothing is unmatchable (topic {topic})"
        );
    }
}

// ---------------------------------------------------------------------------
// ARM 4 — STRICT DESCENT
// ---------------------------------------------------------------------------

/// A=[0,20], B=[10] unbounded. Advancing A to 20 gives span 10 — EQUAL to the
/// head tuple's, not better — so strict descent must REFUSE and serve `(0,10)`.
///
/// The `<` is the earliest-min tie-break, made structural: a `<=` here
/// would fire `(20,10)` and burn A@0 as a counted skip. A@20 then waits for its
/// own partner.
#[test]
#[serial]
fn a_plateau_stops_the_walk_and_serves_the_earliest_frame() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let entry = Box::new(PairFuseUnboundedEntry::with_state(PairFuseUnbounded {
        sets: Some(Arc::clone(&sets)),
        ..Default::default()
    }));
    let (config, factories) = pair_graph("pss_plateau", "psspl", entry);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 8)
        .expect("build per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    publish_at(&mut pub_a, &clock, 0, 0.0);
    publish_at(&mut pub_a, &clock, 20 * MS, 20.0);
    publish_at(&mut pub_b, &clock, 10 * MS, 10.0);
    clock.set(0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        recorded(&sets),
        vec![(0.0, 10.0)],
        "advancing A@0 to A@20 gives an EQUAL span, so the strictly-improving \
         walk refuses and the earliest frame is served (the specified tie-break)"
    );
    let handle = runtime.node_handle("fuse").expect("node handle");
    assert_eq!(
        handle.sync_closer_skip_count(TOPIC_A),
        0,
        "a refused descent skips nothing"
    );
}

// ---------------------------------------------------------------------------
// ARM 7 — THE DEEP WALK
// ---------------------------------------------------------------------------

/// a = [0..9] ms, b = [100] ms, unbounded. B is genuinely scarce, so the gate
/// passes and the walk descends A all the way to its NEAREST arrived frame.
///
/// Oracle: ONE set `(9, 100)`, `closer_skip(a) == 9`, and A's queue empty. A
/// walk that stopped early would serve a smaller `a`; one that overshot cannot
/// (there is nothing past 9).
#[test]
#[serial]
fn an_unbounded_deep_walk_serves_the_nearest_arrived_member_and_counts_the_rest() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let entry = Box::new(PairFuseUnboundedEntry::with_state(PairFuseUnbounded {
        sets: Some(Arc::clone(&sets)),
        ..Default::default()
    }));
    let (config, factories) = pair_graph("pss_deep", "pssdp", entry);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    for i in 0..10u64 {
        publish_at(&mut pub_a, &clock, i * MS, i as f64);
    }
    publish_at(&mut pub_b, &clock, 100 * MS, 100.0);
    clock.set(0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        recorded(&sets),
        vec![(9.0, 100.0)],
        "b is the scarce partner, so the gate passes and the walk descends a to \
         its nearest arrived frame"
    );
    let handle = runtime.node_handle("fuse").expect("node handle");
    assert_eq!(
        handle.sync_closer_skip_count(TOPIC_A),
        9,
        "a@0..a@8 were passed over for a nearer member — counted, never silent"
    );
    assert_eq!(
        handle.sync_unmatched_discard_count(TOPIC_A),
        0,
        "a PASSED-OVER frame is the feature working, NOT an unmatchable one — \
         merging the two counters would make a real fault unreadable"
    );
}

// ---------------------------------------------------------------------------
// ARM 6 — THE COUNTER SPLIT, AND DEATH BEFORE DESCENT
// ---------------------------------------------------------------------------

/// A=[0, 210], B=[200], window 50 ms. A@0 is 200 ms from B@200, so it is
/// provably in NO set: the verdict must be DEATH, and the discard must be
/// counted UNMATCHABLE — not PASSED-OVER.
///
/// A descent-first matcher reaches the SAME final set `(210, 200)` while
/// counting `(closer, unmatched) == (1, 0)`. Only the split tells the two
/// apart, and the split is the operator's whole diagnosis: UNMATCHABLE means
/// "widen the window / fix a producer / raise `depth`", PASSED-OVER means
/// "nothing to do".
#[test]
#[serial]
fn a_window_death_counts_unmatchable_not_passed_over() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let entry = Box::new(PairFuseEntry::with_state(PairFuse {
        sets: Some(Arc::clone(&sets)),
        ..Default::default()
    }));
    let (config, factories) = pair_graph("pss_death", "pssdt", entry);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 8)
        .expect("build per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    publish_at(&mut pub_a, &clock, 0, 0.0);
    publish_at(&mut pub_a, &clock, 210 * MS, 210.0);
    publish_at(&mut pub_b, &clock, 200 * MS, 200.0);
    clock.set(0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        recorded(&sets),
        vec![(210.0, 200.0)],
        "a@0 is unmatchable; the in-window pair is (210, 200)"
    );
    let handle = runtime.node_handle("fuse").expect("node handle");
    assert_eq!(
        (
            handle.sync_unmatched_discard_count(TOPIC_A),
            handle.sync_closer_skip_count(TOPIC_A)
        ),
        (1, 0),
        "death is classified BEFORE descent: a@0 is UNMATCHABLE (1, 0). A \
         descent-first matcher reaches the same set counting (0, 1), which \
         reports a healthy feature where something is wrong"
    );
}

// ---------------------------------------------------------------------------
// ARM 8a / 8b — THE BACKLOG TAIL
// ---------------------------------------------------------------------------

/// 8a — ALL ARRIVED. A=[1,2,3,40], B=[10,20,30], window 50 ms, ONE step.
///
/// Sets 1-2 serve GREEDILY (the gate refuses while both inputs hold a second
/// arrived frame). The THIRD alignment is the tail: B's queue behind its head is
/// empty, so the gate PASSES and the descent tightens `(3,30)` span 27 into
/// `(40,30)` span 10, counting A@3 as PASSED-OVER.
///
/// The count of sets is preserved (3 either way); only the
/// tail's membership is tightened, which is the P3 behaviour. A@3's
/// hypothetical future partner is forfeited — the decision, verbatim: closest among
/// ARRIVED, fire immediately.
#[test]
#[serial]
fn the_tail_of_a_backlog_is_descent_tightened_while_its_body_serves_greedily() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let entry = Box::new(PairFuseEntry::with_state(PairFuse {
        sets: Some(Arc::clone(&sets)),
        ..Default::default()
    }));
    let (config, factories) = pair_graph("pss_tail", "psstl", entry);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 8)
        .expect("build per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    for a in [1u64, 2, 3, 40] {
        publish_at(&mut pub_a, &clock, a * MS, a as f64);
    }
    for b in [10u64, 20, 30] {
        publish_at(&mut pub_b, &clock, b * MS, b as f64);
    }
    clock.set(0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        recorded(&sets),
        vec![(1.0, 10.0), (2.0, 20.0), (40.0, 30.0)],
        "the backlog BODY serves greedily (the gate refuses); its TAIL is the \
         one scarce-partner boundary, where the descent tightens span 27 to 10"
    );
    let handle = runtime.node_handle("fuse").expect("node handle");
    assert_eq!(
        handle.sync_closer_skip_count(TOPIC_A),
        1,
        "exactly a@3 is passed over — the tail's tightening, counted"
    );
    assert_eq!(
        handle.sync_unmatched_discard_count(TOPIC_A),
        0,
        "nothing here is unmatchable (every span is inside the 50 ms window)"
    );
}

/// 8b — the LATE tail. The same stimulus with A@40 published AFTER the step.
///
/// At the third alignment A has NO second arrived frame, so there is nothing to
/// descend to and `(3,30)` fires greedily with ZERO skips. A@40 then waits for
/// its own partner.
///
/// The pair 8a/8b is what makes "closest among ARRIVED" observable: the same
/// three sets, the same window, and the membership of the tail turns entirely on
/// whether A@40 had arrived.
#[test]
#[serial]
fn a_tail_whose_successor_has_not_arrived_fires_greedily_with_no_skips() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let entry = Box::new(PairFuseEntry::with_state(PairFuse {
        sets: Some(Arc::clone(&sets)),
        ..Default::default()
    }));
    let (config, factories) = pair_graph("pss_late", "pssla", entry);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 8)
        .expect("build per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    for a in [1u64, 2, 3] {
        publish_at(&mut pub_a, &clock, a * MS, a as f64);
    }
    for b in [10u64, 20, 30] {
        publish_at(&mut pub_b, &clock, b * MS, b as f64);
    }
    clock.set(0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        recorded(&sets),
        vec![(1.0, 10.0), (2.0, 20.0), (3.0, 30.0)],
        "with no arrived successor on a there is nothing to descend to — the \
         tail fires greedily, and a@40's absence is the ONLY difference from \
         the all-arrived twin"
    );
    let handle = runtime.node_handle("fuse").expect("node handle");
    assert_eq!(
        handle.sync_closer_skip_count(TOPIC_A),
        0,
        "no frame is passed over when no nearer one has arrived"
    );
}

// ---------------------------------------------------------------------------
// ARM 2 — THE SIGNED-OFF 3-TOPIC TABLE, over real transport
// ---------------------------------------------------------------------------

/// `sync_window_ms = 50`; cam = [45], lidar = [50], imu = [0,10,20,30,40].
///
/// The outcome: one set `(cam=45, lidar=50, imu=40)`, span
/// 10 — the arrived imu frame NEAREST the (45,50) pair — with FOUR imu frames
/// passed over and nothing unmatchable.
///
/// The in-module `sync_match` walk asserts the same table's OP SEQUENCE; this
/// asserts it survives the whole transport stack.
#[test]
#[serial]
fn the_signed_off_three_topic_example_delivers_its_ruled_set() {
    let sets: Triples = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = triple_graph(Arc::clone(&sets));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_cam = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on cam");
    let mut pub_lidar = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on lidar");
    let mut pub_imu = mgr
        .create_publisher(TOPIC_C, MaxSliceLen::const_new(64), 0)
        .expect("publisher on imu");

    for i in [0u64, 10, 20, 30, 40] {
        publish_at(&mut pub_imu, &clock, i * MS, i as f64);
    }
    publish_at(&mut pub_cam, &clock, 45 * MS, 45.0);
    publish_at(&mut pub_lidar, &clock, 50 * MS, 50.0);
    clock.set(0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        sets.lock().expect("sets sink poisoned").clone(),
        vec![(45.0, 50.0, 40.0)],
        "the required set minimises the SPAN across all three inputs: imu@40 is \
         the arrived frame nearest the (45, 50) pair"
    );
    let handle = runtime.node_handle("fuse").expect("node handle");
    assert_eq!(
        handle.sync_closer_skip_count(TOPIC_C),
        4,
        "imu@0, @10, @20 and @30 were each passed over for a nearer member"
    );
    for topic in [TOPIC_A, TOPIC_B, TOPIC_C] {
        assert_eq!(
            handle.sync_unmatched_discard_count(topic),
            0,
            "every frame here is either served or passed over — none is \
             unmatchable (topic {topic})"
        );
    }
    assert_eq!(
        handle.sync_closer_skip_count(TOPIC_A),
        0,
        "cam is never walked — it is the scarce partner the gate passes on"
    );
    assert_eq!(
        handle.sync_closer_skip_count(TOPIC_B),
        0,
        "lidar is never even PROBED: the gate short-circuits at cam"
    );
}

// ---------------------------------------------------------------------------
// DETERMINISM (Principle #7)
// ---------------------------------------------------------------------------

/// Two runs of the tail stimulus produce byte-identical set sequences AND both
/// equal the hand oracle — so this is a cross-check, not a self-compare.
#[test]
#[serial]
fn the_tail_stimulus_is_deterministic_across_runs_and_equals_its_oracle() {
    fn run(tag: &str) -> (Vec<(f64, f64)>, u64) {
        let sets: Sets = Arc::new(Mutex::new(Vec::new()));
        let entry = Box::new(PairFuseEntry::with_state(PairFuse {
            sets: Some(Arc::clone(&sets)),
            ..Default::default()
        }));
        let (config, factories) = pair_graph("pss_det", tag, entry);
        let clock = Arc::new(VirtualClock::new());
        let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 8)
            .expect("build per-set sync graph");
        let mgr = runtime.test_transport().expect("test transport parked");
        let mut pub_a = mgr
            .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
            .expect("publisher on /pss/a");
        let mut pub_b = mgr
            .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
            .expect("publisher on /pss/b");
        for a in [1u64, 2, 3, 40] {
            publish_at(&mut pub_a, &clock, a * MS, a as f64);
        }
        for b in [10u64, 20, 30] {
            publish_at(&mut pub_b, &clock, b * MS, b as f64);
        }
        clock.set(0);
        runtime.step(Duration::from_millis(1));
        let handle = runtime.node_handle("fuse").expect("node handle");
        let skips = handle.sync_closer_skip_count(TOPIC_A);
        (recorded(&sets), skips)
    }

    let oracle = (vec![(1.0, 10.0), (2.0, 20.0), (40.0, 30.0)], 1u64);
    let first = run("pssd1");
    let second = run("pssd2");
    assert_eq!(first, oracle, "run 1 must equal the hand oracle");
    assert_eq!(second, oracle, "run 2 must equal the hand oracle");
    assert_eq!(
        first, second,
        "identical stimuli must produce identical set sequences (Principle #7)"
    );
}

// ---------------------------------------------------------------------------
// THE UNSERVED-SET CARRY
// ---------------------------------------------------------------------------

/// A burst that ends with a COMPLETE aligned set still in the heads must report
/// DUE-NOW, or the live loop waits for an unrelated publish (or the 250 ms
/// liveliness cap) where Data recovers at the 1 ms floor.
///
/// SCOPE, corrected: this arm drives a set that FIRES, so what it actually pins
/// is the NEGATIVE half — a node that has seen nothing, and a node whose queues
/// are empty after a fire, both report nothing pending. Its original name
/// claimed the positive half, which its body never constructed: a `false` read
/// around a set that fired is satisfied by a hint stuck permanently low. The
/// UNFIRED shape needs a pre-fire gate to hold a complete boundary-aligned set,
/// and is pinned by
/// `a_set_the_throttle_deferred_before_fire_one_is_reported_due_now`.
/// `sync_backlog_pending()` is the operator-visible mirror of the hint
/// `ns_until_next_fire` reads.
#[test]
#[serial]
fn a_node_that_owes_nothing_reports_nothing_pending() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let entry = Box::new(PairFuseEntry::with_state(PairFuse {
        sets: Some(Arc::clone(&sets)),
        ..Default::default()
    }));
    let (config, factories) = pair_graph("pss_carry", "psscy", entry);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 8)
        .expect("build per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    // The handle is re-fetched after every step: it borrows the runtime, so
    // holding one across a `step` would not compile — and re-reading is also
    // the correct shape, since the flag is a per-step observation.
    assert!(
        !runtime
            .node_handle("fuse")
            .expect("node handle")
            .sync_backlog_pending(),
        "a node that has seen nothing owes nothing"
    );

    publish_at(&mut pub_a, &clock, MS, 1.0);
    publish_at(&mut pub_b, &clock, 10 * MS, 10.0);
    clock.set(0);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        recorded(&sets),
        vec![(1.0, 10.0)],
        "the one available set fires"
    );
    assert!(
        !runtime
            .node_handle("fuse")
            .expect("node handle")
            .sync_backlog_pending(),
        "with the queues empty after the fire, nothing is owed"
    );
}

// ---------------------------------------------------------------------------
// ARM 3 — THE STAGED NEXT SURVIVES A FIRE
// ---------------------------------------------------------------------------

/// A=[0, 60, 100] and B=[10], unbounded. B is scarce, so the gate passes and
/// the matcher POPS A@60 into the staged slot to learn its stamp — and then
/// REFUSES to advance, because `|60 − 10| = 50` is worse than `|0 − 10| = 10`.
///
/// A@60 is now out of the iceoryx2 queue and living in the subscriber's staged
/// slot while the fire consumes A@0. What must happen next is the whole point:
/// the FOLLOWING boundary has to serve A@60 BY PROMOTION rather than receiving
/// from the queue, which would hand the next set A@100 and lose A@60 entirely.
///
/// Oracle: `[(0,10), (60,70)]` with ZERO skips on both counters — A@60 was
/// never passed over, it was retained. An implementation that drops the staged
/// frame fires `(100, 70)` and strands A@60: a WRONG-MEMBERSHIP failure, and an
/// in-order violation (A would deliver 0 then 100, never 60).
///
/// The stamps are chosen so boundary 2's descent also REFUSES (A@100 against
/// B@70 is worse than A@60), which keeps the oracle a membership claim rather
/// than a counter claim.
#[test]
#[serial]
fn a_staged_next_survives_a_fire_and_is_promoted_at_the_following_boundary() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let entry = Box::new(PairFuseUnboundedEntry::with_state(PairFuseUnbounded {
        sets: Some(Arc::clone(&sets)),
        ..Default::default()
    }));
    let (config, factories) = pair_graph("pss_stage", "pssst", entry);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 8)
        .expect("build per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    for a in [0u64, 60, 100] {
        publish_at(&mut pub_a, &clock, a * MS, a as f64);
    }
    publish_at(&mut pub_b, &clock, 10 * MS, 10.0);
    clock.set(0);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        recorded(&sets),
        vec![(0.0, 10.0)],
        "boundary 1: the descent is REFUSED (span 50 is worse than 10), so the \
         head tuple fires and A@60 stays staged"
    );

    // B's partner for the retained frame arrives.
    publish_at(&mut pub_b, &clock, 70 * MS, 70.0);
    clock.set(0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        recorded(&sets),
        vec![(0.0, 10.0), (60.0, 70.0)],
        "boundary 2 must serve the STAGED A@60 by promotion. An implementation \
         that pops the queue instead fires (100, 70) and strands A@60 — a wrong \
         set AND an in-order violation on A (0, then 100, never 60)"
    );
    let handle = runtime.node_handle("fuse").expect("node handle");
    assert_eq!(
        handle.sync_closer_skip_count(TOPIC_A),
        0,
        "A@60 was RETAINED, not passed over — a nonzero count here would mean \
         the frame was consumed as a skip rather than served as a member"
    );
    assert_eq!(
        handle.sync_unmatched_discard_count(TOPIC_A),
        0,
        "unbounded sync cannot produce an unmatchable frame"
    );
}

// ---------------------------------------------------------------------------
// ARM 9 — SERVE-MANY CONTEXT x BURST
// ---------------------------------------------------------------------------

/// A 2-trigger Sync node with a PLAIN (non-trigger) context input.
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct PairFuseCtx {
    #[input(trigger)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    #[input]
    cfg: Vector3,
    sets: Option<Triples>,
}

#[cerulion_node_impl]
impl PairFuseCtx {
    fn tick(&mut self) -> Result<(), NodeError> {
        let triple = (self.a.x, self.b.x, self.cfg.x);
        if let Some(sink) = &self.sets {
            sink.lock().expect("sets sink poisoned").push(triple);
        }
        Ok(())
    }
}

/// EVERY fire of one step's burst reads the SAME frozen context bytes, while
/// each fire reads its OWN set's trigger members.
///
/// This is the composition the two in-flight contracts have to make together:
/// the trigger members are `Sample` slots consumed per fire (taking the head IS
/// the served signal the refill discipline depends on), while the context is a
/// `Held` slot that survives its serve and is re-served to every fire of the
/// step. Get it wrong in either direction and the failure is silent — a
/// context that were consumed would leave fires 2..k reading a LIVE drain (so a
/// mid-step publish could leak in, breaking Principle #7), and a trigger that
/// survived would re-fire the node on one frame.
///
/// The second step is the other half: a context published after the freeze is
/// invisible until the NEXT boundary, and then it is the value every fire sees.
#[test]
#[serial]
fn every_fire_of_a_burst_reads_the_same_frozen_context_with_its_own_members() {
    let sets: Triples = Arc::new(Mutex::new(Vec::new()));
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "pss_ctx".to_string(),
        prefix: "pssctx".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "fuse".to_string(),
            node_type: "pair_fuse_ctx".to_string(),
            inputs: vec![
                InputDef {
                    name: "a".to_string(),
                    source: TOPIC_A.to_string(),
                },
                InputDef {
                    name: "b".to_string(),
                    source: TOPIC_B.to_string(),
                },
                InputDef {
                    name: "cfg".to_string(),
                    source: TOPIC_C.to_string(),
                },
            ],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "fuse".to_string(),
        Box::new(PairFuseCtxEntry::with_state(PairFuseCtx {
            sets: Some(Arc::clone(&sets)),
            ..Default::default()
        })),
    );
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 8)
        .expect("build per-set sync graph with a context input");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on b");
    let mut pub_cfg = mgr
        .create_publisher(TOPIC_C, MaxSliceLen::const_new(64), 0)
        .expect("publisher on cfg");

    publish_at(&mut pub_cfg, &clock, 0, 7.0);
    for (a, b) in [(1u64, 10u64), (2, 20), (3, 30)] {
        publish_at(&mut pub_a, &clock, a * MS, a as f64);
        publish_at(&mut pub_b, &clock, b * MS, b as f64);
    }
    clock.set(0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        sets.lock().expect("sets sink poisoned").clone(),
        vec![(1.0, 10.0, 7.0), (2.0, 20.0, 7.0), (3.0, 30.0, 7.0)],
        "three sets fire in one step; each reads ITS OWN members and ALL of \
         them read the same frozen context"
    );

    // A context published after the freeze belongs to the NEXT boundary.
    publish_at(&mut pub_cfg, &clock, 40 * MS, 9.0);
    publish_at(&mut pub_a, &clock, 4 * MS, 4.0);
    publish_at(&mut pub_b, &clock, 40 * MS, 40.0);
    clock.set(0);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        sets.lock()
            .expect("sets sink poisoned")
            .clone()
            .last()
            .copied(),
        Some((4.0, 40.0, 9.0)),
        "the next step's freeze picks up the new context — the cross-step hold and \
         the step-boundary freeze are untouched by per-set delivery"
    );
}

// ---------------------------------------------------------------------------
// ARM 10b — PANIC x BURST, the POST-READ (shipping) shape
// ---------------------------------------------------------------------------

/// A macro Sync fixture that READS both members and then PANICS.
#[cerulion_node(sync_window_ms = 500)]
#[derive(Default)]
struct PairFusePanic {
    #[input(trigger)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    sets: Option<Sets>,
    /// Kept as a FIELD rather than an unconditional `panic!` so the tick still
    /// has a reachable typed tail — a body whose last statement diverges gives
    /// the macro's rewriter nothing to infer a return type from.
    panic_after_read: bool,
}

#[cerulion_node_impl]
impl PairFusePanic {
    fn tick(&mut self) -> Result<(), NodeError> {
        let pair = (self.a.x, self.b.x);
        if let Some(sink) = &self.sets {
            sink.lock().expect("sets sink poisoned").push(pair);
        }
        if self.panic_after_read {
            panic!("per-set Sync post-read panic fixture");
        }
        Ok(())
    }
}

/// The macro tick takes EVERY trigger member before the user body runs, so a
/// user-body panic leaves the members CONSUMED and lost. The memo predicts the
/// burst then keeps going — aligning FRESH sets into the still-panicking body
/// until `MAX_CONSECUTIVE_PANICS` (3) opens the breaker, all inside one step,
/// mirroring what `tick_data_burst` does for Data on 3 distinct frames.
///
/// **MEASURED: that is not what happens, and the real behaviour is BETTER.**
/// The tick runs while the runtime holds the node's `Mutex`, so a panic
/// unwinding out of it POISONS that mutex — a pre-existing, documented property
/// (`runtime.rs`: "a prior tick panic leaves the entry mutex poisoned"; the
/// runtime recovers it only where reading is safe regardless of user-state
/// consistency, such as `pump_history_all`). Every subsequent align op goes
/// through that same lock, gets `Err`, and maps to `SyncOpAnswer::Failed`,
/// which R-Fail resolves fail-closed: the alignment is incomplete and the burst
/// ENDS.
///
/// So the loss is bounded at ONE set per panic, not three, and it cannot
/// cascade within a step. The same mechanism bounds the Data burst — its refill
/// hook carries the identical `Err(_) => (0, None)` arm, whose comment already
/// says "the burst simply ends here" — so the two twins do still agree, just at
/// one rather than three.
///
/// The node is INERT afterwards (the poison is permanent for the drain path)
/// with a flood-latched loud error naming the failure. That is a pre-existing
/// consequence of a panicking node, not something per-set delivery introduced;
/// the arm pins it so a future change to the poison policy shows up HERE rather
/// than as a silent change in how much data a panicking node eats.
#[test]
#[serial]
fn a_post_read_panic_loses_exactly_one_set_and_the_burst_cannot_cascade() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let entry = Box::new(PairFusePanicEntry::with_state(PairFusePanic {
        sets: Some(Arc::clone(&sets)),
        panic_after_read: true,
        ..Default::default()
    }));
    let (config, factories) = pair_graph("pss_panic", "psspn", entry);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    for i in 1..=5u64 {
        publish_at(&mut pub_a, &clock, i * MS, i as f64);
        publish_at(&mut pub_b, &clock, i * MS + 100_000, (i * 10) as f64);
    }
    clock.set(0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        recorded(&sets),
        vec![(1.0, 10.0)],
        "exactly ONE set reaches the panicking body. The burst cannot cascade: \
         the panic poisoned the node mutex, so the refill alignment's very \
         first op fails and R-Fail ends the pass fail-closed"
    );
    assert_eq!(
        runtime
            .node_handle("fuse")
            .expect("node handle")
            .panic_count(),
        1,
        "one fire, one panic — the breaker never even reaches its threshold \
         inside this step"
    );

    // Sets 2-5 were never touched, and further steps consume nothing.
    for _ in 0..3 {
        clock.set(0);
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(
        recorded(&sets).len(),
        1,
        "the loss is bounded at ONE set — a panicking Sync node does not go on \
         eating its backlog step after step"
    );
}

// ---------------------------------------------------------------------------
// ARM 13 — batching dependence is by design
// ---------------------------------------------------------------------------

/// The SAME three frames, delivered in two different batchings, produce two
/// different memberships. This is by design, not a defect: the descent's
/// moves depend on which frames have arrived, and the decision is closest-among-
/// arrived, fire immediately.
///
/// The arm exists so the property is DECLARED rather than discovered — and so
/// that a future change which accidentally made the two batchings agree (by
/// waiting for a possibly-nearer frame) fails loudly here.
#[test]
#[serial]
fn the_same_frames_batched_two_ways_deliver_two_different_sets_and_that_is_ruled() {
    // BATCHING 1 — everything arrived before the step. b is scarce, so the
    // gate passes and the walk tightens a@0 to a@5.
    let batched: Sets = Arc::new(Mutex::new(Vec::new()));
    {
        let entry = Box::new(PairFuseUnboundedEntry::with_state(PairFuseUnbounded {
            sets: Some(Arc::clone(&batched)),
            ..Default::default()
        }));
        let (config, factories) = pair_graph("pss_batch1", "pssb1", entry);
        let clock = Arc::new(VirtualClock::new());
        let mut runtime =
            GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 8).expect("build");
        let mgr = runtime.test_transport().expect("test transport parked");
        let mut pub_a = mgr
            .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
            .expect("pub a");
        let mut pub_b = mgr
            .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
            .expect("pub b");
        publish_at(&mut pub_a, &clock, 0, 0.0);
        publish_at(&mut pub_a, &clock, 5 * MS, 5.0);
        publish_at(&mut pub_b, &clock, 10 * MS, 10.0);
        clock.set(0);
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(
        recorded(&batched),
        vec![(5.0, 10.0)],
        "all three arrived: the walk tightens to the nearer member"
    );

    // BATCHING 2 — a@5 lands AFTER the step that saw a@0 and b@10. The
    // alignment had nothing to descend to, so it fired immediately.
    let split: Sets = Arc::new(Mutex::new(Vec::new()));
    {
        let entry = Box::new(PairFuseUnboundedEntry::with_state(PairFuseUnbounded {
            sets: Some(Arc::clone(&split)),
            ..Default::default()
        }));
        let (config, factories) = pair_graph("pss_batch2", "pssb2", entry);
        let clock = Arc::new(VirtualClock::new());
        let mut runtime =
            GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 8).expect("build");
        let mgr = runtime.test_transport().expect("test transport parked");
        let mut pub_a = mgr
            .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
            .expect("pub a");
        let mut pub_b = mgr
            .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
            .expect("pub b");
        publish_at(&mut pub_a, &clock, 0, 0.0);
        publish_at(&mut pub_b, &clock, 10 * MS, 10.0);
        clock.set(0);
        runtime.step(Duration::from_millis(1));
        publish_at(&mut pub_a, &clock, 5 * MS, 5.0);
        clock.set(0);
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(
        recorded(&split),
        vec![(0.0, 10.0)],
        "split across two steps: the first alignment had no arrived successor \
         on a, so it fired immediately — and a@5 then waits for its own partner"
    );

    assert_ne!(
        recorded(&batched),
        recorded(&split),
        "the two batchings of ONE frame set deliver DIFFERENT memberships. \
         RULED (closest among ARRIVED, fire immediately) — a design that made \
         them agree would have to wait for frames that have not arrived"
    );
}

// ---------------------------------------------------------------------------
// ARM 14 — THE BORROW BUDGET IS 2
// ---------------------------------------------------------------------------

/// A full descent walk on a topic whose service is pinned at
/// `subscriber_max_borrowed_samples = 2`.
///
/// The whole two-slot design rests on the peak concurrent SHM borrow staying at
/// 2 — `frozen(1)` plus ONE of `{next_head, receive-transient}` — because the
/// pop-one drain moves its single transient straight into a slot rather than
/// holding a `latest` local beside it. If that were wrong, the feature would
/// need `Some(3)` on every Sync trigger topic, which is a provisioning change
/// AND a build-time refusal on any foreign service created at the iceoryx2
/// default.
///
/// So the arm pins the claim where it is observable: the service is created by
/// an EXTERNAL publisher at exactly 2, and the walk pops nine frames through
/// the staged slot. A peak above 2 surfaces as `ExceedsMaxBorrows` from
/// iceoryx2 itself — this cannot pass by accident.
#[test]
#[serial]
fn a_deep_descent_runs_on_a_service_pinned_at_two_borrowed_samples() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let entry = Box::new(PairFuseUnboundedEntry::with_state(PairFuseUnbounded {
        sets: Some(Arc::clone(&sets)),
        ..Default::default()
    }));
    let (config, factories) = pair_graph("pss_borrow", "pssbw", entry);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");

    let mut pinned: TopicServiceConfig = mgr.default_topic_config();
    pinned.subscriber_max_borrowed_samples = Some(2);
    let mut pub_a = mgr
        .create_publisher_with_topic_config(TOPIC_A, MaxSliceLen::const_new(64), 0, pinned)
        .expect("external publisher pinned at borrow 2");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    for i in 0..10u64 {
        publish_at(&mut pub_a, &clock, i * MS, i as f64);
    }
    publish_at(&mut pub_b, &clock, 100 * MS, 100.0);
    clock.set(0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        recorded(&sets),
        vec![(9.0, 100.0)],
        "the walk descends nine frames through the staged slot on a service \
         that allows only TWO borrowed samples — the peak-2 derivation, pinned \
         where iceoryx2 itself would object"
    );
    assert_eq!(
        runtime
            .node_handle("fuse")
            .expect("node handle")
            .sync_closer_skip_count(TOPIC_A),
        9,
        "every popped-past frame is accounted for"
    );
}

// ---------------------------------------------------------------------------
// CLOSURE-BACKED ARMS (10a and 17) — the shapes a MACRO node cannot produce
// ---------------------------------------------------------------------------
//
// The macro nests one `try_view` per input with the user body INNERMOST, so a
// macro node can neither panic before its reads nor read one input twice. Both
// arms below therefore REQUIRE a closure fixture — the memo declares the same
// requirement, and it is not a testing convenience: these are the two states
// the two-slot lifecycle has that the shipping macro path never enters.

/// A 2-trigger Sync `ClosureNodeEntry` whose body is supplied by the caller.
fn closure_sync_graph(
    prefix: &str,
    body: impl FnMut(&mut cerulion_core::NodeContext) -> TransportResult<()> + Send + 'static,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let meta = |name: &str| InputMeta {
        name: name.to_string(),
        schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
        trigger: true,
        depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
        backpressure: BackpressurePolicy::DropOldest,
        expect_within_ms: None,
    };
    let info = NodeInfo::with_meta(vec![meta("a"), meta("b")], vec![])
        .with_policy(MacroPolicy::Sync { window_ms: 500 });
    let entry = ClosureNodeEntry::new(info, body).with_label("pss_closure_fuse");

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("pss_closure_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "fuse".to_string(),
            node_type: "closure_fuse".to_string(),
            inputs: vec![
                InputDef {
                    name: "a".to_string(),
                    source: TOPIC_A.to_string(),
                },
                InputDef {
                    name: "b".to_string(),
                    source: TOPIC_B.to_string(),
                },
            ],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), Box::new(entry));
    (config, factories)
}

/// ARM 10a — PANIC x BURST, the NOT-TAKEN shape.
///
/// A tick that panics BEFORE reading any member leaves every frozen head
/// UNTAKEN. The refill alignment then finds a still-frozen head, that input
/// answers "nothing", the alignment is incomplete, and THE BURST ENDS — at most
/// ONE fire per step, with the members intact.
///
/// So the breaker takes three STEPS to open (not three fires inside one step,
/// which is the post-read shape's cadence), and `reset_node` resumes with set 1
/// still in hand. The two panic shapes have genuinely different guarantees and
/// this is the half where nothing is lost at all.
#[test]
#[serial]
fn a_pre_read_panic_ends_the_burst_and_loses_no_member() {
    let fires = Arc::new(AtomicU64::new(0));
    let fires_body = Arc::clone(&fires);
    let (config, factories) = closure_sync_graph("psspr", move |_ctx| {
        fires_body.fetch_add(1, Ordering::Relaxed);
        panic!("per-set Sync pre-read panic fixture");
    });
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build closure sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    for i in 1..=3u64 {
        publish_at(&mut pub_a, &clock, i * MS, i as f64);
        publish_at(&mut pub_b, &clock, i * MS + 100_000, (i * 10) as f64);
    }
    clock.set(0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        fires.load(Ordering::Relaxed),
        1,
        "the panicking tick never took its members, so the refill alignment is \
         incomplete and the burst ENDS — at most one fire per step, which is \
         what leaves the un-taken members intact"
    );
}

/// ARM 17 — THE SECOND READ SERVES THE STAGED NEXT (R-pop-prime).
///
/// `try_view`'s live arm is the THIRD queue-pop site, and the one an
/// implementation naturally forgets: after a fire has taken the head, a second
/// read on the same input in the same tick finds `frozen` empty and would
/// receive from the queue — handing back a frame NEWER than the one already
/// staged, and inverting that input's delivery order.
///
/// Setup: A=[0,60,100], B=[10]. The gate passes (B is scarce), the matcher pops
/// A@60 into the staged slot to learn its stamp and then REFUSES to advance
/// (span 50 is worse than 10). The tick fires on A@0: read 1 takes it; read 2
/// must serve the STAGED A@60 by promote-serve, never A@100 off the queue.
///
/// Oracle: the two reads are 0 then 60. An implementation that pops the queue
/// reads 0 then 100 — A delivering 0, 100, and 60 never, or last.
#[test]
#[serial]
fn a_second_read_in_one_tick_serves_the_staged_next_not_the_queue() {
    let reads: Arc<Mutex<Vec<f64>>> = Arc::new(Mutex::new(Vec::new()));
    let reads_body = Arc::clone(&reads);
    let (config, factories) = closure_sync_graph("psssr", move |ctx| {
        // TWO reads of `a` in ONE tick — legal, documented, and the only way
        // to reach the post-take state where a staged next is pending.
        for _ in 0..2 {
            if let Some(v) = ctx
                .subscriber_mut("a")
                .and_then(|s| s.try_view::<Vector3, _>(|view| view.x).ok().flatten())
            {
                reads_body.lock().expect("reads sink poisoned").push(v);
            }
        }
        // `b` is read once so the set is genuinely consumed.
        let _ = ctx
            .subscriber_mut("b")
            .and_then(|s| s.try_view::<Vector3, _>(|view| view.x).ok().flatten());
        Ok(())
    });
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build closure sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    for a in [0u64, 60, 100] {
        publish_at(&mut pub_a, &clock, a * MS, a as f64);
    }
    publish_at(&mut pub_b, &clock, 10 * MS, 10.0);
    clock.set(0);
    runtime.step(Duration::from_millis(1));

    let observed = reads.lock().expect("reads sink poisoned").clone();
    assert_eq!(
        observed,
        vec![0.0, 60.0],
        "read 2 must serve the STAGED A@60 by promotion. Popping the queue \
         instead yields 0 then 100 — the in-order inversion, with A@60 either \
         stranded or delivered out of turn"
    );
}

/// THE REFILL SITE IS NOT THE BOUNDARY SITE.
///
/// A boundary drain RE-OFFERS a head the tick never read — Principle #6, a
/// deferred fire or a collapsed read chain must not lose its signal. At a
/// REFILL that same state means the opposite: the fire that just ran did NOT
/// consume the head, so re-offering it fires the node AGAIN on frames it has
/// already been fired for.
///
/// The shape is a tick that reads NOTHING and returns Ok — the read
/// collapse, not a panic. That distinction is the whole reason this arm exists:
/// a panicking tick poisons the node mutex, so every subsequent align op fails
/// and the burst ends for an unrelated reason, which MASKS a refill that
/// re-offers. Only the clean-collapse shape can see it.
///
/// One set queued, one step. Correct: exactly ONE fire. A refill that re-offers
/// fires the same set over and over up to the per-step cap.
#[test]
#[serial]
fn a_refill_must_not_re_offer_a_head_the_tick_never_read() {
    let fires = Arc::new(AtomicU64::new(0));
    let fires_body = Arc::clone(&fires);
    let (config, factories) = closure_sync_graph("psscl", move |_ctx| {
        // Reads NOTHING: every frozen head survives the fire untaken.
        fires_body.fetch_add(1, Ordering::Relaxed);
        Ok(())
    });
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build closure sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    publish_at(&mut pub_a, &clock, MS, 1.0);
    publish_at(&mut pub_b, &clock, 2 * MS, 2.0);
    clock.set(0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        fires.load(Ordering::Relaxed),
        1,
        "the tick never took its members, so the refill must answer `nothing` \
         and END the burst. A refill that re-offers (boundary semantics at the \
         refill site) re-fires the SAME set up to the per-step cap"
    );

    // The members were never consumed, so the NEXT boundary re-offers them and
    // the node is owed exactly one more fire — nothing was lost.
    clock.set(0);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        fires.load(Ordering::Relaxed),
        2,
        "the boundary DOES re-offer an unread head — that is Principle #6, and \
         it is the same state the refill must treat oppositely"
    );
}

// ---------------------------------------------------------------------------
// ARM 15 — THE DEGRADE FORK: LOUD, AND IT NEVER WEDGES
// ---------------------------------------------------------------------------

/// The per-set graph, built for a node that does NOT implement the head ops.
///
/// `with_unified_drain(false)` is the closure spelling of the whole degrade
/// class: `ClosureNodeEntry` gates `supports_sync_head_ops` on that one flag
/// (its doc argues the three conditions really are one condition), so this is
/// the same fork a raw-FFI cdylib, a `from_names` closure and
/// `CERULION_DRAIN_DISCIPLINE=separate` all land in — reached WITHOUT mutating
/// process env, which `#[serial]` alone does not make safe to do here.
fn degraded_sync_graph(
    prefix: &str,
    body: impl FnMut(&mut cerulion_core::NodeContext) -> TransportResult<()> + Send + 'static,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let meta = |name: &str| InputMeta {
        name: name.to_string(),
        schema_hash: <Vector3 as ShmMessage>::SCHEMA_HASH,
        trigger: true,
        depth: cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
        backpressure: BackpressurePolicy::DropOldest,
        expect_within_ms: None,
    };
    let info = NodeInfo::with_meta(vec![meta("a"), meta("b")], vec![])
        .with_policy(MacroPolicy::Sync { window_ms: 50 });
    let entry = ClosureNodeEntry::new(info, body)
        .with_unified_drain(false)
        .with_label("pss_degraded_fuse");

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: format!("pss_degraded_{prefix}"),
        prefix: prefix.to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "fuse".to_string(),
            node_type: "degraded_fuse".to_string(),
            inputs: vec![
                InputDef {
                    name: "a".to_string(),
                    source: TOPIC_A.to_string(),
                },
                InputDef {
                    name: "b".to_string(),
                    source: TOPIC_B.to_string(),
                },
            ],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), Box::new(entry));
    (config, factories)
}

/// A Sync node WITHOUT the head ops keeps the earlier semantic, and the
/// whole point of that semantic is that it SELF-HEALS.
///
/// Two claims, and the second is the regression pin:
///
/// 1. The fork is LOUD — exactly ONE build warn naming the node and the fix.
///    Never a silent semantics fork: two Sync semantics ship until
///    every entry gains the ops, and an operator must be able to tell which one
///    their node is on.
/// 2. The fork never WEDGES. One out-of-window head pair is refused (correctly
///    — firing it would violate the declared window), and the five perfectly
///    aligned pairs that follow must each fire.
///
/// Claim 2 is where this arm earns its place. `align_sync_heads` returns
/// `Incomplete` immediately for a node with no ops, so NOTHING on this path can
/// execute a `DiscardTie` — the verdict reaches `decide_node`, which drops it.
/// Under fill-if-empty the refused pair is therefore never released: no fire, no
/// align pass, no retraction, and the node is silent FOREVER. MEASURED on the
/// pre-fix head with this exact stimulus: ZERO fires. Latest-wins is what makes
/// the next arrival overwrite the stale stamp, exactly as earlier Sync did.
///
/// The oracle is the value pair each fire READ, not a fire count — this file's
/// standing rule — and it is the SAME sequence the per-set path serves on the
/// same stimulus, so a degrade that fired the wrong members would still fail.
#[test]
#[serial]
#[traced_test]
fn a_node_without_the_head_ops_degrades_loudly_and_never_wedges() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let sink = Arc::clone(&sets);
    let (config, factories) = degraded_sync_graph("pssdgr", move |ctx| {
        // The degraded read is drain-to-LATEST — the freshest frame on each
        // trigger, which is exactly what the degrade warn promises.
        let a = ctx
            .subscriber_mut("a")
            .and_then(|s| s.try_view::<Vector3, _>(|v| v.x).ok().flatten());
        let b = ctx
            .subscriber_mut("b")
            .and_then(|s| s.try_view::<Vector3, _>(|v| v.x).ok().flatten());
        if let (Some(a), Some(b)) = (a, b) {
            sink.lock().expect("sets sink poisoned").push((a, b));
        }
        Ok(())
    });
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build degraded sync graph");

    // CLAIM 1 — loud, once, at the build.
    logs_assert(|lines: &[&str]| {
        let hits = lines
            .iter()
            .filter(|l| l.contains("DEGRADED to legacy latest-per-set"))
            .count();
        if hits == 1 {
            Ok(())
        } else {
            Err(format!(
                "the degrade fork must announce itself EXACTLY once per node; got {hits} line(s)"
            ))
        }
    });

    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    // A alone at t=0 — incomplete, so no fire and A's head is now held at 0.
    publish_at(&mut pub_a, &clock, 0, 1.0);
    runtime.step(Duration::from_millis(1));

    // B arrives 61 ms later: the pair spans 61 ms against a 50 ms window, so it
    // is provably in no set and must NOT fire. That refusal is CORRECT — and it
    // is also the state that used to be terminal.
    publish_at(&mut pub_b, &clock, 61 * MS, 1.0);
    runtime.step(Duration::from_millis(1));
    assert!(
        recorded(&sets).is_empty(),
        "an out-of-window pair must not fire — firing it would violate the \
         declared window (this is the PRECONDITION for the healing claim below)"
    );

    // CLAIM 2 — five fresh, perfectly aligned pairs. Every one must fire.
    for k in 2u64..=6 {
        let at = (100 + (k - 2) * 10) * MS;
        publish_at(&mut pub_a, &clock, at, k as f64);
        publish_at(&mut pub_b, &clock, at, k as f64);
        runtime.step(Duration::from_millis(1));
    }

    assert_eq!(
        recorded(&sets),
        vec![(2.0, 2.0), (3.0, 3.0), (4.0, 4.0), (5.0, 5.0), (6.0, 6.0)],
        "a degraded Sync node must SELF-HEAL after an unmatchable pair: the next \
         arrival on each input overwrites the stale head, exactly as \
         Sync did before per-set delivery. An empty sequence here is the permanent wedge"
    );
}

// ---------------------------------------------------------------------------
// THE WATCHDOG INVERSION — expect_within on a per-set Sync trigger
// ---------------------------------------------------------------------------

/// The 2-input fixture with an `expect_within_ms` watchdog on BOTH triggers.
///
/// The window is deliberately SHORT relative to the steps the arms drive, so a
/// stale anchor produces misses within a handful of steps rather than needing a
/// long run — the miss is a threshold, not a rate, so shortening it costs no
/// discrimination.
#[cerulion_node(sync_window_ms = 500)]
#[derive(Default)]
struct PairFuseWatched {
    #[input(trigger, expect_within_ms = 5)]
    a: Vector3,
    #[input(trigger, expect_within_ms = 5)]
    b: Vector3,
    sets: Option<Sets>,
}

#[cerulion_node_impl]
impl PairFuseWatched {
    fn tick(&mut self) -> Result<(), NodeError> {
        let pair = (self.a.x, self.b.x);
        if let Some(sink) = &self.sets {
            sink.lock().expect("sets sink poisoned").push(pair);
        }
        Ok(())
    }
}

/// The watched fixture WITH a producer rate cap, for the held-complete-set arm.
#[cerulion_node(sync_window_ms = 500, throttle_ms = 200)]
#[derive(Default)]
struct PairFuseWatchedThrottled {
    #[input(trigger, expect_within_ms = 5)]
    a: Vector3,
    #[input(trigger, expect_within_ms = 5)]
    b: Vector3,
    sets: Option<Sets>,
}

#[cerulion_node_impl]
impl PairFuseWatchedThrottled {
    fn tick(&mut self) -> Result<(), NodeError> {
        let pair = (self.a.x, self.b.x);
        if let Some(sink) = &self.sets {
            sink.lock().expect("sets sink poisoned").push(pair);
        }
        Ok(())
    }
}

fn watched_pair_graph(
    prefix: &str,
    sets: Sets,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let entry = Box::new(PairFuseWatchedEntry::with_state(PairFuseWatched {
        sets: Some(sets),
        ..Default::default()
    })) as Box<dyn NodeEntry>;
    pair_graph("pss_watched", prefix, entry)
}

/// Count `expect_within` MISS warns naming one input. The node-level counter
/// cannot answer this — `expect_within_missed` is ONE `AtomicU64` per NODE,
/// shared by every watched input — and "which input is late" is the entire
/// question, so the oracle reads the per-input diagnostic instead.
fn miss_warns_for(lines: &[&str], input: &str) -> usize {
    lines
        .iter()
        .filter(|l| {
            l.split_whitespace().any(|t| t == "WARN")
                && l.contains("input `expect_within_ms` exceeded")
                && l.split_whitespace().any(|t| t == format!("input={input}"))
        })
        .count()
}

/// A FLOWING per-set Sync trigger input must not be reported late because its
/// PARTNER starved — and the starved partner must still be reported.
///
/// # The inversion
///
/// A per-set Sync trigger anchors its `expect_within` watchdog when a frame is
/// POPPED. The align pass's PASS 1 leaves an already-`Filled` head alone (it is
/// already this set's member), so once `a`'s head is filled and `b` never
/// arrives, NOTHING pops on `a` and its anchor freezes — while frames keep
/// arriving on it. Pre-fix that made the HEALTHY input accrue
/// `expect_within_missed_count` and one `warn!` per window, naming the input
/// that is fine and saying nothing about the one that is not: the exact
/// inversion already fixed for the Data-backlog shape ("the knob is documented
/// as a producer-liveness detector"), reached by a different route.
///
/// # Why both legs live in ONE body
///
/// "`a` reports no misses" is satisfied by a watchdog that is simply broken.
/// The starved leg is the anti-tautology: the SAME run, the SAME window, must
/// still report `b`. And the suppression must be POSITIVE, not silence — the
/// disjoint `expect_within_backlogged_count` is the Principle #3 surface that
/// distinguishes "we suppressed N windows on purpose" from "the watchdog never
/// ran", so it is asserted too.
#[test]
#[serial]
#[traced_test]
fn a_flowing_sync_trigger_is_not_reported_late_when_its_partner_starves() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = watched_pair_graph("psswa", Arc::clone(&sets));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build watched per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");

    // `a` flows at one frame per step, 10 ms apart — twice the 5 ms window, so
    // a frozen anchor is guaranteed to lapse. `b` never publishes at all.
    const STEPS: u64 = 12;
    for k in 0..STEPS {
        publish_at(&mut pub_a, &clock, (k + 1) * 10 * MS, k as f64);
        runtime.step(Duration::from_millis(1));
    }

    assert!(
        recorded(&sets).is_empty(),
        "PRECONDITION: with `b` silent the set is never complete, so the node \
         never fires — which is what keeps `a`'s head filled and its anchor \
         from being refreshed by a fire"
    );

    let backlogged = runtime
        .node_handle("fuse")
        .expect("node handle")
        .expect_within_backlogged_count();

    logs_assert(move |lines: &[&str]| {
        let on_a = miss_warns_for(lines, "a");
        let on_b = miss_warns_for(lines, "b");
        if on_a != 0 {
            return Err(format!(
                "the FLOWING input must not be reported late while its partner \
                 starves — got {on_a} `expect_within_ms` exceeded warn(s) naming \
                 input=a. That is the inversion an earlier change closed: the knob is a \
                 PRODUCER-liveness detector and `a`'s producer never stopped"
            ));
        }
        if on_b == 0 {
            return Err(
                "ANTI-TAUTOLOGY: the STARVED input must still be reported — zero \
                 warns naming input=b would mean the watchdog was silenced \
                 rather than corrected"
                    .to_string(),
            );
        }
        Ok(())
    });

    assert!(
        backlogged > 0,
        "the suppression must be POSITIVE, not silence: every held-member window \
         is counted into the disjoint `expect_within_backlogged` bucket \
         (Principle #3), so an operator can tell 'suppressed on purpose' from \
         'the watchdog never ran'"
    );
}

/// The CONTROL: a healthy per-set Sync node neither MISSES nor SUPPRESSES.
///
/// Both halves are needed, and the second is the one an "it stopped warning"
/// fix would break. Arrivals keep each input's anchor fresh, so no window ever
/// lapses — which means the suppression branch must not be reached at all and
/// `expect_within_backlogged_count()` stays 0. A suppression that fired here
/// would say the node was holding members it could not consume, on a node that
/// fires every step.
///
/// It is also the anti-tautology for the starved-partner arm: "no misses on
/// `a`" is satisfied by a watchdog that never fires, and this arm is where a
/// broken watchdog would still have to show its own zero for the RIGHT reason
/// (nothing lapsed) rather than by suppression.
#[test]
#[serial]
#[traced_test]
fn two_flowing_sync_triggers_neither_miss_nor_suppress() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = watched_pair_graph("psswb", Arc::clone(&sets));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build watched per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    const STEPS: u64 = 12;
    for k in 0..STEPS {
        let at = (k + 1) * 10 * MS;
        publish_at(&mut pub_a, &clock, at, k as f64);
        publish_at(&mut pub_b, &clock, at, k as f64);
        runtime.step(Duration::from_millis(1));
    }

    let fires = recorded(&sets).len() as u64;
    assert!(
        fires >= STEPS - 1,
        "PRECONDITION: with both inputs flowing the node must actually be firing \
         — a run that never fired would satisfy every assertion below for the \
         wrong reason; got {fires} fires across {STEPS} steps"
    );

    let handle = runtime.node_handle("fuse").expect("node handle");
    assert_eq!(
        handle.expect_within_missed_count(),
        0,
        "a healthy per-set Sync node misses no deadline"
    );
    assert_eq!(
        handle.expect_within_backlogged_count(),
        0,
        "and it SUPPRESSES nothing either — the windows are checked while every \
         head is Filled, so a suppression keyed on that alone would disable the \
         knob for every per-set Sync node in the system"
    );

    logs_assert(|lines: &[&str]| {
        let noisy = miss_warns_for(lines, "a") + miss_warns_for(lines, "b");
        if noisy == 0 {
            Ok(())
        } else {
            Err(format!(
                "a healthy node must log no deadline warns; got {noisy}"
            ))
        }
    });
}

// ---------------------------------------------------------------------------
// ARM 16 — THE FIRST-FIRE DEFER IS DUE-NOW
// ---------------------------------------------------------------------------

/// The 2-input fixture with a producer rate cap, so a pre-fire gate can defer a
/// set the BOUNDARY already aligned.
#[cerulion_node(sync_window_ms = 500, throttle_ms = 50)]
#[derive(Default)]
struct PairFuseThrottled {
    #[input(trigger)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    sets: Option<Sets>,
}

#[cerulion_node_impl]
impl PairFuseThrottled {
    fn tick(&mut self) -> Result<(), NodeError> {
        let pair = (self.a.x, self.b.x);
        if let Some(sink) = &self.sets {
            sink.lock().expect("sets sink poisoned").push(pair);
        }
        Ok(())
    }
}

fn throttled_pair_graph(
    prefix: &str,
    sets: Sets,
) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let entry = Box::new(PairFuseThrottledEntry::with_state(PairFuseThrottled {
        sets: Some(sets),
        ..Default::default()
    })) as Box<dyn NodeEntry>;
    pair_graph("pss_throttled", prefix, entry)
}

/// A COMPLETE aligned set that a pre-fire gate deferred must report DUE-NOW.
///
/// This is the case a literal `refilled_unfired` mirror misses, and it is the
/// reason `sync_backlog_hint`'s setting rule is deliberately BROADER than the
/// Data twin's. `decide_node`'s `pre_fire_check` gate sits ABOVE
/// the policy match, so a deferred node reaches neither the Sync arm nor the
/// burst: a hint set only by the burst would be FALSE for a set the BOUNDARY
/// aligned. The node then owes a fire, reports nothing due, and the live loop
/// waits for an unrelated publish or the 250 ms liveliness cap where a Data
/// node recovers at the 1 ms floor.
///
/// The predecessor arm asserted `sync_backlog_pending()` around a set that
/// FIRED, so it never constructed an unfired one and the `false` it checked was
/// reachable with the hint permanently stuck low. This drives the real shape:
/// FIRE, then defer a second complete set inside the throttle window, then
/// release it.
///
/// NOTE on the name, after the wake-sizing change (PR-C C5): "due now" here is about the
/// HINT — `sync_backlog_pending()` still reports the set as OWED the moment the
/// gate defers it, and that is what this test primarily pins. What C5 changed is
/// the WAKE SIZING underneath: a set held by a RATE CAP is due at the cap's
/// deadline, not in one millisecond, so `ns_until_next_fire` now reports the
/// remaining window instead of zero. The two are deliberately separate — the set
/// is owed (hint true) AND the loop should sleep until it can actually fire — and
/// the assertions below pin both halves.
#[test]
#[serial]
fn a_set_the_throttle_deferred_before_fire_one_is_reported_due_now() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = throttled_pair_graph("psstha", Arc::clone(&sets));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build throttled per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    // SET 1 — fires: the throttle has no previous fire to measure against.
    publish_at(&mut pub_a, &clock, MS, 1.0);
    publish_at(&mut pub_b, &clock, MS, 1.0);
    clock.set(0);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        recorded(&sets),
        vec![(1.0, 1.0)],
        "the first set fires — nothing to throttle against yet"
    );
    assert!(
        !runtime
            .node_handle("fuse")
            .expect("node handle")
            .sync_backlog_pending(),
        "with the queues empty after the fire, nothing is owed"
    );

    // SET 2 — aligned by the BOUNDARY, then DEFERRED before fire 1 of its
    // burst: the step lands ~2 ms after the previous fire, well inside the
    // 50 ms cap.
    publish_at(&mut pub_a, &clock, 2 * MS, 2.0);
    publish_at(&mut pub_b, &clock, 2 * MS, 2.0);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        recorded(&sets),
        vec![(1.0, 1.0)],
        "PRECONDITION: the rate cap must really have deferred it — a second \
         fire here would mean this arm never constructed an unfired set at all"
    );
    assert!(
        runtime
            .node_handle("fuse")
            .expect("node handle")
            .sync_backlog_pending(),
        "a COMPLETE set sitting in the heads unfired is DUE NOW — this is the \
         first-fire defer the burst's own hint cannot see"
    );
    // The wake-sizing change (PR-C C5) CHANGED THIS CONTRACT, deliberately. This arm used
    // to require the timeout to drop to the 1 ms FLOOR, matching a Data node in
    // the same state. That was the defect: a set deferred by a rate cap is not
    // due NOW, it is due at the cap's DEADLINE, and reporting due-now made the
    // live loop poll at the 1 ms floor for the whole throttle window — ~50
    // wakeups here to discover 50 times that nothing may fire yet.
    //
    // `ns_until_next_fire` now returns `due.max(throttle_remaining)`, so the
    // wake is sized to the window that actually has to elapse. The set is still
    // OWED — `sync_backlog_pending()` above is unchanged and still true — which
    // is what keeps this from being "the hint went quiet".
    //
    // Pinned both ways: strictly ABOVE the floor (a floor value would mean the
    // deadline was ignored and the poll storm is back) and no MORE than the cap
    // (the wake must not be sized past the window, which would sleep through
    // the release). Data-backlog due-now is a DIFFERENT path and still reports
    // the floor — see the data arms in this file.
    let throttle_cap = Duration::from_millis(50);
    let sized = runtime.live_timeout_for_test();
    assert!(
        sized > Duration::from_millis(1),
        "a THROTTLE-deferred set is due at the DEADLINE, not now: the wake must be \
         sized to the remaining window, not the 1 ms floor (got {sized:?}). A floor \
         value is the earlier defect — polling every millisecond for the whole \
         {throttle_cap:?} cap."
    );
    assert!(
        sized <= throttle_cap,
        "and it must not be sized PAST the cap ({sized:?} > {throttle_cap:?}), which \
         would sleep through the release the deferred set is waiting for"
    );

    // Release the cap: the SAME set fires, with its own members.
    clock.set(200 * MS);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        recorded(&sets),
        vec![(1.0, 1.0), (2.0, 2.0)],
        "the deferred set is served, in order, with the members it was aligned \
         on — a defer never loses a member (memo §3.2's NOT-TAKEN column)"
    );
    assert!(
        !runtime
            .node_handle("fuse")
            .expect("node handle")
            .sync_backlog_pending(),
        "and nothing is owed once it has been served"
    );
}

// ---------------------------------------------------------------------------
// ARM 12 — A DEFERRED BACKLOG LOSES NOTHING ACROSS MANY POLLED STEPS
// ---------------------------------------------------------------------------

/// k complete sets queued behind a rate cap are ALL delivered, in order, across
/// a long polled run — the NOT-TAKEN row driven to its limit.
///
/// The oracle is the exact MEMBER SEQUENCE, hand-written, not a fire count: a
/// node that fired k times on the freshest members would satisfy a count and is
/// exactly the earlier behaviour. The rate cap is what keeps the sets from
/// collapsing into one step's burst, so the backlog genuinely spans steps.
#[test]
#[serial]
fn a_throttled_backlog_delivers_every_set_in_order_across_forty_polled_steps() {
    const SETS: u64 = 6;
    const STEPS: u64 = 40;

    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = throttled_pair_graph("psstb", Arc::clone(&sets));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build throttled per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    // Queue every set BEFORE the first step, so the whole backlog is arrived
    // and the only thing pacing its delivery is the node's own rate cap.
    for k in 1..=SETS {
        publish_at(&mut pub_a, &clock, k * MS, k as f64);
        publish_at(&mut pub_b, &clock, k * MS, k as f64);
    }

    // Each step advances the clock past the 50 ms cap, so exactly one set can
    // fire per step and the backlog is served over many steps rather than in
    // one burst.
    for step in 0..STEPS {
        clock.set((step + 1) * 100 * MS);
        runtime.step(Duration::from_millis(1));
    }

    let oracle: Vec<(f64, f64)> = (1..=SETS).map(|k| (k as f64, k as f64)).collect();
    assert_eq!(
        recorded(&sets),
        oracle,
        "every queued set must be delivered EXACTLY once, in arrival order, with \
         its own members — no loss, no duplication, no collapse onto the \
         freshest frame"
    );
}

// ---------------------------------------------------------------------------
// ARM 2's CONTROL — INTERIOR DEGENERACY AT k >= 3
// ---------------------------------------------------------------------------

/// With three inputs, a walking member that enters the INTERIOR of the span
/// stops there — at the first frame to enter it, the FIFO tie-break.
///
/// `cam@100`, `lidar@140`, `imu = [95, 105, 115]`, window 50 ms. The head tuple
/// `(100, 140, 95)` spans 45 ms, in-window, and `imu` is the argmin WITH
/// successors, so the gate passes on cam's scarcity and the walk really runs:
/// `95 -> 105` tightens the span 45 -> 40 and is taken, counting `imu@95` as
/// PASSED-OVER.
///
/// Then it stops, and WHY it stops is the interior-degeneracy rule made
/// structural: `imu@105` sits INSIDE `[100, 140]`, so `imu` is no longer the
/// argmin — cam is, and cam has no successor. The set fires on the first frame
/// that entered the interval. A matcher that kept walking the SAME input
/// instead of re-deriving the argmin each round would advance to `imu@115` and
/// fire `(100, 140, 115)` with 2 skips.
///
/// It is the counterpart of the signed-off 3-topic walk, where `imu` starts
/// below the pair and every step strictly tightens, so four frames are skipped.
/// Same fixture, same window, same input count — only where the walking input
/// sits relative to the others differs, and the two oracles must differ with it.
#[test]
#[serial]
fn an_interior_member_stops_the_walk_at_the_first_frame_to_enter_the_interval() {
    let sets: Triples = Arc::new(Mutex::new(Vec::new()));
    let (config, factories) = triple_graph(Arc::clone(&sets));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build triple per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_cam = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_lidar = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");
    let mut pub_imu = mgr
        .create_publisher(TOPIC_C, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/c");

    for t in [95u64, 105, 115] {
        publish_at(&mut pub_imu, &clock, t * MS, t as f64);
    }
    publish_at(&mut pub_cam, &clock, 100 * MS, 100.0);
    publish_at(&mut pub_lidar, &clock, 140 * MS, 140.0);

    clock.set(0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        sets.lock().expect("sets sink poisoned").clone(),
        vec![(100.0, 140.0, 105.0)],
        "the walk enters the interval and STOPS at the first frame that did — \
         `imu@105`, not `imu@115`. Once a member is interior it is no longer \
         the argmin, and the argmin is re-derived every round"
    );
    let handle = runtime.node_handle("fuse").expect("node handle");
    assert_eq!(
        handle.sync_closer_skip_count(TOPIC_C),
        1,
        "exactly ONE frame was passed over (imu@95) — the single strictly \
         improving step. `imu@115` is still queued for its own partners"
    );
    assert_eq!(
        handle.sync_unmatched_discard_count(TOPIC_C),
        0,
        "and nothing was unmatchable — every frame is inside the window"
    );
}

// ---------------------------------------------------------------------------
// ARM 4's TWIN — A DUPLICATE-STAMP PLATEAU
// ---------------------------------------------------------------------------

/// Two frames published inside ONE virtual-clock quantum share a wire stamp,
/// and that plateau STOPS the walk at the earliest of them — possibly SHORT of
/// the arrived global minimum. That is the tie-break, and the memo scopes
/// its own optimality claim to DISTINCT stamps because of exactly this shape.
///
/// `A = [0, 5, 5, 9]`, `B = [10]`, unbounded. B is scarce so the gate passes and
/// the walk descends A: `0 -> 5` strictly improves (span 10 -> 5), then `5 -> 5`
/// does NOT (span 5 -> 5), so strict descent refuses and fires `(5, 10)` — while
/// `(9, 10)`, span 1, was arrived and reachable in order.
///
/// The duplicate is REAL, not simulated: `publish_at` sets the clock, so
/// publishing twice at the same instant is the documented one-quantum shape
/// (`drain_for_trigger`'s own doc names it). A non-strict `<=` walks straight
/// through the plateau and fires `(9, 10)` with 3 skips, which is exactly what
/// this twin exists to catch on a stimulus the distinct-stamp arm cannot reach.
#[test]
#[serial]
fn a_duplicate_stamp_plateau_stops_the_walk_at_the_earliest_of_the_tied_frames() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let entry = Box::new(PairFuseUnboundedEntry::with_state(PairFuseUnbounded {
        sets: Some(Arc::clone(&sets)),
        ..Default::default()
    }));
    let (config, factories) = pair_graph("pss_plateau", "psspl", entry);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build unbounded per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    publish_at(&mut pub_a, &clock, 0, 0.0);
    // The plateau: two frames in ONE quantum, so both carry stamp 5 ms. Their
    // PAYLOADS differ (5.0 then 5.5) so the oracle can name WHICH of the tied
    // frames was served — the earliest, by the FIFO tie-break.
    publish_at(&mut pub_a, &clock, 5 * MS, 5.0);
    publish_at(&mut pub_a, &clock, 5 * MS, 5.5);
    publish_at(&mut pub_a, &clock, 9 * MS, 9.0);
    publish_at(&mut pub_b, &clock, 10 * MS, 10.0);

    clock.set(0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        recorded(&sets),
        vec![(5.0, 10.0)],
        "the walk stops AT the plateau and serves the EARLIEST tied frame — not \
         the arrived global minimum `(9, 10)`, which a non-strict descent would \
         reach. Strict descent IS the specified tie-break"
    );
    assert_eq!(
        runtime
            .node_handle("fuse")
            .expect("node handle")
            .sync_closer_skip_count(TOPIC_A),
        1,
        "exactly ONE frame was passed over (A@0); the second tied frame and A@9 \
         are still queued, un-skipped, waiting for their own partners"
    );
}

/// A node holding a COMPLETE set behind its own rate cap must not be reported
/// late on the inputs that delivered it.
///
/// This is the Data-backlog watchdog scenario with a Sync trigger instead of a Data one:
/// the frames arrived, the node is behind a gate IT declared, and counting the
/// lapsed windows as missed deadlines inverts a producer-liveness knob into a
/// report about the consumer's own throttle — at a defer longer than the
/// window, one `warn!` per window on a perfectly healthy graph.
///
/// It is also the arm that retired the `incomplete` conjunct the first version
/// of this fix carried. That conjunct suppressed only while the tuple was
/// INCOMPLETE, so a complete-but-deferred set — this shape — kept accruing
/// misses; and MEASURED, it defended nothing, because the suppression branch is
/// only reached once a window has already lapsed and a flowing input's window
/// never lapses.
#[test]
#[serial]
#[traced_test]
fn a_complete_set_held_behind_a_rate_cap_is_not_reported_late() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let entry = Box::new(PairFuseWatchedThrottledEntry::with_state(
        PairFuseWatchedThrottled {
            sets: Some(Arc::clone(&sets)),
            ..Default::default()
        },
    )) as Box<dyn NodeEntry>;
    let (config, factories) = pair_graph("pss_watched_throttled", "psswt", entry);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build watched+throttled per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    // Fire once so the rate cap has a previous fire to measure against.
    publish_at(&mut pub_a, &clock, MS, 1.0);
    publish_at(&mut pub_b, &clock, MS, 1.0);
    clock.set(0);
    runtime.step(Duration::from_millis(1));
    assert_eq!(recorded(&sets), vec![(1.0, 1.0)], "the first set fires");

    // A second COMPLETE set, then many steps inside the 200 ms cap, each
    // advancing the clock well past the 5 ms window.
    publish_at(&mut pub_a, &clock, 2 * MS, 2.0);
    publish_at(&mut pub_b, &clock, 2 * MS, 2.0);
    for k in 0..10u64 {
        clock.set(3 * MS + k * 10 * MS);
        runtime.step(Duration::from_millis(1));
    }

    assert_eq!(
        recorded(&sets),
        vec![(1.0, 1.0)],
        "PRECONDITION: the rate cap must really be holding the second set — a \
         fire here would mean this arm never constructed the held shape"
    );

    let handle = runtime.node_handle("fuse").expect("node handle");
    assert_eq!(
        handle.expect_within_missed_count(),
        0,
        "a node behind its OWN declared rate cap has missed no deadline: both \
         inputs delivered, and the frames are held as the set's members"
    );
    assert!(
        handle.expect_within_backlogged_count() > 0,
        "and the suppression is POSITIVE — every held-member window is counted \
         into the disjoint bucket, so this is distinguishable from a watchdog \
         that never ran"
    );

    logs_assert(|lines: &[&str]| {
        let noisy = miss_warns_for(lines, "a") + miss_warns_for(lines, "b");
        if noisy == 0 {
            Ok(())
        } else {
            Err(format!(
                "a rate-capped node must log no deadline warns on the inputs \
                 that delivered its held set; got {noisy}"
            ))
        }
    });
}

/// The watched fixture with a WIDE watchdog, for the promoted-member arm: the
/// window has to straddle two stamps that are ~11 ms apart, which the 5 ms
/// fixture cannot.
#[cerulion_node(sync_window_ms = 500)]
#[derive(Default)]
struct PairFuseWatchedWide {
    #[input(trigger, expect_within_ms = 30)]
    a: Vector3,
    #[input(trigger, expect_within_ms = 30)]
    b: Vector3,
    sets: Option<Sets>,
}

#[cerulion_node_impl]
impl PairFuseWatchedWide {
    fn tick(&mut self) -> Result<(), NodeError> {
        let pair = (self.a.x, self.b.x);
        if let Some(sink) = &self.sets {
            sink.lock().expect("sets sink poisoned").push(pair);
        }
        Ok(())
    }
}

/// A member that was STAGED by the descent and then PROMOTED is never reported
/// late — its watchdog was anchored when the descent POPPED it, not when the
/// set eventually read it.
///
/// This arm exists because that mechanism is easy to get wrong twice. The first
/// implementation of per-set Sync carried a fire-time re-anchor walk
/// (`note_sync_set_arrivals`) that shipped INERT — it looked heads up by
/// RESOLVED TOPIC in a map keyed by macro FIELD NAME — and the obvious repair
/// (fix the key space) was MEASURED to change nothing, because EVERY frame that
/// leaves the queue anchors on the delivered arm of the drain, the descent's
/// `NeedStamp` pop into the staged slot included. So the walk was DELETED, and
/// what holds the property now is the pop itself. This is the arm that says so.
///
/// The stimulus is the staged-next shape, which is the only way to separate
/// "anchored at the pop" from "anchored at the read":
///
/// * `A = [0, 60]`, `B = [10]`. Boundary 1's descent POPS A@60 to learn its
///   stamp (anchoring A at 60 right there), then REFUSES to advance — span 50
///   is worse than 10 — and the set fires `(0, 10)` with A@60 staged.
/// * Boundary 2 (`B@70`): A's member arrives by PROMOTION, and `(60, 70)` fires
///   WITHOUT any queue pop on A.
/// * The probe step then measures A's window: ~12 ms on it against a 30 ms
///   deadline. Anchored where A's last *fill* pop left it (A@0) it would be
///   ~72 ms — a miss, on an input whose member the node read one step ago.
///
/// A's queue is EMPTY after the promotion, which is load-bearing: a third frame
/// would refill and re-anchor A at boundary 3 and erase the distinction.
#[test]
#[serial]
#[traced_test]
fn a_promoted_member_anchors_its_own_watchdog() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let entry = Box::new(PairFuseWatchedWideEntry::with_state(PairFuseWatchedWide {
        sets: Some(Arc::clone(&sets)),
        ..Default::default()
    })) as Box<dyn NodeEntry>;
    let (config, factories) = pair_graph("pss_watched_wide", "psswwd", entry);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build watched-wide per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    publish_at(&mut pub_a, &clock, 0, 0.0);
    publish_at(&mut pub_a, &clock, 60 * MS, 60.0);
    publish_at(&mut pub_b, &clock, 10 * MS, 10.0);
    clock.set(0);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        recorded(&sets),
        vec![(0.0, 10.0)],
        "PRECONDITION: the descent must REFUSE and stage A@60 — a fire on \
         `(60, 10)` here would mean no promotion happens at all"
    );

    publish_at(&mut pub_b, &clock, 70 * MS, 70.0);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        recorded(&sets),
        vec![(0.0, 10.0), (60.0, 70.0)],
        "PRECONDITION: A's member arrives by PROMOTION — the staged frame, not \
         a queue pop, and not A@0 again"
    );

    // Probe: ~12 ms past the promoted member's stamp, ~72 ms past the frame the
    // last POP anchored on. The 30 ms deadline sits between them.
    clock.set(72 * MS);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        runtime
            .node_handle("fuse")
            .expect("node handle")
            .expect_within_missed_count(),
        0,
        "a member the node READ one step ago must not be reported late. If the \
         fire-time walk cannot find this input's watchdog, A's window is still \
         measured from A@0 and this reads as a deadline miss"
    );
    logs_assert(|lines: &[&str]| {
        let on_a = miss_warns_for(lines, "a");
        if on_a == 0 {
            Ok(())
        } else {
            Err(format!(
                "the promoted member's own stamp must anchor its window; got \
                 {on_a} warn(s) naming input=a"
            ))
        }
    });
}

/// A Sync trigger carrying BOTH a read policy and an arrival deadline.
///
/// `sample(40)` accepts at most one frame per 40 ms; `expect_within_ms = 10` is
/// the arrival deadline. `N > M` on purpose — that is the declaration where the
/// two knobs can contradict each other.
#[cerulion_node(sync_window_ms = 500)]
#[derive(Default)]
struct PairFuseSampled {
    #[input(trigger, backpressure = sample(40), expect_within_ms = 10)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    sets: Option<Sets>,
}

#[cerulion_node_impl]
impl PairFuseSampled {
    fn tick(&mut self) -> Result<(), NodeError> {
        let pair = (self.a.x, self.b.x);
        if let Some(sink) = &self.sets {
            sink.lock().expect("sets sink poisoned").push(pair);
        }
        Ok(())
    }
}

/// A DECIMATED arrival still resets a per-set Sync trigger's `expect_within`
/// watchdog.
///
/// The watchdog is a PRODUCER-liveness surface; `sample(N)` is a CONSUMER read
/// policy. A producer running at full rate into a decimating input is healthy,
/// and paging the operator about it inverts the knob. The earlier behaviour
/// reset only on ACCEPTED frames, so `sample(N) + expect_within_ms(M)` with
/// `N > M` tripped the watchdog throughout a perfectly healthy decimation
/// regime — while the SAME declaration on a DATA trigger did not, because that
/// input's Separate drain calls `signal_input_received` per raw arrival. A
/// user-visible semantic that differs by node POLICY is the silent fork this
/// repo refuses.
///
/// Three preconditions make the assertion mean something, and all are checked:
/// decimation really happened, the node really fired, and the run really
/// spanned many windows. Without them "zero misses" is satisfied by a graph
/// that never moved.
#[test]
#[serial]
#[traced_test]
fn a_decimated_arrival_still_resets_the_watchdog_on_a_per_set_sync_trigger() {
    const STEPS: u64 = 30;

    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let entry = Box::new(PairFuseSampledEntry::with_state(PairFuseSampled {
        sets: Some(Arc::clone(&sets)),
        ..Default::default()
    })) as Box<dyn NodeEntry>;
    let (config, factories) = pair_graph("pss_sampled", "psssm", entry);
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 16)
        .expect("build sampled per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    // A healthy producer at one frame per 5 ms — four times the 10 ms deadline's
    // demand, and well inside the 40 ms sample interval, so most arrivals are
    // decimated.
    for k in 0..STEPS {
        let at = k * 5 * MS;
        publish_at(&mut pub_a, &clock, at, k as f64);
        publish_at(&mut pub_b, &clock, at, k as f64);
        runtime.step(Duration::from_millis(1));
    }

    let handle = runtime.node_handle("fuse").expect("node handle");
    assert!(
        handle.backpressure_sampled_count("a") > 0,
        "PRECONDITION: frames must actually have been DECIMATED — with no \
         decimation this arm asserts nothing about the rule"
    );
    assert!(
        !recorded(&sets).is_empty(),
        "PRECONDITION: the node must actually have fired — a node whose set \
         never completes leaves `a`'s head unfilled and its window measured \
         from the anchor, which is the state under test, but a run with no \
         fires at all would mean the graph never moved"
    );

    assert_eq!(
        handle.expect_within_missed_count(),
        0,
        "a healthy producer in a decimation regime must not page anyone: the \
         watchdog is PRODUCER liveness, `sample(N)` is a CONSUMER read policy, \
         and every raw arrival — decimated included — is evidence the producer \
         is alive"
    );
    logs_assert(|lines: &[&str]| {
        let on_a = miss_warns_for(lines, "a");
        if on_a == 0 {
            Ok(())
        } else {
            Err(format!(
                "got {on_a} `expect_within_ms` exceeded warn(s) naming input=a \
                 during a healthy decimation regime — one per window, which is \
                 the log-flood class on a graph with nothing wrong"
            ))
        }
    });
}

/// The POSITIVE direction of `sync_backlog_pending()` — a complete aligned set
/// really is sitting here unfired, and the flag says so.
///
/// Added because its sibling `a_node_that_owes_nothing_reports_nothing_pending`
/// asserts only the FALSE direction, on a stimulus whose queues are empty by
/// the time it looks: an implementation that returned `false` unconditionally
/// passed it, and the accessor would have been inert with a green suite.
/// One arm cannot be trusted to pin a boolean if
/// it only ever drives one of its two answers.
///
/// The shape is a node-level rate cap. Two complete sets are queued before the
/// step; the burst serves the first, and the node's own `throttle_ms` then
/// defers the second — which stays ALIGNED in the heads, which is exactly the
/// state the flag exists to report. Stepping past the cap drains it.
#[test]
#[serial]
fn a_node_holding_an_aligned_set_it_cannot_yet_fire_reports_it_pending() {
    let sets: Sets = Arc::new(Mutex::new(Vec::new()));
    let entry = Box::new(ThrottledPairFuseEntry::with_state(ThrottledPairFuse {
        sets: Some(Arc::clone(&sets)),
        ..Default::default()
    }));
    let (mut config, mut factories) = pair_graph("pss_pending", "psspd", entry);
    config.nodes[0].node_type = "throttled_pair_fuse".to_string();
    let entry = factories.shift_remove("fuse").expect("the fixture entry");
    factories.insert("fuse".to_string(), entry);

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, Arc::clone(&clock), 8)
        .expect("build the throttled per-set sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("publisher on /pss/b");

    // TWO complete sets, both queued before the first step.
    for (a, b) in [(1u64, 2u64), (3, 4)] {
        publish_at(&mut pub_a, &clock, a * MS, a as f64);
        publish_at(&mut pub_b, &clock, b * MS, b as f64);
    }
    clock.set(0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        recorded(&sets),
        vec![(1.0, 2.0)],
        "the rate cap lets exactly ONE set through this step — without that the \
         burst serves both and there is no unfired set to report"
    );
    assert!(
        runtime
            .node_handle("fuse")
            .expect("node handle")
            .sync_backlog_pending(),
        "THE PIN: the second set is complete and aligned in the heads, and the \
         only reason it has not fired is the node's own cap. That is precisely \
         `sync_backlog_pending()`'s question, and a `false` here is what the \
         live loop reads as `nothing to come back for`"
    );

    // Past the cap: the held set fires and the flag clears.
    for _ in 0..60 {
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(
        recorded(&sets),
        vec![(1.0, 2.0), (3.0, 4.0)],
        "the deferred set is SERVED once the cap lifts — deferring is a WHEN, \
         never a loss"
    );
    assert!(
        !runtime
            .node_handle("fuse")
            .expect("node handle")
            .sync_backlog_pending(),
        "and with it served, nothing is owed — the flag really does move in \
         both directions"
    );
}
