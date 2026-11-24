// SPDX-License-Identifier: AGPL-3.0-only
//! The NODE-STOP trace record (kind 7, `trace_format` 12) at each site that
//! stops a node.
//!
//! Each arm records a small graph through `GraphRuntime::build_for_test` (or a
//! bare `Scheduler` for the consecutive-panic breaker, which a runtime node
//! cannot reach) with a REAL trace ring installed, drains the ring, and
//! compares the whole record stream against a HAND-WRITTEN oracle, never a
//! second run: a `Period(10)` node stepped at 10 ms fires once per step at
//! `(k + 1) * 10 ms`, a `Period(5)` node fires twice per step at `10k + 5` and
//! `10k + 10` ms, every node here has no inputs and so fires at level 0, and
//! every step opens with its STEP-BOUNDARY record. The stop sites:
//!
//! - a tick that returned `NodeError::Fatal` (the runtime's fatal arm);
//! - a tick that panicked inside the runtime's tick callback (its panic arm);
//! - an entry poisoned outside a tick, first seen by the callback's poisoned
//!   arm;
//! - the scheduler's consecutive-panic breaker;
//! - a stop inside a catch-up burst, on the serial and the parallel fire path.
//!
//! The oracles spell every record as a struct literal, the stop records
//! included (`record_type` 7, the cause code in `duration_ns`: 1 fatal error,
//! 2 entry poisoned, 3 disabled after consecutive panics).
//!
//! # SHM-touching: run it in the serial shared-memory test window
//!
//! `build_for_test` uses a per-test iceoryx2 SHM root and the ring is real
//! POSIX SHM with a pid-scoped name. `#[serial]` because the parallel arm sets
//! the process-global `CERULION_FIRE_THREADS`.

#![cfg(unix)]

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::scheduler::{NodeConfig, Scheduler, TriggerPolicy};
use cerulion_core::trace_ring::{TraceRingConsumer, TraceRingOwner, TraceRingRecord};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// 10 ms in nanoseconds.
const MS10: u64 = 10_000_000;

// ===========================================================================
// Node types
// ===========================================================================

/// A healthy `Period(10)` source.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct StopSrc {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl StopSrc {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        Ok(())
    }
}

/// A `Period(10)` source that returns `NodeError::Fatal` on its third tick
/// (step 2).
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct FatalAtThird {
    #[output]
    out: Vector3,
    ticks: u32,
}

#[cerulion_node_impl]
impl FatalAtThird {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.ticks += 1;
        if self.ticks == 3 {
            return Err(NodeError::Fatal("stops at its third tick".to_string()));
        }
        self.out.x = f64::from(self.ticks);
        Ok(())
    }
}

/// A `Period(10)` source whose third tick (step 2) panics.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct PanicAtThird {
    #[output]
    out: Vector3,
    ticks: u32,
}

#[cerulion_node_impl]
impl PanicAtThird {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.ticks += 1;
        if self.ticks == 3 {
            panic!("panics at its third tick");
        }
        self.out.x = f64::from(self.ticks);
        Ok(())
    }
}

/// A `Period(5)` source, two fires per 10 ms step, that returns
/// `NodeError::Fatal` on its fourth tick: the second fire of step 1.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct FastFatalAtFourth {
    #[output]
    out: Vector3,
    ticks: u32,
}

#[cerulion_node_impl]
impl FastFatalAtFourth {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.ticks += 1;
        if self.ticks == 4 {
            return Err(NodeError::Fatal("stops at its fourth tick".to_string()));
        }
        self.out.x = f64::from(self.ticks);
        Ok(())
    }
}

// ===========================================================================
// Harness
// ===========================================================================

/// RAII guard for the process-global `CERULION_FIRE_THREADS` (read once at
/// `build_for_test`).
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

fn node_def(id: &str, node_type: &str) -> NodeDef {
    NodeDef {
        isolate: None,
        params: Default::default(),
        ros2: None,
        id: id.to_string(),
        node_type: node_type.to_string(),
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

/// One node of a recorded graph: its id, its node type and its entry.
type Member = (&'static str, &'static str, Box<dyn NodeEntry>);

/// Record `members` for `steps` steps of 10 ms with a trace ring installed
/// before step 0, calling `between(step, runtime)` after each step, and return
/// every record the ring holds.
fn record(
    tag: &str,
    members: Vec<Member>,
    steps: u64,
    mut between: impl FnMut(u64, &mut GraphRuntime),
) -> Vec<TraceRingRecord> {
    let node_ids: Vec<String> = members.iter().map(|m| m.0.to_string()).collect();
    let id_refs: Vec<&str> = node_ids.iter().map(String::as_str).collect();
    let mut owner = TraceRingOwner::create(
        &format!("stop_{tag}_{}", std::process::id()),
        4096,
        0,
        &id_refs,
    )
    .expect("create trace ring");
    let ring_name = owner.name().to_string();
    let producer = owner.producer().expect("mint producer");

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "node_stop_record".to_string(),
        prefix: format!("nsr_{tag}"),
        nodes: members.iter().map(|m| node_def(m.0, m.1)).collect(),
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for (id, _, entry) in members {
        factories.insert(id.to_string(), entry);
    }
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build graph");
    runtime.set_trace_ring_producer(producer, &node_ids);
    for step in 0..steps {
        runtime.step(Duration::from_millis(10));
        between(step, &mut runtime);
    }
    let mut consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
    let mut records = Vec::new();
    consumer.drain(&mut records).expect("drain (no overrun)");
    drop(consumer);
    runtime.shutdown();
    drop(owner);
    records
}

fn boundary(step: u64) -> TraceRingRecord {
    TraceRingRecord {
        step,
        fire_time_ns: (step + 1) * MS10,
        duration_ns: 0,
        node_idx: 0,
        global_level: 0,
        record_type: 3,
        reserved: 0,
    }
}

fn fire(step: u64, fire_time_ns: u64, node_idx: u32) -> TraceRingRecord {
    TraceRingRecord {
        step,
        fire_time_ns,
        duration_ns: 0,
        node_idx,
        global_level: 0,
        record_type: 1,
        reserved: 0,
    }
}

/// The NODE-STOP record the fire at `step`, `fire_time_ns` of node
/// `node_idx` leaves with cause `code`.
fn stop(step: u64, fire_time_ns: u64, node_idx: u32, code: u64) -> TraceRingRecord {
    TraceRingRecord {
        step,
        fire_time_ns,
        duration_ns: code,
        node_idx,
        global_level: 0,
        record_type: 7,
        reserved: 0,
    }
}

/// The stop records of `records`, in stream order.
fn stops_of(records: &[TraceRingRecord]) -> Vec<TraceRingRecord> {
    records
        .iter()
        .filter(|r| r.record_type == 7)
        .copied()
        .collect()
}

// ===========================================================================
// The runtime's stop sites
// ===========================================================================

/// A tick that returns `NodeError::Fatal` at step 2 leaves one stop record
/// with the fatal code directly after that fire's FIRE record, and the node
/// never fires again.
#[test]
#[serial]
fn a_fatal_return_pushes_one_stop_record_directly_after_its_fire() {
    let records = record(
        "fatal",
        vec![("src", "fatal_at_third", Box::new(FatalAtThirdEntry::new()))],
        6,
        |_, _| {},
    );
    let oracle = vec![
        boundary(0),
        fire(0, MS10, 0),
        boundary(1),
        fire(1, 2 * MS10, 0),
        boundary(2),
        fire(2, 3 * MS10, 0),
        stop(2, 3 * MS10, 0, 1),
        boundary(3),
        boundary(4),
        boundary(5),
    ];
    assert_eq!(records, oracle);
}

/// A stop at the second fire of a two-fire burst sits after that fire, so the
/// node's FIRE records of the step before it count the stopping fire's
/// position (2), and the burst ends there.
#[test]
#[serial]
fn a_stop_inside_a_burst_sits_directly_after_the_fire_that_made_it() {
    let records = record(
        "burst",
        vec![(
            "fast",
            "fast_fatal_at_fourth",
            Box::new(FastFatalAtFourthEntry::new()),
        )],
        4,
        |_, _| {},
    );
    let oracle = vec![
        boundary(0),
        fire(0, 5_000_000, 0),
        fire(0, MS10, 0),
        boundary(1),
        fire(1, 15_000_000, 0),
        fire(1, 2 * MS10, 0),
        stop(1, 2 * MS10, 0, 1),
        boundary(2),
        boundary(3),
    ];
    assert_eq!(records, oracle);
}

/// A tick that panics at step 2 poisons the node's entry: one stop record
/// with the poisoned code follows that fire, and the node keeps firing (its
/// tick skipped) with no second stop record.
#[test]
#[serial]
fn a_tick_panic_pushes_one_stop_record_and_the_poisoned_node_keeps_firing() {
    let records = record(
        "panic",
        vec![("src", "panic_at_third", Box::new(PanicAtThirdEntry::new()))],
        6,
        |_, _| {},
    );
    let mut oracle = Vec::new();
    for k in 0..6 {
        oracle.push(boundary(k));
        oracle.push(fire(k, (k + 1) * MS10, 0));
        if k == 2 {
            oracle.push(stop(2, 3 * MS10, 0, 2));
        }
    }
    assert_eq!(records, oracle);
}

/// An entry poisoned OUTSIDE a tick, after step 1, is first seen by the tick
/// callback's poisoned arm at step 2's fire: that fire carries the stop with
/// the poisoned code, and the later fires carry none.
#[test]
#[serial]
fn an_entry_poisoned_outside_a_tick_stops_the_node_at_its_next_fire() {
    let records = record(
        "poisoned",
        vec![("src", "stop_src", Box::new(StopSrcEntry::new()))],
        5,
        |step, runtime| {
            if step == 1 {
                let entry = runtime.node_entry_arc_for_test("src").expect("node");
                let poisoned = std::thread::spawn(move || {
                    let _guard = entry.lock().expect("unpoisoned before this thread");
                    panic!("poison the entry outside a tick");
                })
                .join();
                assert!(poisoned.is_err(), "the helper thread panicked");
            }
        },
    );
    let mut oracle = Vec::new();
    for k in 0..5 {
        oracle.push(boundary(k));
        oracle.push(fire(k, (k + 1) * MS10, 0));
        if k == 2 {
            oracle.push(stop(2, 3 * MS10, 0, 2));
        }
    }
    assert_eq!(records, oracle);
}

/// The PARALLEL fire path: sixteen `Period(10)` sources and the `Period(5)`
/// node that stops at its fourth tick fire as one 17-node level under four
/// fire threads, and the merge pass keeps the stop record directly after the
/// stopping fire (the fast node's second fire of step 1, manifest index 16).
#[test]
#[serial]
fn the_parallel_merge_keeps_a_stop_record_directly_after_its_fire() {
    const SOURCES: &[&str] = &[
        "p00", "p01", "p02", "p03", "p04", "p05", "p06", "p07", "p08", "p09", "p10", "p11", "p12",
        "p13", "p14", "p15",
    ];
    let _threads = FireThreadsGuard::set("4");
    let mut members: Vec<Member> = SOURCES
        .iter()
        .map(|id| {
            (
                *id,
                "stop_src",
                Box::new(StopSrcEntry::new()) as Box<dyn NodeEntry>,
            )
        })
        .collect();
    members.push((
        "fast",
        "fast_fatal_at_fourth",
        Box::new(FastFatalAtFourthEntry::new()),
    ));
    // The level is 17 nodes wide (it fires 18 times at steps 0 and 1, the
    // fast node twice), at or past the node count at which a level fires on
    // the parallel path.
    assert!(
        SOURCES.len() + 1 >= GraphRuntime::parallel_fire_threshold(),
        "the level is wide enough to take the parallel path"
    );
    let records = record("parallel", members, 3, |_, _| {});
    let mut oracle = Vec::new();
    for k in 0..3u64 {
        oracle.push(boundary(k));
        for i in 0..16u32 {
            oracle.push(fire(k, (k + 1) * MS10, i));
        }
        if k < 2 {
            oracle.push(fire(k, k * MS10 + 5_000_000, 16));
            oracle.push(fire(k, (k + 1) * MS10, 16));
        }
        if k == 1 {
            oracle.push(stop(1, 2 * MS10, 16, 1));
        }
    }
    assert_eq!(records, oracle);
}

// ===========================================================================
// The scheduler's consecutive-panic breaker
// ===========================================================================

/// The breaker is reachable on a bare scheduler only (a runtime node's first
/// panic poisons its entry): a callback that panics on every fire opens it at
/// its third fire (step 2), which leaves one stop record with the breaker's
/// code, and the node fires no more.
#[test]
#[serial]
fn the_panic_breaker_pushes_one_stop_record_directly_after_its_fire() {
    let mut owner = TraceRingOwner::create(
        &format!("stop_breaker_{}", std::process::id()),
        64,
        0,
        &["n0"],
    )
    .expect("create trace ring");
    let ring_name = owner.name().to_string();
    let producer = owner.producer().expect("mint producer");
    let mut scheduler = Scheduler::with_virtual_clock(Arc::new(VirtualClock::new()));
    scheduler.set_trace_ring_producer(producer, &["n0".to_string()]);
    scheduler
        .add_node(NodeConfig {
            id: "n0".to_string(),
            policy: TriggerPolicy::Period {
                interval: Duration::from_millis(10),
                max_catchup: None,
            },
            callback: Box::new(|| panic!("panics on every fire")),
        })
        .unwrap();
    for _ in 0..5 {
        scheduler.step_ms(10);
    }
    let mut consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
    let mut records = Vec::new();
    consumer.drain(&mut records).expect("drain (no overrun)");
    drop(consumer);
    drop(owner);
    let oracle = vec![
        boundary(0),
        fire(0, MS10, 0),
        boundary(1),
        fire(1, 2 * MS10, 0),
        boundary(2),
        fire(2, 3 * MS10, 0),
        stop(2, 3 * MS10, 0, 3),
        boundary(3),
        boundary(4),
    ];
    assert_eq!(records, oracle);
}

// ===========================================================================
// Determinism
// ===========================================================================

/// Two recordings of one run carry identical stop records, and both equal the
/// hand oracle: the fatal node (index 0) stops at step 2's fire and the
/// panicking node (index 1) at step 2's fire, in manifest order within the
/// level.
#[test]
#[serial]
fn two_recordings_of_one_run_carry_identical_stop_records() {
    let run = |tag: &str| {
        stops_of(&record(
            tag,
            vec![
                (
                    "fatal",
                    "fatal_at_third",
                    Box::new(FatalAtThirdEntry::new()),
                ),
                (
                    "panic",
                    "panic_at_third",
                    Box::new(PanicAtThirdEntry::new()),
                ),
            ],
            5,
            |_, _| {},
        ))
    };
    let first = run("det_a");
    let second = run("det_b");
    let oracle = vec![stop(2, 3 * MS10, 0, 1), stop(2, 3 * MS10, 1, 2)];
    assert_eq!(first, oracle);
    assert_eq!(second, first);
}
