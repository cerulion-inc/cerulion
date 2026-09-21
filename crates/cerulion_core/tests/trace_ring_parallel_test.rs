// SPDX-License-Identifier: AGPL-3.0-only
//! The trace-ring arm for the scheduler's PARALLEL merge pass.
//!
//! `GraphRuntime::step` routes every ≥2-fire level through
//! `Scheduler::tick_decided_parallel`, whose fragment merge (its pass 3) funnels into
//! `RingTraceSink::push_entry` — the SAME choke point the serial fire path
//! uses, and therefore the trace-ring push site for ALL multi-fire levels.
//! `trace_ring_hook_test.rs` covers the flat serial path only; THIS file pins
//! the parallel arm with a recording hook installed, against HAND oracles:
//!
//! - **wide** (16 same-`Period(10)` sources + one `Period(5)` catch-up node =
//!   one 17-fire level 0, COMFORTABLY ≥ `PARALLEL_FIRE_THRESHOLD` (8) → the
//!   rayon `par_values_mut` path under a forced `CERULION_FIRE_THREADS=4`;
//!   16 ≥ 2×4 so parallelism is genuine by construction — the
//!   `rayon_fire_iox2_test` contract: byte-identity + forced threads, no
//!   thread probe): the ring receives ALL fires each step, byte-equal to the
//!   hand oracle AND 1:1 with the in-memory trace, with every record's
//!   `node_idx` resolving through the ring MANIFEST to the node that fired.
//!   The `Period(5)` node stepped at 10 ms fires a 2-record CATCH-UP BURST
//!   each step: both records must reach the ring in order
//!   through the node's fragment drain in the merge pass.
//! - **narrow** (2-node level < threshold → the serial-REST arm of
//!   `tick_decided_parallel`, still merged by the same merge pass): same assertions.
//! - **completeness through the merge pass** (the ungated ring push, on the parallel path): with
//!   `set_trace_limit` SMALLER than the total fire count, the in-memory trace
//!   caps at the newest `limit` while the ring carries the FULL sequence —
//!   the ring push is ungated by the cap at the merge site too.
//!
//! SPSC stays intact throughout: rayon workers fire into per-node
//! `trace_fragment`s; the merge pass (and thus the producer) runs on the
//! step-calling thread only.
//!
//! # SHM-touching: run it in the serial shared-memory test window
//!
//! `build_for_test` uses a per-test iceoryx2 SHM root and the ring is real
//! POSIX SHM. `#[serial]` because `CERULION_FIRE_THREADS` is
//! process-global (`FireThreadsGuard` mirrors `rayon_fire_iox2_test`).

#![cfg(unix)]

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, NodeDef, OutputDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::scheduler::TraceEntry;
use cerulion_core::trace_ring::{
    TraceRingConsumer, TraceRingOwner, TraceRingRecord, RECORD_TYPE_FIRE, RECORD_TYPE_STEP_BOUNDARY,
};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// RAII guard for the process-global `CERULION_FIRE_THREADS` (read ONCE at
/// `build_for_test`). Mirrors `rayon_fire_iox2_test::FireThreadsGuard`.
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

/// A `Period(10)` source publishing into `Vector3.x` — one macro TYPE reused
/// across every node INSTANCE of the level (the `rayon_fire_iox2_test`
/// wide-level pattern). The payload is irrelevant here; the trace is the
/// subject.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct RingSrc {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl RingSrc {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        Ok(())
    }
}

/// The catch-up twin: `Period(5)` stepped at 10 ms fires TWICE per step (the
/// arithmetic burst reconstruction in `tick_node_into`) — both records must
/// flow through the node's ONE fragment drain (the merge pass) into the ring, in order.
#[cerulion_node(period_ms = 5)]
#[derive(Default)]
struct RingSrcFast {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl RingSrcFast {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 1.0;
        Ok(())
    }
}

fn src_def(id: &str) -> NodeDef {
    NodeDef {
        ros2: None,
        id: id.to_string(),
        node_type: "ring_src".to_string(),
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

fn ids(n: usize) -> Vec<String> {
    (0..n).map(|i| format!("p{i:02}")).collect()
}

/// Build an `n`-wide all-`Period(10)` level (no inputs → no DAG edges → ONE
/// level 0), install a fresh trace ring BEFORE step 0, drive `steps` steps of
/// 10 ms, and return `(ring records, ring manifest, in-memory trace)`.
fn run_level_with_ring(
    tag: &str,
    node_ids: &[String],
    threads: &str,
    steps: u64,
    trace_limit: Option<usize>,
) -> (Vec<TraceRingRecord>, Vec<String>, Vec<TraceEntry>) {
    // Env must be set BEFORE build_for_test reads it (once, at build).
    let _guard = FireThreadsGuard::set(threads);

    let id_refs: Vec<&str> = node_ids.iter().map(String::as_str).collect();
    let mut ring_owner = TraceRingOwner::create(
        &format!("par_{}_{}", tag, std::process::id()),
        4096,
        0,
        &id_refs,
    )
    .expect("create trace ring");
    let ring_name = ring_owner.name().to_string();
    let producer = ring_owner.producer().expect("mint producer");

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "trace_ring_parallel".to_string(),
        prefix: format!("trp_{tag}"),
        nodes: node_ids
            .iter()
            .map(|id| {
                // The literal id "fast" is the Period(5) catch-up node; every
                // other id is a plain Period(10) source.
                if id == "fast" {
                    let mut def = src_def(id);
                    def.node_type = "ring_src_fast".to_string();
                    def
                } else {
                    src_def(id)
                }
            })
            .collect(),
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for id in node_ids {
        if id == "fast" {
            factories.insert(id.clone(), Box::new(RingSrcFastEntry::new()));
        } else {
            factories.insert(id.clone(), Box::new(RingSrcEntry::new()));
        }
    }

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build parallel ring graph");
    if let Some(limit) = trace_limit {
        runtime.set_trace_limit(limit);
    }
    runtime.set_trace_ring_producer(producer, node_ids);

    for _ in 0..steps {
        runtime.step(Duration::from_millis(10));
    }
    let trace = runtime.trace().to_vec();

    let mut consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
    let manifest = consumer.node_ids().to_vec();
    let mut records = Vec::new();
    consumer.drain(&mut records).expect("drain (no overrun)");
    drop(consumer);
    drop(ring_owner);
    (records, manifest, trace)
}

/// One StepBoundary record (contract addendum): pushed at `begin_step` BEFORE
/// the step's fires; `fire_time_ns` = the step's advanced clock.
fn boundary_record(step: u64, advanced_ns: u64) -> TraceRingRecord {
    TraceRingRecord {
        step,
        fire_time_ns: advanced_ns,
        duration_ns: 0,
        node_idx: 0,
        global_level: 0,
        record_type: RECORD_TYPE_STEP_BOUNDARY,
        reserved: 0,
    }
}

/// The hand oracle: each step opens with its StepBoundary (contract addendum),
/// then every node fires once (same period, stepped at the period), in
/// insertion (pos/decision) order, at `fire_time = (k+1) * 10 ms`, level 0,
/// FIRE kind.
fn oracle_records(n_nodes: u32, steps: u64) -> Vec<TraceRingRecord> {
    let mut out = Vec::new();
    for k in 0..steps {
        out.push(boundary_record(k, (k + 1) * 10_000_000));
        for i in 0..n_nodes {
            out.push(TraceRingRecord {
                step: k,
                fire_time_ns: (k + 1) * 10_000_000,
                duration_ns: 0,
                node_idx: i,
                global_level: 0,
                record_type: RECORD_TYPE_FIRE,
                reserved: 0,
            });
        }
    }
    out
}

/// The wide+catch-up hand oracle: `n_regular` `Period(10)` sources fire once
/// per step at `(k+1)*10 ms` in insertion order, then the trailing `Period(5)`
/// "fast" node (manifest index `n_regular`) fires its 2-record catch-up burst
/// at `(10k+5) ms` and `(10k+10) ms` — consecutive, in time order, through the
/// node's single fragment drain in the merge pass.
fn oracle_wide_fast(n_regular: u32, steps: u64) -> Vec<TraceRingRecord> {
    let mut out = Vec::new();
    for k in 0..steps {
        // Contract addendum: the step's boundary precedes every fire.
        out.push(boundary_record(k, (k + 1) * 10_000_000));
        for i in 0..n_regular {
            out.push(TraceRingRecord {
                step: k,
                fire_time_ns: (k + 1) * 10_000_000,
                duration_ns: 0,
                node_idx: i,
                global_level: 0,
                record_type: RECORD_TYPE_FIRE,
                reserved: 0,
            });
        }
        for burst in [5_000_000u64, 10_000_000] {
            out.push(TraceRingRecord {
                step: k,
                fire_time_ns: k * 10_000_000 + burst,
                duration_ns: 0,
                node_idx: n_regular,
                global_level: 0,
                record_type: RECORD_TYPE_FIRE,
                reserved: 0,
            });
        }
    }
    out
}

/// Assert the ring records resolve 1:1 against the in-memory trace: same
/// length, and index-by-index the manifest-resolved node id + step +
/// fire_time + level all agree (the node_idx↔manifest↔trace consistency).
fn assert_ring_matches_trace(
    records: &[TraceRingRecord],
    manifest: &[String],
    trace: &[TraceEntry],
) {
    // Boundaries are RING-ONLY (contract addendum) — the in-memory trace 1:1
    // holds for the FIRE subsequence; no other kinds may appear.
    let from_ring: Vec<(String, u64, u64, u32)> = records
        .iter()
        .filter(|r| r.record_type != RECORD_TYPE_STEP_BOUNDARY)
        .map(|r| {
            assert_eq!(
                r.record_type, RECORD_TYPE_FIRE,
                "only FIRE + boundary kinds"
            );
            assert_eq!(r.reserved, 0, "reserved always 0");
            let name = manifest
                .get(r.node_idx as usize)
                .unwrap_or_else(|| panic!("node_idx {} out of manifest range", r.node_idx))
                .clone();
            (name, r.step, r.fire_time_ns, r.global_level)
        })
        .collect();
    let from_trace: Vec<(String, u64, u64, u32)> = trace
        .iter()
        .map(|e| {
            (
                e.node_id.to_string(),
                e.step,
                e.fire_time_ns,
                e.global_level as u32,
            )
        })
        .collect();
    assert_eq!(
        from_ring, from_trace,
        "ring records must match the in-memory trace 1:1 through the manifest"
    );
}

/// (a) WIDE: 16 `Period(10)` sources + the `Period(5)` catch-up node = a
/// 17-fire level under 4 forced fire threads — the rayon `par_values_mut`
/// arm. The merge pass must push EVERY fire to the ring each step,
/// byte-equal to the hand oracle and 1:1 with the in-memory trace — including
/// the catch-up node's 2-record burst, in order, through its one fragment
/// drain.
///
/// N = 16 is DELIBERATELY ≫ `PARALLEL_FIRE_THRESHOLD` (8): the routing gate
/// is `rest_fire_count < THRESHOLD → serial`, so an exactly-at-threshold
/// level (the old N=8) rode the boundary — one off-by-one (`<` → `<=`) would
/// silently flip it to the narrow arm and this test would still pass
/// byte-identically. The 2× margin (the `rayon_fire_iox2_test` precedent:
/// byte-identity + forced threads, wideness by construction — 16 ≥ 2×4) makes
/// a silent arm-flip require an 8→17 threshold change, not an off-by-one.
#[test]
#[serial]
fn wide_rayon_level_pushes_all_fires_through_pass3_merge() {
    const N: usize = 16;
    const STEPS: u64 = 5;
    let mut node_ids = ids(N);
    node_ids.push("fast".to_string());
    let (records, manifest, trace) = run_level_with_ring("wide", &node_ids, "4", STEPS, None);

    assert_eq!(manifest, node_ids, "manifest == the handed node-id table");
    assert_eq!(
        records,
        oracle_wide_fast(N as u32, STEPS),
        "the wide rayon level's ring records match the hand oracle \
         (all fires, insertion order per step; the Period(5) node's catch-up \
         burst lands as 2 consecutive in-order records)"
    );
    assert_ring_matches_trace(&records, &manifest, &trace);

    // The addendum's ACCEPTANCE pins, asserted explicitly (they survive an
    // oracle refactor):
    // (1) the boundary stream is STRICTLY increasing in fire_time_ns;
    // (2) every FIRE record's step has a boundary whose fire_time_ns >= the
    //     fire's fire_time_ns. The catch-up node is WHY this matters: its
    //     burst fires stamp the reconstructed PERIOD boundaries (10k+5 ms) —
    //     BELOW the step's advanced clock (10k+10 ms) — so replay needs the
    //     step boundary to know how far the clock actually progressed.
    let boundaries: Vec<&TraceRingRecord> = records
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_STEP_BOUNDARY)
        .collect();
    assert_eq!(boundaries.len() as u64, STEPS, "one boundary per step");
    for w in boundaries.windows(2) {
        assert!(
            w[0].fire_time_ns < w[1].fire_time_ns,
            "boundary stream must be strictly increasing in fire_time_ns"
        );
    }
    for f in records.iter().filter(|r| r.record_type == RECORD_TYPE_FIRE) {
        let b = boundaries
            .iter()
            .find(|b| b.step == f.step)
            .unwrap_or_else(|| panic!("fire at step {} has no boundary", f.step));
        assert!(
            b.fire_time_ns >= f.fire_time_ns,
            "step {} boundary ({} ns) must be >= its fire's fire_time ({} ns) — \
             catch-up fires stamp period boundaries below the advanced clock",
            f.step,
            b.fire_time_ns,
            f.fire_time_ns
        );
    }
}

/// (b) NARROW: a 2-fire level < threshold — the serial-REST arm of
/// `tick_decided_parallel`, still funneled through the SAME merge pass.
/// Same assertions as the wide arm.
#[test]
#[serial]
fn narrow_two_fire_level_pushes_all_fires_through_pass3_merge() {
    const N: usize = 2;
    const STEPS: u64 = 5;
    let node_ids = ids(N);
    let (records, manifest, trace) = run_level_with_ring("narrow", &node_ids, "4", STEPS, None);

    assert_eq!(manifest, node_ids, "manifest == the handed node-id table");
    assert_eq!(
        records,
        oracle_records(N as u32, STEPS),
        "the narrow (serial-REST) level's ring records match the hand oracle"
    );
    assert_ring_matches_trace(&records, &manifest, &trace);
}

/// (c) Completeness THROUGH the merge pass (the ungated ring push, on the parallel path): with an
/// in-memory cap smaller than the total fire count, the ring still receives
/// ALL fires (the merge-site ring push is ungated by `max_trace_entries`)
/// while the in-memory trace keeps only the newest `CAP`.
#[test]
#[serial]
fn parallel_path_ring_is_complete_when_in_memory_trace_is_capped() {
    // 16 ≫ THRESHOLD(8): same 2× wide margin as the wide test above, so an
    // off-by-one in the routing gate cannot silently flip this arm to narrow.
    const N: usize = 16;
    const STEPS: u64 = 5;
    const CAP: usize = 5;
    let node_ids = ids(N);
    let (records, manifest, trace) =
        run_level_with_ring("capped", &node_ids, "4", STEPS, Some(CAP));

    // The ring: ALL 40 fires, complete.
    let full_oracle = oracle_records(N as u32, STEPS);
    assert_eq!(
        records, full_oracle,
        "the ring receives the FULL fire sequence through the fragment merge, \
         ungated by the in-memory cap"
    );

    // The in-memory trace: exactly the NEWEST `CAP` FIRES of that sequence
    // (boundaries are ring-only and never enter the in-memory trace, so the
    // newest-CAP window is taken over the FIRE subsequence).
    assert_eq!(trace.len(), CAP, "in-memory trace capped to the limit");
    let oracle_fires: Vec<&TraceRingRecord> = full_oracle
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_FIRE)
        .collect();
    let newest: Vec<(String, u64, u64, u32)> = oracle_fires[oracle_fires.len() - CAP..]
        .iter()
        .map(|r| {
            (
                manifest[r.node_idx as usize].clone(),
                r.step,
                r.fire_time_ns,
                r.global_level,
            )
        })
        .collect();
    let in_memory: Vec<(String, u64, u64, u32)> = trace
        .iter()
        .map(|e| {
            (
                e.node_id.to_string(),
                e.step,
                e.fire_time_ns,
                e.global_level as u32,
            )
        })
        .collect();
    assert_eq!(
        in_memory, newest,
        "the in-memory trace keeps exactly the newest {CAP} fires"
    );
}

/// (d) The `GraphRuntime::trace_ring_unmapped_count` PASSTHROUGH: a
/// producer whose handed `node_ids` table omits one node — installed
/// through the RUNTIME's `set_trace_ring_producer` — skips exactly that
/// node's fires and the passthrough reports the exact skip count, while the
/// mapped node's records stay complete (hand oracle). This is the runtime pin
/// the CLI's run-exit desync ERROR references: `run_graph_recording` derives
/// the manifest and the handed table from ONE `node_ids` source, so the
/// desync is unreachable-by-construction there — the CLI arm is
/// defense-in-depth, pinned here at the layer where it CAN be constructed.
#[test]
#[serial]
fn runtime_passthrough_reports_exact_unmapped_skip_count() {
    const STEPS: u64 = 4;
    let _guard = FireThreadsGuard::set("4");

    // Graph has TWO nodes; the ring manifest AND the handed table cover only
    // the first — p01's fires are the injected desync.
    let node_ids = ids(2);
    let handed = vec![node_ids[0].clone()];
    let mut ring_owner = TraceRingOwner::create(
        &format!("par_unmap_{}", std::process::id()),
        256,
        0,
        &[handed[0].as_str()],
    )
    .expect("create ring");
    let ring_name = ring_owner.name().to_string();
    let producer = ring_owner.producer().expect("mint producer");

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "trace_ring_unmapped_rt".to_string(),
        prefix: "trp_unmap".to_string(),
        nodes: node_ids.iter().map(|id| src_def(id)).collect(),
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    for id in &node_ids {
        factories.insert(id.clone(), Box::new(RingSrcEntry::new()));
    }
    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build unmapped-passthrough graph");
    runtime.set_trace_ring_producer(producer, &handed);

    for _ in 0..STEPS {
        runtime.step(Duration::from_millis(10));
    }

    // The passthrough reports EXACTLY the omitted node's fire count.
    assert_eq!(
        runtime.trace_ring_unmapped_count(),
        STEPS,
        "the runtime passthrough must report the exact number of skipped \
         (unmapped) fire records"
    );

    // The mapped node's records are complete and untouched by the desync.
    let mut consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
    let mut records = Vec::new();
    consumer.drain(&mut records).expect("drain (no overrun)");
    assert_eq!(
        records,
        oracle_records(1, STEPS),
        "the mapped node's records stay complete; unmapped fires are skipped"
    );
    drop(consumer);
    drop(ring_owner);
}

// ---------------------------------------------------------------------------
// (e) One boundary per STEP on a multi-LEVEL DAG
// ---------------------------------------------------------------------------

/// The relay of the 3-level chain: data-trigger in, forward out.
#[cerulion_node]
#[derive(Default)]
struct RingRelay {
    #[input(trigger)]
    inp: Vector3,
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl RingRelay {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = self.inp.x;
        Ok(())
    }
}

/// The sink of the 3-level chain: data-trigger in, no output.
#[cerulion_node]
#[derive(Default)]
struct RingSink {
    #[input(trigger)]
    inp: Vector3,
}

#[cerulion_node_impl]
impl RingSink {
    fn tick(&mut self) -> Result<(), NodeError> {
        Ok(())
    }
}

/// One StepBoundary per STEP — NOT per DAG level. Every other ring-test graph
/// is single-level/source-only, where per-LEVEL == per-STEP and a regression
/// that pushed a boundary per level would pass the whole suite. This 3-LEVEL
/// chain (`Period(10)` source → data-trigger relay → data-trigger sink — the
/// `polled_vs_live` shape, where `GraphRuntime::step` iterates 3 levels per
/// step) pins the invariant: N steps ⇒ EXACTLY N boundaries (a per-level mint
/// would yield 3N), carrying step indices 0..N-1, with the 3 same-step fires
/// (and each consumer level's kind-6 read-outcome record)
/// interleaved after their step's boundary (hand oracle).
#[test]
#[serial]
fn multi_level_dag_pushes_one_boundary_per_step_not_per_level() {
    const STEPS: u64 = 4;
    let node_ids: Vec<String> = ["src", "relay", "sink"]
        .iter()
        .map(|s| s.to_string())
        .collect();
    let id_refs: Vec<&str> = node_ids.iter().map(String::as_str).collect();

    let mut ring_owner =
        TraceRingOwner::create(&format!("par_ml_{}", std::process::id()), 256, 0, &id_refs)
            .expect("create ring");
    let ring_name = ring_owner.name().to_string();
    let producer = ring_owner.producer().expect("mint producer");

    let mut src = src_def("src");
    let mut relay = src_def("relay");
    relay.node_type = "ring_relay".to_string();
    relay.inputs = vec![cerulion_core::graph::config::InputDef {
        name: "inp".to_string(),
        source: "src/out".to_string(),
    }];
    let mut sink = src_def("sink");
    sink.node_type = "ring_sink".to_string();
    sink.inputs = vec![cerulion_core::graph::config::InputDef {
        name: "inp".to_string(),
        source: "relay/out".to_string(),
    }];
    sink.outputs = vec![];
    let _ = &mut src;

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "trace_ring_multi_level".to_string(),
        prefix: "trp_ml".to_string(),
        nodes: vec![src, relay, sink],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("src".to_string(), Box::new(RingSrcEntry::new()));
    factories.insert("relay".to_string(), Box::new(RingRelayEntry::new()));
    factories.insert("sink".to_string(), Box::new(RingSinkEntry::new()));

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build 3-level chain");
    runtime.set_trace_ring_producer(producer, &node_ids);

    for _ in 0..STEPS {
        runtime.step(Duration::from_millis(10));
    }

    let mut consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
    let mut records = Vec::new();
    consumer.drain(&mut records).expect("drain (no overrun)");

    // THE pin: exactly N boundaries (not N × 3 levels), steps 0..N-1.
    let boundaries: Vec<&TraceRingRecord> = records
        .iter()
        .filter(|r| r.record_type == RECORD_TYPE_STEP_BOUNDARY)
        .collect();
    assert_eq!(
        boundaries.len() as u64,
        STEPS,
        "one boundary per STEP — a per-LEVEL mint would push {} on this \
         3-level chain",
        STEPS * 3
    );
    let steps_seen: Vec<u64> = boundaries.iter().map(|b| b.step).collect();
    assert_eq!(
        steps_seen,
        (0..STEPS).collect::<Vec<_>>(),
        "boundaries carry the step indices 0..N-1 in order"
    );

    // Full interleave hand oracle: per step k — boundary, then per level its
    // fire (src level 0, relay level 1, sink level 2, all at the step's
    // clock: the data-trigger cascade fires within the same step). Note:
    // each data-trigger consumer's unified drain additionally logs ONE kind-6
    // DrainedBatch record at its level's end (input 0, popped 1, served-seq =
    // the step index — both producers' commit counters start at 0), so the
    // relay/sink levels carry [FIRE, READ_OUTCOME] pairs.
    let mut oracle = Vec::new();
    for k in 0..STEPS {
        let t = (k + 1) * 10_000_000;
        oracle.push(boundary_record(k, t));
        for (idx, level) in [(0u32, 0u32), (1, 1), (2, 2)] {
            oracle.push(TraceRingRecord {
                step: k,
                fire_time_ns: t,
                duration_ns: 0,
                node_idx: idx,
                global_level: level,
                record_type: RECORD_TYPE_FIRE,
                reserved: 0,
            });
            if idx > 0 {
                oracle.push(TraceRingRecord::read_outcome(
                    k,
                    idx,
                    0,
                    cerulion_core::read_outcome::ReadOutcomeKind::DrainedBatch,
                    // The k-th frame on each edge carries wire sequence k.
                    Some(k as u32),
                    // Each of these reads serves a DISTINCT wire
                    // sequence, so nothing folds — every record stands for one
                    // occurrence, and `once` says so at the type.
                    cerulion_core::trace_ring::ReadRun::once(1),
                    // The record comes from the UNIFIED TRIGGER
                    // DRAIN — the scheduler reading on the node's behalf — so
                    // its wire role is DRAIN. This is a REAL production
                    // stream, so the assertion doubles as the no-inert-shipping
                    // proof that the drain path really stamps `Drain` (a mint
                    // that kept the body role fails here against the oracle).
                    cerulion_core::read_outcome::ReadSiteRole::Drain,
                ));
            }
        }
    }
    assert_eq!(
        records, oracle,
        "boundary + 3-level fire/read-outcome interleave matches the hand oracle"
    );

    drop(ring_owner);
}
