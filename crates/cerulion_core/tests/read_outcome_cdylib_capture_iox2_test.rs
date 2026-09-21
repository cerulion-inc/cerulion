// SPDX-License-Identifier: AGPL-3.0-only
//! Read-outcome capture for a **cdylib**
//! node over a REAL POSIX-SHM trace ring — the A+B open-question-1 pin.
//!
//! The commit-B capture sites run inside whichever copy of `cerulion_core` the
//! drain executes in. For a cdylib node that is the CDYLIB's OWN statically
//! linked copy: its `NodeContext` — subscriber and
//! all — moved across the `init()` FFI, so the snapshot drain that stages read
//! outcomes runs on the far side of the boundary. What makes the records reach
//! the bag anyway is the shared allocation: the `ReadOutcomeStage` `Arc` was
//! installed at WIRING time (host side, before the move), so the cdylib-side
//! drain records into host-visible memory, the host arms it through the same
//! shared cell, and the host scheduler's level-end merge drains it into the
//! ring (the ABI v12 contract stated at `CERULION_ABI_VERSION`).
//!
//! This file proves that end-to-end with EXACT hand oracles: a real
//! `TraceRingOwner::create_with_inputs` ring, the real
//! `test_node_macro_period_input_cdylib` fixture (`period_ms = 10`, plain
//! non-trigger `#[input] inp` — the HOLDING cdylib), and a host-driven
//! external producer fired on chosen steps. The oracle includes the
//! held-replay arm (Held rows replaying the SAME seq) — the exact shape the
//! in-process twin pins in `read_outcome_capture_iox2_test.rs` test (b), now
//! through the FFI.
//!
//! # Running (iceoryx2 SHM singleton + cdylib `NODES` singleton → serial)
//!
//! ```bash
//! cargo build -p test_node_macro_period_input_cdylib
//! cargo test -p cerulion_core --test read_outcome_cdylib_capture_iox2_test -- --test-threads=1
//! ```

#![cfg(unix)]

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::trace_ring::{
    default_capacity_records, unpack_read_outcome_meta, unpack_read_outcome_popped,
    TraceRingConsumer, TraceRingOwner, READ_OUTCOME_HELD, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME,
    READ_OUTCOME_SERVED, RECORD_TYPE_READ_OUTCOME,
};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// The cdylib fixture is `period_ms = 10`; step by the period so it fires on
/// every step.
const STEP: Duration = Duration::from_millis(10);

/// Host-driven external producer (fired via `trigger_external` on chosen
/// steps) publishing a fixed scalar — the harness controls exactly which steps
/// deliver to the cdylib's held input.
#[cerulion_node(external)]
#[derive(Default)]
struct DriveProducer {
    #[output]
    out: Vector3,
}

#[cerulion_node_impl]
impl DriveProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.out.x = 7.0;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

/// A decoded kind-6 record in oracle-comparable form (the in-process twin
/// file's `ReadRec` shape).
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
struct ReadRec {
    step: u64,
    node_idx: u32,
    input_idx: u16,
    kind: u16,
    seq: u64,
    popped: u32,
}

/// Shorthand for the cdylib consumer's expected record (node_idx 1, input 0).
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

fn find_period_input_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_period_input_cdylib")
}

/// (e) THE CDYLIB RECORDING ARM: kind-6 records for the cdylib's non-trigger
/// input appear in a REAL trace ring with exact seq oracles, the held-replay
/// rows included.
///
/// Topology: `producer` (host-driven external macro node, level 0) →
/// `cdy` (the fixture, plain `inp` edge — does NOT levelize, so both share
/// level 0 and the cdy's step-k snapshot sees publishes through step k-1;
/// the producer fires only when `trigger_external`'d). The cdy fires every
/// step (period == step), so its snapshot stages one record per step —
/// INSIDE the cdylib's copy of the drain code, into the host-shared stage.
///
/// Producer fired on steps 2 and 5 ⇒ wire seqs 0 and 1 (the commit counter,
/// gap-free) ⇒ the hand oracle:
///   steps 0-2: NoFrame            (nothing delivered yet; the step-2 publish
///                                  lands after the step-2 snapshot)
///   step  3:   Served(0, 1)       (the step-2 frame drains)
///   steps 4-5: Held(0, 0)         (held replay of the SAME seq — through
///                                  the cdylib's FFI snapshot path)
///   step  6:   Served(1, 1)
///   step  7:   Held(1, 0)
#[test]
#[serial]
fn cdylib_non_trigger_input_read_outcomes_reach_the_real_ring() {
    let node_ids: Vec<String> = vec!["producer".to_string(), "cdy".to_string()];
    let ring_tag = format!("cdylib_{}", std::process::id());
    let mut owner = TraceRingOwner::create_with_inputs(
        &ring_tag,
        default_capacity_records(),
        0,
        &["producer", "cdy"],
        &[&[], &["inp"]],
    )
    .expect("create ring");
    let ring_name = owner.name().to_string();
    let producer_handle = owner.producer().expect("mint producer");

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "read_outcome_cdylib".to_string(),
        prefix: format!("roc{}", std::process::id()),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "producer".to_string(),
                node_type: "drive_producer".to_string(),
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
                ros2: None,
                id: "cdy".to_string(),
                node_type: "period_input".to_string(),
                inputs: vec![InputDef {
                    name: "inp".to_string(),
                    source: "producer/out".to_string(),
                }],
                outputs: vec![OutputDef {
                    name: "out".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("producer".to_string(), Box::new(DriveProducerEntry::new()));
    factories.insert(
        "cdy".to_string(),
        Box::new(DylibNodeEntry::load(&find_period_input_cdylib()).expect(
            "load the period+input cdylib fixture — build it first: \
             `cargo build -p test_node_macro_period_input_cdylib` (ABI v12 \
             requires a REBUILT fixture; a v11 one is refused at load)",
        )),
    );

    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build cdylib graph");
    runtime.set_trace_ring_producer(producer_handle, &node_ids);
    assert!(
        runtime.read_outcome_stages_armed_for_test() > 0,
        "installing the ring must ARM the cdylib input's host-shared stage"
    );

    const STEPS: u64 = 8;
    for step in 0..STEPS {
        if step == 2 || step == 5 {
            runtime.trigger_external("producer").expect("trigger");
        }
        runtime.step(STEP);
    }

    let mut ring_consumer = TraceRingConsumer::open(&ring_name).expect("open consumer");
    // The manifest input table round-trips through the real ring.
    assert_eq!(
        ring_consumer.input_names(),
        &[Vec::<String>::new(), vec!["inp".to_string()]],
        "the ring manifest carries the cdylib consumer's ordered input table"
    );
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

    let expected = vec![
        rec(0, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME, 0),
        rec(1, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME, 0),
        rec(2, READ_OUTCOME_NONE, READ_OUTCOME_NO_FRAME, 0),
        rec(3, READ_OUTCOME_SERVED, 0, 1),
        rec(4, READ_OUTCOME_HELD, 0, 0),
        rec(5, READ_OUTCOME_HELD, 0, 0),
        rec(6, READ_OUTCOME_SERVED, 1, 1),
        rec(7, READ_OUTCOME_HELD, 1, 0),
    ];
    assert_eq!(
        kind6, expected,
        "the cdylib's snapshot drain (running in the cdylib's own copy of the \
         capture code) stages into the host-shared cell and the host merge bags \
         it — served → held → held replays the SAME seq across the FFI"
    );
}
