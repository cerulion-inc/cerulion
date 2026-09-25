// SPDX-License-Identifier: AGPL-3.0-only
//! ABI v8: `#[input(depth = N)]` AND the declared backpressure
//! (A2 — same one-shot bump) are REAL across the cdylib FFI.
//!
//! Pre-v8 the host HARDCODED `depth: DEFAULT_CONSUMER_DEPTH` +
//! `backpressure: DropOldest` when building a dylib node's `InputMeta` —
//! so `depth = 32` / `backpressure = block` were honored in
//! `build_for_test` runs (in-process nodes) but silently defaulted in
//! production `cerulion graph run` (dylib-loaded nodes): a test/live
//! divergence in the queue-depth property, and for `block` a
//! Principle-#6-adjacent one (the silent DropOldest degrade DROPPED DATA
//! live while tests passed). v8 emits both declarations into the cdylib
//! info JSON and the host parses them through to `InputMeta`, where
//! `GraphTopology::build` derives buffer sizing + the block pre-fire gate.
//!
//! Pins, all against DECLARED values (never a self-compare; depth 32 is
//! DOUBLE the global transport default 16 and 3.2× the
//! `DEFAULT_CONSUMER_DEPTH` fallback 10, so it distinguishes the declared
//! value from BOTH pre-v8 failure modes):
//!
//! 1. PARSE — the dylib fixture's `info().input_meta()` carries depth 32 +
//!    `Block` on `inp` and `Sample(7)` on `aux`.
//! 2. PARITY — an in-process node with the IDENTICAL declarations yields
//!    the same `InputMeta` depth + backpressure (the divergence class
//!    the in-process/dylib parity work kills); plus the fallback arm: a bare-`#[input]` cdylib
//!    resolves `DEFAULT_CONSUMER_DEPTH` + `DropOldest`, same as
//!    in-process.
//! 3. E2E over real iceoryx2 — (a) the ceiling oracle: the topic is
//!    provisioned at `max(global 16, 32) = 32` (attach-at-32 succeeds,
//!    33 rejected by iceoryx2 itself); (b) the BLOCK PLATEAU oracle
//!    (patterned on `topic_buffer_sizing_test` /
//!    `backpressure_block_iox2_test`): against the never-firing dylib
//!    consumer, the producer plateaus at EXACTLY 32 fires and the
//!    consumer's `backpressure_block_fires_deferred_count("inp")` goes
//!    positive — only possible when BOTH depth (32, not 10/16) and
//!    `block` (not DropOldest) crossed the FFI. Pre-v8: producer never
//!    defers (DropOldest) and fires all 40 steps — the regression guard.
//!
//! # Build requirement + serial
//!
//! Requires `cargo build -p test_node_macro_depth_cdylib` (and
//! `test_node_macro_period_input_cdylib` for the fallback arm). All tests
//! `#[serial]` (cdylib NODES singleton); run with `--test-threads=1`
//! alongside the other iceoryx2 families.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::transport::{PublisherProvisioning, TopicServiceConfig};
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// The declared depth on the fixture's `#[input(depth = 32)]` — the oracle
/// every assertion compares against (hand-pasted, not read from the
/// fixture).
const DECLARED_DEPTH: usize = 32;

/// Locate a fixture cdylib in the workspace target dir (mirrors
/// `macro_cdylib_policy_round_trip_test::find_cdylib`).
fn find_cdylib(crate_name: &str) -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib(crate_name)
}

// ============================================================
// 1. PARSE: the declared depth survives the FFI
// ============================================================

#[test]
#[serial]
#[tracing_test::traced_test]
fn dylib_declared_depth_and_backpressure_reach_input_meta() {
    let node = DylibNodeEntry::load(&find_cdylib("test_node_macro_depth_cdylib"))
        .expect("load depth fixture");
    let info = node.info().expect("info should parse");
    // Validate-pass fix: the unknown-key warn's negative control must run
    // against REAL macro-emitted JSON, not a hand-authored mirror of the
    // legal-key list — this fixture carries the richest v8 payload shipped
    // (depth + block + the sample(7) OBJECT form + trigger). If a future
    // macro emits a new info-JSON key without extending the legal sets,
    // every real cdylib would warn at load and THIS assertion fails.
    assert!(
        !logs_contain("unknown key in cerulion_node_info() JSON"),
        "real macro-emitted info JSON must parse without unknown-key warns \
         (the legal-key sets must track the emitter)"
    );
    let meta = info.input_meta();
    assert_eq!(meta.len(), 2);
    assert_eq!(meta[0].name, "inp");
    assert_eq!(
        meta[0].depth,
        DECLARED_DEPTH,
        "the declared #[input(depth = 32)] must cross the cdylib FFI \
         (pre-v8 this was the hardcoded DEFAULT_CONSUMER_DEPTH = {})",
        cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH
    );
    assert_eq!(
        meta[0].backpressure,
        BackpressurePolicy::Block,
        "the declared `backpressure = block` must cross the cdylib FFI \
         (pre-A2 it silently degraded to DropOldest)"
    );
    assert_eq!(meta[1].name, "aux");
    assert_eq!(
        meta[1].backpressure,
        BackpressurePolicy::Sample(7),
        "the declared `backpressure = sample(7)` must cross the cdylib FFI"
    );
    assert_eq!(
        meta[1].depth,
        cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
        "aux declares no depth — the host fallback applies"
    );
}

// ============================================================
// 2. PARITY: in-process vs dylib — identical declaration, identical depth
// ============================================================

/// In-process twin of the fixture: the IDENTICAL declaration shape
/// (`external` + `#[input(backpressure = block, depth = 32)]` +
/// `#[input(backpressure = sample(7))]` + HostDriven source).
#[cerulion_node(external)]
#[derive(Default)]
struct InProcessDepthTwin {
    #[input(backpressure = block, depth = 32)]
    inp: Vector3,
    #[input(backpressure = sample(7))]
    aux: Vector3,
}

#[cerulion_node_impl]
impl InProcessDepthTwin {
    fn tick(&mut self) -> Result<(), NodeError> {
        let _ = self.inp.x;
        let _ = self.aux.x;
        Ok(())
    }

    fn external_source(&mut self) -> ExternalSource {
        ExternalSource::HostDriven
    }
}

#[test]
#[serial]
fn in_process_and_dylib_qos_parity_against_declared_values() {
    // Both sides must equal the DECLARED values — asserting each against
    // the literals (not against each other alone) keeps this from passing
    // if BOTH paths regressed to a common wrong value.
    let in_process = InProcessDepthTwinEntry::new()
        .info()
        .expect("in-process info");
    assert_eq!(in_process.input_meta()[0].depth, DECLARED_DEPTH);
    assert_eq!(
        in_process.input_meta()[0].backpressure,
        BackpressurePolicy::Block
    );
    assert_eq!(
        in_process.input_meta()[1].backpressure,
        BackpressurePolicy::Sample(7)
    );

    let dylib = DylibNodeEntry::load(&find_cdylib("test_node_macro_depth_cdylib"))
        .expect("load depth fixture");
    let dylib_info = dylib.info().expect("dylib info");
    assert_eq!(dylib_info.input_meta()[0].depth, DECLARED_DEPTH);
    assert_eq!(
        dylib_info.input_meta()[0].backpressure,
        BackpressurePolicy::Block
    );
    assert_eq!(
        dylib_info.input_meta()[1].backpressure,
        BackpressurePolicy::Sample(7)
    );

    assert_eq!(
        in_process.input_meta()[0].depth,
        dylib_info.input_meta()[0].depth,
        "same declaration must yield the same depth in-process and via FFI \
         — the in-process/dylib divergence class"
    );
    assert_eq!(
        in_process.input_meta()[0].backpressure,
        dylib_info.input_meta()[0].backpressure,
        "same declaration must yield the same backpressure in-process and \
         via FFI (pre-A2: silent DropOldest degrade)"
    );
}

#[test]
#[serial]
fn undeclared_depth_falls_back_to_default_on_both_paths() {
    // Fallback arm: a bare `#[input]` (no depth) cdylib resolves the
    // host-side DEFAULT_CONSUMER_DEPTH — the absent-key path. The
    // period-input fixture declares a bare non-trigger `#[input]`.
    let dylib = DylibNodeEntry::load(&find_cdylib("test_node_macro_period_input_cdylib"))
        .expect("load period+input fixture");
    let info = dylib.info().expect("dylib info");
    assert_eq!(
        info.input_meta()[0].depth,
        cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH,
        "an UNDECLARED depth must resolve DEFAULT_CONSUMER_DEPTH across the \
         FFI (absent JSON key = not declared)"
    );
    assert_eq!(
        info.input_meta()[0].backpressure,
        BackpressurePolicy::DropOldest,
        "an UNDECLARED backpressure must resolve DropOldest across the FFI"
    );
}

// ============================================================
// 3. E2E: the dylib's declared depth provisions the topic ceiling
// ============================================================

/// Period producer publishing one Vector3 per tick on each of two outputs
/// (in-process; the QoS under test lives on the DYLIB consumer). `aux_out`
/// exists so the consumer's `sample(7)` input rides its OWN topic — wiring
/// it onto `/out` would make that topic MIXED (block + sample) and degrade
/// the block gate, killing the plateau oracle.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct DepthFeedProducer {
    #[output]
    out: Vector3,
    #[output]
    aux_out: Vector3,
    n: u32,
}

#[cerulion_node_impl]
impl DepthFeedProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        self.aux_out.x = self.n as f64;
        Ok(())
    }
}

#[test]
#[serial]
fn dylib_declared_depth_provisions_topic_ceiling_e2e() {
    // Graph: in-process producer -> DYLIB depth-32 consumer. The topic's
    // service ceiling is topology-derived as max(global default 16,
    // largest consumer depth) — with the dylib's depth = 32 arriving via
    // the v8 FFI, the ceiling MUST be exactly 32. Pre-v8 (hardcode 10)
    // the ceiling was max(16, 10) = 16 and the require-32 probe below
    // fails — the regression guard.
    let config = GraphConfig {
        execution: None,
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cdd".to_string(),
        prefix: "cdd".to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "producer".to_string(),
                node_type: "depth_feed_producer".to_string(),
                inputs: vec![],
                outputs: vec![
                    OutputDef {
                        name: "out".to_string(),
                        schema: "Vector3".to_string(),
                        max_slice_len: None,
                        history_size: 0,
                        topic: None,
                    },
                    OutputDef {
                        name: "aux_out".to_string(),
                        schema: "Vector3".to_string(),
                        max_slice_len: None,
                        history_size: 0,
                        topic: None,
                    },
                ],
            },
            NodeDef {
                fuse: None,
                ros2: None,
                id: "consumer".to_string(),
                node_type: "depth_probe".to_string(),
                inputs: vec![
                    InputDef {
                        name: "inp".to_string(),
                        source: "producer/out".to_string(),
                    },
                    InputDef {
                        name: "aux".to_string(),
                        source: "producer/aux_out".to_string(),
                    },
                ],
                outputs: vec![],
            },
        ],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "producer".to_string(),
        Box::new(DepthFeedProducerEntry::new()),
    );
    factories.insert(
        "consumer".to_string(),
        Box::new(
            DylibNodeEntry::load(&find_cdylib("test_node_macro_depth_cdylib"))
                .expect("load depth fixture"),
        ),
    );

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("build producer->dylib-consumer graph");
    let mgr = Arc::clone(runtime.test_transport().expect("test transport parked"));
    let topic = "/cdd/producer/out";

    // (a) A default opener (global 16 <= 32) still attaches — CLI
    //     introspection compatibility.
    if let Err(e) = mgr.create_subscriber(topic) {
        panic!("a default opener must attach to the raised-ceiling topic: {e}");
    }
    // (b) An opener requiring the FULL declared depth attaches — the
    //     service really was provisioned at 32 (iceoryx2 rejects openers
    //     requiring more than the creator provisioned).
    if let Err(e) = mgr.create_subscriber_with_buffers(
        topic,
        TopicServiceConfig::for_topology(
            mgr.default_topic_config(),
            DECLARED_DEPTH,
            1,
            PublisherProvisioning::SingleWriter,
            0,
            0,
        ),
        DECLARED_DEPTH,
    ) {
        panic!(
            "an opener requiring the dylib-declared ceiling (32) must attach — \
             the depth must have crossed the FFI into topology provisioning: {e}"
        );
    }
    // (c) 33 > 32 is rejected by iceoryx2 itself — success-at-32 +
    //     failure-at-33 pins the service at EXACTLY the declared depth
    //     (without this arm, (b) would be vacuous on an over-provisioned
    //     service).
    match mgr.create_subscriber_with_buffers(
        topic,
        TopicServiceConfig::for_topology(
            mgr.default_topic_config(),
            DECLARED_DEPTH + 1,
            1,
            PublisherProvisioning::SingleWriter,
            0,
            0,
        ),
        DECLARED_DEPTH + 1,
    ) {
        Ok(_) => panic!("an opener requiring 33 must NOT attach to the 32-provisioned service"),
        Err(e) => {
            let msg = e.to_string();
            assert!(
                msg.contains("33") && msg.contains("smaller buffer ceiling"),
                "the open error must carry the requested ceiling and the \
                 ordering hint: {msg}"
            );
        }
    }

    // (d) THE BLOCK PLATEAU (A2's behavioral acceptance): the dylib
    //     consumer never fires (HostDriven, never triggered), so its
    //     `inp` queue never drains. With BOTH depth=32 and `block`
    //     arriving across the FFI, the scheduler's pre-fire gate defers
    //     the producer the moment outstanding == 32: fire_count plateaus
    //     at EXACTLY 32 over 40 steps, and the consumer's per-input
    //     defer counter goes positive. Pre-A2 (block degraded to
    //     DropOldest) the producer fires all 40; pre-v8-depth it would
    //     plateau at 10 — either way this fails.
    for _ in 0..40 {
        runtime.step(Duration::from_millis(10));
    }
    assert_eq!(
        runtime.node_handle("producer").unwrap().fire_count(),
        DECLARED_DEPTH as u64,
        "producer must plateau at the dylib-declared depth 32 — depth AND \
         block must BOTH have crossed the FFI"
    );
    assert!(
        runtime
            .node_handle("consumer")
            .unwrap()
            .backpressure_block_fires_deferred_count("inp")
            > 0,
        "the dylib consumer's block input must record deferred producer \
         fires (the block machinery engaged across the FFI)"
    );
}
