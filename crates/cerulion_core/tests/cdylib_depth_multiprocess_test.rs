// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib-parity cluster 4: the FFI-parsed `#[input(depth = 32)]` of a cdylib survives
//! into the MULTI-PROCESS provisioning path.
//!
//! `cdylib_depth_ffi_test` proves the declared depth 32 crosses the info-JSON
//! FFI and provisions the topic ceiling in a MONOLITH `build_for_test`. The
//! multi-process delta: when a `process_groups:` graph runs
//! as N worker PROCESSES, each worker builds only its own subgraph, so a
//! producer-owning worker would under-provision a cross-group consumer's topic.
//! The supervisor closes this by HARVESTING the full-graph monolith's
//! `GraphRuntime::topic_requirements()` (the serde-portable per-topic reduction)
//! and stamping it into every producer-owning `WorkerPlan`; each worker then
//! raises its owned topics to that union (`provisioning_overrides`).
//!
//! # What this file pins (portable — no subprocess)
//!
//! The graph `producer(out, aux_out) -> depth-cdylib consumer(inp = block/32,
//! aux = sample/7)` is declared with `process_groups:` and built via
//! `build_for_test` — the MONOLITH-fallback path (what a non-Linux host, or
//! `--single-process`, runs). The harvest is proven to carry the FFI-parsed
//! depth: `topic_requirements()["/cd4/producer/out"].min_buffer == 32`. That map
//! is the EXACT value the supervisor stamps into the producer's `WorkerPlan`, so
//! the depth-32 that a worker provisions with is proven to originate from the
//! cdylib's FFI-declared depth — the "depth survives the supervisor→worker spawn"
//! contract, tested at the harvest seam.
//!
//! # NOTE-SKIP (needs the real binary on Linux, precise blocker)
//!
//! The REAL Linux multi-process SPLIT — the supervisor spawning `cerulion graph
//! run-worker` processes, each unioning the stamped harvest and provisioning its
//! owned topics across address spaces — cannot be expressed from
//! `cerulion_core/tests`: `plan_deployment` / `stamp_topic_requirements` /
//! `build_worker` / the process spawn live in `cerulion_cli_engine` + the
//! `cerulion` binary, and the real-binary harness is
//! `cerulion_cli/tests/mp_supervisor_box_test.rs` (spawns the real binary,
//! Linux-gated). This file covers the portable monolith-fallback + the harvest
//! origin; the cross-address-space split is that harness's arm.
//!
//! # Build requirement + serial
//!
//! Requires `cargo build -p test_node_macro_depth_cdylib`. `#[serial]` (cdylib
//! `NODES` + iceoryx2 SHM singletons).

use std::sync::Arc;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// The declared depth on the fixture's `#[input(depth = 32)]` — the oracle the
/// harvest must carry (hand-pasted, not read from the fixture).
const DECLARED_DEPTH: usize = 32;

fn find_cdylib(crate_name: &str) -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib(crate_name)
}

fn out_def(name: &str) -> OutputDef {
    OutputDef {
        name: name.to_string(),
        schema: "Vector3".to_string(),
        max_slice_len: None,
        history_size: 0,
        topic: None,
    }
}

/// Feeds the depth cdylib's two inputs (`out` → `inp`, `aux_out` → `aux`).
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct MpFeedProducer {
    #[output]
    out: Vector3,
    #[output]
    aux_out: Vector3,
    n: u32,
}
#[cerulion_node_impl]
impl MpFeedProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        self.aux_out.x = self.n as f64;
        Ok(())
    }
}

#[test]
#[serial]
fn dylib_declared_depth_survives_into_multiprocess_provisioning_harvest() {
    // A genuine `process_groups:` graph: the producer and the depth cdylib are
    // in DIFFERENT groups (the shape a real split deploys). `build_for_test`
    // validates the partition (validate_process_groups) and builds the
    // MONOLITH-fallback — exactly what a non-Linux host / `--single-process`
    // runs — then computes the harvest the supervisor would stamp into workers.
    let mut process_groups: IndexMap<String, Vec<String>> = IndexMap::new();
    process_groups.insert("sensors".to_string(), vec!["producer".to_string()]);
    process_groups.insert("perception".to_string(), vec!["consumer".to_string()]);

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups,
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cd4".to_string(),
        prefix: "cd4".to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "producer".to_string(),
                node_type: "mp_feed_producer".to_string(),
                inputs: vec![],
                outputs: vec![out_def("out"), out_def("aux_out")],
            },
            NodeDef {
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
    factories.insert("producer".to_string(), Box::new(MpFeedProducerEntry::new()));
    factories.insert(
        "consumer".to_string(),
        Box::new(
            DylibNodeEntry::load(&find_cdylib("test_node_macro_depth_cdylib"))
                .expect("load depth fixture"),
        ),
    );

    let clock = Arc::new(VirtualClock::new());
    // buffer 16 (the global default) — so max(16, 32) can only be 32 if the
    // FFI-declared depth 32 crossed; a regression to the hardcoded
    // DEFAULT_CONSUMER_DEPTH (10) would yield max(16, 10) = 16.
    let runtime = GraphRuntime::build_for_test(config, factories, clock, 16)
        .expect("a process_groups graph must build via the monolith fallback");

    // THE HARVEST ORACLE: the producer-owned topic /cd4/producer/out (which the
    // block/depth-32 consumer reads) must carry min_buffer == 32 — the exact
    // value the supervisor stamps into the producer's WorkerPlan. This proves
    // the cdylib's FFI-declared depth reaches the multi-process provisioning
    // harvest, not just the monolith topic ceiling.
    let reqs = runtime.topic_requirements();
    // Match by suffix so the assertion is robust to the leading-slash form of
    // the derived topic key. `producer/aux_out` (the sample input's topic) does
    // NOT end with `producer/out`, so this uniquely selects the block topic.
    let (topic, req) = reqs
        .iter()
        .find(|(k, _)| k.ends_with("producer/out"))
        .unwrap_or_else(|| {
            panic!(
                "the producer-owned block topic (…/producer/out) must appear in the \
                 multi-process harvest; harvested topics = {:?}",
                reqs.keys().collect::<Vec<_>>()
            )
        });
    let _ = topic;
    assert_eq!(
        req.min_buffer, DECLARED_DEPTH,
        "the harvest must carry the FFI-declared depth 32 (a regression to the \
         hardcoded DEFAULT_CONSUMER_DEPTH=10 yields max(16,10)=16) — this is the \
         value the supervisor stamps into the producer worker"
    );
}
