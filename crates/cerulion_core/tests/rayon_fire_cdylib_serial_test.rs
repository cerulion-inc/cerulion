// SPDX-License-Identifier: AGPL-3.0-only
//! A real
//! `DylibNodeEntry` (cdylib) routed onto the serial fire path under within-level
//! parallel fire.
//!
//! The executor routes nodes with `performs_input_snapshot() == false` (cdylib /
//! closure) that have non-trigger inputs onto the SERIAL fire path
//! (`serial_fire_node_ids` set A): without a real step-boundary freeze, a
//! same-level producer's PARALLEL publish could be observed mid-level (replay ≠
//! live). `rayon_fire_iox2_test::mixed_macro_and_closure_gated_serial` covers
//! the CLOSURE case; this file adds the REAL cdylib pin (route (a) — a new
//! `period_ms` cdylib fixture carrying a plain non-trigger `#[input]`).
//!
//! Two pins:
//!   1. DIRECT: `DylibNodeEntry::performs_input_snapshot() == false` — the
//!      load-bearing fact (a cdylib inherits the no-op snapshot default, so a
//!      cdylib with non-trigger inputs is routed serial). Kills a regression
//!      that made cdylib entries claim a real snapshot.
//!   2. E2E: a graph mixing macro Period(10) siblings (parallel) with the
//!      cdylib period node (serial-gated, fed by one producer) yields a
//!      BYTE-IDENTICAL trace THREADS=4 vs THREADS=1 — the cdylib node fires
//!      deterministically in its decision position regardless of thread count.
//!
//! # Running
//!
//! ```bash
//! cargo build -p test_node_macro_period_input_cdylib
//! cargo test -p cerulion_core --test rayon_fire_cdylib_serial_test -- --test-threads=1
//! ```
//!
//! `#[serial]` — sets process-global `CERULION_FIRE_THREADS` (RAII-restored)
//! and uses iceoryx2 (`build_for_test` per-test SHM root). `--test-threads=1`
//! per the iceoryx2 singleton convention.

use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{DylibNodeEntry, NodeEntry};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::scheduler::TraceEntry;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

const THREADS_PARALLEL: &str = "4";
const THREADS_SERIAL: &str = "1";

/// RAII guard that sets `CERULION_FIRE_THREADS` and removes it on drop, even on
/// a mid-test panic. Mirrors `rayon_fire_iox2_test::FireThreadsGuard`.
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

/// Locate the period+input cdylib fixture in the workspace target dir.
fn find_period_input_cdylib() -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib("test_node_macro_period_input_cdylib")
}

// A macro Period(10) producer feeding the cdylib's input + the parallel
// siblings. Source code is truth; no fake data.
#[cerulion_node(period_ms = 10)]
#[derive(Default)]
struct CdylibFeederProducer {
    #[output]
    out: Vector3,
    n: u64,
}

#[cerulion_node_impl]
impl CdylibFeederProducer {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.n += 1;
        self.out.x = self.n as f64;
        Ok(())
    }
}

fn producer_def(id: &str) -> NodeDef {
    NodeDef {
        ros2: None,
        id: id.to_string(),
        node_type: "cdylib_feeder_producer".to_string(),
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

// ===========================================================================
// PIN 1 (DIRECT): a loaded cdylib's performs_input_snapshot() is false.
//
// This is the load-bearing fact for route (A) of serial_fire_node_ids: a
// cdylib fires SERIALLY, so a cdylib WITH non-trigger inputs is routed serial.
// A regression that made cdylib entries claim rayon-eligibility (returning
// true) would let them onto the parallel path → a same-level producer's
// parallel publish observable mid-level. Cross-step-hold note: `performs_input_snapshot`
// is the RAYON-eligibility gate and stays `false` for a cdylib EVEN THOUGH the
// cdylib now performs a real cross-step input freeze over the FFI
// (`holds_input_snapshot() == true`) — the freeze is not "no FFI snapshot entry
// point" anymore, but the cdylib still fires serially.
// ===========================================================================
#[test]
#[serial]
fn cdylib_performs_input_snapshot_is_false() {
    let entry =
        DylibNodeEntry::load(&find_period_input_cdylib()).expect("load period+input cdylib");
    assert!(
        !entry.performs_input_snapshot(),
        "a cdylib (DylibNodeEntry) must report performs_input_snapshot() == false \
         (the rayon-eligibility gate) so a cdylib with non-trigger inputs is \
         routed serial — even though the hold gives it a real FFI input freeze"
    );
    // The cdylib DOES hold (real FFI snapshot path) — distinct from the
    // rayon gate above. The rebuilt fixture exports the optional symbols.
    assert!(
        entry.holds_input_snapshot(),
        "the period+input cdylib fixture must export the snapshot FFI \
         symbols → holds_input_snapshot() == true (so its source topics get \
         provisioned at borrow=3 and its non-trigger input is held)"
    );
    // Sanity: the fixture really has the non-trigger input + output that put it
    // in snapshot_input_names (and thus serial_fire_node_ids set A).
    let info = entry.info().expect("cdylib info parses");
    assert_eq!(
        info.input_names(),
        vec!["inp"],
        "fixture must declare the plain non-trigger input 'inp'"
    );
    assert_eq!(
        info.output_names(),
        vec!["out"],
        "fixture must declare the output 'out'"
    );
}

// ===========================================================================
// PIN 2 (E2E): macro Period siblings (parallel) + cdylib period node
// (serial-gated) → byte-identical trace THREADS=4 vs THREADS=1.
//
// Level 0: a feeder producer (feeds the cdylib), 14 macro siblings (parallel),
// and the cdylib period node (serial-gated, declared LAST → ticks in its
// decision position). 16 ≥ 2×4 → genuine parallelism. The merged trace must be
// byte-identical across thread counts: the serial-gated cdylib node fires in
// its TRUE decision slot and never perturbs the merge order.
// ===========================================================================

fn run_cdylib_mixed_trace(prefix: &str, threads: &str, steps: u32) -> Vec<TraceEntry> {
    let _guard = FireThreadsGuard::set(threads);
    const EXTRA_MACROS: usize = 14;

    let mut nodes: Vec<NodeDef> = Vec::new();
    // The producer feeding the cdylib's input.
    nodes.push(producer_def("feeder"));
    // Extra macro siblings (parallel-fired) sharing level 0.
    for i in 0..EXTRA_MACROS {
        nodes.push(producer_def(&format!("m{i:02}")));
    }
    // The cdylib period node, declared LAST → ticks after the macros in
    // decision order; routed serial by serial_fire_node_ids (set A).
    nodes.push(NodeDef {
        ros2: None,
        id: "gated_cdylib".to_string(),
        node_type: "period_input".to_string(),
        inputs: vec![InputDef {
            name: "inp".to_string(),
            source: "feeder/out".to_string(),
        }],
        outputs: vec![OutputDef {
            name: "out".to_string(),
            schema: "Vector3".to_string(),
            max_slice_len: None,
            history_size: 0,
            topic: None,
        }],
    });

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "cdylib_mixed".to_string(),
        prefix: prefix.to_string(),
        nodes,
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert(
        "feeder".to_string(),
        Box::new(CdylibFeederProducerEntry::new()),
    );
    for i in 0..EXTRA_MACROS {
        factories.insert(
            format!("m{i:02}"),
            Box::new(CdylibFeederProducerEntry::new()),
        );
    }
    factories.insert(
        "gated_cdylib".to_string(),
        Box::new(DylibNodeEntry::load(&find_period_input_cdylib()).expect("load cdylib")),
    );

    let clock = Arc::new(VirtualClock::new());
    let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
        .expect("build cdylib mixed graph");
    for _ in 0..steps {
        runtime.step(Duration::from_millis(10));
    }
    runtime.trace().to_vec()
}

#[test]
#[serial]
fn cdylib_gated_serial_trace_byte_identical_parallel_vs_serial() {
    const STEPS: u32 = 8;
    // 1 feeder + 14 macros + 1 cdylib = 16 nodes, all level 0.
    const N: usize = 16;

    let parallel = run_cdylib_mixed_trace("cdy_par", THREADS_PARALLEL, STEPS);
    let serial = run_cdylib_mixed_trace("cdy_ser", THREADS_SERIAL, STEPS);

    // Non-vacuous: all 16 nodes (incl. the gated cdylib) fire every step.
    let expected = N * STEPS as usize;
    assert_eq!(
        parallel.len(),
        expected,
        "cdylib mixed parallel trace: {N} nodes (15 macro + 1 gated cdylib) × {STEPS} steps"
    );
    assert_eq!(
        serial.len(),
        expected,
        "cdylib mixed serial trace: same count"
    );

    assert_eq!(
        parallel, serial,
        "the serial-gated REAL cdylib node (non-trigger input + no-op snapshot → \
         serial_fire_node_ids set A) must fire in its TRUE decision position so \
         the merged trace is BYTE-IDENTICAL THREADS=4 vs THREADS=1 — the gated \
         cdylib never perturbs the merge order. parallel={parallel:?} serial={serial:?}"
    );
}
