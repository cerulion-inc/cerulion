// SPDX-License-Identifier: AGPL-3.0-only
//! Behavioral FIRE e2e for a macro-generated **UnboundedSync** cdylib
//! over real iceoryx2 — the production surface of the `gen_cdylib` policy fix.
//!
//! Bug (verified first-party): the macro's `gen_cdylib` `policy_json` chain had
//! no `unbounded_sync` arm, so a `#[cerulion_node(unbounded_sync)]` cdylib
//! emitted `cerulion_node_info()` JSON with no `"policy"` key. The host parsed
//! `NodeInfo::policy() == None` and — per the `runtime.rs` build loop's
//! `macro_policy: None` arm — defaulted the node to `TriggerPolicy::Data`,
//! which fires on ANY single input arrival. A fusion cdylib (unbounded_sync
//! requires ≥2 trigger inputs) silently dropped its ALL-inputs contract: it
//! fired on the first input to arrive instead of waiting for every input.
//!
//! This file loads the REAL `test_node_macro_unbounded_sync_cdylib` fixture
//! (`a: Vector3`, `b: Vector3` trigger inputs; `fused.x = a.x + b.x` output)
//! and pins the UnboundedSync behavior against HAND oracles (never a
//! self-compare):
//!
//!   (a) both inputs arrive → fires EXACTLY once → output round-trips as
//!       `a.x + b.x` (delivered through a downstream closure sink).
//!   (b)+(c) only ONE input arrives → does NOT fire (the ALL-inputs contract);
//!       then the second input arrives → fires, reading the FIRST input's
//!       earlier value (no lost data — Sync retains the held arrival).
//!   (d) determinism: two full runs are byte-identical (Principle #7).
//!   plus the in-process ⇄ dylib policy PARITY pin (the divergence
//!   class) — an identical in-process twin declaration must yield the same
//!   `NodeInfo::policy()`.
//!
//! Delivery-based, NOT fire-count, for the value oracle — `fire_count` records
//! even on a tick Err, so the SINK's `try_view` read is the reliable signal that
//! the node actually ran AND produced the summed output. Fire-count is used
//! only for the negative/positive "did the node fire at all" assertions, where
//! 0 vs 1 is exactly the ALL-inputs contract.
//!
//! # Required fixture build
//!
//! ```bash
//! cargo build -p test_node_macro_unbounded_sync_cdylib
//! cargo test -p cerulion_core --test cdylib_unbounded_sync_fire_test -- --test-threads=1
//! ```
//!
//! `#[serial]`: the cdylib `NODES` registry is process-global and iceoryx2's
//! SHM singleton wants serial runs (per-test SHM root via `build_for_test`).

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef, OutputDef};
use cerulion_core::graph::node::{
    ClosureNodeEntry, DylibNodeEntry, MacroPolicy, NodeEntry, NodeInfo,
};
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

/// Absolute external source topics — no in-graph producer, so a raw publisher
/// (out of the graph) writes to them, letting the test control EXACTLY which
/// input receives data on which step (the `sync_fire_iox2_test` pattern).
const TOPIC_A: &str = "/uf/a";
const TOPIC_B: &str = "/uf/b";

/// Distinctive integer-valued inputs so the summed oracle (`A_VAL + B_VAL`)
/// cannot coincide with `0` or the `MISSING` sentinel.
const A_VAL: f64 = 11.0;
const B_VAL: f64 = 7.0;
/// Hand oracle for the fired output: `a.x + b.x`.
const SUM: u64 = 18;

/// Sentinel the sink never observes from a real delivery (the node writes
/// `A_VAL + B_VAL`). A measured step that leaves this in `sink_read` means the
/// sink tick did NOT run — loud, not silent.
const MISSING: u64 = u64::MAX;

/// Locate a cdylib fixture in the workspace target dir (the
/// `cdylib_unified_drain_test` pattern).
fn find_cdylib(stem: &str) -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib(stem)
}

/// In-process twin: the IDENTICAL declaration to the cdylib fixture. Used only
/// for the policy PARITY pin — the same `#[cerulion_node(unbounded_sync)]`
/// declaration must surface the same `NodeInfo::policy()` in-process and via
/// the cdylib FFI.
#[cerulion_node(unbounded_sync)]
#[derive(Default)]
struct InProcessUnboundedSyncTwin {
    #[input(trigger)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    #[output]
    fused: Vector3,
}

#[cerulion_node_impl]
impl InProcessUnboundedSyncTwin {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.fused.x = self.a.x + self.b.x;
        Ok(())
    }
}

/// Build the 2-node graph: the cdylib UnboundedSync fuse (`a` ← `/uf/a`,
/// `b` ← `/uf/b`, output `fused`) → a closure sink that records its `fused.x`
/// read into `sink_read`. Returns the runtime (its `test_transport` mints the
/// raw external publishers the test drives).
///
/// The sink is a `DataTrigger` on `fuse/fused`, so it fires — and records —
/// only when the fuse actually publishes (i.e. only when the fuse fired). A
/// step where the fuse does NOT fire leaves `sink_read` at whatever the caller
/// pre-stored (the caller resets it to `MISSING` before each measured step).
fn build_graph(prefix: &str, sink_read: Arc<AtomicU64>) -> GraphRuntime {
    let fuse = DylibNodeEntry::load(&find_cdylib("test_node_macro_unbounded_sync_cdylib"))
        .expect("load unbounded_sync cdylib");

    let sink = ClosureNodeEntry::new(
        NodeInfo::from_names(vec!["in".to_string()], vec![]).with_policy(
            MacroPolicy::DataTrigger {
                input_name: "in".to_string(),
            },
        ),
        move |ctx| {
            let v = ctx
                .subscriber_mut("in")
                .and_then(|s| {
                    s.try_view::<Vector3, _>(|view| view.x as u64)
                        .ok()
                        .flatten()
                })
                .unwrap_or(MISSING);
            sink_read.store(v, Ordering::Relaxed);
            Ok(())
        },
    )
    .with_label("unbounded_sync_sink");

    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "unbounded_sync_fire".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                ros2: None,
                id: "fuse".to_string(),
                node_type: "unbounded_sync_node".to_string(),
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
                outputs: vec![OutputDef {
                    name: "fused".to_string(),
                    schema: "Vector3".to_string(),
                    max_slice_len: None,
                    history_size: 0,
                    topic: None,
                }],
            },
            NodeDef {
                ros2: None,
                id: "sink".to_string(),
                node_type: "unbounded_sync_sink".to_string(),
                inputs: vec![InputDef {
                    name: "in".to_string(),
                    source: "fuse/fused".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), Box::new(fuse));
    factories.insert("sink".to_string(), Box::new(sink));

    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8).expect("build unbounded_sync graph")
}

/// Mint the two raw external publishers on the absolute source topics. Vector3
/// is 24 bytes fixed (< 64 with the 32-byte wire header), matching the
/// `sync_fire_iox2_test` sizing.
fn external_publishers(runtime: &GraphRuntime) -> (CerulionPublisher, CerulionPublisher) {
    let mgr = runtime.test_transport().expect("test transport parked");
    let pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("external publisher on /uf/a");
    let pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("external publisher on /uf/b");
    (pub_a, pub_b)
}

fn fuse_fires(runtime: &GraphRuntime) -> u64 {
    runtime
        .node_handle("fuse")
        .map(|h| h.fire_count())
        .unwrap_or(u64::MAX)
}

// ===========================================================================
// (a) both inputs arrive in one step → fires ONCE → output = a.x + b.x.
// ===========================================================================
#[test]
#[serial]
fn both_inputs_fire_once_and_deliver_the_sum() {
    let sink_read = Arc::new(AtomicU64::new(MISSING));
    let mut runtime = build_graph("uf_both", Arc::clone(&sink_read));
    let (mut pub_a, mut pub_b) = external_publishers(&runtime);

    {
        let mut pa = pub_a.loan_proxy::<Vector3>().expect("loan a");
        pa.x = A_VAL;
    }
    {
        let mut pb = pub_b.loan_proxy::<Vector3>().expect("loan b");
        pb.x = B_VAL;
    }
    sink_read.store(MISSING, Ordering::Relaxed);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        fuse_fires(&runtime),
        1,
        "an UnboundedSync node MUST fire exactly once when BOTH inputs have an \
         unconsumed message"
    );
    assert_eq!(
        sink_read.load(Ordering::Relaxed),
        SUM,
        "the fired output must round-trip as a.x + b.x = {A_VAL} + {B_VAL} = {SUM} \
         (delivered THROUGH the cdylib fuse to the sink's try_view)"
    );
}

// ===========================================================================
// (b)+(c) the ALL-inputs contract + no-lost-data. Only `a` arrives → no fire
// (unlike a TriggerPolicy::Data degrade, which WOULD fire on `a`
// alone). Then `b` arrives → fires, summing the RETAINED earlier `a`.
// ===========================================================================
#[test]
#[serial]
fn one_input_does_not_fire_then_second_input_fires_without_losing_data() {
    let sink_read = Arc::new(AtomicU64::new(MISSING));
    let mut runtime = build_graph("uf_one_then_two", Arc::clone(&sink_read));
    let (mut pub_a, mut pub_b) = external_publishers(&runtime);

    // Phase (b): publish `a` ONCE, then step a LONG silent window with `b`
    // absent. A Data degrade would fire on the very first `a`;
    // UnboundedSync must stay quiet the entire time (all-inputs
    // contract). The window is 500 steps × 1ms = 500 virtual ms — far past
    // any plausible `sync_window_ms` — so phase (c) firing AFTER this gap
    // also behaviorally distinguishes UNBOUNDED sync from a bounded
    // `Sync{window}` mis-emission (a bounded window would have expired the
    // pairing; the policy-identity pins catch that
    // mutation too, this makes the behavioral suite independently prove it).
    {
        let mut pa = pub_a.loan_proxy::<Vector3>().expect("loan a");
        pa.x = A_VAL;
    }
    for _ in 0..500 {
        sink_read.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(1));
        assert_eq!(
            fuse_fires(&runtime),
            0,
            "an UnboundedSync node must NOT fire while only ONE of its two \
             inputs has arrived (a TriggerPolicy::Data degrade WOULD \
             fire here — this is the headline regression pin)"
        );
        assert_eq!(
            sink_read.load(Ordering::Relaxed),
            MISSING,
            "the fuse never fired, so nothing is published to fuse/fused and the \
             sink never runs"
        );
    }

    // Phase (c): now `b` arrives. Both inputs have an unconsumed message → the
    // node fires, reading the RETAINED `a` (published 500 virtual ms ago, held
    // in the body subscriber — no timing bound expires it) and the fresh `b`
    // → no lost data.
    {
        let mut pb = pub_b.loan_proxy::<Vector3>().expect("loan b");
        pb.x = B_VAL;
    }
    sink_read.store(MISSING, Ordering::Relaxed);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        fuse_fires(&runtime),
        1,
        "once the SECOND input arrives the UnboundedSync node fires exactly once"
    );
    assert_eq!(
        sink_read.load(Ordering::Relaxed),
        SUM,
        "no lost data: the fire sums the RETAINED earlier a.x ({A_VAL}) with the \
         fresh b.x ({B_VAL}) = {SUM}"
    );
}

// ===========================================================================
// (d) determinism (Principle #7): two full runs of the (a) sequence are
// byte-identical in both the fire count and the delivered value.
// ===========================================================================
#[test]
#[serial]
fn fire_and_delivery_are_deterministic() {
    fn run(prefix: &str) -> (u64, u64) {
        let sink_read = Arc::new(AtomicU64::new(MISSING));
        let mut runtime = build_graph(prefix, Arc::clone(&sink_read));
        let (mut pub_a, mut pub_b) = external_publishers(&runtime);
        {
            let mut pa = pub_a.loan_proxy::<Vector3>().expect("loan a");
            pa.x = A_VAL;
        }
        {
            let mut pb = pub_b.loan_proxy::<Vector3>().expect("loan b");
            pb.x = B_VAL;
        }
        sink_read.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(1));
        (fuse_fires(&runtime), sink_read.load(Ordering::Relaxed))
    }

    let r1 = run("uf_det_1");
    let r2 = run("uf_det_2");
    assert_eq!(
        r1, r2,
        "UnboundedSync fire + delivery must be byte-identical across runs"
    );
    assert_eq!(
        r1,
        (1, SUM),
        "each deterministic run fires once and delivers the summed oracle"
    );
}

// ===========================================================================
// PARITY (the divergence class): the SAME
// `#[cerulion_node(unbounded_sync)]` declaration must yield the SAME
// NodeInfo::policy() in-process and via the cdylib FFI. Each side is asserted
// against the DECLARED variant (not just against each other) so a common
// regression to a wrong value cannot pass.
// ===========================================================================
#[test]
#[serial]
fn in_process_and_dylib_unbounded_sync_policy_parity() {
    let in_process = InProcessUnboundedSyncTwinEntry::new()
        .info()
        .expect("in-process info");
    assert_eq!(
        in_process.policy(),
        Some(MacroPolicy::UnboundedSync),
        "the in-process macro path must surface UnboundedSync"
    );

    let dylib = DylibNodeEntry::load(&find_cdylib("test_node_macro_unbounded_sync_cdylib"))
        .expect("load unbounded_sync cdylib");
    let dylib_info = dylib.info().expect("dylib info");
    assert_eq!(
        dylib_info.policy(),
        Some(MacroPolicy::UnboundedSync),
        "the cdylib FFI path must surface UnboundedSync (None before the policy_json fix)"
    );

    assert_eq!(
        in_process.policy(),
        dylib_info.policy(),
        "the same #[cerulion_node(unbounded_sync)] declaration must yield the \
         same policy in-process and via the cdylib FFI — the in-process/dylib \
         divergence class"
    );
}
