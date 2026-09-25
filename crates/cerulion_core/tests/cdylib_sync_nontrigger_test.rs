// SPDX-License-Identifier: AGPL-3.0-only
//! Cdylib PARITY for **trigger-scoped Sync** (the
//! no-inert-shipping rule) — the FFI path must prove the same semantics the
//! in-process tests pinned (`sync_fire_iox2_test.rs`).
//!
//! Loads the REAL `test_node_macro_sync_nontrigger_cdylib` fixture
//! (`#[cerulion_node(sync_window_ms = 25)]`; trigger inputs `cam` + `lidar`,
//! plain `#[input] config`, output `fused.x = cam.x + lidar.x` /
//! `fused.y = config.x`) over `build_for_test` + `DylibNodeEntry`, and pins
//! against HAND oracles (never a self-compare):
//!
//!   1. HAPPY: cam+lidar align in-window → fires; `fused.y` proves the tick
//!      read config's latest value; later trigger-aligned fires with config
//!      SILENT read the HELD value: the cross-step hold THROUGH the FFI
//!      (ABI-v9 trigger carry + the `cerulion_node_{set_,}snapshot_inputs`
//!      symbols end-to-end).
//!   2. Trigger scoping through the FFI: a config stamp 200 ms outside the
//!      25 ms window does NOT gate the fire (before trigger scoping, config sat in
//!      `Sync.inputs` and its stale stamp broke `check_sync` — 0 fires); an
//!      ABSENT config still lets the scheduler fire, but the body WAITS
//!      (the pre-first-delivery collapse — no fabricated value, nothing
//!      published).
//!   3. Capability introspection: `holds_input_snapshot() == true`,
//!      `performs_input_snapshot() == false` (serial-fire path), the policy
//!      round-trip, and the ABI-v9 per-input trigger flags.
//!   4. In-process ⇄ dylib PARITY (the class where the two surfaces diverge): an
//!      identical in-process twin under the identical stimulus produces the
//!      identical (fires, per-step reads) sequence — both equal the oracle.
//!   5. Determinism: two full dylib runs are byte-identical (Principle #7).
//!
//! Delivery-based value oracles (the sink's `try_view` read), NOT
//! fire-count — `fire_count` records even when the tick collapses, which is
//! exactly what the absent-config arm distinguishes.
//!
//! # Required fixture build
//!
//! ```bash
//! cargo build -p test_node_macro_sync_nontrigger_cdylib
//! cargo test -p cerulion_core --test cdylib_sync_nontrigger_test -- --test-threads=1
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

/// Absolute external source topics (no in-graph producer) so raw publishers
/// control EXACTLY which input receives data on which step.
const TOPIC_CAM: &str = "/snt/cam";
const TOPIC_LIDAR: &str = "/snt/lidar";
const TOPIC_CONFIG: &str = "/snt/config";

/// Hand-oracle values: cam = k (1..=3), lidar = 10 → sum = 10 + k;
/// config = 7 delivered ONCE.
const LIDAR_VAL: f64 = 10.0;
const CONFIG_VAL: f64 = 7.0;

/// Sentinel a real delivery never produces (`fused.y` is 7; `fused.x` is
/// 11..=13). Left in the sink cells when the sink tick did NOT run — i.e.
/// the fuse published nothing that step.
const MISSING: u64 = u64::MAX;

/// Locate a cdylib fixture in the workspace target dir (the
/// `cdylib_unbounded_sync_fire_test` pattern).
fn find_cdylib(stem: &str) -> std::path::PathBuf {
    cerulion_core::testing::find_fixture_cdylib(stem)
}

fn load_fixture() -> DylibNodeEntry {
    DylibNodeEntry::load(&find_cdylib("test_node_macro_sync_nontrigger_cdylib"))
        .expect("load sync-nontrigger cdylib")
}

/// In-process twin: the IDENTICAL declaration to the cdylib fixture, for the
/// parity arm (the in-process vs dylib divergence class: each side is asserted against
/// the HAND oracle, so a common regression cannot pass).
#[cerulion_node(sync_window_ms = 25)]
#[derive(Default)]
struct InProcessSyncNonTriggerTwin {
    #[input(trigger)]
    cam: Vector3,
    #[input(trigger)]
    lidar: Vector3,
    #[input]
    config: Vector3,
    #[output]
    fused: Vector3,
}

#[cerulion_node_impl]
impl InProcessSyncNonTriggerTwin {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.fused.x = self.cam.x + self.lidar.x;
        self.fused.y = self.config.x;
        Ok(())
    }
}

/// Build the 2-node graph: `fuse` (supplied by the caller — the cdylib OR the
/// in-process twin; identical port names) → a closure sink (DataTrigger on
/// `fuse/fused`) recording `(fused.x, fused.y)` into the two shared cells.
fn build_graph(
    prefix: &str,
    fuse: Box<dyn NodeEntry>,
    sink_x: Arc<AtomicU64>,
    sink_y: Arc<AtomicU64>,
) -> GraphRuntime {
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
                    s.try_view::<Vector3, _>(|view| (view.x as u64, view.y as u64))
                        .ok()
                        .flatten()
                })
                .unwrap_or((MISSING, MISSING));
            sink_x.store(v.0, Ordering::Relaxed);
            sink_y.store(v.1, Ordering::Relaxed);
            Ok(())
        },
    )
    .with_label("sync_nontrigger_sink");

    let config = GraphConfig {
        execution: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        level_assignments: None,
        network: None,
        name: None,
        identity: "sync_nontrigger_ffi".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "fuse".to_string(),
                node_type: "sync_nontrigger_node".to_string(),
                inputs: vec![
                    InputDef {
                        name: "cam".to_string(),
                        source: TOPIC_CAM.to_string(),
                    },
                    InputDef {
                        name: "lidar".to_string(),
                        source: TOPIC_LIDAR.to_string(),
                    },
                    InputDef {
                        name: "config".to_string(),
                        source: TOPIC_CONFIG.to_string(),
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
                fuse: None,
                ros2: None,
                id: "sink".to_string(),
                node_type: "sync_nontrigger_sink".to_string(),
                inputs: vec![InputDef {
                    name: "in".to_string(),
                    source: "fuse/fused".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), fuse);
    factories.insert("sink".to_string(), Box::new(sink));

    let clock = Arc::new(VirtualClock::new());
    GraphRuntime::build_for_test(config, factories, clock, 8).expect("build sync-nontrigger graph")
}

/// Mint the three raw external publishers on the absolute source topics.
fn external_publishers(
    runtime: &GraphRuntime,
) -> (CerulionPublisher, CerulionPublisher, CerulionPublisher) {
    let mgr = runtime.test_transport().expect("test transport parked");
    let pub_cam = mgr
        .create_publisher(TOPIC_CAM, MaxSliceLen::const_new(64), 0)
        .expect("external publisher on /snt/cam");
    let pub_lidar = mgr
        .create_publisher(TOPIC_LIDAR, MaxSliceLen::const_new(64), 0)
        .expect("external publisher on /snt/lidar");
    let pub_config = mgr
        .create_publisher(TOPIC_CONFIG, MaxSliceLen::const_new(64), 0)
        .expect("external publisher on /snt/config");
    (pub_cam, pub_lidar, pub_config)
}

fn publish_x(publisher: &mut CerulionPublisher, x: f64) {
    let mut p = publisher.loan_proxy::<Vector3>().expect("loan");
    p.x = x;
}

fn fuse_fires(runtime: &GraphRuntime) -> u64 {
    runtime
        .node_handle("fuse")
        .map(|h| h.fire_count())
        .unwrap_or(u64::MAX)
}

/// The shared HAPPY stimulus, runnable against the cdylib or the twin:
/// config=7 delivered ONCE (a config-only step must not fire), then 3
/// trigger-aligned rounds (cam=k, lidar=10). Returns `(fuse fire_count,
/// per-round (sink_x, sink_y))`.
///
/// HAND ORACLE: `(3, [(11, 7), (12, 7), (13, 7)])` — every fire reads the
/// config delivered once before the first round (rounds 2-3 are the HELD
/// replay; for the cdylib that hold crosses the snapshot FFI symbols).
fn run_happy_sequence(prefix: &str, fuse: Box<dyn NodeEntry>) -> (u64, Vec<(u64, u64)>) {
    let sink_x = Arc::new(AtomicU64::new(MISSING));
    let sink_y = Arc::new(AtomicU64::new(MISSING));
    let mut runtime = build_graph(prefix, fuse, Arc::clone(&sink_x), Arc::clone(&sink_y));
    let (mut pub_cam, mut pub_lidar, mut pub_config) = external_publishers(&runtime);

    // Config delivered ONCE; a config-only step never fires the node.
    publish_x(&mut pub_config, CONFIG_VAL);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        fuse_fires(&runtime),
        0,
        "config alone must NOT fire a Sync node (non-trigger input)"
    );

    let mut reads = Vec::new();
    for k in 1..=3u64 {
        publish_x(&mut pub_cam, k as f64);
        publish_x(&mut pub_lidar, LIDAR_VAL);
        sink_x.store(MISSING, Ordering::Relaxed);
        sink_y.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(1));
        reads.push((
            sink_x.load(Ordering::Relaxed),
            sink_y.load(Ordering::Relaxed),
        ));
    }
    (fuse_fires(&runtime), reads)
}

/// The hand oracle for [`run_happy_sequence`].
fn happy_oracle() -> (u64, Vec<(u64, u64)>) {
    (3, vec![(11, 7), (12, 7), (13, 7)])
}

// ===========================================================================
// 1. HAPPY: trigger alignment fires; config's latest value is read and HELD
//    across config-silent fires — through the cdylib FFI.
// ===========================================================================
#[test]
#[serial]
fn happy_trigger_alignment_reads_and_holds_config_through_ffi() {
    let got = run_happy_sequence("snth", Box::new(load_fixture()));
    assert_eq!(
        got,
        happy_oracle(),
        "the cdylib must fire on each cam+lidar alignment and read config=7 on \
         EVERY fire (delivered once before the first round — rounds 2-3 are the cross-step \
         hold crossing the snapshot FFI symbols); got {got:?}"
    );
}

// ===========================================================================
// 2a. Trigger scoping through the FFI: a STALE config (200 ms outside the
//     25 ms window) does NOT gate the fire — the sharpest in-process oracle,
//     now over the cdylib. Before trigger-scoped Sync: 0 fires.
// ===========================================================================
#[test]
#[serial]
fn stale_config_does_not_gate_the_fire_through_ffi() {
    let sink_x = Arc::new(AtomicU64::new(MISSING));
    let sink_y = Arc::new(AtomicU64::new(MISSING));
    let mut runtime = build_graph(
        "snts",
        Box::new(load_fixture()),
        Arc::clone(&sink_x),
        Arc::clone(&sink_y),
    );
    let (mut pub_cam, mut pub_lidar, mut pub_config) = external_publishers(&runtime);

    // config stamped at sim-time ~0, then 200 ms of silence.
    publish_x(&mut pub_config, CONFIG_VAL);
    runtime.step(Duration::from_millis(1));
    for _ in 0..200 {
        runtime.step(Duration::from_millis(1));
    }

    // cam+lidar stamped ~201 ms — spread 0 between the TRIGGER inputs.
    publish_x(&mut pub_cam, 1.0);
    publish_x(&mut pub_lidar, LIDAR_VAL);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        fuse_fires(&runtime),
        1,
        "a config stamp 200 ms outside the 25 ms window must NOT block the \
         cdylib's fire — the spread is computed over TRIGGER inputs only \
         (before trigger-scoped Sync this was 0 fires)"
    );
    assert_eq!(
        (
            sink_x.load(Ordering::Relaxed),
            sink_y.load(Ordering::Relaxed)
        ),
        (11, 7),
        "the fire delivers the trigger sum + the held (stale-but-latest) config"
    );
}

// ===========================================================================
// 2b. ABSENT config: the scheduler fire happens (config does not gate), but
//     the body WAITS — the pre-first-delivery collapse crosses the
//     FFI: nothing is published, no value is fabricated.
// ===========================================================================
#[test]
#[serial]
fn absent_config_fire_happens_but_body_waits_through_ffi() {
    let sink_x = Arc::new(AtomicU64::new(MISSING));
    let sink_y = Arc::new(AtomicU64::new(MISSING));
    let mut runtime = build_graph(
        "snta",
        Box::new(load_fixture()),
        Arc::clone(&sink_x),
        Arc::clone(&sink_y),
    );
    let (mut pub_cam, mut pub_lidar, _pub_config) = external_publishers(&runtime);
    // NOTE: /snt/config never receives data.

    publish_x(&mut pub_cam, 1.0);
    publish_x(&mut pub_lidar, LIDAR_VAL);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        fuse_fires(&runtime),
        1,
        "the SCHEDULER fire happens on trigger alignment — an undelivered \
         non-trigger config must not gate it (through the FFI)"
    );
    assert_eq!(
        (
            sink_x.load(Ordering::Relaxed),
            sink_y.load(Ordering::Relaxed)
        ),
        (MISSING, MISSING),
        "the BODY waits: the Empty non-trigger input collapses the cdylib's \
         tick — nothing published, no fabricated config value (Principle #13)"
    );
}

// ===========================================================================
// 3. Capability + ABI-v9 introspection (mirrors cdylib_non_trigger_hold_test).
// ===========================================================================
#[test]
#[serial]
fn capability_and_abi_v9_introspection() {
    let entry = load_fixture();
    assert!(
        entry.holds_input_snapshot(),
        "a macro cdylib exports the snapshot FFI pair → \
         holds_input_snapshot() == true. If false, rebuild the fixture: \
         cargo build -p test_node_macro_sync_nontrigger_cdylib"
    );
    assert!(
        !entry.performs_input_snapshot(),
        "a cdylib must stay OFF the rayon path (performs_input_snapshot() == \
         false) — it HOLDS but fires serially"
    );

    let info = entry.info().expect("fixture info parses");
    assert_eq!(
        info.policy(),
        Some(MacroPolicy::Sync { window_ms: 25 }),
        "the bounded-sync policy must round-trip the FFI"
    );
    // ABI v9: the per-input trigger marks cross the info-JSON FFI — the
    // exact carry the trigger scoping consumes.
    let meta = info.input_meta();
    assert_eq!(meta.len(), 3);
    assert_eq!(meta[0].name, "cam");
    assert!(meta[0].trigger, "cam is `#[input(trigger)]`");
    assert_eq!(meta[1].name, "lidar");
    assert!(meta[1].trigger, "lidar is `#[input(trigger)]`");
    assert_eq!(meta[2].name, "config");
    assert!(
        !meta[2].trigger,
        "config is a plain `#[input]` — the ABI-v9 carry must keep it \
         non-trigger (pre-v9 the flag was dropped on the FFI floor)"
    );
}

// ===========================================================================
// 4. In-process ⇄ dylib PARITY (the class where the two surfaces diverge): identical
//    declarations + identical stimulus → identical (fires, reads) — and BOTH
//    equal the hand oracle, so a common regression cannot pass.
// ===========================================================================
#[test]
#[serial]
fn in_process_twin_parity_identical_sequence() {
    let dylib = run_happy_sequence("sntp_d", Box::new(load_fixture()));
    let twin = run_happy_sequence("sntp_t", Box::new(InProcessSyncNonTriggerTwinEntry::new()));
    assert_eq!(
        dylib,
        happy_oracle(),
        "the cdylib side must match the hand oracle"
    );
    assert_eq!(
        twin,
        happy_oracle(),
        "the in-process side must match the hand oracle"
    );
    assert_eq!(
        dylib, twin,
        "the same declaration must behave identically in-process and via the \
         cdylib FFI — the in-process/dylib divergence class"
    );
}

// ===========================================================================
// 5. DETERMINISM (Principle #7): two full dylib runs are byte-identical.
// ===========================================================================
#[test]
#[serial]
fn determinism_two_dylib_runs_byte_identical() {
    let r1 = run_happy_sequence("sntd1", Box::new(load_fixture()));
    let r2 = run_happy_sequence("sntd2", Box::new(load_fixture()));
    assert_eq!(r1, r2, "two dylib runs must be byte-identical");
    assert_eq!(
        r1,
        happy_oracle(),
        "and both must equal the hand oracle (not a self-compare)"
    );
}

// ===========================================================================
// PER-SET SYNC THROUGH THE REAL FFI
// ===========================================================================
//
// Until these arms, `cerulion_node_sync_head_op` had no caller in any test:
// the whole per-set matcher was exercised only in-process, while the
// DEPLOYMENT surface for every `graph run` node is `DylibNodeEntry`. Per-set
// on that surface rides a hand-written double mapping — host op → code
// (`graph/node.rs`), macro-emitted code → op (`cerulion_macros`), plus the
// `out_kind` answer decode — and a mapping nothing drives is a mapping nobody
// knows is wrong.
//
// The pre-existing arms in this file DO take the per-set path, but their
// stimulus is one frame per input per alignment, which reaches only the
// argmin's `ProbeNext` and never a `PeekNext` or an `Advance`. These two
// drive a DESCENT and a BACKLOG, which is where the other ops live.

/// Per-fire record of `fused.x`, in fire order.
type FusedSeq = Arc<std::sync::Mutex<Vec<u64>>>;

/// The same 2-node graph as [`build_graph`], with two differences the per-set
/// arms need: the caller owns the CLOCK (so publishes carry hand-chosen wire
/// stamps, which is what the matcher's spans are computed from), and the sink
/// records EVERY fire rather than the last.
///
/// The sink declares its input through `NodeInfo::with_meta` rather than
/// `from_names`, which is load-bearing: a names-only closure is wired the
/// legacy Separate way and its `try_view` drains to LATEST, so a 3-fire burst
/// would record the newest frame three times and the sequence oracle would be
/// meaningless. With meta it is Unified per-message FIFO, so one fire reads
/// one frame.
fn build_seq_graph(
    prefix: &str,
    fuse: Box<dyn NodeEntry>,
    seq: FusedSeq,
    clock: Arc<VirtualClock>,
) -> GraphRuntime {
    let sink_meta = cerulion_core::graph::node::InputMeta {
        name: "in".to_string(),
        schema_hash: <Vector3 as cerulion_core::message::ShmMessage>::SCHEMA_HASH,
        trigger: true,
        depth: 16,
        backpressure: cerulion_core::graph::node::BackpressurePolicy::DropOldest,
        expect_within_ms: None,
    };
    let sink = ClosureNodeEntry::new(
        NodeInfo::with_meta(vec![sink_meta], vec![]).with_policy(MacroPolicy::DataTrigger {
            input_name: "in".to_string(),
        }),
        move |ctx| {
            if let Some(x) = ctx.subscriber_mut("in").and_then(|s| {
                s.try_view::<Vector3, _>(|view| view.x as u64)
                    .ok()
                    .flatten()
            }) {
                seq.lock().expect("seq sink poisoned").push(x);
            }
            Ok(())
        },
    )
    .with_label("sync_nontrigger_sink");

    let config = GraphConfig {
        execution: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        level_assignments: None,
        network: None,
        name: None,
        identity: "sync_nontrigger_perset".to_string(),
        prefix: prefix.to_string(),
        nodes: vec![
            NodeDef {
                fuse: None,
                ros2: None,
                id: "fuse".to_string(),
                node_type: "sync_nontrigger_node".to_string(),
                inputs: vec![
                    InputDef {
                        name: "cam".to_string(),
                        source: TOPIC_CAM.to_string(),
                    },
                    InputDef {
                        name: "lidar".to_string(),
                        source: TOPIC_LIDAR.to_string(),
                    },
                    InputDef {
                        name: "config".to_string(),
                        source: TOPIC_CONFIG.to_string(),
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
                fuse: None,
                ros2: None,
                id: "sink".to_string(),
                node_type: "sync_nontrigger_sink".to_string(),
                inputs: vec![InputDef {
                    name: "in".to_string(),
                    source: "fuse/fused".to_string(),
                }],
                outputs: vec![],
            },
        ],
    };

    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), fuse);
    factories.insert("sink".to_string(), Box::new(sink));
    GraphRuntime::build_for_test(config, factories, clock, 8).expect("build per-set sync graph")
}

/// Publish `x` stamped at `at_ns` on the shared virtual clock, so the payload
/// carries its own wire stamp and a fire's members are self-identifying.
fn publish_stamped(publisher: &mut CerulionPublisher, clock: &VirtualClock, at_ns: u64, x: f64) {
    clock.set(at_ns);
    let mut p = publisher.loan_proxy::<Vector3>().expect("loan");
    p.x = x;
}

const MS_NS: u64 = 1_000_000;

/// A DESCENT runs through the real FFI: `PeekNext` and `Advance` cross the
/// cdylib boundary nine times and land on the nearest arrived member.
///
/// `cam = [0..9] ms`, `lidar = [20] ms`, window 25 ms. `lidar` is genuinely
/// scarce, so the scarcity gate passes and the walk descends `cam` all the way:
/// each step strictly tightens the span (20 → 19 → … → 11) and counts one
/// PASSED-OVER skip.
///
/// HAND ORACLE: exactly ONE fire reading `cam@9 + lidar@20 = 29`, with
/// `sync_closer_skip_count(cam) == 9` and nothing unmatchable. A walk that
/// stopped early would serve a smaller sum; one that never ran would serve
/// `0 + 20 = 20`; a `PeekNext` that CONSUMED would land short and lose frames.
/// None of those is reachable without the ops actually crossing the FFI.
#[test]
#[serial]
fn cdylib_per_set_descends_through_the_real_ffi() {
    let seq: FusedSeq = Arc::new(std::sync::Mutex::new(Vec::new()));
    let clock = Arc::new(VirtualClock::new());
    let fuse = Box::new(load_fixture()) as Box<dyn NodeEntry>;
    let mut runtime = build_seq_graph("sntd", fuse, Arc::clone(&seq), Arc::clone(&clock));
    let (mut pub_cam, mut pub_lidar, mut pub_config) = external_publishers(&runtime);

    // The non-trigger context must be delivered once or the tick collapses
    // (the pre-first-delivery WAIT) and nothing publishes.
    publish_stamped(&mut pub_config, &clock, 0, CONFIG_VAL);
    for k in 0..10u64 {
        publish_stamped(&mut pub_cam, &clock, k * MS_NS, k as f64);
    }
    publish_stamped(&mut pub_lidar, &clock, 20 * MS_NS, 20.0);

    clock.set(0);
    for _ in 0..3 {
        runtime.step(Duration::from_millis(1));
    }

    assert_eq!(
        *seq.lock().expect("seq sink poisoned"),
        vec![29u64],
        "the descent must cross the FFI and land on cam@9: ONE set, \
         `cam@9 + lidar@20`. A walk that never ran serves 20"
    );
    let handle = runtime.node_handle("fuse").expect("node handle");
    assert_eq!(
        handle.sync_closer_skip_count(TOPIC_CAM),
        9,
        "nine frames were PASSED OVER — one per `Advance` that crossed the \
         FFI. This counter is written only inside the ops-driven align pass, \
         so a nonzero value is itself evidence the ops ran"
    );
    assert_eq!(
        handle.sync_unmatched_discard_count(TOPIC_CAM),
        0,
        "and nothing died: every cam frame is inside the 25 ms window of \
         lidar@20"
    );
}

/// A queued BACKLOG is served as k fires in set order through the real FFI —
/// the contract that separates per-set from the earlier collapse, driven
/// on the surface every `graph run` node actually uses.
///
/// `cam = [1, 2, 3] ms`, `lidar = [5, 10, 15] ms`, window 25 ms: three
/// complete sets among arrived frames. The gate REFUSES to descend while both
/// inputs still hold a second arrived frame, so sets 1-2 serve greedily; set 3
/// fires greedily too, because its argmin has no successor.
///
/// HAND ORACLE: `[6, 12, 18]` — `(1+5)`, `(2+10)`, `(3+15)` — with ZERO skips
/// on both counters. Before the per-set Sync change this exact stimulus produced ONE fire
/// reading the freshest members (`3 + 15 = 18`), so a regression to the
/// collapse fails on the LENGTH, and a matcher that eats the backlog to
/// tighten one set fails on the VALUES.
#[test]
#[serial]
fn cdylib_per_set_serves_a_queued_backlog_as_k_fires_through_the_real_ffi() {
    let seq: FusedSeq = Arc::new(std::sync::Mutex::new(Vec::new()));
    let clock = Arc::new(VirtualClock::new());
    let fuse = Box::new(load_fixture()) as Box<dyn NodeEntry>;
    let mut runtime = build_seq_graph("sntb", fuse, Arc::clone(&seq), Arc::clone(&clock));
    let (mut pub_cam, mut pub_lidar, mut pub_config) = external_publishers(&runtime);

    publish_stamped(&mut pub_config, &clock, 0, CONFIG_VAL);
    for k in 1..=3u64 {
        publish_stamped(&mut pub_cam, &clock, k * MS_NS, k as f64);
        publish_stamped(&mut pub_lidar, &clock, 5 * k * MS_NS, (5 * k) as f64);
    }

    clock.set(0);
    for _ in 0..4 {
        runtime.step(Duration::from_millis(1));
    }

    assert_eq!(
        *seq.lock().expect("seq sink poisoned"),
        vec![6u64, 12, 18],
        "three complete sets are present among arrived frames, so THREE fires \
         must serve them in set order, each reading its OWN members. One fire \
         reading 18 is the pre-per-set collapse"
    );
    let handle = runtime.node_handle("fuse").expect("node handle");
    assert_eq!(
        (
            handle.sync_closer_skip_count(TOPIC_CAM),
            handle.sync_unmatched_discard_count(TOPIC_CAM),
            handle.sync_closer_skip_count(TOPIC_LIDAR),
            handle.sync_unmatched_discard_count(TOPIC_LIDAR),
        ),
        (0, 0, 0, 0),
        "and NOTHING is skipped on either counter: the gate forbids descending \
         into a backlog whose every input still holds a second arrived frame"
    );
}
