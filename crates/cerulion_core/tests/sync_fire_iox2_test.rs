// SPDX-License-Identifier: AGPL-3.0-only
//! A `Sync` node FIRES end-to-end through `GraphRuntime`.
//!
//! This covers what `macro_sync_threading_test.rs` does not
//! (its build-only arm cannot assert end-to-end firing).
//! Before the Sync fire-path wiring, a multi-trigger Sync
//! node did NOT fire through `GraphRuntime`: the build loop synthesized a drain
//! subscriber + binding ONLY for `MacroPolicy::DataTrigger`, so `drain_level`
//! never called `signal_sync_input` → `sync_input_timestamps` stayed empty →
//! `evaluate_node`'s Sync arm never fired. `test_sync_fires_when_both_inputs_arrive`
//! is the REGRESSION PIN: it FAILS on the pre-wiring code (zero fires).
//!
//! `#[serial]` — real iceoryx2 over the process-global SHM singleton;
//! per-test SHM root via `build_for_test`.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef};
use cerulion_core::graph::node::NodeEntry;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::prelude::*;
use cerulion_core::wire::MaxSliceLen;
use indexmap::IndexMap;
use native_ros2_messages::geometry_msgs::Vector3;
use serial_test::serial;

const TOPIC_A: &str = "/sync/a";
const TOPIC_B: &str = "/sync/b";

/// A 2-input bounded-Sync node: fires when BOTH inputs arrive within the
/// `sync_window_ms` window. Shares an `Arc<AtomicU64>` fire counter.
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct SyncFuse {
    #[input(trigger)]
    a: Vector3,
    #[input(trigger)]
    b: Vector3,
    fires: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl SyncFuse {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.fires.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// A single `SyncFuse` consumer whose two trigger inputs are sourced from the
/// absolute external topics `/sync/a` and `/sync/b` (no in-graph producer →
/// External topics that an out-of-graph publisher can write to).
fn sync_graph(fires: Arc<AtomicU64>) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        level_assignments: None,
        network: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        name: None,
        identity: "sync_fire_test".to_string(),
        prefix: "syncf".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "fuse".to_string(),
            node_type: "sync_fuse".to_string(),
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
    factories.insert(
        "fuse".to_string(),
        Box::new(SyncFuseEntry::with_state(SyncFuse {
            fires,
            ..Default::default()
        })),
    );
    (config, factories)
}

/// REGRESSION PIN: both inputs arrive within the window → the Sync node fires.
/// Zero fires on the pre-wiring code (the bug). Under `VirtualClock` both
/// publishes are stamped at sim-time 0 (no `step`/`advance` yet), so they are
/// trivially within the 50 ms window → one fire on the first `step`.
#[test]
#[serial]
fn test_sync_fires_when_both_inputs_arrive() {
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = sync_graph(Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");

    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("external publisher on /sync/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("external publisher on /sync/b");

    // Publish to BOTH inputs (same sim-time stamp), then step → both drained,
    // both signal_sync_input, check_sync passes → exactly one fire.
    {
        let mut pa = pub_a.loan_proxy::<Vector3>().expect("loan a");
        pa.x = 1.0;
    }
    {
        let mut pb = pub_b.loan_proxy::<Vector3>().expect("loan b");
        pb.x = 2.0;
    }
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        fires.load(Ordering::Relaxed),
        1,
        "a Sync node MUST fire once when both inputs arrive within the window \
         (this is zero on the pre-wiring code — the bug)"
    );
}

/// NEGATIVE: only ONE input arrives → the Sync node never fires (check_sync
/// requires every input to have a recent timestamp).
#[test]
#[serial]
fn test_sync_does_not_fire_with_one_input() {
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = sync_graph(Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");

    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("external publisher on /sync/a");
    // Note: /sync/b never receives data.

    for _ in 0..5 {
        {
            let mut pa = pub_a.loan_proxy::<Vector3>().expect("loan a");
            pa.x = 1.0;
        }
        runtime.step(Duration::from_millis(1));
    }

    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "a Sync node must NOT fire when only one of its two inputs has arrived"
    );
}

/// NEGATIVE: both inputs arrive but spread WIDER than the window → no fire.
#[test]
#[serial]
fn test_sync_does_not_fire_outside_window() {
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = sync_graph(Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");

    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("external publisher on /sync/a");
    let mut pub_b = mgr
        .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
        .expect("external publisher on /sync/b");

    // a at sim-time 0, drained on the first step.
    {
        let mut pa = pub_a.loan_proxy::<Vector3>().expect("loan a");
        pa.x = 1.0;
    }
    runtime.step(Duration::from_millis(1));
    // Advance well past the 50 ms window, then publish b → its stamp is ~200 ms
    // after a's, so check_sync's spread > window → no fire.
    for _ in 0..200 {
        runtime.step(Duration::from_millis(1));
    }
    {
        let mut pb = pub_b.loan_proxy::<Vector3>().expect("loan b");
        pb.x = 2.0;
    }
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        fires.load(Ordering::Relaxed),
        0,
        "a Sync node must NOT fire when its inputs arrive >window apart (a@0ms, b@~201ms, window=50ms)"
    );
}

/// DETERMINISM (Principle #7): two identical builds + identical publish/step
/// sequences produce the identical fire count.
#[test]
#[serial]
fn test_sync_fire_is_deterministic() {
    fn run() -> u64 {
        let fires = Arc::new(AtomicU64::new(0));
        let (config, factories) = sync_graph(Arc::clone(&fires));
        let clock = Arc::new(VirtualClock::new());
        let mut runtime =
            GraphRuntime::build_for_test(config, factories, clock, 8).expect("build sync graph");
        let mgr = runtime.test_transport().expect("test transport parked");
        let mut pub_a = mgr
            .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
            .expect("pub a");
        let mut pub_b = mgr
            .create_publisher(TOPIC_B, MaxSliceLen::const_new(64), 0)
            .expect("pub b");
        for _ in 0..3 {
            {
                let mut pa = pub_a.loan_proxy::<Vector3>().expect("loan a");
                pa.x = 1.0;
            }
            {
                let mut pb = pub_b.loan_proxy::<Vector3>().expect("loan b");
                pb.x = 2.0;
            }
            runtime.step(Duration::from_millis(1));
        }
        fires.load(Ordering::Relaxed)
    }
    let r1 = run();
    let r2 = run();
    assert_eq!(r1, r2, "Sync fire count must be deterministic across runs");
    assert!(
        r1 >= 1,
        "the deterministic run must fire at least once, got {r1}"
    );
}

/// Live-wake path: the Sync drain listeners are WaitSet sources, so
/// sync data WAKES the live loop promptly (else a pure-Sync graph would fire
/// only on the ~250 ms liveliness cadence). `run_waitset_reactor_once_for_test`
/// returns the node ids whose listeners fired; publishing to a sync input must
/// surface the Sync node — proving its drain listener is attached to the
/// WaitSet. Without the wake-path wiring the sync listener is not a source and
/// the node never appears in the fired set.
#[test]
#[serial]
fn test_sync_input_wakes_the_waitset() {
    let fires = Arc::new(AtomicU64::new(0));
    let (config, factories) = sync_graph(Arc::clone(&fires));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_a = mgr
        .create_publisher(TOPIC_A, MaxSliceLen::const_new(64), 0)
        .expect("external publisher on /sync/a");

    // Prime: drain build/attach connection-lifecycle events off the event queue
    // so the assertion below is attributable to the data publish.
    {
        let mut pa = pub_a.loan_proxy::<Vector3>().expect("loan a");
        pa.x = 1.0;
    }
    let _ = runtime.run_waitset_reactor_once_for_test(Duration::from_millis(50));

    // Publish to the sync input → its drain listener (a WaitSet source) fires →
    // the Sync node id surfaces in the reactor's fired set.
    {
        let mut pa = pub_a.loan_proxy::<Vector3>().expect("loan a");
        pa.x = 2.0;
    }
    let fired = runtime.run_waitset_reactor_once_for_test(Duration::from_millis(250));
    assert!(
        fired.contains(&"fuse".to_string()),
        "publishing to a Sync input must WAKE the WaitSet (its drain listener is \
         a source) — expected \"fuse\" in the fired set, got {fired:?}"
    );
}

// ===========================================================================
// Sync/UnboundedSync align ONLY `#[input(trigger)]`-marked inputs.
// A plain `#[input]` on a sync node is a latest-value read (step-boundary
// snapshot + cross-step hold) that never gates the fire — exactly like a
// DataTrigger node's non-trigger inputs. The pre-existing tests above keep
// the all-trigger shape (`SyncFuse`: both inputs marked) green, pinning the
// no-regression half of the flip.
// ===========================================================================

const TOPIC_CAM: &str = "/s425/cam";
const TOPIC_LIDAR: &str = "/s425/lidar";
const TOPIC_CONFIG: &str = "/s425/config";

/// Sentinel the config producer never publishes (it publishes 5, 7, or 9).
/// Left in `last_config` when the tick body did NOT run — an Empty
/// (never-delivered) non-trigger input collapses the whole tick to a no-op
/// (the pre-first-delivery WAIT; mirrors
/// `non_trigger_hold_iox2_test`'s MISSING-sentinel mechanism).
const MISSING: u64 = u64::MAX;

/// The trigger-scoped fusion shape: 2 `#[input(trigger)]` (cam, lidar) + 1 plain
/// `#[input]` (config), bounded 50 ms window. The tick records what it reads
/// from `config` and counts its own body runs (distinct from the scheduler's
/// `fire_count`, which also counts fires whose body collapsed on an Empty
/// non-trigger input).
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct SyncFuseCtx {
    #[input(trigger)]
    cam: Vector3,
    #[input(trigger)]
    lidar: Vector3,
    #[input]
    config: Vector3,
    body_runs: Arc<AtomicU64>,
    last_config: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl SyncFuseCtx {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last_config
            .store(self.config.x as u64, Ordering::Relaxed);
        self.body_runs.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// [`SyncFuseCtx`] twin whose `lidar` trigger carries an
/// `expect_within_ms = 5` input watchdog. Used by the starved-trigger
/// watchdog arm; the ports keep the same names so [`ctx_graph`] serves it.
#[cerulion_node(sync_window_ms = 50)]
#[derive(Default)]
struct SyncFuseWatch {
    #[input(trigger)]
    cam: Vector3,
    #[input(trigger, expect_within_ms = 5)]
    lidar: Vector3,
    #[input]
    config: Vector3,
    body_runs: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl SyncFuseWatch {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.body_runs.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// The UnboundedSync twin of [`SyncFuseCtx`] — no timing bound: fires when
/// both trigger inputs have unconsumed messages, whenever that happens.
#[cerulion_node(unbounded_sync)]
#[derive(Default)]
struct UnboundedFuseCtx {
    #[input(trigger)]
    cam: Vector3,
    #[input(trigger)]
    lidar: Vector3,
    #[input]
    config: Vector3,
    body_runs: Arc<AtomicU64>,
    last_config: Arc<AtomicU64>,
}

#[cerulion_node_impl]
impl UnboundedFuseCtx {
    fn tick(&mut self) -> Result<(), NodeError> {
        self.last_config
            .store(self.config.x as u64, Ordering::Relaxed);
        self.body_runs.fetch_add(1, Ordering::Relaxed);
        Ok(())
    }
}

/// Single fuse node wiring cam/lidar/config to the three absolute external
/// topics. `entry` is supplied by the caller so the SAME topology serves the
/// bounded ([`SyncFuseCtx`]) and unbounded ([`UnboundedFuseCtx`]) variants.
fn ctx_graph(entry: Box<dyn NodeEntry>) -> (GraphConfig, IndexMap<String, Box<dyn NodeEntry>>) {
    let config = GraphConfig {
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        level_assignments: None,
        network: None,
        name: None,
        identity: "sync_ctx_fire_test".to_string(),
        prefix: "s425f".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "fuse".to_string(),
            node_type: "sync_fuse_ctx".to_string(),
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
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("fuse".to_string(), entry);
    (config, factories)
}

/// Publish `x` into `topic_pub` (fixed Vector3 — the loan writes straight
/// into SHM).
fn publish_x(topic_pub: &mut cerulion_core::CerulionPublisher, x: f64) {
    let mut p = topic_pub.loan_proxy::<Vector3>().expect("loan");
    p.x = x;
}

/// HEADLINE: the node fires when cam+lidar align in-window
/// REGARDLESS of config's arrival state; each fire reads config's LATEST
/// value; config HOLDS across fires (delivered ONCE, read on every
/// subsequent fire). HAND ORACLE: config=7.0 published once, then 3
/// trigger-aligned rounds → body runs [1,2,3] each reading 7.
#[test]
#[serial]
fn test_sync_fires_on_trigger_alignment_and_holds_config() {
    let body_runs = Arc::new(AtomicU64::new(0));
    let last_config = Arc::new(AtomicU64::new(MISSING));
    let (config, factories) = ctx_graph(Box::new(SyncFuseCtxEntry::with_state(SyncFuseCtx {
        body_runs: Arc::clone(&body_runs),
        last_config: Arc::clone(&last_config),
        ..Default::default()
    })));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build ctx sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_cam = mgr
        .create_publisher(TOPIC_CAM, MaxSliceLen::const_new(64), 0)
        .expect("pub cam");
    let mut pub_lidar = mgr
        .create_publisher(TOPIC_LIDAR, MaxSliceLen::const_new(64), 0)
        .expect("pub lidar");
    let mut pub_config = mgr
        .create_publisher(TOPIC_CONFIG, MaxSliceLen::const_new(64), 0)
        .expect("pub config");

    // Config delivered ONCE. A config-only step never fires the node — the
    // plain #[input] is not a trigger.
    publish_x(&mut pub_config, 7.0);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        body_runs.load(Ordering::Relaxed),
        0,
        "config alone must NOT fire a Sync node (non-trigger input)"
    );

    // Three trigger-aligned rounds; every fire reads the HELD config=7.
    for k in 1..=3u64 {
        publish_x(&mut pub_cam, k as f64);
        publish_x(&mut pub_lidar, k as f64);
        last_config.store(MISSING, Ordering::Relaxed);
        runtime.step(Duration::from_millis(1));
        assert_eq!(
            body_runs.load(Ordering::Relaxed),
            k,
            "fire {k}: cam+lidar aligned in-window must fire the body"
        );
        assert_eq!(
            last_config.load(Ordering::Relaxed),
            7,
            "fire {k}: the tick reads config's latest value (7.0, delivered once \
             and HELD across fires)"
        );
    }
}

/// UnboundedSync variant: fires when both triggers have unconsumed messages —
/// no timing bound (cam retained across many silent steps) — and config never
/// gates. HAND ORACLE: cam@t0, 10 silent ms, lidar@t11 → exactly 1 fire
/// reading config=9.
#[test]
#[serial]
fn test_unbounded_sync_config_never_gates() {
    let body_runs = Arc::new(AtomicU64::new(0));
    let last_config = Arc::new(AtomicU64::new(MISSING));
    let (config, factories) = ctx_graph(Box::new(UnboundedFuseCtxEntry::with_state(
        UnboundedFuseCtx {
            body_runs: Arc::clone(&body_runs),
            last_config: Arc::clone(&last_config),
            ..Default::default()
        },
    )));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build unbounded graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_cam = mgr
        .create_publisher(TOPIC_CAM, MaxSliceLen::const_new(64), 0)
        .expect("pub cam");
    let mut pub_lidar = mgr
        .create_publisher(TOPIC_LIDAR, MaxSliceLen::const_new(64), 0)
        .expect("pub lidar");
    let mut pub_config = mgr
        .create_publisher(TOPIC_CONFIG, MaxSliceLen::const_new(64), 0)
        .expect("pub config");

    publish_x(&mut pub_config, 9.0);
    publish_x(&mut pub_cam, 1.0);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        body_runs.load(Ordering::Relaxed),
        0,
        "cam alone must not fire (lidar's trigger slot is empty)"
    );

    // 10 silent steps — no window to expire (unbounded); cam stays armed.
    for _ in 0..10 {
        runtime.step(Duration::from_millis(1));
    }
    assert_eq!(body_runs.load(Ordering::Relaxed), 0);

    publish_x(&mut pub_lidar, 2.0);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        body_runs.load(Ordering::Relaxed),
        1,
        "lidar arriving 11 ms after cam must fire (unbounded — no window; the \
         retained cam still counts)"
    );
    assert_eq!(
        last_config.load(Ordering::Relaxed),
        9,
        "the fire reads config's latest value; config never gated the fire"
    );
}

/// THE SHARPEST FLIP ORACLE: a STALE config timestamp far outside the sync
/// window must NOT block the fire. Before the trigger-scoping flip, config's topic was part of
/// `TriggerPolicy::Sync.inputs`, so its 200 ms-old stamp made the spread
/// exceed the 50 ms window → NO fire (this test FAILS on the pre-flip
/// classification). Post-flip the spread is computed over TRIGGER inputs
/// only → fires, reading the held config.
#[test]
#[serial]
fn test_stale_config_outside_window_does_not_block_fire() {
    let body_runs = Arc::new(AtomicU64::new(0));
    let last_config = Arc::new(AtomicU64::new(MISSING));
    let (config, factories) = ctx_graph(Box::new(SyncFuseCtxEntry::with_state(SyncFuseCtx {
        body_runs: Arc::clone(&body_runs),
        last_config: Arc::clone(&last_config),
        ..Default::default()
    })));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build ctx sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_cam = mgr
        .create_publisher(TOPIC_CAM, MaxSliceLen::const_new(64), 0)
        .expect("pub cam");
    let mut pub_lidar = mgr
        .create_publisher(TOPIC_LIDAR, MaxSliceLen::const_new(64), 0)
        .expect("pub lidar");
    let mut pub_config = mgr
        .create_publisher(TOPIC_CONFIG, MaxSliceLen::const_new(64), 0)
        .expect("pub config");

    // config stamped at sim-time ~0, then 200 ms of silence.
    publish_x(&mut pub_config, 7.0);
    runtime.step(Duration::from_millis(1));
    for _ in 0..200 {
        runtime.step(Duration::from_millis(1));
    }

    // cam+lidar stamped ~201 ms — 200 ms after config, spread 0 between the
    // two TRIGGER inputs.
    publish_x(&mut pub_cam, 1.0);
    publish_x(&mut pub_lidar, 2.0);
    runtime.step(Duration::from_millis(1));

    assert_eq!(
        body_runs.load(Ordering::Relaxed),
        1,
        "a config stamp 200 ms outside the 50 ms window must NOT block the fire \
         — the window spread is computed over TRIGGER inputs only (before the flip \
         this was 0 fires: config's stale stamp broke check_sync)"
    );
    assert_eq!(
        last_config.load(Ordering::Relaxed),
        7,
        "the fire reads the held (stale-but-latest) config value"
    );
}

/// EDGE (pre-first-delivery hold): config NEVER delivered → the trigger
/// alignment still FIRES the node at the scheduler (fire_count == 1 — config
/// does not gate), but the body WAITS: the Empty non-trigger input collapses
/// the tick (no fabricated default, Principle #13), so `body_runs` stays 0
/// and `last_config` keeps the MISSING sentinel (mirrors
/// `non_trigger_hold_iox2_test`'s pre-delivery arm).
#[test]
#[serial]
fn test_sync_fires_with_undelivered_config_but_body_waits() {
    let body_runs = Arc::new(AtomicU64::new(0));
    let last_config = Arc::new(AtomicU64::new(MISSING));
    let (config, factories) = ctx_graph(Box::new(SyncFuseCtxEntry::with_state(SyncFuseCtx {
        body_runs: Arc::clone(&body_runs),
        last_config: Arc::clone(&last_config),
        ..Default::default()
    })));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build ctx sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_cam = mgr
        .create_publisher(TOPIC_CAM, MaxSliceLen::const_new(64), 0)
        .expect("pub cam");
    let mut pub_lidar = mgr
        .create_publisher(TOPIC_LIDAR, MaxSliceLen::const_new(64), 0)
        .expect("pub lidar");
    // NOTE: /s425/config never receives data.

    publish_x(&mut pub_cam, 1.0);
    publish_x(&mut pub_lidar, 2.0);
    runtime.step(Duration::from_millis(1));

    let fire_count = runtime
        .node_handle("fuse")
        .expect("fuse handle")
        .fire_count();
    assert_eq!(
        fire_count, 1,
        "the SCHEDULER fire happens on trigger alignment — an undelivered \
         non-trigger config must not gate it"
    );
    assert_eq!(
        body_runs.load(Ordering::Relaxed),
        0,
        "the BODY waits: an Empty (never-delivered) non-trigger input collapses \
         the tick to a no-op — no fabricated config value (Principle #13)"
    );
    assert_eq!(
        last_config.load(Ordering::Relaxed),
        MISSING,
        "no value was ever read from the undelivered config input"
    );
}

/// ADVERSARIAL: starved trigger — cam AND config flowing, lidar silent →
/// NEVER fires (scheduler-level too). When lidar finally arrives, exactly one
/// fire, proving cam's data was retained by the sync drain and config's
/// latest value is read. HAND ORACLE: 5 starved rounds → 0 fires; +lidar →
/// 1 fire reading config=5.
#[test]
#[serial]
fn test_starved_trigger_never_fires_despite_flowing_config() {
    let body_runs = Arc::new(AtomicU64::new(0));
    let last_config = Arc::new(AtomicU64::new(MISSING));
    let (config, factories) = ctx_graph(Box::new(SyncFuseCtxEntry::with_state(SyncFuseCtx {
        body_runs: Arc::clone(&body_runs),
        last_config: Arc::clone(&last_config),
        ..Default::default()
    })));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build ctx sync graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_cam = mgr
        .create_publisher(TOPIC_CAM, MaxSliceLen::const_new(64), 0)
        .expect("pub cam");
    let mut pub_lidar = mgr
        .create_publisher(TOPIC_LIDAR, MaxSliceLen::const_new(64), 0)
        .expect("pub lidar");
    let mut pub_config = mgr
        .create_publisher(TOPIC_CONFIG, MaxSliceLen::const_new(64), 0)
        .expect("pub config");

    for k in 1..=5u64 {
        publish_x(&mut pub_cam, k as f64);
        publish_x(&mut pub_config, 5.0);
        runtime.step(Duration::from_millis(1));
        assert_eq!(
            body_runs.load(Ordering::Relaxed),
            0,
            "round {k}: cam + config flowing but lidar silent → must NOT fire"
        );
    }
    assert_eq!(
        runtime
            .node_handle("fuse")
            .expect("fuse handle")
            .fire_count(),
        0,
        "starved trigger: zero scheduler fires too (flowing config is not a trigger)"
    );

    publish_x(&mut pub_lidar, 6.0);
    runtime.step(Duration::from_millis(1));
    assert_eq!(
        body_runs.load(Ordering::Relaxed),
        1,
        "lidar arriving must complete the alignment — cam's data was RETAINED \
         by the sync drain across the starved rounds"
    );
    assert_eq!(
        last_config.load(Ordering::Relaxed),
        5,
        "the fire reads config's latest flowed value"
    );
}

/// The watchdog survives the narrowed drain set: a STARVED
/// trigger input carrying `#[input(trigger, expect_within_ms = 5)]` still
/// trips its watchdog while the node never fires. `lidar` is silent for 30
/// simulated ms while cam + config flow — the sync node cannot align (0
/// fires, 0 body runs), yet `expect_within_missed_count` climbs (the
/// scheduler's per-step QoS window check runs regardless of fires; the
/// starved TRIGGER input's binding + watchdog registration survived the
/// trigger-set narrowing).
#[test]
#[serial]
fn test_starved_trigger_watchdog_trips_while_node_never_fires() {
    let body_runs = Arc::new(AtomicU64::new(0));
    let (config, factories) = ctx_graph(Box::new(SyncFuseWatchEntry::with_state(SyncFuseWatch {
        body_runs: Arc::clone(&body_runs),
        ..Default::default()
    })));
    let clock = Arc::new(VirtualClock::new());
    let mut runtime =
        GraphRuntime::build_for_test(config, factories, clock, 8).expect("build watch graph");
    let mgr = runtime.test_transport().expect("test transport parked");
    let mut pub_cam = mgr
        .create_publisher(TOPIC_CAM, MaxSliceLen::const_new(64), 0)
        .expect("pub cam");
    let mut pub_config = mgr
        .create_publisher(TOPIC_CONFIG, MaxSliceLen::const_new(64), 0)
        .expect("pub config");
    // NOTE: /s425/lidar never receives data (the starved trigger).

    for k in 1..=30u64 {
        publish_x(&mut pub_cam, k as f64);
        publish_x(&mut pub_config, 5.0);
        runtime.step(Duration::from_millis(1));
    }

    let handle = runtime.node_handle("fuse").expect("fuse handle");
    assert_eq!(
        handle.fire_count(),
        0,
        "the starved trigger must keep the sync node from ever firing"
    );
    assert_eq!(
        body_runs.load(Ordering::Relaxed),
        0,
        "no fire → no body run"
    );
    assert!(
        handle.expect_within_missed_count() > 0,
        "the starved trigger's `expect_within_ms = 5` watchdog must trip over \
         a 30 ms silence even though the node never fires — got {} misses",
        handle.expect_within_missed_count()
    );
    // The backlog guard's boundary, asserted rather than assumed: the guard is
    // scoped to per-message FIFO trigger inputs, and a Sync node has none —
    // `signal_sync_input` touches only `sync_input_timestamps`, never
    // `pending_data_count`. So a starved Sync trigger can never be suppressed
    // as backlog. This is an INVARIANT pin, not a regression test: the guard
    // structurally cannot make a Sync node's pending count non-zero.
    assert_eq!(
        handle.expect_within_backlogged_count(),
        0,
        "a Sync node carries no per-message FIFO backlog, so none of its \
         starved-trigger windows may be reported as backlog"
    );
}

/// DETERMINISM (Principle #7): two identical builds + identical
/// publish/step sequences produce byte-identical (body_runs, per-fire config
/// reads) — and both equal the HAND ORACLE [7, 7, 7] (not a self-compare).
#[test]
#[serial]
fn test_sync_ctx_fire_is_deterministic() {
    fn run() -> (u64, Vec<u64>) {
        let body_runs = Arc::new(AtomicU64::new(0));
        let last_config = Arc::new(AtomicU64::new(MISSING));
        let (config, factories) = ctx_graph(Box::new(SyncFuseCtxEntry::with_state(SyncFuseCtx {
            body_runs: Arc::clone(&body_runs),
            last_config: Arc::clone(&last_config),
            ..Default::default()
        })));
        let clock = Arc::new(VirtualClock::new());
        let mut runtime = GraphRuntime::build_for_test(config, factories, clock, 8)
            .expect("build ctx sync graph");
        let mgr = runtime.test_transport().expect("test transport parked");
        let mut pub_cam = mgr
            .create_publisher(TOPIC_CAM, MaxSliceLen::const_new(64), 0)
            .expect("pub cam");
        let mut pub_lidar = mgr
            .create_publisher(TOPIC_LIDAR, MaxSliceLen::const_new(64), 0)
            .expect("pub lidar");
        let mut pub_config = mgr
            .create_publisher(TOPIC_CONFIG, MaxSliceLen::const_new(64), 0)
            .expect("pub config");

        publish_x(&mut pub_config, 7.0);
        runtime.step(Duration::from_millis(1));
        let mut reads = Vec::new();
        for k in 1..=3u64 {
            publish_x(&mut pub_cam, k as f64);
            publish_x(&mut pub_lidar, k as f64);
            last_config.store(MISSING, Ordering::Relaxed);
            runtime.step(Duration::from_millis(1));
            reads.push(last_config.load(Ordering::Relaxed));
        }
        (body_runs.load(Ordering::Relaxed), reads)
    }
    let r1 = run();
    let r2 = run();
    assert_eq!(r1, r2, "two runs must be byte-identical (Principle #7)");
    assert_eq!(
        r1,
        (3, vec![7, 7, 7]),
        "and both must equal the HAND ORACLE: 3 fires, each reading the held 7"
    );
}
