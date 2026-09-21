// SPDX-License-Identifier: AGPL-3.0-only
//! The TF end-to-end: the producer → tf-sink mapping → Rerun
//! `memory()` sink shows the full tree, over a REAL graph + iceoryx2.
//!
//! **Lives HERE, not beside the producer.** This test was
//! `examples/go2/nodes/go2_tf_source/tests/e2e_test.rs`, which forced the ROBOT
//! node crate to carry `cerulion_viz` + `rerun` DEV-dependencies — a rerun MSRV
//! floor and a 257-crate Rerun SDK edge on a crate that ships to the robot,
//! contradicting the rule that no viz/rerun runs on the robot (the robot ships
//! RAW frames and the desk decodes + renders). Inverting the edge — the DESK
//! crate dev-depends on the producer, instead of the producer dev-depending on
//! the desk stack — keeps the exact same oracles while leaving `go2_tf_source`
//! with ZERO rerun edges even under `-e normal,dev`. The `rerun-leanness` CI
//! job pins both halves.
//!
//! `go2_tf_source` (period node) publishes `odom → base` on `/tf` and the
//! static mount table on `/tf_static`. Two drain-all closure consumers apply
//! the `cerulion_viz::tf` mapping (decode via the pure `go2_tf` codec, log to a
//! Rerun `memory()` sink) and capture the decoded transforms. Asserts:
//!
//! - the `/tf` stream is the `odom → base` identity (the placeholder stub),
//! - every `/tf` transform carries the HAND-ORACLE stamp of its fire (that is,
//!   `Time::from_ns(k × 100ms)` from the known VirtualClock start, the
//!   sequence crossing the 1 s boundary so a sec/nanosec swap cannot pass —
//!   the determinism check alone is a self-compare for stamps, since both
//!   runs start a fresh clock from 0),
//! - the `/tf_static` stream is exactly `[base→lidar, base→camera]` repeating
//!   with the documented mount offsets and the latched-static 0/0 stamps (a
//!   hand oracle),
//! - the memory store recorded the tree, and
//! - determinism (Principle #7): two full runs capture BYTE-IDENTICAL
//!   sequences (stamps included).
//!
//! Single `#[test]` body (the shared Rerun stream + one isolated transport at
//! a time — the lidar e2e caution); the two runs are sequential inside it.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::node::{
    BackpressurePolicy, ClosureNodeEntry, InputMeta, MacroPolicy, NodeEntry, NodeInfo,
};
use cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH;
use cerulion_core::graph::{parse_graph_raw, GraphRuntime};
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::WireHeader;
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::tf::{log_transforms, transforms_from_frame, UnknownFrameLog};
use go2_tf::{TfTransform, IDENTITY_QUAT};
use go2_tf_source::Go2TfSourceEntry;
use indexmap::IndexMap;
use native_ros2_messages::tf2_msgs::TFMessage;

const GRAPH_YAML: &str = r#"
name: go2_tf_loop
prefix: tfsrc
nodes:
  - id: source
    type: go2_tf_source
    outputs:
      - name: tf
        schema: tf2_msgs/TFMessage
        topic: /tf
      - name: tf_static
        schema: tf2_msgs/TFMessage
        topic: /tf_static
  - id: viz_tf
    type: tf_consumer
    inputs:
      - name: tf
        source: /tf
  - id: viz_static
    type: tf_consumer
    inputs:
      - name: tf
        source: /tf_static
"#;

/// A drain-all closure consumer that captures every decoded transform of its
/// input into `sink` and logs it to the shared Rerun memory sink.
fn capture_consumer(sink: Arc<Mutex<Vec<TfTransform>>>) -> ClosureNodeEntry {
    let walker = builtin_walker();
    let mut unknown = UnknownFrameLog::new();
    let info = NodeInfo::with_meta(
        vec![InputMeta {
            name: "tf".to_string(),
            schema_hash: <TFMessage as ShmMessage>::SCHEMA_HASH,
            trigger: true,
            depth: DEFAULT_CONSUMER_DEPTH,
            backpressure: BackpressurePolicy::DropOldest,
            expect_within_ms: None,
        }],
        vec![],
    )
    .with_policy(MacroPolicy::DataTrigger {
        input_name: "tf".to_string(),
    });
    ClosureNodeEntry::new(info, move |ctx| {
        let rec = cerulion_viz::stream::recording_stream();
        let sub = ctx.subscriber("tf").expect("tf subscriber wired");
        let _drained = sub.try_receive(|msg| {
            let h = msg.header();
            let payload = msg.payload();
            let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
            h.write_to_buf(&mut frame[..WireHeader::SIZE]);
            frame[WireHeader::SIZE..].copy_from_slice(payload);
            let (ts, records) =
                transforms_from_frame(&walker, &frame).expect("producer frame decodes");
            if let Some(r) = &rec {
                log_transforms(r, ts, &records, false, &mut unknown);
            }
            sink.lock().expect("sink mutex").extend(records);
        })?;
        Ok(())
    })
    .with_unified_drain(false)
}

/// One full run: build the producer→consumers graph, drive `steps` × 100 ms,
/// return the captured (`/tf`, `/tf_static`) transform sequences. Asserts the
/// memory store recorded something along the way.
fn run_once(tag: &str, steps: usize) -> (Vec<TfTransform>, Vec<TfTransform>) {
    cerulion_viz::stream::reset_for_test();
    let (rec, storage) = rerun::RecordingStreamBuilder::new("go2_test")
        .recording_id("go2_tf_loop_test")
        .memory()
        .expect("memory sink");
    cerulion_viz::stream::set_stream(rec.clone());

    let clock = Arc::new(VirtualClock::new());
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: format!("go2_tf_loop_{tag}"),
            clock: clock.clone(),
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated e2e transport");

    let seen_tf = Arc::new(Mutex::new(Vec::<TfTransform>::new()));
    let seen_static = Arc::new(Mutex::new(Vec::<TfTransform>::new()));

    let config = parse_graph_raw(GRAPH_YAML).expect("graph YAML parses");
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("source".to_string(), Box::new(Go2TfSourceEntry::new()));
    factories.insert(
        "viz_tf".to_string(),
        Box::new(capture_consumer(Arc::clone(&seen_tf))),
    );
    factories.insert(
        "viz_static".to_string(),
        Box::new(capture_consumer(Arc::clone(&seen_static))),
    );
    let mut rt = GraphRuntime::build(config, factories, &mgr, clock.clone())
        .expect("producer→consumers graph builds");

    let baseline = storage.num_msgs();
    for _ in 0..steps {
        rt.step(Duration::from_millis(100));
    }
    rec.flush_blocking().expect("flush");
    assert!(
        storage.num_msgs() > baseline,
        "the memory store must record the produced transforms"
    );
    rt.shutdown();

    let tf = seen_tf.lock().expect("seen_tf mutex").clone();
    let statics = seen_static.lock().expect("seen_static mutex").clone();
    (tf, statics)
}

#[test]
fn producer_feeds_the_full_tree_and_is_deterministic() {
    // ---- Stamp HAND ORACLE. Derivation:
    // `run_once` starts a fresh `VirtualClock` at 0 ns and the node is
    // `period_ms = 100` with `next_fire = now + interval` at registration
    // (= 100 ms). `GraphRuntime::step(100ms)` advances the clock BEFORE any
    // node fires (`Scheduler::begin_step` advance-then-fire), so fire k
    // (1-based) ticks at `now_ns = k × 100_000_000` and stamps the /tf
    // transform with `Time::from_ns(k × 100_000_000)`. The table is
    // HAND-WRITTEN (never re-derived via the ns→(sec,nanosec) split under
    // test), because the determinism check below is a SELF-COMPARE for
    // stamps — both runs start a fresh clock from 0, so a deterministically
    // wrong stamp (swapped sec/nanosec, ns-truncation, ms-vs-ns, a wall-clock
    // read instead of `self.now_ns()`) would agree across runs. 12 fires
    // deliberately CROSS the 1 s boundary (fires 10..12 land at 1.0/1.1/1.2 s)
    // so a sec/nanosec swap cannot pass anywhere in the sequence.
    const EXPECTED_TF_STAMPS: [(i32, u32); 12] = [
        (0, 100_000_000), // fire 1 @ 0.1 s
        (0, 200_000_000), // fire 2 @ 0.2 s
        (0, 300_000_000), // fire 3 @ 0.3 s
        (0, 400_000_000), // fire 4 @ 0.4 s
        (0, 500_000_000), // fire 5 @ 0.5 s
        (0, 600_000_000), // fire 6 @ 0.6 s
        (0, 700_000_000), // fire 7 @ 0.7 s
        (0, 800_000_000), // fire 8 @ 0.8 s
        (0, 900_000_000), // fire 9 @ 0.9 s
        (1, 0),           // fire 10 @ exactly 1 s — the second boundary
        (1, 100_000_000), // fire 11 @ 1.1 s
        (1, 200_000_000), // fire 12 @ 1.2 s
    ];
    const STEPS: usize = EXPECTED_TF_STAMPS.len();

    let (tf_a, static_a) = run_once("a", STEPS);
    let (tf_b, static_b) = run_once("b", STEPS);

    // ---- Geometry oracle: /tf is the odom→base identity stub.
    assert!(
        !tf_a.is_empty(),
        "the producer must publish at least one /tf"
    );
    for t in &tf_a {
        assert_eq!(t.frame_id, "odom");
        assert_eq!(t.child_frame_id, "base");
        assert_eq!(t.translation, [0.0, 0.0, 0.0]);
        assert_eq!(t.rotation, IDENTITY_QUAT);
    }

    // ---- Stamp oracle: one /tf transform per fire, each stamped with the
    // node clock at that fire (see the hand-oracle derivation above). The
    // exact-count assert makes the index↔fire mapping rigorous (advance-then-
    // fire + same-step DAG delivery: producer fires at level 0, the
    // data-trigger consumer drains at level 1 of the SAME step — the
    // `polled_vs_live` / barrier-gate precedent).
    assert_eq!(
        tf_a.len(),
        STEPS,
        "one captured /tf transform per producer fire ({STEPS} steps → {STEPS} fires)"
    );
    for (i, t) in tf_a.iter().enumerate() {
        let (want_sec, want_nanosec) = EXPECTED_TF_STAMPS[i];
        assert_eq!(
            (t.stamp_sec, t.stamp_nanosec),
            (want_sec, want_nanosec),
            "/tf transform {i} must carry the hand-oracle stamp of fire {fire} \
             (node clock {fire}00 ms; Time::from_ns split)",
            fire = i + 1,
        );
    }

    // ---- Geometry oracle: /tf_static is [base→lidar, base→camera] repeating,
    // exactly two mounts per producer fire.
    assert_eq!(
        static_a.len(),
        2 * tf_a.len(),
        "each fire publishes /tf (1 transform) + /tf_static (2 mounts)"
    );
    for pair in static_a.as_chunks::<2>().0 {
        assert_eq!(pair[0].frame_id, "base");
        assert_eq!(pair[0].child_frame_id, "lidar");
        assert_eq!(pair[0].translation, [0.17, 0.0, 0.11]);
        assert_eq!(pair[0].rotation, IDENTITY_QUAT);
        assert_eq!(pair[1].child_frame_id, "camera");
        assert_eq!(pair[1].translation, [0.27, 0.0, 0.05]);
        // Latched-static convention: the mounts carry stamp 0/0 (the wire
        // stamp is the sink's timeline) — pinned so a node-clock stamp
        // leaking into the static table is caught.
        assert_eq!((pair[0].stamp_sec, pair[0].stamp_nanosec), (0, 0));
        assert_eq!((pair[1].stamp_sec, pair[1].stamp_nanosec), (0, 0));
    }

    // ---- Determinism (Principle #7): two runs are byte-identical (stamps too).
    assert_eq!(tf_a, tf_b, "/tf capture must be deterministic across runs");
    assert_eq!(
        static_a, static_b,
        "/tf_static capture must be deterministic across runs"
    );
}
