// SPDX-License-Identifier: AGPL-3.0-only
//! The DRAIN-ALL `/tf` read pattern over real iceoryx2.
//!
//! `/tf` is an accumulate-all topic — a latest-only read would drop sibling
//! transforms sharing the topic (the `multi_publisher` /tf contract). This
//! pins the interim read shape a data-trigger consumer uses today: a
//! `ClosureNodeEntry` that OPTS OUT of the unified trigger drain
//! (`.with_unified_drain(false)`) and reads its body subscriber
//! ACCUMULATE-ALL via `ctx.subscriber("tf").try_receive(...)`, decoding each
//! queued frame through the generic frame walker (`transforms_from_frame`).
//!
//! N distinct-transform frames (each targeting a DISTINCT known-frame entity —
//! see `nth_transform` for why that also keeps the Rerun chunk count exact)
//! are published BEFORE the consumer steps: a correct drain-all captures ALL N
//! in order (a hand oracle); a latest-only read would capture only the last. The unified drain's frozen slot serves
//! `try_view` (latest-wins) only, which is exactly why the production macro
//! sinks (latest-only) do not cover the multi-writer case — see the sink
//! module docs.
//!
//! Single `#[test]` body (the lidar e2e caution): the drain-all closure logs
//! into the process-shared Rerun `memory()` sink, so only one isolated
//! transport + one installed stream may exist at a time.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use cerulion_core::clock::VirtualClock;
use cerulion_core::graph::config::{GraphConfig, InputDef, NodeDef};
use cerulion_core::graph::node::{
    BackpressurePolicy, ClosureNodeEntry, InputMeta, MacroPolicy, NodeEntry, NodeInfo,
};
use cerulion_core::graph::topology::DEFAULT_CONSUMER_DEPTH;
use cerulion_core::graph::GraphRuntime;
use cerulion_core::message::ShmMessage;
use cerulion_core::transport::{TransportConfig, TransportManager};
use cerulion_core::wire::{MaxSliceLen, WireHeader};
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::tf::{log_transforms, transforms_from_frame, UnknownFrameLog};
use go2_tf::{encode_tf_transforms, TfTransform, IDENTITY_QUAT};
use indexmap::IndexMap;
use native_ros2_messages::tf2_msgs::TFMessage;

const TF_TOPIC: &str = "/tf";

/// Build a full TFMessage wire frame (crib: `tfmessage_oracle_frame`).
fn build_tf_frame(seq: u32, ts: u64, transforms_bytes: &[u8]) -> Vec<u8> {
    let offset = (TFMessage::WIRE_FIXED_SIZE + 8 * TFMessage::VARIABLE_FIELD_COUNT) as u32;
    let length = transforms_bytes.len() as u32;
    let mut payload = Vec::with_capacity(offset as usize + transforms_bytes.len());
    payload.extend_from_slice(&offset.to_le_bytes());
    payload.extend_from_slice(&length.to_le_bytes());
    payload.extend_from_slice(transforms_bytes);
    let header = WireHeader {
        schema_hash: TFMessage::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + TFMessage::WIRE_FIXED_SIZE) as u32,
        offset_table_count: TFMessage::VARIABLE_FIELD_COUNT as u32,
        sequence: seq,
        timestamp_ns: ts,
    };
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// The i-th frame carries ONE distinct transform at `[i,0,0]` with a DISTINCT
/// child frame, so:
///
/// - a drain-all read captures the full ordered sequence a latest-only read
///   cannot fake (latest-only yields only frame N-1's record), and
/// - each frame logs to a DIFFERENT entity path (all from the known-frame
///   table — no unknown-frame warns), which keeps the store-count assertion
///   EXACT: Rerun's micro-batcher compacts multiple rows logged to the SAME
///   entity within one flush window into ONE chunk, and
///   `MemorySinkStorage::num_msgs()` counts CHUNKS — N same-entity rows would
///   correctly count 1. Distinct entities → one
///   chunk per drained frame.
fn nth_transform(i: usize) -> TfTransform {
    // (parent, child) pairs all resolving through the KNOWN-frame table to
    // four DISTINCT entity paths: world/odom, world/odom/base,
    // world/odom/base/lidar, world/odom/base/camera.
    const PAIRS: [(&str, &str); 4] = [
        ("map", "odom"),
        ("odom", "base"),
        ("base", "lidar"),
        ("base", "camera"),
    ];
    let (parent, child) = PAIRS[i % PAIRS.len()];
    TfTransform::new(
        parent,
        child,
        [i as f64, 0.0, 0.0],
        IDENTITY_QUAT,
        i as i32,
        0,
    )
}

#[test]
fn drain_all_consumer_captures_every_queued_tf_frame() {
    const N: usize = 4;
    // The exact chunk-count assertion below requires every frame to hit a
    // DISTINCT entity (see nth_transform); the known-frame table gives 4,
    // so a future N bump must extend the table first (compile-time guard —
    // a runtime assert on consts trips clippy::assertions_on_constants).
    const _: () = assert!(N <= 4, "N > 4 would repeat entity paths");

    // Install a fresh in-memory Rerun sink (the closure logs into it).
    cerulion_viz::stream::reset_for_test();
    let (rec, storage) = rerun::RecordingStreamBuilder::new("go2_test")
        .recording_id("go2_tf_drain_test")
        .memory()
        .expect("memory sink");
    cerulion_viz::stream::set_stream(rec.clone());

    let clock = Arc::new(VirtualClock::new());
    let mgr = TransportManager::init_for_test(
        TransportConfig {
            node_name: "go2_tf_drain_e2e".to_string(),
            clock: clock.clone(),
            subscriber_buffer_size: 16,
            network: None,
        },
        cerulion_core::testing::iceoryx_test_config(),
    )
    .expect("init isolated e2e transport");

    // The drain-all consumer: every queued /tf frame is decoded via the walker
    // and its transforms captured (accumulate-all).
    let seen = Arc::new(Mutex::new(Vec::<TfTransform>::new()));
    let seen_c = Arc::clone(&seen);
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
    let consumer = ClosureNodeEntry::new(info, move |ctx| {
        let rec = cerulion_viz::stream::recording_stream();
        let sub = ctx.subscriber("tf").expect("tf subscriber wired");
        // Drain ALL queued /tf frames this tick (accumulate-all).
        let _drained = sub.try_receive(|msg| {
            let h = msg.header();
            let payload = msg.payload();
            let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
            h.write_to_buf(&mut frame[..WireHeader::SIZE]);
            frame[WireHeader::SIZE..].copy_from_slice(payload);
            let (ts, records) = transforms_from_frame(&walker, &frame)
                .expect("every published TFMessage frame decodes");
            if let Some(r) = &rec {
                log_transforms(r, ts, &records, false, &mut unknown);
            }
            seen_c.lock().expect("seen mutex").extend(records);
        })?;
        Ok(())
    })
    .with_label("tf_drain_all")
    // Accumulate-all read: the unified drain's frozen slot serves try_view
    // (latest-wins) only, so unified would silently lose every frame but the
    // newest. Keep DrainSource::Separate.
    .with_unified_drain(false);

    let config = GraphConfig {
        level_assignments: None,
        process_groups: Default::default(),
        process_group_order: Default::default(),
        multi_publisher_topics: Vec::new(),
        network: None,
        name: None,
        identity: "go2_tf_drain_e2e".to_string(),
        prefix: "drain".to_string(),
        nodes: vec![NodeDef {
            ros2: None,
            id: "tf_viz".to_string(),
            node_type: "tf_drain_consumer".to_string(),
            inputs: vec![InputDef {
                name: "tf".to_string(),
                source: TF_TOPIC.to_string(),
            }],
            outputs: vec![],
        }],
    };
    let mut factories: IndexMap<String, Box<dyn NodeEntry>> = IndexMap::new();
    factories.insert("tf_viz".to_string(), Box::new(consumer));
    let mut rt = GraphRuntime::build(config, factories, &mgr, clock.clone())
        .expect("drain-all graph builds (absolute external /tf source)");

    // Raw external publisher on /tf (the zenoh-ingress re-inject shape).
    let mut publisher = mgr
        .create_publisher(TF_TOPIC, MaxSliceLen::const_new(1 << 16), 0)
        .expect("external /tf publisher attaches");

    // Publish N distinct frames, THEN one step: a correct drain-all sees all N.
    let mut oracle = Vec::new();
    for i in 0..N {
        let t = nth_transform(i);
        oracle.push(t.clone());
        let frame = build_tf_frame(
            i as u32,
            1_000 * (i as u64 + 1),
            &encode_tf_transforms(&[t]),
        );
        publisher
            .publish_raw(&frame)
            .expect("publish_raw /tf frame");
    }
    // All N frames are queued; the consumer fires and drains them all. Step a
    // few times so a delivery lag can't leave the fire un-triggered (whichever
    // step fires drains every queued frame — extra steps see no new data).
    let store_before = storage.num_msgs();
    for _ in 0..3 {
        rt.step(Duration::from_millis(10));
    }
    rec.flush_blocking().expect("flush memory sink");

    // Accumulate-all: the consumer captured EVERY frame's transform in order —
    // a latest-only read would have captured only nth_transform(N-1).
    assert_eq!(
        *seen.lock().expect("seen mutex"),
        oracle,
        "drain-all must capture all {N} queued /tf transforms in order (latest-only \
         would give only the last)"
    );
    // The memory store grew by one Transform3D CHUNK per drained frame. Exact
    // because each frame's transform targets a DISTINCT entity path (see
    // nth_transform): num_msgs() counts chunks, and Rerun's micro-batcher
    // compacts same-entity rows within a flush window into one chunk.
    assert_eq!(
        storage.num_msgs() - store_before,
        N,
        "one Transform3D chunk per drained /tf frame (distinct entities)"
    );

    rt.shutdown();
}
