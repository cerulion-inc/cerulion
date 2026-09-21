// SPDX-License-Identifier: AGPL-3.0-only
//! The generic walker → archetype → Rerun `memory` path, WITHOUT a
//! transport. Proves the "one adapter" ingest (decode a frame → dispatch by
//! schema name → log) records into Rerun's in-memory store, and that the
//! direct archetype builders record too.
//!
//! No iceoryx2 here — frames are hand-built and decoded by the built-in frame
//! walker, so these run in the fast parallel-safe path.

use cerulion_core::codegen::layout::LayoutResolver;
use cerulion_core::codegen::{parse_rosmsg, MessageSchema};
use cerulion_core::message::ShmMessage;
use cerulion_core::wire::WireHeader;
use cerulion_viz::archetype::{log_field_dump, log_points3d};
use cerulion_viz::pointcloud::DecodedCloud;
use cerulion_viz::schema_registry::builtin_walker;
use native_ros2_messages::geometry_msgs::Twist;

// `MemorySinkStorage` lives in `rerun::sink` (not re-exported at the crate
// root in 0.34); `num_msgs()` flushes the stream and returns the stored count.
fn memory() -> (rerun::RecordingStream, rerun::sink::MemorySinkStorage) {
    rerun::RecordingStreamBuilder::new("go2_test")
        .recording_id("go2_arch_test")
        .memory()
        .expect("memory sink")
}

/// Build a `geometry_msgs/Twist` wire frame (two fixed `Vector3`s = 6 f64,
/// fixed-only) with the given linear/angular components, using the real
/// layout engine so the walker decodes it byte-correctly.
fn build_twist_frame(linear: [f64; 3], angular: [f64; 3]) -> Vec<u8> {
    let mut schemas: Vec<MessageSchema> = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(s);
        }
    }
    let (mut resolver, _) = LayoutResolver::new(schemas);
    let layout = resolver
        .layout_of("geometry_msgs/Twist")
        .expect("Twist is built-in");
    let lin_off = layout
        .fixed_fields
        .iter()
        .find(|f| f.name == "linear")
        .expect("linear field")
        .offset;
    let ang_off = layout
        .fixed_fields
        .iter()
        .find(|f| f.name == "angular")
        .expect("angular field")
        .offset;

    let mut payload = vec![0u8; layout.fixed_size];
    for (i, v) in linear.iter().enumerate() {
        payload[lin_off + i * 8..lin_off + i * 8 + 8].copy_from_slice(&v.to_le_bytes());
    }
    for (i, v) in angular.iter().enumerate() {
        payload[ang_off + i * 8..ang_off + i * 8 + 8].copy_from_slice(&v.to_le_bytes());
    }

    let mut frame = vec![0u8; WireHeader::SIZE];
    let header = WireHeader {
        schema_hash: <Twist as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + layout.fixed_size) as u32,
        offset_table_count: 0,
        sequence: 0,
        timestamp_ns: 42_000,
    };
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

#[test]
fn log_points3d_direct_records_to_memory() {
    let (rec, storage) = memory();
    let cloud = DecodedCloud {
        positions: vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
        colors: Some(vec![[255, 0, 0, 255], [0, 255, 0, 255]]),
        skipped: 0,
    };
    log_points3d(&rec, "world/go2/lidar", 1_000, &cloud);
    rec.flush_blocking().expect("flush memory sink");
    assert!(
        storage.num_msgs() >= 1,
        "Points3D log must record (got {})",
        storage.num_msgs()
    );
}

// The generic-dispatch arm that stood here is GONE with
// `archetype::log_frame_value`. It walked the SAME `geometry_msgs/Twist` frame
// (`[0.5,0,0]` / `[0,0,0.2]`) against the SAME "EXACTLY 6 component messages"
// oracle as `sink_dispatch_test::twist_dispatches_to_exactly_six_scalars`, which
// drives the PRODUCTION `dispatch_frame` and additionally pins each component's
// VALUE at its own named scalar path — so the oracle survives strictly stronger
// on the path production uses.

#[test]
fn field_dump_fallback_records_for_any_frame() {
    let walker = builtin_walker();
    let frame = build_twist_frame([1.0, 1.0, 1.0], [0.0, 0.0, 0.0]);
    let fv = walker.walk("geometry_msgs/Twist", &frame).expect("walk");

    let (rec, storage) = memory();
    // Direct field-dump (the unknown-schema fallback path).
    log_field_dump(&rec, "world/inspect", 7, &fv);
    rec.flush_blocking().expect("flush memory sink");
    assert!(storage.num_msgs() >= 1, "field dump must record");
}
