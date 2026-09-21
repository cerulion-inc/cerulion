// SPDX-License-Identifier: AGPL-3.0-only
//! The generic `cerulion_viz::sink::dispatch_frame` path (walk-by-hash →
//! archetype table → Rerun `memory()` sink), WITHOUT a transport. Hand-built
//! wire frames are decoded by the built-in frame walker and dispatched by
//! schema; each test asserts against a HAND oracle (exact recorded-chunk
//! deltas), never a self-compare.
//!
//! No iceoryx2 here — frames are hand-built (crib: `archetype_memory_test` for
//! fixed-only messages, `tf_drain_all_test` for TFMessage, and the deleted
//! `rerun_camera_sink` / `rerun_lidar_sink` e2e builders for
//! CompressedImage / PointCloud2 — the media-path pins),
//! so these run in the fast parallel-safe path (each test owns its `memory()`
//! sink + `SinkState`).

use cerulion_core::codegen::layout::{LayoutResolver, WireLayout};
use cerulion_core::codegen::{parse_rosmsg, FrameValueKind, FrameWalker, MessageSchema};
use cerulion_core::message::ShmMessage;
use cerulion_core::shm_runtime::write_offset_entry;
use cerulion_core::wire::WireHeader;
use cerulion_viz::archetype::{
    box3d_parts, cloud_from_frame_value, declares_plottable_series, image_data_from_frame_value,
    occupancy_image, odometry_twist_scalars, planar_pose_of, pose_transform_parts,
    rotation_only_of, scalar_samples, scalars_from_frame_value, scan_element_arrays,
    scan_element_arrays_for_kind, spatial_sibling_series, BoxParts, ElementArrayScan,
    ElementGeometry, MAX_ELEMENT_INSTANCES,
};
use cerulion_viz::blueprint::{archetype_components, views_for_archetype, ViewKind};
use cerulion_viz::plot_rate::DUMP_REFRESH_FRAME_FLOOR;
use cerulion_viz::representation::{
    Representation, FORCED_DUMP_FRAME_FLOOR, FORCED_DUMP_MIN_INTERVAL_NS,
};
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::sink::{
    classify_schema, dispatch_frame, dispatch_or_stage, infer_archetype_from_shape,
    name_mapped_forces_ordered, route_for_input, ArchetypeKind, SinkState,
};
use cerulion_viz::skeleton::{rearm_skeleton_statics, Skeleton};
// The media table is gone — an input's entity is `world/<name>`.
const CLOUD_ENTITY: &str = "world/cloud";
const IMAGE_ENTITY: &str = "world/image";
use go2_tf::{encode_tf_transforms, TfTransform, IDENTITY_QUAT};
use native_ros2_messages::builtin_interfaces::Time;
use native_ros2_messages::geometry_msgs::{
    PolygonStamped, Pose, Pose2D, QuaternionStamped, Twist, Vector3, Wrench,
};
use native_ros2_messages::nav_msgs::{OccupancyGrid, Odometry};
use native_ros2_messages::radar_msgs::RadarTrack;
use native_ros2_messages::sensor_msgs::{CompressedImage, Image, JointState, PointCloud2};
use native_ros2_messages::std_msgs::Bool;
use native_ros2_messages::tf2_msgs::TFMessage;
use native_ros2_messages::vision_msgs::BoundingBox3D;
use tracing_test::traced_test;

fn memory() -> (rerun::RecordingStream, rerun::sink::MemorySinkStorage) {
    rerun::RecordingStreamBuilder::new("go2_test")
        .recording_id("go2_sink_dispatch_test")
        .memory()
        .expect("memory sink")
}

/// A memory sink whose batcher flushes on EVERY log, so `num_msgs`
/// counts `rec.log` CALLS exactly.
///
/// The default batcher is wall-clock timed, so a chunk delta reflects how long a
/// run took as much as what it logged — measured: two runs producing an
/// identical recording differed 60 vs 42. That makes the default sink unusable
/// as a render-side oracle, and it is why the earlier chunk asserts in this
/// file only ever compare runs of one shape.
fn memory_always_flush() -> (rerun::RecordingStream, rerun::sink::MemorySinkStorage) {
    rerun::RecordingStreamBuilder::new("go2_test")
        .recording_id("go2_sink_dispatch_rate_gate")
        .batcher_config(rerun::log::ChunkBatcherConfig::ALWAYS_TEST_ONLY)
        .memory()
        .expect("memory sink")
}

/// Drive ONE poll tick's worth of drained `frames` through the staging seam
/// EXACTLY as the node does: each frame → `dispatch_or_stage`, then flush the
/// single staged (newest replacing-kind) frame via `dispatch_frame`. Returns
/// the number of frames coalesced away this tick (the node folds this into
/// `SinkState::record_coalesced`).
fn drain_tick(
    rec: &rerun::RecordingStream,
    walker: &cerulion_core::codegen::FrameWalker,
    input: &str,
    frames: Vec<Vec<u8>>,
    state: &mut SinkState,
) -> u64 {
    let mut staged: Option<Vec<u8>> = None;
    let mut coalesced = 0u64;
    for f in frames {
        dispatch_or_stage(rec, walker, input, f, state, &mut staged, &mut coalesced);
    }
    if let Some(frame) = staged.take() {
        dispatch_frame(rec, walker, input, &frame, state);
    }
    coalesced
}

fn all_schemas() -> Vec<MessageSchema> {
    let mut schemas = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(s);
        }
    }
    schemas
}

/// Build a fixed-only message wire frame by writing `(field_path_offset, LE
/// bytes)` pairs into the fixed section, using the real layout engine so the
/// walker decodes it byte-correctly. `nested` locates a nested fixed field.
fn build_fixed_frame(qname: &str, schema_hash: u64, writes: &[(usize, Vec<u8>)]) -> Vec<u8> {
    build_fixed_frame_at(qname, schema_hash, writes, 42_000)
}

/// [`build_fixed_frame`] with an explicit wire timestamp.
fn build_fixed_frame_at(
    qname: &str,
    schema_hash: u64,
    writes: &[(usize, Vec<u8>)],
    timestamp_ns: u64,
) -> Vec<u8> {
    let (mut resolver, _) = LayoutResolver::new(all_schemas());
    let layout = resolver.layout_of(qname).expect("built-in schema");
    let mut payload = vec![0u8; layout.fixed_size];
    for (off, bytes) in writes {
        payload[*off..*off + bytes.len()].copy_from_slice(bytes);
    }
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    let header = WireHeader {
        schema_hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + layout.fixed_size) as u32,
        offset_table_count: 0,
        sequence: 0,
        timestamp_ns,
    };
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// Offset of a top-level fixed field within a schema's fixed section.
fn field_offset(qname: &str, field: &str) -> usize {
    let (mut resolver, _) = LayoutResolver::new(all_schemas());
    let layout = resolver.layout_of(qname).expect("schema");
    layout
        .fixed_fields
        .iter()
        .find(|f| f.name == field)
        .unwrap_or_else(|| panic!("field {field} of {qname}"))
        .offset
}

/// Build a `geometry_msgs/Twist` frame (two nested `Vector3`s = 6 f64).
fn build_twist(linear: [f64; 3], angular: [f64; 3]) -> Vec<u8> {
    build_twist_at(linear, angular, 42_000)
}

/// [`build_twist`] with an explicit publisher WIRE timestamp — what a test
/// feeding a STREAM of frames on one topic needs, since the series gate keys a plot
/// topic's sample rate on exactly that advancement.
fn build_twist_at(linear: [f64; 3], angular: [f64; 3], timestamp_ns: u64) -> Vec<u8> {
    let lin = field_offset("geometry_msgs/Twist", "linear");
    let ang = field_offset("geometry_msgs/Twist", "angular");
    let mut writes = Vec::new();
    for (i, v) in linear.iter().enumerate() {
        writes.push((lin + i * 8, v.to_le_bytes().to_vec()));
    }
    for (i, v) in angular.iter().enumerate() {
        writes.push((ang + i * 8, v.to_le_bytes().to_vec()));
    }
    build_fixed_frame_at(
        "geometry_msgs/Twist",
        <Twist as ShmMessage>::SCHEMA_HASH,
        &writes,
        timestamp_ns,
    )
}

/// Build a `geometry_msgs/Vector3` frame (3 f64, fixed-only) — an UNTABLED
/// schema whose bare `{x,y,z}` shape-infers to three Scalar plots (a velocity /
/// force / RPY, never a misleading point at the origin).
fn build_vector3(v: [f64; 3]) -> Vec<u8> {
    let writes: Vec<(usize, Vec<u8>)> = v
        .iter()
        .enumerate()
        .map(|(i, x)| (i * 8, x.to_le_bytes().to_vec()))
        .collect();
    build_fixed_frame(
        "geometry_msgs/Vector3",
        <Vector3 as ShmMessage>::SCHEMA_HASH,
        &writes,
    )
}

/// Build a fixed-only `geometry_msgs/Pose` frame (position `Point` 3×f64 +
/// orientation `Quaternion` 4×f64) — a NAME-mapped `Transform3D`.
fn build_pose(position: [f64; 3], orientation: [f64; 4]) -> Vec<u8> {
    let pos = field_offset("geometry_msgs/Pose", "position");
    let ori = field_offset("geometry_msgs/Pose", "orientation");
    let mut writes = Vec::new();
    for (i, v) in position.iter().enumerate() {
        writes.push((pos + i * 8, v.to_le_bytes().to_vec()));
    }
    for (i, v) in orientation.iter().enumerate() {
        writes.push((ori + i * 8, v.to_le_bytes().to_vec()));
    }
    build_fixed_frame(
        "geometry_msgs/Pose",
        <Pose as ShmMessage>::SCHEMA_HASH,
        &writes,
    )
}

/// Build a fixed-only `builtin_interfaces/Time` frame (`sec` i32 + `nanosec`
/// u32) — an UNMAPPED schema whose two top-level numerics shape-infer to a
/// scalar bag (2 plots).
fn build_time(sec: i32, nanosec: u32) -> Vec<u8> {
    let sec_off = field_offset("builtin_interfaces/Time", "sec");
    let nsec_off = field_offset("builtin_interfaces/Time", "nanosec");
    build_fixed_frame(
        "builtin_interfaces/Time",
        <Time as ShmMessage>::SCHEMA_HASH,
        &[
            (sec_off, sec.to_le_bytes().to_vec()),
            (nsec_off, nanosec.to_le_bytes().to_vec()),
        ],
    )
}

/// Build a fixed-only `std_msgs/Bool` frame (`data` bool) — an UNMAPPED schema
/// that infers to NOTHING (bools are not auto-plotted) → the field-dump
/// fallback.
fn build_bool(v: bool) -> Vec<u8> {
    build_fixed_frame(
        "std_msgs/Bool",
        <Bool as ShmMessage>::SCHEMA_HASH,
        &[(0, vec![v as u8])],
    )
}

/// Resolve a schema's wire layout via the same engine the generated readers
/// use (full built-in set so nested `Header` resolves).
fn layout_of(qname: &str) -> WireLayout {
    let (mut resolver, _) = LayoutResolver::new(all_schemas());
    resolver.layout_of(qname).expect("built-in schema")
}

/// Build a `sensor_msgs/CompressedImage` wire frame: empty `header`,
/// `format` = "jpeg", `data` = the given JPEG bytes (crib: the deleted
/// `rerun_camera_sink` e2e's builder). All three fields are variable (fixed
/// section is empty), declaration order header(0) / format(1) / data(2).
fn build_jpeg_frame(jpeg: &[u8], timestamp_ns: u64) -> Vec<u8> {
    let layout = layout_of("sensor_msgs/CompressedImage");
    assert_eq!(
        layout
            .variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["header", "format", "data"],
        "CompressedImage variable-field declaration order changed — update this builder"
    );
    let fixed = layout.fixed_size; // 0 (all fields variable)
    let table = layout.offset_table_bytes(); // 3 × 8 = 24

    let format = b"jpeg";
    let mut payload = vec![0u8; fixed + table];
    let format_off = (fixed + table) as u32;
    let data_off = format_off + format.len() as u32;
    write_offset_entry(&mut payload, fixed, 0, 0, 0); // header: empty
    write_offset_entry(&mut payload, fixed, 1, format_off, format.len() as u32);
    write_offset_entry(&mut payload, fixed, 2, data_off, jpeg.len() as u32);
    payload.extend_from_slice(format);
    payload.extend_from_slice(jpeg);

    let mut frame = vec![0u8; WireHeader::SIZE];
    let header = WireHeader {
        schema_hash: <CompressedImage as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: 3,
        sequence: 0,
        timestamp_ns,
    };
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

/// Build a RAW `sensor_msgs/Image` wire frame (un-encoded pixels): a 2×2 image
/// with fixed height/width/step set, the given `encoding`, and 12 raw pixel
/// bytes (a valid rgb8 buffer; ignored on the degrade path). Variable
/// declaration order header(0) / encoding(1) / data(2).
fn build_raw_image_frame(encoding: &str, timestamp_ns: u64) -> Vec<u8> {
    let layout = layout_of("sensor_msgs/Image");
    assert_eq!(
        layout
            .variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["header", "encoding", "data"],
        "Image variable-field declaration order changed — update this builder"
    );
    let fixed = layout.fixed_size;
    let table = layout.offset_table_bytes(); // 3 × 8 = 24
    let fixed_off = |name: &str| {
        layout
            .fixed_fields
            .iter()
            .find(|f| f.name == name)
            .unwrap_or_else(|| panic!("Image has no fixed field '{name}'"))
            .offset
    };

    // A 2×2 image: 12 raw pixel bytes (deliberately NOT a JPEG stream).
    let encoding = encoding.as_bytes();
    let pixels: Vec<u8> = (0..12u8).collect();
    let mut payload = vec![0u8; fixed + table];
    let put_u32 = |buf: &mut [u8], off: usize, v: u32| {
        buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
    };
    put_u32(&mut payload, fixed_off("height"), 2);
    put_u32(&mut payload, fixed_off("width"), 2);
    put_u32(&mut payload, fixed_off("step"), 6);
    let encoding_off = (fixed + table) as u32;
    let data_off = encoding_off + encoding.len() as u32;
    write_offset_entry(&mut payload, fixed, 0, 0, 0); // header: empty
    write_offset_entry(&mut payload, fixed, 1, encoding_off, encoding.len() as u32);
    write_offset_entry(&mut payload, fixed, 2, data_off, pixels.len() as u32);
    payload.extend_from_slice(encoding);
    payload.extend_from_slice(&pixels);

    let mut frame = vec![0u8; WireHeader::SIZE];
    let header = WireHeader {
        schema_hash: <Image as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: 3,
        sequence: 0,
        timestamp_ns,
    };
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

/// XYZI packed point stride (3×f32 + intensity f32) — the deleted
/// `rerun_lidar_sink` e2e's convention.
const POINT_STEP: u32 = 16;

/// Build a `sensor_msgs/PointCloud2` wire frame: `points` as XYZI float32
/// packed at [`POINT_STEP`], empty `header`/`fields` (the current-producer
/// convention — the codec infers the XYZI layout from point_step), stamped
/// `timestamp_ns` (crib: the deleted `rerun_lidar_sink` e2e's builder).
fn build_cloud_frame(points: &[[f32; 4]], timestamp_ns: u64) -> Vec<u8> {
    let layout = layout_of("sensor_msgs/PointCloud2");
    assert_eq!(
        layout
            .variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["header", "fields", "data"],
        "PointCloud2 variable-field declaration order changed — update this builder"
    );
    let fixed = layout.fixed_size;
    let table = layout.offset_table_bytes(); // 3 variable fields × 8 = 24
    let n = points.len();
    let fixed_off = |name: &str| {
        layout
            .fixed_fields
            .iter()
            .find(|f| f.name == name)
            .unwrap_or_else(|| panic!("PointCloud2 has no fixed field '{name}'"))
            .offset
    };

    // Point data blob.
    let mut data = vec![0u8; n * POINT_STEP as usize];
    for (i, p) in points.iter().enumerate() {
        let b = i * POINT_STEP as usize;
        data[b..b + 4].copy_from_slice(&p[0].to_le_bytes());
        data[b + 4..b + 8].copy_from_slice(&p[1].to_le_bytes());
        data[b + 8..b + 12].copy_from_slice(&p[2].to_le_bytes());
        data[b + 12..b + 16].copy_from_slice(&p[3].to_le_bytes());
    }

    // Fixed section + offset table.
    let mut payload = vec![0u8; fixed + table];
    let put_u32 = |buf: &mut [u8], off: usize, v: u32| {
        buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
    };
    put_u32(&mut payload, fixed_off("height"), 1);
    put_u32(&mut payload, fixed_off("width"), n as u32);
    put_u32(&mut payload, fixed_off("point_step"), POINT_STEP);
    put_u32(&mut payload, fixed_off("row_step"), POINT_STEP * n as u32);
    payload[fixed_off("is_dense")] = 1;

    // Variable fields in declaration order: header(0), fields(1), data(2).
    let data_off = (fixed + table) as u32;
    write_offset_entry(&mut payload, fixed, 0, 0, 0); // header: empty
    write_offset_entry(&mut payload, fixed, 1, 0, 0); // fields: empty
    write_offset_entry(&mut payload, fixed, 2, data_off, data.len() as u32);
    payload.extend_from_slice(&data);

    let mut frame = vec![0u8; WireHeader::SIZE];
    let header = WireHeader {
        schema_hash: <PointCloud2 as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: 3,
        sequence: 0,
        timestamp_ns,
    };
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

/// Build a full TFMessage wire frame (crib: `tf_drain_all_test::build_tf_frame`).
fn build_tf(seq: u32, ts: u64, transforms_bytes: &[u8]) -> Vec<u8> {
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

/// Build a `nav_msgs/Odometry` wire frame with empty `header` / `child_frame_id`
/// (both variable) and the fixed `pose.pose.{position,orientation}` +
/// `twist.twist.{linear,angular}` filled from hand values (the covariance
/// sub-arrays stay zeroed). `pose` / `twist` are all-fixed nested structs in the
/// fixed section; within each, ROS declaration order is packed (Point 3×f64 then
/// Quaternion 4×f64; Vector3 then Vector3), matching the packed native wire
/// format that `build_twist` also relies on — the content assertion in the test
/// pins that the offsets are right.
fn build_odometry_frame(
    position: [f64; 3],
    orientation: [f64; 4],
    linear: [f64; 3],
    angular: [f64; 3],
    timestamp_ns: u64,
) -> Vec<u8> {
    let layout = layout_of("nav_msgs/Odometry");
    assert_eq!(
        layout
            .variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["header", "child_frame_id"],
        "Odometry variable-field declaration order changed — update this builder"
    );
    let fixed = layout.fixed_size;
    let table = layout.offset_table_bytes(); // 2 variable fields × 8 = 16
    let fixed_off = |name: &str| {
        layout
            .fixed_fields
            .iter()
            .find(|f| f.name == name)
            .unwrap_or_else(|| panic!("Odometry has no fixed field '{name}'"))
            .offset
    };
    // pose = PoseWithCovariance { pose: Pose @0, .. }; Pose = { position: Point
    // (3 f64) @0, orientation: Quaternion (4 f64) @24 }.
    let pose_off = fixed_off("pose");
    // twist = TwistWithCovariance { twist: Twist @0, .. }; Twist = { linear:
    // Vector3 (3 f64) @0, angular: Vector3 (3 f64) @24 }.
    let twist_off = fixed_off("twist");

    let mut payload = vec![0u8; fixed + table];
    let put_f64 = |buf: &mut [u8], off: usize, v: f64| {
        buf[off..off + 8].copy_from_slice(&v.to_le_bytes());
    };
    for (i, v) in position.iter().enumerate() {
        put_f64(&mut payload, pose_off + i * 8, *v);
    }
    for (i, v) in orientation.iter().enumerate() {
        put_f64(&mut payload, pose_off + 24 + i * 8, *v);
    }
    for (i, v) in linear.iter().enumerate() {
        put_f64(&mut payload, twist_off + i * 8, *v);
    }
    for (i, v) in angular.iter().enumerate() {
        put_f64(&mut payload, twist_off + 24 + i * 8, *v);
    }
    // Variable fields header(0) + child_frame_id(1), both empty.
    write_offset_entry(&mut payload, fixed, 0, 0, 0);
    write_offset_entry(&mut payload, fixed, 1, 0, 0);

    let mut frame = vec![0u8; WireHeader::SIZE];
    let header = WireHeader {
        schema_hash: <Odometry as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: 2,
        sequence: 0,
        timestamp_ns,
    };
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

/// The exact per-component hand oracle for `build_twist([0.5,0,0],[0,0,0.2])`
/// — names are entity-relative component paths, values the distinctive
/// hand-set floats. A linear/angular swap fails on BOTH pairs.
fn twist_scalar_oracle() -> Vec<(String, f64)> {
    vec![
        ("linear/x".to_string(), 0.5),
        ("linear/y".to_string(), 0.0),
        ("linear/z".to_string(), 0.0),
        ("angular/x".to_string(), 0.0),
        ("angular/y".to_string(), 0.0),
        ("angular/z".to_string(), 0.2),
    ]
}

#[test]
fn twist_dispatches_to_exactly_six_scalars() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let frame = build_twist([0.5, 0.0, 0.0], [0.0, 0.0, 0.2]);

    // Value pin: the extraction seam that feeds each
    // `log_scalar` call must land the distinctive values on the CORRECT
    // component names — a linear/angular swap passes a bare count assert.
    let fv = walker.walk_by_hash(&frame).expect("walk Twist");
    assert_eq!(
        scalars_from_frame_value(&fv),
        twist_scalar_oracle(),
        "each component value must land on its own named scalar path"
    );

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "twist", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    // Six Scalars messages — one per Twist component at its own entity path
    // (linear/{x,y,z} + angular/{x,y,z}), each a distinct chunk. A regression
    // to the field-dump fallback would record exactly 1.
    assert_eq!(storage.num_msgs() - baseline, 6);
}

// ---- The plot-sample rate gate, through the PRODUCTION dispatch ----

/// Dispatch `count` Twist frames on one input, spaced `period_ns` of WIRE time
/// apart, and report `(admitted, decimated)` — `admitted` derived from the
/// production `plot_frames_decimated()` observable so it can never disagree with
/// what the sink actually did.
fn dispatch_twist_stream(state: &mut SinkState, count: u64, period_ns: u64) -> (u64, u64) {
    let frames: Vec<(u64, f64)> = (0..count).map(|i| (i * period_ns, i as f64)).collect();
    dispatch_twist_frames(state, &frames).0
}

/// Dispatch an EXPLICIT `(wire timestamp, linear.x)` frame list on one input and
/// report `((admitted, decimated), recording chunk delta)`.
///
/// The message delta lets a test assert on what was actually LOGGED and not only
/// on the counter — both halves are needed, because a gate that kept its refusal
/// bookkeeping and rendered anyway satisfies the counter oracle alone (measured:
/// that exact regression left every counter-based arm in this file green). The
/// always-flush sink makes the delta exactly `rendered frames × 6` (a Twist's six
/// scalar paths), so it is a hand oracle rather than a batcher artefact.
fn dispatch_twist_frames(state: &mut SinkState, frames: &[(u64, f64)]) -> ((u64, u64), usize) {
    let walker = builtin_walker();
    let (rec, storage) = memory_always_flush();
    let before = state.plot_frames_decimated();
    let base = storage.num_msgs();
    for (ts, x) in frames {
        let frame = build_twist_at([*x, 0.0, 0.0], [0.0, 0.0, 0.0], *ts);
        dispatch_frame(&rec, &walker, "twist", &frame, state);
    }
    rec.flush_blocking().expect("flush");
    let decimated = state.plot_frames_decimated() - before;
    (
        (frames.len() as u64 - decimated, decimated),
        storage.num_msgs() - base,
    )
}

/// THE production-path pin: a plot topic published FASTER than its budget is
/// decimated on the publisher's own wire clock, and the survivors are exactly the
/// hand-computed set.
///
/// A `Twist` harvests 6 series ⇒ a 3 ms minimum wire gap (6 / 2000 s). Fed at
/// 1 ms wire spacing, frame 0 is admitted (it teaches the gate the width) and
/// thereafter every 3rd frame is — 1 + floor(29/3) = 10 of 30.
#[test]
fn a_plot_topic_faster_than_its_budget_is_decimated_on_the_wire_clock() {
    let mut state = SinkState::new();
    let (admitted, decimated) = dispatch_twist_stream(&mut state, 30, 1_000_000);
    assert_eq!(admitted, 10, "hand oracle: frame 0, then every 3 ms");
    assert_eq!(decimated, 20);
}

/// THE RENDER-SIDE pin: the gate must WITHHOLD renders, not merely COUNT them.
///
/// Measured, not hypothesised — a variant that kept the refusal bookkeeping and
/// returned `true` anyway left every counter-based arm in this file green, so
/// this one reads the RECORDING.
///
/// The sink flushes on every log ([`memory_always_flush`]), so the message delta
/// is exactly `rendered frames × 6` — a Twist's six scalar paths. That makes the
/// oracle a hand-computed number rather than a batcher artefact:
///
/// - `dense`: 30 frames at 1 ms wire spacing. A 6-series Twist earns a 3 ms gap,
///   so the frames at 0, 3, 6 … 27 ms are admitted — 10 of 30 ⇒ 60 messages.
/// - `sparse`: precisely those 10 `(stamp, value)` pairs, none refused ⇒ 60 too.
/// - `twice_as_many`: 20 admitted ⇒ 120, the anti-vacuity arm proving the probe
///   moves with the render count at all.
///
/// Under a counter-only variant, `dense` renders all 30 and logs 180.
#[test]
fn the_rate_gate_withholds_renders_not_just_counts_them() {
    let dense: Vec<(u64, f64)> = (0..30u64).map(|i| (i * 1_000_000, i as f64)).collect();
    let admitted_subset: Vec<(u64, f64)> = (0..10u64)
        .map(|k| (k * 3_000_000, (k * 3) as f64))
        .collect();
    let twice_as_many: Vec<(u64, f64)> = (0..20u64)
        .map(|k| (k * 3_000_000, (k * 3) as f64))
        .collect();

    let (dense_counts, dense_msgs) = dispatch_twist_frames(&mut SinkState::new(), &dense);
    let (sparse_counts, sparse_msgs) =
        dispatch_twist_frames(&mut SinkState::new(), &admitted_subset);
    let (wide_counts, wide_msgs) = dispatch_twist_frames(&mut SinkState::new(), &twice_as_many);
    assert_eq!(dense_counts, (10, 20), "10 of 30 admitted");
    assert_eq!(sparse_counts, (10, 0), "the subset is admitted whole");
    assert_eq!(wide_counts, (20, 0), "the wider subset too");

    assert_eq!(dense_msgs, 60, "10 rendered frames x 6 scalar paths");
    assert_eq!(
        dense_msgs, sparse_msgs,
        "the decimated run LOGS exactly what its admitted frames alone produce"
    );
    assert_eq!(
        wide_msgs, 120,
        "twice the admitted frames log twice as much — the probe really moves"
    );
}

/// **The COST claim, on the topic class it was written for.**
///
/// A gate arm driving only a NAME-MAPPED schema (`geometry_msgs/Twist`),
/// which short-circuits `classify_schema` and never reaches the shape ladder, pins
/// "a refused frame costs neither the walk nor the logs" only where it
/// is trivially true. A SHAPE-INFERRED plot topic (the entire rate-gate target
/// class, `/lowstate` included) classifies `ElementsUndecided`, so its inference
/// memo is EVICTED every frame and the ladder re-runs — two full harvest walks,
/// each allocating a `String` per sample, BEFORE the render arm the gate lived
/// in. This drives the unmapped 20 x 12 bank and reads the production
/// `inference_runs()` counter: a refused frame must not advance it.
#[test]
fn a_rate_refused_frame_does_not_re_run_the_classification_ladder() {
    let walker = wide_bank_walker();
    let mut state = SinkState::new();
    let (rec, _storage) = memory();

    // The topic is genuinely SHAPE-INFERRED (the precondition the mapped
    // fixtures silently dodged), and genuinely a re-inferring one.
    assert_eq!(classify_schema("unitree_go/LowState"), None);
    let probe = build_wide_bank_frame(0);
    let fv = walker.walk_by_hash(&probe).expect("walk wide bank");
    // The bank's curation withholds nine of every motor's twelve
    // members, so the topic earns the dump beside its plots. Still the SAME rung
    // of the ladder (`ElementsUndecided`), which is what this test is about.
    assert_eq!(
        infer_archetype_from_shape(&fv),
        ArchetypeKind::ScalarsWithText
    );

    // 60 curated series ⇒ a 30 ms wire gap. At 1 ms spacing, frames 0, 30, 60 …
    // are admitted: 1 + floor(89 / 30) = 3 of 90.
    for i in 0..90u64 {
        dispatch_frame(
            &rec,
            &walker,
            "lowstate",
            &build_wide_bank_frame(i * 1_000_000),
            &mut state,
        );
    }
    assert_eq!(state.plot_frames_decimated(), 87, "3 of 90 admitted");
    assert_eq!(
        state.inference_runs(),
        3,
        "a REFUSED frame is dropped on its header — it must not re-run the ladder"
    );
}

/// The anti-tautology partner: the ladder DOES re-run for every ADMITTED frame,
/// so the pin above measures the gate rather than a memo that silently froze.
/// Same topic, same shape, stamps spaced wide enough that nothing is refused.
#[test]
fn every_admitted_frame_of_a_re_inferring_topic_still_runs_the_ladder() {
    let walker = wide_bank_walker();
    let mut state = SinkState::new();
    let (rec, _storage) = memory();
    for i in 0..5u64 {
        dispatch_frame(
            &rec,
            &walker,
            "lowstate",
            &build_wide_bank_frame(i * 100_000_000),
            &mut state,
        );
    }
    assert_eq!(state.plot_frames_decimated(), 0, "100 ms > the 30 ms gap");
    assert_eq!(state.inference_runs(), 5);
}

/// **The dump is metered: the text tracks the plot, and a
/// small/slow topic is untouched.**
///
/// A rule of "the dump is never metered" is affordable while
/// `ScalarsWithText` means a low-rate status message. Classification also elects the same
/// archetype for a `/lowstate`-class firehose, where unmetered means a MEASURED
/// 2 150-byte document at ~500 Hz. The dump rides the SERIES verdict, and the
/// requirement an exemption would defend — the text must never freeze — is kept
/// by `DUMP_REFRESH_FRAME_FLOOR` (pinned by the stopped-clock test below).
///
/// This arm pins the half that did NOT change: a topic whose series clear their
/// own budget every frame dumps on every frame. A StatusReport harvests ONE
/// series (`voltage`; `firmware` is text), so its gap is 0.5 ms — three frames
/// 1 ms apart are ALL admitted, and the single-series class keeps its earlier
/// behaviour exactly. The anti-tautology partner is in the same body: the same
/// three frames 0.2 ms apart (inside the budget) refuse two frames, and those two
/// render NO document, which is the metering this issue added.
#[test]
fn a_small_topics_text_half_still_renders_on_every_frame() {
    let walker = status_report_walker();
    let (rec, storage) = memory_always_flush();

    // 1 ms apart: every frame clears the 0.5 ms budget.
    let mut state = SinkState::new();
    let base = storage.num_msgs();
    for i in 0..3u64 {
        let frame = status_report_frame_at(48.0 + i as f64, &format!("fw-{i}"), i * 1_000_000);
        dispatch_frame(&rec, &walker, "status", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");
    assert_eq!(state.plot_frames_decimated(), 0, "1 ms > the 0.5 ms gap");
    assert_eq!(
        state.dump_renders_withheld(),
        0,
        "an admitted frame always dumps"
    );
    assert_eq!(
        storage.num_msgs() - base,
        6,
        "3 documents + 3 single-series frames — the single-series class is unmetered in \
         practice, because its own budget admits it"
    );

    // 0.2 ms apart: inside the budget, so two frames are refused — and now their
    // documents are withheld too (earlier they were logged anyway).
    let mut state = SinkState::new();
    let base = storage.num_msgs();
    for i in 0..3u64 {
        let frame = status_report_frame_at(48.0 + i as f64, &format!("fw-{i}"), i * 200_000);
        dispatch_frame(&rec, &walker, "status", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");
    assert_eq!(state.plot_frames_decimated(), 2);
    assert_eq!(state.dump_renders_withheld(), 2);
    assert_eq!(
        storage.num_msgs() - base,
        2,
        "1 document + 1 series — the refused frames render NEITHER half"
    );
}

/// **A plots-only topic never consults the dump gate at all.**
///
/// `decide_plot_frame_before_walk` asks the dump gate only of a topic whose kind
/// renders one (`renders_dump && …`). Dropping that guard costs two things, and
/// neither shows up in what the viewer draws — which is why it needs its own pin:
///
/// - the Principle #3 counter would report withheld renders for a document the
///   topic never logs (the render arm gates on the KIND, so nothing changes on
///   screen — only the observability lies);
/// - the anti-freeze floor would periodically answer `dump = true` for such a
///   topic, and a frame whose dump is owed is NOT dropped before the walk — so
///   the header-only drop would silently lapse once per floor period on
///   every plain `Scalars` firehose.
///
/// Driven on a STALLED clock (identical stamps), which is the only shape that
/// produces a long enough run of refusals to reach the floor: a `Twist` topic is
/// name-mapped to plain `Scalars`, six series, so frame 1 is admitted and every
/// frame after it is refused forever.
///
/// Deleting the `renders_dump &&` guard passed the entire
/// suite (307 lib + 81 here) before this test existed.
#[test]
fn a_plots_only_topic_never_consults_the_dump_gate() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory_always_flush();
    let base = storage.num_msgs();

    // Two full floor periods of refusals — twice what it takes to fire.
    let frames = 1 + 2 * (DUMP_REFRESH_FRAME_FLOOR + 1);
    for _ in 0..frames {
        dispatch_frame(
            &rec,
            &walker,
            "cmd_vel",
            &build_twist_at([1.0, 2.0, 3.0], [0.1, 0.2, 0.3], 9_000),
            &mut state,
        );
    }
    rec.flush_blocking().expect("flush");

    // The precondition that makes this test about the guard: the series really
    // were refused, for long enough to reach the floor twice over.
    assert_eq!(
        state.plot_frames_decimated(),
        frames - 1,
        "a stalled clock admits exactly one frame's series"
    );
    // THE PIN: the gate was never asked, so it counted nothing.
    assert_eq!(
        state.dump_renders_withheld(),
        0,
        "a topic that renders no document must not accrue withheld renders"
    );
    // And nothing but frame 1's six series was ever logged — no document
    // appeared, whatever the floor thought.
    assert_eq!(storage.num_msgs() - base, 6, "6 scalars, no TextDocument");

    // ANTI-TAUTOLOGY: the same drive on a topic that DOES render a document
    // consults the gate and accrues withheld renders — so the assertion above
    // reads the guard rather than a counter that never moves.
    let walker = status_report_walker();
    let mut state = SinkState::new();
    for i in 0..frames {
        let frame = status_report_frame_at(48.0 + i as f64, &format!("fw-{i}"), 9_000);
        dispatch_frame(&rec, &walker, "status", &frame, &mut state);
    }
    assert!(
        state.dump_renders_withheld() > 0,
        "a ScalarsWithText topic under the same stimulus DOES consult its gate"
    );
}

/// **The anti-freeze floor: a stopped publisher clock must never freeze the text
/// at frame 1 for the life of the run.** This is the anti-freeze requirement, and
/// metering keeps it intact, by a different mechanism than an exemption.
///
/// A publisher whose wire stamps never advance is admitted ONCE and refused
/// forever (the documented gate behaviour), so a dump that ONLY rode the series
/// verdict would show frame 1's document for the whole run — on exactly the topic
/// whose plot has stopped moving and whose text is the only thing left to read.
/// `DUMP_REFRESH_FRAME_FLOOR` frames later it re-renders regardless.
///
/// Driven past the floor twice, with the counts asserted as functions of the
/// SHIPPED constant rather than of a hardcoded number, so raising or lowering it
/// cannot silently invalidate the oracle.
#[test]
fn a_stopped_clock_publishers_text_refreshes_on_the_anti_freeze_floor() {
    let walker = status_report_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory_always_flush();
    let base = storage.num_msgs();

    // Two full floor periods of identical stamps, plus the admitted first frame.
    let frames = 1 + 2 * (DUMP_REFRESH_FRAME_FLOOR + 1);
    for i in 0..frames {
        let frame = status_report_frame_at(48.0 + i as f64, &format!("fw-{i}"), 9_000);
        dispatch_frame(&rec, &walker, "status", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");

    // The series half is frozen after frame 1 — the documented stalled-clock
    // behaviour, and the precondition that makes this test about the text.
    assert_eq!(
        state.plot_frames_decimated(),
        frames - 1,
        "a stalled clock admits exactly one frame's series"
    );
    // The text half is NOT frozen: 1 admitted + 2 floor refreshes.
    assert_eq!(
        storage.num_msgs() - base,
        3 + 1,
        "3 documents (frame 1 + two floor refreshes) + frame 1's single series"
    );
    assert_eq!(
        state.dump_renders_withheld(),
        frames - 3,
        "every other frame's document is withheld — the floor is a backstop, not a \
         second cadence"
    );
}

/// **The gate covers the SPATIAL arms too.**
///
/// A gate in the `Scalars` arm alone covers 2 of the 6 arms that log the same
/// curated harvest. A wide bank sitting BESIDE a pose is exactly the composite
/// the curation was built for, it classifies `Transform3DWithScalars` (a
/// shape-inferred kind), and it would be completely ungated.
///
/// The fixture is a pose plus a 20 x 12 motor bank. The POSE must keep rendering
/// on every frame (whole state under latest-at — a withheld transform leaves the
/// robot at a stale position), while the SERIES are metered.
#[test]
fn a_wide_bank_beside_a_pose_is_rate_gated_while_the_pose_still_renders() {
    let walker = posed_bank_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory_always_flush();

    // Precondition: this really is the composite shape, shape-inferred.
    assert_eq!(classify_schema("acme/PosedBank"), None);
    let probe = build_posed_bank_frame(0);
    let fv = walker.walk_by_hash(&probe).expect("walk posed bank");
    assert_eq!(
        infer_archetype_from_shape(&fv),
        ArchetypeKind::Transform3DWithScalars
    );

    let base = storage.num_msgs();
    // 60 curated series ⇒ a 30 ms gap; 4 frames 1 ms apart admit exactly one.
    for i in 0..4u64 {
        dispatch_frame(
            &rec,
            &walker,
            "posed",
            &build_posed_bank_frame(i * 1_000_000),
            &mut state,
        );
    }
    rec.flush_blocking().expect("flush");
    assert_eq!(
        state.plot_frames_decimated(),
        3,
        "the spatial arm's series ride the SAME per-topic budget"
    );
    // 4 transforms (every frame) + 60 series (the one admitted frame).
    assert_eq!(
        storage.num_msgs() - base,
        64,
        "the pose renders on EVERY frame; only its sibling series are metered"
    );
}

/// The ANTI-TAUTOLOGY control, without which the test above passes a gate that
/// simply throttles everything: the SAME topic published INSIDE its budget is not
/// touched at all, and never claims the operator report.
#[test]
fn a_plot_topic_inside_its_budget_is_not_decimated_at_all() {
    let mut state = SinkState::new();
    // 10 ms wire spacing (100 Hz) against a 3 ms budget for 6 series.
    let (admitted, decimated) = dispatch_twist_stream(&mut state, 30, 10_000_000);
    assert_eq!(admitted, 30, "every frame of a normal-rate topic plots");
    assert_eq!(decimated, 0);
}

/// A publisher that STALLS and then BURSTS: the frames after a long silence are
/// admitted immediately (the gate holds a wire-time gap, not a token bucket that
/// has to refill), and the burst itself is still held to the budget. The stall is
/// wire time, so this arm is about the publisher's clock and not the test's.
#[test]
fn a_stall_then_burst_admits_at_once_and_still_holds_the_burst_to_budget() {
    let walker = builtin_walker();
    let (rec, _storage) = memory();
    let mut state = SinkState::new();
    let twist = |t: u64| build_twist_at([1.0, 0.0, 0.0], [0.0, 0.0, 0.0], t);

    dispatch_frame(&rec, &walker, "twist", &twist(0), &mut state);
    assert_eq!(state.plot_frames_decimated(), 0, "the first frame plots");

    // A ten-second wire-time stall, then a 1 ms-spaced burst of 10.
    let burst_start = 10_000_000_000u64;
    for i in 0..10u64 {
        dispatch_frame(
            &rec,
            &walker,
            "twist",
            &twist(burst_start + i * 1_000_000),
            &mut state,
        );
    }
    rec.flush_blocking().expect("flush");
    // Burst frame 0 is admitted at once (the gap since the last admitted frame is
    // ten seconds), then the 3 ms budget applies within the burst: frames at
    // +0, +3 ms, +6 ms, +9 ms ⇒ 4 admitted, 6 decimated.
    assert_eq!(
        state.plot_frames_decimated(),
        6,
        "the post-stall frame is admitted at once; the burst is held to budget"
    );
}

/// A publisher whose wire clock never ADVANCES plots once and is then held — the
/// documented consequence of gating on the plot's own X axis (every one of those
/// frames would land at the identical timeline position). The refusals are
/// COUNTED, so the topic is never silently stalled.
#[test]
fn a_stopped_publisher_clock_plots_one_frame_and_counts_the_rest() {
    let mut state = SinkState::new();
    let (admitted, decimated) = dispatch_twist_stream(&mut state, 25, 0);
    assert_eq!(admitted, 1);
    assert_eq!(decimated, 24);
}

/// A stamp REGRESSION is a publisher restart, not a stall: the first frame of the
/// new epoch is admitted at once rather than waiting for the old epoch's clock to
/// be overtaken. Driven through the production dispatch with a two-hour-uptime
/// stamp followed by a fresh-from-zero one — the shipping shape when a worker is
/// restarted under a long-lived desk daemon.
#[test]
fn a_restarted_publisher_plots_again_immediately() {
    let walker = builtin_walker();
    let (rec, _storage) = memory();
    let mut state = SinkState::new();
    let twist = |t: u64| build_twist_at([1.0, 0.0, 0.0], [0.0, 0.0, 0.0], t);

    dispatch_frame(
        &rec,
        &walker,
        "twist",
        &twist(7_200_000_000_000),
        &mut state,
    );
    dispatch_frame(&rec, &walker, "twist", &twist(1_000), &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        state.plot_frames_decimated(),
        0,
        "a restarted publisher's first frame is admitted, not held"
    );
}

/// Two topics keep INDEPENDENT gates: a firehose must not hold back a slow
/// sibling, and the sibling's stamps must not credit the firehose. Keyed per
/// input, so this holds even for two inputs carrying the SAME schema.
#[test]
fn each_input_carries_its_own_rate_gate() {
    let walker = builtin_walker();
    let (rec, _storage) = memory();
    let mut state = SinkState::new();
    // `fast` at 1 ms wire spacing, `slow` at 10 ms, interleaved.
    for i in 0..30u64 {
        let fast = build_twist_at([1.0, 0.0, 0.0], [0.0, 0.0, 0.0], i * 1_000_000);
        dispatch_frame(&rec, &walker, "fast", &fast, &mut state);
        let slow = build_twist_at([2.0, 0.0, 0.0], [0.0, 0.0, 0.0], i * 10_000_000);
        dispatch_frame(&rec, &walker, "slow", &slow, &mut state);
    }
    rec.flush_blocking().expect("flush");
    // Only `fast` is decimated: 30 frames at 1 ms ⇒ 10 admitted, 20 refused.
    // `slow` contributes 0, so a shared gate (or a leaked anchor) reads > 20.
    assert_eq!(state.plot_frames_decimated(), 20);
}

/// DETERMINISM (Principle #7): the decimated set is a pure function of the frame
/// stream, so two independent runs over the same stamps decimate identically. The
/// gate is what makes this true — per-tick coalescing, the other rate mechanism
/// in this crate, keys on the WALL-clock poll cadence and could not make this
/// claim.
#[test]
fn the_decimated_set_is_identical_across_two_runs() {
    let mut a = SinkState::new();
    let mut b = SinkState::new();
    let first = dispatch_twist_stream(&mut a, 40, 1_100_000);
    let second = dispatch_twist_stream(&mut b, 40, 1_100_000);
    assert_eq!(first, second);
    // …and equals the hand oracle: 6 series ⇒ 3 ms; at 1.1 ms spacing the
    // admitted frames are 0, 3, 6, … (every 3rd), i.e. 1 + floor(39/3) = 14.
    assert_eq!(first, (14, 26));
}

/// The camera headline path. A hand CompressedImage
/// frame (REAL JPEG magic bytes, `format` set) dispatches to EncodedImage —
/// exact chunk delta, exact byte round-trip through the extraction seam, and
/// the `image` input's camera entity route.
#[test]
fn compressed_image_dispatches_to_encoded_image_with_exact_bytes() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    // JPEG SOI + APP0 magic prefix — `EncodedImage` sniffs the media type
    // from these bytes (the deleted camera e2e's blob).
    let jpeg: Vec<u8> = [0xFFu8, 0xD8, 0xFF, 0xE0]
        .into_iter()
        .chain((0..64).map(|i| i as u8))
        .collect();
    let frame = build_jpeg_frame(&jpeg, 5_000);

    // Content seam: the walker surfaces the EXACT hand JPEG bytes the
    // EncodedImage is built from (zero decode — byte identity to the oracle).
    let fv = walker.walk_by_hash(&frame).expect("walk CompressedImage");
    assert_eq!(fv.schema_name, "sensor_msgs/CompressedImage");
    assert_eq!(image_data_from_frame_value(&fv), Some(jpeg.as_slice()));

    // Entity route: the `image` input renders at its OWN entity —
    // media names no longer fold onto a shared canonical camera entity.
    assert_eq!(route_for_input("image").entity, IMAGE_ENTITY);

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "image", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "exactly one EncodedImage chunk for one JPEG frame"
    );
    // Structural discriminator: the native EncodedImage arm never takes the
    // AnyValues field-dump fallback.
    assert!(
        !state.took_anyvalues_fallback("sensor_msgs/CompressedImage"),
        "a JPEG frame renders natively, not as a field dump"
    );
}

/// A RAW `sensor_msgs/Image` with a common 8-bit encoding (rgb8) now
/// renders as a native `rerun::Image` (one chunk), NOT a field dump — and does
/// NOT take the AnyValues fallback.
#[test]
fn raw_rgb8_image_dispatches_to_native_image() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let frame = build_raw_image_frame("rgb8", 6_000);

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "image", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "exactly one native Image chunk for the raw rgb8 frame"
    );
    // The discriminator: a native Image never marks the AnyValues latch.
    assert!(
        !state.took_anyvalues_fallback("sensor_msgs/Image"),
        "a decodable raw image renders natively, not as a field dump"
    );
}

/// A RAW image with an encoding we do NOT decode (16UC1) degrades to
/// the documented field dump — raw bytes are never mis-rendered as pixels.
#[test]
fn raw_image_unknown_encoding_degrades_to_field_dump() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let frame = build_raw_image_frame("16UC1", 6_000);

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "image", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "exactly one field-dump chunk for an undecodable-encoding image"
    );
    assert!(
        state.took_anyvalues_fallback("sensor_msgs/Image"),
        "an unsupported image encoding degrades to the field dump"
    );
}

/// The empty-JPEG path (behavioral half): empty JPEG frames are skipped
/// (nothing logged — no fabricated image), and a later non-empty frame still
/// renders (the latch regime heals; skipping is never a permanent mute). The
/// warn→debug level mapping itself is the oracle-pinned `FieldsWarnLatch`
/// contract (`pointcloud.rs` unit tests).
#[test]
fn empty_jpeg_frames_skip_and_a_nonempty_frame_recovers() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();

    let empty = build_jpeg_frame(&[], 1_000);
    let base = storage.num_msgs();
    dispatch_frame(&rec, &walker, "image", &empty, &mut state);
    dispatch_frame(&rec, &walker, "image", &empty, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - base,
        0,
        "empty JPEG frames log nothing (no fabricated image)"
    );

    // A non-empty frame heals the empty-frame regime and renders normally.
    let jpeg: Vec<u8> = [0xFFu8, 0xD8, 0xFF, 0xE0]
        .into_iter()
        .chain((0..8).map(|i| i as u8))
        .collect();
    dispatch_frame(
        &rec,
        &walker,
        "image",
        &build_jpeg_frame(&jpeg, 2_000),
        &mut state,
    );
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - base,
        1,
        "a non-empty frame renders after an empty regime (skip is not a mute)"
    );
}

/// The lidar headline path. A hand PointCloud2 frame
/// (2 XYZI points, empty `fields` blob → point_step-inferred layout)
/// dispatches to Points3D — exact chunk delta plus the EXACT decoded points
/// through the `cloud_from_frame_value` seam.
#[test]
fn pointcloud_dispatches_to_points3d_with_exact_points() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    // Intensity 0.0 → t=0 (blue), 255.0 → t=1 (red) after the /255 clamp.
    let points = [[1.0f32, 2.0, 3.0, 0.0], [-4.0, 5.0, -6.0, 255.0]];
    let frame = build_cloud_frame(&points, 7_000);

    // Content seam: the walker + codec produce the EXACT hand points (binary
    // f32 values — exact equality, no tolerance).
    let fv = walker.walk_by_hash(&frame).expect("walk PointCloud2");
    assert_eq!(fv.schema_name, "sensor_msgs/PointCloud2");
    let cloud = cloud_from_frame_value(&fv);
    assert_eq!(
        cloud.positions,
        vec![[1.0, 2.0, 3.0], [-4.0, 5.0, -6.0]],
        "exact XYZ hand oracle"
    );
    let cols = cloud.colors.as_ref().expect("intensity channel → colours");
    assert_eq!(
        *cols,
        vec![[0, 0, 255, 255], [255, 0, 0, 255]],
        "intensity ramp endpoints: 0 → blue, 255 → red"
    );
    assert_eq!(cloud.skipped, 0);

    // Entity route: the `cloud` input renders at its OWN entity —
    // media names no longer fold onto a shared canonical lidar entity.
    assert_eq!(route_for_input("cloud").entity, CLOUD_ENTITY);

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "cloud", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "exactly one Points3D chunk for one cloud frame"
    );
}

/// A canonical-element-framing regression pin. `PointField` is
/// variable (`string name`), so a `PointCloud2.fields` blob written in the
/// canonical element framing (the `ros2 attach` CDR ingress) decodes to a
/// `NestedArray`. `cloud_from_frame_value` is a `NestedArrayOpaque`
/// consumer too: a `_ => &[]` wildcard there would swallow
/// the decoded array and `resolve_fields` would take its EMPTY-blob branch, telling
/// the operator "`fields` empty — inferring the standard XYZ layout" for
/// descriptors that were present and fully decoded. Reading the variant's `raw`
/// keeps the truthful "blob undecodable" branch.
#[traced_test]
#[test]
fn canonical_point_fields_are_reported_undecodable_not_empty() {
    let walker = builtin_walker();
    let layout = layout_of("sensor_msgs/PointCloud2");
    let fixed = layout.fixed_size;
    let table = layout.offset_table_bytes();

    // A canonical `PointField[]` blob: `u32 count` + per element `u32 len` +
    // headerless sub-frame (`[fixed][table][name]`).
    let pf = layout_of("sensor_msgs/PointField");
    let mut body = vec![0u8; pf.fixed_size + pf.offset_table_bytes()];
    write_offset_entry(
        &mut body,
        pf.fixed_size,
        0,
        (pf.fixed_size + pf.offset_table_bytes()) as u32,
        1,
    );
    body.push(b'x');
    let mut fields_blob = 1u32.to_le_bytes().to_vec();
    fields_blob.extend_from_slice(&(body.len() as u32).to_le_bytes());
    fields_blob.extend_from_slice(&body);

    let fixed_off = |name: &str| {
        layout
            .fixed_fields
            .iter()
            .find(|f| f.name == name)
            .unwrap_or_else(|| panic!("PointCloud2 has no fixed field '{name}'"))
            .offset
    };
    let mut payload = vec![0u8; fixed + table];
    payload[fixed_off("height")..fixed_off("height") + 4].copy_from_slice(&1u32.to_le_bytes());
    payload[fixed_off("width")..fixed_off("width") + 4].copy_from_slice(&1u32.to_le_bytes());
    payload[fixed_off("point_step")..fixed_off("point_step") + 4]
        .copy_from_slice(&POINT_STEP.to_le_bytes());
    let fields_off = (fixed + table) as u32;
    write_offset_entry(&mut payload, fixed, 0, 0, 0); // header: empty
    write_offset_entry(&mut payload, fixed, 1, fields_off, fields_blob.len() as u32);
    write_offset_entry(
        &mut payload,
        fixed,
        2,
        fields_off + fields_blob.len() as u32,
        POINT_STEP,
    );
    payload.extend_from_slice(&fields_blob);
    payload.extend_from_slice(&vec![0u8; POINT_STEP as usize]);

    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: <PointCloud2 as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: 3,
        sequence: 0,
        timestamp_ns: 1_000,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);

    // Precondition: the walker really DECODES the fields array (else the
    // wildcard under test is never reached and the assertions are vacuous).
    let fv = walker.walk_by_hash(&frame).expect("walk PointCloud2");
    match fv.field("fields") {
        Some(cerulion_core::codegen::FrameValueKind::NestedArray { elements, raw }) => {
            assert_eq!(elements.len(), 1, "one decoded PointField");
            assert_eq!(*raw, fields_blob.as_slice(), "raw carries the field slice");
        }
        other => panic!("expected a DECODED canonical fields array, got {other:?}"),
    }

    // THE PIN: the bytes reach the bespoke parser, which reports them
    // UNDECODABLE — never "empty". `GENERIC_FIELDS_LATCH` is process-global and
    // a sibling test may already have opened the inference regime, so the
    // assertion accepts EITHER level's rendering of the same fact: the loud
    // first-of-regime `warn!`'s text, or the sustained `debug!`'s
    // `undecodable=true` field. Both discriminate against the empty branch
    // (which renders `undecodable=false` and the "`fields` empty" text).
    let _ = cloud_from_frame_value(&fv);
    assert!(
        logs_contain("`fields` blob undecodable") || logs_contain("undecodable=true"),
        "the operator must get the TRUTHFUL diagnosis"
    );
    assert!(
        !logs_contain("`fields` empty"),
        "the field IS present and decoded — that message is a lie"
    );
}

/// Decision: an unmapped `Vector3` `{x,y,z}` shape-infers to three Scalar
/// plots (a velocity / force / RPY), NOT a single 3D point — a velocity drawn
/// as a point would be a misleading dot at the origin. Still not a field dump.
#[test]
fn vector3_infers_to_three_scalars() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let frame = build_vector3([1.0, 2.0, 3.0]);

    // Content seam: each component lands on its own named scalar plot (the
    // exact hand oracle — never a self-compare).
    let fv = walker.walk_by_hash(&frame).expect("walk Vector3");
    assert_eq!(
        scalar_samples(&fv),
        vec![
            ("x".to_string(), 1.0),
            ("y".to_string(), 2.0),
            ("z".to_string(), 3.0),
        ],
        "each component becomes its own named scalar plot"
    );

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "state", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    // Three Scalars chunks — one per component at its own entity path
    // (x/y/z). A regression to a single Points3D point would record 1.
    assert_eq!(
        storage.num_msgs() - baseline,
        3,
        "a bare {{x,y,z}} infers to three Scalar plots (x/y/z), not one point"
    );
    assert!(
        !state.took_anyvalues_fallback("geometry_msgs/Vector3"),
        "an inferable shape does NOT take the field-dump fallback"
    );
}

/// An unmapped `builtin_interfaces/Time` (two top-level numerics)
/// shape-infers to a scalar bag — one plot per numeric field (2 chunks). The
/// "never-seen numeric telemetry becomes live plots" win.
#[test]
fn time_infers_to_scalar_bag() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let frame = build_time(12, 500);

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "clock", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        2,
        "sec + nanosec each become their own scalar plot"
    );
    assert!(!state.took_anyvalues_fallback("builtin_interfaces/Time"));
}

/// A name-mapped `geometry_msgs/Pose` renders as exactly one
/// Transform3D (the frame moves in the 3D view).
#[test]
fn pose_dispatches_to_one_transform3d() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let frame = build_pose([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]);

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "pose", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "a Pose logs exactly one Transform3D chunk"
    );
    assert!(!state.took_anyvalues_fallback("geometry_msgs/Pose"));
}

/// A schema that is neither mapped nor shape-inferable (`std_msgs/Bool`
/// — a lone bool is not auto-plotted) DOES take the field-dump fallback (one
/// chunk) — the AnyValues goal survives (nothing is un-visualizable).
#[test]
fn unmapped_uninferable_schema_falls_back_to_field_dump() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let frame = build_bool(true);

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "estop", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "a bool message records exactly one field-dump chunk"
    );
    assert!(
        state.took_anyvalues_fallback("std_msgs/Bool"),
        "an un-inferable schema still lands as an inspectable dump"
    );
}

/// Within ONE poll tick, a batch of coalescible cloud frames renders
/// ONLY the newest (latest-wins) — the earlier N-1 are coalesced away, not
/// clamped by any timestamp. The newest lands on exactly one `sweep/{k}`
/// sub-entity (one chunk), and the coalesced count folds into the run total.
#[test]
fn coalescible_cloud_batch_renders_only_the_newest() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let points = [[1.0f32, 2.0, 3.0, 0.0]];
    // Five clouds drained in one tick (distinct wire stamps — irrelevant now:
    // nothing is timestamp-gated).
    let frames: Vec<Vec<u8>> = (0..5)
        .map(|i| build_cloud_frame(&points, 1_000 + i))
        .collect();

    let base = storage.num_msgs();
    let coalesced = drain_tick(&rec, &walker, "cloud", frames, &mut state);
    rec.flush_blocking().expect("flush");

    assert_eq!(
        coalesced, 4,
        "N-1 of the 5 batched clouds are coalesced away"
    );
    assert_eq!(
        state.accepted_sweeps(CLOUD_ENTITY),
        1,
        "exactly one sweep rendered per tick"
    );
    assert_eq!(
        storage.num_msgs() - base,
        1,
        "one rendered sweep → one chunk (the other four never drew)"
    );
    // The node folds the per-tick count into the run total.
    state.record_coalesced(coalesced);
    assert_eq!(state.coalesced_frames(), 4);
}

/// A NON-coalescible (per-sample) kind renders EVERY drained frame — nothing is
/// ever staged, nothing is coalesced. Six Twist components render on six
/// distinct scalar paths (repeated same-path logs compact per path, so the
/// chunk delta is six regardless of frame count); the structural proof that all
/// three frames took the immediate render path is that `staged` is never set.
///
/// The frames carry ADVANCING wire stamps (10 ms apart), which is what
/// makes "renders EVERY drained frame" true. A plot topic is rate-gated on its
/// publisher's clock, so a fixture stamping all three identically would model a
/// stopped clock and only the first would render — leaving the claim in this
/// test's name false while its `staged` assertion still passed.
#[test]
fn non_coalescible_kind_renders_every_frame_never_staged() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let base = storage.num_msgs();

    let mut staged: Option<Vec<u8>> = None;
    let mut coalesced = 0u64;
    for i in 0..3u64 {
        let frame = build_twist_at([0.5, 0.0, 0.0], [0.0, 0.0, 0.1 * i as f64], i * 10_000_000);
        dispatch_or_stage(
            &rec,
            &walker,
            "twist",
            frame,
            &mut state,
            &mut staged,
            &mut coalesced,
        );
        assert!(
            staged.is_none(),
            "a per-sample (Scalars) frame is rendered immediately, never staged"
        );
    }
    rec.flush_blocking().expect("flush");
    assert_eq!(coalesced, 0, "no per-sample frame is coalesced away");
    // The EXACT "every drained frame rendered" oracle. The chunk delta
    // cannot serve as one — rerun batches by time as well as by path, so it
    // varies with the stamp spacing rather than with the render count (the
    // earlier `== 6` held only because all three frames carried an identical
    // stamp). The rate gate's own counter answers the question exactly.
    assert_eq!(
        state.plot_frames_decimated(),
        0,
        "all three frames were inside the plot budget, so all three rendered"
    );
    assert!(
        storage.num_msgs() - base >= 6,
        "at least the six scalar paths (linear+angular xyz) are in the recording"
    );
}

/// A source SLOWER than the poll rate (one cloud per tick) renders EVERY frame —
/// the "nothing lost when slower than the poll" contract. Over M ticks the ring
/// advances M times (M distinct `sweep/{k}` → M chunks) and nothing coalesces.
#[test]
fn one_frame_per_tick_over_m_ticks_renders_all_m() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let points = [[1.0f32, 2.0, 3.0, 0.0]];
    let base = storage.num_msgs();

    let m: u64 = 3;
    let mut total_coalesced = 0u64;
    for k in 0..m {
        total_coalesced += drain_tick(
            &rec,
            &walker,
            "cloud",
            vec![build_cloud_frame(&points, 1_000 + k)],
            &mut state,
        );
    }
    rec.flush_blocking().expect("flush");

    assert_eq!(
        state.accepted_sweeps(CLOUD_ENTITY),
        m,
        "every one-per-tick frame renders (M ticks → M sweeps)"
    );
    assert_eq!(
        total_coalesced, 0,
        "a one-per-tick source coalesces nothing"
    );
    assert_eq!(
        storage.num_msgs() - base,
        m as usize,
        "M sweeps on M distinct sub-entities → M chunks"
    );
}

/// A `nav_msgs/Odometry` frame on an ODOM-named input poses the
/// robot root (`world/tf-tree/robot`) IN ADDITION to its leaf pose + twist scalars; the
/// SAME frame on a non-electing input does NOT — the difference is exactly one
/// `world/tf-tree/robot` transform chunk.
#[test]
fn odometry_on_odom_input_also_poses_robot_root() {
    let walker = builtin_walker();
    // Distinctive hand values so the content seam is a real oracle.
    let frame = build_odometry_frame(
        [1.0, 2.0, 3.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.5, 0.0, 0.0],
        [0.0, 0.0, 0.7],
        6_000,
    );

    // Content seam: the walker decodes the nested pose + twist to the exact hand
    // values (proves the frame builder's offsets, so the chunk deltas below are
    // meaningful — never a self-compare).
    let fv = walker.walk_by_hash(&frame).expect("walk Odometry");
    assert_eq!(fv.schema_name, "nav_msgs/Odometry");
    let parts = pose_transform_parts(&fv).expect("odom pose parts");
    assert_eq!(parts.translation, [1.0, 2.0, 3.0]);
    assert_eq!(parts.rotation, Some([0.0, 0.0, 0.0, 1.0]));
    assert_eq!(
        odometry_twist_scalars(&fv),
        vec![
            ("linear/x".to_string(), 0.5),
            ("linear/y".to_string(), 0.0),
            ("linear/z".to_string(), 0.0),
            ("angular/x".to_string(), 0.0),
            ("angular/y".to_string(), 0.0),
            ("angular/z".to_string(), 0.7),
        ]
    );

    // (a) an odom-named input: leaf Transform3D (1) + 6 twist scalars (6) + the
    // world/tf-tree/robot Transform3D (1) = 8 distinct entity chunks in one dispatch.
    let mut state_a = SinkState::new();
    let (rec_a, storage_a) = memory();
    let base_a = storage_a.num_msgs();
    dispatch_frame(&rec_a, &walker, "odom", &frame, &mut state_a);
    rec_a.flush_blocking().expect("flush");
    let odom_delta = storage_a.num_msgs() - base_a;
    assert_eq!(
        odom_delta, 8,
        "leaf Transform3D + 6 twist scalars + the world/tf-tree/robot Transform3D"
    );

    // (b) a NON-electing input (same frame): leaf pose + 6 twist scalars = 7,
    // NO world/tf-tree/robot.
    let mut state_b = SinkState::new();
    let (rec_b, storage_b) = memory();
    let base_b = storage_b.num_msgs();
    dispatch_frame(&rec_b, &walker, "pose_est", &frame, &mut state_b);
    rec_b.flush_blocking().expect("flush");
    let pose_est_delta = storage_b.num_msgs() - base_b;
    assert_eq!(
        pose_est_delta, 7,
        "leaf Transform3D + 6 twist scalars, NO world/tf-tree/robot"
    );

    assert_eq!(
        odom_delta,
        pose_est_delta + 1,
        "the odom election adds EXACTLY the one world/tf-tree/robot transform"
    );
}

/// A CANONICALLY-framed `transforms` blob — the shape
/// `CdrCodec::decode` (the `ros2 attach` / `dds_bridge` ingress) writes for a
/// `TransformStamped[]`, and the shape `cerulion_core`'s FrameWalker DECODES.
/// Hand-laid: `u32 count` + per element `u32 len` + a headerless
/// `TransformStamped` sub-frame (`[fixed 56][table 16][header][child]`), whose
/// `header` is itself `[fixed 8][table 8][frame_id]`.
fn canonical_transforms_blob(frame_id: &str, child: &str) -> Vec<u8> {
    let mut hdr = vec![0u8; 16];
    hdr[8..12].copy_from_slice(&16u32.to_le_bytes()); // entry[0] offset
    hdr[12..16].copy_from_slice(&(frame_id.len() as u32).to_le_bytes());
    hdr.extend_from_slice(frame_id.as_bytes());

    let mut body = Vec::new();
    for v in [0.1f64, 0.2, 0.3] {
        body.extend_from_slice(&v.to_le_bytes());
    }
    for v in IDENTITY_QUAT {
        body.extend_from_slice(&v.to_le_bytes());
    }
    assert_eq!(body.len(), 56, "TransformStamped fixed section");
    body.extend_from_slice(&72u32.to_le_bytes()); // header offset
    body.extend_from_slice(&(hdr.len() as u32).to_le_bytes());
    body.extend_from_slice(&(72 + hdr.len() as u32).to_le_bytes()); // child offset
    body.extend_from_slice(&(child.len() as u32).to_le_bytes());
    body.extend_from_slice(&hdr);
    body.extend_from_slice(child.as_bytes());

    let mut blob = 1u32.to_le_bytes().to_vec();
    blob.extend_from_slice(&(body.len() as u32).to_le_bytes());
    blob.extend_from_slice(&body);
    blob
}

#[test]
fn tf_frame_dispatches_transforms_temporal() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    // One known transform (base → lidar → the lidar entity).
    let t = TfTransform::new("base", "lidar", [0.1, 0.2, 0.3], IDENTITY_QUAT, 0, 0);
    let frame = build_tf(0, 5_000, &encode_tf_transforms(&[t]));

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "tf", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "one Transform3D chunk for the one transform"
    );
}

/// A canonical-element-framing regression pin. A
/// canonically-framed `/tf` (which the `ros2 attach` CDR path really produces)
/// must reach this module's bespoke decoder and be reported truthfully as an
/// undecodable blob. Handling only the empty decoded array would let a
/// non-empty one fall to `_ => None`, and the operator would be told the frame "has
/// no `transforms` array" — false: the array is present, and was fully decoded.
#[traced_test]
#[test]
fn canonical_tf_frame_warns_undecodable_never_field_missing() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let blob = canonical_transforms_blob("odom", "base");
    // Sanity: the walker really DECODES this blob (else the arm under test is
    // never reached and the assertions below are vacuous).
    let tf_frame = build_tf(0, 5_000, &blob);
    let fv = walker.walk_by_hash(&tf_frame).expect("walk");
    match fv.field("transforms") {
        Some(cerulion_core::codegen::FrameValueKind::NestedArray { elements, raw }) => {
            assert_eq!(elements.len(), 1, "one decoded TransformStamped element");
            assert_eq!(
                *raw,
                blob.as_slice(),
                "raw carries the field slice verbatim"
            );
        }
        other => panic!("expected a DECODED canonical array, got {other:?}"),
    }

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "tf", &tf_frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        0,
        "the bespoke decoder cannot read canonical bytes, so nothing is logged"
    );
    assert!(
        logs_contain("undecodable TF transforms blob"),
        "the operator must get the TRUTHFUL diagnosis"
    );
    assert!(
        !logs_contain("has no `transforms` array"),
        "the field IS present and decoded — that message is a lie"
    );
}

#[test]
fn tf_static_dedups_identical_rebroadcasts() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let t = TfTransform::new("base", "lidar", [0.1, 0.2, 0.3], IDENTITY_QUAT, 0, 0);
    let tf_bytes = encode_tf_transforms(&[t]);

    // First static broadcast logs; an identical re-broadcast is deduped.
    let base = storage.num_msgs();
    dispatch_frame(
        &rec,
        &walker,
        "tf_static",
        &build_tf(0, 1_000, &tf_bytes),
        &mut state,
    );
    rec.flush_blocking().expect("flush");
    let after_first = storage.num_msgs();
    assert_eq!(after_first - base, 1, "first static transform logs");

    dispatch_frame(
        &rec,
        &walker,
        "tf_static",
        &build_tf(1, 2_000, &tf_bytes),
        &mut state,
    );
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - after_first,
        0,
        "an identical /tf_static re-broadcast is NOT re-logged (dedup)"
    );

    // A CHANGED mount table logs again (re-arm proof — not a permanent mute).
    let t2 = TfTransform::new("base", "camera", [0.0, 0.0, 0.5], IDENTITY_QUAT, 0, 0);
    dispatch_frame(
        &rec,
        &walker,
        "tf_static",
        &build_tf(2, 3_000, &encode_tf_transforms(&[t2])),
        &mut state,
    );
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - after_first,
        1,
        "a CHANGED /tf_static table logs again"
    );
}

#[test]
fn unknown_schema_hash_is_skipped_not_a_crash() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    // A valid Twist frame with its schema_hash corrupted to a value no built-in
    // schema owns → the walker cannot decode it → skipped (records nothing).
    let mut frame = build_twist([1.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
    let bogus = 0xDEAD_BEEF_DEAD_BEEFu64;
    frame[0..8].copy_from_slice(&bogus.to_le_bytes()); // schema_hash is bytes [0..8]

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "cloud", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        0,
        "an undecodable-hash frame is skipped (no fabricated record)"
    );
}

#[test]
fn dispatch_is_deterministic_across_runs() {
    let walker = builtin_walker();
    let frame = build_twist([0.5, 0.0, 0.0], [0.0, 0.0, 0.2]);

    // Per run: the recorded-chunk delta AND the content that feeds each log
    // call (the named scalar values — counts alone would
    // pass a run that logged different VALUES). `num_msgs()` returns usize —
    // keep the delta usize (anything else is an E0277 type error).
    let runs: Vec<(usize, Vec<(String, f64)>)> = (0..2)
        .map(|_| {
            let mut state = SinkState::new();
            let (rec, storage) = memory();
            let base = storage.num_msgs();
            let fv = walker.walk_by_hash(&frame).expect("walk Twist");
            let scalars = scalars_from_frame_value(&fv);
            dispatch_frame(&rec, &walker, "twist", &frame, &mut state);
            rec.flush_blocking().expect("flush");
            (storage.num_msgs() - base, scalars)
        })
        .collect();
    assert_eq!(runs[0], runs[1], "identical input → identical CONTENT");
    // Anchor to the hand oracle so this is never a self-compare: both runs
    // must equal the exact expected (count, component values).
    assert_eq!(runs[0].0, 6, "six scalar chunks per run");
    assert_eq!(
        runs[0].1,
        twist_scalar_oracle(),
        "content matches the hand oracle, not merely itself"
    );
}

// ---- Skeleton dispatch arm (F-C2) ---------------------------------------------

/// A minimal hermetic Go2 URDF: base → {FR_hip → FR_thigh, radar}. Two revolute
/// leg joints (matching `GO2_MOTOR_JOINTS[0..2]`, so motors 0/1 bind) + a fixed
/// radar mount. Enough for the skeleton to render an active stick figure.
const SKELETON_URDF: &str = r#"<?xml version="1.0"?>
<robot name="fixture">
  <link name="base"/>
  <link name="FR_hip"/>
  <link name="FR_thigh"/>
  <link name="radar"/>
  <joint name="FR_hip_joint" type="revolute">
    <origin xyz="0.1934 -0.0465 0" rpy="0 0 0"/>
    <parent link="base"/>
    <child link="FR_hip"/>
    <axis xyz="1 0 0"/>
  </joint>
  <joint name="FR_thigh_joint" type="revolute">
    <origin xyz="0 -0.0955 0" rpy="0 0 0"/>
    <parent link="FR_hip"/>
    <child link="FR_thigh"/>
    <axis xyz="0 1 0"/>
  </joint>
  <joint name="radar_joint" type="fixed">
    <origin xyz="0.28945 0 -0.046825" rpy="0 2.8782 0"/>
    <parent link="base"/>
    <child link="radar"/>
  </joint>
</robot>"#;

/// The two minimal `unitree_go` schemas the skeleton reads: a fixed
/// `MotorState` (just `q`) and a `LowState` carrying the fixed
/// `MotorState[20] motor_state` array. Parsed with the SAME parser the
/// production walker uses (matching how the harvested unitree store seeds the
/// walker), so the LowState wire frame decodes as an Array-of-Nested exactly
/// like the on-robot type.
fn unitree_lowstate_schemas() -> Vec<MessageSchema> {
    let motor =
        parse_rosmsg("float32 q\n", "MotorState", Some("unitree_go")).expect("MotorState parses");
    let low = parse_rosmsg(
        "unitree_go/MotorState[20] motor_state\n",
        "LowState",
        Some("unitree_go"),
    )
    .expect("LowState parses");
    vec![motor, low]
}

/// A walker that knows the two unitree schemas (the builtins-only walker cannot
/// decode `unitree_go/LowState` — its hash is unknown).
fn unitree_walker() -> FrameWalker {
    let (walker, _warnings) = FrameWalker::new(unitree_lowstate_schemas());
    walker
}

/// Build a `unitree_go/LowState` wire frame: `motor_state[0..20].q` = `qs`
/// (radians). `MotorState` is all-fixed (one f32), so the array is a FIXED
/// 80-byte section and motor `i`'s `q` sits at byte `i*4`. Stamps the exact
/// schema_hash the walker keys on (parser-derived, so it matches).
fn build_lowstate_frame(qs: &[f32; 20], timestamp_ns: u64) -> Vec<u8> {
    let (mut resolver, _) = LayoutResolver::new(unitree_lowstate_schemas());
    let layout = resolver
        .layout_of("unitree_go/LowState")
        .expect("LowState layout");
    // The walker keys on the RESOLVED layout hash (recipe-3, nested MotorState
    // folded), NOT the unresolved `MessageSchema::schema_hash()` — stamp that.
    let low_hash = layout.schema_hash;
    assert_eq!(
        layout.fixed_size, 80,
        "MotorState[20] × 4-byte q = 80 fixed bytes (drift guard)"
    );
    assert!(
        layout.variable_fields.is_empty(),
        "LowState (motor_state only) is all-fixed"
    );
    let mut payload = vec![0u8; layout.fixed_size];
    for (i, q) in qs.iter().enumerate() {
        payload[i * 4..i * 4 + 4].copy_from_slice(&q.to_le_bytes());
    }
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    let header = WireHeader {
        schema_hash: low_hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + layout.fixed_size) as u32,
        offset_table_count: 0,
        sequence: 0,
        timestamp_ns,
    };
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// The REALISTIC joint-bank shape — a `MotorState` declaring TWELVE
/// numeric members (the width a real quadruped/humanoid firmware publishes: a
/// position, a velocity, an acceleration, a torque estimate, their raw twins, a
/// mode, a temperature, …) inside a `MotorState[20]` bank. 20 × 12 = 240 plot
/// series from one field, which is the case the per-struct-array cap exists for.
///
/// Deliberately NOT named after any vendor's members: the class is "many
/// identical nested structs at high rate", so the fixture declares `f0..f11` and
/// the curation rule is exercised on shape alone.
fn wide_bank_schemas() -> Vec<MessageSchema> {
    let members: String = (0..12).map(|i| format!("float32 f{i}\n")).collect();
    let motor = parse_rosmsg(&members, "MotorState", Some("unitree_go")).expect("MotorState");
    let low = parse_rosmsg(
        "unitree_go/MotorState[20] motor_state\n",
        "LowState",
        Some("unitree_go"),
    )
    .expect("LowState");
    vec![motor, low]
}

/// A walker over the wide-bank schemas.
fn wide_bank_walker() -> FrameWalker {
    FrameWalker::new(wide_bank_schemas()).0
}

/// A wide-bank frame stamped `timestamp_ns`; element `i` member `j` carries
/// `i * 100 + j`, so a mis-indexed decode shows up as a wrong value.
fn build_wide_bank_frame(timestamp_ns: u64) -> Vec<u8> {
    let (mut resolver, _) = LayoutResolver::new(wide_bank_schemas());
    let layout = resolver.layout_of("unitree_go/LowState").expect("layout");
    assert_eq!(
        layout.fixed_size,
        20 * 12 * 4,
        "MotorState[20] × 12 × 4-byte f32 (drift guard)"
    );
    let mut payload = vec![0u8; layout.fixed_size];
    for i in 0..20usize {
        for j in 0..12usize {
            let off = (i * 12 + j) * 4;
            payload[off..off + 4].copy_from_slice(&((i * 100 + j) as f32).to_le_bytes());
        }
    }
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    let header = WireHeader {
        schema_hash: layout.schema_hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + layout.fixed_size) as u32,
        offset_table_count: 0,
        sequence: 0,
        timestamp_ns,
    };
    header.write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// The COMPOSITE shape the gate was built for and was inert on — a
/// pose (which classifies the topic spatial) sitting BESIDE a wide motor bank.
///
/// All-fixed so the frame builder stays a byte layout rather than an offset
/// table. Deliberately vendor-neutral: the class is "geometry plus a joint bank",
/// which every arm, humanoid and quadruped publishes under its own type name.
fn posed_bank_schemas() -> Vec<MessageSchema> {
    let members: String = (0..12).map(|i| format!("float32 f{i}\n")).collect();
    let motor = parse_rosmsg(&members, "MotorState", Some("unitree_go")).expect("MotorState");
    let bank = parse_rosmsg(
        "geometry_msgs/Pose pose\nunitree_go/MotorState[20] motor_state\n",
        "PosedBank",
        Some("acme"),
    )
    .expect("PosedBank");
    let mut out = all_schemas();
    out.push(motor);
    out.push(bank);
    out
}

/// A walker over [`posed_bank_schemas`] (built-ins included, for the nested
/// `geometry_msgs/Pose`).
fn posed_bank_walker() -> FrameWalker {
    FrameWalker::new(posed_bank_schemas()).0
}

/// An `acme/PosedBank` frame stamped `timestamp_ns`: an identity-ish pose then
/// element `i` member `j` = `i * 100 + j`.
fn build_posed_bank_frame(timestamp_ns: u64) -> Vec<u8> {
    let (mut resolver, _) = LayoutResolver::new(posed_bank_schemas());
    let layout = resolver.layout_of("acme/PosedBank").expect("layout");
    // Pose = 3 + 4 f64 = 56 bytes, then MotorState[20] x 12 x 4 = 960.
    assert_eq!(
        layout.fixed_size,
        56 + 20 * 12 * 4,
        "PosedBank shape drifted — update this builder"
    );
    assert!(layout.variable_fields.is_empty(), "PosedBank is all-fixed");
    let mut payload = vec![0u8; layout.fixed_size];
    // orientation.w = 1.0 (a real unit quaternion, so the pose extracts).
    payload[48..56].copy_from_slice(&1.0f64.to_le_bytes());
    for i in 0..20usize {
        for j in 0..12usize {
            let off = 56 + (i * 12 + j) * 4;
            payload[off..off + 4].copy_from_slice(&((i * 100 + j) as f32).to_le_bytes());
        }
    }
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    WireHeader {
        schema_hash: layout.schema_hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + layout.fixed_size) as u32,
        offset_table_count: 0,
        sequence: 0,
        timestamp_ns,
    }
    .write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// A robot whose schema declares a ZERO-LENGTH fixed struct array
/// (`unitree_go/MotorState[0]`). Accepted by the rosmsg parser, so it arrives
/// from REMOTE-supplied schema text — the desk never chose it.
fn empty_bank_schemas() -> Vec<MessageSchema> {
    let motor = parse_rosmsg("float32 q\n", "MotorState", Some("unitree_go")).expect("MotorState");
    let low = parse_rosmsg(
        "float32 power_v\nunitree_go/MotorState[0] motor_state\n",
        "LowState",
        Some("unitree_go"),
    )
    .expect("LowState");
    vec![motor, low]
}

/// A walker over [`empty_bank_schemas`].
fn empty_bank_walker() -> FrameWalker {
    FrameWalker::new(empty_bank_schemas()).0
}

/// The zero-element bank frame: just the `power_v` scalar, since the array
/// contributes no bytes.
fn build_empty_bank_frame(timestamp_ns: u64) -> Vec<u8> {
    let (mut resolver, _) = LayoutResolver::new(empty_bank_schemas());
    let layout = resolver.layout_of("unitree_go/LowState").expect("layout");
    assert_eq!(
        layout.fixed_size, 4,
        "a [0] array occupies no bytes — only power_v does (drift guard)"
    );
    let mut payload = vec![0u8; layout.fixed_size];
    payload[..4].copy_from_slice(&12.5f32.to_le_bytes());
    let mut frame = vec![0u8; WireHeader::SIZE + payload.len()];
    WireHeader {
        schema_hash: layout.schema_hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + layout.fixed_size) as u32,
        offset_table_count: 0,
        sequence: 0,
        timestamp_ns,
    }
    .write_to_buf(&mut frame[..WireHeader::SIZE]);
    frame[WireHeader::SIZE..].copy_from_slice(&payload);
    frame
}

/// **The zero-length array, through the PRODUCTION dispatch.** A `Type[0]` fixed struct
/// array can panic the desk sink with "attempt to divide by zero": the per-element
/// budget division runs before the regroup's `elements == 0` guard, and the element
/// count comes from a ROBOT's schema text.
///
/// A panic here is not a rendering bug — the viz worker owns the RecordingStream
/// for every attached topic, so one malformed remote schema takes down the whole
/// desk's rendering. The arm drives the real walker + `dispatch_frame`; without
/// the guard it panics rather than failing an assertion.
#[test]
fn a_zero_length_fixed_struct_array_does_not_panic_the_sink() {
    let walker = empty_bank_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory_always_flush();

    // Precondition: the walker really does surface an EMPTY fixed Array (this is
    // the shape the harvest divides by), and the sibling scalar is really there.
    let frame = build_empty_bank_frame(0);
    let fv = walker.walk_by_hash(&frame).expect("walk empty bank");
    let Some(FrameValueKind::Array(elems)) = fv.field("motor_state") else {
        panic!("motor_state must decode as an Array");
    };
    assert!(elems.is_empty(), "a [0] array decodes to zero elements");

    let base = storage.num_msgs();
    dispatch_frame(&rec, &walker, "lowstate", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    // The correct render for zero elements: the sibling scalar plots, the empty
    // array contributes nothing, and the topic is NOT degraded to a dump.
    assert_eq!(
        storage.num_msgs() - base,
        1,
        "the one real series (power_v) renders; the empty array adds nothing"
    );
    assert!(!state.took_anyvalues_fallback("unitree_go/LowState"));
}

/// An installed skeleton NO LONGER INTERCEPTS a joint-state topic.
///
/// This is the regression guard for the fix, and it is deliberately the
/// skeleton-INSTALLED arm: the schema row is gone from `classify_schema`, so the
/// frame rides the shape ladder to plots whether or not a skeleton exists.
/// Asserting only the inert case would pass a "fix" that merely left the mapping
/// in place while the skeleton happened to be uninstalled — which is exactly the
/// state the robot-side viz removal already produced and this mapping removal is about.
///
/// The discriminator is the exact chunk delta: 20 plot series (one per motor's
/// `q`) versus the skeleton's static tree + per-joint transforms. That the frame
/// renders NATIVELY (rather than being degraded into a view it was not given) is
/// pinned by `took_anyvalues_fallback` staying false.
///
/// The skeleton static-tree guard (`SKELETON_STATICS_LOGGED`) is process-GLOBAL,
/// so the tests that install a skeleton must not run concurrently — this
/// file-local mutex serializes them (the codebase's file-local-Mutex test
/// pattern; this workspace has no `serial_test` dep). Poisoning is tolerated
/// (a prior panic already failed its test) so one failure does not cascade.
static SKELETON_STATICS_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

#[test]
fn an_installed_skeleton_no_longer_intercepts_a_joint_state_topic() {
    let _serial = SKELETON_STATICS_LOCK
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    rearm_skeleton_statics();
    let walker = unitree_walker();
    let mut state = SinkState::new();
    state.install_skeleton(Skeleton::from_urdf_str(SKELETON_URDF).expect("active skeleton"));
    let (rec, storage) = memory();

    // Distinct q per motor so the content seam is a real oracle.
    let qs: [f32; 20] = std::array::from_fn(|i| i as f32 * 0.1);
    let frame = build_lowstate_frame(&qs, 9_000);

    // Content seam: the walker decodes `motor_state` as an Array-of-Nested with
    // the hand q values — the numbers the plots are built from (never a
    // self-compare).
    let fv = walker.walk_by_hash(&frame).expect("walk LowState");
    assert_eq!(fv.schema_name, "unitree_go/LowState");
    let Some(FrameValueKind::Array(elems)) = fv.field("motor_state") else {
        panic!("motor_state must decode as an Array");
    };
    assert_eq!(elems.len(), 20, "20 motor states");
    let FrameValueKind::Nested(m1) = &elems[1] else {
        panic!("motor 1 must decode as a Nested MotorState");
    };
    assert!(
        matches!(m1.field("q"), Some(FrameValueKind::F32(q)) if (*q - 0.1).abs() < 1e-6),
        "motor 1's q is the hand value 0.1"
    );

    let base = storage.num_msgs();
    dispatch_frame(&rec, &walker, "lowstate", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - base,
        20,
        "one plot series per motor — NOT the skeleton's static tree + joint \
         transforms, and NOT a single field-dump chunk"
    );
    assert!(
        !state.took_anyvalues_fallback("unitree_go/LowState"),
        "the joint bank renders natively as plots, never as a degraded dump"
    );
    rearm_skeleton_statics();
}

/// The mapping-removal headline: with NO skeleton installed (the SHIPPING state since
/// the robot-side viz removal deleted the last `install_skeleton` caller) a joint-state frame
/// renders PLOTS.
///
/// Mapped to the skeleton, this same frame produces exactly one field-dump chunk logged
/// into an entity whose only view is 3D: invisible, and strictly LESS than an
/// unmapped schema of the same shape renders. The exact chunk delta
/// (20, one per motor) is what separates "plots" from "one dump".
#[test]
fn a_joint_state_bank_rides_the_shape_ladder_to_plots() {
    let walker = unitree_walker();
    let mut state = SinkState::new(); // no skeleton installed ⇒ the live shape
    let (rec, storage) = memory();
    let qs: [f32; 20] = std::array::from_fn(|i| i as f32 * 0.1);
    let frame = build_lowstate_frame(&qs, 9_000);

    // The classification the ladder reaches, asserted directly: the schema is
    // unmapped, and its SHAPE declares plottable series.
    assert_eq!(classify_schema("unitree_go/LowState"), None);
    let fv = walker.walk_by_hash(&frame).expect("walk LowState");
    assert!(declares_plottable_series(&fv));

    let base = storage.num_msgs();
    dispatch_frame(&rec, &walker, "lowstate", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - base,
        20,
        "20 plot series, not the single field-dump chunk the old mapping produced"
    );
    assert!(
        !state.took_anyvalues_fallback("unitree_go/LowState"),
        "the ladder renders this natively — no degradation, so nothing lands in \
         a view the topic was not given"
    );
}

/// Mapping removal x series curation, composed: the mapping removal is only SHIPPABLE with the
/// curation, so this drives a REALISTIC joint bank (20 motors x 12 numeric
/// members = 240 declared series, the measured live shape) through the production
/// dispatch and pins BOTH halves at once.
///
/// Without the curation this frame would log 240 series at the publisher's full
/// rate. With it: 60 series (the whole-field boundary, 64 / 20 = 3 fields for
/// EVERY motor), and the next frame 1 ms later is rate-gated away.
///
/// **The dump election adds the 61st message: the field dump.** The curation withheld nine
/// of each motor's twelve members, and before the election they rendered NOWHERE: the topic
/// classified `Scalars`, whose only view is the plot. It now classifies
/// `ScalarsWithText`, so the same dispatch also logs a `TextDocument` at the
/// topic entity. The count is asserted as `60 + 1` rather than `61` so the two
/// halves stay separable in the failure message.
#[test]
fn a_wide_joint_bank_is_curated_and_rate_gated_end_to_end() {
    let walker = wide_bank_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();

    // The DECLARED width, read off the walker's own decode — 240 series before
    // curation, so the numbers below are measured rather than assumed.
    let frame = build_wide_bank_frame(0);
    let fv = walker.walk_by_hash(&frame).expect("walk wide bank");
    let Some(FrameValueKind::Array(elems)) = fv.field("motor_state") else {
        panic!("motor_state must decode as an Array");
    };
    assert_eq!(elems.len(), 20);
    let FrameValueKind::Nested(m0) = &elems[0] else {
        panic!("nested element");
    };
    assert_eq!(m0.fields.len(), 12, "20 x 12 = 240 declared series");

    let base = storage.num_msgs();
    dispatch_frame(&rec, &walker, "lowstate", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - base,
        60 + 1,
        "curated to whole fields for EVERY motor (64 / 20 = 3, x 20), not 240 — \
         PLUS the elected field dump, since the curation withheld the other nine \
         members of every motor"
    );

    // …and the rate gate holds the topic on the publisher's own clock: 60 series
    // earns a 30 ms wire gap (60 / 2000 s), so a frame 1 ms later is refused and
    // one 30 ms later is not.
    dispatch_frame(
        &rec,
        &walker,
        "lowstate",
        &build_wide_bank_frame(1_000_000),
        &mut state,
    );
    assert_eq!(state.plot_frames_decimated(), 1);
    dispatch_frame(
        &rec,
        &walker,
        "lowstate",
        &build_wide_bank_frame(30_000_000),
        &mut state,
    );
    assert_eq!(
        state.plot_frames_decimated(),
        1,
        "a frame past the wire-time gap plots"
    );
}

/// **The user-facing claim end to end: the `/lowstate` dump returns,
/// and it is the window onto what the curation withheld.**
///
/// The three halves a reader would otherwise have to compose by hand:
///
/// 1. the topic is elected to `ScalarsWithText`, so its LAYOUT carries a
///    `text_document` view — without that the document would land in an entity
///    nothing displays, which is the bug the companion pane exists to close;
/// 2. the dispatch really logs a `TextDocument` at the topic entity;
/// 3. the document CONTAINS the withheld members. `f3` is the first member the
///    64-series budget drops (3 of 12 fields survive for every motor), so its
///    presence in the text is exactly the data the plot cannot show — asserted
///    against a HAND-BUILT expected line, never a read-back of the renderer.
#[test]
fn a_curated_banks_withheld_members_are_readable_in_its_field_dump() {
    let walker = wide_bank_walker();
    let frame = build_wide_bank_frame(0);
    let fv = walker.walk_by_hash(&frame).expect("walk wide bank");

    // (1) the election + the layout that makes it renderable.
    assert_eq!(
        infer_archetype_from_shape(&fv),
        ArchetypeKind::ScalarsWithText,
        "a curated bank earns the dual view"
    );
    assert!(
        views_for_archetype(ArchetypeKind::ScalarsWithText).contains(&ViewKind::TextDocument),
        "…and the layout gives it somewhere to land"
    );

    // (2) the render: a TextDocument at the topic entity.
    let mut state = SinkState::new();
    let (rec, storage) = memory_always_flush();
    let base = storage.num_msgs();
    dispatch_frame(&rec, &walker, "lowstate", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(storage.num_msgs() - base, 61, "60 series + 1 document");

    // (3) the CONTENT: the withheld members are in the text.
    let dump = cerulion_viz::archetype::structured_dump(&fv);
    // The curated plot keeps f0..f2 for every motor; f3.. are withheld. Element
    // `i` member `j` carries `i * 100 + j` (the fixture's own rule), so motor 1's
    // withheld `f3` is 103 — a value that appears in NO plot series.
    assert!(
        dump.contains("`f3` = 103"),
        "the dump must carry a member the curation withheld: {dump}"
    );
    assert!(
        dump.contains("`f11` = 111"),
        "…including the LAST one: {dump}"
    );
    // The dump covers the first DUMP_ARRAY_PREVIEW of the 20
    // motors, not all of them — and it DISCLOSES that. Asserting the tail keeps
    // the claim accurate and stops a narrowed preview from silently passing this
    // test while the withheld members of motors 8..19 quietly leave the pane.
    assert!(
        dump.contains("20 total"),
        "the dump must disclose the elements it elided: {dump}"
    );
    assert!(
        !dump.contains("`f3` = 1903"),
        "motor 19 is past the element preview, so its members are NOT in the pane \
         — the coverage claim is 'the first elements', never 'all of them'"
    );
    // Anti-tautology: the plot really did withhold it, so the assertion above is
    // about the dump rather than about a member that was plotted anyway.
    let plotted = cerulion_viz::archetype::scalar_samples(&fv);
    assert!(
        plotted.iter().any(|(p, _)| p == "motor_state/1/f0"),
        "f0 IS plotted for every motor"
    );
    assert!(
        !plotted.iter().any(|(p, _)| p == "motor_state/1/f3"),
        "f3 is NOT plotted — it exists only in the dump"
    );
}

/// Write a minimal valid GLB (12-byte header + one JSON chunk) — enough for the
/// `.glb` sibling existence check AND for `Asset3D::from_file_path` (bytes read +
/// media-type guess at log time; GLB structure is validated only by the viewer).
/// Binary fixtures are NEVER committed — built in a tempdir at test time.
fn write_min_glb(path: &std::path::Path) {
    let mut json = br#"{"asset":{"version":"2.0"}}"#.to_vec();
    while !json.len().is_multiple_of(4) {
        json.push(b' ');
    }
    let total = 12 + 8 + json.len();
    let mut glb = Vec::with_capacity(total);
    glb.extend_from_slice(b"glTF");
    glb.extend_from_slice(&2u32.to_le_bytes());
    glb.extend_from_slice(&(total as u32).to_le_bytes());
    glb.extend_from_slice(&(json.len() as u32).to_le_bytes());
    glb.extend_from_slice(&0x4E4F_534Au32.to_le_bytes()); // "JSON"
    glb.extend_from_slice(&json);
    std::fs::write(path, glb).expect("write glb");
}

/// Mesh chunk discriminator: a skeleton whose URDF carries a link mesh
/// visual with a PRESENT `.glb` sibling logs MORE static chunks (the mesh Asset3D
/// plus its scaled Transform3D) than the SAME URDF whose `.glb` is absent — the
/// file's exact-chunk-delta convention. The stick figure is byte-identical in
/// both arms; the mesh is purely additive (the sticks-always-render contract).
#[test]
fn mesh_bearing_urdf_with_present_glb_logs_more_static_chunks() {
    let _serial = SKELETON_STATICS_LOCK
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner);
    // base carries a SCALED mesh visual (scale 2 ⇒ the mesh gets BOTH an Asset3D
    // and its Transform3D), plus the two leg joints so the skeleton is active.
    let urdf = |mesh: &str| {
        format!(
            r#"<?xml version="1.0"?>
<robot name="mesh_fixture">
  <link name="base">
    <visual>
      <origin xyz="0 0 0"/>
      <geometry><mesh filename="{mesh}" scale="2 2 2"/></geometry>
    </visual>
  </link>
  <link name="FR_hip"/>
  <link name="FR_thigh"/>
  <joint name="FR_hip_joint" type="revolute">
    <origin xyz="0.1934 -0.0465 0" rpy="0 0 0"/>
    <parent link="base"/><child link="FR_hip"/><axis xyz="1 0 0"/>
  </joint>
  <joint name="FR_thigh_joint" type="revolute">
    <origin xyz="0 -0.0955 0" rpy="0 0 0"/>
    <parent link="FR_hip"/><child link="FR_thigh"/><axis xyz="0 1 0"/>
  </joint>
</robot>"#
        )
    };
    const MESH_REF: &str = "package://go2_description/meshes/base.dae";

    // Build a `<pkg>/urdf/go2.urdf` layout in a tempdir; write the `.glb` sibling
    // ONLY when `with_glb`.
    let build = |with_glb: bool| -> tempfile::TempDir {
        let dir = tempfile::tempdir().unwrap();
        let pkg = dir.path().join("go2_description");
        std::fs::create_dir_all(pkg.join("urdf")).unwrap();
        std::fs::create_dir_all(pkg.join("meshes")).unwrap();
        if with_glb {
            write_min_glb(&pkg.join("meshes/base.glb"));
        }
        std::fs::write(pkg.join("urdf/go2.urdf"), urdf(MESH_REF)).unwrap();
        dir
    };

    // Note: the statics used to be reached by DISPATCHING a joint-state
    // frame, which no longer routes to the skeleton (no schema classifies to
    // `ArchetypeKind::Skeleton` any more). The subject here is the skeleton's own
    // mesh-asset handling, not the routing that used to reach it, so this drives
    // `log_statics_once` through the seam that exists for exactly that
    // (`skeleton_log_statics_for_test`, as `coordinate_frame_test` already does).
    let count = |dir: &tempfile::TempDir| -> usize {
        let path = dir.path().join("go2_description/urdf/go2.urdf");
        rearm_skeleton_statics();
        let mut state = SinkState::new();
        state.install_skeleton(Skeleton::load(Some(path.to_str().unwrap())));
        let (rec, storage) = memory();
        let base = storage.num_msgs();
        state.skeleton_log_statics_for_test(&rec);
        rec.flush_blocking().expect("flush");
        let n = storage.num_msgs() - base;
        rearm_skeleton_statics();
        n
    };

    let with_dir = build(true);
    let without_dir = build(false);
    // Sanity: the with-arm skeleton actually resolved the mesh (guards against a
    // silently-empty asset set masking the discriminator).
    rearm_skeleton_statics();
    let sk = Skeleton::load(Some(
        with_dir
            .path()
            .join("go2_description/urdf/go2.urdf")
            .to_str()
            .unwrap(),
    ));
    assert_eq!(sk.link_mesh_assets().len(), 1, "the base mesh resolved");
    rearm_skeleton_statics();

    let with_glb = count(&with_dir);
    let without_glb = count(&without_dir);
    assert!(
        with_glb > without_glb,
        "a present .glb adds static chunks (with={with_glb}, without={without_glb})"
    );
    assert_eq!(
        with_glb - without_glb,
        2,
        "the present mesh adds EXACTLY its Asset3D + scaled Transform3D (2 chunks)"
    );
}

// ────────────────────────────────────────────────────────────────────────────
// The "can't visualize" gap classes, END TO END over REAL wire
// frames (built with the real layout engine, decoded by the real built-in
// walker, dispatched through the real `SinkState`). Every one of these SCHEMAS
// fell to the AnyValues text dump before the new archetypes landed, so each test pairs an EXACT
// recorded-chunk hand oracle with the structural discriminator
// `!took_anyvalues_fallback(schema)` — a regression to the dump records ONE
// chunk and flips that flag.
// ────────────────────────────────────────────────────────────────────────────

/// Build a fixed-only `geometry_msgs/Wrench` frame (two nested `Vector3`s).
fn build_wrench(force: [f64; 3], torque: [f64; 3]) -> Vec<u8> {
    let f_off = field_offset("geometry_msgs/Wrench", "force");
    let t_off = field_offset("geometry_msgs/Wrench", "torque");
    let mut writes = Vec::new();
    for (i, v) in force.iter().enumerate() {
        writes.push((f_off + i * 8, v.to_le_bytes().to_vec()));
    }
    for (i, v) in torque.iter().enumerate() {
        writes.push((t_off + i * 8, v.to_le_bytes().to_vec()));
    }
    build_fixed_frame(
        "geometry_msgs/Wrench",
        <Wrench as ShmMessage>::SCHEMA_HASH,
        &writes,
    )
}

#[test]
fn wrench_frame_plots_six_nested_scalars_not_a_dump() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let frame = build_wrench([1.0, 2.0, 3.0], [4.0, 5.0, 6.0]);

    // Content seam FIRST: the nested payload is harvested path-qualified (the
    // exact hand oracle — `force`/`torque` are NOT the `linear`/`angular` names
    // the earlier twist special case knew, which is why this shape used to
    // harvest nothing and fall to a text dump).
    let fv = walker.walk_by_hash(&frame).expect("walk Wrench");
    assert_eq!(
        scalar_samples(&fv),
        vec![
            ("force/x".to_string(), 1.0),
            ("force/y".to_string(), 2.0),
            ("force/z".to_string(), 3.0),
            ("torque/x".to_string(), 4.0),
            ("torque/y".to_string(), 5.0),
            ("torque/z".to_string(), 6.0),
        ]
    );

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "wrench", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        6,
        "six Scalars chunks — one per force/torque component"
    );
    assert!(
        !state.took_anyvalues_fallback("geometry_msgs/Wrench"),
        "a Wrench must render natively, never as a field dump"
    );
}

#[test]
fn pose2d_frame_renders_one_yaw_lifted_transform() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    // theta = pi/2 → a +90 degree yaw about +Z.
    let x_off = field_offset("geometry_msgs/Pose2D", "x");
    let y_off = field_offset("geometry_msgs/Pose2D", "y");
    let t_off = field_offset("geometry_msgs/Pose2D", "theta");
    let frame = build_fixed_frame(
        "geometry_msgs/Pose2D",
        <Pose2D as ShmMessage>::SCHEMA_HASH,
        &[
            (x_off, 2.0f64.to_le_bytes().to_vec()),
            (y_off, (-3.0f64).to_le_bytes().to_vec()),
            (t_off, std::f64::consts::FRAC_PI_2.to_le_bytes().to_vec()),
        ],
    );

    // Content seam: the planar pose lifts to a full rigid transform (hand
    // oracle — translation on the z=0 plane, sin/cos(pi/4) yaw quaternion).
    let fv = walker.walk_by_hash(&frame).expect("walk Pose2D");
    let parts = planar_pose_of(&fv).expect("planar pose");
    assert_eq!(parts.translation, [2.0, -3.0, 0.0]);
    let q = parts.rotation.expect("yaw quaternion");
    let root_half = std::f32::consts::FRAC_1_SQRT_2;
    assert!((q[2] - root_half).abs() < 1e-6 && (q[3] - root_half).abs() < 1e-6);

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "pose2d", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "exactly one Transform3D chunk for a planar pose"
    );
    assert!(!state.took_anyvalues_fallback("geometry_msgs/Pose2D"));
}

#[test]
fn bounding_box3d_frame_renders_one_boxes3d() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let c_off = field_offset("vision_msgs/BoundingBox3D", "center");
    let s_off = field_offset("vision_msgs/BoundingBox3D", "size");
    // `center` is a nested Pose {position, orientation}; `size` a Vector3.
    let pose_layout = layout_of("geometry_msgs/Pose");
    let pose_rel = |name: &str| {
        pose_layout
            .fixed_fields
            .iter()
            .find(|f| f.name == name)
            .unwrap_or_else(|| panic!("Pose.{name}"))
            .offset
    };
    let mut writes = Vec::new();
    for (i, v) in [1.0f64, 2.0, 3.0].iter().enumerate() {
        writes.push((
            c_off + pose_rel("position") + i * 8,
            v.to_le_bytes().to_vec(),
        ));
    }
    // Identity quaternion [x, y, z, w] = [0, 0, 0, 1] → only `w` is non-zero.
    writes.push((
        c_off + pose_rel("orientation") + 24,
        1.0f64.to_le_bytes().to_vec(),
    ));
    for (i, v) in [0.4f64, 0.4, 1.8].iter().enumerate() {
        writes.push((s_off + i * 8, v.to_le_bytes().to_vec()));
    }
    let frame = build_fixed_frame(
        "vision_msgs/BoundingBox3D",
        <BoundingBox3D as ShmMessage>::SCHEMA_HASH,
        &writes,
    );

    // Content seam: centre + FULL size + orientation (hand oracle).
    let fv = walker.walk_by_hash(&frame).expect("walk BoundingBox3D");
    assert_eq!(
        box3d_parts(&fv),
        Some(BoxParts {
            center: [1.0, 2.0, 3.0],
            size: [0.4, 0.4, 1.8],
            rotation: Some([0.0, 0.0, 0.0, 1.0]),
        })
    );

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "detection", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "exactly one Boxes3D chunk for one detection box"
    );
    assert!(!state.took_anyvalues_fallback("vision_msgs/BoundingBox3D"));
}

/// Build a `geometry_msgs/QuaternionStamped` frame: empty `header` (the one
/// variable field), `quaternion` written into the fixed section.
fn build_quaternion_stamped(q: [f64; 4]) -> Vec<u8> {
    let layout = layout_of("geometry_msgs/QuaternionStamped");
    assert_eq!(
        layout
            .variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["header"],
        "QuaternionStamped variable-field order changed — update this builder"
    );
    let fixed = layout.fixed_size;
    let table = layout.offset_table_bytes();
    let q_off = layout
        .fixed_fields
        .iter()
        .find(|f| f.name == "quaternion")
        .expect("quaternion")
        .offset;
    let mut payload = vec![0u8; fixed + table];
    for (i, v) in q.iter().enumerate() {
        let at = q_off + i * 8;
        payload[at..at + 8].copy_from_slice(&v.to_le_bytes());
    }
    write_offset_entry(&mut payload, fixed, 0, 0, 0); // header: empty
    let mut frame = vec![0u8; WireHeader::SIZE];
    let header = WireHeader {
        schema_hash: <QuaternionStamped as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: 1,
        sequence: 0,
        timestamp_ns: 7_000,
    };
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

#[test]
fn quaternion_stamped_frame_renders_one_rotation_transform() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let frame = build_quaternion_stamped([0.0, 0.0, 0.0, 1.0]);

    // Content seam: the NAMED `quaternion` field is read as the rotation (hand
    // oracle) — a bare top-level {x,y,z,w} would still be a scalar bag.
    let fv = walker.walk_by_hash(&frame).expect("walk QuaternionStamped");
    assert_eq!(rotation_only_of(&fv), Some([0.0, 0.0, 0.0, 1.0]));

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "attitude", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "exactly one rotation Transform3D chunk"
    );
    assert!(!state.took_anyvalues_fallback("geometry_msgs/QuaternionStamped"));
}

/// Build a `sensor_msgs/JointState` frame: empty `header` + `name`, and the
/// given `position` / `velocity` / `effort` float64 banks.
fn build_joint_state(position: &[f64], velocity: &[f64], effort: &[f64]) -> Vec<u8> {
    let layout = layout_of("sensor_msgs/JointState");
    assert_eq!(
        layout
            .variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["header", "name", "position", "velocity", "effort"],
        "JointState variable-field order changed — update this builder"
    );
    let fixed = layout.fixed_size;
    let table = layout.offset_table_bytes();
    let le = |vals: &[f64]| -> Vec<u8> { vals.iter().flat_map(|v| v.to_le_bytes()).collect() };
    let (pos, vel, eff) = (le(position), le(velocity), le(effort));
    let mut payload = vec![0u8; fixed + table];
    let mut cursor = (fixed + table) as u32;
    write_offset_entry(&mut payload, fixed, 0, 0, 0); // header: empty
    write_offset_entry(&mut payload, fixed, 1, 0, 0); // name: empty (this builder's shape; the populated-names twin is build_joint_state_with_names)
    for (slot, bytes) in [(2usize, &pos), (3, &vel), (4, &eff)] {
        write_offset_entry(&mut payload, fixed, slot, cursor, bytes.len() as u32);
        cursor += bytes.len() as u32;
    }
    payload.extend_from_slice(&pos);
    payload.extend_from_slice(&vel);
    payload.extend_from_slice(&eff);

    let mut frame = vec![0u8; WireHeader::SIZE];
    let header = WireHeader {
        schema_hash: <JointState as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: 5,
        sequence: 0,
        timestamp_ns: 9_000,
    };
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

#[test]
fn joint_state_frame_plots_the_whole_joint_bank() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    // Three joints: positions + velocities, no efforts published.
    let frame = build_joint_state(&[0.1, 0.2, 0.3], &[-1.0, 0.0, 1.0], &[]);

    // Content seam: each array element becomes its own indexed series and an
    // EMPTY bank contributes nothing (hand oracle — never a fabricated zero).
    let fv = walker.walk_by_hash(&frame).expect("walk JointState");
    assert_eq!(
        scalar_samples(&fv),
        vec![
            ("position/0".to_string(), 0.1),
            ("position/1".to_string(), 0.2),
            ("position/2".to_string(), 0.3),
            ("velocity/0".to_string(), -1.0),
            ("velocity/1".to_string(), 0.0),
            ("velocity/2".to_string(), 1.0),
        ]
    );

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "joint_states", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        6,
        "six Scalars chunks — 3 positions + 3 velocities, effort empty"
    );
    assert!(
        !state.took_anyvalues_fallback("sensor_msgs/JointState"),
        "the most-published ROS topic there is must not be a text dump"
    );
}

/// Build a `nav_msgs/OccupancyGrid` frame: empty `header`, `info.width/height`
/// in the fixed section, and `data` as the variable `int8[]` cell buffer.
fn build_occupancy_grid(width: u32, height: u32, cells: &[u8]) -> Vec<u8> {
    let layout = layout_of("nav_msgs/OccupancyGrid");
    assert_eq!(
        layout
            .variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["header", "data"],
        "OccupancyGrid variable-field order changed — update this builder"
    );
    let fixed = layout.fixed_size;
    let table = layout.offset_table_bytes();
    let info_off = layout
        .fixed_fields
        .iter()
        .find(|f| f.name == "info")
        .expect("info")
        .offset;
    let meta = layout_of("nav_msgs/MapMetaData");
    let rel = |name: &str| {
        meta.fixed_fields
            .iter()
            .find(|f| f.name == name)
            .unwrap_or_else(|| panic!("MapMetaData.{name}"))
            .offset
    };
    let mut payload = vec![0u8; fixed + table];
    let put_u32 = |buf: &mut [u8], off: usize, v: u32| {
        buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
    };
    put_u32(&mut payload, info_off + rel("width"), width);
    put_u32(&mut payload, info_off + rel("height"), height);
    let data_off = (fixed + table) as u32;
    write_offset_entry(&mut payload, fixed, 0, 0, 0); // header: empty
    write_offset_entry(&mut payload, fixed, 1, data_off, cells.len() as u32);
    payload.extend_from_slice(cells);

    let mut frame = vec![0u8; WireHeader::SIZE];
    let header = WireHeader {
        schema_hash: <OccupancyGrid as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: 2,
        sequence: 0,
        timestamp_ns: 11_000,
    };
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

#[test]
fn occupancy_grid_frame_renders_one_grayscale_image() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    // ROS row 0 (the origin row, +y up) = [free, occupied]; row 1 = [unknown, mid].
    let frame = build_occupancy_grid(2, 2, &[0, 100, 0xFF, 50]);

    // Content seam: the hand-computed grayscale, rows flipped so rerun's TOP row
    // is the grid's LAST ROS row (what makes the map match rviz, not mirrored).
    let fv = walker.walk_by_hash(&frame).expect("walk OccupancyGrid");
    let grid = occupancy_image(&fv).expect("occupancy image");
    assert_eq!((grid.width, grid.height), (2, 2));
    assert_eq!(grid.gray, vec![128, 127, 255, 0]);

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "map", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "exactly one grayscale Image chunk for the map"
    );
    assert!(
        !state.took_anyvalues_fallback("nav_msgs/OccupancyGrid"),
        "/map must render as a picture, not a `data = <N bytes>` dump"
    );
}

#[test]
fn occupancy_grid_with_a_short_cell_buffer_degrades_to_a_dump() {
    // ANTI-TAUTOLOGY + safe degrade: a grid whose `data` is shorter than
    // width*height must NOT be rendered half-garbage — it takes the field-dump
    // path (one chunk) and the fallback flag flips.
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let frame = build_occupancy_grid(4, 4, &[0, 0, 0]);

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "map", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(storage.num_msgs() - baseline, 1, "one field-dump chunk");
    assert!(
        state.took_anyvalues_fallback("nav_msgs/OccupancyGrid"),
        "an undecodable grid degrades LOUDLY to the dump"
    );
}

#[test]
fn new_gap_class_frames_are_deterministic_across_two_runs() {
    // Principle #7: the same frames dispatched twice record identically (the
    // chunk deltas are the observable; nothing wall-clock enters the path — every
    // stamp is the frame's own wire timestamp).
    let walker = builtin_walker();
    let frames: Vec<(&str, Vec<u8>)> = vec![
        ("wrench", build_wrench([1.0, 2.0, 3.0], [4.0, 5.0, 6.0])),
        ("joint_states", build_joint_state(&[0.1, 0.2], &[], &[])),
        ("map", build_occupancy_grid(2, 2, &[0, 100, 0xFF, 50])),
        ("attitude", build_quaternion_stamped([0.0, 0.0, 0.0, 1.0])),
    ];
    let run = || -> Vec<usize> {
        let mut state = SinkState::new();
        let (rec, storage) = memory();
        frames
            .iter()
            .map(|(input, frame)| {
                let base = storage.num_msgs();
                dispatch_frame(&rec, &walker, input, frame, &mut state);
                rec.flush_blocking().expect("flush");
                storage.num_msgs() - base
            })
            .collect()
    };
    let first = run();
    // Hand oracle, so neither leg is a self-compare: 6 wrench components,
    // 2 joint positions, 1 map image, 1 rotation transform.
    assert_eq!(first, vec![6, 2, 1, 1]);
    assert_eq!(run(), first, "two runs record identically");
}

// ── A spatial shape must not swallow its sibling telemetry ────────────────────

/// Build a fixed-only `radar_msgs/RadarTrack` frame — a REAL ROS message with
/// exactly the shape the regression hit: a named `position` (which makes it
/// classify spatially) PLUS `velocity` / `acceleration` / `size` / `classification`
/// telemetry AND four `*_covariance` matrices. Every field is fixed-size, so the
/// fixed-frame builder covers it.
fn build_radar_track(
    position: [f64; 3],
    velocity: [f64; 3],
    acceleration: [f64; 3],
    size: [f64; 3],
    classification: u16,
) -> Vec<u8> {
    let mut writes = Vec::new();
    for (field, vals) in [
        ("position", position),
        ("velocity", velocity),
        ("acceleration", acceleration),
        ("size", size),
    ] {
        let base = field_offset("radar_msgs/RadarTrack", field);
        for (i, v) in vals.iter().enumerate() {
            writes.push((base + i * 8, v.to_le_bytes().to_vec()));
        }
    }
    writes.push((
        field_offset("radar_msgs/RadarTrack", "classification"),
        classification.to_le_bytes().to_vec(),
    ));
    build_fixed_frame(
        "radar_msgs/RadarTrack",
        <RadarTrack as ShmMessage>::SCHEMA_HASH,
        &writes,
    )
}

/// THE headline regression pin. A `radar_msgs/RadarTrack`
/// classifies spatially on its named `position` — and without the sibling-telemetry
/// arm that means the bare `Point3D` archetype, whose render arm logs ONE dot. `velocity`,
/// `acceleration`, `size` and `classification` are dropped SILENTLY, and the
/// layout mapping gives the topic no plot view to put them in. (Classified as `Scalars`,
/// the same message plots its numbers, so this is a
/// regression, not a missing feature.)
#[test]
fn a_spatial_frame_with_sibling_telemetry_logs_the_point_and_the_plots() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let frame = build_radar_track(
        [1.0, 2.0, 3.0],
        [0.5, 0.0, -0.25],
        [0.1, 0.2, 0.3],
        [4.0, 5.0, 6.0],
        2,
    );

    // Content seam FIRST (a hand oracle over the real decoded frame, never a
    // self-compare): the classification is the both-views archetype, and the
    // sibling series are exactly the numbers the point did NOT consume —
    // `position/*` is the rendered point, so it is absent from the plots.
    let fv = walker.walk_by_hash(&frame).expect("walk RadarTrack");
    assert_eq!(
        infer_archetype_from_shape(&fv),
        ArchetypeKind::Point3DWithScalars,
        "a position + telemetry message must keep BOTH halves"
    );
    let (samples, skipped) = spatial_sibling_series(&fv);
    assert_eq!(
        samples,
        vec![
            ("velocity/x".to_string(), 0.5),
            ("velocity/y".to_string(), 0.0),
            ("velocity/z".to_string(), -0.25),
            ("acceleration/x".to_string(), 0.1),
            ("acceleration/y".to_string(), 0.2),
            ("acceleration/z".to_string(), 0.3),
            ("size/x".to_string(), 4.0),
            ("size/y".to_string(), 5.0),
            ("size/z".to_string(), 6.0),
            ("classification".to_string(), 2.0),
        ],
        "every sibling number plots; the pose's own components never re-plot"
    );
    // The `uuid` field (a `unique_identifier_msgs/UUID` wrapping `uint8[16]`)
    // walks as an opaque byte blob, not a numeric array, so it contributes NO
    // series — the harvest does not turn an identifier into 16 flat plot lines.
    assert!(
        !samples.iter().any(|(n, _)| n.starts_with("uuid")),
        "an opaque id blob is not telemetry: {samples:?}"
    );
    assert!(
        !samples.iter().any(|(n, _)| n.starts_with("position")),
        "the rendered point is never duplicated as plot series: {samples:?}"
    );
    // The four `*_covariance` matrices are REPORTED, not silently dropped.
    assert_eq!(
        skipped.len(),
        4,
        "four covariance matrices reported: {skipped:?}"
    );
    assert!(skipped
        .iter()
        .all(|s| s.to_string().contains("a covariance matrix")));

    // Render seam: one Points3D chunk + one Scalars chunk per sibling series —
    // 11 chunks total, against the 1 a bare `Point3D` render records.
    assert_eq!(samples.len(), 10, "the exact telemetry series count");
    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "track", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1 + samples.len(),
        "a bare Point3D render records exactly 1 chunk (the dot) and drops the rest"
    );
    assert!(
        !state.took_anyvalues_fallback("radar_msgs/RadarTrack"),
        "it renders natively, not as a field dump"
    );
}

/// The ANTI-TAUTOLOGY control for the RENDER side: a PURE spatial frame
/// (`geometry_msgs/Pose`, name-mapped `Transform3D`) still records exactly ONE
/// transform chunk. If the fix had made every spatial frame run the scalar
/// harvest into plots regardless, this count would grow.
///
/// SCOPE: this frame is NAME-mapped, so it never reaches
/// `infer_archetype_from_shape` — it cannot guard the classifier's WithScalars
/// split. And because the two twins render IDENTICALLY (they differ only in the
/// layout mapping), NO chunk-count assertion can discriminate them. The
/// classifier-split control is `an_unmapped_pure_spatial_frame_is_not_given_a_plot_view`
/// below, which asserts the archetype and the views it earns.
#[test]
fn a_pure_pose_frame_still_records_exactly_one_transform() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let frame = build_pose([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]);

    let fv = walker.walk_by_hash(&frame).expect("walk Pose");
    assert!(
        spatial_sibling_series(&fv).0.is_empty(),
        "a bare Pose carries no sibling telemetry"
    );

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "pose", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "a pure pose is ONE transform chunk — no fabricated plot series"
    );
}

/// THE anti-tautology control for the CLASSIFIER split, over a REAL wire frame
/// that actually reaches it: `geometry_msgs/Pose2D` is UNMAPPED (so it is
/// classified by shape, unlike `geometry_msgs/Pose`) and PURE spatial (`x`, `y`
/// and `theta` are all consumed by the yaw lift). It must classify to the plain
/// `Transform3D` and earn NO plot view — this test FAILS if the WithScalars arm
/// over-fires, which the chunk-count control above cannot detect, because the
/// twins render identically and differ only in the layout mapping.
#[test]
fn an_unmapped_pure_spatial_frame_is_not_given_a_plot_view() {
    let walker = builtin_walker();
    let frame = build_fixed_frame(
        "geometry_msgs/Pose2D",
        <Pose2D as ShmMessage>::SCHEMA_HASH,
        &[
            (
                field_offset("geometry_msgs/Pose2D", "x"),
                2.0f64.to_le_bytes().to_vec(),
            ),
            (
                field_offset("geometry_msgs/Pose2D", "y"),
                (-3.0f64).to_le_bytes().to_vec(),
            ),
            (
                field_offset("geometry_msgs/Pose2D", "theta"),
                std::f64::consts::FRAC_PI_2.to_le_bytes().to_vec(),
            ),
        ],
    );
    let fv = walker.walk_by_hash(&frame).expect("walk Pose2D");
    // It really is shape-classified (the mapped table must not answer for it —
    // otherwise this control would be as vacuous as the Pose one).
    assert_eq!(
        cerulion_viz::sink::classify_schema("geometry_msgs/Pose2D"),
        None,
        "Pose2D must reach infer_archetype_from_shape for this control to bite"
    );
    assert_eq!(
        infer_archetype_from_shape(&fv),
        ArchetypeKind::Transform3D,
        "a PURE planar pose must not be handed the both-views twin"
    );
    assert!(
        spatial_sibling_series(&fv).0.is_empty(),
        "x/y/theta are ALL consumed by the yaw lift — nothing is left to plot"
    );
    // The classification is view-load-bearing: the plain twin earns no plot
    // panel, the WithScalars twin would.
    assert!(!views_for_archetype(ArchetypeKind::Transform3D).contains(&ViewKind::TimeSeries));
    assert!(
        views_for_archetype(ArchetypeKind::Transform3DWithScalars).contains(&ViewKind::TimeSeries)
    );
}

// The STATELESS spatial-plus-siblings parity test that stood here is GONE with
// `archetype::log_frame_value`. Its leg 1 drove the SAME `radar_msgs/RadarTrack`
// frame against the SAME `1 + spatial_sibling_series(..).len()` == 11 oracle as
// `a_spatial_frame_with_sibling_telemetry_logs_the_point_and_the_plots` above,
// which also pins the classification, the 10-entry sibling oracle and the
// covariance skips — so that half is subsumed, not lost. Its leg 2 (a rotation
// plus one sibling ⇒ 2 chunks) had no dispatch twin and is ported to
// `a_rotation_with_sibling_telemetry_renders_the_transform_and_its_plots`.
// ────────────────────────────────────────────────────────────────────────────
// The ELEMENT-ARRAY ladder over REAL wire frames.
//
// Hand-built CANONICAL-V1 element payloads (`u32 count` + per element `u32 len` +
// element sub-frame for a variable element; back-to-back fixed sections for a
// fixed one — see `cerulion_core::codegen::CdrCodec`), decoded by the REAL
// built-in walker and dispatched through the REAL `SinkState`. Each test pairs
// the PURE extractor against a hand oracle with the EXACT recorded-chunk delta
// and the structural `!took_anyvalues_fallback(schema)` discriminator — a
// regression to the text dump records ONE chunk and flips that flag.
//
// Every one of these schemas rendered as an AnyValues text dump before the canonical element framing landed.
// ────────────────────────────────────────────────────────────────────────────

/// `u32 count` + per element (`u32 len`, body) — the canonical COUNTED framing a
/// VARIABLE element rides (crib: `frame_walker.rs`'s `counted_blob`).
fn counted_blob(elements: &[Vec<u8>]) -> Vec<u8> {
    let mut v = Vec::new();
    v.extend_from_slice(&(elements.len() as u32).to_le_bytes());
    for e in elements {
        v.extend_from_slice(&(e.len() as u32).to_le_bytes());
        v.extend_from_slice(e);
    }
    v
}

/// One `geometry_msgs/Pose` fixed section: `Point{x,y,z}` then
/// `Quaternion{x,y,z,w}`, seven f64 LE = 56 bytes.
fn pose_fixed_section(pos: [f64; 3], quat: [f64; 4]) -> Vec<u8> {
    let mut v = Vec::with_capacity(56);
    for c in pos.iter().chain(quat.iter()) {
        v.extend_from_slice(&c.to_le_bytes());
    }
    assert_eq!(v.len(), 56, "Pose fixed section drifted");
    v
}

/// A `std_msgs/Header` sub-frame: fixed `Time{sec, nanosec}` (8 B) | entry[0]
/// `frame_id` (8 B) | UTF-8.
fn header_body(frame_id: &str) -> Vec<u8> {
    let l = layout_of("std_msgs/Header");
    assert_eq!(
        (l.fixed_size, l.offset_table_bytes()),
        (8, 8),
        "Header shape drifted — update this builder"
    );
    let mut v = vec![0u8; 16];
    write_offset_entry(&mut v, 8, 0, 16, frame_id.len() as u32);
    v.extend_from_slice(frame_id.as_bytes());
    v
}

/// A `geometry_msgs/PoseStamped` element body: fixed `pose` (56 B) | entry[0]
/// `header` (8 B) | the header sub-frame.
fn pose_stamped_body(pos: [f64; 3], quat: [f64; 4], frame_id: &str) -> Vec<u8> {
    let l = layout_of("geometry_msgs/PoseStamped");
    assert_eq!(
        (l.fixed_size, l.offset_table_bytes()),
        (56, 8),
        "PoseStamped shape drifted — update this builder"
    );
    let hdr = header_body(frame_id);
    let mut v = pose_fixed_section(pos, quat);
    v.extend_from_slice(&[0u8; 8]); // entry[0] placeholder
    write_offset_entry(&mut v, 56, 0, 64, hdr.len() as u32);
    v.extend_from_slice(&hdr);
    v
}

/// A `vision_msgs/Detection3D` element body: fixed `bbox` (80 B — a `Pose`
/// centre then a `Vector3` size) | entries[0..3] `header`/`results`/`id` (24 B) |
/// the header sub-frame, an EMPTY `results` array and the `id` string.
fn detection3d_body(center: [f64; 3], quat: [f64; 4], size: [f64; 3], id: &str) -> Vec<u8> {
    let l = layout_of("vision_msgs/Detection3D");
    assert_eq!(
        (l.fixed_size, l.offset_table_bytes()),
        (80, 24),
        "Detection3D shape drifted — update this builder"
    );
    assert_eq!(
        l.variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["header", "results", "id"],
        "Detection3D variable-field declaration order changed — update this builder"
    );
    let bbox_l = layout_of("vision_msgs/BoundingBox3D");
    assert_eq!(bbox_l.fixed_size, 80, "BoundingBox3D fixed size drifted");
    let hdr = header_body("camera");
    // bbox = center Pose (56 B) then size Vector3 (3 × f64 = 24 B).
    let mut v = pose_fixed_section(center, quat);
    for c in size {
        v.extend_from_slice(&c.to_le_bytes());
    }
    assert_eq!(v.len(), 80);
    v.extend_from_slice(&[0u8; 24]); // three entry placeholders
    let var_start = 104u32;
    write_offset_entry(&mut v, 80, 0, var_start, hdr.len() as u32);
    // `results` is an empty ObjectHypothesisWithPose[]: a real offset, zero length.
    write_offset_entry(&mut v, 80, 1, var_start + hdr.len() as u32, 0);
    write_offset_entry(&mut v, 80, 2, var_start + hdr.len() as u32, id.len() as u32);
    v.extend_from_slice(&hdr);
    v.extend_from_slice(id.as_bytes());
    v
}

/// Build a `{header, <array>}` wire frame carrying `blob` verbatim in entry 1 and
/// an intentionally-empty `header` in entry 0 — the shape `nav_msgs/Path`,
/// `geometry_msgs/PoseArray` and `vision_msgs/Detection3DArray` all share.
fn build_header_plus_array_frame(
    qname: &str,
    schema_hash: u64,
    array: &str,
    blob: &[u8],
) -> Vec<u8> {
    let l = layout_of(qname);
    assert_eq!(
        l.variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["header", array],
        "{qname} variable-field declaration order changed — update this builder"
    );
    assert_eq!(l.fixed_size, 0, "{qname} gained a fixed section");
    let table = l.offset_table_bytes();
    let mut payload = vec![0u8; table];
    // entry 0 = `header`: left (0, 0) — the empty-nested producer idiom.
    write_offset_entry(&mut payload, 0, 1, table as u32, blob.len() as u32);
    payload.extend_from_slice(blob);
    let mut frame = vec![0u8; WireHeader::SIZE];
    let header = WireHeader {
        schema_hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: WireHeader::SIZE as u32,
        offset_table_count: l.variable_fields.len() as u32,
        sequence: 0,
        timestamp_ns: 42_000,
    };
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

/// A `geometry_msgs/Polygon` SUB-FRAME body carrying `points` bytes verbatim: one
/// offset entry (8 B) then the blob. Used to plant an undecodable array ONE HOP
/// DOWN, where `opaque_element_arrays` (top-level only) cannot see it.
fn polygon_body(points_blob: &[u8]) -> Vec<u8> {
    let l = layout_of("geometry_msgs/Polygon");
    assert_eq!(
        (l.fixed_size, l.offset_table_bytes()),
        (0, 8),
        "Polygon shape drifted — update this builder"
    );
    let mut v = vec![0u8; 8];
    write_offset_entry(&mut v, 0, 0, 8, points_blob.len() as u32);
    v.extend_from_slice(points_blob);
    v
}

// ── The fallback-diagnostic markers, one per reason ─────────────────────────
//
// The contract these pin: a frame that cannot be drawn produces EXACTLY ONE of
// these lines. An undecodable rmw-packed array must not produce two — the
// archetype-level reason AND a second, out-of-convention framing warn
// (`DELETED_SECOND_DIAGNOSTIC`) that restates the same condition in internal
// vocabulary.

/// The ONE line a frame whose top-level array bytes the walker refused reports.
const ELEMENT_BYTES_MARKER: &str = "element bytes this build cannot decode";
/// The archetype-level element-array reason (nothing specific to name).
const ELEMENT_ARRAY_MARKER: &str = "path / pose-array / detection-set / marker-array archetype";
/// The bounding-box / occupancy-grid reason.
const BOX_MARKER: &str = "box / occupancy-grid archetype";
/// The second diagnostic that was DELETED — it must never come back.
const DELETED_SECOND_DIAGNOSTIC: &str = "NOT canonically framed";
/// The unmapped-schema reason WITHOUT undecodable arrays: the once-per-schema info.
const UNMAPPED_INFO_MARKER: &str = "AnyValues fallback; nothing is un-visualizable";
/// The unmapped-schema reason WITH undecodable arrays: one per-INPUT warn carrying
/// both remediations. Deliberately worded apart from [`ELEMENT_BYTES_MARKER`] so a
/// test can tell WHICH reason spoke.
const UNMAPPED_OPAQUE_MARKER: &str = "an encoding this build cannot decode";

/// `nav_msgs/Path` / `vision_msgs/Detection3DArray` element bodies packed with no
/// offset table — the shape whose elements the walker's audit refuses, so the array
/// arrives `NestedArrayOpaque`. (Today's live producer of that shape is the ROS 2
/// rmw bridge, which the rmw framing convergence handles; the DIAGNOSTIC under test deliberately
/// names the encoding gap rather than any one producer.)
fn undecodable_path_frame() -> Vec<u8> {
    let bodies: Vec<Vec<u8>> = [[1.0f64, 2.0, 3.0], [4.0, 5.0, 6.0]]
        .iter()
        .map(|p| pose_fixed_section(*p, [0.0, 0.0, 0.0, 1.0]))
        .collect();
    build_header_plus_array_frame(
        "nav_msgs/Path",
        <native_ros2_messages::nav_msgs::Path as ShmMessage>::SCHEMA_HASH,
        "poses",
        &counted_blob(&bodies),
    )
}

/// A `vision_msgs/Detection3DArray` whose `detections` elements are bare 80-byte
/// `bbox` sections (a `Detection3D` body missing its header/results/id table), so
/// the walker refuses them → `NestedArrayOpaque` → the scan is `Absent`.
fn undecodable_detection_set_frame() -> Vec<u8> {
    let bodies: Vec<Vec<u8>> = [
        ([1.0f64, 2.0, 3.0], [0.4f64, 0.4, 1.8]),
        ([-1.0, 0.5, 2.0], [2.0, 1.0, 1.5]),
    ]
    .iter()
    .map(|(c, s)| {
        let mut v = pose_fixed_section(*c, [0.0, 0.0, 0.0, 1.0]);
        for x in s {
            v.extend_from_slice(&x.to_le_bytes());
        }
        assert_eq!(v.len(), 80, "bare bbox section drifted");
        v
    })
    .collect();
    build_header_plus_array_frame(
        "vision_msgs/Detection3DArray",
        <native_ros2_messages::vision_msgs::Detection3DArray as ShmMessage>::SCHEMA_HASH,
        "detections",
        &counted_blob(&bodies),
    )
}

/// A `diagnostic_msgs/DiagnosticArray` with `n` element bodies the walker refuses
/// (a `DiagnosticStatus` carries four variable fields, so a body with no offset
/// table is undecodable) → `status` arrives `NestedArrayOpaque`.
///
/// **Why this schema.** It must be genuinely UNMAPPED (absent from
/// `classify_schema` AND not shape-inferable) so the fallback reason is
/// `UnmappedSchema`, and it must carry a TOP-LEVEL variable-element array so the
/// opaque half co-occurs. `DiagnosticArray` is both. (`visualization_msgs/MarkerArray`
/// has a real archetype and is no longer unmapped, so it no longer serves as the
/// vehicle here; its opaque-array behaviour is pinned
/// by `marker_array_test`.)
///
/// The blob's byte length tracks `n`, which is what makes it the size-VARYING
/// vehicle: on a real DiagnosticArray it also shifts with any status message.
fn undecodable_diagnostic_array_frame(n: usize) -> Vec<u8> {
    let bodies: Vec<Vec<u8>> = (0..n)
        .map(|i| pose_fixed_section([i as f64, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]))
        .collect();
    build_header_plus_array_frame(
        "diagnostic_msgs/DiagnosticArray",
        <native_ros2_messages::diagnostic_msgs::DiagnosticArray as ShmMessage>::SCHEMA_HASH,
        "status",
        &counted_blob(&bodies),
    )
}

/// A `geometry_msgs/PolygonStamped` whose `polygon.points` blob is not a multiple
/// of `Point32`'s 12-byte stride, so the walker refuses it ONE HOP DOWN — the
/// reachable shape where the archetype-level reason is the correct answer, because
/// no TOP-LEVEL array field is the culprit.
fn nested_undecodable_polygon_frame() -> Vec<u8> {
    let body = polygon_body(&[7u8; 13]);
    build_header_plus_array_frame(
        "geometry_msgs/PolygonStamped",
        <PolygonStamped as ShmMessage>::SCHEMA_HASH,
        "polygon",
        &body,
    )
}

#[test]
fn path_frame_renders_a_polyline_and_its_vertices() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    // THE issue's acceptance case: a nav2 `/plan`. Three `PoseStamped` elements
    // with hand-written positions, canonical COUNTED framing (each element carries
    // its own `Header`, so each body has its own offset table).
    let oracle: [[f64; 3]; 3] = [[1.0, 2.0, 3.0], [-4.5, 5.25, 6.125], [7.0, 8.0, 9.0]];
    let bodies: Vec<Vec<u8>> = oracle
        .iter()
        .map(|p| pose_stamped_body(*p, [0.0, 0.0, 0.0, 1.0], "map"))
        .collect();
    let frame = build_header_plus_array_frame(
        "nav_msgs/Path",
        <native_ros2_messages::nav_msgs::Path as ShmMessage>::SCHEMA_HASH,
        "poses",
        &counted_blob(&bodies),
    );

    // Content seam: the extractor against a HAND oracle (f64 → f32 narrowed).
    let fv = walker.walk_by_hash(&frame).expect("walk Path");
    let parts = match scan_element_arrays(&fv) {
        ElementArrayScan::Geometry(p) => p,
        other => panic!("expected Geometry, got {other:?}"),
    };
    assert_eq!(parts.field, "poses");
    assert_eq!(parts.truncated, 0);
    assert_eq!(
        parts.geometry,
        ElementGeometry::Path(vec![[1.0, 2.0, 3.0], [-4.5, 5.25, 6.125], [7.0, 8.0, 9.0],]),
        "stamped elements ⇒ an ORDERED path"
    );
    // And the classifier agrees (here via the NAMED table, which is what makes an
    // idle `/plan` render too).
    assert_eq!(
        classify_schema(&fv.schema_name),
        Some(ArchetypeKind::Path3D)
    );

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "plan", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        2,
        "one LineStrips3D polyline + one Points3D of its vertices"
    );
    assert!(!state.took_anyvalues_fallback("nav_msgs/Path"));
    // The polyline lands in the 3D scene, never a 2D pane. The dump-companion rule adds the
    // status pane the SAME arm degrades into when a frame carries no decodable
    // element array — this frame did, so nothing was dumped (asserted above).
    assert_eq!(
        views_for_archetype(ArchetypeKind::Path3D),
        &[ViewKind::Spatial3d, ViewKind::TextDocument]
    );
}

#[test]
fn pose_array_frame_renders_one_points3d_not_a_polyline() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    // `geometry_msgs/PoseArray`: `Pose` is recursively FIXED, so its elements ride
    // the canonical STRIDE form — back-to-back 56-byte sections, NO count prefix.
    let oracle: [[f64; 3]; 2] = [[1.0, 2.0, 3.0], [-4.5, 5.25, 6.125]];
    let mut blob = Vec::new();
    for p in oracle {
        blob.extend_from_slice(&pose_fixed_section(p, [0.0, 0.0, 0.0, 1.0]));
    }
    assert_eq!(blob.len(), 2 * 56, "no count prefix in the stride form");
    let frame = build_header_plus_array_frame(
        "geometry_msgs/PoseArray",
        <native_ros2_messages::geometry_msgs::PoseArray as ShmMessage>::SCHEMA_HASH,
        "poses",
        &blob,
    );

    let fv = walker.walk_by_hash(&frame).expect("walk PoseArray");
    // Bare `Pose` elements carry no per-element stamp ⇒ UNORDERED points. Same
    // extractor, different geometry from the path above — the discriminator.
    assert_eq!(
        scan_element_arrays(&fv).parts().map(|p| p.geometry.clone()),
        Some(ElementGeometry::Points(vec![
            [1.0, 2.0, 3.0],
            [-4.5, 5.25, 6.125],
        ]))
    );
    assert_eq!(
        classify_schema(&fv.schema_name),
        Some(ArchetypeKind::PoseArray3D)
    );

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "poses", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "exactly one Points3D chunk — and NO polyline (the path arm logs 2)"
    );
    assert!(!state.took_anyvalues_fallback("geometry_msgs/PoseArray"));
}

/// Build a `geometry_msgs/Polygon` wire frame: one variable field `points`
/// carrying N `Point32` sections in the canonical STRIDE form (Point32 is
/// recursively FIXED — 3 × f32 = 12 bytes each, NO count prefix), and NO header
/// (Polygon is a bare `Point32[]`).
fn build_polygon_frame(points: &[[f32; 3]]) -> Vec<u8> {
    let l = layout_of("geometry_msgs/Polygon");
    assert_eq!(l.fixed_size, 0, "Polygon gained a fixed section");
    assert_eq!(
        l.variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["points"],
        "Polygon variable-field shape drifted — update this builder"
    );
    let mut blob = Vec::new();
    for p in points {
        for c in p {
            blob.extend_from_slice(&c.to_le_bytes());
        }
    }
    assert_eq!(
        blob.len(),
        points.len() * 12,
        "Point32 stride drifted (no count prefix in the fixed-element form)"
    );
    let table = l.offset_table_bytes();
    let mut payload = vec![0u8; table];
    write_offset_entry(&mut payload, 0, 0, table as u32, blob.len() as u32);
    payload.extend_from_slice(&blob);
    let mut frame = vec![0u8; WireHeader::SIZE];
    let header = WireHeader {
        schema_hash: <native_ros2_messages::geometry_msgs::Polygon as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: WireHeader::SIZE as u32,
        offset_table_count: l.variable_fields.len() as u32,
        sequence: 0,
        timestamp_ns: 42_000,
    };
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

#[test]
fn polygon_frame_name_mapped_to_path3d_renders_a_polyline_not_points() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    // A nav2 footprint: `geometry_msgs/Polygon`, UNSTAMPED `Point32` elements. A
    // ring's order is semantic but no element shape carries it, so Polygon is
    // NAME-mapped to Path3D — and the render must honor that classification instead
    // of re-deciding by shape (the classify-vs-render defect).
    let oracle: [[f32; 3]; 3] = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]];
    let frame = build_polygon_frame(&oracle);
    let fv = walker.walk_by_hash(&frame).expect("walk Polygon");

    // The name map classifies it Path3D...
    assert_eq!(
        classify_schema(&fv.schema_name),
        Some(ArchetypeKind::Path3D)
    );
    // ... while the SHAPE ladder ALONE sees unordered points (the record of
    // what shape can and cannot decide: Polygon and GridCells are byte-identical).
    assert_eq!(
        scan_element_arrays(&fv).parts().map(|p| p.geometry.clone()),
        Some(ElementGeometry::Points(oracle.to_vec())),
        "shape alone cannot know a Polygon's ring order"
    );
    // ...and the classified-kind promotion turns it into an ORDERED path.
    assert_eq!(
        scan_element_arrays_for_kind(&fv, ArchetypeKind::Path3D.forces_ordered_elements())
            .parts()
            .map(|p| p.geometry.clone()),
        Some(ElementGeometry::Path(oracle.to_vec()))
    );

    // The real dispatch draws the polyline + its vertices (2 chunks), NOT a single
    // Points3D. Reverting the kind-threading drops this to 1 (points only) — FAILS.
    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "footprint", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        2,
        "one LineStrips3D polyline + one Points3D of its vertices (the path arm), \
         not the single Points3D a shape-only render produces"
    );
    assert!(!state.took_anyvalues_fallback("geometry_msgs/Polygon"));
    // Blueprint-vs-render agreement now holds: Path3D advertises exactly the two
    // component families the render just produced.
    assert_eq!(
        archetype_components(ArchetypeKind::Path3D),
        &["LineStrips3D", "Points3D"]
    );
    // Plus the status pane for the frames that carry NO decodable
    // element array — the geometry view is unchanged and still first.
    assert_eq!(
        views_for_archetype(ArchetypeKind::Path3D),
        &[ViewKind::Spatial3d, ViewKind::TextDocument]
    );
}

#[test]
fn detection3d_array_frame_renders_one_multi_instance_boxes3d() {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    // `vision_msgs/Detection3DArray`: VARIABLE elements (each `Detection3D` has a
    // header, a nested `results` array and an `id` string), the bounding box one hop down
    // in `bbox`.
    let oracle: [([f64; 3], [f64; 3]); 3] = [
        ([1.0, 2.0, 3.0], [0.4, 0.4, 1.8]),
        ([-1.0, 0.5, 2.0], [2.0, 1.0, 1.5]),
        ([4.0, 4.0, 0.0], [0.5, 0.5, 0.5]),
    ];
    let bodies: Vec<Vec<u8>> = oracle
        .iter()
        .enumerate()
        .map(|(i, (c, s))| detection3d_body(*c, [0.0, 0.0, 0.0, 1.0], *s, &format!("obj-{i}")))
        .collect();
    let frame = build_header_plus_array_frame(
        "vision_msgs/Detection3DArray",
        <native_ros2_messages::vision_msgs::Detection3DArray as ShmMessage>::SCHEMA_HASH,
        "detections",
        &counted_blob(&bodies),
    );

    let fv = walker.walk_by_hash(&frame).expect("walk Detection3DArray");
    assert_eq!(
        scan_element_arrays(&fv).parts().map(|p| p.geometry.clone()),
        Some(ElementGeometry::Boxes(
            oracle
                .iter()
                .map(|(c, s)| BoxParts {
                    center: [c[0] as f32, c[1] as f32, c[2] as f32],
                    size: [s[0] as f32, s[1] as f32, s[2] as f32],
                    rotation: Some([0.0, 0.0, 0.0, 1.0]),
                })
                .collect()
        )),
        "the bounding-box rung wins over the pose rung, one BoxParts per detection"
    );
    assert_eq!(
        classify_schema(&fv.schema_name),
        Some(ArchetypeKind::Boxes3D)
    );

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "detections", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "THREE detections → ONE multi-instance Boxes3D chunk (not three)"
    );
    assert!(!state.took_anyvalues_fallback("vision_msgs/Detection3DArray"));
}

#[traced_test]
#[test]
fn an_undecodable_detection_set_reports_exactly_one_reason_and_it_names_the_field() {
    // Canonical element framing — THE headline. A `vision_msgs/Detection3DArray` whose
    // `detections` element bytes the walker refused used to emit TWO lines for ONE
    // condition:
    //
    //   1. the BOX reason ("a box / occupancy-grid archetype ... no extractable
    //      geometry (a box needs a reachable `center`/`pose`; a grid needs
    //      `info.width`/`info.height` ...)"), chosen by the CLASSIFIED archetype, and
    //   2. a second framing warn naming `detections` in internal vocabulary
    //      ("NOT canonically framed", the canonical-v1 byte spec, a module path).
    //
    // Both were reachable because (1) was picked from the CALLING ARM while (2) fired
    // unconditionally beside it. (1) is also the WRONG place to look for this frame:
    // the boxes are present, one hop inside an array whose elements would not decode,
    // so `center`/`pose` and `info.width` are a wasted debug cycle.
    //
    // The reason is resolved from the FRAME: a refused TOP-LEVEL array is
    // the most specific cause and the only one that names something an operator can
    // act on, so it OVERRIDES the arm's category reason and is the ONLY line.
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let frame = undecodable_detection_set_frame();

    let fv = walker.walk_by_hash(&frame).expect("walk Detection3DArray");
    // Premises: classifies Boxes3D, carries no top-level box, the element array is
    // undecodable → exactly the branch the fix touches.
    assert_eq!(
        classify_schema(&fv.schema_name),
        Some(ArchetypeKind::Boxes3D)
    );
    assert!(
        box3d_parts(&fv).is_none(),
        "premise: no top-level box, so log_boxes3d_from_frame fails"
    );
    assert_eq!(
        scan_element_arrays(&fv),
        ElementArrayScan::Absent,
        "premise: the non-canonical detections array is undecodable"
    );

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "detections", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "exactly one TextDocument field dump — never a mis-drawn shape"
    );
    assert!(state.took_anyvalues_fallback("vision_msgs/Detection3DArray"));

    // ONE reason, and it names the field. The two superseded lines are absent.
    logs_assert(|lines: &[&str]| {
        let count = |m: &str| lines.iter().filter(|l| l.contains(m)).count();
        let named = lines
            .iter()
            .filter(|l| l.contains(ELEMENT_BYTES_MARKER) && l.contains("detections"))
            .count();
        match (
            count(ELEMENT_BYTES_MARKER),
            named,
            count(BOX_MARKER),
            count(ELEMENT_ARRAY_MARKER),
            count(DELETED_SECOND_DIAGNOSTIC),
        ) {
            (1, 1, 0, 0, 0) => Ok(()),
            got => Err(format!(
                "want (element-bytes, names `detections`, box, element-array, deleted) \
                 = (1, 1, 0, 0, 0), got {got:?}; lines: {lines:?}"
            )),
        }
    });

    // ANTI-TAUTOLOGY — the two zeroes above are real absences, not an apparatus that
    // cannot see those strings. The SAME `logs_assert` capture, driven with the two
    // frames that DO raise each category reason, must now count exactly one of each,
    // while the element-bytes count stays at 1 (neither of them added one).
    let occ = build_occupancy_grid(4, 4, &[0, 0, 0]); // cells shorter than width*height
    dispatch_frame(&rec, &walker, "map", &occ, &mut state);
    let nested = nested_undecodable_polygon_frame(); // array one hop down
    dispatch_frame(&rec, &walker, "footprint", &nested, &mut state);
    logs_assert(|lines: &[&str]| {
        let count = |m: &str| lines.iter().filter(|l| l.contains(m)).count();
        match (
            count(BOX_MARKER),
            count(ELEMENT_ARRAY_MARKER),
            count(ELEMENT_BYTES_MARKER),
        ) {
            (1, 1, 1) => Ok(()),
            got => Err(format!(
                "anti-tautology: both category markers must be OBSERVABLE here \
                 (want (box, element-array, element-bytes) = (1, 1, 1), got {got:?})"
            )),
        }
    });
}

#[traced_test]
#[test]
fn an_undecodable_path_array_reports_one_field_naming_line_per_input() {
    // The `Path3D` twin of the headline: a `nav_msgs/Path` whose `poses` element
    // bodies carry no offset table. Same contract — exactly ONE line, naming `poses`,
    // with the archetype-level reason and the deleted framing warn both absent.
    //
    // Also pins the LATCH: once per INPUT (element encoding is a property of the
    // PRODUCER, so one schema's natively-produced topic and its bridged twin are
    // different facts and each earns a line), never once per frame.
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let frame = undecodable_path_frame();

    let fv = walker.walk_by_hash(&frame).expect("walk Path");
    assert!(
        matches!(
            fv.field("poses"),
            Some(cerulion_core::codegen::FrameValueKind::NestedArrayOpaque(_))
        ),
        "premise: a packed element body must stay OPAQUE, got {:?}",
        fv.field("poses")
    );
    assert_eq!(scan_element_arrays(&fv), ElementArrayScan::Absent);

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "plan", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "exactly one TextDocument field dump — never a polyline guessed from bytes"
    );
    assert!(state.took_anyvalues_fallback("nav_msgs/Path"));

    // Five more frames on the SAME input add no more lines.
    for _ in 0..5 {
        dispatch_frame(&rec, &walker, "plan", &frame, &mut state);
    }
    logs_assert(|lines: &[&str]| {
        let count = |m: &str| lines.iter().filter(|l| l.contains(m)).count();
        let named = lines
            .iter()
            .filter(|l| l.contains(ELEMENT_BYTES_MARKER) && l.contains("poses"))
            .count();
        match (
            count(ELEMENT_BYTES_MARKER),
            named,
            count(ELEMENT_ARRAY_MARKER),
            count(DELETED_SECOND_DIAGNOSTIC),
        ) {
            (1, 1, 0, 0) => Ok(()),
            got => Err(format!(
                "want (element-bytes, names `poses`, element-array, deleted) = (1, 1, 0, 0), \
                 got {got:?}; lines: {lines:?}"
            )),
        }
    });

    // A DIFFERENT input carrying the same schema is a different producer fact — it
    // earns its own line (and doubles as the anti-tautology for the count above:
    // the latch really is what held it at 1, not a silent path).
    dispatch_frame(&rec, &walker, "plan_b", &frame, &mut state);
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains(ELEMENT_BYTES_MARKER))
            .count();
        if n == 2 {
            Ok(())
        } else {
            Err(format!(
                "a second INPUT must report too; want 2 lines, got {n}"
            ))
        }
    });
}

#[traced_test]
#[test]
fn a_nested_undecodable_array_keeps_the_archetype_level_reason() {
    // The complement: the archetype-level reason is NOT dead code and NOT silently
    // superseded. `opaque_element_arrays` inspects TOP-LEVEL fields only, so a
    // `geometry_msgs/PolygonStamped` whose `polygon.points` the walker refused ONE
    // HOP DOWN reaches `Absent` with nothing top-level to name — and there the
    // caller's category reason is the correct answer.
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let frame = nested_undecodable_polygon_frame();

    let fv = walker.walk_by_hash(&frame).expect("walk PolygonStamped");
    // Premises: Path3D-classified, scan Absent, and NO top-level opaque array — so
    // the frame-derived override must NOT fire.
    assert_eq!(
        classify_schema(&fv.schema_name),
        Some(ArchetypeKind::Path3D)
    );
    assert_eq!(scan_element_arrays(&fv), ElementArrayScan::Absent);
    assert!(
        cerulion_viz::archetype::opaque_element_arrays(&fv).is_empty(),
        "premise: the refused array is one hop down, invisible to the top-level scan"
    );

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "footprint", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(storage.num_msgs() - baseline, 1, "one field-dump chunk");
    assert!(state.took_anyvalues_fallback("geometry_msgs/PolygonStamped"));
    logs_assert(|lines: &[&str]| {
        let count = |m: &str| lines.iter().filter(|l| l.contains(m)).count();
        match (count(ELEMENT_ARRAY_MARKER), count(ELEMENT_BYTES_MARKER)) {
            (1, 0) => Ok(()),
            got => Err(format!(
                "want (element-array, element-bytes) = (1, 0), got {got:?}; lines: {lines:?}"
            )),
        }
    });

    // ANTI-TAUTOLOGY for the zero: the element-bytes line IS observable in this same
    // capture — a top-level refused array raises it.
    let top_level = undecodable_path_frame();
    dispatch_frame(&rec, &walker, "plan", &top_level, &mut state);
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains(ELEMENT_BYTES_MARKER))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!(
                "anti-tautology: the element-bytes line must be observable here, got {n}"
            ))
        }
    });
}

#[traced_test]
#[test]
fn a_drawable_element_array_emits_no_fallback_diagnostic_at_all() {
    // CONTROL: the happy path stays silent. A canonically-framed detection set draws
    // N boxes and produces NONE of the four fallback lines — so every count above is
    // measuring a real degrade, not ambient chatter from the render path.
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let bodies: Vec<Vec<u8>> = [
        ([1.0f64, 2.0, 3.0], [0.4f64, 0.4, 1.8]),
        ([-1.0, 0.5, 2.0], [2.0, 1.0, 1.5]),
    ]
    .iter()
    .enumerate()
    .map(|(i, (c, s))| detection3d_body(*c, [0.0, 0.0, 0.0, 1.0], *s, &format!("obj-{i}")))
    .collect();
    let frame = build_header_plus_array_frame(
        "vision_msgs/Detection3DArray",
        <native_ros2_messages::vision_msgs::Detection3DArray as ShmMessage>::SCHEMA_HASH,
        "detections",
        &counted_blob(&bodies),
    );

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "detections", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "one multi-instance Boxes3D chunk"
    );
    assert!(!state.took_anyvalues_fallback("vision_msgs/Detection3DArray"));
    logs_assert(|lines: &[&str]| {
        let noisy: Vec<&&str> = lines
            .iter()
            .filter(|l| {
                [
                    ELEMENT_BYTES_MARKER,
                    ELEMENT_ARRAY_MARKER,
                    BOX_MARKER,
                    DELETED_SECOND_DIAGNOSTIC,
                ]
                .iter()
                .any(|m| l.contains(m))
            })
            .collect();
        if noisy.is_empty() {
            Ok(())
        } else {
            Err(format!(
                "a drawable frame must emit no fallback line: {noisy:?}"
            ))
        }
    });

    // ANTI-TAUTOLOGY: the SAME capture does see a fallback line when one is earned.
    let bad = undecodable_detection_set_frame();
    dispatch_frame(&rec, &walker, "detections", &bad, &mut state);
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains(ELEMENT_BYTES_MARKER))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!(
                "anti-tautology: a fallback line must be observable here, got {n}"
            ))
        }
    });
}

#[test]
fn a_go2_tf_encoded_tfmessage_still_routes_to_the_tf_path() {
    // NEGATIVE CONTROL (canonical-framing risk R4): `/tf` carries a BESPOKE `go2_tf` blob
    // that the walker deliberately refuses (its element audit rejects the 76-byte
    // pseudo-element). The `Transforms` archetype must keep reading those bytes
    // through its own decoder — the element ladder must not touch it, and the new
    // opaque diagnostic must NOT fire (it is scoped to the dump path).
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let blob = encode_tf_transforms(&[TfTransform::new(
        "odom",
        "base",
        [1.0, 2.0, 3.0],
        IDENTITY_QUAT,
        7,
        0,
    )]);
    let frame = build_tf(0, 42_000, &blob);
    let fv = walker.walk_by_hash(&frame).expect("walk TFMessage");
    // The bespoke blob stays opaque, so the element ladder finds no geometry — the
    // bytes still reach `crate::tf`'s decoder unchanged.
    assert_eq!(scan_element_arrays(&fv), ElementArrayScan::Absent);
    assert_eq!(
        classify_schema(&fv.schema_name),
        Some(ArchetypeKind::Transforms),
        "the named TF mapping wins — an element-array archetype would be a regression"
    );

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "tf", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert!(
        storage.num_msgs() - baseline >= 1,
        "the TF path still logs the transform tree"
    );
    assert!(
        !state.took_anyvalues_fallback("tf2_msgs/TFMessage"),
        "/tf must NOT degrade to a field dump"
    );
}

#[traced_test]
#[test]
fn an_over_cap_element_array_renders_the_ceiling_and_warns_once_per_input() {
    // The render ceiling is REAL and LOUD: a hostile/huge producer's array is
    // clipped at `MAX_ELEMENT_INSTANCES` (an unbounded per-frame expansion is the
    // render-buffer hazard) and the drop is NAMED once per input, because a
    // silently short polyline reads as a producer bug.
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    const OVER: usize = MAX_ELEMENT_INSTANCES + 37;
    let mut blob = Vec::with_capacity(OVER * 56);
    for i in 0..OVER {
        blob.extend_from_slice(&pose_fixed_section(
            [i as f64, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ));
    }
    let frame = build_header_plus_array_frame(
        "geometry_msgs/PoseArray",
        <native_ros2_messages::geometry_msgs::PoseArray as ShmMessage>::SCHEMA_HASH,
        "poses",
        &blob,
    );

    let fv = walker.walk_by_hash(&frame).expect("walk PoseArray");
    let parts = scan_element_arrays(&fv).parts().cloned().expect("geometry");
    assert_eq!(parts.truncated, 37, "the remainder is counted exactly");
    match &parts.geometry {
        ElementGeometry::Points(p) => {
            assert_eq!(p.len(), MAX_ELEMENT_INSTANCES, "clipped at the ceiling");
            // Hand oracle on the boundary: the FIRST elements are kept.
            assert_eq!(p[0], [0.0, 0.0, 0.0]);
            assert_eq!(
                p[MAX_ELEMENT_INSTANCES - 1],
                [(MAX_ELEMENT_INSTANCES - 1) as f32, 0.0, 0.0]
            );
        }
        other => panic!("expected Points, got {other:?}"),
    }

    // TWO dispatches — the minimum that proves the report is once-per-INPUT
    // rather than once-per-frame. The element-cap change cut this from four: `dispatch_frame`
    // re-walks the frame internally, so at the raised cap (300 000) each extra
    // call re-decodes 300 000 elements for no additional claim.
    let baseline = storage.num_msgs();
    for _ in 0..2 {
        dispatch_frame(&rec, &walker, "poses", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");
    assert!(
        storage.num_msgs() - baseline >= 1,
        "the clipped point set still renders"
    );
    assert!(!state.took_anyvalues_fallback("geometry_msgs/PoseArray"));
    assert!(
        logs_contain("max_element_instances"),
        "the ceiling is named"
    );
    // ONCE per input across BOTH frames, not once per frame. (The loop
    // above was cut from four to two by the element-cap change — this line still said "four".)
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("longer than max_element_instances"))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!("expected exactly 1 truncation warn, got {n}"))
        }
    });
    // ANTI-TAUTOLOGY: an in-bounds array on a FRESH state warns nothing.
    let mut fresh = SinkState::new();
    let small = build_header_plus_array_frame(
        "geometry_msgs/PoseArray",
        <native_ros2_messages::geometry_msgs::PoseArray as ShmMessage>::SCHEMA_HASH,
        "poses",
        &pose_fixed_section([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]),
    );
    let small_fv = walker.walk_by_hash(&small).expect("walk small PoseArray");
    assert_eq!(
        scan_element_arrays(&small_fv).parts().map(|p| p.truncated),
        Some(0),
        "an in-bounds array is never marked truncated"
    );
    dispatch_frame(&rec, &walker, "small", &small, &mut fresh);
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("longer than max_element_instances"))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!(
                "the in-bounds frame must add no warn (still expected 1 total), got {n}"
            ))
        }
    });
}

// ── The undecodable-array flood latch ───────────────────────────────────────
//
// The report is latched on the CONDITION — `input::<field names>` — never on the
// human-readable line, which quotes byte sizes. A variable-length array's byte
// size shifts frame to frame (a `nav_msgs/Path` blob is `4 + Σ(4 + body)`; a
// MarkerArray's shifts with any marker text), so a size-keyed latch re-warns on
// every new size at the producer's rate AND retains one `String` per distinct size
// in an unbounded set. The disk-fill incident is the standing evidence for what a sustained log
// flood costs (a 234 GB disk).

/// `nav_msgs/Path` with `n` non-canonically-packed `poses` — [`undecodable_path_frame`]
/// with the ELEMENT COUNT (and therefore the blob's byte length) under the caller's
/// control, so a test can vary the size while holding the field-name set fixed.
fn undecodable_path_frame_of(n: usize) -> Vec<u8> {
    let bodies: Vec<Vec<u8>> = (0..n)
        .map(|i| pose_fixed_section([i as f64, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]))
        .collect();
    build_header_plus_array_frame(
        "nav_msgs/Path",
        <native_ros2_messages::nav_msgs::Path as ShmMessage>::SCHEMA_HASH,
        "poses",
        &counted_blob(&bodies),
    )
}

/// The opaque byte length of `field` in a walked frame — the number the log line
/// quotes and a size-keyed latch would key on. Used to PROVE the size really varied
/// before asserting that the warn count did not.
fn opaque_len(walker: &FrameWalker, frame: &[u8], field: &str) -> usize {
    let fv = walker.walk_by_hash(frame).expect("walk");
    cerulion_viz::archetype::opaque_element_arrays(&fv)
        .into_iter()
        .find(|(name, _)| name == field)
        .unwrap_or_else(|| panic!("premise: `{field}` must be an undecodable array"))
        .1
}

#[traced_test]
#[test]
fn an_undecodable_array_that_changes_size_does_not_re_warn_but_a_new_field_set_does() {
    // THE flood pin. Same input, same undecodable field, DIFFERENT element counts —
    // the shape a real `/plan` or MarkerArray produces every frame. Exactly ONE
    // line, forever.
    //
    // A latch key that embeds the byte SIZES (`poses (124 bytes)`) mints a new key
    // per size: this test's three frames would produce THREE warns at
    // the producer's rate, and the key set would grow without bound.
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, _storage) = memory();

    let frames = [
        undecodable_path_frame_of(2),
        undecodable_path_frame_of(5),
        undecodable_path_frame_of(3),
    ];
    // Premise: the sizes really do differ (else the count assert below is vacuous —
    // it would pass on a size-keyed latch too).
    let sizes: Vec<usize> = frames
        .iter()
        .map(|f| opaque_len(&walker, f, "poses"))
        .collect();
    let mut distinct = sizes.clone();
    distinct.sort_unstable();
    distinct.dedup();
    assert_eq!(
        distinct.len(),
        3,
        "premise: three DISTINCT blob sizes, got {sizes:?}"
    );

    for frame in &frames {
        dispatch_frame(&rec, &walker, "plan", frame, &mut state);
    }
    logs_assert(move |lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains(ELEMENT_BYTES_MARKER))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!(
                "a size-varying undecodable array must warn ONCE, not once per size \
                 (sizes {sizes:?}); got {n} lines: {lines:?}"
            ))
        }
    });

    // The documented flip side: the latch keys on the field-NAME set, so a CHANGED
    // set is a genuinely new producer fact and reports again. (On one input that is
    // a producer that starts framing one of two arrays canonically; here the same
    // condition is reached by dispatching a frame whose undecodable field is named
    // `detections` instead of `poses` on the same input.) Doubles as the
    // anti-tautology for the 1 above: the latch is what held it, not a silent path.
    dispatch_frame(
        &rec,
        &walker,
        "plan",
        &undecodable_detection_set_frame(),
        &mut state,
    );
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains(ELEMENT_BYTES_MARKER))
            .count();
        if n == 2 {
            Ok(())
        } else {
            Err(format!(
                "a CHANGED undecodable field set must report again; want 2, got {n}"
            ))
        }
    });
}

#[traced_test]
#[test]
fn an_unmapped_schema_with_undecodable_arrays_warns_per_input_never_once_per_schema() {
    // `diagnostic_msgs/DiagnosticArray` is genuinely UNMAPPED (no `classify_schema`
    // row, and its shape infers nothing), so it reaches `anyvalues_fallback` with
    // `UnmappedSchema` — and one whose `status` bodies the walker refuses is the
    // live shape that carries BOTH facts at once. (The marker render arm moved MarkerArray, the
    // previous vehicle, out of the unmapped class.)
    //
    // Reported only as a context field on the
    // once-per-SCHEMA info line, that combination would cost two things this test pins: a SECOND
    // input on the same unmapped schema would log NOTHING (its producer is a different
    // fact), and an operator running at `RUST_LOG=warn` would see neither.
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();

    let frame = undecodable_diagnostic_array_frame(2);
    let fv = walker.walk_by_hash(&frame).expect("walk DiagnosticArray");
    assert_eq!(
        classify_schema(&fv.schema_name),
        None,
        "premise: the schema is UNMAPPED, so the reason is UnmappedSchema"
    );
    assert_eq!(
        infer_archetype_from_shape(&fv),
        ArchetypeKind::AnyValues,
        "premise: nor is it shape-inferable"
    );
    assert_eq!(
        cerulion_viz::archetype::opaque_element_arrays(&fv)
            .iter()
            .map(|(n, _)| n.as_str())
            .collect::<Vec<_>>(),
        vec!["status"],
        "premise: the status array is undecodable"
    );

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "diag_a", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(storage.num_msgs() - baseline, 1, "one field-dump chunk");
    assert!(state.took_anyvalues_fallback("diagnostic_msgs/DiagnosticArray"));

    // One WARN naming the field; the pure-unmapped INFO is NOT also emitted (one
    // condition, one line).
    logs_assert(|lines: &[&str]| {
        let count = |m: &str| lines.iter().filter(|l| l.contains(m)).count();
        let named = lines
            .iter()
            .filter(|l| l.contains(UNMAPPED_OPAQUE_MARKER) && l.contains("status"))
            .count();
        match (named, count(UNMAPPED_INFO_MARKER)) {
            (1, 0) => Ok(()),
            got => Err(format!(
                "want (unmapped+opaque warn naming `status`, pure-unmapped info) = (1, 0), \
                 got {got:?}; lines: {lines:?}"
            )),
        }
    });

    // A different SIZE on the same input adds nothing (the same flood contract as
    // the element-bytes arm — both share `opaque_latch_key`).
    dispatch_frame(
        &rec,
        &walker,
        "diag_a",
        &undecodable_diagnostic_array_frame(7),
        &mut state,
    );
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains(UNMAPPED_OPAQUE_MARKER))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!("a size change must not re-warn; want 1, got {n}"))
        }
    });

    // THE per-input pin: a SECOND input carrying the same unmapped schema is a
    // different producer and earns its own line. A schema-keyed latch alone logs
    // zero here (it has already fired on `diag_a`).
    dispatch_frame(&rec, &walker, "diag_b", &frame, &mut state);
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains(UNMAPPED_OPAQUE_MARKER))
            .count();
        if n == 2 {
            Ok(())
        } else {
            Err(format!(
                "a second INPUT on the same unmapped schema must report too; want 2, got {n}"
            ))
        }
    });
}

#[traced_test]
#[test]
fn an_unmapped_schema_with_no_undecodable_arrays_keeps_the_once_per_schema_info() {
    // The CONTROL for the split: without undecodable arrays the unmapped reason is
    // unchanged — the per-SCHEMA info, and none of the opaque wording. Without this,
    // a regression that routed EVERY unmapped frame through the new warn would pass
    // the test above.
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, _storage) = memory();

    let frame = build_bool(true);
    dispatch_frame(&rec, &walker, "estop", &frame, &mut state);
    dispatch_frame(&rec, &walker, "estop_b", &frame, &mut state);
    assert!(state.took_anyvalues_fallback("std_msgs/Bool"));
    logs_assert(|lines: &[&str]| {
        let count = |m: &str| lines.iter().filter(|l| l.contains(m)).count();
        match (count(UNMAPPED_INFO_MARKER), count(UNMAPPED_OPAQUE_MARKER)) {
            // Once per SCHEMA — the second input adds nothing, which is correct
            // here: with no undecodable array there is no per-producer fact.
            (1, 0) => Ok(()),
            got => Err(format!(
                "want (pure-unmapped info, unmapped+opaque warn) = (1, 0), got {got:?}; \
                 lines: {lines:?}"
            )),
        }
    });
}

// ── The shape-inference MEMO on the unmapped ("automagic") path ─────
//
// The defect: `classify_and_route` re-ran `infer_archetype_from_shape` on every
// frame. Its element rung runs the cap-governed `scan_element_arrays` and drops
// the geometry, then the render arm scans the same array again — so an UNMAPPED
// element topic paid the ceiling-governed scan TWICE per frame (measured
// ~90 ms/frame at the 300 000 cap; see `tests/element_cap_bench.rs`), on exactly
// the unseen-vendor path the ladder exists for. Every arm below is anchored to a
// hand oracle: the element positions the frame was BUILT from, or a count
// computed by hand.

/// A `control_msgs/MotionPrimitive` carrying `poses` as `PoseStamped` elements —
/// the UNMAPPED twin of `nav_msgs/Path`.
///
/// Chosen for two reasons, both load-bearing: it is absent from
/// [`classify_schema`]'s table (so it reaches the shape ladder — asserted in the
/// tests below, never assumed), and its element type is the SAME
/// `geometry_msgs/PoseStamped` a nav2 `/plan` carries, so the mapped and unmapped
/// paths differ ONLY in how the archetype is decided.
///
/// Its variable fields are `[additional_arguments, poses, joint_positions]`; the
/// two the sink does not read are written EMPTY, which also makes
/// `additional_arguments` an empty first element-array candidate the scan must
/// skip past to reach `poses`.
fn motion_primitive_frame(positions: &[[f64; 3]]) -> Vec<u8> {
    motion_primitive_frame_at(positions, 42_000)
}

/// [`motion_primitive_frame`] with an explicit publisher WIRE timestamp — what a
/// test feeding a STREAM of frames on one topic needs, since the series gate keys a plot
/// topic's sample rate on exactly that advancement.
fn motion_primitive_frame_at(positions: &[[f64; 3]], timestamp_ns: u64) -> Vec<u8> {
    let l = layout_of("control_msgs/MotionPrimitive");
    assert_eq!(
        l.variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["additional_arguments", "poses", "joint_positions"],
        "MotionPrimitive variable-field declaration order changed — update this builder"
    );
    assert_eq!(
        (l.fixed_size, l.offset_table_bytes()),
        (16, 24),
        "MotionPrimitive shape drifted — update this builder"
    );
    let bodies: Vec<Vec<u8>> = positions
        .iter()
        .map(|p| pose_stamped_body(*p, [0.0, 0.0, 0.0, 1.0], "map"))
        .collect();
    let blob = counted_blob(&bodies);

    let fixed = l.fixed_size;
    let table = l.offset_table_bytes();
    let base = (fixed + table) as u32;
    let mut payload = vec![0u8; fixed + table];
    write_offset_entry(&mut payload, fixed, 0, base, 0); // additional_arguments: empty
    write_offset_entry(&mut payload, fixed, 1, base, blob.len() as u32); // poses
    write_offset_entry(&mut payload, fixed, 2, base + blob.len() as u32, 0); // joint_positions
    payload.extend_from_slice(&blob);

    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash:
            <native_ros2_messages::control_msgs::MotionPrimitive as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: WireHeader::SIZE as u32,
        offset_table_count: l.variable_fields.len() as u32,
        sequence: 0,
        timestamp_ns,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

/// The `[f32; 3]` vertices `motion_primitive_frame(positions)` must render — the
/// hand oracle, derived from the positions the frame was built from rather than
/// from anything the code under test produced.
fn motion_vertex_oracle(positions: &[[f64; 3]]) -> Vec<[f32; 3]> {
    positions
        .iter()
        .map(|p| [p[0] as f32, p[1] as f32, p[2] as f32])
        .collect()
}

#[test]
fn an_unmapped_element_topic_infers_once_across_many_frames() {
    // THE headline pin. N frames of one unmapped element topic must run the
    // shape ladder exactly ONCE — reverting the memo makes this read N.
    const FRAMES: u64 = 5;
    let oracle: [[f64; 3]; 3] = [[1.0, 2.0, 3.0], [-4.5, 5.25, 6.125], [7.0, 8.0, 9.0]];
    let frame = motion_primitive_frame(&oracle);
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();

    // ANTI-TAUTOLOGY: this really is the unmapped path, and the ladder really
    // does reach its element rung on this frame.
    assert_eq!(
        classify_schema("control_msgs/MotionPrimitive"),
        None,
        "the arm is only meaningful while this schema is UNMAPPED"
    );
    let fv = walker.walk_by_hash(&frame).expect("walk MotionPrimitive");
    assert_eq!(
        infer_archetype_from_shape(&fv),
        ArchetypeKind::Path3D,
        "stamped PoseStamped elements infer to an ordered path"
    );
    // Hand oracle on the CONTENT the render arm draws: the positions the frame
    // was built from, in order.
    match scan_element_arrays(&fv).parts().map(|p| p.geometry.clone()) {
        Some(ElementGeometry::Path(v)) => assert_eq!(v, motion_vertex_oracle(&oracle)),
        other => panic!("expected a Path geometry, got {other:?}"),
    }

    let baseline = storage.num_msgs();
    for _ in 0..FRAMES {
        dispatch_frame(&rec, &walker, "motion", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");

    assert_eq!(
        state.inference_runs(),
        1,
        "the shape ladder runs ONCE per (input, schema), not once per frame"
    );
    assert_eq!(
        state.cached_archetype("motion"),
        Some(ArchetypeKind::Path3D),
        "the memo holds the inferred archetype"
    );
    // The memo changed WHEN the ladder runs, never WHAT is drawn: the topic
    // still renders its polyline + vertices. (Same-entity logs COMPACT, so this
    // count cannot tell 1 rendered from N — the warm-vs-cold equivalence is
    // pinned by `the_memo_renders_identically_to_a_cold_classification_on_a_steady_stream`.)
    assert_eq!(
        storage.num_msgs() - baseline,
        2,
        "one LineStrips3D polyline + one Points3D of its vertices"
    );
    assert!(!state.took_anyvalues_fallback("control_msgs/MotionPrimitive"));
}

#[test]
fn a_name_mapped_element_topic_never_runs_the_inference_ladder() {
    // The CONTROL that makes the count above meaningful: the name-mapped half
    // (the one the element-cap bench's 0.32 us/element slope was measured on) never infers, so
    // it never memoizes either.
    let oracle: [[f64; 3]; 2] = [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]];
    let bodies: Vec<Vec<u8>> = oracle
        .iter()
        .map(|p| pose_stamped_body(*p, [0.0, 0.0, 0.0, 1.0], "map"))
        .collect();
    let frame = build_header_plus_array_frame(
        "nav_msgs/Path",
        <native_ros2_messages::nav_msgs::Path as ShmMessage>::SCHEMA_HASH,
        "poses",
        &counted_blob(&bodies),
    );
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, _storage) = memory();
    assert_eq!(
        classify_schema("nav_msgs/Path"),
        Some(ArchetypeKind::Path3D),
        "the control is only meaningful while this schema IS mapped"
    );

    for _ in 0..5 {
        dispatch_frame(&rec, &walker, "plan", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");

    assert_eq!(
        state.inference_runs(),
        0,
        "a name-mapped schema never reaches the shape ladder"
    );
    assert_eq!(
        state.cached_archetype("plan"),
        None,
        "and therefore never memoizes"
    );
}

#[test]
fn an_unmapped_topic_whose_array_is_empty_re_infers_until_it_carries_elements() {
    // THE correctness pin for `KindStability::ElementsUndecided`, and the kill
    // for a memo that caches unconditionally: an idle producer's frames carry NO
    // elements, so the ladder answers from a later rung. Freezing that answer is
    // the frame-dependent-layout failure mode: every later populated frame logged
    // into a view that does not exist.
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, _storage) = memory();

    let idle = motion_primitive_frame(&[]);
    let idle_fv = walker.walk_by_hash(&idle).expect("walk idle");
    // Hand oracle: with no elements the element rung yields nothing, so the
    // numeric harvest (`type`, `blend_radius`) answers instead.
    assert_eq!(infer_archetype_from_shape(&idle_fv), ArchetypeKind::Scalars);

    // The frames carry ADVANCING wire stamps (10 ms apart, a 100 Hz
    // producer). A `Scalars` topic is remembered as fully-gated, so a frame the
    // rate gate REFUSES is now dropped on its header and never reaches the
    // ladder — with all three frames stamped identically this loop would infer
    // ONCE and the re-inference contract under test would be invisible. The
    // gate's own effect on re-inference is pinned separately by
    // `a_rate_refused_frame_does_not_re_run_the_classification_ladder`.
    for i in 0..3u64 {
        let frame = motion_primitive_frame_at(&[], i * 10_000_000);
        dispatch_frame(&rec, &walker, "motion", &frame, &mut state);
    }
    assert_eq!(
        state.cached_archetype("motion"),
        None,
        "an answer reached PAST the element rung is never memoized"
    );
    assert_eq!(
        state.inference_runs(),
        3,
        "so each idle frame re-infers — the cost of not lying about the layout"
    );

    // The producer starts publishing motion: the SAME topic must now classify as
    // a path, which a frozen memo could never do.
    let oracle: [[f64; 3]; 2] = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]];
    let busy = motion_primitive_frame(&oracle);
    let busy_fv = walker.walk_by_hash(&busy).expect("walk busy");
    match scan_element_arrays(&busy_fv)
        .parts()
        .map(|p| p.geometry.clone())
    {
        Some(ElementGeometry::Path(v)) => assert_eq!(v, motion_vertex_oracle(&oracle)),
        other => panic!("expected a Path geometry, got {other:?}"),
    }
    let busy = motion_primitive_frame_at(&oracle, 30_000_000);
    dispatch_frame(&rec, &walker, "motion", &busy, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        state.cached_archetype("motion"),
        Some(ArchetypeKind::Path3D),
        "the first frame that DOES carry elements decides, and is memoized"
    );
    assert_eq!(state.inference_runs(), 4);
}

#[test]
fn a_topic_that_changes_schema_re_infers_instead_of_replaying_the_old_answer() {
    // The memo is keyed by INPUT (so a hit costs no allocation), which makes the
    // schema it was derived from part of the VALUE and load-bearing: a topic
    // re-pointed at a different producer must re-infer. A memo keyed on the input
    // alone would render the previous producer's archetype forever.
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, _storage) = memory();

    let path = motion_primitive_frame(&[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
    dispatch_frame(&rec, &walker, "x", &path, &mut state);
    assert_eq!(
        state.cached_archetype("x"),
        Some(ArchetypeKind::Path3D),
        "first producer: an unmapped element array"
    );
    assert_eq!(state.inference_runs(), 1);

    // A DIFFERENT unmapped schema on the same input — one whose answer is ALSO
    // memoizable, so the assertion is Some(A) -> Some(B) rather than merely "the
    // stale entry went away".
    let attitude = build_quaternion_stamped([0.0, 0.0, 0.0, 1.0]);
    let attitude_fv = walker.walk_by_hash(&attitude).expect("walk attitude");
    assert_eq!(classify_schema("geometry_msgs/QuaternionStamped"), None);
    assert_eq!(
        infer_archetype_from_shape(&attitude_fv),
        ArchetypeKind::Transform3D,
        "hand oracle for the second producer's archetype"
    );
    dispatch_frame(&rec, &walker, "x", &attitude, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        state.cached_archetype("x"),
        Some(ArchetypeKind::Transform3D),
        "the new schema's answer replaces the old one"
    );
    assert_eq!(
        state.inference_runs(),
        2,
        "the schema change forced a real re-inference"
    );

    // THIRD producer, whose inference is `ElementsUndecided` — the arm that
    // REMOVES the entry rather than replacing it. Without it the memo would hold
    // a superseded answer that `cached_archetype` reports as live.
    let bare = build_bool(true);
    let bare_fv = walker.walk_by_hash(&bare).expect("walk Bool");
    assert_eq!(classify_schema("std_msgs/Bool"), None);
    assert_eq!(
        infer_archetype_from_shape(&bare_fv),
        ArchetypeKind::AnyValues,
        "a lone bool is deliberately not plotted — it reaches the dump PAST the \
         element rung, so its answer is never memoized"
    );
    dispatch_frame(&rec, &walker, "x", &bare, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        state.cached_archetype("x"),
        None,
        "an undecided answer CLEARS the superseded entry — it never lingers"
    );
    assert_eq!(state.inference_runs(), 3);
}

#[test]
fn an_input_that_switches_to_a_name_mapped_schema_is_answered_by_the_table() {
    // The transition the memo deliberately does NOT tidy: once an input carries a
    // NAME-MAPPED schema the table answers first and the memo is never consulted
    // for it again, so the old entry is inert rather than cleared (paying a map
    // probe on every frame of every mapped topic to tidy a value nothing reads is
    // the wrong trade). This pins the BEHAVIOUR — the rendered archetype follows
    // the table, not the stale memo — and the accessor's documented wording.
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();

    let unmapped = motion_primitive_frame(&[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
    dispatch_frame(&rec, &walker, "x", &unmapped, &mut state);
    assert_eq!(state.cached_archetype("x"), Some(ArchetypeKind::Path3D));

    // A name-mapped PoseArray on the SAME input: unordered `Pose` elements, so the
    // table's `PoseArray3D` must win over the memo's `Path3D` — one Points3D chunk
    // and NO polyline+vertices pair.
    let oracle: [[f64; 3]; 2] = [[1.0, 2.0, 3.0], [-4.5, 5.25, 6.125]];
    let mut blob = Vec::new();
    for p in oracle {
        blob.extend_from_slice(&pose_fixed_section(p, [0.0, 0.0, 0.0, 1.0]));
    }
    let mapped = build_header_plus_array_frame(
        "geometry_msgs/PoseArray",
        <native_ros2_messages::geometry_msgs::PoseArray as ShmMessage>::SCHEMA_HASH,
        "poses",
        &blob,
    );
    assert_eq!(
        classify_schema("geometry_msgs/PoseArray"),
        Some(ArchetypeKind::PoseArray3D)
    );

    let baseline = storage.num_msgs();
    let before = state.inference_runs();
    dispatch_frame(&rec, &walker, "x", &mapped, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        state.inference_runs(),
        before,
        "a name-mapped frame never reaches the shape ladder, memo or not"
    );
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "one Points3D chunk — the TABLE's answer, not the memo's polyline+vertices"
    );
}

/// An UNSEEN VENDOR type carrying TWO element arrays of different order-semantics:
/// a stamped trajectory (⇒ `ElementGeometry::Path`) and an unstamped waypoint bag
/// (⇒ `ElementGeometry::Points`), only one of which is populated per frame.
///
/// This shape is what makes the promotion decoupling OBSERVABLE, and no
/// built-in carries it — every vendored message with a `PoseStamped[]` has no
/// unstamped pose array beside it (checked across the corpus), so the divergence
/// cannot be reached with a stock schema. It is declared here in `.msg` text and
/// parsed by the REAL `parse_rosmsg`, the same pattern
/// [`unitree_lowstate_schemas`] uses — and it is exactly the message an unseen
/// robot would ship, which is the path the whole element ladder exists for.
fn survey_plan_schemas() -> Vec<MessageSchema> {
    let mut schemas = all_schemas();
    schemas.push(
        parse_rosmsg(
            "geometry_msgs/PoseStamped[] trajectory\ngeometry_msgs/Pose[] waypoints\n",
            "SurveyPlan",
            Some("acme"),
        )
        .expect("SurveyPlan parses"),
    );
    schemas
}

/// A walker that knows `acme/SurveyPlan` (the builtins-only walker cannot decode
/// it — its hash is unknown).
fn survey_walker() -> FrameWalker {
    let (walker, _warnings) = FrameWalker::new(survey_plan_schemas());
    walker
}

/// One `acme/SurveyPlan` frame. `trajectory` carries STAMPED `PoseStamped`
/// elements (counted framing — each body has its own offset table); `waypoints`
/// carries UNSTAMPED `Pose` elements (recursively fixed ⇒ back-to-back 56-byte
/// sections, no count). Either may be empty.
fn survey_plan_frame(trajectory: &[[f64; 3]], waypoints: &[[f64; 3]]) -> Vec<u8> {
    let (mut resolver, _) = LayoutResolver::new(survey_plan_schemas());
    let l = resolver
        .layout_of("acme/SurveyPlan")
        .expect("SurveyPlan layout");
    assert_eq!(
        l.variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["trajectory", "waypoints"],
        "SurveyPlan declaration order is load-bearing (trajectory is the FIRST candidate)"
    );
    assert_eq!((l.fixed_size, l.offset_table_bytes()), (0, 16));

    let traj_bodies: Vec<Vec<u8>> = trajectory
        .iter()
        .map(|p| pose_stamped_body(*p, [0.0, 0.0, 0.0, 1.0], "map"))
        .collect();
    let traj_blob = counted_blob(&traj_bodies);
    let mut way_blob = Vec::new();
    for p in waypoints {
        way_blob.extend_from_slice(&pose_fixed_section(*p, [0.0, 0.0, 0.0, 1.0]));
    }

    let table = l.offset_table_bytes();
    let mut payload = vec![0u8; table];
    write_offset_entry(&mut payload, 0, 0, table as u32, traj_blob.len() as u32);
    write_offset_entry(
        &mut payload,
        0,
        1,
        (table + traj_blob.len()) as u32,
        way_blob.len() as u32,
    );
    payload.extend_from_slice(&traj_blob);
    payload.extend_from_slice(&way_blob);

    let hash = survey_walker()
        .schema_hash_for("acme/SurveyPlan")
        .expect("SurveyPlan hash");
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: WireHeader::SIZE as u32,
        offset_table_count: l.variable_fields.len() as u32,
        sequence: 0,
        timestamp_ns: 42_000,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

#[test]
fn a_stale_path_memo_cannot_draw_a_polyline_through_an_unordered_array() {
    // THE behavioural pin for the promotion decoupling — the exact regression the
    // memo would otherwise have introduced.
    //
    // Frame 1 populates the STAMPED `trajectory`, so the topic memoizes `Path3D`.
    // Frame 2 leaves `trajectory` empty and populates the UNSTAMPED `waypoints`,
    // so `scan_element_arrays` — which returns the first NON-EMPTY array that
    // yields geometry — is now decided by a different array whose order is NOT
    // semantic. The memo still says `Path3D`.
    //
    // Taking the promotion from that kind (`kind.forces_ordered_elements()`, the
    // earlier call) promotes those points to a polyline and DRAWS A LINE
    // THROUGH AN UNORDERED BAG — inventing structure the message does not carry.
    // Asking the schema NAME does not.
    let walker = survey_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();

    assert_eq!(
        classify_schema("acme/SurveyPlan"),
        None,
        "the vendor type must be UNMAPPED — that is what puts its kind in the memo"
    );

    // Frame 1: stamped trajectory ⇒ Path3D, memoized.
    let traj: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]];
    let busy = survey_plan_frame(&traj, &[]);
    let busy_fv = walker.walk_by_hash(&busy).expect("walk trajectory frame");
    match scan_element_arrays(&busy_fv).parts() {
        Some(p) => {
            assert_eq!(p.field, "trajectory");
            assert!(matches!(p.geometry, ElementGeometry::Path(_)));
        }
        None => panic!("the trajectory frame must yield Path geometry"),
    }
    dispatch_frame(&rec, &walker, "survey", &busy, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        state.cached_archetype("survey"),
        Some(ArchetypeKind::Path3D),
        "the memo is now warm at Path3D — the stale-kind precondition"
    );

    // Frame 2: the SAME topic + SAME schema hash, decided by the UNSTAMPED array.
    let ways: [[f64; 3]; 2] = [[9.0, 8.0, 7.0], [6.0, 5.0, 4.0]];
    let unordered = survey_plan_frame(&[], &ways);
    let un_fv = walker
        .walk_by_hash(&unordered)
        .expect("walk waypoint frame");
    match scan_element_arrays(&un_fv).parts() {
        Some(p) => {
            assert_eq!(
                p.field, "waypoints",
                "a DIFFERENT array decides this frame — the whole premise"
            );
            assert_eq!(
                p.geometry,
                ElementGeometry::Points(vec![[9.0, 8.0, 7.0], [6.0, 5.0, 4.0]]),
                "and it is UNORDERED, against a hand oracle"
            );
        }
        None => panic!("the waypoint frame must yield Points geometry"),
    }

    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "survey", &unordered, &mut state);
    rec.flush_blocking().expect("flush");

    // THE assertion. A Points render is ONE `Points3D` chunk; the promoted path
    // render is TWO (a `LineStrips3D` polyline + its `viz-vertices` child).
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "one Points3D chunk — the memo's stale Path3D must NOT promote an \
         unordered array to a polyline (2 chunks would mean a fabricated order)"
    );
    // The memo itself is unchanged and still stale — the fix is that nothing
    // ORDER-RELATED is taken from it, not that it was invalidated.
    assert_eq!(
        state.cached_archetype("survey"),
        Some(ArchetypeKind::Path3D)
    );
    assert_eq!(state.inference_runs(), 1);
}

#[test]
fn only_a_name_mapped_path_promotes_an_unordered_array_to_a_polyline() {
    // The ordered-polyline promotion is asked of the schema NAME, never
    // of the classified (possibly MEMOIZED) archetype.
    //
    // Pre-memo the two agreed by construction: a SHAPE-inferred `Path3D` only ever
    // arose from elements the same frame's scan had just found stamped, so the
    // promotion was a no-op on that path. A memoized kind can outlive that
    // agreement — on a message declaring several element arrays the winner is
    // whichever is non-empty THIS frame — and a stale `Path3D` would then draw a
    // polyline through a genuinely unordered array, inventing an order the message
    // does not carry. Asking the NAME keeps the pre-memo behaviour exactly.
    //
    // The two predicates must therefore DISAGREE exactly on the unmapped inferred
    // `Path3D` — which is not observable from render output alone, so it is
    // asserted here directly.
    for mapped in ["geometry_msgs/Polygon", "geometry_msgs/PolygonStamped"] {
        assert_eq!(
            classify_schema(mapped),
            Some(ArchetypeKind::Path3D),
            "{mapped} is the reason the promotion exists"
        );
        assert!(
            name_mapped_forces_ordered(mapped),
            "{mapped}: an unstamped ring's order IS semantic — still promoted"
        );
    }
    // Name-mapped but deliberately unordered, and name-mapped-and-already-ordered.
    for never in ["geometry_msgs/PoseArray", "nav_msgs/GridCells"] {
        assert!(
            !name_mapped_forces_ordered(never),
            "{never}: an unordered bag is never connected with a line"
        );
    }
    assert!(name_mapped_forces_ordered("nav_msgs/Path"));

    // THE decoupling: an UNMAPPED schema that INFERS to `Path3D` — the kind whose
    // memo could go stale — never promotes, even though the kind itself says it
    // would. If these two ever agree again, a stale memo can fabricate an order.
    assert_eq!(classify_schema("control_msgs/MotionPrimitive"), None);
    let walker = builtin_walker();
    let frame = motion_primitive_frame(&[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
    let fv = walker.walk_by_hash(&frame).expect("walk MotionPrimitive");
    assert_eq!(infer_archetype_from_shape(&fv), ArchetypeKind::Path3D);
    assert!(
        ArchetypeKind::Path3D.forces_ordered_elements(),
        "the KIND still says a Path3D is ordered — the predicate is unchanged; \
         what changed is that the RENDER no longer asks it"
    );
    assert!(
        !name_mapped_forces_ordered("control_msgs/MotionPrimitive"),
        "but the RENDER promotion is not taken from it — the memo cannot invent order"
    );
    // An unknown vendor type is likewise never promoted.
    assert!(!name_mapped_forces_ordered("acme/WaypointList"));
}

#[test]
fn two_inputs_of_one_unmapped_schema_memoize_independently() {
    // Per-INPUT, not per-schema: two topics carrying the same vendor type each
    // hold their own entry, while the ladder still runs once per topic rather
    // than once per frame.
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, _storage) = memory();
    let frame = motion_primitive_frame(&[[1.0, 2.0, 3.0]]);

    for _ in 0..3 {
        dispatch_frame(&rec, &walker, "arm_left", &frame, &mut state);
        dispatch_frame(&rec, &walker, "arm_right", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");

    assert_eq!(
        state.inference_runs(),
        2,
        "once per INPUT across six frames — not once per frame, not once per schema"
    );
    assert_eq!(
        state.cached_archetype("arm_left"),
        Some(ArchetypeKind::Path3D)
    );
    assert_eq!(
        state.cached_archetype("arm_right"),
        Some(ArchetypeKind::Path3D)
    );
    assert_eq!(state.cached_archetype("arm_other"), None);
}

#[test]
fn a_warmed_topic_that_goes_idle_keeps_its_archetype_instead_of_flipping_to_plots() {
    // The memo's DELIBERATE divergence — the memo answers frames the COLD ladder
    // would have called `ElementsUndecided`.
    //
    // `archetype_for` consults the memo BEFORE stability, so once a Stable
    // element-rung answer is memoized it also answers a later frame whose array is
    // EMPTY. Cold, that frame classifies `Scalars` and plots the message's other
    // numbers; warm, it stays `Path3D` and draws nothing until the array refills.
    //
    // That is the intended behaviour, not an oversight: a topic's layout is
    // resolved ONCE from its first decodable frame, and the NAME-MAPPED half
    // behaves identically — an idle `nav_msgs/Path` is `Path3D` forever and never
    // plots its `header` numbers. Without it an unmapped `/plan`-shaped topic would
    // FLIP between a 3D polyline and a plot view every time the plan cleared, which
    // is the churn `ElementsUndecided` exists to avoid on the way IN (idle→busy),
    // not a promise to reverse on the way OUT.
    //
    // Pinned on the RENDERED OUTPUT, warm vs cold, so the divergence is asserted
    // rather than assumed — and so a future change to it cannot pass silently.
    let walker = builtin_walker();
    let busy = motion_primitive_frame(&[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
    let idle = motion_primitive_frame(&[]);

    // WARM: one state across both frames — the production shape.
    let (warm_rec, warm_storage) = memory();
    let mut warm = SinkState::new();
    dispatch_frame(&warm_rec, &walker, "motion", &busy, &mut warm);
    warm_rec.flush_blocking().expect("flush");
    let after_busy = warm_storage.num_msgs();
    dispatch_frame(&warm_rec, &walker, "motion", &idle, &mut warm);
    warm_rec.flush_blocking().expect("flush");
    let warm_idle_chunks = warm_storage.num_msgs() - after_busy;

    assert_eq!(
        warm.cached_archetype("motion"),
        Some(ArchetypeKind::Path3D),
        "the idle frame did NOT re-open the classification"
    );
    assert_eq!(
        warm.inference_runs(),
        1,
        "and did not re-run the ladder either"
    );
    assert_eq!(
        warm_idle_chunks, 0,
        "an idle array under a warm Path3D memo draws NOTHING — no plots, and no \
         field dump either (an empty element array is not a failure)"
    );

    // COLD: the same idle frame classified from scratch — what the memo overrides.
    let (cold_rec, cold_storage) = memory();
    let mut cold = SinkState::new();
    let cold_base = cold_storage.num_msgs();
    dispatch_frame(&cold_rec, &walker, "motion", &idle, &mut cold);
    cold_rec.flush_blocking().expect("flush");
    let cold_idle_chunks = cold_storage.num_msgs() - cold_base;

    let idle_fv = walker.walk_by_hash(&idle).expect("walk idle");
    assert_eq!(
        infer_archetype_from_shape(&idle_fv),
        ArchetypeKind::Scalars,
        "cold, the same frame is a plot topic"
    );
    // Hand oracle: `MotionPrimitive`'s two top-level numbers (`type`,
    // `blend_radius`) — so cold really does render something the warm path does
    // not, and the divergence is a measured fact rather than an assumption.
    assert_eq!(
        cold_idle_chunks, 2,
        "cold plots `type` + `blend_radius` — exactly what the memo suppresses"
    );
    assert!(
        cold_idle_chunks > warm_idle_chunks,
        "the divergence is REAL and in the documented direction"
    );
}

#[test]
fn the_memo_renders_identically_to_a_cold_classification_on_a_steady_stream() {
    // The firewall, scoped to what it actually covers: a STEADY stream of frames
    // that each carry elements. Warm (one classification) and cold (one per frame)
    // must render the same content, anchored to the hand oracle.
    //
    // It deliberately does NOT cover a stream whose array empties — there the memo
    // and a cold ladder DIVERGE by design, which is pinned separately by
    // `a_warmed_topic_that_goes_idle_keeps_its_archetype_instead_of_flipping_to_plots`.
    let oracle: [[f64; 3]; 3] = [[1.0, 2.0, 3.0], [-4.5, 5.25, 6.125], [7.0, 8.0, 9.0]];
    let frame = motion_primitive_frame(&oracle);
    let walker = builtin_walker();
    const FRAMES: usize = 4;

    let fv = walker.walk_by_hash(&frame).expect("walk MotionPrimitive");
    match scan_element_arrays(&fv).parts().map(|p| p.geometry.clone()) {
        Some(ElementGeometry::Path(v)) => assert_eq!(v, motion_vertex_oracle(&oracle)),
        other => panic!("expected a Path geometry, got {other:?}"),
    }

    // WARM: one `SinkState` across all frames (production) — one inference.
    let (warm_rec, warm_storage) = memory();
    let mut warm = SinkState::new();
    let warm_base = warm_storage.num_msgs();
    for _ in 0..FRAMES {
        dispatch_frame(&warm_rec, &walker, "motion", &frame, &mut warm);
    }
    warm_rec.flush_blocking().expect("flush");
    let warm_chunks = warm_storage.num_msgs() - warm_base;

    // COLD: a fresh `SinkState` per frame — the earlier shape, where every
    // frame re-classified (and so re-scanned the whole element array).
    let (cold_rec, cold_storage) = memory();
    let cold_base = cold_storage.num_msgs();
    let mut cold_runs = 0;
    for _ in 0..FRAMES {
        let mut cold = SinkState::new();
        dispatch_frame(&cold_rec, &walker, "motion", &frame, &mut cold);
        cold_runs += cold.inference_runs();
    }
    cold_rec.flush_blocking().expect("flush");
    let cold_chunks = cold_storage.num_msgs() - cold_base;

    assert_eq!(warm.inference_runs(), 1, "warm: one classification");
    assert_eq!(
        cold_runs, FRAMES as u64,
        "cold: one classification per frame — the un-memoized cost, reproduced"
    );
    assert_eq!(
        warm_chunks, cold_chunks,
        "identical rendered chunk counts either way"
    );
    // Hand oracle, not a self-compare: one LineStrips3D polyline + one Points3D
    // of its vertices, whichever way the archetype was decided. (Same-entity logs
    // COMPACT, so `FRAMES` frames of one topic still land as those 2 chunks.)
    assert_eq!(
        warm_chunks, 2,
        "polyline + vertices, {FRAMES} frames compacted"
    );
}

// ── A Scalars-shaped message that also carries TEXT ──────────────────
//
// The `Scalars` arm logs numeric samples and NOTHING else, and its layout is a
// lone `time_series` view — so a vendor status / response / diagnostic message
// plotted its numeric envelope while its string payload vanished with no
// diagnostic anywhere. These pin the fix END TO END: the classification
// (frame-invariant, decided once), the layout it earns, and the render actually
// logging BOTH halves at the entities those views display.
//
// The schemas are declared here in `.msg` text and parsed by the REAL
// `parse_rosmsg` (crib: `survey_plan_schemas`) — an UNSEEN vendor message is
// exactly the path this ladder exists for, and no built-in carries the trio of
// shapes these arms need to separate.

/// `acme/StatusReport`, the dual-view shape: a numeric envelope + a string
/// payload. Plus the two CONTROLS: a text-free twin, and a stamped twin whose
/// only string is its `header.frame_id` (envelope, not payload).
fn status_report_schemas() -> Vec<MessageSchema> {
    let mut schemas = all_schemas();
    schemas.push(
        parse_rosmsg(
            "float64 voltage\nstring firmware\n",
            "StatusReport",
            Some("acme"),
        )
        .expect("StatusReport parses"),
    );
    schemas.push(
        parse_rosmsg("float64 voltage\n", "Telemetry", Some("acme")).expect("Telemetry parses"),
    );
    schemas.push(
        parse_rosmsg(
            "std_msgs/Header header\nfloat64 voltage\n",
            "StampedTelemetry",
            Some("acme"),
        )
        .expect("StampedTelemetry parses"),
    );
    schemas
}

fn status_report_walker() -> FrameWalker {
    let (walker, _warnings) = FrameWalker::new(status_report_schemas());
    walker
}

fn status_report_layout(qname: &str) -> WireLayout {
    let (mut resolver, _) = LayoutResolver::new(status_report_schemas());
    resolver.layout_of(qname).expect("status report schema")
}

fn acme_frame(qname: &str, payload: Vec<u8>, fixed: usize, table_count: u32) -> Vec<u8> {
    acme_frame_at(qname, payload, fixed, table_count, 42_000)
}

/// [`acme_frame`] with an explicit publisher WIRE timestamp.
fn acme_frame_at(
    qname: &str,
    payload: Vec<u8>,
    fixed: usize,
    table_count: u32,
    timestamp_ns: u64,
) -> Vec<u8> {
    let hash = status_report_walker()
        .schema_hash_for(qname)
        .unwrap_or_else(|| panic!("{qname} hash"));
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: table_count,
        sequence: 0,
        timestamp_ns,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

/// One `acme/StatusReport` frame: `voltage` fixed, `firmware` a variable string.
/// An EMPTY `firmware` is a real frame too — that is the frame-invariance arm.
fn status_report_frame(voltage: f64, firmware: &str) -> Vec<u8> {
    status_report_frame_at(voltage, firmware, 42_000)
}

/// [`status_report_frame`] with an explicit publisher WIRE timestamp — needed by
/// any test feeding a STREAM of frames on one topic (the series rate gate).
fn status_report_frame_at(voltage: f64, firmware: &str, timestamp_ns: u64) -> Vec<u8> {
    let l = status_report_layout("acme/StatusReport");
    assert_eq!(
        (l.fixed_size, l.variable_fields.len()),
        (8, 1),
        "StatusReport layout changed — update this builder"
    );
    let table = l.offset_table_bytes();
    let mut payload = vec![0u8; l.fixed_size + table];
    payload[..8].copy_from_slice(&voltage.to_le_bytes());
    write_offset_entry(
        &mut payload,
        l.fixed_size,
        0,
        (l.fixed_size + table) as u32,
        firmware.len() as u32,
    );
    payload.extend_from_slice(firmware.as_bytes());
    acme_frame_at("acme/StatusReport", payload, l.fixed_size, 1, timestamp_ns)
}

/// One `acme/Telemetry` frame — numbers only, no variable fields.
fn telemetry_frame(voltage: f64) -> Vec<u8> {
    let l = status_report_layout("acme/Telemetry");
    assert_eq!((l.fixed_size, l.variable_fields.len()), (8, 0));
    acme_frame("acme/Telemetry", voltage.to_le_bytes().to_vec(), 8, 0)
}

/// A standalone `std_msgs/Header` sub-frame body (fixed stamp + variable
/// `frame_id`), built through the REAL layout so the walker decodes it.
fn status_report_header_body(frame_id: &str) -> Vec<u8> {
    let l = status_report_layout("std_msgs/Header");
    let table = l.offset_table_bytes();
    let mut body = vec![0u8; l.fixed_size + table];
    write_offset_entry(
        &mut body,
        l.fixed_size,
        0,
        (l.fixed_size + table) as u32,
        frame_id.len() as u32,
    );
    body.extend_from_slice(frame_id.as_bytes());
    body
}

/// One `acme/StampedTelemetry` frame — a REAL `std_msgs/Header` (whose
/// `frame_id` is a string) beside a number. The envelope control.
fn stamped_telemetry_frame(voltage: f64, frame_id: &str) -> Vec<u8> {
    let l = status_report_layout("acme/StampedTelemetry");
    assert_eq!(
        l.variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["header"],
        "StampedTelemetry layout changed — update this builder"
    );
    let header_body = status_report_header_body(frame_id);
    let table = l.offset_table_bytes();
    let mut payload = vec![0u8; l.fixed_size + table];
    let v_off = l
        .fixed_fields
        .iter()
        .find(|f| f.name == "voltage")
        .expect("voltage is a fixed field")
        .offset;
    payload[v_off..v_off + 8].copy_from_slice(&voltage.to_le_bytes());
    write_offset_entry(
        &mut payload,
        l.fixed_size,
        0,
        (l.fixed_size + table) as u32,
        header_body.len() as u32,
    );
    payload.extend_from_slice(&header_body);
    acme_frame("acme/StampedTelemetry", payload, l.fixed_size, 1)
}

/// `build_joint_state` with a POPULATED `string[] name` — the shape the
/// `string[]` narrowing is really about. Names ride the canonical `string[]`
/// framing (`u32 count` + per element `u32 len` + UTF-8), the same encoding
/// [`counted_blob`] builds for message elements.
fn build_joint_state_with_names(
    names: &[&str],
    position: &[f64],
    velocity: &[f64],
    effort: &[f64],
) -> Vec<u8> {
    let layout = layout_of("sensor_msgs/JointState");
    assert_eq!(
        layout
            .variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["header", "name", "position", "velocity", "effort"],
        "JointState variable-field order changed — update this builder"
    );
    let fixed = layout.fixed_size;
    let table = layout.offset_table_bytes();
    let le = |vals: &[f64]| -> Vec<u8> { vals.iter().flat_map(|v| v.to_le_bytes()).collect() };
    let name_bodies: Vec<Vec<u8>> = names.iter().map(|n| n.as_bytes().to_vec()).collect();
    let name_blob = counted_blob(&name_bodies);
    let (pos, vel, eff) = (le(position), le(velocity), le(effort));

    let mut payload = vec![0u8; fixed + table];
    let mut cursor = (fixed + table) as u32;
    write_offset_entry(&mut payload, fixed, 0, 0, 0); // header: empty
    for (slot, bytes) in [(1usize, &name_blob), (2, &pos), (3, &vel), (4, &eff)] {
        write_offset_entry(&mut payload, fixed, slot, cursor, bytes.len() as u32);
        cursor += bytes.len() as u32;
    }
    payload.extend_from_slice(&name_blob);
    payload.extend_from_slice(&pos);
    payload.extend_from_slice(&vel);
    payload.extend_from_slice(&eff);

    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: <JointState as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: 5,
        sequence: 0,
        timestamp_ns: 9_000,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

/// Every `(entity path, component descriptor)` pair the sink emitted, read back
/// off the REAL rerun store — so these arms observe what a VIEWER would, not
/// merely how many chunks were produced.
fn logged_components(storage: &rerun::sink::MemorySinkStorage) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for msg in storage.take() {
        let rerun::log::LogMsg::ArrowMsg(_, arrow_msg) = msg else {
            continue;
        };
        let chunk = rerun::log::Chunk::from_arrow_msg(&arrow_msg).expect("decode chunk");
        let entity = chunk
            .entity_path()
            .to_string()
            .trim_start_matches('/')
            .to_string();
        for descr in chunk.components().keys() {
            out.push((entity.clone(), descr.as_str().to_string()));
        }
    }
    out
}

#[test]
fn a_numeric_message_carrying_text_classifies_as_the_dual_view_twin() {
    let walker = status_report_walker();
    let frame = status_report_frame(48.0, "fw-1.2.3");
    let fv = walker.walk_by_hash(&frame).expect("StatusReport decodes");
    // The CLASSIFICATION (the decision the layout is derived from).
    assert_eq!(
        infer_archetype_from_shape(&fv),
        ArchetypeKind::ScalarsWithText,
        "numbers AND text ⇒ the twin that renders both"
    );
    // The LAYOUT it earns — the whole point: without the text_document view the
    // dump would be logged into an entity no view displays.
    assert_eq!(
        views_for_archetype(ArchetypeKind::ScalarsWithText),
        &[ViewKind::TimeSeries, ViewKind::TextDocument]
    );
    assert_eq!(
        archetype_components(ArchetypeKind::ScalarsWithText),
        &["Scalars", "TextDocument"]
    );

    // FRAME-INVARIANCE: the same schema whose string is EMPTY this frame must
    // classify identically, or a topic whose first frame happens to carry an
    // empty payload freezes into the plots-only twin forever (the
    // frame-dependent-layout failure mode).
    let empty_frame = status_report_frame(48.0, "");
    let fv_empty = walker.walk_by_hash(&empty_frame).expect("decodes");
    assert_eq!(
        infer_archetype_from_shape(&fv_empty),
        ArchetypeKind::ScalarsWithText,
        "an empty string is still a DECLARED string"
    );
}

/// A REAL `sensor_msgs/JointState` with a POPULATED `string[] name` — the
/// classification-level pin for the `string[]` narrowing.
///
/// `declares_text_fields` declines a `string[]` because its element types are
/// invisible when the array is empty, so counting it would make a once-resolved
/// layout depend on which frame arrived first. That narrowing is unit-tested on
/// hand-built values; THIS arm proves the consequence on the most-published ROS
/// topic there is, decoded by the REAL walker from REAL wire bytes: JointState
/// stays `Scalars` (plots only, no status pane) even when its names are present.
///
/// Without it the only JointState-shaped fixture in the tree was
/// `sink.rs`'s `empty_bank`, which carries a SCALAR `Str` and therefore
/// classifies as the twin — so the tree asserted the opposite of what a real
/// JointState does, and the comment there said so.
#[test]
fn a_real_joint_state_frame_stays_plots_only() {
    let walker = builtin_walker();
    let frame =
        build_joint_state_with_names(&["shoulder", "elbow"], &[0.1, 0.2], &[-1.0, 1.0], &[]);
    let fv = walker.walk_by_hash(&frame).expect("walk JointState");

    // Precondition: the names really ARE decoded (else the arm is vacuous — an
    // empty array would take the narrowing for the trivial reason).
    let names = fv
        .fields
        .iter()
        .find(|f| f.name == "name")
        .map(|f| &f.value)
        .expect("JointState declares `name`");
    match names {
        FrameValueKind::NestedArray { elements, .. } => assert_eq!(
            elements.len(),
            2,
            "the fixture's two names must decode, or this test proves nothing"
        ),
        other => panic!("expected a decoded string[], got {other:?}"),
    }

    assert_eq!(
        infer_archetype_from_shape(&fv),
        ArchetypeKind::Scalars,
        "a string[] does not elect the text twin — its element types vanish when \
         the array is empty, so a once-resolved layout must not depend on it"
    );
    assert_eq!(
        views_for_archetype(ArchetypeKind::Scalars),
        &[ViewKind::TimeSeries],
        "…so the most-published ROS topic there is keeps its single plot view"
    );
}

#[test]
fn a_purely_numeric_message_stays_plots_only() {
    // THE CONTROL. Without it, "the twin renders text" would be satisfied by a
    // classifier that gave EVERY numeric topic a status pane.
    let walker = status_report_walker();
    let numeric = telemetry_frame(48.0);
    let fv = walker.walk_by_hash(&numeric).expect("Telemetry decodes");
    assert_eq!(infer_archetype_from_shape(&fv), ArchetypeKind::Scalars);
    assert_eq!(
        views_for_archetype(ArchetypeKind::Scalars),
        &[ViewKind::TimeSeries],
        "a pure-numeric topic must not be handed a permanently empty text pane"
    );

    // …and a STAMPED numeric message is the sharper control: it really does
    // carry a string (`header.frame_id`), and counting it would hand a text pane
    // to every `*Stamped` message in ROS.
    let stamped_frame = stamped_telemetry_frame(48.0, "base_link");
    let stamped = walker
        .walk_by_hash(&stamped_frame)
        .expect("StampedTelemetry decodes");
    assert_eq!(
        infer_archetype_from_shape(&stamped),
        ArchetypeKind::Scalars,
        "a header's frame_id is envelope — it must not elect the text twin"
    );
}

#[test]
fn the_text_twin_renders_both_the_plots_and_the_dump_at_the_entities_its_views_display() {
    // THE HEADLINE, over the production `dispatch_frame`: the plots land under
    // the topic entity (where the time_series view reads them) AND the field
    // dump lands AT the topic entity (where the text_document view reads it).
    let walker = status_report_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    dispatch_frame(
        &rec,
        &walker,
        "status",
        &status_report_frame(48.0, "fw-1.2.3"),
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let logged = logged_components(&storage);
    // Hand oracle: the numeric field plots at `world/status/voltage`, and the
    // structured dump is a TextDocument at `world/status`.
    assert!(
        logged
            .iter()
            .any(|(e, c)| e == "world/status/voltage" && c.starts_with("Scalars:")),
        "the numeric half must plot under the topic entity: {logged:?}"
    );
    assert!(
        logged
            .iter()
            .any(|(e, c)| e == "world/status" && c.starts_with("TextDocument:")),
        "the TEXT half must render as a TextDocument at the topic entity — this is \
         the whole issue: {logged:?}"
    );
    // The dump is NOT a degradation here — the schema resolved fine.
    assert!(!state.took_anyvalues_fallback("acme/StatusReport"));
}

#[test]
fn a_purely_numeric_message_renders_no_text_document() {
    // The render-side CONTROL for the arm above: a plots-only topic must emit no
    // TextDocument at all (an empty pane is the cost this control prevents).
    let walker = status_report_walker();
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    dispatch_frame(&rec, &walker, "telem", &telemetry_frame(48.0), &mut state);
    rec.flush_blocking().expect("flush");

    let logged = logged_components(&storage);
    assert!(
        logged
            .iter()
            .any(|(e, c)| e == "world/telem/voltage" && c.starts_with("Scalars:")),
        "the plots still render: {logged:?}"
    );
    assert!(
        !logged.iter().any(|(_, c)| c.starts_with("TextDocument:")),
        "a pure-numeric topic must log NO TextDocument: {logged:?}"
    );
}

// The STATELESS text-twin parity test that stood here is GONE with
// `archetype::log_frame_value`. Every property it asserted is pinned on the
// STATEFUL path in this file already, on the same two frames: the numeric+text
// twin's `world/status/voltage` Scalars and `world/status` TextDocument by
// `the_text_twin_renders_both_the_plots_and_the_dump_at_the_entities_its_views_display`,
// and the pure-numeric control's "plots but NO TextDocument" by
// `a_purely_numeric_message_renders_no_text_document` immediately above. What is
// gone with it is the `stateless == stateful` comparison, which had nothing left
// to compare.
#[test]
fn the_text_twin_renders_deterministically() {
    // Principle #7: two runs of the same frame produce the same (entity,
    // component) set — and it carries the hand-oracle pair, so this is not a
    // bare self-compare.
    let walker = status_report_walker();
    let frame = status_report_frame(48.0, "fw-1.2.3");
    let run = || {
        let mut state = SinkState::new();
        let (rec, storage) = memory();
        dispatch_frame(&rec, &walker, "status", &frame, &mut state);
        rec.flush_blocking().expect("flush");
        let mut pairs = logged_components(&storage);
        pairs.sort();
        pairs.dedup();
        pairs
    };
    let a = run();
    assert_eq!(a, run(), "two runs are identical");
    assert!(
        a.iter()
            .any(|(e, c)| e == "world/status" && c.starts_with("TextDocument:"))
            && a.iter()
                .any(|(e, c)| e == "world/status/voltage" && c.starts_with("Scalars:")),
        "…and both halves are present in that stable set: {a:?}"
    );
}

// ---- The per-topic REPRESENTATION override, through the PRODUCTION
// dispatch ----
//
// The pure decision table lives in `cerulion_viz::representation`'s oracle
// vectors. These arms are the other half of the same claim: that the choice
// reaches the RENDER — an override the sink resolved and then ignored would pass
// every one of those vectors.
//
// Every oracle here is a recording-chunk delta against `memory_always_flush()`,
// so it reads what was LOGGED rather than what a counter believes: a `Twist` is
// name-mapped to six `Scalars` paths, and the structured dump is exactly one
// more `TextDocument` chunk at the topic entity.

/// Dispatch `count` Twist frames on one input under `representation`, spaced
/// `period_ns` of WIRE time apart, and report the recording-chunk delta.
///
/// A fresh `SinkState` per call, so nothing carries between arms.
fn twist_under(representation: Representation, count: u64, period_ns: u64) -> (usize, SinkState) {
    let walker = builtin_walker();
    let mut state = SinkState::new();
    state.set_representation("twist", representation);
    let (rec, storage) = memory_always_flush();
    let base = storage.num_msgs();
    for i in 0..count {
        let frame = build_twist_at([i as f64, 0.0, 0.0], [0.0, 0.0, 0.0], i * period_ns);
        dispatch_frame(&rec, &walker, "twist", &frame, &mut state);
    }
    rec.flush_blocking().expect("flush");
    (storage.num_msgs() - base, state)
}

/// **The reported case, both directions in one body.** A plot topic gains the
/// message dump on request, and can be reduced to the dump alone.
///
/// `/cmd_vel`-shaped `Twist` is exactly the IMU-class topic the ask names: the
/// ladder elects plain `Scalars` for it, so without the message dump there is no way to read the
/// message. Each frame is spaced a full forced-dump interval apart so the meter
/// admits every document and the counts stay hand-computable.
///
/// Oracles, per frame: `Auto` = 6 scalars (today's rendering, unchanged);
/// `Both` = 6 scalars + 1 document; `Text` = 1 document and NO scalars; `Visual`
/// = `Auto` (there is no text half on a plain `Scalars` topic to strip).
#[test]
fn a_plot_topic_renders_the_message_dump_on_request() {
    const FRAMES: u64 = 3;
    let step = FORCED_DUMP_MIN_INTERVAL_NS;

    let (auto, auto_state) = twist_under(Representation::Auto, FRAMES, step);
    assert_eq!(
        auto,
        6 * FRAMES as usize,
        "Auto is today's rendering: six scalar paths per frame, no document"
    );
    assert_eq!(
        auto_state.forced_dump_renders_withheld(),
        0,
        "an un-overridden topic never touches the forced-dump gate"
    );

    let (both, _) = twist_under(Representation::Both, FRAMES, step);
    assert_eq!(
        both,
        7 * FRAMES as usize,
        "Both adds EXACTLY one TextDocument per frame beside the six scalars"
    );

    let (text, text_state) = twist_under(Representation::Text, FRAMES, step);
    assert_eq!(
        text, FRAMES as usize,
        "Text is the document alone — the six scalar paths are suppressed"
    );
    assert_eq!(
        text_state.plot_frames_decimated(),
        0,
        "a suppressed visual half never consults the series gate at all"
    );

    let (visual, _) = twist_under(Representation::Visual, FRAMES, step);
    assert_eq!(
        visual, auto,
        "Visual on a kind with no separable text half is a no-op, not a blank pane"
    );
}

/// The other direction of the same lever: a topic the election ALREADY
/// gave the dump can have it taken away, and asking for `Both` there does not
/// log the document twice.
///
/// The `status_report_walker` status topic is a real `ScalarsWithText` election (it
/// declares a string), so `Auto` renders one series + one document per frame.
/// Stamps advance a full interval so both meters admit every frame.
#[test]
fn a_curated_text_topic_can_drop_its_dump_and_never_doubles_it() {
    const FRAMES: u64 = 3;

    fn status_under(representation: Representation, frames: u64) -> usize {
        let walker = status_report_walker();
        let mut state = SinkState::new();
        state.set_representation("status", representation);
        let (rec, storage) = memory_always_flush();
        let base = storage.num_msgs();
        for i in 0..frames {
            let frame = status_report_frame_at(
                48.0 + i as f64,
                &format!("fw-{i}"),
                i * FORCED_DUMP_MIN_INTERVAL_NS,
            );
            dispatch_frame(&rec, &walker, "status", &frame, &mut state);
        }
        rec.flush_blocking().expect("flush");
        storage.num_msgs() - base
    }

    let auto = status_under(Representation::Auto, FRAMES);
    assert_eq!(
        auto,
        2 * FRAMES as usize,
        "the dump election renders one series + one document per frame"
    );
    assert_eq!(
        status_under(Representation::Visual, FRAMES),
        FRAMES as usize,
        "Visual strips the text half: the series alone"
    );
    assert_eq!(
        status_under(Representation::Text, FRAMES),
        FRAMES as usize,
        "Text keeps the document alone"
    );
    assert_eq!(
        status_under(Representation::Both, FRAMES),
        auto,
        "Both on a kind whose own arm dumps must add NOTHING — never a second \
         document at the same entity"
    );
}

/// `Both` on a topic whose visual half is not a plot at all still yields the
/// dump — nothing about "show me the message as text" is specific to numeric
/// telemetry, and the walker decodes a cloud or a JPEG envelope as readily.
///
/// The camera arm additionally carries the PRODUCTION-path kill for the
/// predicate the overlay resolves against. `ArchetypeKind::renders_text_document`
/// is the natural mistake — it is what the view table is pinned on — but it also
/// covers every kind that merely CAN degrade to a dump, and `Image` is one. Under
/// that predicate a camera topic reads as already-dumping and `Both` renders the
/// JPEG and NO document.
///
/// **KILL ATTRIBUTION:** swapping `renders_own_dump`
/// for `renders_text_document` fails the camera assertion here and
/// `representation::tests::the_decision_table_is_exactly_what_each_arm_promises`
/// (`Image under Both`, `(Some(Image), false)` where `(Some(Image), true)` is
/// required). It does NOT fail the cloud assertions — `Points3D` cannot degrade,
/// so both predicates agree on it — and it does not fail
/// `both_overlays_exactly_the_kinds_that_do_not_dump_themselves`, which reads
/// `renders_own_dump` on both sides of its equality and therefore pins the
/// RELATIONSHIP rather than the predicate's content.
#[test]
fn both_reaches_a_topic_whose_visual_half_is_not_a_plot() {
    fn under(input: &str, frame: &[u8], representation: Representation) -> usize {
        let walker = builtin_walker();
        let mut state = SinkState::new();
        state.set_representation(input, representation);
        let (rec, storage) = memory_always_flush();
        let base = storage.num_msgs();
        dispatch_frame(&rec, &walker, input, frame, &mut state);
        rec.flush_blocking().expect("flush");
        storage.num_msgs() - base
    }

    // A cloud: a visual half that is not a plot, and a kind that never degrades.
    let cloud = build_cloud_frame(&[[1.0, 2.0, 3.0, 0.5], [4.0, 5.0, 6.0, 0.25]], 1_000);
    let auto = under("cloud", &cloud, Representation::Auto);
    assert!(auto > 0, "the cloud renders something under Auto");
    assert_eq!(
        under("cloud", &cloud, Representation::Both),
        auto + 1,
        "Both adds exactly one TextDocument to whatever the cloud logs"
    );
    assert_eq!(
        under("cloud", &cloud, Representation::Text),
        1,
        "Text on a cloud is the document alone — no points, no frame"
    );

    // A camera: a kind that CAN degrade to a dump, so the wrong predicate reads
    // it as already-dumping.
    let jpeg = build_jpeg_frame(&[0xFF, 0xD8, 0xFF, 0xD9], 1_000);
    let auto = under("image", &jpeg, Representation::Auto);
    assert!(auto > 0, "the JPEG renders something under Auto");
    assert_eq!(
        under("image", &jpeg, Representation::Both),
        auto + 1,
        "Both adds a document to a camera topic too — a kind that CAN degrade to \
         a dump has not dumped THIS frame"
    );
}

/// A forced dump is METERED on the publisher's wire clock, so asking for text on
/// a firehose cannot re-open the ~1 MB/s of markdown the metering change metered the
/// automatic election down from.
///
/// Driven at a quarter of the interval, so the hand oracle is "one document
/// every fourth frame": frames 0, 4, 8, … render and the rest are withheld —
/// counts expressed as functions of the SHIPPED constant, never a literal.
#[test]
fn a_forced_dump_is_metered_on_the_wire_clock() {
    let step = FORCED_DUMP_MIN_INTERVAL_NS / 4;
    const FRAMES: u64 = 12;
    let (logged, state) = twist_under(Representation::Text, FRAMES, step);
    assert_eq!(logged, 3, "frames 0, 4 and 8 render; frames 9..11 do not");
    assert_eq!(
        state.forced_dump_renders_withheld(),
        FRAMES - 3,
        "every refused frame is accounted, not silently dropped"
    );

    // A publisher whose clock has STOPPED still refreshes, at the floor — the
    // same anti-freeze requirement, on this gate.
    let frames = 1 + FORCED_DUMP_FRAME_FLOOR + 1;
    let (stalled, _) = twist_under(Representation::Text, frames, 0);
    assert_eq!(
        stalled, 2,
        "frame 1 plus one floor refresh, despite an unmoving clock"
    );
}

/// `Auto` set EXPLICITLY is indistinguishable from never having been set — the
/// additivity promise, and what makes a topic returned to automagic clean rather
/// than merely quiet.
///
/// Also the determinism arm: the same stimulus twice yields the same log.
#[test]
fn auto_is_indistinguishable_from_no_override_and_is_deterministic() {
    let step = FORCED_DUMP_MIN_INTERVAL_NS;
    let walker = builtin_walker();

    // Never set.
    let mut untouched = SinkState::new();
    let (rec, storage) = memory_always_flush();
    let base = storage.num_msgs();
    for i in 0..3u64 {
        let frame = build_twist_at([i as f64, 0.0, 0.0], [0.0, 0.0, 0.0], i * step);
        dispatch_frame(&rec, &walker, "twist", &frame, &mut untouched);
    }
    rec.flush_blocking().expect("flush");
    let untouched_delta = storage.num_msgs() - base;

    // Set to Both and then RETURNED to Auto — the operator changing their mind.
    let walker = builtin_walker();
    let mut returned = SinkState::new();
    returned.set_representation("twist", Representation::Both);
    returned.set_representation("twist", Representation::Auto);
    let (rec, storage) = memory_always_flush();
    let base = storage.num_msgs();
    for i in 0..3u64 {
        let frame = build_twist_at([i as f64, 0.0, 0.0], [0.0, 0.0, 0.0], i * step);
        dispatch_frame(&rec, &walker, "twist", &frame, &mut returned);
    }
    rec.flush_blocking().expect("flush");

    assert_eq!(storage.num_msgs() - base, untouched_delta);
    assert_eq!(returned.representation_for("twist"), Representation::Auto);
    assert_eq!(
        returned.forced_dump_renders_withheld(),
        0,
        "a topic returned to Auto carries no residue from the choice it left"
    );

    // Determinism: two runs of the overridden stimulus log the same counts.
    let (a, _) = twist_under(Representation::Both, 5, step);
    let (b, _) = twist_under(Representation::Both, 5, step);
    assert_eq!(a, b);
    assert_eq!(a, 7 * 5);
}

/// An override survives the header-only pre-walk drop.
///
/// That fast path drops a fully-gated plot frame on the HEADER alone, keyed on
/// the SERIES verdict — the right answer for a topic rendering plots and nothing
/// else, and the wrong one the moment a document rides the same frames. Under
/// `Both` on a stalled clock the series gate refuses every frame after the
/// first, so without the exemption every later frame is dropped whole and the
/// document freezes at frame 1 for the life of the run — the exact freeze
/// the anti-freeze requirement forbids, re-entered through the other gate.
///
/// **The discriminating shape is `Both`, not `Text` — a check keyed on `Text` alone
/// would miss it.** The fast path can only fire
/// once the series gate knows the topic's series count, and that count is
/// recorded when the series RENDER — so on a `Text` topic, whose visual half
/// never renders, the gate admits everything and the drop is unreachable no
/// matter what the guard says. `Both` renders the series, so the count is real
/// from frame 1 and the drop is live from frame 2.
#[test]
fn an_overridden_topic_is_not_dropped_before_the_walk() {
    // Long enough that the drop would have eaten every frame after the first.
    let frames = 1 + FORCED_DUMP_FRAME_FLOOR + 1;

    let (logged, state) = twist_under(Representation::Both, frames, 0);
    assert_eq!(
        logged,
        6 + 1 + 1,
        "frame 1's six series and its document, then the floor's refresh — a \
         dropped frame renders neither, so a fast-dropping run logs 7"
    );
    assert_eq!(
        state.forced_dump_renders_withheld(),
        FORCED_DUMP_FRAME_FLOOR,
        "every frame after the first REACHED the forced-dump gate and was \
         accounted there — a run that dropped them pre-walk withholds 0"
    );

    // ANTI-TAUTOLOGY: the same drive with NO override takes the fast path — the
    // series gate runs and decimates, exactly as before the forced-representation override landed.
    let (_, auto_state) = twist_under(Representation::Auto, frames, 0);
    assert_eq!(
        auto_state.plot_frames_decimated(),
        frames - 1,
        "an un-overridden stalled-clock plot topic still decimates as before"
    );

    // And the `Text` shape, stated for what it actually is: no series render
    // means no series count, so the fast path has nothing to gate on and the
    // documents arrive on the floor's cadence.
    let (text_logged, text_state) = twist_under(Representation::Text, frames, 0);
    assert_eq!(text_logged, 2, "frame 1's document plus one floor refresh");
    assert_eq!(
        text_state.plot_frames_decimated(),
        0,
        "a topic rendering no series never consults the series gate"
    );
}

/// **The pre-walk dump VERDICT term, on the only shape that can see
/// it.** `Both` on a `ScalarsWithText` topic is plan-identical to `Auto`, so it
/// keeps the fast path — and its dump verdict must still come from
/// `admit_field_dump`, or the dump metering goes inert on exactly the topic
/// class it was written for. The sibling arms all drive a plain `Scalars`
/// fixture, where `plan.visual == Some(ScalarsWithText)` is dead.
///
/// Driven on a stalled clock so the series gate refuses everything after frame 1:
/// the dump then rides its own floor, which is what the gate being CONSULTED
/// looks like from outside.
#[test]
fn both_on_a_curated_text_topic_still_meters_its_own_dump() {
    fn status_under(representation: Representation, frames: u64) -> (usize, SinkState) {
        let walker = status_report_walker();
        let mut state = SinkState::new();
        state.set_representation("status", representation);
        let (rec, storage) = memory_always_flush();
        let base = storage.num_msgs();
        for i in 0..frames {
            let frame = status_report_frame_at(48.0 + i as f64, &format!("fw-{i}"), 9_000);
            dispatch_frame(&rec, &walker, "status", &frame, &mut state);
        }
        rec.flush_blocking().expect("flush");
        (storage.num_msgs() - base, state)
    }
    let frames = 1 + 2 * (DUMP_REFRESH_FRAME_FLOOR + 1);

    let (auto_logged, auto) = status_under(Representation::Auto, frames);
    assert!(
        auto.dump_renders_withheld() > 0,
        "the dump-metering gate must be live for this comparison to mean anything"
    );

    // THE PIN: `Both` is plan-identical to `Auto` here, so it must be
    // observationally identical too — same documents, same withheld count. A
    // verdict term that skipped the gate would dump on EVERY frame instead.
    let (both_logged, both) = status_under(Representation::Both, frames);
    assert_eq!(
        both_logged, auto_logged,
        "Both on ScalarsWithText renders as Auto"
    );
    assert_eq!(both.dump_renders_withheld(), auto.dump_renders_withheld());
    assert_eq!(
        both.pre_walk_drops(),
        auto.pre_walk_drops(),
        "…and keeps the fast path, since no overlay rides these frames"
    );
}

/// **The exemption is keyed on the PLAN, not on the map.** `Visual`
/// puts no document on the stream, so the series verdict alone is the right drop
/// criterion and the header-only fast path must stay ON.
///
/// This is not a nicety: `Visual` on a `ScalarsWithText` topic is one of the
/// override's two headline gestures, and on the ~500 Hz `/lowstate` it takes the dump away.
/// A blanket `contains_key` exemption INVERTS that trade — asking for strictly
/// less rendering surrenders a drop that refuses ~98 % of frames on their header,
/// so it buys ~15-64× more classification work on the one topic class a
/// user will click it on.
///
/// The oracle is `pre_walk_drops()`, not a timing: a refused frame renders nothing
/// either way, so the ONLY difference an exemption makes is whether the frame was
/// decoded and classified first. A CPU measurement would answer that and fail OPEN
/// on a loaded runner (the load-sensitive class); this counts the decision itself.
#[test]
fn visual_keeps_the_pre_walk_fast_path_while_text_and_both_exempt_it() {
    let frames = 1 + FORCED_DUMP_FRAME_FLOOR + 1;

    // The baseline: an un-overridden stalled-clock plot topic drops on the header
    // for every frame after the first.
    let (_, auto) = twist_under(Representation::Auto, frames, 0);
    let auto_drops = auto.pre_walk_drops();
    assert!(
        auto_drops > 0,
        "the fast path must be live at all for this test to mean anything"
    );

    // THE PIN: `Visual` drops exactly as `Auto` does. A blanket override
    // exemption yields 0 here.
    let (_, visual) = twist_under(Representation::Visual, frames, 0);
    assert_eq!(
        visual.pre_walk_drops(),
        auto_drops,
        "Visual carries no dump, so it must keep the header-only drop"
    );

    // …and the two arms that DO put a document on refused frames keep the
    // exemption, or the stalled-clock freeze comes back.
    for choice in [Representation::Text, Representation::Both] {
        let (_, state) = twist_under(choice, frames, 0);
        assert_eq!(
            state.pre_walk_drops(),
            0,
            "{choice:?} owes a document on series-refused frames, so it must be exempt"
        );
    }
}

// ────────────────────────────────────────────────────────────────────────────
// The render arms whose ONLY witness used to be the stateless `log_frame_value`
// path.
//
// That path carried no `SinkState`, had no production caller, and was a SECOND
// ladder through `classify_frame` maintained only so tests could avoid building
// a wire frame. It is retired; the properties it pinned are not. Each arm below
// drives a REAL wire frame through the production `dispatch_frame` against the
// SAME hand oracle the stateless arm asserted — so the render arm is now pinned
// where production actually reaches it.
//
// The stateless arms that were BYTE-FOR-BYTE subsumed by an existing arm in this
// file were deleted rather than duplicated; each deletion names its survivor.
// ────────────────────────────────────────────────────────────────────────────

/// A `sensor_msgs/Imu` wire frame: fixed 296 B (quaternion + 3 covariance
/// matrices + 2 vectors), then entry[0] `header` left empty. Crib:
/// `coordinate_frame_test.rs::build_imu_frame`, which writes only the
/// orientation — this one also writes the two vectors, because the oracle below
/// counts the SCALAR half.
fn build_imu_frame(
    orientation: [f64; 4],
    angular_velocity: [f64; 3],
    linear_acceleration: [f64; 3],
    timestamp_ns: u64,
) -> Vec<u8> {
    let l = layout_of("sensor_msgs/Imu");
    assert_eq!(
        (l.fixed_size, l.offset_table_bytes()),
        (296, 8),
        "Imu shape drifted — update this builder"
    );
    let mut payload = vec![0u8; l.fixed_size];
    let put = |buf: &mut [u8], base: usize, vals: &[f64]| {
        for (i, v) in vals.iter().enumerate() {
            buf[base + i * 8..base + i * 8 + 8].copy_from_slice(&v.to_le_bytes());
        }
    };
    put(
        &mut payload,
        field_offset("sensor_msgs/Imu", "orientation"),
        &orientation,
    );
    put(
        &mut payload,
        field_offset("sensor_msgs/Imu", "angular_velocity"),
        &angular_velocity,
    );
    put(
        &mut payload,
        field_offset("sensor_msgs/Imu", "linear_acceleration"),
        &linear_acceleration,
    );
    // entry[0] = `header`, left (0, 0) — the empty-nested producer idiom.
    payload.extend_from_slice(&[0u8; 8]);
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: <native_ros2_messages::sensor_msgs::Imu as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + l.fixed_size) as u32,
        offset_table_count: 1,
        sequence: 0,
        timestamp_ns,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

/// The `sensor_msgs/Imu` render arm: ONE attitude `Transform3D` at the entity +
/// SIX scalar plots (accel xyz then gyro xyz), each at its own child entity.
///
/// No `dispatch_frame` arm covered this before — `coordinate_frame_test.rs`
/// drives an Imu frame but asserts parent-frame POSING, and its builder leaves
/// accel/gyro zeroed, so the scalar half was witnessed only by the retired
/// stateless path. A regression that drops the plots records 1 chunk; one that
/// drops the transform records 6.
#[test]
fn imu_frame_renders_one_rotation_transform_and_six_scalars() {
    let walker = builtin_walker();
    let frame = build_imu_frame(
        [0.0, 0.0, 0.0, 1.0],
        [0.1, 0.2, 0.3],
        [1.0, 2.0, 3.0],
        42_000,
    );

    // Content seam first, so the chunk delta below is meaningful (the builder's
    // offsets are proven, never assumed).
    let fv = walker.walk_by_hash(&frame).expect("walk Imu");
    assert_eq!(fv.schema_name, "sensor_msgs/Imu");
    assert_eq!(
        cerulion_viz::archetype::imu_scalars(&fv),
        vec![
            ("linear_acceleration/x".to_string(), 1.0),
            ("linear_acceleration/y".to_string(), 2.0),
            ("linear_acceleration/z".to_string(), 3.0),
            ("angular_velocity/x".to_string(), 0.1),
            ("angular_velocity/y".to_string(), 0.2),
            ("angular_velocity/z".to_string(), 0.3),
        ],
        "the six series, each carrying its own hand value"
    );
    assert_eq!(
        rotation_only_of(&fv),
        Some([0.0, 0.0, 0.0, 1.0]),
        "the attitude quaternion the Transform3D is built from"
    );

    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "imu", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        7,
        "1 rotation Transform3D + 6 scalar plots (accel xyz + gyro xyz)"
    );
    assert!(!state.took_anyvalues_fallback("sensor_msgs/Imu"));
}

/// A `std_msgs/String` wire frame: no fixed section, one variable field `data`.
fn build_string_frame(text: &str, timestamp_ns: u64) -> Vec<u8> {
    let l = layout_of("std_msgs/String");
    assert_eq!(
        (l.fixed_size, l.offset_table_bytes()),
        (0, 8),
        "std_msgs/String shape drifted — update this builder"
    );
    let table = l.offset_table_bytes();
    let mut payload = vec![0u8; table];
    write_offset_entry(&mut payload, 0, 0, table as u32, text.len() as u32);
    payload.extend_from_slice(text.as_bytes());
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: <native_ros2_messages::std_msgs::String as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: WireHeader::SIZE as u32,
        offset_table_count: 1,
        sequence: 0,
        timestamp_ns,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

/// The TextLog mirror, at the production seam: an inferred single-string frame logs a
/// scrolling `TextLog` AT the entity **and** a rolling-latest `TextDocument`
/// mirror at `<entity>/text` — TWO chunks, at two named entities.
///
/// Before that fix this logged ONE chunk (TextLog only) and the mapped view rendered
/// empty. Nothing drove `sink.rs`'s `ArchetypeKind::TextLog` arm through
/// `dispatch_frame` before: the TextLog rows elsewhere in the tree are
/// blueprint/layout tables that never dispatch a frame.
#[test]
fn a_single_string_frame_renders_a_textlog_and_its_text_document_mirror() {
    let walker = builtin_walker();
    let frame = build_string_frame("all systems nominal", 42_000);

    let fv = walker.walk_by_hash(&frame).expect("walk String");
    assert_eq!(
        classify_schema("std_msgs/String"),
        None,
        "the premise: it is UNMAPPED, so the SHAPE ladder must reach TextLog"
    );
    assert_eq!(infer_archetype_from_shape(&fv), ArchetypeKind::TextLog);

    let mut state = SinkState::new();
    let (rec, storage) = memory_always_flush();
    dispatch_frame(&rec, &walker, "status", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    let logged = logged_components(&storage);
    assert!(
        logged
            .iter()
            .any(|(e, c)| e == "world/status" && c.starts_with("TextLog:")),
        "the scrolling log line lands AT the entity: {logged:?}"
    );
    assert!(
        logged
            .iter()
            .any(|(e, c)| e == "world/status/text" && c.starts_with("TextDocument:")),
        "and the rolling-latest mirror lands at <entity>/text — without it the \
         mapped view renders empty: {logged:?}"
    );
}

/// A `sensor_msgs/LaserScan` wire frame: seven fixed `float32`s, then three
/// variable entries in declaration order `header(0)` / `ranges(1)` /
/// `intensities(2)`. `ranges` is a PRIMITIVE f32 array — raw LE bytes back to
/// back, no count prefix and no canonical element framing.
fn build_laserscan_frame(
    angle_min: f32,
    angle_increment: f32,
    range_min: f32,
    range_max: f32,
    ranges: &[f32],
    timestamp_ns: u64,
) -> Vec<u8> {
    let l = layout_of("sensor_msgs/LaserScan");
    assert_eq!(
        l.variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["header", "ranges", "intensities"],
        "LaserScan variable-field declaration order changed — update this builder"
    );
    let fixed = l.fixed_size;
    let table = l.offset_table_bytes();
    let fixed_off = |name: &str| {
        l.fixed_fields
            .iter()
            .find(|f| f.name == name)
            .unwrap_or_else(|| panic!("LaserScan has no fixed field '{name}'"))
            .offset
    };
    let mut payload = vec![0u8; fixed + table];
    for (name, v) in [
        ("angle_min", angle_min),
        ("angle_increment", angle_increment),
        ("range_min", range_min),
        ("range_max", range_max),
    ] {
        let off = fixed_off(name);
        payload[off..off + 4].copy_from_slice(&v.to_le_bytes());
    }
    let mut blob = Vec::new();
    for r in ranges {
        blob.extend_from_slice(&r.to_le_bytes());
    }
    write_offset_entry(&mut payload, fixed, 0, 0, 0); // header: empty
    write_offset_entry(
        &mut payload,
        fixed,
        1,
        (fixed + table) as u32,
        blob.len() as u32,
    );
    write_offset_entry(&mut payload, fixed, 2, 0, 0); // intensities: empty
    payload.extend_from_slice(&blob);

    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: <native_ros2_messages::sensor_msgs::LaserScan as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: 3,
        sequence: 0,
        timestamp_ns,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

/// The `sensor_msgs/LaserScan` render arm: the whole scan is ONE `Points3D`
/// ring, not one chunk per ray.
///
/// Nothing drove `sink.rs`'s `ArchetypeKind::LaserScan` arm through
/// `dispatch_frame` before — LaserScan appeared only in layout/blueprint tables.
/// Hand oracle on the polar→cartesian lift: two in-range rays at 0 and π/2.
#[test]
fn laserscan_frame_renders_one_points3d_ring() {
    let walker = builtin_walker();
    let frame = build_laserscan_frame(
        0.0,
        std::f32::consts::FRAC_PI_2,
        0.0,
        10.0,
        &[1.0, 2.0],
        42_000,
    );

    let fv = walker.walk_by_hash(&frame).expect("walk LaserScan");
    assert_eq!(
        classify_schema("sensor_msgs/LaserScan"),
        Some(ArchetypeKind::LaserScan)
    );
    let pts = cerulion_viz::archetype::laserscan_points(&fv);
    assert_eq!(pts.len(), 2, "two in-range rays: {pts:?}");
    // Hand oracle: r=1 at angle 0 → (1,0,0); r=2 at angle π/2 → (0,2,0).
    assert!(
        (pts[0][0] - 1.0).abs() < 1e-5 && pts[0][1].abs() < 1e-5,
        "ray 0 lifts to (1,0,0): {pts:?}"
    );
    assert!(
        pts[1][0].abs() < 1e-5 && (pts[1][1] - 2.0).abs() < 1e-5,
        "ray 1 lifts to (0,2,0): {pts:?}"
    );

    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "scan", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "the whole ring is ONE Points3D chunk"
    );
    assert!(!state.took_anyvalues_fallback("sensor_msgs/LaserScan"));
}

/// A NAME-MAPPED element array that is EMPTY draws nothing — and takes no field
/// dump either, because an empty array is not a failure. The SAME topic once it
/// fills draws the polyline + its vertices, so the zero is "nothing to draw",
/// not "this topic never draws".
///
/// `a_warmed_topic_that_goes_idle_keeps_its_archetype_instead_of_flipping_to_plots`
/// also asserts 0, but its zero is MEMO-driven on an UNMAPPED schema — its own
/// cold leg on the same idle frame records 2. This is the cold NAME-MAPPED case,
/// which nothing dispatched before.
#[test]
fn an_idle_name_mapped_element_array_draws_nothing_then_draws_when_it_fills() {
    let walker = builtin_walker();
    let idle = build_header_plus_array_frame(
        "nav_msgs/Path",
        <native_ros2_messages::nav_msgs::Path as ShmMessage>::SCHEMA_HASH,
        "poses",
        &[],
    );
    let idle_fv = walker.walk_by_hash(&idle).expect("walk empty Path");
    assert_eq!(
        scan_element_arrays(&idle_fv),
        ElementArrayScan::Empty {
            field: "poses".to_string()
        },
        "the premise, and the load-bearing distinction: an empty array is EMPTY, \
         not ABSENT — `Absent` is what takes the field dump"
    );

    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "plan", &idle, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        0,
        "an idle NAME-MAPPED array draws NOTHING — no plots, and no field dump \
         either"
    );
    assert!(
        !state.took_anyvalues_fallback("nav_msgs/Path"),
        "an empty element array is not a decode failure"
    );

    // The SAME topic, now carrying two poses.
    let planned = build_header_plus_array_frame(
        "nav_msgs/Path",
        <native_ros2_messages::nav_msgs::Path as ShmMessage>::SCHEMA_HASH,
        "poses",
        &counted_blob(&[
            pose_stamped_body([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], "map"),
            pose_stamped_body([2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], "map"),
        ]),
    );
    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "plan", &planned, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        2,
        "one LineStrips3D polyline + one Points3D of its vertices"
    );
}

/// An UNMAPPED vendor type whose STAMPED element array must reach the polyline
/// render through the SHAPE ladder alone — the automagic half.
///
/// `path_frame_renders_a_polyline_and_its_vertices` pins the same 2 chunks for
/// NAME-mapped `nav_msgs/Path`, and
/// `a_stale_path_memo_cannot_draw_a_polyline_through_an_unordered_array` drives
/// this vendor type but asserts its UNSTAMPED leg (1 chunk). The stamped leg's
/// chunk delta was witnessed only by the retired stateless path.
#[test]
fn an_unmapped_stamped_element_array_renders_a_polyline_and_its_vertices() {
    let walker = survey_walker();
    assert_eq!(
        classify_schema("acme/SurveyPlan"),
        None,
        "the premise: unmapped, so only the element SHAPE can decide"
    );
    let traj: [[f64; 3]; 3] = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]];
    let frame = survey_plan_frame(&traj, &[]);
    let fv = walker.walk_by_hash(&frame).expect("walk SurveyPlan");
    let oracle: Vec<[f32; 3]> = traj
        .iter()
        .map(|p| [p[0] as f32, p[1] as f32, p[2] as f32])
        .collect();
    assert_eq!(
        scan_element_arrays(&fv).parts().map(|p| p.geometry.clone()),
        Some(ElementGeometry::Path(oracle)),
        "the stamped elements resolve to an ORDERED path, against a hand oracle"
    );

    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "survey", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        2,
        "one LineStrips3D polyline + one Points3D of its vertices — the shape \
         ladder reaches the same render the name map does"
    );
    assert!(!state.took_anyvalues_fallback("acme/SurveyPlan"));
}

/// An UNMAPPED vendor type carrying an element array of something NOT SPATIAL:
/// a bag of readings, not a set of poses.
///
/// The ANTI-TAUTOLOGY half of the element ladder — without it, "an element array
/// draws geometry" is satisfied by a render that draws geometry for every
/// element array. Each element is recursively FIXED (one `float64`), so the
/// array rides the STRIDE form: back-to-back 8-byte sections, no count prefix.
fn readings_schemas() -> Vec<MessageSchema> {
    let mut schemas = all_schemas();
    schemas.push(parse_rosmsg("float64 volts\n", "Reading", Some("acme")).expect("Reading parses"));
    schemas.push(
        parse_rosmsg("acme/Reading[] readings\n", "Readings", Some("acme"))
            .expect("Readings parses"),
    );
    schemas
}

fn readings_walker() -> FrameWalker {
    let (walker, _warnings) = FrameWalker::new(readings_schemas());
    walker
}

/// One `acme/Readings` frame — `readings` is the sole (variable) field.
fn readings_frame(volts: &[f64]) -> Vec<u8> {
    let (mut resolver, _) = LayoutResolver::new(readings_schemas());
    let l = resolver
        .layout_of("acme/Readings")
        .expect("Readings layout");
    assert_eq!(
        (l.fixed_size, l.offset_table_bytes()),
        (0, 8),
        "Readings shape drifted — update this builder"
    );
    let mut blob = Vec::new();
    for v in volts {
        blob.extend_from_slice(&v.to_le_bytes());
    }
    let table = l.offset_table_bytes();
    let mut payload = vec![0u8; table];
    write_offset_entry(&mut payload, 0, 0, table as u32, blob.len() as u32);
    payload.extend_from_slice(&blob);
    let hash = readings_walker()
        .schema_hash_for("acme/Readings")
        .expect("Readings hash");
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: WireHeader::SIZE as u32,
        offset_table_count: 1,
        sequence: 0,
        timestamp_ns: 42_000,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

#[test]
fn a_non_spatial_element_array_takes_the_dump_instead_of_drawing_geometry() {
    let walker = readings_walker();
    let frame = readings_frame(&[12.4]);
    let fv = walker.walk_by_hash(&frame).expect("walk Readings");
    assert_eq!(
        classify_schema("acme/Readings"),
        None,
        "the premise: unmapped, so only the element SHAPE can decide"
    );
    assert_eq!(
        scan_element_arrays(&fv),
        ElementArrayScan::Absent,
        "an element array of NOTHING SPATIAL yields no geometry"
    );

    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "readings", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        1,
        "exactly ONE TextDocument field dump — never a polyline or a point cloud \
         invented from a bag of numbers"
    );
    assert!(state.took_anyvalues_fallback("acme/Readings"));
}

/// A `geometry_msgs/PolygonStamped` frame whose `points` array sits ONE HOP DOWN
/// inside `polygon` — the name map still promotes it to an ordered polyline.
///
/// `polygon_frame_name_mapped_to_path3d_renders_a_polyline_not_points` pins the
/// flat `geometry_msgs/Polygon`; the one-hop-down variant is dispatched here only
/// as an UNDECODABLE array (`nested_undecodable_polygon_frame`), so the DRAWING
/// case was witnessed only by the retired stateless path.
#[test]
fn a_polygon_stamped_frame_draws_a_polyline_from_an_array_one_hop_down() {
    let walker = builtin_walker();
    assert_eq!(
        classify_schema("geometry_msgs/PolygonStamped"),
        Some(ArchetypeKind::Path3D),
        "the name-map premise this whole arm rests on"
    );
    // `geometry_msgs/Point32` components are f32 on the wire, so the oracle is
    // f32 too — no lossy hop between what is written and what is asserted.
    let oracle: [[f32; 3]; 3] = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 2.0, 0.0]];
    let mut points_blob = Vec::new();
    for p in &oracle {
        for c in p {
            points_blob.extend_from_slice(&c.to_le_bytes());
        }
    }
    let frame = build_header_plus_array_frame(
        "geometry_msgs/PolygonStamped",
        <PolygonStamped as ShmMessage>::SCHEMA_HASH,
        "polygon",
        &polygon_body(&points_blob),
    );

    let fv = walker.walk_by_hash(&frame).expect("walk PolygonStamped");
    assert_eq!(
        scan_element_arrays_for_kind(&fv, ArchetypeKind::Path3D.forces_ordered_elements())
            .parts()
            .map(|p| p.geometry.clone()),
        Some(ElementGeometry::Path(oracle.to_vec())),
        "the array one hop down is found and promoted, against a hand oracle"
    );

    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "footprint", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        2,
        "one LineStrips3D polyline + one Points3D of its vertices"
    );
    assert!(!state.took_anyvalues_fallback("geometry_msgs/PolygonStamped"));
}

/// An UNMAPPED vendor type carrying a ROTATION plus one sibling numeric —
/// `Transform3DWithScalars`: one rotation chunk + one sibling series.
///
/// `a_pure_pose_frame_still_records_exactly_one_transform` drives the plain
/// `Transform3D` (1 chunk) and `a_spatial_frame_with_sibling_telemetry_…` drives
/// the POINT-plus-siblings twin (11); the ROTATION-plus-siblings arm was
/// witnessed only by the retired stateless path, and it is the one where a
/// regression that drops the siblings still records a plausible-looking 1.
fn attitude_schemas() -> Vec<MessageSchema> {
    let mut schemas = all_schemas();
    schemas.push(
        parse_rosmsg(
            "geometry_msgs/Quaternion orientation\nfloat64 temperature_c\n",
            "AttitudeReport",
            Some("acme"),
        )
        .expect("AttitudeReport parses"),
    );
    schemas
}

fn attitude_walker() -> FrameWalker {
    let (walker, _warnings) = FrameWalker::new(attitude_schemas());
    walker
}

/// One fixed-only `acme/AttitudeReport` frame (nested `Quaternion` then an f64).
fn attitude_report_frame(orientation: [f64; 4], temperature_c: f64) -> Vec<u8> {
    let (mut resolver, _) = LayoutResolver::new(attitude_schemas());
    let l = resolver
        .layout_of("acme/AttitudeReport")
        .expect("AttitudeReport layout");
    assert_eq!(
        (l.fixed_size, l.offset_table_bytes()),
        (40, 0),
        "AttitudeReport shape drifted — update this builder"
    );
    let off = |name: &str| {
        l.fixed_fields
            .iter()
            .find(|f| f.name == name)
            .unwrap_or_else(|| panic!("AttitudeReport has no fixed field '{name}'"))
            .offset
    };
    let mut payload = vec![0u8; l.fixed_size];
    let base = off("orientation");
    for (i, c) in orientation.iter().enumerate() {
        payload[base + i * 8..base + i * 8 + 8].copy_from_slice(&c.to_le_bytes());
    }
    let t = off("temperature_c");
    payload[t..t + 8].copy_from_slice(&temperature_c.to_le_bytes());

    let hash = attitude_walker()
        .schema_hash_for("acme/AttitudeReport")
        .expect("AttitudeReport hash");
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + l.fixed_size) as u32,
        offset_table_count: 0,
        sequence: 0,
        timestamp_ns: 42_000,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

#[test]
fn a_rotation_with_sibling_telemetry_renders_the_transform_and_its_plots() {
    let walker = attitude_walker();
    let frame = attitude_report_frame([0.0, 0.0, 0.0, 1.0], 41.5);
    let fv = walker.walk_by_hash(&frame).expect("walk AttitudeReport");
    assert_eq!(
        infer_archetype_from_shape(&fv),
        ArchetypeKind::Transform3DWithScalars
    );
    // Hand oracle: the point/rotation is NOT re-plotted, the sibling is.
    assert_eq!(
        spatial_sibling_series(&fv).0,
        vec![("temperature_c".to_string(), 41.5)],
        "exactly the one sibling series — the quaternion is geometry, not telemetry"
    );

    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let baseline = storage.num_msgs();
    dispatch_frame(&rec, &walker, "att", &frame, &mut state);
    rec.flush_blocking().expect("flush");
    assert_eq!(
        storage.num_msgs() - baseline,
        2,
        "one rotation chunk + one sibling series"
    );
    assert!(!state.took_anyvalues_fallback("acme/AttitudeReport"));
}
