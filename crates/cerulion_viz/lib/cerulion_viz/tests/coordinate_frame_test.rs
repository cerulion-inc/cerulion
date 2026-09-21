// SPDX-License-Identifier: AGPL-3.0-only
//! A topic is POSED by naming its coordinate FRAME, not by
//! where it sits in the entity-path tree.
//!
//! Entity paths are per-topic unique (`world/<topic>`), which rules out
//! posing geometry by path: media topics would have to FOLD onto
//! `world/odom/base/lidar` / `.../camera`, i.e. onto entities nested under the
//! `/tf` tree, for Rerun to compose the chain by path hierarchy. Posing works
//! WITHOUT that fold: the sink logs a rerun 0.34
//! [`rerun::CoordinateFrame`] naming the implicit frame of the entity `/tf`
//! already logs that frame's transform at, so composition stays Rerun's job and
//! nothing here does a tf2-style `lookupTransform`.
//!
//! # TWO mechanisms, and which one is correct depends on the PAYLOAD
//!
//! rerun 0.34 has two components here and they answer different questions:
//!
//! - `CoordinateFrame:frame` relocates an entity's own visualizer DATA into
//!   another frame. The transform cache never reads it.
//! - `Transform3D:parent_frame` names the frame a logged transform is expressed
//!   in, i.e. it RE-PARENTS the frame that entity defines. `re_tf`'s transform
//!   forest reads exactly this when walking the chain; null ⇒ the entity's PATH
//!   parent.
//!
//! So a DATA payload (`Points3D`, `LineStrips3D`, images) is posed by a
//! `CoordinateFrame`, and a TRANSFORM payload (`Odometry`, `Pose`, `Imu`, and the
//! robot root) by its own `parent_frame`.
//!
//! **It is easy to get this backwards** and pose everything with a
//! `CoordinateFrame`, on the belief that "a `Transform3D` carrying explicit
//! `parent_frame`/`child_frame` declares a frame-graph EDGE and does not pose the
//! entity at all". For the 7 of the Go2's 17 frame-carrying topics whose payload
//! IS a transform, doing so makes the pose inert (the chain still composes through the
//! path parent) while still moving the entity's own geometry — and on the robot
//! root, which heads the URDF link chain, it visibly TEARS the skeleton: the base
//! marker jumps to the odom pose while every leg link stays put.
//!
//! The tests that pin this assert COMPOSITION — the exact `parent_frame`
//! component and the ABSENCE of a `CoordinateFrame` — never chunk presence, which
//! is what let the broken mechanism look correct:
//! `an_odometry_payload_transform_is_posed_by_its_parent_frame_not_a_coordinate_frame`,
//! `an_odom_elected_topic_poses_the_robot_root_without_tearing_the_skeleton`.
//!
//! # Oracles
//!
//! Frames are HAND-BUILT wire frames (real layout engine, real walker, real
//! `dispatch_frame`), and every assertion is against a hand-written expected
//! value or a paired control — never a self-compare. Chunk counts come from a
//! rerun MemorySink, and the log-level assertions from `#[traced_test]`.

mod common;

use std::sync::{Mutex, MutexGuard, OnceLock};

use cerulion_core::codegen::layout::{LayoutResolver, WireLayout};
use cerulion_core::codegen::{parse_rosmsg, FrameWalker, MessageSchema};
use cerulion_core::message::ShmMessage;
use cerulion_core::wire::WireHeader;
use native_ros2_messages::nav_msgs::Odometry;
use native_ros2_messages::sensor_msgs::PointCloud2;
use native_ros2_messages::tf2_msgs::TFMessage;

use cerulion_viz::sink::{
    dispatch_frame, reported_entity_for, route_for_input, route_key_for_topic, ArchetypeKind,
    SinkState,
};
use cerulion_viz::tf::{frame_id_of, FrameRegistry};

use common::all_schemas;

/// The `unitree_go/HeightMap` definition VERBATIM from the Go2's own served
/// schema (`research/go2_harvest/unitree_go_msg/HeightMap.msg`) — the
/// real-world type that carries `frame_id` TOP-LEVEL with no `std_msgs/Header`.
/// A `header.frame_id`-only extractor misses it SILENTLY, which is the whole
/// reason [`frame_id_of`] reads both shapes.
const HEIGHT_MAP_MSG: &str = "\
float64 stamp
string frame_id
float32 resolution
uint32 width
uint32 height
float32[2] origin
float32[] data
";

/// The built-in schema set PLUS `unitree_go/HeightMap`.
fn schemas_with_height_map() -> Vec<MessageSchema> {
    let mut schemas = all_schemas();
    schemas.push(
        parse_rosmsg(HEIGHT_MAP_MSG, "HeightMap", Some("unitree_go")).expect("HeightMap parses"),
    );
    schemas
}

fn walker() -> FrameWalker {
    FrameWalker::new(schemas_with_height_map()).0
}

fn layout_of(qname: &str) -> WireLayout {
    let (mut resolver, _) = LayoutResolver::new(schemas_with_height_map());
    resolver.layout_of(qname).expect("schema resolves")
}

fn height_map_schema_hash() -> u64 {
    schemas_with_height_map()
        .iter()
        .find(|s| s.qualified_name() == "unitree_go/HeightMap")
        .expect("HeightMap in the set")
        .schema_hash()
}

/// Write one 8-byte offset-table entry (`offset` u32, `length` u32) at
/// `table_start + index * 8`.
fn write_offset_entry(buf: &mut [u8], table_start: usize, index: usize, offset: u32, length: u32) {
    let at = table_start + index * 8;
    buf[at..at + 4].copy_from_slice(&offset.to_le_bytes());
    buf[at + 4..at + 8].copy_from_slice(&length.to_le_bytes());
}

/// A `std_msgs/Header` sub-frame: fixed `Time{sec, nanosec}` (8 B) | entry[0]
/// `frame_id` (8 B) | UTF-8 (crib: `sink_dispatch_test::header_body`).
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

/// Build a `sensor_msgs/PointCloud2` frame with a POPULATED `header.frame_id`
/// and one XYZI point. Variable declaration order header(0) / fields(1) /
/// data(2).
fn build_cloud_frame(frame_id: &str, timestamp_ns: u64) -> Vec<u8> {
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
    let table = layout.offset_table_bytes();
    let fixed_off = |name: &str| {
        layout
            .fixed_fields
            .iter()
            .find(|f| f.name == name)
            .unwrap_or_else(|| panic!("PointCloud2 has no fixed field '{name}'"))
            .offset
    };

    let hdr = header_body(frame_id);
    let mut points = Vec::new();
    for c in [1.0f32, 2.0, 3.0, 0.5] {
        points.extend_from_slice(&c.to_le_bytes());
    }

    let mut payload = vec![0u8; fixed + table];
    let put_u32 = |buf: &mut [u8], off: usize, v: u32| {
        buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
    };
    put_u32(&mut payload, fixed_off("height"), 1);
    put_u32(&mut payload, fixed_off("width"), 1);
    put_u32(&mut payload, fixed_off("point_step"), 16);
    put_u32(&mut payload, fixed_off("row_step"), 16);

    let hdr_off = (fixed + table) as u32;
    let data_off = hdr_off + hdr.len() as u32;
    write_offset_entry(&mut payload, fixed, 0, hdr_off, hdr.len() as u32);
    write_offset_entry(&mut payload, fixed, 1, 0, 0); // fields: empty
    write_offset_entry(&mut payload, fixed, 2, data_off, points.len() as u32);
    payload.extend_from_slice(&hdr);
    payload.extend_from_slice(&points);

    frame_with_header(
        <PointCloud2 as ShmMessage>::SCHEMA_HASH,
        fixed,
        3,
        timestamp_ns,
        payload,
    )
}

/// Build a `nav_msgs/Odometry` frame with a POPULATED `header.frame_id` and a
/// non-identity pose. Variable declaration order header(0) / child_frame_id(1).
fn build_odometry_frame(frame_id: &str, position: [f64; 3], timestamp_ns: u64) -> Vec<u8> {
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
    let table = layout.offset_table_bytes();
    let pose_off = layout
        .fixed_fields
        .iter()
        .find(|f| f.name == "pose")
        .expect("Odometry has a fixed `pose`")
        .offset;

    let hdr = header_body(frame_id);
    let child = b"base";
    let mut payload = vec![0u8; fixed + table];
    // PoseWithCovariance = Pose { Point position, Quaternion orientation } + cov.
    for (i, v) in position.iter().enumerate() {
        let at = pose_off + i * 8;
        payload[at..at + 8].copy_from_slice(&v.to_le_bytes());
    }
    // Unit quaternion w = 1 (position 3 doubles in, then x,y,z,w).
    let w_at = pose_off + 6 * 8;
    payload[w_at..w_at + 8].copy_from_slice(&1.0f64.to_le_bytes());

    let hdr_off = (fixed + table) as u32;
    let child_off = hdr_off + hdr.len() as u32;
    write_offset_entry(&mut payload, fixed, 0, hdr_off, hdr.len() as u32);
    write_offset_entry(&mut payload, fixed, 1, child_off, child.len() as u32);
    payload.extend_from_slice(&hdr);
    payload.extend_from_slice(child);

    frame_with_header(
        <Odometry as ShmMessage>::SCHEMA_HASH,
        fixed,
        2,
        timestamp_ns,
        payload,
    )
}

/// Build a `unitree_go/HeightMap` frame — `frame_id` TOP-LEVEL, no `Header`.
/// Variable declaration order frame_id(0) / data(1).
fn build_height_map_frame(frame_id: &str, timestamp_ns: u64) -> Vec<u8> {
    let layout = layout_of("unitree_go/HeightMap");
    assert_eq!(
        layout
            .variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["frame_id", "data"],
        "HeightMap variable-field declaration order changed — update this builder"
    );
    let fixed = layout.fixed_size;
    let table = layout.offset_table_bytes();
    let cells: Vec<u8> = 1.0f32.to_le_bytes().to_vec();

    let mut payload = vec![0u8; fixed + table];
    let fid_off = (fixed + table) as u32;
    let data_off = fid_off + frame_id.len() as u32;
    write_offset_entry(&mut payload, fixed, 0, fid_off, frame_id.len() as u32);
    write_offset_entry(&mut payload, fixed, 1, data_off, cells.len() as u32);
    payload.extend_from_slice(frame_id.as_bytes());
    payload.extend_from_slice(&cells);

    frame_with_header(height_map_schema_hash(), fixed, 2, timestamp_ns, payload)
}

/// Build a `tf2_msgs/TFMessage` frame carrying ONE transform, using the
/// production `go2_tf` encoder (so the sink's real decoder reads it).
fn build_tf_frame(parent: &str, child: &str, timestamp_ns: u64) -> Vec<u8> {
    let layout = layout_of("tf2_msgs/TFMessage");
    let fixed = layout.fixed_size;
    let table = layout.offset_table_bytes();
    let transforms = go2_tf::encode_tf_transforms(&[go2_tf::TfTransform {
        stamp_sec: 0,
        stamp_nanosec: 0,
        frame_id: parent.to_string(),
        child_frame_id: child.to_string(),
        translation: [1.0, 0.0, 0.0],
        rotation: [0.0, 0.0, 0.0, 1.0],
    }]);
    let mut payload = vec![0u8; fixed + table];
    write_offset_entry(
        &mut payload,
        fixed,
        0,
        (fixed + table) as u32,
        transforms.len() as u32,
    );
    payload.extend_from_slice(&transforms);
    frame_with_header(
        <TFMessage as ShmMessage>::SCHEMA_HASH,
        fixed,
        1,
        timestamp_ns,
        payload,
    )
}

fn frame_with_header(
    schema_hash: u64,
    fixed: usize,
    table_count: u32,
    timestamp_ns: u64,
    payload: Vec<u8>,
) -> Vec<u8> {
    let mut frame = vec![0u8; WireHeader::SIZE];
    let header = WireHeader {
        schema_hash,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: table_count,
        sequence: 0,
        timestamp_ns,
    };
    header.write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);
    frame
}

/// Rerun's `RecordingStream` machinery is process-shared enough that concurrent
/// memory sinks in one binary interleave; every test here takes this lock.
fn rerun_lock() -> MutexGuard<'static, ()> {
    static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
    LOCK.get_or_init(|| Mutex::new(()))
        .lock()
        .unwrap_or_else(|e| e.into_inner())
}

fn memory_sink(tag: &str) -> (rerun::RecordingStream, rerun::sink::MemorySinkStorage) {
    let (rec, storage) = rerun::RecordingStreamBuilder::new(tag.to_string())
        .memory()
        .expect("memory recording");
    (rec, storage)
}

/// A chunk's entity path in the SAME spelling `route_for_input` produces (rerun
/// renders an `EntityPath` with a leading `/`; our entity strings are un-rooted).
fn entity_path_string(chunk: &rerun::log::Chunk) -> String {
    chunk
        .entity_path()
        .to_string()
        .trim_start_matches('/')
        .to_string()
}

/// Every chunk the sink emitted, decoded back out of the memory sink.
fn chunks(storage: &rerun::sink::MemorySinkStorage) -> Vec<rerun::log::Chunk> {
    storage
        .take()
        .into_iter()
        .filter_map(|msg| match msg {
            rerun::log::LogMsg::ArrowMsg(_, arrow_msg) => {
                Some(rerun::log::Chunk::from_arrow_msg(&arrow_msg).expect("decode chunk"))
            }
            _ => None,
        })
        .collect()
}

/// Every `CoordinateFrame` chunk in the sink's output, as
/// `(entity path, frame name)` pairs in log order — read back off the REAL
/// rerun store, so this observes what a viewer would.
fn coordinate_frames(storage: &rerun::sink::MemorySinkStorage) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for chunk in chunks(storage) {
        // rerun renders an EntityPath with a leading '/'; our entity strings are
        // the un-rooted form (`world/...`), so normalize once here.
        let entity = entity_path_string(&chunk);
        for (descr, list) in chunk.components().iter() {
            if descr.as_str() != "CoordinateFrame:frame" {
                continue;
            }
            // Deserialize through the real component type rather than poking at
            // arrow, so the read is the same one a viewer performs.
            let ids = <rerun::components::TransformFrameId as rerun::Loggable>::from_arrow(
                list.list_array.values().as_ref(),
            )
            .expect("TransformFrameId column");
            for id in ids {
                out.push((entity.clone(), id.0.to_string()));
            }
        }
    }
    out
}

/// Read a `TransformFrameId` component column back through the REAL component
/// type (the same read a viewer performs), as one string per row.
fn frame_id_column(values: &dyn rerun::external::arrow::array::Array) -> Vec<String> {
    <rerun::components::TransformFrameId as rerun::Loggable>::from_arrow(values)
        .expect("TransformFrameId column")
        .into_iter()
        .map(|id| id.0.to_string())
        .collect()
}

/// Everything the sink logged, read out of the store in ONE drain (the memory
/// sink is consumed by reading it, so a test that needs both views must take
/// them together).
#[derive(Debug, Default)]
struct Logged {
    /// `(entity, frame)` per `CoordinateFrame:frame` row — poses an entity's own
    /// visualizer DATA. The transform cache never reads this.
    coordinate_frames: Vec<(String, String)>,
    /// `(entity, parent_frame)` per `Transform3D` chunk — `None` when the chunk
    /// carries no `parent_frame` component, i.e. the transform composes through
    /// the entity's PATH parent (rerun's default). THIS is the component
    /// `re_tf`'s transform forest walks, so it is what decides COMPOSITION.
    transforms: Vec<(String, Option<String>)>,
    /// `(entity, child_frame)` per explicit `Transform3D:child_frame` row —
    /// renames the frame the entity DEFINES. Must stay empty: the skeleton's
    /// links chain to the robot root's IMPLICIT (path-derived) child frame.
    child_frames: Vec<(String, String)>,
}

impl Logged {
    /// The parent frame recorded for `entity`'s transform, if one was logged.
    /// `Some(None)` = a transform with no explicit parent (path-parented);
    /// `None` = no transform logged there at all.
    fn parent_frame_of(&self, entity: &str) -> Option<Option<String>> {
        self.transforms
            .iter()
            .find(|(e, _)| e == entity)
            .map(|(_, f)| f.clone())
    }

    /// Every entity that received a `CoordinateFrame`.
    fn coordinate_framed_entities(&self) -> Vec<String> {
        self.coordinate_frames
            .iter()
            .map(|(e, _)| e.clone())
            .collect()
    }
}

fn logged(storage: &rerun::sink::MemorySinkStorage) -> Logged {
    let mut out = Logged::default();
    for chunk in chunks(storage) {
        let entity = entity_path_string(&chunk);
        // A chunk carries MANY rows (rerun compacts same-entity logs), so read the
        // parent-frame column ROW-WISE — collapsing it to one entry per chunk
        // would hide exactly the change-over-time this file asserts.
        let mut is_transform = false;
        let mut parents: Vec<String> = Vec::new();
        for (descr, list) in chunk.components().iter() {
            match descr.as_str() {
                "CoordinateFrame:frame" => {
                    for f in frame_id_column(list.list_array.values().as_ref()) {
                        out.coordinate_frames.push((entity.clone(), f));
                    }
                }
                "Transform3D:parent_frame" => {
                    is_transform = true;
                    parents = frame_id_column(list.list_array.values().as_ref());
                }
                "Transform3D:child_frame" => {
                    for f in frame_id_column(list.list_array.values().as_ref()) {
                        out.child_frames.push((entity.clone(), f));
                    }
                }
                "Transform3D:translation" | "Transform3D:quaternion" => is_transform = true,
                _ => {}
            }
        }
        if is_transform {
            if parents.is_empty() {
                // No `parent_frame` component at all ⇒ every row composes through
                // the entity's PATH parent (rerun's default).
                for _ in 0..chunk.num_rows() {
                    out.transforms.push((entity.clone(), None));
                }
            } else {
                for f in parents {
                    out.transforms.push((entity.clone(), Some(f)));
                }
            }
        }
    }
    out
}

/// Route + dispatch a topic's frames exactly as the daemon does.
fn dispatch(
    rec: &rerun::RecordingStream,
    walker: &FrameWalker,
    topic: &str,
    frames: &[Vec<u8>],
    state: &mut SinkState,
) {
    let key = route_key_for_topic(topic, None);
    for frame in frames {
        dispatch_frame(rec, walker, &key, frame, state);
    }
}

fn entity_of(topic: &str) -> String {
    route_for_input(&route_key_for_topic(topic, None)).entity
}

// ── 1. frame_id extraction: BOTH wire shapes ─────────────────────────────────

#[test]
fn frame_id_is_read_from_a_nested_header_and_from_a_top_level_field() {
    let walker = walker();

    // (a) the CONVENTION: `header.frame_id` (PointCloud2 and friends).
    let cloud = build_cloud_frame("odom", 1_000);
    let fv = walker.walk_by_hash(&cloud).expect("walk PointCloud2");
    assert_eq!(frame_id_of(&fv), Some("odom"), "header.frame_id");

    // (b) the vendor shape: TOP-LEVEL `frame_id`, no `std_msgs/Header` at all.
    // A `header`-only extractor returns None here and the topic silently never
    // gets posed — the exact silent-miss this arm exists to prevent.
    let hm = build_height_map_frame("utlidar_lidar", 2_000);
    let fv = walker.walk_by_hash(&hm).expect("walk HeightMap");
    assert_eq!(
        frame_id_of(&fv),
        Some("utlidar_lidar"),
        "top-level frame_id"
    );

    // (c) a message with NEITHER reads absent (a Vector3 has no frame at all).
    let twist = common::build_twist([1.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
    let fv = walker.walk_by_hash(&twist).expect("walk Twist");
    assert_eq!(frame_id_of(&fv), None, "no frame_id anywhere");

    // (d) an EMPTY frame_id is ABSENT, not a frame named "" (ROS treats "" as
    // "no frame stated"; resolving it would fabricate a pose).
    let empty = build_cloud_frame("", 3_000);
    let fv = walker.walk_by_hash(&empty).expect("walk PointCloud2");
    assert_eq!(frame_id_of(&fv), None, "an empty frame_id is absent");
}

// ── 2. resolution: known alias / observed-on-/tf / unknown ───────────────────

#[test]
fn frame_resolution_needs_an_alias_or_an_observed_tf_transform() {
    let mut reg = FrameRegistry::new();
    // A KNOWN alias resolves without any /tf at all (the tree states where it
    // belongs) — hand oracles for the four canonical Go2 entities.
    assert_eq!(
        reg.resolve("odom").as_deref(),
        Some("tf#/world/tf-tree/odom")
    );
    assert_eq!(
        reg.resolve("base").as_deref(),
        Some("tf#/world/tf-tree/odom/base")
    );
    assert_eq!(
        reg.resolve("base_link").as_deref(),
        Some("tf#/world/tf-tree/odom/base")
    );
    assert_eq!(
        reg.resolve("livox_frame").as_deref(),
        Some("tf#/world/tf-tree/odom/base/lidar")
    );
    // A leading '/' on the frame id is tolerated (absolute frame ids).
    assert_eq!(
        reg.resolve("/odom").as_deref(),
        Some("tf#/world/tf-tree/odom")
    );

    // An UNKNOWN frame does NOT resolve — the caller must log nothing rather than
    // name the fallback entity, which is connected to `world` by identity and
    // would render the sensor exactly at the robot base (a fabricated mount).
    assert_eq!(reg.resolve("velodyne_link"), None);
    assert!(!reg.has_observed("velodyne_link"));

    // …until /tf actually publishes a transform for it, at which point the tree
    // CAN place it and the very same id resolves to the entity /tf logged at.
    reg.observe_child("velodyne_link");
    assert!(reg.has_observed("velodyne_link"));
    assert_eq!(
        reg.resolve("velodyne_link").as_deref(),
        Some("tf#/world/tf-tree/odom/base/velodyne_link"),
    );
    // Observation is normalized the same way resolution is.
    let mut reg2 = FrameRegistry::new();
    reg2.observe_child("/wrist_mount");
    assert!(reg2.has_observed("wrist_mount"));
    assert_eq!(
        reg2.resolve("wrist_mount").as_deref(),
        Some("tf#/world/tf-tree/odom/base/wrist_mount")
    );
    // A frame whose raw id needed sanitizing keeps the aliasing suffix, so two
    // frames differing only in separators never share one coordinate frame.
    let mut reg3 = FrameRegistry::new();
    reg3.observe_child("a/b");
    reg3.observe_child("a.b");
    assert_ne!(reg3.resolve("a/b"), reg3.resolve("a.b"));
}

// ── 3. the headline: a resolvable frame poses the topic's own entity ─────────

#[test]
fn a_cloud_with_a_known_frame_is_posed_at_its_own_entity() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("cloud_known");
    let walker = walker();
    let mut state = SinkState::new();

    dispatch(
        &rec,
        &walker,
        "/utlidar/cloud",
        &[build_cloud_frame("livox_frame", 1_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    // The cloud's geometry lives at the rotating `sweep/0` sub-entity, so THAT is
    // where the frame assignment must land — a child's implicit frame chains to
    // its PATH parent, not to the frame the parent was re-pointed at, so an
    // assignment only at `world/utlidar/cloud` would leave the points unposed.
    let entity = entity_of("/utlidar/cloud");
    assert_eq!(entity, "world/utlidar/cloud");
    let frames = coordinate_frames(&storage);
    assert!(
        frames.contains(&(
            entity.clone(),
            "tf#/world/tf-tree/odom/base/lidar".to_string()
        )),
        "the topic entity is posed in the lidar frame: {frames:?}"
    );
    assert!(
        frames.contains(&(
            format!("{entity}/viz-sweep/0"),
            "tf#/world/tf-tree/odom/base/lidar".to_string()
        )),
        "the sweep sub-entity carrying the points is posed too: {frames:?}"
    );
}

#[test]
fn a_height_map_with_a_top_level_frame_id_is_posed_too() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("height_map");
    let walker = walker();
    let mut state = SinkState::new();

    // `utlidar_lidar` is a KNOWN lidar-frame alias, so it resolves with no /tf.
    dispatch(
        &rec,
        &walker,
        "/utlidar/height_map_array",
        &[build_height_map_frame("utlidar_lidar", 1_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let entity = entity_of("/utlidar/height_map_array");
    assert_eq!(entity, "world/utlidar/height_map_array");
    assert!(
        coordinate_frames(&storage)
            .contains(&(entity, "tf#/world/tf-tree/odom/base/lidar".to_string())),
        "a top-level frame_id poses the entity exactly like a header one"
    );
}

// ── 4. change-triggered emission (the DATA-payload mechanism) ────────────────

/// A `CoordinateFrame` is emitted only when the resolved frame CHANGES — the
/// per-entity dedup that keeps a constant-`frame_id` topic at ONE chunk for the
/// whole run instead of one per message.
///
/// Driven by a DATA payload (a cloud). Every TRANSFORM payload rides
/// `Transform3D:parent_frame` instead of this mechanism, which is written on every
/// row by construction — its own change-tracking twin is
/// `a_transform_payloads_parent_frame_tracks_the_messages_frame_every_row`.
#[test]
fn the_frame_is_emitted_on_change_not_on_every_message() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("change_triggered");
    let walker = walker();
    let mut state = SinkState::new();

    // frame_id sequence ["livox_frame", "livox_frame", "base"] over three messages
    // of ONE topic ⇒ EXACTLY 2 CoordinateFrame rows, at ticks 0 and 2.
    let topic = "/velodyne/points";
    let seq = ["livox_frame", "livox_frame", "base"];
    let frames: Vec<Vec<u8>> = seq
        .iter()
        .enumerate()
        .map(|(i, f)| build_cloud_frame(f, 1_000 * (i as u64 + 1)))
        .collect();
    dispatch(&rec, &walker, topic, &frames, &mut state);
    rec.flush_blocking().expect("flush");

    let entity = entity_of(topic);
    let emitted: Vec<String> = coordinate_frames(&storage)
        .into_iter()
        .filter(|(e, _)| *e == entity)
        .map(|(_, f)| f)
        .collect();
    assert_eq!(
        emitted,
        vec![
            "tf#/world/tf-tree/odom/base/lidar".to_string(),
            "tf#/world/tf-tree/odom/base".to_string()
        ],
        "exactly one row per CHANGE (the repeat of `livox_frame` emits nothing)"
    );
}

/// The TRANSFORM-payload twin: `parent_frame` rides the transform archetype, so it
/// is written on EVERY row and tracks the message's frame message-by-message.
///
/// It must NOT be deduped the way `emit_frame_at` dedups `CoordinateFrame`: `re_tf`
/// resolves a transform atomically per row, so a row that omitted the component
/// would RESET to the entity's path parent — un-posing every message after the
/// first.
#[test]
fn a_transform_payloads_parent_frame_tracks_the_messages_frame_every_row() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("parent_frame_per_row");
    let walker = walker();
    let mut state = SinkState::new();

    let topic = "/utlidar/robot_pose";
    let seq = ["odom", "odom", "base"];
    let frames: Vec<Vec<u8>> = seq
        .iter()
        .enumerate()
        .map(|(i, f)| build_odometry_frame(f, [i as f64, 0.0, 0.0], 1_000 * (i as u64 + 1)))
        .collect();
    dispatch(&rec, &walker, topic, &frames, &mut state);
    rec.flush_blocking().expect("flush");

    let entity = entity_of(topic);
    let l = logged(&storage);
    let parents: Vec<Option<String>> = l
        .transforms
        .iter()
        .filter(|(e, _)| *e == entity)
        .map(|(_, f)| f.clone())
        .collect();
    assert_eq!(
        parents,
        vec![
            Some("tf#/world/tf-tree/odom".to_string()),
            Some("tf#/world/tf-tree/odom".to_string()),
            Some("tf#/world/tf-tree/odom/base".to_string()),
        ],
        "one parent frame per ROW, tracking the message: {l:?}"
    );
    assert!(l.coordinate_framed_entities().is_empty(), "{l:?}");
}

// ── 5. a TRANSFORM payload is posed by `parent_frame`, never CoordinateFrame ──

/// **THE MECHANISM PIN.**
///
/// The tempting model is that a `Transform3D`'s `parent_frame` "declares a
/// frame-graph EDGE and does not pose the entity at all", which would pose everything
/// with a `CoordinateFrame`. That is backwards, and it is checkable in rerun
/// 0.34's own source: `re_tf`'s transform forest resolves a frame's parent from
/// `Transform3D:parent_frame` (falling back to the entity's PATH parent when it is
/// null), and NEVER reads `CoordinateFrame:frame` — that component only relocates
/// the entity's own visualizer DATA.
///
/// So for an `nav_msgs/Odometry`, whose payload IS a `Transform3D`, a
/// `CoordinateFrame` was the worst of both: the transform kept composing through
/// the path parent (the pose was NOT placed in the odom frame) while the arrow's
/// own geometry jumped into it.
///
/// Asserts COMPOSITION, not presence: the exact `parent_frame` on the payload
/// transform, and that no `CoordinateFrame` is emitted at that entity at all.
#[test]
fn an_odometry_payload_transform_is_posed_by_its_parent_frame_not_a_coordinate_frame() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("payload_parent_frame");
    let walker = walker();
    let mut state = SinkState::new();

    let topic = "/uslam/frontend/odom";
    dispatch(
        &rec,
        &walker,
        topic,
        &[build_odometry_frame("odom", [7.0, 8.0, 9.0], 1_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let entity = entity_of(topic);
    let l = logged(&storage);
    // The payload transform is expressed IN the odom frame — the component the
    // transform cache actually walks.
    assert_eq!(
        l.parent_frame_of(&entity),
        Some(Some("tf#/world/tf-tree/odom".to_string())),
        "the Odometry payload transform must carry the resolved parent frame: {l:?}"
    );
    // …and NOT via the data-only component.
    assert!(
        !l.coordinate_framed_entities().contains(&entity),
        "a transform payload must NOT also get a CoordinateFrame (it would move the \
         arrow's geometry while leaving the chain path-parented): {l:?}"
    );
    // `child_frame` stays implicit: renaming the frame this entity DEFINES would
    // orphan anything path-parented under it.
    assert!(l.child_frames.is_empty(), "{l:?}");
}

/// The WITHHOLD half of the mechanism: an unresolvable frame must leave the
/// payload transform composing through its PATH parent, and this pins that it says
/// so EXPLICITLY rather than by omission.
///
/// (Omission would be equivalent — `re_tf` resolves a `Transform3D` atomically per
/// row and falls back to the path parent when the row carries no `parent_frame`.
/// The explicit value makes the withhold an asserted decision instead of an
/// absence. What that per-row model DOES forbid is deduping the component across
/// rows, which `a_transform_payloads_parent_frame_tracks_the_messages_frame_every_row`
/// pins.)
#[test]
fn an_unresolvable_frame_pins_the_payload_transform_back_to_its_path_parent() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("payload_withhold");
    let walker = walker();
    let mut state = SinkState::new();

    let topic = "/uslam/frontend/odom";
    dispatch(
        &rec,
        &walker,
        topic,
        &[
            // Resolvable first, so there IS a live assignment to stale out.
            build_odometry_frame("odom", [1.0, 0.0, 0.0], 1_000),
            // …then a frame the tree cannot place.
            build_odometry_frame("map", [2.0, 0.0, 0.0], 2_000),
        ],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let entity = entity_of(topic);
    let l = logged(&storage);
    let parents: Vec<Option<String>> = l
        .transforms
        .iter()
        .filter(|(e, _)| *e == entity)
        .map(|(_, f)| f.clone())
        .collect();
    // Hand oracle: posed in odom, then put BACK to the entity's path parent
    // (`world/uslam/frontend`) — never left at the stale odom mount, and never
    // silently omitted.
    assert_eq!(
        parents,
        vec![
            Some("tf#/world/tf-tree/odom".to_string()),
            Some("tf#/world/uslam/frontend".to_string()),
        ],
        "the withheld arm must name the path parent explicitly: {l:?}"
    );
    assert!(l.coordinate_framed_entities().is_empty(), "{l:?}");
}

// ── 6. an unresolvable frame: nothing logged, warned exactly once ───────────

#[test]
#[tracing_test::traced_test]
fn an_unknown_frame_logs_no_coordinate_frame_and_warns_exactly_once() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("unknown_frame");
    let walker = walker();
    let mut state = SinkState::new();

    // `velodyne_link` is not a known alias and no /tf transform for it has been
    // seen, so the tree cannot place it.
    let frames: Vec<Vec<u8>> = (0..4)
        .map(|i| build_cloud_frame("velodyne_link", 1_000 * (i + 1)))
        .collect();
    dispatch(&rec, &walker, "/velodyne/points", &frames, &mut state);
    rec.flush_blocking().expect("flush");

    assert!(
        coordinate_frames(&storage).is_empty(),
        "an unresolvable frame must log NOTHING — naming the fallback entity \
         would render the sensor at a fabricated pose (the robot base)"
    );
    // …but it must never be silent: the warn is the ONLY signal the data is not
    // localized, and it fires once per input, not once per message.
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("the transform tree cannot place"))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!(
                "expected exactly 1 unresolvable-frame warn, got {n}"
            ))
        }
    });
}

/// The paired CONTROL: publishing `/tf` for that same frame makes the very next
/// message resolve. Without this arm the test above would also pass if
/// resolution were broken for EVERY frame.
#[test]
fn a_tf_transform_for_the_frame_makes_the_next_message_resolve() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("tf_then_resolve");
    let walker = walker();
    let mut state = SinkState::new();

    // BEFORE: unresolvable ⇒ nothing.
    dispatch(
        &rec,
        &walker,
        "/velodyne/points",
        &[build_cloud_frame("velodyne_link", 1_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");
    assert!(
        coordinate_frames(&storage).is_empty(),
        "baseline: the frame is not placeable yet"
    );

    // /tf publishes `base -> velodyne_link`, which the sink observes.
    dispatch(
        &rec,
        &walker,
        "/tf",
        &[build_tf_frame("base", "velodyne_link", 2_000)],
        &mut state,
    );
    // AFTER: the same topic's next message resolves to the entity /tf logged at.
    dispatch(
        &rec,
        &walker,
        "/velodyne/points",
        &[build_cloud_frame("velodyne_link", 3_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let entity = entity_of("/velodyne/points");
    let frames = coordinate_frames(&storage);
    assert!(
        frames.contains(&(
            entity,
            "tf#/world/tf-tree/odom/base/velodyne_link".to_string()
        )),
        "once /tf places the frame, the topic is posed in it: {frames:?}"
    );
}

// ── 7. no frame_id at all: silent, and NOT posed ────────────────────────────

#[test]
#[tracing_test::traced_test]
fn a_topic_with_no_frame_id_is_silent_and_unposed() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("no_frame");
    let walker = walker();
    let mut state = SinkState::new();

    // 58 of the Go2's 75 topics carry no frame_id. They log no CoordinateFrame
    // and warn about nothing — their entity keeps its own implicit
    // `tf#/world/<topic>` frame, identity-connected to `tf#/world`, so it renders
    // at the world origin.
    dispatch(
        &rec,
        &walker,
        "/cmd_vel",
        &[common::build_twist([1.0, 0.0, 0.0], [0.0, 0.0, 0.5])],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    assert!(coordinate_frames(&storage).is_empty());
    logs_assert(|lines: &[&str]| {
        if lines
            .iter()
            .any(|l| l.contains("the transform tree cannot place"))
        {
            Err("a frame-LESS topic must not warn about an unplaceable frame".to_string())
        } else {
            Ok(())
        }
    });
}

// ── 8. determinism ─────────────────────────────────────────────────────────

#[test]
fn two_identical_runs_emit_the_identical_frame_sequence() {
    let _g = rerun_lock();
    let walker = walker();
    let run = |tag: &str| {
        let (rec, storage) = memory_sink(tag);
        let mut state = SinkState::new();
        dispatch(
            &rec,
            &walker,
            "/tf",
            &[build_tf_frame("base", "wrist_mount", 500)],
            &mut state,
        );
        for (i, f) in ["odom", "odom", "wrist_mount", "base"].iter().enumerate() {
            dispatch(
                &rec,
                &walker,
                "/utlidar/robot_pose",
                &[build_odometry_frame(
                    f,
                    [i as f64, 0.0, 0.0],
                    1_000 * (i as u64 + 1),
                )],
                &mut state,
            );
        }
        rec.flush_blocking().expect("flush");
        let l = logged(&storage);
        let entity = entity_of("/utlidar/robot_pose");
        l.transforms
            .into_iter()
            .filter(|(e, _)| *e == entity)
            .map(|(_, f)| f)
            .collect::<Vec<_>>()
    };
    let a = run("det_a");
    let b = run("det_b");
    // Anchored to a HAND oracle, so this is not a self-compare: the run is
    // wire-timestamp-driven with nothing wall-clock in it. `wrist_mount` resolves
    // only because the `/tf` frame above published a transform for it.
    let oracle: Vec<Option<String>> = vec![
        Some("tf#/world/tf-tree/odom".to_string()),
        Some("tf#/world/tf-tree/odom".to_string()),
        Some("tf#/world/tf-tree/odom/base/wrist_mount".to_string()),
        Some("tf#/world/tf-tree/odom/base".to_string()),
    ];
    assert_eq!(a, oracle, "run A matches the hand oracle");
    assert_eq!(b, oracle, "run B matches the hand oracle");
}

// ── 11. the CONFIGURED-frame branch (`InputRoute::frame`) ───────────────────

/// A minimal hermetic Go2 URDF with a `radar` mount — the one link
/// `Skeleton::reparent_cloud_route` looks for (crib: `sink_dispatch_test`).
const SKELETON_URDF: &str = r#"<?xml version="1.0"?>
<robot name="fixture">
  <link name="base"/>
  <link name="radar"/>
  <joint name="radar_joint" type="fixed">
    <origin xyz="0.28945 0 -0.046825" rpy="0 2.8782 0"/>
    <parent link="base"/>
    <child link="radar"/>
  </joint>
</robot>"#;

/// **The CONFIGURED-frame branch end to end.**
///
/// `InputRoute::frame` has exactly one producer — `Skeleton::reparent_cloud_route`,
/// which poses the lidar cloud in the URDF `radar` link's frame so the fixed
/// `base → radar` extrinsic superposes cloud + skeleton — and exactly one
/// consumer, `assign_coordinate_frame`'s first arm. Nothing joined the two: every
/// `install_skeleton` in the suite dispatched a `LowState`, never a cloud, so
/// DELETING the consumer arm left the whole suite green while the cloud silently
/// resolved through the DATA path instead (`utlidar_lidar` is a known lidar
/// alias, so it lands on the TF-tree lidar mount — a plausible-looking WRONG
/// pose, i.e. the defect restored invisibly).
///
/// Previously the mechanism was a `route.entity` REWRITE, which every `log_*`
/// call consumed by construction and so could not be silently dropped; making it
/// an opt-in field is exactly what created the hole this test closes.
#[test]
fn a_configured_route_frame_poses_the_cloud_and_beats_its_own_data_frame() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("configured_frame");
    let walker = walker();
    let mut state = SinkState::new();
    state.install_skeleton(
        cerulion_viz::skeleton::Skeleton::from_urdf_str(SKELETON_URDF).expect("active skeleton"),
    );

    // The frame_id is `utlidar_lidar` — a KNOWN alias that resolves on its own to
    // the TF-tree lidar mount. The configured frame must WIN (configuration beats
    // data), which is the second thing nothing pinned.
    dispatch(
        &rec,
        &walker,
        "/utlidar/cloud",
        &[build_cloud_frame("utlidar_lidar", 1_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let entity = entity_of("/utlidar/cloud");
    assert_eq!(
        entity, "world/utlidar/cloud",
        "the topic KEEPS its own entity"
    );
    let frames = coordinate_frames(&storage);
    let radar = "tf#/world/tf-tree/robot/radar".to_string();
    assert!(
        frames.contains(&(entity.clone(), radar.clone())),
        "the cloud is posed in the URDF radar frame: {frames:?}"
    );
    assert!(
        frames.contains(&(format!("{entity}/viz-sweep/0"), radar.clone())),
        "the sweep sub-entity carrying the POINTS is posed there too: {frames:?}"
    );
    // The data-derived answer must NOT appear anywhere — configuration beats data.
    let data_frame = "tf#/world/tf-tree/odom/base/lidar".to_string();
    assert!(
        !frames.iter().any(|(_, f)| *f == data_frame),
        "the configured frame must beat the resolvable data frame: {frames:?}"
    );
}

/// The paired CONTROL: with NO skeleton the same topic + same frame_id resolves
/// through the data path. Without this arm the test above could pass on a build
/// where `reparent_cloud_route` never fires and the radar frame came from
/// somewhere else.
#[test]
fn without_a_skeleton_the_same_cloud_resolves_through_its_data_frame() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("configured_frame_control");
    let walker = walker();
    let mut state = SinkState::new();

    dispatch(
        &rec,
        &walker,
        "/utlidar/cloud",
        &[build_cloud_frame("utlidar_lidar", 1_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let entity = entity_of("/utlidar/cloud");
    let frames = coordinate_frames(&storage);
    assert!(
        frames.contains(&(entity, "tf#/world/tf-tree/odom/base/lidar".to_string())),
        "an inert skeleton leaves the data path in charge: {frames:?}"
    );
    assert!(
        !frames
            .iter()
            .any(|(_, f)| f == "tf#/world/tf-tree/robot/radar"),
        "no skeleton, no radar frame: {frames:?}"
    );
}

// ── 12. an UNRESOLVABLE frame CLEARS a live assignment ──────────────────────

/// **The withhold rule must hold on EVERY message, not just the first.**
///
/// `log_coordinate_frame` is deliberately TEMPORAL, so rerun's latest-at keeps the
/// last assignment live indefinitely. A `None` resolution used to be a pure no-op,
/// so a topic that was posed once and then started stamping a frame the tree
/// cannot place kept rendering rigidly attached to the STALE mount — precisely the
/// fabricated pose `FrameRegistry::resolve`'s doc rejects — while the warn told the
/// operator its data was "rendered UNPOSED at the world origin". Now a `None`
/// resolution re-points the entity at its OWN implicit frame.
#[test]
#[tracing_test::traced_test]
fn an_unresolvable_frame_un_poses_a_topic_that_was_posed_before() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("clear_stale");
    let walker = walker();
    let mut state = SinkState::new();

    // Posed (a known alias), then the driver is reconfigured and starts stamping a
    // frame nothing can place.
    dispatch(
        &rec,
        &walker,
        "/velodyne/points",
        &[
            build_cloud_frame("livox_frame", 1_000),
            build_cloud_frame("velodyne_link", 2_000),
        ],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let entity = entity_of("/velodyne/points");
    let frames = coordinate_frames(&storage);
    // Hand oracle: posed, then put BACK to the entity's own implicit frame.
    assert_eq!(
        frames
            .iter()
            .filter(|(e, _)| *e == entity)
            .cloned()
            .collect::<Vec<_>>(),
        vec![
            (
                entity.clone(),
                "tf#/world/tf-tree/odom/base/lidar".to_string()
            ),
            (entity.clone(), format!("tf#/{entity}")),
        ],
        "the stale mount must be cleared, not left in place: {frames:?}"
    );
    // The warn is the operator's only signal, and it now tells the truth.
    assert!(logs_contain("the transform tree cannot place"));
}

/// The ABSENT-frame_id sub-case, which was worse: `frame_id_of(fv)?` returned
/// BEFORE the warn, so a driver that stops stamping a frame on degraded frames
/// left a stale pose with no signal at all. It must clear too.
#[test]
fn a_message_that_stops_carrying_a_frame_id_un_poses_its_topic() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("clear_absent");
    let walker = walker();
    let mut state = SinkState::new();

    dispatch(
        &rec,
        &walker,
        "/velodyne/points",
        &[
            build_cloud_frame("livox_frame", 1_000),
            // An EMPTY frame_id reads as ABSENT (ROS treats "" as "no frame").
            build_cloud_frame("", 2_000),
        ],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let entity = entity_of("/velodyne/points");
    let frames = coordinate_frames(&storage);
    assert_eq!(
        frames
            .iter()
            .filter(|(e, _)| *e == entity)
            .cloned()
            .collect::<Vec<_>>(),
        vec![
            (
                entity.clone(),
                "tf#/world/tf-tree/odom/base/lidar".to_string()
            ),
            (entity.clone(), format!("tf#/{entity}")),
        ],
        "an absent frame_id un-poses too: {frames:?}"
    );
    // …and a topic that was NEVER posed still emits nothing (no phantom row).
    let mut fresh = SinkState::new();
    let (rec2, storage2) = memory_sink("clear_absent_control");
    dispatch(
        &rec2,
        &walker,
        "/cmd_vel",
        &[common::build_twist([1.0, 0.0, 0.0], [0.0, 0.0, 0.5])],
        &mut fresh,
    );
    rec2.flush_blocking().expect("flush");
    assert!(
        coordinate_frames(&storage2).is_empty(),
        "a never-posed entity must not get a phantom clear"
    );
}

// ── 13. the `<entity>/viz-vertices` sub-entity assignment ──────────────────

/// A `geometry_msgs/Pose` fixed section: `Point{x,y,z}` then
/// `Quaternion{x,y,z,w}`, seven f64 LE = 56 bytes.
fn pose_fixed_section(pos: [f64; 3], quat: [f64; 4]) -> Vec<u8> {
    let mut v = Vec::with_capacity(56);
    for c in pos.iter().chain(quat.iter()) {
        v.extend_from_slice(&c.to_le_bytes());
    }
    v
}

/// One `geometry_msgs/PoseStamped` element body (crib: `sink_dispatch_test`).
fn pose_stamped_body(pos: [f64; 3], frame_id: &str) -> Vec<u8> {
    let l = layout_of("geometry_msgs/PoseStamped");
    assert_eq!(
        (l.fixed_size, l.offset_table_bytes()),
        (56, 8),
        "PoseStamped shape drifted — update this builder"
    );
    let hdr = header_body(frame_id);
    let mut v = pose_fixed_section(pos, [0.0, 0.0, 0.0, 1.0]);
    v.extend_from_slice(&[0u8; 8]); // entry[0] placeholder
    write_offset_entry(&mut v, 56, 0, 64, hdr.len() as u32);
    v.extend_from_slice(&hdr);
    v
}

/// The counted element framing: `u32 count` + per element `u32 len` + body.
fn counted_blob(elements: &[Vec<u8>]) -> Vec<u8> {
    let mut v = Vec::new();
    v.extend_from_slice(&(elements.len() as u32).to_le_bytes());
    for e in elements {
        v.extend_from_slice(&(e.len() as u32).to_le_bytes());
        v.extend_from_slice(e);
    }
    v
}

/// A `nav_msgs/Path` frame with a POPULATED top-level `header.frame_id` — the
/// nav2 `/plan` shape. (The sibling builder in `sink_dispatch_test` leaves the
/// header EMPTY, which is why that file's Path tests are frame-blind.)
fn build_path_frame(frame_id: &str, timestamp_ns: u64) -> Vec<u8> {
    let l = layout_of("nav_msgs/Path");
    assert_eq!(
        l.variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["header", "poses"],
        "nav_msgs/Path variable-field declaration order changed — update this builder"
    );
    assert_eq!(l.fixed_size, 0, "nav_msgs/Path gained a fixed section");
    let hdr = header_body(frame_id);
    let blob = counted_blob(&[
        pose_stamped_body([1.0, 2.0, 3.0], frame_id),
        pose_stamped_body([4.0, 5.0, 6.0], frame_id),
    ]);
    let table = l.offset_table_bytes();
    let mut payload = vec![0u8; table];
    write_offset_entry(&mut payload, 0, 0, table as u32, hdr.len() as u32);
    write_offset_entry(
        &mut payload,
        0,
        1,
        (table + hdr.len()) as u32,
        blob.len() as u32,
    );
    payload.extend_from_slice(&hdr);
    payload.extend_from_slice(&blob);
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: <native_ros2_messages::nav_msgs::Path as ShmMessage>::SCHEMA_HASH,
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

/// **The Path3D twin of the `sweep/{k}` assignment.**
///
/// There are exactly two sub-entity frame assignments. The cloud's `sweep/{k}`
/// one is pinned by `a_cloud_with_a_known_frame_is_posed_at_its_own_entity`;
/// the path's `<entity>/viz-vertices` one was not, and deleting it left the suite
/// green — every existing Path test builds its frame with an EMPTY header, so
/// NEITHER assignment fires and the mutation is invisible to all of them.
///
/// A child's implicit frame is derived from its PATH, so it chains to the
/// parent's path frame, NOT to the frame the parent was re-pointed at: without
/// the repeat, a nav2 `/plan` draws its polyline posed while its waypoints stay
/// at the world origin — they detach from their own line the moment the robot
/// moves.
#[test]
fn a_path_poses_its_vertices_child_as_well_as_its_polyline() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("path_vertices");
    let walker = walker();
    let mut state = SinkState::new();

    // `odom` is a known alias, so this resolves with no /tf at all.
    dispatch(
        &rec,
        &walker,
        "/plan",
        &[build_path_frame("odom", 1_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let entity = entity_of("/plan");
    assert_eq!(entity, "world/plan");
    let frames = coordinate_frames(&storage);
    let odom = "tf#/world/tf-tree/odom".to_string();
    assert!(
        frames.contains(&(entity.clone(), odom.clone())),
        "the polyline's entity is posed: {frames:?}"
    );
    assert!(
        frames.contains(&(format!("{entity}/viz-vertices"), odom)),
        "the waypoints child is posed too, or it floats beside its own line: {frames:?}"
    );
}

// ── 14. a tf-NAMED topic carrying a non-TFMessage schema ────────────────────

/// **The SCHEMA gets a vote on the tf NAME arm.**
///
/// `route_for_input` answers the bare viz root `world` for any topic whose last
/// segment is `tf`/`tf_static`, and that is only correct for a real TFMessage —
/// `dispatch_transforms` ignores `route.entity` entirely. Every OTHER archetype
/// arm renders AT `route.entity`, and `assign_coordinate_frame` runs before all
/// of them, so a `/robot1/tf` carrying something else wrote its geometry AND a
/// `CoordinateFrame` at the scene ROOT, whose transform composes onto the entire
/// recording. `entity_path_for_route_key` is explicitly guarded so "a
/// degenerate topic can never claim the TF tree's root entity"; the tf arm was
/// the one remaining path that could.
#[test]
#[tracing_test::traced_test]
fn a_tf_named_topic_with_a_non_tf_schema_never_logs_at_the_viz_root() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("tf_named_wrong_schema");
    let walker = walker();
    let mut state = SinkState::new();

    // A `/robot1/tf` carrying `nav_msgs/Odometry` (frame_id `odom`, a resolvable
    // alias — so a schema-blind tf arm would re-point `world` itself).
    dispatch(
        &rec,
        &walker,
        "/robot1/tf",
        &[build_odometry_frame("odom", [1.0, 2.0, 3.0], 1_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let all = chunks(&storage);
    // No chunk of ANY kind lands on the bare root — logging a transform there
    // would re-pose the entire recording.
    for chunk in &all {
        assert_ne!(
            entity_path_string(chunk),
            "world",
            "a non-TFMessage frame must never log at the scene root"
        );
    }
    // It renders at its own mechanical entity instead, posed normally — and
    // because an Odometry payload IS a transform, that posing rides the
    // transform's `parent_frame`.
    let entities: Vec<String> = all.iter().map(entity_path_string).collect();
    assert!(
        entities.contains(&"world/robot1/tf".to_string()),
        "the topic falls back to its mechanical entity: {entities:?}"
    );
    let parents: Vec<String> = all
        .iter()
        .filter(|c| entity_path_string(c) == "world/robot1/tf")
        .flat_map(|c| {
            c.components()
                .iter()
                .filter(|(d, _)| d.as_str() == "Transform3D:parent_frame")
                .flat_map(|(_, l)| frame_id_column(l.list_array.values().as_ref()))
                .collect::<Vec<_>>()
        })
        .collect();
    assert_eq!(
        parents,
        vec!["tf#/world/tf-tree/odom".to_string()],
        "posed at its own entity through the transform's parent_frame"
    );
    assert!(logs_contain("does not carry tf2_msgs/TFMessage"));
}

/// The paired CONTROL: a REAL TFMessage on the same tf-named topic keeps the
/// root route (its transforms are logged at their own child-frame entities, so
/// `route.entity` is never a render target) and does NOT fire the warn.
#[test]
#[tracing_test::traced_test]
fn a_real_tf_message_on_a_namespaced_tf_topic_keeps_the_root_route() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("tf_named_right_schema");
    let walker = walker();
    let mut state = SinkState::new();

    dispatch(
        &rec,
        &walker,
        "/robot1/tf",
        &[build_tf_frame("base", "wrist_mount", 1_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let entities: Vec<String> = chunks(&storage).iter().map(entity_path_string).collect();
    assert!(
        entities.contains(&"world/tf-tree/odom/base/wrist_mount".to_string()),
        "a real TFMessage logs at its child-frame entity: {entities:?}"
    );
    assert!(
        !entities.contains(&"world/robot1/tf".to_string()),
        "a TFMessage never renders at its route entity: {entities:?}"
    );
    logs_assert(|lines: &[&str]| {
        if lines
            .iter()
            .any(|l| l.contains("does not carry tf2_msgs/TFMessage"))
        {
            Err("a real TFMessage must not fire the schema-mismatch warn".to_string())
        } else {
            Ok(())
        }
    });
}

// ── 15. the ROBOT ROOT: posed WITHOUT tearing the skeleton ──────────────────

/// **The skeleton-integrity pin.**
///
/// The Odometry arm writes the same payload pose to a SECOND entity, the skeleton
/// root, which sits at the head of the URDF link chain. The first fix posed it
/// with a `CoordinateFrame`, which is doubly wrong there:
///
/// - **inert for the chain** — `re_tf` resolves a frame's parent from
///   `Transform3D:parent_frame` (null ⇒ the entity's PATH parent) and never reads
///   `CoordinateFrame:frame`, so every leg link kept composing exactly as before;
/// - **actively tearing** — the root's OWN geometry (its base-link marker and
///   bones) DID move into the odom frame, so on the shipped Go2 demo path the base
///   sat at the world origin while the legs hung at the odom pose.
///
/// Asserts COMPOSITION rather than presence: the root's transform must carry the
/// resolved frame as its `parent_frame`, a leg link must carry NO explicit parent
/// (so it chains to its PATH parent — the root — i.e. through the SAME chain), and
/// nothing in the robot subtree may get a `CoordinateFrame`.
#[test]
fn an_odom_elected_topic_poses_the_robot_root_without_tearing_the_skeleton() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("robot_root_frame");
    let walker = walker();
    let mut state = SinkState::new();
    // A loaded URDF, so the skeleton's link chain is really in the recording.
    //
    // `log_statics_once` is guarded by a PROCESS-global `AtomicBool` that is never
    // re-armed, so this test would otherwise be correct only while it happens to be
    // the first thing in the binary to trip it — the moment a sibling dispatches a
    // `LowState`, the statics no-op, nothing lands at the radar link, and assertion
    // (3) fails intermittently on libtest's nondeterministic order. Re-arming makes
    // it order-INDEPENDENT; `rerun_lock` (held above) provides the serialization the
    // guard's own doc requires for tests that reset it.
    cerulion_viz::skeleton::rearm_skeleton_statics();
    state.install_skeleton(
        cerulion_viz::skeleton::Skeleton::from_urdf_str(SKELETON_URDF).expect("active skeleton"),
    );
    state.skeleton_log_statics_for_test(&rec);

    dispatch(
        &rec,
        &walker,
        "/uslam/frontend/odom",
        &[build_odometry_frame("odom", [1.0, 2.0, 3.0], 1_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let l = logged(&storage);
    let odom = Some("tf#/world/tf-tree/odom".to_string());
    // (1) the topic's own entity is posed in the odom frame…
    assert_eq!(
        l.parent_frame_of("world/uslam/frontend/odom"),
        Some(odom.clone()),
        "{l:?}"
    );
    // (2) …and so is the ROBOT ROOT, through the same component.
    assert_eq!(
        l.parent_frame_of("world/tf-tree/robot"),
        Some(odom),
        "the robot root must be posed by its transform's parent_frame: {l:?}"
    );
    // (3) THE INTEGRITY ASSERT: the child link carries NO explicit parent, so it
    // chains to its PATH parent — the root — and therefore through the SAME frame
    // the root was just re-parented into. Base and legs move together.
    assert_eq!(
        l.parent_frame_of("world/tf-tree/robot/radar"),
        Some(None),
        "a skeleton link must stay path-parented to the root: {l:?}"
    );
    // (4) …and nothing in the robot subtree gets a CoordinateFrame, which is what
    // detached the root's own geometry from its own chain.
    assert!(
        !l.coordinate_framed_entities()
            .iter()
            .any(|e| e.starts_with("world/tf-tree/robot")),
        "no CoordinateFrame anywhere in the skeleton subtree: {l:?}"
    );
}

/// …and the WITHHOLD half: an unresolvable frame poses neither entity, and pins
/// BOTH transforms back to their path parents rather than fabricating a mount.
#[test]
fn an_unresolvable_odom_frame_leaves_the_robot_root_unposed() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("robot_root_withhold");
    let walker = walker();
    let mut state = SinkState::new();

    dispatch(
        &rec,
        &walker,
        "/uslam/frontend/odom",
        &[build_odometry_frame("map", [1.0, 2.0, 3.0], 1_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let l = logged(&storage);
    assert!(
        l.coordinate_frames.is_empty(),
        "an unplaceable frame fabricates nothing, at either entity: {l:?}"
    );
    // The pose itself still renders (unposed at the world origin) — withholding
    // is about the FRAME, never about dropping the data — with the path parent
    // named EXPLICITLY so no earlier assignment can linger under latest-at.
    assert_eq!(
        l.parent_frame_of("world/tf-tree/robot"),
        Some(Some("tf#/world/tf-tree".to_string())),
        "{l:?}"
    );
    assert_eq!(
        l.parent_frame_of("world/uslam/frontend/odom"),
        Some(Some("tf#/world/uslam/frontend".to_string())),
        "{l:?}"
    );
}

/// **A SECOND robot-root elector is LOUD.**
///
/// `world/tf-tree/robot` is ONE entity, and the Go2 ships FOUR odom-named topics
/// (`/uslam/frontend/odom`, `/uslam/localization/odom`,
/// `/lio_sam_ros2/mapping/odometry`, `/utlidar/robot_odom`). Attaching two makes
/// the skeleton snap between their localization estimates every frame — the same
/// silent wrong spatial answer per-topic entities fixed one level down, and
/// fixing the leaf makes the scene look RIGHT, which makes the surviving root
/// fight more deceptive. Arbitrating it would mean guessing which estimate the
/// operator trusts, so it stays a fight — but a loud one.
#[test]
#[tracing_test::traced_test]
fn a_second_odom_topic_electing_the_robot_root_warns_exactly_once() {
    let _g = rerun_lock();
    let (rec, _storage) = memory_sink("robot_root_fight");
    let walker = walker();
    let mut state = SinkState::new();

    // ONE elector: silent (the intended single-source configuration).
    dispatch(
        &rec,
        &walker,
        "/uslam/frontend/odom",
        &[build_odometry_frame("odom", [1.0, 0.0, 0.0], 1_000)],
        &mut state,
    );
    logs_assert(|lines: &[&str]| {
        if lines.iter().any(|l| l.contains("posing the robot root")) {
            Err("a single elector must not warn".to_string())
        } else {
            Ok(())
        }
    });

    // A SECOND one: loud, ONCE, naming both.
    for i in 0..4u64 {
        dispatch(
            &rec,
            &walker,
            "/uslam/localization/odom",
            &[build_odometry_frame("odom", [2.0, 0.0, 0.0], 2_000 + i)],
            &mut state,
        );
    }
    rec.flush_blocking().expect("flush");
    logs_assert(|lines: &[&str]| {
        let hits: Vec<&&str> = lines
            .iter()
            .filter(|l| l.contains("posing the robot root"))
            .collect();
        if hits.len() != 1 {
            return Err(format!("expected exactly one warn, got {}", hits.len()));
        }
        let l = hits[0];
        if !l.contains("uslam/frontend/odom") || !l.contains("uslam/localization/odom") {
            return Err(format!("the warn must name BOTH electors: {l}"));
        }
        Ok(())
    });
}

// ── 16. the REPORTED entity must be where the sink actually renders ─────────

/// **`reconcile_tf_route` must not diverge from what the daemon reports.**
///
/// `route_for_input` answers the bare viz root for a `tf`/`tf_static`-named topic,
/// and the daemon reports that verbatim. Once a frame proves the topic is NOT a
/// TFMessage, the sink moves it to its own entity — so `discover`/`list`/`attach`
/// were naming a location nothing is ever logged at, and an `entity` override fed
/// back from such a report could not round-trip. Both sides now derive through
/// `reported_entity_for`.
#[test]
fn the_reported_entity_agrees_with_where_a_tf_named_non_tf_topic_renders() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("reported_entity_agrees");
    let walker = walker();
    let mut state = SinkState::new();

    let key = route_key_for_topic("/robot1/tf", None);
    // Before any frame resolves, the name-derived answer stands.
    assert_eq!(reported_entity_for(&key, None), "world");
    // Once it is known to be an Odometry, the report moves with the sink.
    let reported = reported_entity_for(&key, Some(ArchetypeKind::Odometry));
    assert_eq!(reported, "world/robot1/tf");

    dispatch(
        &rec,
        &walker,
        "/robot1/tf",
        &[build_odometry_frame("odom", [1.0, 2.0, 3.0], 1_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    // THE AGREEMENT: the reported entity is exactly where a chunk landed.
    let entities: Vec<String> = chunks(&storage).iter().map(entity_path_string).collect();
    assert!(
        entities.contains(&reported),
        "the daemon reports {reported} but the sink logged at {entities:?}"
    );
    // A real TFMessage keeps the root answer on both sides.
    assert_eq!(
        reported_entity_for(&key, Some(ArchetypeKind::Transforms)),
        "world"
    );
}

/// …and the ENTITY OVERRIDE survives the reconcile: it names the entity, so a
/// tf-named non-TFMessage topic attached with one renders THERE, not at the
/// mechanical path — and the reported entity says so.
#[test]
fn a_tf_named_topic_with_an_override_renders_at_the_overridden_entity() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("reported_entity_override");
    let walker = walker();
    let mut state = SinkState::new();

    let key = route_key_for_topic("/robot1/tf", Some("world/cam"));
    let reported = reported_entity_for(&key, Some(ArchetypeKind::Odometry));
    assert_eq!(reported, "world/cam", "the override names the entity");

    let frame = build_odometry_frame("odom", [1.0, 2.0, 3.0], 1_000);
    dispatch_frame(&rec, &walker, &key, &frame, &mut state);
    rec.flush_blocking().expect("flush");

    let entities: Vec<String> = chunks(&storage).iter().map(entity_path_string).collect();
    assert!(
        entities.contains(&"world/cam".to_string()),
        "the override must survive the tf-schema reconcile: {entities:?}"
    );
    assert!(
        !entities.contains(&"world/robot1/tf".to_string()),
        "{entities:?}"
    );
}

// ── 17. the FrameRegistry saturation warn ──────────────────────────────────

/// The operator's ONLY signal that frame learning has stopped — previously
/// untested, so nothing caught a warn-on-every-transform variant of its
/// once-per-run latch (which would flood a `/tf` stream at lidar rate).
#[test]
#[tracing_test::traced_test]
fn the_frame_registry_saturation_warn_fires_exactly_once() {
    let mut reg = FrameRegistry::new();
    for i in 0..cerulion_viz::tf::MAX_OBSERVED_FRAMES + 50 {
        reg.observe_child(&format!("tag_36h11_{i}"));
    }
    assert_eq!(reg.observed_len(), cerulion_viz::tf::MAX_OBSERVED_FRAMES);
    logs_assert(|lines: &[&str]| {
        let hits = lines
            .iter()
            .filter(|l| l.contains("no FURTHER frame will be learned"))
            .count();
        if hits != 1 {
            return Err(format!("expected exactly one saturation warn, got {hits}"));
        }
        Ok(())
    });
}

/// The anti-tautology control: a registry BELOW the cap warns about nothing.
#[test]
#[tracing_test::traced_test]
fn an_unsaturated_frame_registry_never_warns() {
    let mut reg = FrameRegistry::new();
    for i in 0..64 {
        reg.observe_child(&format!("link_{i}"));
    }
    logs_assert(|lines: &[&str]| {
        if lines.iter().any(|l| l.contains("no FURTHER frame")) {
            Err("an unsaturated registry must not warn".to_string())
        } else {
            Ok(())
        }
    });
}

// ── 18. the OTHER `payload_is_transform` arms ──────────────────────────────

/// A `sensor_msgs/Imu` frame with a populated `header.frame_id`. Fixed 296 B
/// (quaternion + 3 covariance matrices + 2 vectors) then entry[0] `header`.
fn build_imu_frame(frame_id: &str, timestamp_ns: u64) -> Vec<u8> {
    let l = layout_of("sensor_msgs/Imu");
    assert_eq!(
        (l.fixed_size, l.offset_table_bytes()),
        (296, 8),
        "Imu shape drifted — update this builder"
    );
    let hdr = header_body(frame_id);
    let mut payload = vec![0u8; 296];
    // `orientation` is the leading `geometry_msgs/Quaternion` (x,y,z,w f64) — a
    // unit quaternion so the rotation is real, not a degenerate zero.
    for (i, c) in [0.0f64, 0.0, 0.0, 1.0].iter().enumerate() {
        payload[i * 8..i * 8 + 8].copy_from_slice(&c.to_le_bytes());
    }
    payload.extend_from_slice(&[0u8; 8]); // entry[0] placeholder
    write_offset_entry(&mut payload, 296, 0, 304, hdr.len() as u32);
    payload.extend_from_slice(&hdr);
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: <native_ros2_messages::sensor_msgs::Imu as ShmMessage>::SCHEMA_HASH,
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

/// A top-level `geometry_msgs/PoseStamped` frame (fixed `Pose` 56 B then
/// entry[0] `header`) — the same body shape `pose_stamped_body` builds as a
/// nav_msgs/Path element, promoted to a whole frame.
fn build_pose_stamped_frame(frame_id: &str, pos: [f64; 3], timestamp_ns: u64) -> Vec<u8> {
    let l = layout_of("geometry_msgs/PoseStamped");
    assert_eq!(
        (l.fixed_size, l.offset_table_bytes()),
        (56, 8),
        "PoseStamped shape drifted — update this builder"
    );
    let payload = pose_stamped_body(pos, frame_id);
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: <native_ros2_messages::geometry_msgs::PoseStamped as ShmMessage>::SCHEMA_HASH,
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

/// **The `Imu` arm of `payload_is_transform`.**
///
/// An IMU's attitude IS a `Transform3D`, so it must be posed by `parent_frame`.
/// Every composition arm at that point drove `build_odometry_frame`, so dropping the
/// `parent_frame` argument on JUST the `ArchetypeKind::Imu` render arm left the
/// whole suite green while every `sensor_msgs/Imu` topic — a Go2
/// publishes three — silently lost its pose (no `parent_frame` AND no
/// `CoordinateFrame`, since the else branch is not taken either).
#[test]
fn an_imu_payload_transform_is_posed_by_its_parent_frame() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("imu_parent_frame");
    let walker = walker();
    let mut state = SinkState::new();

    dispatch(
        &rec,
        &walker,
        "/utlidar/imu",
        &[build_imu_frame("odom", 1_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let entity = entity_of("/utlidar/imu");
    let l = logged(&storage);
    assert_eq!(
        l.parent_frame_of(&entity),
        Some(Some("tf#/world/tf-tree/odom".to_string())),
        "the IMU attitude transform must carry the resolved parent frame: {l:?}"
    );
    assert!(
        !l.coordinate_framed_entities().contains(&entity),
        "a transform payload must not also get a CoordinateFrame: {l:?}"
    );
}

/// **The `Transform3D` / `Transform3DWithScalars` arm (Pose / PoseStamped).**
///
/// Dispatched by the existing suite, but only ever asserted by CHUNK COUNT — and
/// the count is provably invariant to the mutation, since `log_transform3d_in_frame`
/// issues exactly one `rec.log` whether or not `.with_parent_frame` is applied. This
/// asserts the component.
#[test]
fn a_pose_stamped_payload_transform_is_posed_by_its_parent_frame() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("pose_parent_frame");
    let walker = walker();
    let mut state = SinkState::new();

    dispatch(
        &rec,
        &walker,
        "/goal_pose",
        &[build_pose_stamped_frame("odom", [1.0, 2.0, 3.0], 1_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let entity = entity_of("/goal_pose");
    let l = logged(&storage);
    assert_eq!(
        l.parent_frame_of(&entity),
        Some(Some("tf#/world/tf-tree/odom".to_string())),
        "the pose transform must carry the resolved parent frame: {l:?}"
    );
    assert!(
        !l.coordinate_framed_entities().contains(&entity),
        "a transform payload must not also get a CoordinateFrame: {l:?}"
    );
}

/// The paired DATA-payload control, so the two arms above cannot pass by the
/// switch having become unconditional: a cloud is posed by a `CoordinateFrame`
/// and its transform slot stays untouched.
#[test]
fn a_data_payload_is_still_posed_by_a_coordinate_frame_not_a_parent_frame() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("data_payload_control");
    let walker = walker();
    let mut state = SinkState::new();

    dispatch(
        &rec,
        &walker,
        "/velodyne/points",
        &[build_cloud_frame("odom", 1_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let entity = entity_of("/velodyne/points");
    let l = logged(&storage);
    assert!(
        l.coordinate_frames
            .contains(&(entity.clone(), "tf#/world/tf-tree/odom".to_string())),
        "a DATA payload is posed by a CoordinateFrame: {l:?}"
    );
    assert_eq!(
        l.parent_frame_of(&entity),
        None,
        "a data payload logs no transform at its own entity: {l:?}"
    );
}

// ── 19. the closed-set oracle for `payload_is_transform` ───────────────────

/// **`payload_is_transform` is the WHOLE posing switch: pin it as a CLOSED SET.**
///
/// It decides `parent_frame` vs `CoordinateFrame` for every archetype, and its
/// `matches!` has an implicit `false` catch-all — so a NEW variant that logs a
/// `Transform3D` at its route entity silently defaults to the original defect. The
/// in-file precedent for exactly this hazard is `sink::tests::coalesces_exact_set_oracle`,
/// whose own comment records that "the earlier two-array form silently ignored new
/// variants".
///
/// The oracle is hand-written, and it is CROSS-CHECKED against the other table
/// that names Transform3D-logging kinds — `blueprint::archetype_components`. The
/// two deliberately disagree on exactly three kinds, and that disagreement is
/// asserted here rather than left in prose: `Transforms` logs at each transform's
/// own CHILD-FRAME entity, `Skeleton` at its per-LINK entities, and the
/// `MarkerArray` at its per-MARKER `<entity>/viz-markers/<ns>/<id>` entities — so
/// none of them renders a transform at `route.entity` and none may take the
/// parent-frame branch. (Each still poses its own child entities through those
/// children's `Transform3D:parent_frame`; what they must NOT do is claim the
/// ROUTE entity's posing decision.)
#[test]
fn payload_is_transform_is_a_closed_set_cross_checked_against_archetype_components() {
    use cerulion_viz::blueprint::archetype_components;

    // HAND oracle — written out in `ArchetypeKind::ALL` declaration order, never
    // derived from the function under test.
    let expected: &[ArchetypeKind] = &[
        ArchetypeKind::Transform3D,
        ArchetypeKind::Transform3DWithScalars,
        ArchetypeKind::Imu,
        ArchetypeKind::Odometry,
    ];
    let actual: Vec<ArchetypeKind> = ArchetypeKind::ALL
        .into_iter()
        .filter(|k| k.payload_is_transform())
        .collect();
    assert_eq!(
        actual, expected,
        "payload_is_transform's members changed — a kind that logs a Transform3D at \
         its route entity MUST be here, or it silently gets a CoordinateFrame instead"
    );

    // Every kind is classified deliberately: the parent-frame branch is exactly the
    // Transform3D-logging kinds MINUS the two that log somewhere else.
    let logs_transform3d = |k: ArchetypeKind| archetype_components(k).contains(&"Transform3D");
    for kind in ArchetypeKind::ALL {
        let excluded = matches!(
            kind,
            ArchetypeKind::Transforms | ArchetypeKind::Skeleton | ArchetypeKind::MarkerArray
        );
        let want = logs_transform3d(kind) && !excluded;
        assert_eq!(
            kind.payload_is_transform(),
            want,
            "{kind:?}: payload_is_transform disagrees with archetype_components"
        );
    }

    // The two exclusions, asserted rather than described: both DO log a Transform3D
    // (so they would be swept in by the cross-check above), and both are excluded.
    for kind in [ArchetypeKind::Transforms, ArchetypeKind::Skeleton] {
        assert!(
            logs_transform3d(kind),
            "{kind:?} must still be a Transform3D-logging kind, or this exclusion is moot"
        );
        assert!(
            !kind.payload_is_transform(),
            "{kind:?} logs its transforms at frame/link entities, never at route.entity"
        );
    }
}

/// The BEHAVIOURAL half of the `Transforms` exclusion: a real TFMessage logs at
/// its child-frame entities and gets NO parent frame at its route entity — which
/// is why it must not take the parent-frame branch.
#[test]
fn a_tf_message_logs_no_transform_at_its_route_entity() {
    let _g = rerun_lock();
    let (rec, storage) = memory_sink("tf_exclusion");
    let walker = walker();
    let mut state = SinkState::new();

    dispatch(
        &rec,
        &walker,
        "/tf",
        &[build_tf_frame("base", "wrist_mount", 1_000)],
        &mut state,
    );
    rec.flush_blocking().expect("flush");

    let l = logged(&storage);
    // Nothing at the route entity (the viz root) at all…
    assert_eq!(l.parent_frame_of("world"), None, "{l:?}");
    assert!(
        !l.coordinate_framed_entities()
            .contains(&"world".to_string()),
        "{l:?}"
    );
    // …the transform landed at the CHILD-FRAME entity, path-parented as the tree
    // composes it.
    assert_eq!(
        l.parent_frame_of("world/tf-tree/odom/base/wrist_mount"),
        Some(None),
        "{l:?}"
    );
}
