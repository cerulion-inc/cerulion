// SPDX-License-Identifier: AGPL-3.0-only
//! The live voxel map codec (`cerulion_viz::voxel_map`): the field-layout
//! probe, the frame decoder over both `fields` framings, the seven ops, the
//! epoch rules, determinism, the static entity tree the viewer receives, the
//! wall geometry, the colour ramp and the trail; then the sink's side of it:
//! the classification rung, the no-coalesce rule, and the PNG `CompressedImage`
//! path a floor plan rides on. Every oracle is hand-written.

use std::collections::{BTreeMap, BTreeSet};

use cerulion_core::codegen::layout::{LayoutResolver, WireLayout};
use cerulion_core::codegen::{parse_rosmsg, FrameValueKind, MessageSchema};
use cerulion_core::message::ShmMessage;
use cerulion_core::shm_runtime::write_offset_entry;
use cerulion_core::wire::WireHeader;
use cerulion_viz::pointcloud::PointFieldDesc;
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::sink::{
    classify_frame, coalesces, dispatch_frame, dispatch_or_stage, route_for_input, ArchetypeKind,
    SinkState,
};
use cerulion_viz::voxel_map::{
    decode_ops, decode_voxel_message, execute, height_rgb, tile_of, tile_segment,
    voxel_delta_layout, voxel_layout_of, wall_cells, wall_geometry, LogAction, VoxelDeltaLayout,
    VoxelMapState, VoxelMessage, VoxelOp, BLUE_RGB, CERULEAN_RGB, EMBER_RGB, FLOOR_RGB, OP_CLEAR,
    OP_END_TILE, OP_FLOOR, OP_RESET, OP_ROBOT, OP_SET, OP_TILE, TRAIL_MAX_POINTS,
};
use native_ros2_messages::sensor_msgs::{CompressedImage, PointCloud2};

// ---- Frame builders ---------------------------------------------------------

fn all_schemas() -> Vec<MessageSchema> {
    let mut schemas = Vec::new();
    for (pkg, name, text) in native_ros2_messages::BUILTIN_MSGS {
        if let Ok(s) = parse_rosmsg(text, name, Some(pkg)) {
            schemas.push(s);
        }
    }
    schemas
}

fn layout_of(qname: &str) -> WireLayout {
    let (mut resolver, _) = LayoutResolver::new(all_schemas());
    resolver.layout_of(qname).expect("built-in schema")
}

fn fixed_off(layout: &WireLayout, name: &str) -> usize {
    layout
        .fixed_fields
        .iter()
        .find(|f| f.name == name)
        .unwrap_or_else(|| panic!("no fixed field '{name}'"))
        .offset
}

/// One `PointField` as `(name, offset, datatype, count)`.
type Field = (&'static str, u32, u8, u32);

/// The exact voxel-delta layout at `edge_mm`.
fn voxel_fields(edge_mm: u16) -> Vec<(String, u32, u8, u32)> {
    vec![
        (format!("vx_{edge_mm}mm"), 0, 3, 1),
        (format!("vy_{edge_mm}mm"), 2, 3, 1),
        (format!("vz_{edge_mm}mm"), 4, 3, 1),
        ("hits".to_string(), 6, 2, 1),
        ("op".to_string(), 7, 2, 1),
    ]
}

/// `fields` in the CANONICAL element framing (`u32 count`, then per element
/// `u32 len` + a headerless `PointField` sub-frame): what the Go2 demo's map
/// node writes. Built on the real layout engine.
fn canonical_fields(fields: &[(String, u32, u8, u32)]) -> Vec<u8> {
    let pf = layout_of("sensor_msgs/PointField");
    let mut blob = (fields.len() as u32).to_le_bytes().to_vec();
    for (name, offset, datatype, count) in fields {
        let head = pf.fixed_size + pf.offset_table_bytes();
        let mut body = vec![0u8; head];
        body[fixed_off(&pf, "offset")..][..4].copy_from_slice(&offset.to_le_bytes());
        body[fixed_off(&pf, "datatype")] = *datatype;
        body[fixed_off(&pf, "count")..][..4].copy_from_slice(&count.to_le_bytes());
        write_offset_entry(&mut body, pf.fixed_size, 0, head as u32, name.len() as u32);
        body.extend_from_slice(name.as_bytes());
        blob.extend_from_slice(&(body.len() as u32).to_le_bytes());
        blob.extend_from_slice(&body);
    }
    blob
}

/// `fields` in the PACKED layout (`name_len`, name, `offset`, `datatype`,
/// `count`, back to back): the bridge's typed cloud route.
fn packed_fields(fields: &[(String, u32, u8, u32)]) -> Vec<u8> {
    let mut blob = Vec::new();
    for (name, offset, datatype, count) in fields {
        blob.extend_from_slice(&(name.len() as u32).to_le_bytes());
        blob.extend_from_slice(name.as_bytes());
        blob.extend_from_slice(&offset.to_le_bytes());
        blob.push(*datatype);
        blob.extend_from_slice(&count.to_le_bytes());
    }
    blob
}

/// A `std_msgs/Header` sub-frame carrying `frame_id` (stamp zero).
fn header_blob(frame_id: &str) -> Vec<u8> {
    let h = layout_of("std_msgs/Header");
    let head = h.fixed_size + h.offset_table_bytes();
    let mut body = vec![0u8; head];
    write_offset_entry(
        &mut body,
        h.fixed_size,
        0,
        head as u32,
        frame_id.len() as u32,
    );
    body.extend_from_slice(frame_id.as_bytes());
    body
}

/// One op as its 8 wire bytes (little-endian).
fn op(x: i16, y: i16, z: i16, hits: u8, code: u8) -> [u8; 8] {
    let mut b = [0u8; 8];
    b[0..2].copy_from_slice(&x.to_le_bytes());
    b[2..4].copy_from_slice(&y.to_le_bytes());
    b[4..6].copy_from_slice(&z.to_le_bytes());
    b[6] = hits;
    b[7] = code;
    b
}

fn set(x: i16, y: i16, z: i16) -> [u8; 8] {
    op(x, y, z, 4, OP_SET)
}

fn floor(epoch: u16, floor_iz: i16) -> [u8; 8] {
    op(epoch as i16, 0, floor_iz, 0, OP_FLOOR)
}

fn robot(x: i16, y: i16) -> [u8; 8] {
    op(x, y, 6, 0, OP_ROBOT)
}

/// A whole `sensor_msgs/PointCloud2` wire frame: `header.frame_id` = `frame_id`,
/// `fields` as given, `point_step`, and the op bytes as `data`.
fn cloud_frame(
    fields_blob: &[u8],
    point_step: u32,
    ops: &[[u8; 8]],
    frame_id: &str,
    timestamp_ns: u64,
) -> Vec<u8> {
    let layout = layout_of("sensor_msgs/PointCloud2");
    assert_eq!(
        layout
            .variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["header", "fields", "data"],
        "PointCloud2 variable-field order changed; update this builder"
    );
    let fixed = layout.fixed_size;
    let table = layout.offset_table_bytes();
    let data: Vec<u8> = ops.iter().flatten().copied().collect();
    let header = header_blob(frame_id);
    let mut payload = vec![0u8; fixed + table];
    let put = |buf: &mut [u8], name: &str, v: u32| {
        let off = fixed_off(&layout, name);
        buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
    };
    put(&mut payload, "height", 1);
    put(&mut payload, "width", ops.len() as u32);
    put(&mut payload, "point_step", point_step);
    put(&mut payload, "row_step", point_step * ops.len() as u32);
    payload[fixed_off(&layout, "is_dense")] = 1;
    let header_off = (fixed + table) as u32;
    let fields_off = header_off + header.len() as u32;
    let data_off = fields_off + fields_blob.len() as u32;
    write_offset_entry(&mut payload, fixed, 0, header_off, header.len() as u32);
    write_offset_entry(&mut payload, fixed, 1, fields_off, fields_blob.len() as u32);
    write_offset_entry(&mut payload, fixed, 2, data_off, data.len() as u32);
    payload.extend_from_slice(&header);
    payload.extend_from_slice(fields_blob);
    payload.extend_from_slice(&data);
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: <PointCloud2 as ShmMessage>::SCHEMA_HASH,
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

/// A voxel-delta frame at 5 cm in the `odom` frame, canonical fields.
fn voxel_frame(ops: &[[u8; 8]], timestamp_ns: u64) -> Vec<u8> {
    cloud_frame(
        &canonical_fields(&voxel_fields(50)),
        8,
        ops,
        "odom",
        timestamp_ns,
    )
}

fn message(ops: &[[u8; 8]]) -> VoxelMessage {
    let data: Vec<u8> = ops.iter().flatten().copied().collect();
    VoxelMessage {
        edge_mm: 50,
        ops: decode_ops(&data, ops.len(), false),
        trailing_bytes: 0,
    }
}

const TOPIC: &str = "/go2/map_view/voxels";
const ROOT: &str = "world/go2/map_view/voxels";
const SECOND: u64 = 1_000_000_000;

fn memory() -> (rerun::RecordingStream, rerun::sink::MemorySinkStorage) {
    rerun::RecordingStreamBuilder::new("voxel_map_test")
        .memory()
        .expect("memory sink")
}

/// Every non-reserved chunk the sink emitted.
fn chunks(
    rec: &rerun::RecordingStream,
    storage: &rerun::sink::MemorySinkStorage,
) -> Vec<rerun::log::Chunk> {
    rec.flush_blocking().expect("flush");
    storage
        .take()
        .into_iter()
        .filter_map(|msg| match msg {
            rerun::log::LogMsg::ArrowMsg(_, arrow) => {
                Some(rerun::log::Chunk::from_arrow_msg(&arrow).expect("decode chunk"))
            }
            _ => None,
        })
        .filter(|c| {
            !c.entity_path()
                .to_string()
                .trim_start_matches('/')
                .starts_with("__")
        })
        .collect()
}

/// `entity -> (archetype short names, every chunk static?)`.
fn rendered(chunks: &[rerun::log::Chunk]) -> BTreeMap<String, (BTreeSet<String>, bool)> {
    let mut out: BTreeMap<String, (BTreeSet<String>, bool)> = BTreeMap::new();
    for chunk in chunks {
        let entity = chunk
            .entity_path()
            .to_string()
            .trim_start_matches('/')
            .to_string();
        let entry = out.entry(entity).or_insert((BTreeSet::new(), true));
        entry.1 &= chunk.is_static();
        for descr in chunk.components().component_descriptors() {
            if let Some(a) = descr.archetype.as_ref() {
                entry.0.insert(a.short_name().to_string());
            }
        }
    }
    out
}

// ---- 1. The field-layout probe and the decoder -----------------------------

fn desc(fields: &[Field]) -> Vec<PointFieldDesc> {
    fields
        .iter()
        .map(|(name, offset, datatype, count)| PointFieldDesc {
            name: (*name).to_string(),
            offset: *offset,
            datatype: *datatype,
            count: *count,
        })
        .collect()
}

#[test]
fn the_probe_accepts_exactly_the_voxel_layout() {
    let exact: &[Field] = &[
        ("vx_50mm", 0, 3, 1),
        ("vy_50mm", 2, 3, 1),
        ("vz_50mm", 4, 3, 1),
        ("hits", 6, 2, 1),
        ("op", 7, 2, 1),
    ];
    assert_eq!(
        voxel_delta_layout(&desc(exact), 8),
        Some(VoxelDeltaLayout { edge_mm: 50 })
    );
    assert_eq!(voxel_delta_layout(&desc(exact), 8).unwrap().edge_m(), 0.05);
    // The whole range of N, and nothing outside it.
    for (n, ok) in [(1u16, true), (1000, true), (0, false), (1001, false)] {
        let names = [
            format!("vx_{n}mm"),
            format!("vy_{n}mm"),
            format!("vz_{n}mm"),
        ];
        let mut f = desc(exact);
        for (i, name) in names.iter().enumerate() {
            f[i].name = name.clone();
        }
        assert_eq!(voxel_delta_layout(&f, 8).is_some(), ok, "N = {n}");
    }

    let refused: &[(&str, Vec<Field>, u32)] = &[
        (
            "mismatched N",
            vec![
                ("vx_50mm", 0, 3, 1),
                ("vy_50mm", 2, 3, 1),
                ("vz_40mm", 4, 3, 1),
                ("hits", 6, 2, 1),
                ("op", 7, 2, 1),
            ],
            8,
        ),
        (
            "wrong datatype (UINT16 index)",
            vec![
                ("vx_50mm", 0, 4, 1),
                ("vy_50mm", 2, 3, 1),
                ("vz_50mm", 4, 3, 1),
                ("hits", 6, 2, 1),
                ("op", 7, 2, 1),
            ],
            8,
        ),
        (
            "wrong datatype (INT8 op)",
            vec![
                ("vx_50mm", 0, 3, 1),
                ("vy_50mm", 2, 3, 1),
                ("vz_50mm", 4, 3, 1),
                ("hits", 6, 2, 1),
                ("op", 7, 1, 1),
            ],
            8,
        ),
        (
            "wrong name (x)",
            vec![
                ("x_50mm", 0, 3, 1),
                ("vy_50mm", 2, 3, 1),
                ("vz_50mm", 4, 3, 1),
                ("hits", 6, 2, 1),
                ("op", 7, 2, 1),
            ],
            8,
        ),
        (
            "wrong name (leading zero)",
            vec![
                ("vx_050mm", 0, 3, 1),
                ("vy_050mm", 2, 3, 1),
                ("vz_050mm", 4, 3, 1),
                ("hits", 6, 2, 1),
                ("op", 7, 2, 1),
            ],
            8,
        ),
        (
            "wrong name (unit)",
            vec![
                ("vx_5cm", 0, 3, 1),
                ("vy_5cm", 2, 3, 1),
                ("vz_5cm", 4, 3, 1),
                ("hits", 6, 2, 1),
                ("op", 7, 2, 1),
            ],
            8,
        ),
        (
            "wrong name (score)",
            vec![
                ("vx_50mm", 0, 3, 1),
                ("vy_50mm", 2, 3, 1),
                ("vz_50mm", 4, 3, 1),
                ("score", 6, 2, 1),
                ("op", 7, 2, 1),
            ],
            8,
        ),
        (
            "wrong offset",
            vec![
                ("vx_50mm", 0, 3, 1),
                ("vy_50mm", 4, 3, 1),
                ("vz_50mm", 2, 3, 1),
                ("hits", 6, 2, 1),
                ("op", 7, 2, 1),
            ],
            8,
        ),
        (
            "count 2",
            vec![
                ("vx_50mm", 0, 3, 2),
                ("vy_50mm", 2, 3, 1),
                ("vz_50mm", 4, 3, 1),
                ("hits", 6, 2, 1),
                ("op", 7, 2, 1),
            ],
            8,
        ),
        (
            "an extra field",
            vec![
                ("vx_50mm", 0, 3, 1),
                ("vy_50mm", 2, 3, 1),
                ("vz_50mm", 4, 3, 1),
                ("hits", 6, 2, 1),
                ("op", 7, 2, 1),
                ("pad", 8, 2, 1),
            ],
            8,
        ),
        ("point_step 9", exact.to_vec(), 9),
        ("point_step 16", exact.to_vec(), 16),
    ];
    for (why, fields, step) in refused {
        assert_eq!(voxel_delta_layout(&desc(fields), *step), None, "{why}");
    }
}

#[test]
fn a_voxel_frame_classifies_as_voxel_map_in_either_field_framing() {
    let walker = builtin_walker();
    let ops = [floor(1, 0), set(1, 2, 3), robot(1, 2)];
    for (framing, blob) in [
        ("canonical", canonical_fields(&voxel_fields(50))),
        ("packed", packed_fields(&voxel_fields(50))),
    ] {
        let frame = cloud_frame(&blob, 8, &ops, "odom", 1_000);
        let fv = walker.walk_by_hash(&frame).expect("walk PointCloud2");
        if framing == "canonical" {
            // Precondition: the walker really DECODED the elements.
            assert!(
                matches!(fv.field("fields"), Some(FrameValueKind::NestedArray { .. })),
                "canonical fields must decode to a NestedArray"
            );
        }
        assert_eq!(
            voxel_layout_of(&fv),
            Some(VoxelDeltaLayout { edge_mm: 50 }),
            "{framing}"
        );
        assert_eq!(classify_frame(&fv), ArchetypeKind::VoxelMap, "{framing}");
        let msg = decode_voxel_message(&fv).expect("decodes");
        assert_eq!(msg.edge_mm, 50, "{framing}");
        assert_eq!(msg.trailing_bytes, 0, "{framing}");
        assert_eq!(
            msg.ops,
            vec![
                VoxelOp::Floor {
                    epoch: 1,
                    floor_iz: 0
                },
                VoxelOp::Set {
                    x: 1,
                    y: 2,
                    z: 3,
                    hits: 4
                },
                VoxelOp::Robot {
                    x: 1,
                    y: 2,
                    z: 6,
                    yaw: 0
                },
            ],
            "{framing}"
        );
    }
    // A near miss stays a point cloud: the NAME row answers.
    let mut near = voxel_fields(50);
    near[2].0 = "vz_40mm".to_string();
    let frame = cloud_frame(&canonical_fields(&near), 8, &ops, "odom", 1_000);
    let fv = walker.walk_by_hash(&frame).expect("walk");
    assert_eq!(voxel_layout_of(&fv), None);
    assert_eq!(decode_voxel_message(&fv), None);
    assert_eq!(classify_frame(&fv), ArchetypeKind::Points3D);
    // Nor is any other schema, whatever its fields say.
    let mut other = walker.walk_by_hash(&frame).expect("walk");
    other.schema_name = "sensor_msgs/PointCloud".to_string();
    assert_eq!(voxel_layout_of(&other), None);
}

#[test]
fn voxel_map_is_not_coalesced_so_a_clear_in_a_batch_survives() {
    assert!(!coalesces(ArchetypeKind::VoxelMap));
    // Behaviour: two frames drained in ONE tick both apply. Coalescing would
    // keep only the second, and the first frame's CLEAR would be lost.
    let walker = builtin_walker();
    let mut state = SinkState::new();
    let (rec, _storage) = memory();
    dispatch_frame(
        &rec,
        &walker,
        TOPIC,
        &voxel_frame(
            &[floor(1, 0), set(1, 1, 5), set(2, 2, 5), robot(0, 0)],
            SECOND,
        ),
        &mut state,
    );
    let batch = vec![
        voxel_frame(
            &[floor(1, 0), op(1, 1, 5, 0, OP_CLEAR), robot(0, 0)],
            2 * SECOND,
        ),
        voxel_frame(&[floor(1, 0), set(3, 3, 5), robot(0, 0)], 2 * SECOND + 1),
    ];
    let mut staged = None;
    let mut coalesced = 0;
    for f in batch {
        dispatch_or_stage(
            &rec,
            &walker,
            TOPIC,
            f,
            &mut state,
            &mut staged,
            &mut coalesced,
        );
    }
    assert!(staged.is_none(), "a voxel frame is never staged");
    assert_eq!(coalesced, 0);
    let map = state.voxel_map(TOPIC).expect("state");
    assert!(!map.contains(1, 1, 5), "the first frame's CLEAR applied");
    assert!(map.contains(2, 2, 5) && map.contains(3, 3, 5));
    assert_eq!(map.visible_count(), 2);
}

// ---- 2. Ops, epochs, determinism --------------------------------------------

#[test]
fn ops_decode_in_both_byte_orders() {
    let le: Vec<u8> = [op(-2, 300, -1, 9, OP_SET), op(5, 6, 7, 200, OP_ROBOT)]
        .iter()
        .flatten()
        .copied()
        .collect();
    let mut be = Vec::new();
    for (x, y, z, hits, code) in [
        (-2i16, 300i16, -1i16, 9u8, OP_SET),
        (5, 6, 7, 200, OP_ROBOT),
    ] {
        be.extend_from_slice(&x.to_be_bytes());
        be.extend_from_slice(&y.to_be_bytes());
        be.extend_from_slice(&z.to_be_bytes());
        be.push(hits);
        be.push(code);
    }
    let want = vec![
        VoxelOp::Set {
            x: -2,
            y: 300,
            z: -1,
            hits: 9,
        },
        VoxelOp::Robot {
            x: 5,
            y: 6,
            z: 7,
            yaw: 200,
        },
    ];
    assert_eq!(decode_ops(&le, 2, false), want);
    assert_eq!(decode_ops(&be, 2, true), want);
    // An op past `n` or past the data is not read.
    assert_eq!(decode_ops(&le, 1, false), want[..1].to_vec());
    assert_eq!(decode_ops(&le[..12], 2, false), want[..1].to_vec());
}

#[test]
fn all_seven_ops_apply_as_documented() {
    let mut map = VoxelMapState::new();
    // RESET (epoch 7) + FLOOR + SETs + ROBOT.
    let a = map.apply(
        ROOT,
        Some("tf#/world/odom"),
        &message(&[
            op(7, 0, 0, 0, OP_RESET),
            floor(7, -1),
            set(0, 0, 4),
            set(40, 0, 4),
            robot(0, 0),
        ]),
        SECOND,
    );
    assert_eq!(map.epoch(), Some(7));
    assert_eq!(map.floor_iz(), Some(-1));
    assert_eq!(map.visible_count(), 2);
    assert_eq!(map.counters().resets, 1);
    assert_eq!(
        a[0],
        LogAction::ClearRecursive {
            entity: ROOT.to_string()
        },
        "a new epoch clears the whole map entity first"
    );
    // CLEAR removes; a CLEAR of an absent voxel is a no-op (idempotent).
    map.apply(
        ROOT,
        Some("tf#/world/odom"),
        &message(&[
            floor(7, -1),
            op(0, 0, 4, 0, OP_CLEAR),
            op(0, 0, 4, 0, OP_CLEAR),
        ]),
        2 * SECOND,
    );
    assert!(!map.contains(0, 0, 4));
    assert_eq!(map.visible_count(), 1);
    // TILE empties the tile holding (32..64, 0..32); its SETs are its content;
    // END_TILE closes the group. The other tile is untouched.
    map.apply(
        ROOT,
        Some("tf#/world/odom"),
        &message(&[
            floor(7, -1),
            set(0, 1, 4),
            op(32, 0, 0, 0, OP_TILE),
            set(33, 1, 4),
            op(0, 0, 0, 0, OP_END_TILE),
        ]),
        3 * SECOND,
    );
    assert!(!map.contains(40, 0, 4), "TILE emptied the old tile content");
    assert!(map.contains(33, 1, 4) && map.contains(0, 1, 4));
    assert_eq!(map.visible_count(), 2);
    // ROBOT: the trail starts at the voxel centre.
    assert_eq!(map.trail(), vec![[0.025, 0.025]]);
    // An unknown op code is counted and skipped.
    map.apply(
        ROOT,
        None,
        &message(&[floor(7, -1), op(0, 0, 0, 0, 99)]),
        4 * SECOND,
    );
    assert_eq!(map.counters().unknown_ops, 1);
    assert_eq!(map.visible_count(), 2);
    // Applying the same message twice changes nothing the second time.
    let m = message(&[floor(7, -1), set(5, 5, 5), op(0, 1, 4, 0, OP_CLEAR)]);
    map.apply(ROOT, None, &m, 5 * SECOND);
    let before = map.visible_count();
    map.apply(ROOT, None, &m, 6 * SECOND);
    assert_eq!(map.visible_count(), before);
}

#[test]
fn a_lost_reset_is_healed_by_a_floor_with_a_new_epoch() {
    let mut map = VoxelMapState::new();
    map.apply(
        ROOT,
        Some("f"),
        &message(&[floor(1, 0), set(1, 1, 5), set(2, 2, 5), robot(1, 1)]),
        SECOND,
    );
    assert_eq!(map.visible_count(), 2);
    // The RESET that started epoch 2 was lost; the next message's FLOOR names 2.
    let actions = map.apply(
        ROOT,
        Some("f"),
        &message(&[floor(2, 0), set(9, 9, 5), robot(9, 9)]),
        2 * SECOND,
    );
    assert_eq!(map.epoch(), Some(2));
    assert_eq!(map.counters().healed_resets, 1);
    assert_eq!(map.counters().resets, 0);
    assert!(!map.contains(1, 1, 5) && !map.contains(2, 2, 5));
    assert!(map.contains(9, 9, 5));
    assert_eq!(map.trail().len(), 1, "the trail restarts with the epoch");
    assert_eq!(
        actions[0],
        LogAction::ClearRecursive {
            entity: ROOT.to_string()
        }
    );
}

#[test]
fn two_replays_of_the_same_frames_give_identical_log_calls() {
    let frames: Vec<(Vec<[u8; 8]>, u64)> = vec![
        (vec![op(3, 0, 0, 0, OP_RESET), floor(3, 0), robot(0, 0)], 0),
        (
            (0..40i16)
                .flat_map(|i| [set(i, 10, 3), set(i, 10, 4), set(i, 10, 5)])
                .chain([floor(3, 0), robot(2, 0)])
                .collect(),
            SECOND / 2,
        ),
        (
            vec![floor(3, 0), op(5, 10, 4, 0, OP_CLEAR), robot(4, 0)],
            SECOND,
        ),
        (vec![floor(3, 1), robot(6, 0)], 2 * SECOND),
    ];
    let run = || {
        let mut map = VoxelMapState::new();
        frames
            .iter()
            .map(|(ops, t)| map.apply(ROOT, Some("tf#/world/odom"), &message(ops), *t))
            .collect::<Vec<_>>()
    };
    let (a, b) = (run(), run());
    assert_eq!(a, b);
    assert!(
        a.iter().map(Vec::len).sum::<usize>() > 10,
        "the replay drew"
    );
}

#[test]
fn cubes_draw_at_most_every_500_ms_of_wire_time() {
    let mut map = VoxelMapState::new();
    let cubes = |a: &[LogAction]| {
        a.iter()
            .filter(|x| matches!(x, LogAction::Cubes { .. }))
            .count()
    };
    let t0 = 10 * SECOND;
    assert_eq!(
        cubes(&map.apply(ROOT, None, &message(&[floor(1, 0), set(0, 0, 3)]), t0)),
        1
    );
    // 100 ms later: dirty, but held.
    assert_eq!(
        cubes(&map.apply(
            ROOT,
            None,
            &message(&[floor(1, 0), set(0, 0, 4)]),
            t0 + SECOND / 10
        )),
        0
    );
    // A 496 ms tick (producer jitter) still draws.
    assert_eq!(
        cubes(&map.apply(ROOT, None, &message(&[floor(1, 0)]), t0 + 496_000_000)),
        1
    );
    // A stamp that went BACKWARDS opens the gate instead of stalling it.
    assert_eq!(
        cubes(&map.apply(
            ROOT,
            None,
            &message(&[floor(1, 0), set(0, 0, 9)]),
            t0 - SECOND
        )),
        1
    );
}

// ---- 3. What the viewer receives --------------------------------------------

/// The live-map entity tree, all static, as [`execute`] logs it into a memory
/// recording from frames walked by the built-in schema walker.
#[test]
fn the_map_is_a_static_entity_tree_under_the_topic() {
    let walker = builtin_walker();
    let mut map = VoxelMapState::new();
    let (rec, storage) = memory();
    // A wall: two columns wide, 0.10..1.50 m tall, on floor layer 0.
    let mut ops = vec![op(1, 0, 0, 0, OP_RESET), floor(1, 0)];
    for x in 0..2i16 {
        ops.extend((2..=30i16).map(|z| set(x, 4, z)));
        ops.push(set(x, 0, 0));
    }
    ops.push(robot(0, 0));
    for (ops, stamp) in [(ops, SECOND), (vec![floor(1, 0), robot(10, 0)], 2 * SECOND)] {
        let frame = voxel_frame(&ops, stamp);
        let fv = walker.walk_by_hash(&frame).expect("walk PointCloud2");
        let msg = decode_voxel_message(&fv).expect("a voxel-delta frame");
        let actions = map.apply(ROOT, Some("tf#/world/odom"), &msg, stamp);
        execute(&rec, &actions);
    }
    let all = chunks(&rec, &storage);
    let r = rendered(&all);
    let tile = tile_segment((0, 0));
    assert_eq!(tile, "t_32768_32768");
    for (path, family) in [
        (format!("{ROOT}/viz-cubes/{tile}"), "VoxelGridMap"),
        (format!("{ROOT}/viz-walls/{tile}"), "Mesh3D"),
        (format!("{ROOT}/viz-edges/{tile}"), "LineStrips3D"),
        (format!("{ROOT}/viz-trail"), "LineStrips3D"),
    ] {
        let (families, all_static) = r
            .get(&path)
            .unwrap_or_else(|| panic!("{path} was not logged: {:?}", r.keys()));
        assert!(families.contains(family), "{path}: {families:?}");
        assert!(
            families.contains("CoordinateFrame"),
            "{path} is posed in the message's frame: {families:?}"
        );
        assert!(all_static, "{path} must be logged STATIC");
    }
    // The epoch's recursive Clear at the topic entity is static too, and it is
    // the only thing the codec logs there.
    let (root_families, root_static) = r
        .get(ROOT)
        .unwrap_or_else(|| panic!("{ROOT} was not cleared: {:?}", r.keys()));
    assert_eq!(root_families, &BTreeSet::from(["Clear".to_string()]));
    assert!(
        root_static,
        "a fresh map clears its entity first, statically"
    );
    // Nothing else is drawn: no point cloud, no sweep ring.
    let expected: BTreeSet<String> = [
        ROOT.to_string(),
        format!("{ROOT}/viz-cubes/{tile}"),
        format!("{ROOT}/viz-walls/{tile}"),
        format!("{ROOT}/viz-edges/{tile}"),
        format!("{ROOT}/viz-trail"),
    ]
    .into_iter()
    .collect();
    assert_eq!(r.keys().cloned().collect::<BTreeSet<_>>(), expected);
    assert_eq!(map.counters().messages, 2);
    assert_eq!(map.counters().resets, 1);
}

#[test]
fn a_tile_is_translated_to_its_origin_and_indexed_inside_it() {
    assert_eq!(tile_of(-1, 33), (-1, 1));
    assert_eq!(tile_of(31, 32), (0, 1));
    assert_eq!(tile_segment((-1, 1)), "t_32767_32769");
    let mut map = VoxelMapState::new();
    let actions = map.apply(ROOT, None, &message(&[floor(1, 0), set(-1, 33, 2)]), SECOND);
    let cubes: Vec<&LogAction> = actions
        .iter()
        .filter(|a| matches!(a, LogAction::Cubes { .. }))
        .collect();
    assert_eq!(
        cubes,
        vec![&LogAction::Cubes {
            entity: format!("{ROOT}/viz-cubes/t_32767_32769"),
            translation: [-1.6, 1.6, 0.0],
            voxel_size: 0.05,
            indices: vec![[31, 1, 2]],
            colors: vec![height_rgb(0.10)],
        }]
    );
    // Rerun places voxel [i, j, k] at translation + (index + 0.5) * size, so the
    // centre is (-0.025, 1.675, 0.125): the voxel floor(-0.025/0.05) = -1 etc.
}

#[test]
fn an_emptied_tile_is_cleared_and_its_frame_logged_again_when_it_refills() {
    let mut map = VoxelMapState::new();
    map.apply(
        ROOT,
        Some("F"),
        &message(&[floor(1, 0), set(0, 0, 3)]),
        SECOND,
    );
    let entity = format!("{ROOT}/viz-cubes/t_32768_32768");
    let a = map.apply(
        ROOT,
        Some("F"),
        &message(&[floor(1, 0), op(0, 0, 3, 0, OP_CLEAR)]),
        2 * SECOND,
    );
    assert!(a.contains(&LogAction::ClearFlat {
        entity: entity.clone()
    }));
    let a = map.apply(
        ROOT,
        Some("F"),
        &message(&[floor(1, 0), set(0, 0, 3)]),
        3 * SECOND,
    );
    assert!(
        a.contains(&LogAction::Frame {
            entity: entity.clone(),
            frame: "F".to_string()
        }),
        "a flat Clear shadows the frame too, so it is logged again: {a:?}"
    );
}

#[test]
fn a_reconnect_redraws_every_tile_and_the_trail() {
    let mut map = VoxelMapState::new();
    map.apply(
        ROOT,
        Some("F"),
        &message(&[floor(1, 0), set(0, 0, 3), set(100, 100, 3), robot(0, 0)]),
        SECOND,
    );
    map.rearm();
    let a = map.apply(ROOT, Some("F"), &message(&[floor(1, 0)]), SECOND + 1);
    let cubes = a
        .iter()
        .filter(|x| matches!(x, LogAction::Cubes { .. }))
        .count();
    let frames = a
        .iter()
        .filter(|x| matches!(x, LogAction::Frame { .. }))
        .count();
    assert_eq!(cubes, 2, "both tiles again: {a:?}");
    assert!(a.iter().any(|x| matches!(x, LogAction::Trail { .. })));
    assert!(frames >= 3, "cubes x2 + trail are framed again");
    assert!(
        !a.iter()
            .any(|x| matches!(x, LogAction::ClearRecursive { .. })),
        "a reconnect is not a new epoch"
    );
}

// ---- 4. Look: walls, ramp, trail --------------------------------------------

/// A synthetic room corner: a floor layer at iz = 0 and one straight wall four
/// cells long (x = 0..8 columns, y column 10), 0.10..1.50 m tall.
#[test]
fn a_synthetic_wall_extrudes_to_the_hand_counted_mesh() {
    let mut voxels: BTreeMap<(i16, i16, i16), u8> = BTreeMap::new();
    for x in 0..16i16 {
        for y in 0..16i16 {
            voxels.insert((x, y, 0), 3); // floor layer
        }
    }
    for x in 0..8i16 {
        for z in 2..=30i16 {
            voxels.insert((x, 10, z), 3);
        }
    }
    let cells = wall_cells(&voxels, 0, 50);
    // Floor voxels (h = 0) are never wall voxels; the wall is cells (0..4, 5).
    assert_eq!(
        cells.keys().copied().collect::<Vec<_>>(),
        vec![(0, 5), (1, 5), (2, 5), (3, 5)]
    );
    assert!(cells.values().all(|&top| top == 30));
    let lookup = cells.clone();
    let (mesh, edges) = wall_geometry(&cells, |c| lookup.get(&c).copied(), 0, 50);
    // Per cell: a top (2) + the two long sides (4); the two ends (2 each).
    assert_eq!(mesh.triangles.len(), 4 * (2 + 4) + 2 * 2);
    assert_eq!(mesh.positions.len(), mesh.triangles.len() * 2);
    // 1.50 m tall >= 1.0 m, so every side face gets a top outline.
    assert_eq!(edges.len(), 4 * 2 + 2);
    // Heights: from the floor top (0.05) to the top face of layer 30 (1.55).
    let zs: BTreeSet<i64> = mesh
        .positions
        .iter()
        .map(|p| (p[2] * 1000.0).round() as i64)
        .collect();
    assert_eq!(zs, BTreeSet::from([50, 1550]));
    // Normals: unit length, OUTWARD from the wall's centre line, and agreeing
    // with each triangle's counter-clockwise winding.
    let centre = [0.2f32, 0.55, 0.8];
    for t in &mesh.triangles {
        let [a, b, c] = t.map(|i| mesh.positions[i as usize]);
        let n = mesh.normals[t[0] as usize];
        assert!((n[0] * n[0] + n[1] * n[1] + n[2] * n[2] - 1.0).abs() < 1e-6);
        let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let cross = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        let dot = |u: [f32; 3], v: [f32; 3]| u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
        assert!(dot(cross, n) > 0.0, "winding agrees with the normal");
        let mid = [
            (a[0] + b[0] + c[0]) / 3.0 - centre[0],
            (a[1] + b[1] + c[1]) / 3.0 - centre[1],
            (a[2] + b[2] + c[2]) / 3.0 - centre[2],
        ];
        assert!(dot(mid, n) > 0.0, "normal {n:?} points outward");
    }
    // A short wall (0.10..0.80 m) gets no outline.
    let mut low = BTreeMap::new();
    for z in 2..=16i16 {
        low.insert((0i16, 0i16, z), 1u8);
    }
    let low_cells = wall_cells(&low, 0, 50);
    let (_, low_edges) = wall_geometry(&low_cells, |_| None, 0, 50);
    assert!(low_edges.is_empty());
    // Two voxels in the band are not a wall.
    let two: BTreeMap<_, _> = [((0i16, 0i16, 3i16), 1u8), ((0, 0, 4), 1)].into();
    assert!(wall_cells(&two, 0, 50).is_empty());
}

#[test]
fn the_colour_ramp_hits_every_stop() {
    assert_eq!(height_rgb(-1.0), FLOOR_RGB);
    assert_eq!(height_rgb(0.0), FLOOR_RGB);
    assert_eq!(
        height_rgb(0.05),
        FLOOR_RGB,
        "one voxel above the floor is floor"
    );
    assert_eq!(height_rgb(0.30), CERULEAN_RGB);
    assert_eq!(height_rgb(1.20), BLUE_RGB);
    assert_eq!(height_rgb(1.80), EMBER_RGB);
    assert_eq!(height_rgb(4.0), EMBER_RGB);
    assert_eq!(height_rgb(f64::NAN), FLOOR_RGB);
    assert_eq!(height_rgb(0.75), [0x00, 0xA0, 0xFF]);
    assert_eq!(FLOOR_RGB, [0x31, 0x41, 0x58]);
    assert_eq!(CERULEAN_RGB, [0x00, 0xC0, 0xFF]);
    assert_eq!(BLUE_RGB, [0x00, 0x80, 0xFF]);
    assert_eq!(EMBER_RGB, [0xFF, 0x82, 0x1C]);
}

#[test]
fn the_trail_steps_every_10_cm_and_keeps_the_newest_5000_points() {
    let mut map = VoxelMapState::new();
    // One voxel (5 cm) per message: a trail point every second message.
    for i in 0..10i16 {
        map.apply(
            ROOT,
            None,
            &message(&[floor(1, 0), robot(i, 0)]),
            u64::from(i as u16) * SECOND,
        );
    }
    let xs: Vec<f32> = map.trail().iter().map(|p| p[0]).collect();
    assert_eq!(xs.len(), 5);
    for (k, x) in xs.iter().enumerate() {
        assert!((x - (0.025 + 0.10 * k as f32)).abs() < 1e-5, "{xs:?}");
    }
    // The cap: 6 000 steps of 10 cm keep the newest 5 000.
    let mut map = VoxelMapState::new();
    for i in 0..6_000i32 {
        let (x, y) = ((i % 2_000) as i16 * 2, (i / 2_000) as i16 * 20);
        map.apply(ROOT, None, &message(&[floor(1, 0), robot(x, y)]), i as u64);
    }
    let trail = map.trail();
    assert_eq!(trail.len(), TRAIL_MAX_POINTS);
    assert!(
        (trail[0][0] - 0.025 - 0.10 * 1_000.0).abs() < 1e-3,
        "{:?}",
        trail[0]
    );
    // The drawn strip floats 2 cm over the floor top (layer 0, so 0.05 m).
    let mut map = VoxelMapState::new();
    let a = map.apply(ROOT, None, &message(&[floor(1, 0), robot(0, 0)]), SECOND);
    let Some(LogAction::Trail { points, .. }) =
        a.iter().find(|x| matches!(x, LogAction::Trail { .. }))
    else {
        panic!("no trail drawn: {a:?}");
    };
    assert!((points[0][2] - 0.07).abs() < 1e-6);
}

// ---- 5. PNG floor plan on the existing EncodedImage path --------------------

/// A valid 1 x 1 RGBA PNG.
const PNG_1X1: &[u8] = &[
    0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, 0x00, 0x00, 0x00, 0x0D, 0x49, 0x48, 0x44, 0x52,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x08, 0x06, 0x00, 0x00, 0x00, 0x1F, 0x15, 0xC4,
    0x89, 0x00, 0x00, 0x00, 0x0A, 0x49, 0x44, 0x41, 0x54, 0x78, 0x9C, 0x63, 0x00, 0x01, 0x00, 0x00,
    0x05, 0x00, 0x01, 0x0D, 0x0A, 0x2D, 0xB4, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4E, 0x44, 0xAE,
    0x42, 0x60, 0x82,
];

/// The map node's floor plan is a PNG `sensor_msgs/CompressedImage`; it rides
/// the existing `EncodedImage` path and must never fall to a text dump.
#[test]
fn a_png_compressed_image_is_logged_as_an_encoded_image() {
    let layout = layout_of("sensor_msgs/CompressedImage");
    let fixed = layout.fixed_size;
    let table = layout.offset_table_bytes();
    let format = b"png";
    let mut payload = vec![0u8; fixed + table];
    let format_off = (fixed + table) as u32;
    let data_off = format_off + format.len() as u32;
    write_offset_entry(&mut payload, fixed, 0, 0, 0);
    write_offset_entry(&mut payload, fixed, 1, format_off, format.len() as u32);
    write_offset_entry(&mut payload, fixed, 2, data_off, PNG_1X1.len() as u32);
    payload.extend_from_slice(format);
    payload.extend_from_slice(PNG_1X1);
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: <CompressedImage as ShmMessage>::SCHEMA_HASH,
        total_size: (WireHeader::SIZE + payload.len()) as u32,
        offset_table_offset: (WireHeader::SIZE + fixed) as u32,
        offset_table_count: 3,
        sequence: 0,
        timestamp_ns: 5_000,
    }
    .write_to_buf(&mut frame);
    frame.extend_from_slice(&payload);

    let walker = builtin_walker();
    let fv = walker.walk_by_hash(&frame).expect("walk");
    assert_eq!(classify_frame(&fv), ArchetypeKind::Image);
    let mut state = SinkState::new();
    let (rec, storage) = memory();
    let topic = "/go2/map_view/plan";
    dispatch_frame(&rec, &walker, topic, &frame, &mut state);
    let r = rendered(&chunks(&rec, &storage));
    let entity = route_for_input(topic).entity;
    let (families, _) = &r[&entity];
    assert!(families.contains("EncodedImage"), "{families:?}");
    assert!(
        !r.values().any(|(f, _)| f.contains("TextDocument")),
        "a PNG plan must never become a text dump: {r:?}"
    );
}
