// SPDX-License-Identifier: AGPL-3.0-only
//! The live voxel map codec (`cerulion_viz::voxel_map`): the field-layout
//! probe, the frame decoder over both `fields` framings, the seven ops, the
//! epoch rules, determinism, the static entity tree the viewer receives, the
//! wall geometry, the colour ramp and the trail. Every oracle is hand-written.

use std::collections::{BTreeMap, BTreeSet};

use cerulion_core::codegen::layout::{LayoutResolver, WireLayout};
use cerulion_core::codegen::{parse_rosmsg, FrameValueKind, MessageSchema};
use cerulion_core::message::ShmMessage;
use cerulion_core::shm_runtime::write_offset_entry;
use cerulion_core::wire::WireHeader;
use cerulion_viz::pointcloud::PointFieldDesc;
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::voxel_map::{
    decode_ops, decode_rows, decode_voxel_message, execute, height_rgb, tile_of, tile_segment,
    voxel_delta_layout, voxel_layout_of, wall_cells, wall_geometry, LogAction, VoxelDeltaLayout,
    VoxelMapState, VoxelMessage, VoxelOp, BLUE_RGB, CERULEAN_RGB, EMBER_RGB, FLOOR_RGB,
    MAX_WALL_TRIANGLES_TOTAL, OP_CLEAR, OP_END_TILE, OP_FLOOR, OP_RESET, OP_ROBOT, OP_SET, OP_TILE,
    TRAIL_MAX_POINTS,
};
use native_ros2_messages::sensor_msgs::PointCloud2;

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

/// One op as its 8 wire bytes in either byte order.
fn op_bytes(x: i16, y: i16, z: i16, hits: u8, code: u8, big_endian: bool) -> [u8; 8] {
    let i16_bytes = |v: i16| {
        if big_endian {
            v.to_be_bytes()
        } else {
            v.to_le_bytes()
        }
    };
    let mut b = [0u8; 8];
    b[0..2].copy_from_slice(&i16_bytes(x));
    b[2..4].copy_from_slice(&i16_bytes(y));
    b[4..6].copy_from_slice(&i16_bytes(z));
    b[6] = hits;
    b[7] = code;
    b
}

/// One op as its 8 wire bytes (little-endian).
fn op(x: i16, y: i16, z: i16, hits: u8, code: u8) -> [u8; 8] {
    op_bytes(x, y, z, hits, code, false)
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

/// The geometry fields of a PointCloud2: how `data` is declared, independent of
/// what it holds.
#[derive(Clone, Copy)]
struct CloudShape {
    width: u32,
    height: u32,
    row_step: u32,
    big_endian: bool,
}

/// A whole `sensor_msgs/PointCloud2` wire frame: `header.frame_id` = `frame_id`,
/// `fields` as given, `point_step`, and the op bytes as one unorganized
/// little-endian row (`height` 1, `row_step` = the row's bytes).
fn cloud_frame(
    fields_blob: &[u8],
    point_step: u32,
    ops: &[[u8; 8]],
    frame_id: &str,
    timestamp_ns: u64,
) -> Vec<u8> {
    let data: Vec<u8> = ops.iter().flatten().copied().collect();
    let shape = CloudShape {
        width: ops.len() as u32,
        height: 1,
        row_step: point_step * ops.len() as u32,
        big_endian: false,
    };
    shaped_cloud_frame(
        fields_blob,
        point_step,
        &data,
        shape,
        frame_id,
        timestamp_ns,
    )
}

/// [`cloud_frame`] with the geometry and byte order chosen by the caller and
/// `data` given as raw bytes (padded rows included).
fn shaped_cloud_frame(
    fields_blob: &[u8],
    point_step: u32,
    data: &[u8],
    shape: CloudShape,
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
    let header = header_blob(frame_id);
    let mut payload = vec![0u8; fixed + table];
    let put = |buf: &mut [u8], name: &str, v: u32| {
        let off = fixed_off(&layout, name);
        buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
    };
    put(&mut payload, "height", shape.height);
    put(&mut payload, "width", shape.width);
    put(&mut payload, "point_step", point_step);
    put(&mut payload, "row_step", shape.row_step);
    payload[fixed_off(&layout, "is_bigendian")] = u8::from(shape.big_endian);
    payload[fixed_off(&layout, "is_dense")] = 1;
    let header_off = (fixed + table) as u32;
    let fields_off = header_off + header.len() as u32;
    let data_off = fields_off + fields_blob.len() as u32;
    write_offset_entry(&mut payload, fixed, 0, header_off, header.len() as u32);
    write_offset_entry(&mut payload, fixed, 1, fields_off, fields_blob.len() as u32);
    write_offset_entry(&mut payload, fixed, 2, data_off, data.len() as u32);
    payload.extend_from_slice(&header);
    payload.extend_from_slice(fields_blob);
    payload.extend_from_slice(data);
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
fn a_voxel_frame_decodes_in_either_field_framing() {
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
    // A near miss is not a voxel stream: the NAME row answers.
    let mut near = voxel_fields(50);
    near[2].0 = "vz_40mm".to_string();
    let frame = cloud_frame(&canonical_fields(&near), 8, &ops, "odom", 1_000);
    let fv = walker.walk_by_hash(&frame).expect("walk");
    assert_eq!(voxel_layout_of(&fv), None);
    assert_eq!(decode_voxel_message(&fv), None);
    // Nor is any other schema, whatever its fields say.
    let mut other = walker.walk_by_hash(&frame).expect("walk");
    other.schema_name = "sensor_msgs/PointCloud".to_string();
    assert_eq!(voxel_layout_of(&other), None);
}

#[test]
fn a_big_endian_frame_decodes_through_the_cloud_header() {
    let walker = builtin_walker();
    let fields = canonical_fields(&voxel_fields(50));
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
    for big_endian in [true, false] {
        let data: Vec<u8> = [
            op_bytes(-2, 300, -1, 9, OP_SET, big_endian),
            op_bytes(5, 6, 7, 200, OP_ROBOT, big_endian),
        ]
        .iter()
        .flatten()
        .copied()
        .collect();
        let shape = CloudShape {
            width: 2,
            height: 1,
            row_step: 16,
            big_endian,
        };
        let frame = shaped_cloud_frame(&fields, 8, &data, shape, "odom", 1_000);
        let fv = walker.walk_by_hash(&frame).expect("walk PointCloud2");
        assert_eq!(
            fv.field("is_bigendian"),
            Some(&FrameValueKind::Bool(big_endian)),
            "precondition: the frame carries is_bigendian"
        );
        let msg = decode_voxel_message(&fv).expect("decodes");
        assert_eq!(msg.ops, want, "is_bigendian = {big_endian}");
        assert_eq!(msg.trailing_bytes, 0);
    }
    // The byte order is READ from the frame: the same big-endian bytes under a
    // little-endian header decode to other voxels, so nothing is hardcoded.
    let be_data: Vec<u8> = op_bytes(-2, 300, -1, 9, OP_SET, true).to_vec();
    let shape = CloudShape {
        width: 1,
        height: 1,
        row_step: 8,
        big_endian: false,
    };
    let frame = shaped_cloud_frame(&fields, 8, &be_data, shape, "odom", 1_000);
    let fv = walker.walk_by_hash(&frame).expect("walk");
    assert_ne!(
        decode_voxel_message(&fv).expect("decodes").ops,
        want[..1].to_vec()
    );
}

#[test]
fn an_organized_cloud_is_read_row_by_row_and_its_padding_is_never_an_op() {
    // Two rows of one op each, `row_step` 16: eight padding bytes follow each
    // op. Read back to back, the padding would decode as `SET (0, 0, 0)` and
    // the second row's op would be dropped.
    let a = set(1, 2, 3);
    let b = set(4, 5, 6);
    let mut data = Vec::new();
    data.extend_from_slice(&a);
    data.extend_from_slice(&[0u8; 8]);
    data.extend_from_slice(&b);
    data.extend_from_slice(&[0u8; 8]);
    let want = vec![
        VoxelOp::Set {
            x: 1,
            y: 2,
            z: 3,
            hits: 4,
        },
        VoxelOp::Set {
            x: 4,
            y: 5,
            z: 6,
            hits: 4,
        },
    ];
    assert_eq!(decode_rows(&data, 1, 2, 16, false), (want.clone(), 16));
    // Through the whole frame too.
    let walker = builtin_walker();
    let shape = CloudShape {
        width: 1,
        height: 2,
        row_step: 16,
        big_endian: false,
    };
    let frame = shaped_cloud_frame(
        &canonical_fields(&voxel_fields(50)),
        8,
        &data,
        shape,
        "odom",
        1_000,
    );
    let fv = walker.walk_by_hash(&frame).expect("walk PointCloud2");
    let msg = decode_voxel_message(&fv).expect("decodes");
    assert_eq!(msg.ops, want);
    assert_eq!(msg.trailing_bytes, 16, "the padding is reported, not read");
    // Rows declared past the data are not read; a row cut short of one op is
    // its unread bytes.
    assert_eq!(decode_rows(&data[..24], 1, 3, 16, false), (want.clone(), 8));
    assert_eq!(
        decode_rows(&data[..20], 1, 2, 16, false),
        (want[..1].to_vec(), 12)
    );
    // A packed organized cloud (`row_step` = the row's bytes) and a malformed
    // `row_step` below it read the rows back to back, as `point_count` counts.
    let packed: Vec<u8> = [a, b].iter().flatten().copied().collect();
    assert_eq!(decode_rows(&packed, 1, 2, 8, false), (want.clone(), 0));
    assert_eq!(decode_rows(&packed, 1, 2, 0, false), (want.clone(), 0));
    // An unorganized cloud ignores `row_step`, whatever it says.
    assert_eq!(decode_rows(&packed, 2, 1, 1_000, false), (want, 0));
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
    assert_eq!(map.counters().split_tile_groups, 0, "the group was whole");
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
fn a_repeated_reset_for_the_held_epoch_changes_nothing() {
    let mut map = VoxelMapState::new();
    map.apply(
        ROOT,
        Some("F"),
        &message(&[
            op(7, 0, 0, 0, OP_RESET),
            floor(7, -1),
            set(0, 0, 4),
            robot(0, 0),
        ]),
        SECOND,
    );
    assert_eq!(map.visible_count(), 1);
    // The same RESET again (a duplicated or replayed message): idempotent.
    let a = map.apply(
        ROOT,
        Some("F"),
        &message(&[op(7, 0, 0, 0, OP_RESET), floor(7, -1)]),
        2 * SECOND,
    );
    assert_eq!(map.counters().resets, 2, "counted, not acted on");
    assert_eq!(map.epoch(), Some(7));
    assert_eq!(map.floor_iz(), Some(-1));
    assert!(map.contains(0, 0, 4));
    assert_eq!(map.trail().len(), 1);
    assert!(a.is_empty(), "nothing changed, so nothing is logged: {a:?}");
    // A RESET naming a NEW epoch still empties everything.
    let b = map.apply(
        ROOT,
        Some("F"),
        &message(&[op(8, 0, 0, 0, OP_RESET), floor(8, -1)]),
        3 * SECOND,
    );
    assert_eq!(map.epoch(), Some(8));
    assert_eq!(map.visible_count(), 0);
    assert!(map.trail().is_empty());
    assert_eq!(
        b,
        vec![LogAction::ClearRecursive {
            entity: ROOT.to_string()
        }]
    );
}

#[test]
fn a_tile_group_the_message_ends_inside_is_counted() {
    let mut map = VoxelMapState::new();
    // TILE and its SETs, but the message ends before END_TILE.
    map.apply(
        ROOT,
        None,
        &message(&[floor(1, 0), op(0, 0, 0, 0, OP_TILE), set(1, 1, 3)]),
        SECOND,
    );
    assert_eq!(map.counters().split_tile_groups, 1);
    assert!(map.contains(1, 1, 3), "the SETs still apply");
    // The next message's END_TILE closes nothing that was opened in it.
    map.apply(
        ROOT,
        None,
        &message(&[floor(1, 0), op(0, 0, 0, 0, OP_END_TILE)]),
        2 * SECOND,
    );
    assert_eq!(map.counters().split_tile_groups, 1);
}

#[test]
fn a_message_carries_one_robot_so_a_replayed_message_draws_no_trip() {
    let mut map = VoxelMapState::new();
    // Two ROBOT ops 10 cm apart in ONE message: the last is the tick's position.
    let m = message(&[floor(1, 0), robot(0, 0), robot(2, 0)]);
    map.apply(ROOT, None, &m, SECOND);
    assert_eq!(map.trail(), vec![[0.125, 0.025]]);
    assert_eq!(map.counters().extra_robot_ops, 1);
    // The same message again: the robot is where it was, so no trail point and
    // nothing to log.
    let a = map.apply(ROOT, None, &m, 2 * SECOND);
    assert_eq!(map.trail().len(), 1);
    assert!(a.is_empty(), "{a:?}");
    assert_eq!(
        map.counters().extra_robot_ops,
        2,
        "the replay's extra is counted too"
    );
    // A real move in the next message extends the trail by one point.
    map.apply(
        ROOT,
        None,
        &message(&[floor(1, 0), robot(4, 0)]),
        3 * SECOND,
    );
    assert_eq!(map.trail().len(), 2);
    assert_eq!(map.counters().extra_robot_ops, 2);
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

/// The producer's round-robin refresh re-sends whole tiles that mostly did not
/// change. A static write APPENDS in the viewer's store, so an unchanged tile
/// must log nothing, however many gates open.
#[test]
fn an_unchanged_tile_refresh_logs_nothing() {
    let mut map = VoxelMapState::new();
    let count = |a: &[LogAction], f: fn(&LogAction) -> bool| a.iter().filter(|x| f(x)).count();
    let is_cubes = |x: &LogAction| matches!(x, LogAction::Cubes { .. });
    let is_walls = |x: &LogAction| matches!(x, LogAction::Walls { .. });
    // One tile: a wall cell (three voxels in the band) and a loose voxel.
    let mut refresh = vec![floor(1, 0), op(0, 0, 0, 0, OP_TILE)];
    refresh.extend((2..=4i16).map(|z| set(0, 0, z)));
    refresh.push(set(9, 9, 3));
    refresh.push(op(0, 0, 0, 0, OP_END_TILE));
    let refresh = message(&refresh);
    let a = map.apply(ROOT, Some("F"), &refresh, SECOND);
    assert_eq!(count(&a, is_cubes), 1);
    assert_eq!(count(&a, is_walls), 1);
    // The identical refresh, two open gates later: nothing at all is logged.
    for stamp in [3 * SECOND, 5 * SECOND] {
        let b = map.apply(ROOT, Some("F"), &refresh, stamp);
        assert!(b.is_empty(), "an unchanged refresh re-logs nothing: {b:?}");
    }
    assert_eq!(map.visible_count(), 4);
    // A refresh whose content changed draws the tile's cubes once more; its
    // wall did not change, so the wall is not logged again.
    let mut changed = vec![floor(1, 0), op(0, 0, 0, 0, OP_TILE)];
    changed.extend((2..=4i16).map(|z| set(0, 0, z)));
    changed.push(op(0, 0, 0, 0, OP_END_TILE));
    let c = map.apply(ROOT, Some("F"), &message(&changed), 7 * SECOND);
    assert_eq!(count(&c, is_cubes), 1, "{c:?}");
    assert_eq!(count(&c, is_walls), 0, "{c:?}");
    assert_eq!(map.visible_count(), 3);
}

/// A wall tile the TOTAL triangle budget held back is drawn as soon as another
/// tile shrinks enough, not only when the held tile itself changes.
#[test]
fn a_wall_tile_held_by_the_total_budget_is_drawn_when_capacity_frees() {
    // A global checkerboard of wall cells (cell (cx, cy) is a wall when cx + cy
    // is even): every wall cell has four lower neighbours, so it is one top and
    // four sides = 10 triangles, 128 cells per tile = 1 280 triangles, however
    // the tile borders fall. 157 tiles in a row along x: 156 fit the total
    // budget of 200 000 (199 680), the 157th does not.
    const PER_TILE: usize = 1_280;
    let fit = MAX_WALL_TRIANGLES_TOTAL / PER_TILE;
    assert_eq!(fit, 156);
    let tiles = fit + 1;
    let mut ops = vec![floor(1, 0)];
    for tx in 0..tiles as i16 {
        for cx in 0..16i16 {
            for cy in 0..16i16 {
                if (cx + cy) % 2 != 0 {
                    continue;
                }
                let (x, y) = (tx * 32 + cx * 2, cy * 2);
                ops.extend((2..=4i16).map(|z| set(x, y, z)));
            }
        }
    }
    let mut map = VoxelMapState::new();
    let walls_of = |a: &[LogAction]| -> BTreeSet<String> {
        a.iter()
            .filter_map(|x| match x {
                LogAction::Walls { entity, .. } => Some(entity.clone()),
                _ => None,
            })
            .collect()
    };
    let last = format!("{ROOT}/viz-walls/{}", tile_segment((fit as i16, 0)));
    let a = map.apply(ROOT, Some("F"), &message(&ops), SECOND);
    let drawn = walls_of(&a);
    assert_eq!(drawn.len(), fit, "every tile but the last fits");
    assert!(!drawn.contains(&last), "the last tile is held back");
    assert_eq!(map.counters().wall_tiles_over_budget, 1);
    // The first tile is emptied by a refresh with no content: 1 280 triangles
    // free up. This pass redraws that tile and its neighbours only.
    let b = map.apply(
        ROOT,
        Some("F"),
        &message(&[
            floor(1, 0),
            op(0, 0, 0, 0, OP_TILE),
            op(0, 0, 0, 0, OP_END_TILE),
        ]),
        2 * SECOND,
    );
    assert!(!walls_of(&b).contains(&last), "{b:?}");
    assert!(b.contains(&LogAction::ClearFlat {
        entity: format!("{ROOT}/viz-walls/{}", tile_segment((0, 0)))
    }));
    // At the next surfaces gate the held tile gets its pass and now fits.
    let c = map.apply(ROOT, Some("F"), &message(&[floor(1, 0)]), 3 * SECOND);
    assert_eq!(walls_of(&c), BTreeSet::from([last]), "{}", c.len());
    assert_eq!(map.counters().wall_tiles_over_budget, 1, "no new rejection");
    // Nothing is left to draw: the map settled.
    let d = map.apply(ROOT, Some("F"), &message(&[floor(1, 0)]), 4 * SECOND);
    assert!(d.is_empty(), "{d:?}");
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
