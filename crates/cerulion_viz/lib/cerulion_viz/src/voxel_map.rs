// SPDX-License-Identifier: AGPL-3.0-only
//! The LIVE VOXEL MAP: a voxel-delta stream carried in a
//! `sensor_msgs/PointCloud2`, rendered as a lit 3D world that grows while the
//! robot walks.
//!
//! # Why a delta stream
//!
//! A map that grows live cannot be re-sent whole: at 5 cm a room is tens of
//! thousands of voxels, and the robot reaches this desk over links as thin as a
//! relay. The producer (the Go2 demo's map node, which lives outside this
//! repository) keeps the voxel grid and sends only what CHANGED, plus a slow round-robin refresh of whole
//! tiles so a late viewer, a restarted daemon or a lost frame heals by itself.
//! Everything drawn here (cubes, walls, their top edges, the walked trail) is
//! built on THIS side from that stream; nothing but indices crosses the link.
//!
//! # Wire layout
//!
//! Classified by FIELD LAYOUT, not by topic name ([`voxel_delta_layout`]):
//! `point_step` 8, fields
//! `vx_<N>mm`/`vy_<N>mm`/`vz_<N>mm` (`int16` at 0/2/4, `N` = the voxel edge in
//! mm), `hits` (`uint8` at 6) and `op` (`uint8` at 7). Each "point" is one op:
//!
//! | op | name | fields | meaning |
//! |---|---|---|---|
//! | 0 | `SET` | voxel; `hits` = score | the voxel is visible |
//! | 1 | `CLEAR` | voxel | the voxel is no longer visible |
//! | 2 | `TILE` | `vx`,`vy` = the tile's smallest column (multiples of 32) | empty the tile; the `SET`s that follow are its full content |
//! | 3 | `RESET` | `vx` = epoch (low 16 bits) | new epoch: empty everything; a `RESET` naming the epoch already held changes nothing |
//! | 4 | `ROBOT` | robot position in voxels at this tick; `hits` = yaw in 1/256 turn | trail; one per message, the last wins |
//! | 5 | `FLOOR` | `vx` = epoch; `vz` = the floor layer | floor; an epoch this viewer does not hold is a lost `RESET` |
//! | 6 | `END_TILE` | none | closes the current `TILE` group |
//!
//! Every op is idempotent, so a replayed or duplicated message is harmless (a
//! producer that restarts must therefore pick a NEW epoch). `ROBOT` is the
//! robot's position at the message's tick, so a message carries one: when it
//! carries more, the last wins and the extras are counted
//! ([`VoxelMapCounters::extra_robot_ops`]), which keeps a duplicated message
//! from drawing a trip the robot never made. A `TILE` group
//! (`TILE`, its `SET`s, `END_TILE`) is whole within one message: the state is
//! drawn only between messages, so a group never shows half-filled; a message
//! that ends inside a group is counted
//! ([`VoxelMapCounters::split_tile_groups`]), never healed. The stream must
//! never be coalesced: a dropped message may carry the only `CLEAR` for a voxel.
//! An organized cloud (`height` > 1) is read row by row at `row_step`, so row
//! padding is never an op.
//!
//! # What is drawn (all `log_static`, under the topic entity `E`)
//!
//! | Entity | Archetype | Look |
//! |---|---|---|
//! | `E/viz-cubes/t_<u>_<v>` | `VoxelGridMap` | one tile of 32 x 32 columns, coloured by height above the floor ([`height_rgb`]) and lit by the viewer |
//! | `E/viz-trail` | `LineStrips3D` | where the robot walked, `#FFB347`, a point per 10 cm, the last 5 000 |
//! | `E/viz-walls/t_<u>_<v>` | `Mesh3D` | a 2.5D extrusion of the wall columns, flat normals, cerulean sides, pale tops |
//! | `E/viz-edges/t_<u>_<v>` | `LineStrips3D` | the top outline of walls at least 1 m tall |
//!
//! `u = tx + 32768`, `v = ty + 32768` for the tile index `(tx, ty)`. STATIC,
//! because the map is state, not a time series: the viewer shows the newest
//! static write. Its store APPENDS every one, though, so an entity is logged
//! again only when its payload CHANGED: the state keeps the last payload it
//! logged per entity and a round-robin tile refresh that changed nothing logs
//! nothing. Storage grows with the map's changes, never with the refresh
//! cadence or uptime. A `RESET` is one recursive static `Clear` at each of the
//! four map children (`E/viz-cubes`, `E/viz-walls`, `E/viz-edges`, `E/viz-trail`),
//! never at `E` itself: another topic attached under the map's path renders at a
//! descendant of `E` and must survive a map reset.
//! Each drawn entity gets a static `CoordinateFrame` (the message's `frame_id`,
//! resolved as every other data topic's is), logged once per entity, so the map
//! and a model posed in the same frame cannot separate.
//!
//! # Determinism
//!
//! [`VoxelMapState::apply`] is pure: the log calls it returns are a function of the
//! frames and their WIRE stamps only (the render cadence gates read wire time,
//! never the wall clock), so two replays of one recording draw the same calls.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use cerulion_core::codegen::{FrameValue, FrameValueKind};
use rerun::RecordingStream;

use crate::pointcloud::{parse_point_fields, point_count, PointFieldDesc};
use crate::tf::implicit_frame_of;

// ---- Wire layout probe ------------------------------------------------------

/// The `point_step` of a voxel-delta cloud: three `int16` indices, `hits` and
/// `op` (see [`voxel_delta_layout`]).
pub const VOXEL_DELTA_POINT_STEP: u32 = 8;
/// The number of `fields` descriptors a voxel-delta cloud declares.
pub const VOXEL_DELTA_FIELD_COUNT: usize = 5;

/// The largest voxel edge the layout names, in millimetres (1 m).
pub const VOXEL_DELTA_MAX_EDGE_MM: u16 = 1000;

/// A PointCloud2 whose fields are a VOXEL-DELTA stream, not points (see the
/// module docs). The only varying part of the layout is the voxel edge, which
/// the index field names carry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VoxelDeltaLayout {
    /// The voxel edge in millimetres (the `<N>` of `vx_<N>mm`), `1..=1000`.
    pub edge_mm: u16,
}

impl VoxelDeltaLayout {
    /// The voxel edge in metres.
    pub fn edge_m(self) -> f32 {
        f32::from(self.edge_mm) / 1000.0
    }
}

/// The `<N>` of a `v<axis>_<N>mm` field name: decimal digits with no sign and no
/// leading zero, `1..=VOXEL_DELTA_MAX_EDGE_MM`. `None` for any other name.
fn voxel_axis_edge_mm(name: &str, axis: char) -> Option<u16> {
    let rest = name
        .strip_prefix('v')?
        .strip_prefix(axis)?
        .strip_prefix('_')?;
    let digits = rest.strip_suffix("mm")?;
    if digits.is_empty() || digits.starts_with('0') || !digits.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    let n: u16 = digits.parse().ok()?;
    (1..=VOXEL_DELTA_MAX_EDGE_MM).contains(&n).then_some(n)
}

/// The FIELD-LAYOUT probe that tells a voxel-delta stream apart from a point
/// cloud: `Some` exactly when `fields` are, in this order and nothing else,
///
/// | name | offset | datatype | count |
/// |---|---|---|---|
/// | `vx_<N>mm` | 0 | INT16 (3) | 1 |
/// | `vy_<N>mm` | 2 | INT16 | 1 |
/// | `vz_<N>mm` | 4 | INT16 | 1 |
/// | `hits` | 6 | UINT8 (2) | 1 |
/// | `op` | 7 | UINT8 | 1 |
///
/// with ONE `N` for all three index fields (`1 <= N <= 1000`) and `point_step`
/// [`VOXEL_DELTA_POINT_STEP`]. Pure over the decoded descriptors, so it does not
/// depend on which `fields` framing the producer wrote.
///
/// A cloud with these fields has no `x`/`y`/`z` channel, so the point decoder
/// could never draw it; the layout is specific enough that no sensor cloud
/// matches it by accident.
pub fn voxel_delta_layout(fields: &[PointFieldDesc], point_step: u32) -> Option<VoxelDeltaLayout> {
    const INT16: u8 = 3;
    const UINT8: u8 = 2;
    if point_step != VOXEL_DELTA_POINT_STEP || fields.len() != VOXEL_DELTA_FIELD_COUNT {
        return None;
    }
    let shape_ok = |f: &PointFieldDesc, offset: u32, datatype: u8| {
        f.offset == offset && f.datatype == datatype && f.count == 1
    };
    let mut edge: Option<u16> = None;
    for (i, axis) in ['x', 'y', 'z'].into_iter().enumerate() {
        let f = &fields[i];
        let n = voxel_axis_edge_mm(&f.name, axis)?;
        if !shape_ok(f, 2 * i as u32, INT16) || edge.is_some_and(|e| e != n) {
            return None;
        }
        edge = Some(n);
    }
    let (hits, op) = (&fields[3], &fields[4]);
    if hits.name != "hits"
        || !shape_ok(hits, 6, UINT8)
        || op.name != "op"
        || !shape_ok(op, 7, UINT8)
    {
        return None;
    }
    edge.map(|edge_mm| VoxelDeltaLayout { edge_mm })
}

// ---- Wire ops ---------------------------------------------------------------

/// `op` code: the voxel is visible.
pub const OP_SET: u8 = 0;
/// `op` code: the voxel is no longer visible.
pub const OP_CLEAR: u8 = 1;
/// `op` code: empty the tile; its full content follows.
pub const OP_TILE: u8 = 2;
/// `op` code: a new epoch, empty everything.
pub const OP_RESET: u8 = 3;
/// `op` code: the robot's position and yaw.
pub const OP_ROBOT: u8 = 4;
/// `op` code: the epoch and the floor layer.
pub const OP_FLOOR: u8 = 5;
/// `op` code: closes a `TILE` group.
pub const OP_END_TILE: u8 = 6;

/// One decoded op (see the module docs for the table).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoxelOp {
    /// The voxel `(x, y, z)` is visible; `hits` is the producer's score.
    Set { x: i16, y: i16, z: i16, hits: u8 },
    /// The voxel `(x, y, z)` is no longer visible.
    Clear { x: i16, y: i16, z: i16 },
    /// Empty the tile holding column `(x, y)`; the `SET`s that follow are its content.
    Tile { x: i16, y: i16 },
    /// A new epoch (`epoch` = its low 16 bits): empty everything.
    Reset { epoch: u16 },
    /// The robot's position in voxels at this message's tick and its yaw in
    /// 1/256 turn (one per message; the last wins).
    Robot { x: i16, y: i16, z: i16, yaw: u8 },
    /// The epoch (low 16 bits) and the floor layer `floor_iz`.
    Floor { epoch: u16, floor_iz: i16 },
    /// Closes the current `TILE` group.
    EndTile,
    /// An `op` code this build does not know (counted and skipped).
    Unknown(u8),
}

/// One decoded voxel-delta message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VoxelMessage {
    /// The voxel edge in millimetres, from the field names.
    pub edge_mm: u16,
    /// The ops, in wire order.
    pub ops: Vec<VoxelOp>,
    /// `data` bytes not read as an op: the row padding of an organized cloud and
    /// a tail short of one 8-byte op (reported, never read).
    pub trailing_bytes: usize,
}

fn read_i16(b: &[u8], big_endian: bool) -> i16 {
    if big_endian {
        i16::from_be_bytes([b[0], b[1]])
    } else {
        i16::from_le_bytes([b[0], b[1]])
    }
}

/// Decode `n_ops` 8-byte ops from `data`, honouring `big_endian` (the cloud's
/// `is_bigendian`). Pure; an op past the end of `data` is not read.
pub fn decode_ops(data: &[u8], n_ops: usize, big_endian: bool) -> Vec<VoxelOp> {
    let step = VOXEL_DELTA_POINT_STEP as usize;
    data.chunks_exact(step)
        .take(n_ops)
        .map(|p| {
            let (x, y, z) = (
                read_i16(&p[0..2], big_endian),
                read_i16(&p[2..4], big_endian),
                read_i16(&p[4..6], big_endian),
            );
            let (hits, op) = (p[6], p[7]);
            match op {
                OP_SET => VoxelOp::Set { x, y, z, hits },
                OP_CLEAR => VoxelOp::Clear { x, y, z },
                OP_TILE => VoxelOp::Tile { x, y },
                OP_RESET => VoxelOp::Reset { epoch: x as u16 },
                OP_ROBOT => VoxelOp::Robot { x, y, z, yaw: hits },
                OP_FLOOR => VoxelOp::Floor {
                    epoch: x as u16,
                    floor_iz: z,
                },
                OP_END_TILE => VoxelOp::EndTile,
                other => VoxelOp::Unknown(other),
            }
        })
        .collect()
}

/// Decode the ops of a `width` x `height` cloud, honouring `row_step`: an
/// organized cloud (`height` > 1) may pad each row past `width * point_step`,
/// and padding is never an op. A `row_step` at or below the row's own bytes
/// (an unorganized cloud, or a malformed one) reads the rows back to back, as
/// [`point_count`] counts them. Pure. Returns the ops and the number of `data`
/// bytes not read as an op.
pub fn decode_rows(
    data: &[u8],
    width: u32,
    height: u32,
    row_step: u32,
    big_endian: bool,
) -> (Vec<VoxelOp>, usize) {
    let step = VOXEL_DELTA_POINT_STEP as usize;
    let row_bytes = (width as usize).saturating_mul(step);
    let row_step = row_step as usize;
    let ops = if height <= 1 || row_step <= row_bytes {
        let n = point_count(width, height, VOXEL_DELTA_POINT_STEP, data.len());
        decode_ops(data, n, big_endian)
    } else {
        // The declared geometry is wire input: never more capacity than the
        // bytes present can hold.
        let declared = (width as usize).saturating_mul(height as usize);
        let mut ops = Vec::with_capacity(declared.min(data.len() / step));
        for row in 0..height as usize {
            let start = row.saturating_mul(row_step);
            if start >= data.len() {
                break;
            }
            let end = start.saturating_add(row_bytes).min(data.len());
            ops.extend(decode_ops(&data[start..end], width as usize, big_endian));
        }
        ops
    };
    let unread = data.len() - ops.len() * step;
    (ops, unread)
}

fn field_u32(fv: &FrameValue, name: &str) -> Option<u32> {
    match fv.field(name) {
        Some(FrameValueKind::U32(v)) => Some(*v),
        _ => None,
    }
}

/// The `fields` descriptors of a PointCloud2 frame, from whichever framing the
/// producer wrote: the walker's decoded elements (the canonical element framing
/// the Go2 workspace nodes and the bridge's raw routes write), else the packed
/// layout [`parse_point_fields`] reads. `None` when neither decodes.
pub fn point_fields_of(fv: &FrameValue) -> Option<Vec<PointFieldDesc>> {
    match fv.field("fields")? {
        FrameValueKind::NestedArray { elements, raw } => {
            fields_from_elements(elements).or_else(|| parse_point_fields(raw).ok())
        }
        FrameValueKind::NestedArrayOpaque(b) | FrameValueKind::Bytes(b) => {
            parse_point_fields(b).ok()
        }
        _ => None,
    }
}

fn fields_from_elements(elements: &[FrameValueKind]) -> Option<Vec<PointFieldDesc>> {
    elements
        .iter()
        .map(|element| {
            let FrameValueKind::Nested(pf) = element else {
                return None;
            };
            let name = match pf.field("name")? {
                FrameValueKind::Str(s) => (*s).to_string(),
                _ => return None,
            };
            let offset = match pf.field("offset")? {
                FrameValueKind::U32(v) => *v,
                _ => return None,
            };
            let datatype = match pf.field("datatype")? {
                FrameValueKind::U8(v) => *v,
                _ => return None,
            };
            let count = match pf.field("count")? {
                FrameValueKind::U32(v) => *v,
                _ => return None,
            };
            Some(PointFieldDesc {
                name,
                offset,
                datatype,
                count,
            })
        })
        .collect()
}

/// The CONTENT rung of the archetype ladder: `Some` when `fv` is a
/// `sensor_msgs/PointCloud2` in the voxel-delta layout. Cheap for every other
/// message (a string compare), and O(fields) for a point cloud.
pub fn voxel_layout_of(fv: &FrameValue) -> Option<VoxelDeltaLayout> {
    if fv.schema_name != "sensor_msgs/PointCloud2" {
        return None;
    }
    let point_step = field_u32(fv, "point_step")?;
    if point_step != VOXEL_DELTA_POINT_STEP {
        return None;
    }
    // A sensor cloud (`x`/`y`/`z` and a few more) is rejected before its
    // descriptors are built: this rung runs on EVERY cloud frame, and the
    // descriptor read allocates. The packed framing has no count without a
    // parse; an element framing that decoded to nothing still falls through to it.
    if let Some(FrameValueKind::NestedArray { elements, .. }) = fv.field("fields") {
        if !elements.is_empty() && elements.len() != VOXEL_DELTA_FIELD_COUNT {
            return None;
        }
    }
    voxel_delta_layout(&point_fields_of(fv)?, point_step)
}

/// Decode a voxel-delta frame, or `None` when `fv` is not one.
pub fn decode_voxel_message(fv: &FrameValue) -> Option<VoxelMessage> {
    let layout = voxel_layout_of(fv)?;
    let data = match fv.field("data") {
        Some(FrameValueKind::Bytes(b)) => *b,
        _ => &[],
    };
    let width = field_u32(fv, "width").unwrap_or(0);
    let height = field_u32(fv, "height").unwrap_or(0);
    let row_step = field_u32(fv, "row_step").unwrap_or(0);
    let big_endian = matches!(fv.field("is_bigendian"), Some(FrameValueKind::Bool(true)));
    let (ops, trailing_bytes) = decode_rows(data, width, height, row_step, big_endian);
    Some(VoxelMessage {
        edge_mm: layout.edge_mm,
        ops,
        trailing_bytes,
    })
}

// ---- Look -------------------------------------------------------------------

/// Tile edge in columns (1.6 m at 5 cm).
pub const TILE_COLUMNS: i16 = 32;
/// The cubes child segment (the `-` keeps it out of every topic's namespace, as
/// for [`crate::sink::SWEEP_CHILD`]).
pub const CUBES_CHILD: &str = "viz-cubes";
/// The trail child segment.
pub const TRAIL_CHILD: &str = "viz-trail";
/// The wall-mesh child segment.
pub const WALLS_CHILD: &str = "viz-walls";
/// The wall-edge child segment.
pub const EDGES_CHILD: &str = "viz-edges";

/// Dirty cube tiles are drawn at most once per this much WIRE time: the producer
/// ticks every 500 ms, and the 50 ms slack absorbs its stamp jitter so every tick
/// still draws.
pub const CUBES_MIN_INTERVAL_NS: u64 = 450_000_000;
/// Dirty walls and edges are drawn at most once per this much wire time (1 s,
/// less the same slack).
pub const SURFACES_MIN_INTERVAL_NS: u64 = 950_000_000;

/// A new trail point when the robot has moved this far (XY) from the last one, mm.
pub const TRAIL_STEP_MM: i64 = 100;
/// The trail keeps the newest this-many points.
pub const TRAIL_MAX_POINTS: usize = 5_000;
/// The trail floats this far above the floor top, so it is never z-fought.
pub const TRAIL_LIFT_M: f32 = 0.02;
/// Trail colour, the warm accent `#FFB347`.
pub const TRAIL_RGB: [u8; 3] = [0xFF, 0xB3, 0x47];
/// Trail line width in UI points (rerun takes a radius: half of this).
pub const TRAIL_WIDTH_UI: f32 = 1.5;

/// A wall cell is this many columns on a side (10 cm at 5 cm).
pub const WALL_CELL_COLUMNS: i16 = 2;
/// Voxels whose centre is at least this high above the floor layer's centre
/// count toward a wall, mm.
pub const WALL_BAND_LOW_MM: i32 = 100;
/// ...and at most this high, mm.
pub const WALL_BAND_HIGH_MM: i32 = 1_800;
/// A wall's top is its highest voxel up to this height, mm.
pub const WALL_TOP_MAX_MM: i32 = 2_500;
/// A cell is a wall when it holds at least this many voxels in the band.
pub const WALL_MIN_VOXELS: usize = 3;
/// Wall side colour at the floor.
pub const WALL_SIDE_BOTTOM_RGB: [u8; 3] = [0x06, 0x28, 0x3D];
/// Wall side colour at the top.
pub const WALL_SIDE_TOP_RGB: [u8; 3] = [0x00, 0xA6, 0xE0];
/// Wall top colour.
pub const WALL_TOP_RGB: [u8; 3] = [0x7F, 0xE3, 0xFF];
/// Walls at least this tall (top above the floor top) get a top outline, mm.
pub const EDGE_MIN_HEIGHT_MM: i32 = 1_000;
/// Top-outline colour.
pub const EDGE_RGB: [u8; 3] = [0xBF, 0xF3, 0xFF];
/// Top-outline width in UI points (rerun takes a radius: half of this).
pub const EDGE_WIDTH_UI: f32 = 1.25;
/// The outline floats this far above the wall top.
pub const EDGE_LIFT_M: f32 = 0.002;
/// A tile whose wall mesh is bigger than this is not drawn (one warn).
pub const MAX_WALL_TRIANGLES_PER_TILE: usize = 8_000;
/// Walls stop being drawn past this many triangles in total (one warn).
pub const MAX_WALL_TRIANGLES_TOTAL: usize = 200_000;

// The height ramp restates the Cerulion design system colours the Go2 demo's map
// crate also carries: the framework cannot depend on a demo crate.
/// `ink-600`, floor voxels.
pub const FLOOR_RGB: [u8; 3] = [0x31, 0x41, 0x58];
/// `cerulean-400`.
pub const CERULEAN_RGB: [u8; 3] = [0x00, 0xC0, 0xFF];
/// `cerulean-500`.
pub const BLUE_RGB: [u8; 3] = [0x00, 0x80, 0xFF];
/// `ember-400`, the warm end.
pub const EMBER_RGB: [u8; 3] = [0xFF, 0x82, 0x1C];
/// Voxels up to this height above the floor layer are floor, m.
pub const FLOOR_BAND_M: f64 = 0.075;
const RAMP: [(f64, [u8; 3]); 5] = [
    (FLOOR_BAND_M, FLOOR_RGB),
    (0.30, CERULEAN_RGB),
    (1.20, BLUE_RGB),
    (1.80, EMBER_RGB),
    (f64::INFINITY, EMBER_RGB),
];

fn lerp_rgb(a: [u8; 3], b: [u8; 3], t: f64) -> [u8; 3] {
    let t = t.clamp(0.0, 1.0);
    [0, 1, 2].map(|i| (f64::from(a[i]) + (f64::from(b[i]) - f64::from(a[i])) * t).round() as u8)
}

/// The colour of a voxel whose centre is `h` metres above the floor layer's
/// centre: the floor (up to 7.5 cm, one voxel either side) is dark slate, then
/// slate to cerulean at 0.30 m, to blue at 1.20 m, to ember at 1.80 m and above.
pub fn height_rgb(h: f64) -> [u8; 3] {
    if h.is_nan() || h <= RAMP[0].0 {
        return FLOOR_RGB;
    }
    for pair in RAMP.windows(2) {
        let ((h0, c0), (h1, c1)) = (pair[0], pair[1]);
        if h <= h1 {
            if h1.is_infinite() {
                return c1;
            }
            return lerp_rgb(c0, c1, (h - h0) / (h1 - h0));
        }
    }
    EMBER_RGB
}

/// The tile holding column `(x, y)`.
pub fn tile_of(x: i16, y: i16) -> (i16, i16) {
    (x.div_euclid(TILE_COLUMNS), y.div_euclid(TILE_COLUMNS))
}

/// The entity segment of a tile: `t_<tx + 32768>_<ty + 32768>` (never negative).
pub fn tile_segment(tile: (i16, i16)) -> String {
    format!(
        "t_{}_{}",
        i32::from(tile.0) + 32_768,
        i32::from(tile.1) + 32_768
    )
}

// ---- Walls (pure geometry) -------------------------------------------------

/// One wall side: the neighbour cell's offset, the outward normal, and the
/// face's bottom edge `[ax, ay, bx, by]` (counter-clockwise seen from outside).
type Face = ((i32, i32), [f32; 3], [f64; 4]);

/// One tile's visible voxels, `(x, y, z) -> hits`.
pub type TileVoxels = BTreeMap<(i16, i16, i16), u8>;

/// A triangle mesh with flat per-vertex normals and vertex colours.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct WallMesh {
    /// Vertex positions, metres, in the map frame.
    pub positions: Vec<[f32; 3]>,
    /// Unit outward normal per vertex (flat: every vertex of a face shares it).
    pub normals: Vec<[f32; 3]>,
    /// RGB per vertex.
    pub colors: Vec<[u8; 3]>,
    /// Counter-clockwise (seen from outside) vertex index triples.
    pub triangles: Vec<[u32; 3]>,
}

impl WallMesh {
    fn quad(&mut self, corners: [[f32; 3]; 4], normal: [f32; 3], colors: [[u8; 3]; 4]) {
        let base = self.positions.len() as u32;
        self.positions.extend_from_slice(&corners);
        self.normals.extend_from_slice(&[normal; 4]);
        self.colors.extend_from_slice(&colors);
        self.triangles.push([base, base + 1, base + 2]);
        self.triangles.push([base, base + 2, base + 3]);
    }
}

/// The wall cells of one tile's voxels: `cell -> the top layer index`. A cell is
/// [`WALL_CELL_COLUMNS`] by [`WALL_CELL_COLUMNS`] columns; it is a wall when at least [`WALL_MIN_VOXELS`]
/// of its voxels sit between [`WALL_BAND_LOW_MM`] and [`WALL_BAND_HIGH_MM`] above
/// the floor layer, and its top is its highest voxel up to [`WALL_TOP_MAX_MM`].
/// Heights are compared in whole millimetres, so no threshold depends on float
/// rounding.
pub fn wall_cells(voxels: &TileVoxels, floor_iz: i16, edge_mm: u16) -> BTreeMap<(i32, i32), i32> {
    let mut stats: BTreeMap<(i32, i32), (usize, i32)> = BTreeMap::new();
    for &(x, y, z) in voxels.keys() {
        let h_mm = (i32::from(z) - i32::from(floor_iz)) * i32::from(edge_mm);
        if !(WALL_BAND_LOW_MM..=WALL_TOP_MAX_MM).contains(&h_mm) {
            continue;
        }
        let cell = (
            i32::from(x.div_euclid(WALL_CELL_COLUMNS)),
            i32::from(y.div_euclid(WALL_CELL_COLUMNS)),
        );
        let entry = stats.entry(cell).or_insert((0, i32::MIN));
        if h_mm <= WALL_BAND_HIGH_MM {
            entry.0 += 1;
        }
        entry.1 = entry.1.max(i32::from(z));
    }
    stats
        .into_iter()
        .filter(|(_, (count, _))| *count >= WALL_MIN_VOXELS)
        .map(|(cell, (_, top))| (cell, top))
        .collect()
}

/// The wall mesh and top outlines of `cells` (one tile's [`wall_cells`]).
/// `neighbour_top` answers for ANY cell, including those of the next tile, so a
/// side is drawn only where the neighbour is lower and the tiles meet without
/// seams. The extrusion runs from the floor top (the top face of layer
/// `floor_iz`) to the top face of the cell's top voxel.
pub fn wall_geometry(
    cells: &BTreeMap<(i32, i32), i32>,
    neighbour_top: impl Fn((i32, i32)) -> Option<i32>,
    floor_iz: i16,
    edge_mm: u16,
) -> (WallMesh, Vec<[[f32; 3]; 2]>) {
    let v = f64::from(edge_mm) / 1000.0;
    let cell_m = v * f64::from(WALL_CELL_COLUMNS);
    let z0 = (f64::from(floor_iz) + 1.0) * v;
    let mut mesh = WallMesh::default();
    let mut edges: Vec<[[f32; 3]; 2]> = Vec::new();
    for (&(cx, cy), &top) in cells {
        let z1 = (f64::from(top) + 1.0) * v;
        let (x0, y0) = (f64::from(cx) * cell_m, f64::from(cy) * cell_m);
        let (x1, y1) = (x0 + cell_m, y0 + cell_m);
        let p = |x: f64, y: f64, z: f64| [x as f32, y as f32, z as f32];
        let side_rgb = |z: f64| {
            lerp_rgb(
                WALL_SIDE_BOTTOM_RGB,
                WALL_SIDE_TOP_RGB,
                (z - z0) / (z1 - z0),
            )
        };
        mesh.quad(
            [p(x0, y0, z1), p(x1, y0, z1), p(x1, y1, z1), p(x0, y1, z1)],
            [0.0, 0.0, 1.0],
            [WALL_TOP_RGB; 4],
        );
        let tall = (top - i32::from(floor_iz)) * i32::from(edge_mm) >= EDGE_MIN_HEIGHT_MM;
        let lift = f64::from(EDGE_LIFT_M);
        // (neighbour offset, outward normal, the face's bottom edge in
        // counter-clockwise order seen from outside).
        let faces: [Face; 4] = [
            ((1, 0), [1.0, 0.0, 0.0], [x1, y0, x1, y1]),
            ((-1, 0), [-1.0, 0.0, 0.0], [x0, y1, x0, y0]),
            ((0, 1), [0.0, 1.0, 0.0], [x1, y1, x0, y1]),
            ((0, -1), [0.0, -1.0, 0.0], [x0, y0, x1, y0]),
        ];
        for ((dx, dy), normal, [ax, ay, bx, by]) in faces {
            let n_top = neighbour_top((cx + dx, cy + dy));
            let zb = match n_top {
                Some(n) if n >= top => continue,
                Some(n) => ((f64::from(n) + 1.0) * v).max(z0),
                None => z0,
            };
            mesh.quad(
                [p(ax, ay, zb), p(bx, by, zb), p(bx, by, z1), p(ax, ay, z1)],
                normal,
                [side_rgb(zb), side_rgb(zb), side_rgb(z1), side_rgb(z1)],
            );
            if tall {
                edges.push([p(ax, ay, z1 + lift), p(bx, by, z1 + lift)]);
            }
        }
    }
    (mesh, edges)
}

// ---- State ------------------------------------------------------------------

/// One static log call the voxel map makes: the pure output of
/// [`VoxelMapState::apply`], executed by [`execute`].
#[derive(Debug, Clone, PartialEq)]
pub enum LogAction {
    /// A recursive static `Clear` at `entity` (a new epoch).
    ClearRecursive { entity: String },
    /// A flat static `Clear` at `entity` (a tile, trail or surface became empty).
    ClearFlat { entity: String },
    /// A static `CoordinateFrame` at `entity`.
    Frame { entity: String, frame: String },
    /// One tile's cubes: a `VoxelGridMap` with the minimum corner of index
    /// `[0, 0, 0]` at `translation`.
    Cubes {
        entity: String,
        translation: [f32; 3],
        voxel_size: f32,
        indices: Vec<[i32; 3]>,
        colors: Vec<[u8; 3]>,
    },
    /// The walked trail, one strip.
    Trail {
        entity: String,
        points: Vec<[f32; 3]>,
    },
    /// One tile's wall mesh.
    Walls { entity: String, mesh: WallMesh },
    /// One tile's wall top outlines, one two-point strip each.
    Edges {
        entity: String,
        strips: Vec<[[f32; 3]; 2]>,
    },
}

/// Running counters (observability and test seam).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct VoxelMapCounters {
    /// Messages applied.
    pub messages: u64,
    /// Ops applied.
    pub ops: u64,
    /// `RESET` ops.
    pub resets: u64,
    /// `FLOOR` ops whose epoch differed from the held one (a lost `RESET`, healed).
    pub healed_resets: u64,
    /// Ops with an unknown code (skipped).
    pub unknown_ops: u64,
    /// `TILE` ops whose column was not a multiple of [`TILE_COLUMNS`].
    pub misaligned_tiles: u64,
    /// `ROBOT` ops beyond the first in one message (the last wins; see the
    /// module docs).
    pub extra_robot_ops: u64,
    /// `TILE` groups a message ended before their `END_TILE` (a group must be
    /// whole within one message; see the module docs).
    pub split_tile_groups: u64,
    /// Wall tiles not drawn because of a triangle budget.
    pub wall_tiles_over_budget: u64,
    /// Messages whose `data` carried bytes past the last whole op (ignored).
    pub trailing_byte_messages: u64,
}

/// The per-input state of one voxel-map topic (see the module docs).
#[derive(Debug, Clone)]
pub struct VoxelMapState {
    edge_mm: Option<u16>,
    epoch: Option<u16>,
    floor_iz: Option<i16>,
    tiles: BTreeMap<(i16, i16), TileVoxels>,
    dirty_cubes: BTreeSet<(i16, i16)>,
    dirty_surfaces: BTreeSet<(i16, i16)>,
    /// Trail points as voxel columns (exact; metres are derived at draw time).
    trail: VecDeque<(i16, i16)>,
    trail_dirty: bool,
    robot: Option<(i16, i16, i16, u8)>,
    last_cubes_ns: Option<u64>,
    last_surfaces_ns: Option<u64>,
    /// A `TILE` group is open: its `END_TILE` has not arrived in this message.
    tile_open: bool,
    /// The `ROBOT` of the message being applied (the last one wins).
    msg_robot: Option<(i16, i16, i16, u8)>,
    /// Entities holding static data in the viewer right now, with the payload
    /// last logged there: rerun's static store APPENDS every write, so an
    /// unchanged payload is never logged again (the `tf_static` rule).
    logged: BTreeMap<String, LogAction>,
    /// The static `CoordinateFrame` logged per entity.
    framed: BTreeMap<String, String>,
    /// The frame the last message resolved to.
    frame: Option<String>,
    wall_triangles: BTreeMap<(i16, i16), usize>,
    /// Wall tiles held back by the TOTAL triangle budget; drawn again as soon as
    /// a surfaces pass lowers the total.
    budget_held: BTreeSet<(i16, i16)>,
    /// A recursive `Clear` is owed before the next draw (a fresh state or a new
    /// epoch), so the viewer never mixes epochs.
    needs_clear: bool,
    /// Messages were applied WITHOUT being drawn ([`Self::apply_hidden`]), so
    /// the viewer's picture of this map is unknown: the next drawn message
    /// clears the four children and draws everything again.
    redraw_pending: bool,
    budget_warned: bool,
    unknown_warned: bool,
    trailing_warned: bool,
    counters: VoxelMapCounters,
}

impl Default for VoxelMapState {
    fn default() -> Self {
        Self {
            edge_mm: None,
            epoch: None,
            floor_iz: None,
            tiles: BTreeMap::new(),
            dirty_cubes: BTreeSet::new(),
            dirty_surfaces: BTreeSet::new(),
            trail: VecDeque::new(),
            trail_dirty: false,
            robot: None,
            last_cubes_ns: None,
            last_surfaces_ns: None,
            tile_open: false,
            msg_robot: None,
            logged: BTreeMap::new(),
            framed: BTreeMap::new(),
            frame: None,
            wall_triangles: BTreeMap::new(),
            budget_held: BTreeSet::new(),
            // A fresh viewer state starts from a clean slate: whatever an earlier
            // daemon drew under this entity belongs to an epoch this state never saw.
            needs_clear: true,
            redraw_pending: false,
            budget_warned: false,
            unknown_warned: false,
            trailing_warned: false,
            counters: VoxelMapCounters::default(),
        }
    }
}

fn gate_open(last: Option<u64>, now: u64, min_interval_ns: u64) -> bool {
    match last {
        None => true,
        // A stamp that went BACKWARDS (a replay loop, a restarted producer) opens
        // the gate rather than stalling it until wire time catches up.
        Some(last) => now < last || now - last >= min_interval_ns,
    }
}

impl VoxelMapState {
    /// A fresh state (the first message clears the entity: see the module docs).
    pub fn new() -> Self {
        Self::default()
    }

    /// The number of visible voxels this state holds.
    pub fn visible_count(&self) -> usize {
        self.tiles.values().map(BTreeMap::len).sum()
    }

    /// Whether voxel `(x, y, z)` is visible.
    pub fn contains(&self, x: i16, y: i16, z: i16) -> bool {
        self.tiles
            .get(&tile_of(x, y))
            .is_some_and(|t| t.contains_key(&(x, y, z)))
    }

    /// The epoch held (low 16 bits), `None` before the first `FLOOR`/`RESET`.
    pub fn epoch(&self) -> Option<u16> {
        self.epoch
    }

    /// The floor layer held.
    pub fn floor_iz(&self) -> Option<i16> {
        self.floor_iz
    }

    /// The trail's XY points in metres (voxel column centres), oldest first.
    pub fn trail(&self) -> Vec<[f32; 2]> {
        let v = self.voxel_m();
        self.trail
            .iter()
            .map(|&(x, y)| {
                [
                    ((f64::from(x) + 0.5) * v) as f32,
                    ((f64::from(y) + 0.5) * v) as f32,
                ]
            })
            .collect()
    }

    /// The running counters.
    pub fn counters(&self) -> VoxelMapCounters {
        self.counters
    }

    /// A viewer RECONNECT: the new server holds nothing, so every tile, the trail
    /// and every frame assignment are drawn again on the next message.
    pub fn rearm(&mut self) {
        self.logged.clear();
        self.framed.clear();
        self.wall_triangles.clear();
        self.budget_held.clear();
        let tiles: Vec<(i16, i16)> = self.tiles.keys().copied().collect();
        self.dirty_cubes.extend(tiles.iter().copied());
        self.dirty_surfaces.extend(tiles);
        self.trail_dirty = !self.trail.is_empty();
        self.last_cubes_ns = None;
        self.last_surfaces_ns = None;
    }

    fn voxel_m(&self) -> f64 {
        f64::from(self.edge_mm.unwrap_or(50)) / 1000.0
    }

    fn floor_top_m(&self) -> f64 {
        (f64::from(self.floor_iz.unwrap_or(0)) + 1.0) * self.voxel_m()
    }

    fn reset_contents(&mut self) {
        self.tiles.clear();
        self.dirty_cubes.clear();
        self.dirty_surfaces.clear();
        self.trail.clear();
        self.trail_dirty = false;
        self.robot = None;
        // A ROBOT read earlier in this message belongs to the epoch being
        // dropped: it must not seed the new trail.
        self.msg_robot = None;
        self.floor_iz = None;
        self.last_cubes_ns = None;
        self.last_surfaces_ns = None;
        self.logged.clear();
        self.framed.clear();
        self.wall_triangles.clear();
        self.budget_held.clear();
        self.needs_clear = true;
    }

    fn mark_all_dirty(&mut self) {
        let tiles: Vec<(i16, i16)> = self.tiles.keys().copied().collect();
        self.dirty_cubes.extend(tiles.iter().copied());
        self.dirty_surfaces.extend(tiles);
        self.trail_dirty = !self.trail.is_empty();
    }

    fn mark_dirty(&mut self, tile: (i16, i16)) {
        self.dirty_cubes.insert(tile);
        // A wall side at a tile border depends on the neighbour tile's cells.
        for (dx, dy) in [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)] {
            self.dirty_surfaces
                .insert((tile.0.saturating_add(dx), tile.1.saturating_add(dy)));
        }
    }

    fn apply_op(&mut self, op: VoxelOp) {
        self.counters.ops += 1;
        match op {
            VoxelOp::Set { x, y, z, hits } => {
                let tile = tile_of(x, y);
                if self
                    .tiles
                    .entry(tile)
                    .or_default()
                    .insert((x, y, z), hits)
                    .is_none()
                {
                    self.mark_dirty(tile);
                }
            }
            VoxelOp::Clear { x, y, z } => {
                let tile = tile_of(x, y);
                if let Some(t) = self.tiles.get_mut(&tile) {
                    if t.remove(&(x, y, z)).is_some() {
                        if t.is_empty() {
                            self.tiles.remove(&tile);
                        }
                        self.mark_dirty(tile);
                    }
                }
            }
            VoxelOp::Tile { x, y } => {
                if x.rem_euclid(TILE_COLUMNS) != 0 || y.rem_euclid(TILE_COLUMNS) != 0 {
                    self.counters.misaligned_tiles += 1;
                }
                let tile = tile_of(x, y);
                self.tiles.remove(&tile);
                self.mark_dirty(tile);
                self.tile_open = true;
            }
            VoxelOp::EndTile => self.tile_open = false,
            VoxelOp::Reset { epoch } => {
                self.counters.resets += 1;
                // A RESET for the epoch already held is a duplicate: every op is
                // idempotent, so it empties nothing.
                if self.epoch != Some(epoch) {
                    self.reset_contents();
                    self.epoch = Some(epoch);
                }
            }
            VoxelOp::Floor { epoch, floor_iz } => {
                if self.epoch.is_some_and(|held| held != epoch) {
                    // A lost RESET: this viewer's voxels belong to an older epoch.
                    self.counters.healed_resets += 1;
                    self.reset_contents();
                }
                self.epoch = Some(epoch);
                if self.floor_iz != Some(floor_iz) {
                    self.floor_iz = Some(floor_iz);
                    // Every colour and every wall is relative to the floor.
                    self.mark_all_dirty();
                }
            }
            VoxelOp::Robot { x, y, z, yaw } => {
                // The position at this message's tick: applied once the message
                // is read, so a message with several ROBOT ops, replayed, cannot
                // draw a trip between them.
                if self.msg_robot.replace((x, y, z, yaw)).is_some() {
                    self.counters.extra_robot_ops += 1;
                }
            }
            VoxelOp::Unknown(code) => {
                self.counters.unknown_ops += 1;
                if !self.unknown_warned {
                    self.unknown_warned = true;
                    tracing::warn!(
                        op = code,
                        "cerulion_viz: voxel map carries an op code this build does not know; \
                         skipping it (reported once per topic)"
                    );
                }
            }
        }
    }

    /// The message's `ROBOT`: a new trail point when the robot moved
    /// [`TRAIL_STEP_MM`] or more (XY) from the last one.
    fn step_trail(&mut self, (x, y, z, yaw): (i16, i16, i16, u8)) {
        self.robot = Some((x, y, z, yaw));
        // Whole voxel columns times the edge in mm: exact integers, so the
        // step never depends on float rounding far from the origin.
        let edge = i64::from(self.edge_mm.unwrap_or(50));
        let far = self.trail.back().is_none_or(|&(lx, ly)| {
            let dx = (i64::from(x) - i64::from(lx)) * edge;
            let dy = (i64::from(y) - i64::from(ly)) * edge;
            dx * dx + dy * dy >= TRAIL_STEP_MM * TRAIL_STEP_MM
        });
        if far {
            self.trail.push_back((x, y));
            while self.trail.len() > TRAIL_MAX_POINTS {
                self.trail.pop_front();
            }
            self.trail_dirty = true;
        }
    }

    /// Apply one message and return the static log calls that bring the viewer
    /// up to date. `root` is the topic entity, `frame` the message's resolved
    /// coordinate frame, `stamp_ns` its WIRE stamp (the only clock read).
    pub fn apply(
        &mut self,
        root: &str,
        frame: Option<&str>,
        msg: &VoxelMessage,
        stamp_ns: u64,
    ) -> Vec<LogAction> {
        self.track(msg);
        self.draw_pending(root, frame, stamp_ns)
    }

    /// Apply one message WITHOUT drawing it. The operator suppressed this map's
    /// visual half (a `Text` representation), but the set must keep following
    /// the stream: it is a DELTA stream whose `CLEAR`s are never re-sent, so a
    /// message that is not applied is a voxel the map holds forever, and the
    /// epoch, floor and trail move on without it. Nothing is logged. The viewer's
    /// picture of this map is unknown from here on (it still shows the tiles as
    /// they were when the visual half was suppressed), so the next DRAWN message
    /// starts with a recursive `Clear` of the four children and draws every
    /// tile, the trail and their frames again.
    pub fn apply_hidden(&mut self, msg: &VoxelMessage) {
        self.track(msg);
        self.redraw_pending = true;
    }

    /// The state half of [`Self::apply`]: count the message, apply its ops to
    /// the set and step the trail. Draws nothing.
    fn track(&mut self, msg: &VoxelMessage) {
        self.counters.messages += 1;
        if msg.trailing_bytes > 0 {
            self.counters.trailing_byte_messages += 1;
            if !self.trailing_warned {
                self.trailing_warned = true;
                tracing::warn!(
                    trailing_bytes = msg.trailing_bytes,
                    "cerulion_viz: voxel-map message has `data` bytes past its last whole op (or \
                     width x height disagrees with the data); they are ignored (reported once \
                     per topic)"
                );
            }
        }
        if self.edge_mm.is_some_and(|e| e != msg.edge_mm) {
            // Another voxel size: nothing held can be drawn at the new scale.
            self.reset_contents();
            self.epoch = None;
        }
        self.edge_mm = Some(msg.edge_mm);
        for &op in &msg.ops {
            self.apply_op(op);
        }
        if std::mem::take(&mut self.tile_open) {
            self.counters.split_tile_groups += 1;
        }
        if let Some(robot) = self.msg_robot.take() {
            self.step_trail(robot);
        }
    }

    /// The drawing half of [`Self::apply`]: the static log calls that bring
    /// the viewer from what it holds to what the set now says.
    fn draw_pending(&mut self, root: &str, frame: Option<&str>, stamp_ns: u64) -> Vec<LogAction> {
        if std::mem::take(&mut self.redraw_pending) {
            // Messages were applied unseen: the viewer still shows the tiles from
            // before, which may hold voxels since cleared and tiles since emptied
            // (nothing cleared them there). Same recovery as a reconnect, plus
            // the Clear a reconnect does not need: the four children are wiped
            // and everything held is drawn again, with the cadence gates open.
            self.rearm();
            self.needs_clear = true;
        }
        let mut actions = Vec::new();
        if std::mem::take(&mut self.needs_clear) {
            // One recursive Clear per map-owned child, never at `root` itself:
            // `root` is the topic entity, and another attached topic whose path
            // nests under this one renders at a descendant of it (the daemon
            // places topics by path). A Clear at `root` would wipe that topic's
            // statics on every epoch and on the first frame after a restart. The
            // `-` in the child segments keeps every topic out of them.
            for child in [CUBES_CHILD, WALLS_CHILD, EDGES_CHILD, TRAIL_CHILD] {
                actions.push(LogAction::ClearRecursive {
                    entity: format!("{root}/{child}"),
                });
            }
        }
        if self.frame.as_deref() != frame {
            self.frame = frame.map(str::to_string);
            let logged: Vec<String> = self.logged.keys().cloned().collect();
            for entity in logged {
                self.ensure_frame(&entity, &mut actions);
            }
        }
        if !self.dirty_cubes.is_empty()
            && gate_open(self.last_cubes_ns, stamp_ns, CUBES_MIN_INTERVAL_NS)
        {
            self.last_cubes_ns = Some(stamp_ns);
            for tile in std::mem::take(&mut self.dirty_cubes) {
                self.draw_cubes(root, tile, &mut actions);
            }
        }
        if !self.dirty_surfaces.is_empty()
            && gate_open(self.last_surfaces_ns, stamp_ns, SURFACES_MIN_INTERVAL_NS)
        {
            self.last_surfaces_ns = Some(stamp_ns);
            let dirty = std::mem::take(&mut self.dirty_surfaces);
            self.draw_surfaces(root, &dirty, &mut actions);
        }
        if std::mem::take(&mut self.trail_dirty) {
            self.draw_trail(root, &mut actions);
        }
        actions
    }

    fn ensure_frame(&mut self, entity: &str, actions: &mut Vec<LogAction>) {
        let want = match &self.frame {
            Some(frame) => Some(frame.clone()),
            // Un-posed now but posed before: point it back at its own frame.
            None => self
                .framed
                .contains_key(entity)
                .then(|| implicit_frame_of(entity)),
        };
        if let Some(want) = want {
            if self.framed.get(entity) != Some(&want) {
                self.framed.insert(entity.to_string(), want.clone());
                actions.push(LogAction::Frame {
                    entity: entity.to_string(),
                    frame: want,
                });
            }
        }
    }

    /// Log `action` at `entity`, or a flat `Clear` when there is nothing to draw.
    /// An action whose payload equals the one last logged there is skipped: the
    /// viewer already shows it, and a static write is an APPEND in its store.
    fn draw(&mut self, entity: String, action: Option<LogAction>, actions: &mut Vec<LogAction>) {
        match action {
            Some(action) => {
                self.ensure_frame(&entity, actions);
                if self.logged.get(&entity) != Some(&action) {
                    actions.push(action.clone());
                    self.logged.insert(entity, action);
                }
            }
            None => {
                if self.logged.remove(&entity).is_some() {
                    // A flat Clear also shadows the entity's frame assignment.
                    self.framed.remove(&entity);
                    actions.push(LogAction::ClearFlat { entity });
                }
            }
        }
    }

    fn draw_cubes(&mut self, root: &str, tile: (i16, i16), actions: &mut Vec<LogAction>) {
        let entity = format!("{root}/{CUBES_CHILD}/{}", tile_segment(tile));
        let action = self
            .tiles
            .get(&tile)
            .filter(|t| !t.is_empty())
            .map(|voxels| {
                let edge_mm = self.edge_mm.unwrap_or(50);
                let v = f64::from(edge_mm) / 1000.0;
                let floor_iz = i32::from(self.floor_iz.unwrap_or(0));
                let (ox, oy) = (
                    i32::from(tile.0) * i32::from(TILE_COLUMNS),
                    i32::from(tile.1) * i32::from(TILE_COLUMNS),
                );
                let mut indices = Vec::with_capacity(voxels.len());
                let mut colors = Vec::with_capacity(voxels.len());
                for &(x, y, z) in voxels.keys() {
                    indices.push([i32::from(x) - ox, i32::from(y) - oy, i32::from(z)]);
                    let h = f64::from((i32::from(z) - floor_iz) * i32::from(edge_mm)) / 1000.0;
                    colors.push(height_rgb(h));
                }
                LogAction::Cubes {
                    entity: entity.clone(),
                    translation: [(f64::from(ox) * v) as f32, (f64::from(oy) * v) as f32, 0.0],
                    voxel_size: v as f32,
                    indices,
                    colors,
                }
            });
        self.draw(entity, action, actions);
    }

    fn draw_surfaces(
        &mut self,
        root: &str,
        dirty: &BTreeSet<(i16, i16)>,
        actions: &mut Vec<LogAction>,
    ) {
        let edge_mm = self.edge_mm.unwrap_or(50);
        let floor_iz = self.floor_iz.unwrap_or(0);
        // Every tile a dirty tile's borders read, computed once for the pass.
        let mut cells: BTreeMap<(i16, i16), BTreeMap<(i32, i32), i32>> = BTreeMap::new();
        for &tile in dirty {
            for (dx, dy) in [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)] {
                let t = (tile.0.saturating_add(dx), tile.1.saturating_add(dy));
                cells.entry(t).or_insert_with(|| {
                    self.tiles
                        .get(&t)
                        .map(|voxels| wall_cells(voxels, floor_iz, edge_mm))
                        .unwrap_or_default()
                });
            }
        }
        let cells_per_tile = i32::from(TILE_COLUMNS / WALL_CELL_COLUMNS);
        let total_before: usize = self.wall_triangles.values().sum();
        for &tile in dirty {
            let own = &cells[&tile];
            let neighbour_top = |c: (i32, i32)| {
                let t = (
                    c.0.div_euclid(cells_per_tile) as i16,
                    c.1.div_euclid(cells_per_tile) as i16,
                );
                cells.get(&t).and_then(|m| m.get(&c).copied())
            };
            let (mut mesh, mut edges) = wall_geometry(own, neighbour_top, floor_iz, edge_mm);
            let others: usize = self
                .wall_triangles
                .iter()
                .filter(|(t, _)| **t != tile)
                .map(|(_, n)| *n)
                .sum();
            let n = mesh.triangles.len();
            let over_tile = n > MAX_WALL_TRIANGLES_PER_TILE;
            // Only the TOTAL budget can free up later; a tile over its own cap
            // stays out until its content changes.
            let over_total = !over_tile && others + n > MAX_WALL_TRIANGLES_TOTAL;
            if over_total {
                self.budget_held.insert(tile);
            } else {
                self.budget_held.remove(&tile);
            }
            if over_tile || over_total {
                self.counters.wall_tiles_over_budget += 1;
                if !self.budget_warned {
                    self.budget_warned = true;
                    tracing::warn!(
                        triangles = n,
                        total = others + n,
                        per_tile_cap = MAX_WALL_TRIANGLES_PER_TILE,
                        total_cap = MAX_WALL_TRIANGLES_TOTAL,
                        "cerulion_viz: voxel-map walls are over the triangle budget; the tile is \
                         not drawn as a surface (its cubes still are). Reported once per topic"
                    );
                }
                mesh = WallMesh::default();
                edges.clear();
            }
            self.wall_triangles.insert(tile, mesh.triangles.len());
            let segment = tile_segment(tile);
            let walls_entity = format!("{root}/{WALLS_CHILD}/{segment}");
            let walls = (!mesh.triangles.is_empty()).then(|| LogAction::Walls {
                entity: walls_entity.clone(),
                mesh,
            });
            self.draw(walls_entity, walls, actions);
            let edges_entity = format!("{root}/{EDGES_CHILD}/{segment}");
            let edges = (!edges.is_empty()).then(|| LogAction::Edges {
                entity: edges_entity.clone(),
                strips: edges,
            });
            self.draw(edges_entity, edges, actions);
        }
        // Capacity freed: the tiles the total budget held back get another
        // pass at the next surfaces gate. A pass that frees nothing re-queues
        // nothing, so a map that stays over budget settles.
        let total_after: usize = self.wall_triangles.values().sum();
        if total_after < total_before {
            self.dirty_surfaces.extend(self.budget_held.iter().copied());
        }
    }

    fn draw_trail(&mut self, root: &str, actions: &mut Vec<LogAction>) {
        let entity = format!("{root}/{TRAIL_CHILD}");
        let z = (self.floor_top_m() + f64::from(TRAIL_LIFT_M)) as f32;
        let action = (!self.trail.is_empty()).then(|| LogAction::Trail {
            entity: entity.clone(),
            points: self.trail().into_iter().map(|p| [p[0], p[1], z]).collect(),
        });
        self.draw(entity, action, actions);
    }
}

// ---- Logging ----------------------------------------------------------------

fn color(rgb: [u8; 3]) -> rerun::Color {
    rerun::Color::from_rgb(rgb[0], rgb[1], rgb[2])
}

/// Execute `actions` on `rec`, every one `log_static`. A failed log is a
/// serialization error for that one action: it is warned and the rest still run.
/// The sink buffers, so a viewer that went away never surfaces here; a reconnect
/// is healed by [`VoxelMapState::rearm`], not by a retry.
pub fn execute(rec: &RecordingStream, actions: &[LogAction]) {
    for action in actions {
        let (entity, result) = match action {
            LogAction::ClearRecursive { entity } => (
                entity,
                rec.log_static(entity.as_str(), &rerun::Clear::recursive()),
            ),
            LogAction::ClearFlat { entity } => (
                entity,
                rec.log_static(entity.as_str(), &rerun::Clear::flat()),
            ),
            LogAction::Frame { entity, frame } => (
                entity,
                rec.log_static(
                    entity.as_str(),
                    &rerun::CoordinateFrame::new(frame.as_str()),
                ),
            ),
            LogAction::Cubes {
                entity,
                translation,
                voxel_size,
                indices,
                colors,
            } => {
                let map = rerun::VoxelGridMap::new(
                    indices.iter().map(|i| (i[0], i[1], i[2])),
                    [*voxel_size; 3],
                )
                .with_translation(*translation)
                .with_colors(colors.iter().map(|c| color(*c)));
                (entity, rec.log_static(entity.as_str(), &map))
            }
            LogAction::Trail { entity, points } => {
                let strip = rerun::LineStrips3D::new([points.iter().copied()])
                    .with_colors([color(TRAIL_RGB)])
                    .with_radii([rerun::Radius::new_ui_points(TRAIL_WIDTH_UI / 2.0)]);
                (entity, rec.log_static(entity.as_str(), &strip))
            }
            LogAction::Walls { entity, mesh } => {
                let m = rerun::Mesh3D::new(mesh.positions.iter().copied())
                    .with_vertex_normals(mesh.normals.iter().copied())
                    .with_vertex_colors(mesh.colors.iter().map(|c| color(*c)))
                    .with_triangle_indices(mesh.triangles.iter().copied());
                (entity, rec.log_static(entity.as_str(), &m))
            }
            LogAction::Edges { entity, strips } => {
                let lines = rerun::LineStrips3D::new(strips.iter().map(|s| s.to_vec()))
                    .with_colors([color(EDGE_RGB)])
                    .with_radii([rerun::Radius::new_ui_points(EDGE_WIDTH_UI / 2.0)]);
                (entity, rec.log_static(entity.as_str(), &lines))
            }
        };
        if let Err(e) = result {
            tracing::warn!(error = %e, entity = %entity, "Rerun: voxel map log failed");
        }
    }
}
