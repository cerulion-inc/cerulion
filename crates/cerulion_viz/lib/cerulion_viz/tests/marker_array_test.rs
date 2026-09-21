// SPDX-License-Identifier: AGPL-3.0-only
//! `visualization_msgs/MarkerArray` — the 13 kinds and the
//! DELETE / DELETEALL entity CLEAR, over HAND-BUILT canonical wire frames.
//!
//! Two layers, asserted separately, both against hand oracles:
//!
//! 1. the PURE extractor ([`scan_marker_array`]) — a frame in, an exact
//!    [`MarkerGeometry`] out, with the numbers written by hand (a `SPHERE` with
//!    `scale = [2, 4, 6]` must yield `half_size = [1, 2, 3]`, and that 2 is
//!    written here, not derived from the code under test);
//! 2. the RENDER + CLEAR path — real `dispatch_frame` calls into a rerun memory
//!    sink, read back as `(entity path, archetype short names)` off the REAL
//!    store, which is what a viewer would see.
//!
//! Every "draws" assertion is paired with a negative (a frame that must draw
//! NOTHING), so a test cannot pass merely because the apparatus moved.
//!
//! No iceoryx2, no transport, no daemon — parallel-safe.

use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::sync::{Mutex, OnceLock};

use cerulion_core::codegen::layout::{LayoutResolver, WireLayout};
use cerulion_core::codegen::{parse_rosmsg, FrameWalker, MessageSchema};
use cerulion_core::message::ShmMessage;
use cerulion_core::shm_runtime::write_offset_entry;
use cerulion_core::wire::WireHeader;
use cerulion_viz::marker::{
    marker_entity, scan_marker_array, MarkerAction, MarkerArrayScan, MarkerGeometry, MarkerKind,
    MARKER_CHILD, MAX_LIVE_MARKERS, MAX_MARKER_INSTANCES, MAX_MARKER_VERTICES,
};
use cerulion_viz::schema_registry::builtin_walker;
use cerulion_viz::sink::{
    dispatch_frame, dispatch_or_stage, route_for_input, ArchetypeKind, SinkState,
};
use native_ros2_messages::visualization_msgs::MarkerArray;
use tracing_test::traced_test;

// ────────────────────────────────────────────────────────────────────────────
// Frame builders. Every one asserts the layout facts it depends on, so a
// re-vendor of `Marker.msg` (the upstream-drift fix did exactly that mid-issue) fails LOUDLY
// here instead of silently producing a frame that decodes to something else.
// ────────────────────────────────────────────────────────────────────────────

const MARKER: &str = "visualization_msgs/Marker";

fn all_schemas() -> Vec<MessageSchema> {
    native_ros2_messages::BUILTIN_MSGS
        .iter()
        .filter_map(|(pkg, name, text)| parse_rosmsg(text, name, Some(pkg)).ok())
        .collect()
}

/// Resolved layouts, MEMOIZED. Resolving parses the whole built-in corpus, and
/// the cap tests build ten thousand marker bodies — doing it per body turns a
/// millisecond test into a multi-minute one.
fn layout_of(qname: &str) -> WireLayout {
    static CACHE: OnceLock<Mutex<HashMap<String, WireLayout>>> = OnceLock::new();
    let cache = CACHE.get_or_init(|| Mutex::new(HashMap::new()));
    let mut cache = cache.lock().expect("layout cache");
    cache
        .entry(qname.to_string())
        .or_insert_with(|| {
            let (mut resolver, _) = LayoutResolver::new(all_schemas());
            resolver
                .layout_of(qname)
                .unwrap_or_else(|| panic!("no layout for {qname}"))
        })
        .clone()
}

/// The ONE walker, memoized for the same reason (it seeds from the whole
/// built-in corpus).
fn walker() -> &'static FrameWalker {
    static WALKER: OnceLock<FrameWalker> = OnceLock::new();
    WALKER.get_or_init(builtin_walker)
}

/// A fixed field's byte offset inside its message's fixed section.
fn fixed_off(qname: &str, field: &str) -> usize {
    layout_of(qname)
        .fixed_fields
        .iter()
        .find(|f| f.name == field)
        .unwrap_or_else(|| panic!("{qname}.{field} is not a fixed field"))
        .offset
}

/// One marker, in the terms the `.msg` uses. `Default` is a unit CUBE at the
/// origin so a test writes only the fields it is about.
#[derive(Clone)]
struct MarkerSpec {
    ns: String,
    id: i32,
    kind: i32,
    action: i32,
    pos: [f64; 3],
    quat: [f64; 4],
    scale: [f64; 3],
    color: [f32; 4],
    points: Vec<[f64; 3]>,
    colors: Vec<[f32; 4]>,
    text: String,
    mesh: String,
    lifetime: (i32, u32),
    frame_locked: bool,
    /// The marker's own `header.frame_id` (v1 does not read it — the deferral is
    /// reported once per input).
    frame_id: String,
    /// RAW bytes for `text`, overriding [`MarkerSpec::text`]. The only way to
    /// build a marker whose string is NOT valid UTF-8 — which a real producer
    /// can emit and the walker degrades to `Bytes`, so `field_str` yields `None`
    /// and the marker is dropped as undecodable.
    raw_text: Option<Vec<u8>>,
}

impl Default for MarkerSpec {
    fn default() -> Self {
        Self {
            ns: "ns".to_string(),
            id: 0,
            kind: MarkerKind::Cube.wire_value(),
            action: MarkerAction::Add.wire_value(),
            pos: [0.0; 3],
            quat: [0.0, 0.0, 0.0, 1.0],
            scale: [1.0; 3],
            color: [1.0, 1.0, 1.0, 1.0],
            points: Vec::new(),
            colors: Vec::new(),
            text: String::new(),
            mesh: String::new(),
            lifetime: (0, 0),
            frame_locked: false,
            frame_id: String::new(),
            raw_text: None,
        }
    }
}

impl MarkerSpec {
    fn new(kind: MarkerKind, ns: &str, id: i32) -> Self {
        Self {
            ns: ns.to_string(),
            id,
            kind: kind.wire_value(),
            ..Default::default()
        }
    }

    fn action(mut self, action: MarkerAction) -> Self {
        self.action = action.wire_value();
        self
    }

    fn scale(mut self, s: [f64; 3]) -> Self {
        self.scale = s;
        self
    }

    fn points(mut self, p: &[[f64; 3]]) -> Self {
        self.points = p.to_vec();
        self
    }
}

/// A DELETE / DELETEALL element: the action is the only field that matters, but
/// `id`/`ns` still identify a DELETE's target.
fn delete(ns: &str, id: i32) -> MarkerSpec {
    MarkerSpec {
        ns: ns.to_string(),
        id,
        ..Default::default()
    }
    .action(MarkerAction::Delete)
}

fn delete_all() -> MarkerSpec {
    MarkerSpec::default().action(MarkerAction::DeleteAll)
}

/// The `geometry_msgs/Point[]` blob: fixed-stride elements, back to back, NO
/// count (the canonical framing for a recursively-FIXED element).
fn points_blob(points: &[[f64; 3]]) -> Vec<u8> {
    let stride = layout_of("geometry_msgs/Point").fixed_size;
    assert_eq!(stride, 24, "geometry_msgs/Point stride drifted");
    let mut v = Vec::with_capacity(points.len() * stride);
    for p in points {
        for c in p {
            v.extend_from_slice(&c.to_le_bytes());
        }
    }
    v
}

/// The `std_msgs/ColorRGBA[]` blob — same fixed-stride framing, f32 components.
fn colors_blob(colors: &[[f32; 4]]) -> Vec<u8> {
    let stride = layout_of("std_msgs/ColorRGBA").fixed_size;
    assert_eq!(stride, 16, "std_msgs/ColorRGBA stride drifted");
    let mut v = Vec::with_capacity(colors.len() * stride);
    for c in colors {
        for k in c {
            v.extend_from_slice(&k.to_le_bytes());
        }
    }
    v
}

/// `[fixed section][offset table][blobs]`, blobs laid out CONTIGUOUSLY from the
/// data floor in declaration order — the canonical sub-frame shape.
///
/// **Contiguity is required, not cosmetic.** A canonical ELEMENT body is walked
/// under `PayloadAudit::Element`, which demands exact internal accounting: every
/// entry (zero-length included) must sit at or just past the previous field's
/// end, and the last must end exactly at the body's end.
fn assemble(qname: &str, fixed: &[u8], blobs: &[&[u8]]) -> Vec<u8> {
    let l = layout_of(qname);
    assert_eq!(fixed.len(), l.fixed_size, "{qname} fixed section drifted");
    assert_eq!(
        blobs.len(),
        l.variable_fields.len(),
        "{qname} variable-field count drifted"
    );
    let table = l.offset_table_bytes();
    let mut body = fixed.to_vec();
    body.resize(l.fixed_size + table, 0);
    let mut cursor = (l.fixed_size + table) as u32;
    for (idx, blob) in blobs.iter().enumerate() {
        write_offset_entry(&mut body, l.fixed_size, idx, cursor, blob.len() as u32);
        cursor += blob.len() as u32;
    }
    for blob in blobs {
        body.extend_from_slice(blob);
    }
    body
}

/// An empty-but-CANONICAL `std_msgs/Header` sub-frame.
///
/// It cannot be a zero-length blob: the `set_<f>_bytes(&[])` empty-nested idiom
/// is FRAME-ONLY, and a zero-length NESTED entry inside an element body is not
/// canonical — it degrades the whole array to `NestedArrayOpaque`. So every
/// nested field of a `Marker` element rides its real `[fixed][table]`.
fn header_subframe(frame_id: &str) -> Vec<u8> {
    let l = layout_of("std_msgs/Header");
    assert_eq!(
        l.variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["frame_id"],
    );
    assemble(
        "std_msgs/Header",
        &vec![0u8; l.fixed_size],
        &[frame_id.as_bytes()],
    )
}

/// An empty-but-canonical `sensor_msgs/CompressedImage` (`Marker.texture`),
/// carrying a real empty `Header` of its own.
fn compressed_image_subframe() -> Vec<u8> {
    let l = layout_of("sensor_msgs/CompressedImage");
    assert_eq!(
        l.variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["header", "format", "data"],
    );
    // The TEXTURE's own header is irrelevant to every oracle here — only the
    // MARKER's `header.frame_id` is read (and only to be reported as ignored).
    let hdr = header_subframe("");
    assemble(
        "sensor_msgs/CompressedImage",
        &vec![0u8; l.fixed_size],
        &[&hdr, &[], &[]],
    )
}

/// An empty-but-canonical `visualization_msgs/MeshFile` (`Marker.mesh_file`).
fn mesh_file_subframe() -> Vec<u8> {
    let l = layout_of("visualization_msgs/MeshFile");
    assert_eq!(
        l.variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["filename", "data"],
    );
    assemble(
        "visualization_msgs/MeshFile",
        &vec![0u8; l.fixed_size],
        &[&[], &[]],
    )
}

/// ONE `Marker` element body: the headerless sub-frame a canonical counted
/// element array carries.
fn marker_body(spec: &MarkerSpec) -> Vec<u8> {
    let l = layout_of(MARKER);
    // The builder's own drift guard: it writes at these offsets and in this
    // variable order, so if either moved the frame would decode to something
    // else entirely (and every oracle below would be measuring the wrong thing).
    assert_eq!(l.fixed_size, 128, "Marker fixed section drifted");
    assert_eq!(
        l.variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec![
            "header",
            "ns",
            "points",
            "colors",
            "texture_resource",
            "texture",
            "uv_coordinates",
            "text",
            "mesh_resource",
            "mesh_file",
        ],
        "Marker variable-field declaration order changed — update this builder"
    );
    assert_eq!(l.offset_table_bytes(), 80, "Marker offset table drifted");

    let mut body = vec![0u8; l.fixed_size];
    let put = |body: &mut Vec<u8>, off: usize, bytes: &[u8]| {
        body[off..off + bytes.len()].copy_from_slice(bytes);
    };
    put(&mut body, fixed_off(MARKER, "id"), &spec.id.to_le_bytes());
    put(
        &mut body,
        fixed_off(MARKER, "type"),
        &spec.kind.to_le_bytes(),
    );
    put(
        &mut body,
        fixed_off(MARKER, "action"),
        &spec.action.to_le_bytes(),
    );
    // `pose` = a nested `geometry_msgs/Pose` {position: Point, orientation: Quaternion}.
    let pose_at = fixed_off(MARKER, "pose");
    let pos_at = pose_at + fixed_off("geometry_msgs/Pose", "position");
    let quat_at = pose_at + fixed_off("geometry_msgs/Pose", "orientation");
    for (i, c) in spec.pos.iter().enumerate() {
        put(&mut body, pos_at + i * 8, &c.to_le_bytes());
    }
    for (i, c) in spec.quat.iter().enumerate() {
        put(&mut body, quat_at + i * 8, &c.to_le_bytes());
    }
    let scale_at = fixed_off(MARKER, "scale");
    for (i, c) in spec.scale.iter().enumerate() {
        put(&mut body, scale_at + i * 8, &c.to_le_bytes());
    }
    let color_at = fixed_off(MARKER, "color");
    for (i, c) in spec.color.iter().enumerate() {
        put(&mut body, color_at + i * 4, &c.to_le_bytes());
    }
    let lifetime_at = fixed_off(MARKER, "lifetime");
    put(&mut body, lifetime_at, &spec.lifetime.0.to_le_bytes());
    put(&mut body, lifetime_at + 4, &spec.lifetime.1.to_le_bytes());
    put(
        &mut body,
        fixed_off(MARKER, "frame_locked"),
        &[u8::from(spec.frame_locked)],
    );

    // Variable payload, in declaration order. The NESTED fields (`header`,
    // `texture`, `mesh_file`) carry real canonical sub-frames — see
    // `header_subframe`. `uv_coordinates` is a fixed-stride ARRAY, whose empty
    // form legitimately IS zero bytes; the strings likewise.
    let pts = points_blob(&spec.points);
    let cols = colors_blob(&spec.colors);
    let hdr = header_subframe(&spec.frame_id);
    let texture = compressed_image_subframe();
    let mesh_file = mesh_file_subframe();
    assemble(
        MARKER,
        &body,
        &[
            &hdr,
            spec.ns.as_bytes(),
            &pts,
            &cols,
            &[],
            &texture,
            &[],
            spec.raw_text.as_deref().unwrap_or(spec.text.as_bytes()),
            spec.mesh.as_bytes(),
            &mesh_file,
        ],
    )
}

/// `u32 count` + per element (`u32 len`, body) — the canonical COUNTED framing a
/// VARIABLE element rides.
fn counted_blob(elements: &[Vec<u8>]) -> Vec<u8> {
    let mut v = Vec::new();
    v.extend_from_slice(&(elements.len() as u32).to_le_bytes());
    for e in elements {
        v.extend_from_slice(&(e.len() as u32).to_le_bytes());
        v.extend_from_slice(e);
    }
    v
}

/// A whole `visualization_msgs/MarkerArray` wire frame carrying `blob` as its
/// one variable field.
fn marker_array_frame_from_blob(blob: &[u8], timestamp_ns: u64) -> Vec<u8> {
    let l = layout_of("visualization_msgs/MarkerArray");
    assert_eq!(l.fixed_size, 0, "MarkerArray gained a fixed section");
    assert_eq!(
        l.variable_fields
            .iter()
            .map(|f| f.name.as_str())
            .collect::<Vec<_>>(),
        vec!["markers"],
        "MarkerArray variable-field declaration order changed"
    );
    let table = l.offset_table_bytes();
    let mut payload = vec![0u8; table];
    write_offset_entry(&mut payload, 0, 0, table as u32, blob.len() as u32);
    payload.extend_from_slice(blob);
    let mut frame = vec![0u8; WireHeader::SIZE];
    WireHeader {
        schema_hash: <MarkerArray as ShmMessage>::SCHEMA_HASH,
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

fn marker_array_frame(specs: &[MarkerSpec]) -> Vec<u8> {
    let bodies: Vec<Vec<u8>> = specs.iter().map(marker_body).collect();
    marker_array_frame_from_blob(&counted_blob(&bodies), 42_000)
}

/// The plan a frame decodes to, or a panic naming the outcome that was not a
/// plan (so a test never silently asserts against a defaulted plan).
fn plan_of(specs: &[MarkerSpec]) -> cerulion_viz::marker::MarkerArrayPlan {
    let frame = marker_array_frame(specs);
    let fv = walker().walk_by_hash(&frame).expect("walk MarkerArray");
    match scan_marker_array(&fv) {
        MarkerArrayScan::Plan(p) => *p,
        other => panic!("expected a decoded plan, got {other:?}"),
    }
}

/// The geometry of the single marker a one-marker frame decodes to.
fn geometry_of(spec: MarkerSpec) -> MarkerGeometry {
    let plan = plan_of(&[spec]);
    assert_eq!(plan.ops.len(), 1, "expected exactly one op: {plan:?}");
    match &plan.ops[0] {
        cerulion_viz::marker::MarkerOp::Draw(d) => d.geometry.clone(),
        other => panic!("expected a Draw op, got {other:?}"),
    }
}

// ────────────────────────────────────────────────────────────────────────────
// Rerun read-back: what a VIEWER would see.
// ────────────────────────────────────────────────────────────────────────────

fn memory() -> (rerun::RecordingStream, rerun::sink::MemorySinkStorage) {
    rerun::RecordingStreamBuilder::new("marker_test")
        .memory()
        .expect("memory sink")
}

/// Every chunk the sink emitted, as `entity -> the archetype short names logged
/// on it` (accumulated, since same-entity logs may compact into one chunk).
fn rendered(
    rec: &rerun::RecordingStream,
    storage: &rerun::sink::MemorySinkStorage,
) -> BTreeMap<String, BTreeSet<String>> {
    rec.flush_blocking().expect("flush");
    let mut out: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for msg in storage.take() {
        let rerun::log::LogMsg::ArrowMsg(_, arrow) = msg else {
            continue;
        };
        let chunk = rerun::log::Chunk::from_arrow_msg(&arrow).expect("decode chunk");
        let entity = chunk
            .entity_path()
            .to_string()
            .trim_start_matches('/')
            .to_string();
        // rerun writes its own recording bookkeeping under the RESERVED `__`
        // namespace (`__properties`). It is not sink output, and an entity a
        // topic can reach never starts with `__` (the sanitizer emits only
        // `[A-Za-z0-9_]`, but a leading `__` would have to come from a name that
        // already looks like one — and no reserved-namespace entity is ours).
        if entity.starts_with("__") {
            continue;
        }
        let names = out.entry(entity).or_default();
        for descr in chunk.components().component_descriptors() {
            if let Some(a) = descr.archetype.as_ref() {
                names.insert(a.short_name().to_string());
            }
        }
    }
    out
}

/// Every RECORDING chunk the sink emitted, decoded back off the real store.
///
/// The `(entity → archetype names)` view above answers "which family drew"; this
/// answers "with WHAT VALUES", which is what makes an oracle a real oracle
/// (radii, colours, extents and the parent frame are all invisible to a
/// name-only assertion).
fn rendered_chunks(
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

/// Read one component column back through the REAL component type (the same read
/// a viewer performs) at `entity`, keyed by its `Archetype:field` descriptor.
fn component_at<C: rerun::Loggable>(
    chunks: &[rerun::log::Chunk],
    entity: &str,
    descriptor: &str,
) -> Vec<C> {
    let mut out = Vec::new();
    for chunk in chunks {
        if chunk.entity_path().to_string().trim_start_matches('/') != entity {
            continue;
        }
        for (descr, list) in chunk.components().iter() {
            if descr.as_str() != descriptor {
                continue;
            }
            out.extend(
                C::from_arrow(list.list_array.values().as_ref())
                    .unwrap_or_else(|e| panic!("{descriptor} column: {e}")),
            );
        }
    }
    out
}

const INPUT: &str = "markers";

fn topic_entity(input: &str) -> String {
    route_for_input(input).entity
}

fn entity_for(input: &str, ns: &str, id: i32) -> String {
    marker_entity(&topic_entity(input), &format!("{ns}/id_{id}"))
}

/// Dispatch one frame through the production sink path.
fn dispatch(
    rec: &rerun::RecordingStream,
    input: &str,
    specs: &[MarkerSpec],
    state: &mut SinkState,
) {
    let frame = marker_array_frame(specs);
    dispatch_frame(rec, walker(), input, &frame, state);
}

// ════════════════════════════════════════════════════════════════════════════
// 1. The PURE extractor — per kind, against hand-written numbers.
// ════════════════════════════════════════════════════════════════════════════

const P: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]];
const PF: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]];

#[test]
fn every_kind_maps_to_its_hand_written_geometry() {
    use MarkerKind as K;
    // Each row is (kind, the spec, the EXACT geometry expected) — the numbers on
    // the right are written by hand from the ROS scale semantics, never derived
    // from the extractor.
    let cases: Vec<(K, MarkerSpec, MarkerGeometry)> = vec![
        (
            // Pose form: `scale.x` is the arrow LENGTH along local +X, `scale.y`
            // the shaft DIAMETER.
            K::Arrow,
            MarkerSpec::new(K::Arrow, "a", 0).scale([2.0, 0.5, 0.5]),
            MarkerGeometry::Arrows {
                origins: vec![[0.0, 0.0, 0.0]],
                vectors: vec![[2.0, 0.0, 0.0]],
                radius: 0.25,
            },
        ),
        (
            K::Cube,
            MarkerSpec::new(K::Cube, "a", 0).scale([1.0, 2.0, 3.0]),
            MarkerGeometry::Boxes {
                centers: vec![[0.0, 0.0, 0.0]],
                size: [1.0, 2.0, 3.0],
            },
        ),
        (
            // ROS `scale` is the DIAMETER per axis.
            K::Sphere,
            MarkerSpec::new(K::Sphere, "a", 0).scale([2.0, 4.0, 6.0]),
            MarkerGeometry::Ellipsoids {
                centers: vec![[0.0, 0.0, 0.0]],
                half_size: [1.0, 2.0, 3.0],
            },
        ),
        (
            // Diameters x/y, height z ⇒ radius (2+2)/4 = 1, length 5.
            K::Cylinder,
            MarkerSpec::new(K::Cylinder, "a", 0).scale([2.0, 2.0, 5.0]),
            MarkerGeometry::Cylinder {
                length: 5.0,
                radius: 1.0,
            },
        ),
        (
            K::LineStrip,
            MarkerSpec::new(K::LineStrip, "a", 0)
                .scale([0.1, 0.0, 0.0])
                .points(&P),
            MarkerGeometry::LineStrips {
                strips: vec![PF.to_vec()],
                radius: 0.05,
            },
        ),
        (
            K::LineList,
            MarkerSpec::new(K::LineList, "a", 0)
                .scale([0.1, 0.0, 0.0])
                .points(&[P[0], P[1], P[2], [4.0, 0.0, 0.0]]),
            MarkerGeometry::LineStrips {
                strips: vec![vec![PF[0], PF[1]], vec![PF[2], [4.0, 0.0, 0.0]]],
                radius: 0.05,
            },
        ),
        (
            K::CubeList,
            MarkerSpec::new(K::CubeList, "a", 0)
                .scale([0.5, 0.5, 0.5])
                .points(&P),
            MarkerGeometry::Boxes {
                centers: PF.to_vec(),
                size: [0.5, 0.5, 0.5],
            },
        ),
        (
            K::SphereList,
            MarkerSpec::new(K::SphereList, "a", 0)
                .scale([2.0, 2.0, 2.0])
                .points(&P),
            MarkerGeometry::Ellipsoids {
                centers: PF.to_vec(),
                half_size: [1.0, 1.0, 1.0],
            },
        ),
        (
            K::Points,
            MarkerSpec::new(K::Points, "a", 0)
                .scale([0.2, 0.2, 0.0])
                .points(&P),
            MarkerGeometry::Points {
                positions: PF.to_vec(),
                radius: 0.1,
            },
        ),
        (
            K::TextViewFacing,
            MarkerSpec {
                text: "hello".to_string(),
                ..MarkerSpec::new(K::TextViewFacing, "a", 0)
            },
            MarkerGeometry::Text {
                text: "hello".to_string(),
            },
        ),
        (
            K::MeshResource,
            MarkerSpec {
                mesh: "package://acme/meshes/base.dae".to_string(),
                ..MarkerSpec::new(K::MeshResource, "a", 0).scale([1.0, 2.0, 3.0])
            },
            MarkerGeometry::MeshProxy {
                size: [1.0, 2.0, 3.0],
                uri: "package://acme/meshes/base.dae".to_string(),
            },
        ),
        (
            // ROS applies the marker `scale` to a triangle list's VERTICES.
            K::TriangleList,
            MarkerSpec::new(K::TriangleList, "a", 0)
                .scale([2.0, 2.0, 2.0])
                .points(&P),
            MarkerGeometry::Triangles {
                vertices: vec![[2.0, 0.0, 0.0], [4.0, 0.0, 0.0], [6.0, 0.0, 0.0]],
            },
        ),
        (
            // One arrow per consecutive PAIR.
            K::ArrowStrip,
            MarkerSpec::new(K::ArrowStrip, "a", 0)
                .scale([0.2, 0.0, 0.0])
                .points(&P),
            MarkerGeometry::Arrows {
                origins: vec![PF[0], PF[1]],
                vectors: vec![[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                radius: 0.1,
            },
        ),
    ];
    // TOTALITY: every declared kind is covered, in wire order.
    assert_eq!(
        cases.iter().map(|(k, ..)| *k).collect::<Vec<_>>(),
        MarkerKind::ALL.to_vec(),
        "a kind is missing from (or misordered in) this table"
    );
    for (kind, spec, expected) in cases {
        assert_eq!(geometry_of(spec), expected, "{kind:?} geometry");
    }
}

#[test]
fn an_arrow_with_two_points_spans_them_instead_of_using_scale_x_as_a_length() {
    // The two-form rule: with `points`, `scale.x` becomes the shaft DIAMETER and
    // the arrow runs point[0] -> point[1].
    let geometry = geometry_of(
        MarkerSpec::new(MarkerKind::Arrow, "a", 0)
            .scale([0.4, 9.0, 9.0])
            .points(&[[1.0, 1.0, 1.0], [1.0, 1.0, 4.0]]),
    );
    assert_eq!(
        geometry,
        MarkerGeometry::Arrows {
            origins: vec![[1.0, 1.0, 1.0]],
            vectors: vec![[0.0, 0.0, 3.0]],
            radius: 0.2,
        }
    );
}

#[test]
fn a_line_list_drops_its_odd_trailing_point_and_reports_it() {
    let plan = plan_of(&[MarkerSpec::new(MarkerKind::LineList, "a", 0)
        .scale([0.1, 0.0, 0.0])
        .points(&[P[0], P[1], P[2], [4.0, 0.0, 0.0], [5.0, 0.0, 0.0]])]);
    let cerulion_viz::marker::MarkerOp::Draw(d) = &plan.ops[0] else {
        panic!("expected a Draw");
    };
    let MarkerGeometry::LineStrips { strips, .. } = &d.geometry else {
        panic!("expected LineStrips");
    };
    assert_eq!(strips.len(), 2, "5 points ⇒ 2 whole segments, tail dropped");
    assert_eq!(plan.reports.dropped_tail_kinds, vec![MarkerKind::LineList]);
}

/// The remaining three of the five kinds that can drop a trailing point — so the
/// report covers its whole domain, not just the two obvious groupings.
#[test]
fn every_tail_dropping_kind_reports_its_own_grouping_failure() {
    use MarkerKind as K;
    // (kind, points, why it does not fit)
    let cases: Vec<(K, Vec<[f64; 3]>)> = vec![
        // A LINE_STRIP needs at least two points to be a line at all.
        (K::LineStrip, vec![P[0]]),
        // An ARROW_STRIP needs at least one consecutive PAIR.
        (K::ArrowStrip, vec![P[0]]),
        // A single ARROW uses exactly two points; a third is not a second arrow.
        (K::Arrow, vec![P[0], P[1], P[2]]),
    ];
    for (kind, points) in cases {
        let plan = plan_of(&[MarkerSpec::new(kind, "a", 0)
            .scale([0.1, 0.1, 0.1])
            .points(&points)]);
        assert_eq!(
            plan.reports.dropped_tail_kinds,
            vec![kind],
            "{kind:?} with {} point(s) must report its dropped tail",
            points.len()
        );
    }
    // ANTI-TAUTOLOGY: the same kinds with WELL-FORMED point counts report nothing.
    for (kind, points) in [
        (K::LineStrip, vec![P[0], P[1]]),
        (K::ArrowStrip, vec![P[0], P[1]]),
        (K::Arrow, vec![P[0], P[1]]),
    ] {
        let plan = plan_of(&[MarkerSpec::new(kind, "a", 0)
            .scale([0.1, 0.1, 0.1])
            .points(&points)]);
        assert!(
            plan.reports.dropped_tail_kinds.is_empty(),
            "{kind:?} with a well-formed point count must be quiet"
        );
    }
}

#[test]
fn a_triangle_list_drops_a_partial_triangle_and_reports_it() {
    let plan = plan_of(&[MarkerSpec::new(MarkerKind::TriangleList, "a", 0)
        .scale([1.0, 1.0, 1.0])
        .points(&[P[0], P[1], P[2], [4.0, 0.0, 0.0], [5.0, 0.0, 0.0]])]);
    let cerulion_viz::marker::MarkerOp::Draw(d) = &plan.ops[0] else {
        panic!("expected a Draw");
    };
    assert_eq!(
        d.geometry,
        MarkerGeometry::Triangles {
            vertices: vec![PF[0], PF[1], PF[2]],
        },
        "8 points would be 2 triangles + 2 spare; 5 is 1 triangle + 2 spare"
    );
    assert_eq!(
        plan.reports.dropped_tail_kinds,
        vec![MarkerKind::TriangleList]
    );
}

#[test]
fn an_unknown_type_is_skipped_and_reported_while_the_rest_still_draw() {
    // The ONE deliberate divergence from the all-or-nothing element rule: a
    // MarkerArray is a BAG of independent objects, so rendering two and naming
    // the third fabricates nothing, whereas dropping all three would be worse.
    let mut odd = MarkerSpec::new(MarkerKind::Cube, "a", 2);
    odd.kind = 99;
    let plan = plan_of(&[
        MarkerSpec::new(MarkerKind::Cube, "a", 1),
        odd,
        MarkerSpec::new(MarkerKind::Sphere, "a", 3),
    ]);
    assert_eq!(plan.ops.len(), 2, "the two known markers still draw");
    assert_eq!(plan.reports.unknown_types, vec![99]);
}

#[test]
fn an_undeclared_action_is_skipped_and_reported() {
    // `1` is genuinely unassigned in ROS (ADD == MODIFY == 0), so there is no
    // valid reading of it — the marker is skipped, not guessed into an ADD.
    let mut odd = MarkerSpec::new(MarkerKind::Cube, "a", 2);
    odd.action = 1;
    let plan = plan_of(&[MarkerSpec::new(MarkerKind::Cube, "a", 1), odd]);
    assert_eq!(plan.ops.len(), 1);
    assert_eq!(plan.reports.unknown_actions, vec![1]);
}

#[test]
fn deletes_become_ops_not_draws() {
    let plan = plan_of(&[
        delete_all(),
        MarkerSpec::new(MarkerKind::Cube, "a", 1),
        delete("b", 7),
    ]);
    use cerulion_viz::marker::MarkerOp;
    assert!(matches!(plan.ops[0], MarkerOp::DeleteAll));
    assert!(matches!(plan.ops[1], MarkerOp::Draw(_)));
    match &plan.ops[2] {
        MarkerOp::Delete(k) => assert_eq!((k.ns.as_str(), k.id), ("b", 7)),
        other => panic!("expected a Delete, got {other:?}"),
    }
    assert_eq!(plan.ops.len(), 3);
}

#[test]
fn an_empty_marker_array_is_empty_not_absent_and_not_a_deleteall() {
    let frame = marker_array_frame(&[]);
    let w = walker();
    let fv = w.walk_by_hash(&frame).expect("walk");
    // An idle publisher has not asked for anything to be erased.
    assert_eq!(scan_marker_array(&fv), MarkerArrayScan::Empty);
}

#[test]
fn an_undecodable_markers_array_is_absent_not_an_empty_plan() {
    // The live case is an rmw-published MarkerArray whose element bodies the
    // walker refuses — it must NOT look like "zero markers".
    let junk = vec![0xABu8; 37];
    let frame = marker_array_frame_from_blob(&junk, 42_000);
    let w = walker();
    let fv = w.walk_by_hash(&frame).expect("walk");
    assert_eq!(scan_marker_array(&fv), MarkerArrayScan::Absent);
}

#[test]
fn a_mismatched_colors_bank_falls_back_to_the_flat_color() {
    // Zipping a short colour bank would mis-colour the tail, so the whole bank is
    // refused and the flat colour used.
    let mut spec = MarkerSpec::new(MarkerKind::Points, "a", 0)
        .scale([0.2, 0.2, 0.0])
        .points(&P);
    spec.colors = vec![[1.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.0, 1.0]];
    let plan = plan_of(&[spec]);
    let cerulion_viz::marker::MarkerOp::Draw(d) = &plan.ops[0] else {
        panic!("expected a Draw");
    };
    assert_eq!(d.instance_colors, None);
    assert_eq!(plan.reports.color_length_mismatches, 1);

    // Control: a MATCHING bank IS used (so the arm above is a real refusal, not
    // a per-vertex path that never works).
    let mut ok = MarkerSpec::new(MarkerKind::Points, "a", 0)
        .scale([0.2, 0.2, 0.0])
        .points(&P);
    ok.colors = vec![
        [1.0, 0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0, 1.0],
        [0.0, 0.0, 1.0, 0.5],
    ];
    let plan = plan_of(&[ok]);
    let cerulion_viz::marker::MarkerOp::Draw(d) = &plan.ops[0] else {
        panic!("expected a Draw");
    };
    assert_eq!(
        d.instance_colors,
        Some(vec![[255, 0, 0, 255], [0, 255, 0, 255], [0, 0, 255, 128],]),
        "ColorRGBA 0..=1 floats round to sRGB bytes"
    );
    assert_eq!(plan.reports.color_length_mismatches, 0);
}

#[test]
fn over_the_marker_cap_the_tail_is_not_inspected_and_the_count_is_reported() {
    const OVER: usize = 5;
    let specs: Vec<MarkerSpec> = (0..(MAX_MARKER_INSTANCES + OVER) as i32)
        .map(|i| MarkerSpec::new(MarkerKind::Cube, "a", i))
        .collect();
    let plan = plan_of(&specs);
    assert_eq!(
        plan.ops.len(),
        MAX_MARKER_INSTANCES,
        "the per-FRAME inspection ceiling"
    );
    assert_eq!(plan.reports.truncated_markers, OVER);
}

#[test]
fn the_vertex_budget_skips_whole_markers_never_half_a_polyline() {
    // Two markers, each just over half the budget: the first fits, the second
    // cannot — and it must be absent ENTIRELY rather than clipped, because a
    // half-drawn polyline is a plausible-but-wrong picture.
    let big: Vec<[f64; 3]> = (0..(MAX_MARKER_VERTICES / 2 + 1))
        .map(|i| [i as f64, 0.0, 0.0])
        .collect();
    let plan = plan_of(&[
        MarkerSpec::new(MarkerKind::Points, "a", 1)
            .scale([0.1, 0.1, 0.0])
            .points(&big),
        MarkerSpec::new(MarkerKind::Points, "a", 2)
            .scale([0.1, 0.1, 0.0])
            .points(&big),
    ]);
    assert_eq!(plan.ops.len(), 1, "the straddling marker is skipped WHOLE");
    assert_eq!(plan.reports.truncated_vertices, 1);
    let cerulion_viz::marker::MarkerOp::Draw(d) = &plan.ops[0] else {
        panic!("expected a Draw");
    };
    let MarkerGeometry::Points { positions, .. } = &d.geometry else {
        panic!("expected Points");
    };
    assert_eq!(
        positions.len(),
        big.len(),
        "the marker that DID fit keeps every vertex"
    );
}

#[test]
fn lifetime_and_frame_locked_are_recorded_as_unhonoured() {
    let plan = plan_of(&[
        MarkerSpec {
            lifetime: (3, 0),
            ..MarkerSpec::new(MarkerKind::Cube, "a", 1)
        },
        MarkerSpec {
            frame_locked: true,
            ..MarkerSpec::new(MarkerKind::Cube, "a", 2)
        },
        // A ZERO lifetime is the normal "forever" value and must not be reported.
        MarkerSpec::new(MarkerKind::Cube, "a", 3),
    ]);
    assert_eq!(plan.reports.lifetime_markers, 1);
    assert_eq!(plan.reports.frame_locked_markers, 1);
}

#[test]
fn a_zero_scale_marker_is_reported_but_still_drawn() {
    // The publisher asked for a zero-size cube; nothing is substituted. The note
    // exists so an empty view is explainable.
    let plan = plan_of(&[MarkerSpec::new(MarkerKind::Cube, "a", 1).scale([0.0, 0.0, 0.0])]);
    assert_eq!(plan.ops.len(), 1, "still drawn");
    assert_eq!(plan.reports.degenerate_scales, 1);
    // Control: a normal scale is not reported.
    let plan = plan_of(&[MarkerSpec::new(MarkerKind::Cube, "a", 1).scale([1.0, 1.0, 1.0])]);
    assert_eq!(plan.reports.degenerate_scales, 0);
}

#[test]
fn an_elliptical_cylinder_is_reported_as_a_circular_approximation() {
    let plan = plan_of(&[MarkerSpec::new(MarkerKind::Cylinder, "a", 1).scale([1.0, 3.0, 2.0])]);
    assert_eq!(plan.reports.elliptical_cylinders, 1);
    let cerulion_viz::marker::MarkerOp::Draw(d) = &plan.ops[0] else {
        panic!("expected a Draw");
    };
    assert_eq!(
        d.geometry,
        MarkerGeometry::Cylinder {
            length: 2.0,
            radius: 1.0, // (1 + 3) / 4
        }
    );
    let plan = plan_of(&[MarkerSpec::new(MarkerKind::Cylinder, "a", 1).scale([2.0, 2.0, 2.0])]);
    assert_eq!(
        plan.reports.elliptical_cylinders, 0,
        "a circular one is not reported"
    );
}

#[test]
fn the_same_bytes_decode_to_the_identical_plan_twice() {
    let specs = [
        delete_all(),
        MarkerSpec::new(MarkerKind::Cube, "zeta", 2),
        MarkerSpec::new(MarkerKind::Sphere, "alpha", -1).scale([2.0, 2.0, 2.0]),
        delete("gone", 4),
    ];
    let a = plan_of(&specs);
    let b = plan_of(&specs);
    assert_eq!(a, b);
    // ...and both equal a hand-written op SHAPE, so neither leg is a self-compare.
    use cerulion_viz::marker::MarkerOp;
    assert!(matches!(
        (&a.ops[0], &a.ops[1], &a.ops[2], &a.ops[3]),
        (
            MarkerOp::DeleteAll,
            MarkerOp::Draw(_),
            MarkerOp::Draw(_),
            MarkerOp::Delete(_)
        )
    ));
}

// ════════════════════════════════════════════════════════════════════════════
// 2. RENDER + CLEAR, through the production `dispatch_frame`.
// ════════════════════════════════════════════════════════════════════════════

#[test]
fn each_marker_gets_its_own_entity_carrying_a_transform_and_its_geometry() {
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    dispatch(
        &rec,
        INPUT,
        &[
            MarkerSpec::new(MarkerKind::Cube, "costmap", 7),
            MarkerSpec::new(MarkerKind::Sphere, "costmap", -3).scale([2.0, 2.0, 2.0]),
        ],
        &mut state,
    );
    let seen = rendered(&rec, &storage);
    let base = topic_entity(INPUT);
    let cube = format!("{base}/{MARKER_CHILD}/costmap/id_7");
    let sphere = format!("{base}/{MARKER_CHILD}/costmap/id_n3");
    assert_eq!(
        seen.keys().cloned().collect::<Vec<_>>(),
        vec![cube.clone(), sphere.clone()],
        "exactly the two marker entities, hand-written"
    );
    // Each entity carries BOTH its pose transform and its geometry — the pose is
    // what places the geometry, since everything is logged marker-local.
    assert_eq!(
        seen[&cube],
        BTreeSet::from(["Transform3D".to_string(), "Boxes3D".to_string()])
    );
    assert_eq!(
        seen[&sphere],
        BTreeSet::from(["Transform3D".to_string(), "Ellipsoids3D".to_string()])
    );
    assert_eq!(state.live_marker_count(INPUT), 2);
}

#[test]
fn every_kind_records_the_archetype_its_mapping_promises() {
    // The RENDER-side totality twin of the extractor table. Each row is
    // CROSS-CHECKED against `blueprint::archetype_components(MarkerArray)` — the
    // union the agent is told a marker topic produces — so a kind cannot render
    // a family the blueprint never advertised (nor the reverse, since the union
    // itself is pinned exactly by `layout_mapping_test`).
    use MarkerKind as K;
    let advertised = cerulion_viz::blueprint::archetype_components(ArchetypeKind::MarkerArray);
    let cases: Vec<(K, MarkerSpec, &str)> = vec![
        (K::Arrow, MarkerSpec::new(K::Arrow, "k", 0), "Arrows3D"),
        (K::Cube, MarkerSpec::new(K::Cube, "k", 1), "Boxes3D"),
        (
            K::Sphere,
            MarkerSpec::new(K::Sphere, "k", 2),
            "Ellipsoids3D",
        ),
        (
            K::Cylinder,
            MarkerSpec::new(K::Cylinder, "k", 3),
            "Cylinders3D",
        ),
        (
            K::LineStrip,
            MarkerSpec::new(K::LineStrip, "k", 4).points(&P),
            "LineStrips3D",
        ),
        (
            K::LineList,
            MarkerSpec::new(K::LineList, "k", 5).points(&[P[0], P[1]]),
            "LineStrips3D",
        ),
        (
            K::CubeList,
            MarkerSpec::new(K::CubeList, "k", 6).points(&P),
            "Boxes3D",
        ),
        (
            K::SphereList,
            MarkerSpec::new(K::SphereList, "k", 7).points(&P),
            "Ellipsoids3D",
        ),
        (
            K::Points,
            MarkerSpec::new(K::Points, "k", 8).points(&P),
            "Points3D",
        ),
        (
            K::TextViewFacing,
            MarkerSpec {
                text: "hi".to_string(),
                ..MarkerSpec::new(K::TextViewFacing, "k", 9)
            },
            // The deliberate degrade: rerun 0.34 has NO 3D text archetype, so in-scene
            // text is a zero-radius labelled point. Pinned explicitly so a future
            // rerun `Text3D` is a DELIBERATE change, not a silent one.
            "Points3D",
        ),
        (
            K::MeshResource,
            MarkerSpec {
                mesh: "package://a/b.dae".to_string(),
                ..MarkerSpec::new(K::MeshResource, "k", 10)
            },
            // Also a declared degrade: a labelled PROXY box, never a fetched mesh.
            "Boxes3D",
        ),
        (
            K::TriangleList,
            MarkerSpec::new(K::TriangleList, "k", 11).points(&P),
            "Mesh3D",
        ),
        (
            K::ArrowStrip,
            MarkerSpec::new(K::ArrowStrip, "k", 12).points(&P),
            "Arrows3D",
        ),
    ];
    assert_eq!(
        cases.iter().map(|(k, ..)| *k).collect::<Vec<_>>(),
        MarkerKind::ALL.to_vec(),
        "a kind is missing from (or misordered in) this render table"
    );
    for (kind, spec, archetype) in cases {
        assert!(
            advertised.contains(&archetype),
            "{kind:?} renders {archetype}, which archetype_components(MarkerArray) does not \
             advertise — the agent would be told this topic never produces it"
        );
        assert!(
            advertised.contains(&"Transform3D"),
            "every marker also logs its pose Transform3D"
        );
        let (rec, storage) = memory();
        let mut state = SinkState::new();
        let id = spec.id;
        dispatch(&rec, INPUT, &[spec], &mut state);
        let seen = rendered(&rec, &storage);
        let entity = entity_for(INPUT, "k", id);
        assert_eq!(
            seen.get(&entity),
            Some(&BTreeSet::from([
                "Transform3D".to_string(),
                archetype.to_string()
            ])),
            "{kind:?} must render {archetype} at {entity}"
        );
    }
}

#[test]
fn a_marker_array_bypasses_the_inference_memo_on_every_frame() {
    // The memo fix landed a per-INPUT memo of the SHAPE-INFERRED archetype. A
    // MarkerArray is NAME-mapped, so `archetype_for` must return from the table
    // BEFORE the memo is consulted or written — a warm memo can never re-route a
    // marker frame, and the inference ladder must never run for it at all.
    //
    // The assertion is on BOTH halves, because either alone is weak: chunks
    // without the counter would pass a build that inferred and happened to agree,
    // and the counter without the chunks would pass a build that drew nothing.
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    let specs = [
        MarkerSpec::new(MarkerKind::Cube, "warm", 1),
        MarkerSpec::new(MarkerKind::Sphere, "warm", 2).scale([2.0, 2.0, 2.0]),
    ];
    for frame in 1..=2 {
        dispatch(&rec, INPUT, &specs, &mut state);
        let seen = rendered(&rec, &storage);
        assert_eq!(
            seen.get(&entity_for(INPUT, "warm", 1)),
            Some(&BTreeSet::from([
                "Transform3D".to_string(),
                "Boxes3D".to_string()
            ])),
            "frame {frame} must still render marker archetypes"
        );
        assert_eq!(
            seen.get(&entity_for(INPUT, "warm", 2)),
            Some(&BTreeSet::from([
                "Transform3D".to_string(),
                "Ellipsoids3D".to_string()
            ])),
            "frame {frame}: the SECOND marker's kind is not memoized away either"
        );
        assert_eq!(
            state.inference_runs(),
            0,
            "frame {frame}: the shape ladder must NEVER run for a name-mapped schema"
        );
    }
    assert!(!state.took_anyvalues_fallback("visualization_msgs/MarkerArray"));
}

#[test]
fn a_marker_array_that_draws_nothing_records_nothing() {
    // ANTI-TAUTOLOGY: the apparatus above must not record chunks merely by being
    // driven. A DELETEALL with nothing tracked has nothing to enumerate, so it
    // touches the store zero times.
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    dispatch(&rec, INPUT, &[delete_all(), delete_all()], &mut state);
    assert!(
        rendered(&rec, &storage).is_empty(),
        "no phantom clears, no phantom draws"
    );
    assert_eq!(state.live_marker_count(INPUT), 0);
}

#[test]
fn an_explicit_delete_clears_even_an_untracked_marker() {
    // The reversal, end to end. Tracking is bounded
    // (`MAX_LIVE_MARKERS`), so gating a DELETE on "was it live?" makes every
    // DELETE past the ceiling a silent no-op and its marker a permanent ghost. A
    // Clear on a leaf entity is idempotent; a ghost is a wrong picture.
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    dispatch(&rec, INPUT, &[delete("ghost", 1)], &mut state);
    let seen = rendered(&rec, &storage);
    assert_eq!(
        seen.keys().cloned().collect::<Vec<_>>(),
        vec![entity_for(INPUT, "ghost", 1)]
    );
    assert_eq!(
        seen[&entity_for(INPUT, "ghost", 1)],
        BTreeSet::from(["Clear".to_string()])
    );
}

#[test]
fn an_add_whose_geometry_is_empty_clears_the_entity_instead_of_leaving_it_stale() {
    // THE stale-geometry pin, end to end. Under rerun latest-at, an ADD that logs only a
    // Transform3D leaves the PREVIOUS frame's geometry standing as current — so
    // the "shrink to zero points = nothing detected this cycle" idiom would show
    // stale data indefinitely.
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    let populated = MarkerSpec::new(MarkerKind::Points, "det", 1)
        .scale([0.2, 0.2, 0.0])
        .points(&P);
    dispatch(&rec, INPUT, std::slice::from_ref(&populated), &mut state);
    let seen = rendered(&rec, &storage);
    assert_eq!(
        seen.get(&entity_for(INPUT, "det", 1)),
        Some(&BTreeSet::from([
            "Transform3D".to_string(),
            "Points3D".to_string()
        ])),
        "premise: the populated frame drew"
    );

    // Same ns/id, ZERO points — "nothing detected this cycle".
    let emptied = MarkerSpec::new(MarkerKind::Points, "det", 1).scale([0.2, 0.2, 0.0]);
    dispatch(&rec, INPUT, &[emptied], &mut state);
    let seen = rendered(&rec, &storage);
    assert_eq!(
        seen.get(&entity_for(INPUT, "det", 1)),
        Some(&BTreeSet::from(["Clear".to_string()])),
        "the entity is CLEARED, not left showing the previous frame's points"
    );
    assert_eq!(state.live_marker_count(INPUT), 0);
}

#[test]
fn a_delete_clears_exactly_its_own_entity_and_leaves_its_sibling_alone() {
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    dispatch(
        &rec,
        INPUT,
        &[
            MarkerSpec::new(MarkerKind::Cube, "a", 1),
            MarkerSpec::new(MarkerKind::Cube, "a", 2),
        ],
        &mut state,
    );
    let _ = rendered(&rec, &storage); // drain the ADD frame
    dispatch(&rec, INPUT, &[delete("a", 1)], &mut state);
    let seen = rendered(&rec, &storage);
    assert_eq!(
        seen.keys().cloned().collect::<Vec<_>>(),
        vec![entity_for(INPUT, "a", 1)],
        "exactly ONE entity touched by the delete frame"
    );
    assert_eq!(
        seen[&entity_for(INPUT, "a", 1)],
        BTreeSet::from(["Clear".to_string()])
    );
    assert_eq!(state.live_marker_count(INPUT), 1, "the sibling stays live");
}

#[test]
fn a_deleteall_clears_every_live_entity_but_not_what_the_same_frame_re_adds() {
    // THE correctness pin. The dominant real idiom is DELETEALL at markers[0]
    // followed by the fresh set: only the markers that genuinely went away may be
    // cleared, and no entity may be cleared AND drawn at one timestamp (rerun does
    // not specify how that tie resolves).
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    dispatch(
        &rec,
        INPUT,
        &[
            MarkerSpec::new(MarkerKind::Cube, "a", 1),
            MarkerSpec::new(MarkerKind::Cube, "a", 9),
            MarkerSpec::new(MarkerKind::Cube, "b", 1),
        ],
        &mut state,
    );
    let _ = rendered(&rec, &storage);

    dispatch(
        &rec,
        INPUT,
        &[
            delete_all(),
            MarkerSpec::new(MarkerKind::Cube, "a", 1),
            MarkerSpec::new(MarkerKind::Sphere, "a", 2).scale([2.0, 2.0, 2.0]),
        ],
        &mut state,
    );
    let seen = rendered(&rec, &storage);
    let cleared: Vec<String> = seen
        .iter()
        .filter(|(_, a)| a.contains("Clear"))
        .map(|(e, _)| e.clone())
        .collect();
    let mut expected_clears = vec![entity_for(INPUT, "a", 9), entity_for(INPUT, "b", 1)];
    expected_clears.sort();
    assert_eq!(
        cleared, expected_clears,
        "only the markers that really went away — across BOTH namespaces"
    );
    // The survivor and the newcomer DREW, and neither was also cleared.
    for (ns, id, archetype) in [("a", 1, "Boxes3D"), ("a", 2, "Ellipsoids3D")] {
        let e = entity_for(INPUT, ns, id);
        assert_eq!(
            seen.get(&e),
            Some(&BTreeSet::from([
                "Transform3D".to_string(),
                archetype.to_string()
            ])),
            "{e} must draw, and must NOT carry a Clear"
        );
    }
    assert_eq!(state.live_marker_count(INPUT), 2);
}

#[test]
fn an_empty_array_neither_draws_nor_clears_and_leaves_the_live_set_intact() {
    // An empty array is NOT a DELETEALL: a publisher that went quiet has not
    // asked for its markers to be erased.
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    dispatch(
        &rec,
        INPUT,
        &[MarkerSpec::new(MarkerKind::Cube, "a", 1)],
        &mut state,
    );
    let _ = rendered(&rec, &storage);
    dispatch(&rec, INPUT, &[], &mut state);
    assert!(rendered(&rec, &storage).is_empty(), "zero new chunks");
    assert_eq!(state.live_marker_count(INPUT), 1);
    assert!(
        !state.took_anyvalues_fallback("visualization_msgs/MarkerArray"),
        "an idle array must NOT flip the topic to a text dump"
    );
}

#[test]
fn a_marker_can_be_re_added_after_being_deleted() {
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    let add = [MarkerSpec::new(MarkerKind::Cube, "a", 1)];
    dispatch(&rec, INPUT, &add, &mut state);
    dispatch(&rec, INPUT, &[delete("a", 1)], &mut state);
    let _ = rendered(&rec, &storage);
    dispatch(&rec, INPUT, &add, &mut state);
    let seen = rendered(&rec, &storage);
    assert_eq!(
        seen.get(&entity_for(INPUT, "a", 1)),
        Some(&BTreeSet::from([
            "Transform3D".to_string(),
            "Boxes3D".to_string()
        ])),
        "the third frame draws again — the full cycle"
    );
    assert_eq!(state.live_marker_count(INPUT), 1);
}

#[test]
fn two_inputs_do_not_share_marker_state() {
    // The live set is PER INPUT: a DELETEALL on one topic must not clear another
    // topic's markers.
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    dispatch(
        &rec,
        "markers_a",
        &[MarkerSpec::new(MarkerKind::Cube, "n", 1)],
        &mut state,
    );
    dispatch(
        &rec,
        "markers_b",
        &[MarkerSpec::new(MarkerKind::Cube, "n", 1)],
        &mut state,
    );
    let _ = rendered(&rec, &storage);
    dispatch(&rec, "markers_a", &[delete_all()], &mut state);
    let seen = rendered(&rec, &storage);
    assert_eq!(
        seen.keys().cloned().collect::<Vec<_>>(),
        vec![entity_for("markers_a", "n", 1)],
        "only topic A's entity is touched"
    );
    assert_eq!(state.live_marker_count("markers_a"), 0);
    assert_eq!(state.live_marker_count("markers_b"), 1);
}

#[test]
fn a_reconnect_resets_the_live_marker_set_so_a_later_deleteall_clears_nothing() {
    // Deleting the `state.reset_marker_state()` call from the
    // worker's reconnect hook (or the method body) makes this fail with ONE Clear
    // chunk at an entity the fresh viewer has never seen.
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    dispatch(
        &rec,
        INPUT,
        &[MarkerSpec::new(MarkerKind::Cube, "a", 1)],
        &mut state,
    );
    let _ = rendered(&rec, &storage);
    assert_eq!(state.live_marker_count(INPUT), 1, "believed live before");

    state.reset_marker_state(); // exactly what the reconnect hook calls
    assert_eq!(state.live_marker_count(INPUT), 0);

    dispatch(&rec, INPUT, &[delete_all()], &mut state);
    assert!(
        rendered(&rec, &storage).is_empty(),
        "nothing is believed live, so nothing is cleared"
    );
}

#[test]
fn a_repeated_delete_records_exactly_one_clear_chunk() {
    // THE overflow-(c) pin, end to end: the common ROS "resend the full set with
    // its retirements" idiom sends the same DELETE every frame. Clearing removes
    // the key from the live set, so without the cleared-key memory this would
    // emit one Clear chunk per frame FOREVER for an otherwise idle publisher.
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    dispatch(
        &rec,
        INPUT,
        &[MarkerSpec::new(MarkerKind::Cube, "a", 1)],
        &mut state,
    );
    let _ = rendered(&rec, &storage);

    let retire = [delete("a", 1)];
    dispatch(&rec, INPUT, &retire, &mut state);
    assert_eq!(
        rendered(&rec, &storage).get(&entity_for(INPUT, "a", 1)),
        Some(&BTreeSet::from(["Clear".to_string()])),
        "the first retirement clears"
    );
    for frame in 2..=6 {
        dispatch(&rec, INPUT, &retire, &mut state);
        assert!(
            rendered(&rec, &storage).is_empty(),
            "frame {frame} re-cleared an already-retired marker"
        );
    }
    // A genuine re-ADD re-arms it: the next DELETE clears again.
    dispatch(
        &rec,
        INPUT,
        &[MarkerSpec::new(MarkerKind::Cube, "a", 1)],
        &mut state,
    );
    let _ = rendered(&rec, &storage);
    dispatch(&rec, INPUT, &retire, &mut state);
    assert_eq!(
        rendered(&rec, &storage).get(&entity_for(INPUT, "a", 1)),
        Some(&BTreeSet::from(["Clear".to_string()]))
    );
}

// The STATELESS marker arm that stood here is GONE with
// `archetype::log_frame_value`. Both properties it asserted are pinned on the
// PRODUCTION path in this file: a Cube marker drawing `{Transform3D, Boxes3D}`
// at its own `…/<ns>/id_<n>` entity by
// `each_marker_gets_its_own_entity_carrying_a_transform_and_its_geometry`, and
// an undecodable array degrading to ONE TextDocument at the topic entity — from
// the SAME `[0xAB; 37]` blob — by
// `an_undecodable_marker_array_degrades_to_a_dump_and_names_the_field_once`.
//
// Its third claim, "a DELETE emits nothing", is NOT ported and cannot be: it
// described the stateless path's INABILITY to clear (no live set to clear
// against), and on the production path the opposite is the contract, pinned by
// `a_delete_clears_exactly_its_own_entity_and_leaves_its_sibling_alone`.

#[traced_test]
#[test]
fn the_cleared_cap_warns_about_repeats_not_about_markers_being_live() {
    // THE arm the aggregated counter made impossible to state truthfully. The
    // CLEARED half of the ceiling fills on an id-cycling publisher that retires
    // what it replaces — and it fills with `live_count() == 0`, where a warn
    // saying "more than N markers LIVE at once" describes a state that is not
    // occurring and recommends a DELETEALL that would change nothing.
    let (rec, _storage) = memory();
    let mut state = SinkState::new();
    const CHUNK: i32 = 2_000;

    let retire_ids = |state: &mut SinkState, ids: std::ops::Range<i32>| {
        for chunk in ids.collect::<Vec<_>>().chunks(CHUNK as usize) {
            let adds: Vec<MarkerSpec> = chunk
                .iter()
                .map(|i| MarkerSpec::new(MarkerKind::Cube, "c", *i))
                .collect();
            dispatch(&rec, INPUT, &adds, state);
            let dels: Vec<MarkerSpec> = chunk.iter().map(|i| delete("c", *i)).collect();
            dispatch(&rec, INPUT, &dels, state);
        }
    };
    // Retire exactly the ceiling's worth, then two more.
    retire_ids(&mut state, 0..(MAX_LIVE_MARKERS as i32));
    assert_eq!(
        state.live_marker_count(INPUT),
        0,
        "premise: everything retired, NOTHING is live"
    );
    retire_ids(
        &mut state,
        (MAX_LIVE_MARKERS as i32)..(MAX_LIVE_MARKERS as i32 + 2),
    );
    assert_eq!(state.live_marker_count(INPUT), 0, "still nothing live");

    logs_assert(|lines: &[&str]| {
        let retired = lines
            .iter()
            .filter(|l| l.contains("RETIRED more than max_live_markers"))
            .count();
        // The LIVE-cap warn must NOT fire: nothing was ever live past the ceiling.
        let live = lines
            .iter()
            .filter(|l| l.contains("markers LIVE at once"))
            .count();
        match (retired, live) {
            (1, 0) => Ok(()),
            got => Err(format!(
                "want (cleared-cap warn, live-cap warn) = (1, 0), got {got:?}"
            )),
        }
    });
    // ...and the message names the real consequence + a remedy that applies here.
    assert!(logs_contain("re-emits"));
    assert!(logs_contain("already retired"));
}

#[traced_test]
#[test]
fn a_reconnect_re_arms_the_degradation_reports_for_the_new_session() {
    // The NOTES half of `reset_marker_state`, pinned BEHAVIORALLY. The
    // source-probe only greps the call string and the live-set test drives the
    // other half, so deleting `marker_notes.clear()` left the suite green.
    //
    // A reconnect starts a new operator-visible session: the fresh viewer's
    // operator has seen NONE of this run's diagnostics, so each must be restated
    // once. (It also bounds the latch set across a long run.)
    let (rec, _storage) = memory();
    let mut state = SinkState::new();
    let mut odd = MarkerSpec::new(MarkerKind::Cube, "a", 1);
    odd.kind = 4242;

    for _ in 0..3 {
        dispatch(&rec, INPUT, std::slice::from_ref(&odd), &mut state);
    }
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("Marker.type this build does not"))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!("before the reconnect: want 1 note, got {n}"))
        }
    });

    state.reset_marker_state(); // exactly what the reconnect hook calls

    for _ in 0..3 {
        dispatch(&rec, INPUT, std::slice::from_ref(&odd), &mut state);
    }
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("Marker.type this build does not"))
            .count();
        if n == 2 {
            Ok(())
        } else {
            Err(format!(
                "after the reconnect the note must be RESTATED exactly once \
                 (want 2 total), got {n}"
            ))
        }
    });
}

#[test]
fn a_marker_array_is_not_coalesced_so_a_delete_in_a_batch_survives() {
    // The data-loss pin, BEHAVIOURAL rather than a restatement of
    // `coalesces`: two frames drained in ONE tick where the first carries the
    // DELETE. If MarkerArray coalesced, only the newest would render and the
    // DELETE's Clear would never be emitted — a permanent ghost.
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    dispatch(
        &rec,
        INPUT,
        &[
            MarkerSpec::new(MarkerKind::Cube, "a", 1),
            MarkerSpec::new(MarkerKind::Cube, "a", 2),
        ],
        &mut state,
    );
    let _ = rendered(&rec, &storage);

    let mut staged: Option<Vec<u8>> = None;
    let mut coalesced = 0u64;
    for specs in [
        vec![delete("a", 1)],
        vec![MarkerSpec::new(MarkerKind::Cube, "a", 3)],
    ] {
        dispatch_or_stage(
            &rec,
            walker(),
            INPUT,
            marker_array_frame(&specs),
            &mut state,
            &mut staged,
            &mut coalesced,
        );
    }
    assert!(staged.is_none(), "nothing may be staged");
    assert_eq!(coalesced, 0, "nothing may be coalesced away");
    let seen = rendered(&rec, &storage);
    assert_eq!(
        seen.get(&entity_for(INPUT, "a", 1)),
        Some(&BTreeSet::from(["Clear".to_string()])),
        "the DELETE from the FIRST frame of the batch still cleared"
    );
    assert!(
        seen.contains_key(&entity_for(INPUT, "a", 3)),
        "and the second frame's marker still drew"
    );
}

#[test]
fn the_render_sequence_is_deterministic_across_two_identical_runs() {
    let script: Vec<Vec<MarkerSpec>> = vec![
        vec![
            MarkerSpec::new(MarkerKind::Cube, "zeta", 2),
            MarkerSpec::new(MarkerKind::Sphere, "alpha", 1).scale([2.0, 2.0, 2.0]),
        ],
        vec![delete("zeta", 2)],
        vec![
            delete_all(),
            MarkerSpec::new(MarkerKind::Points, "alpha", 1).points(&P),
        ],
    ];
    let run = || {
        let (rec, storage) = memory();
        let mut state = SinkState::new();
        let mut log = Vec::new();
        for specs in &script {
            dispatch(&rec, INPUT, specs, &mut state);
            log.push(rendered(&rec, &storage));
        }
        log
    };
    let a = run();
    let b = run();
    assert_eq!(a, b, "same frames in, same calls out");

    // ...and the third frame equals a HAND oracle, so neither leg is a
    // self-compare: `alpha/1` is re-added (drawn, never cleared) while `zeta/2`
    // was already gone, so the DELETEALL clears NOTHING.
    let alpha = entity_for(INPUT, "alpha", 1);
    assert_eq!(
        a[2].keys().cloned().collect::<Vec<_>>(),
        vec![alpha.clone()]
    );
    assert_eq!(
        a[2][&alpha],
        BTreeSet::from(["Transform3D".to_string(), "Points3D".to_string()])
    );
}

// ════════════════════════════════════════════════════════════════════════════
// 2b. COMPONENT VALUES — one per geometry family, against hand oracles.
//
// The archetype-NAME assertions above pin which family drew; these pin what it
// drew. Radii, colours, extents, vertex counts and the parent frame are all
// invisible to a name-only read, so without these a mapping could be entirely
// wrong in its numbers and still look green.
// ════════════════════════════════════════════════════════════════════════════

#[test]
fn a_cube_records_the_half_extents_and_pose_its_scale_implies() {
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    let mut spec = MarkerSpec::new(MarkerKind::Cube, "v", 1).scale([1.0, 2.0, 3.0]);
    spec.pos = [4.0, 5.0, 6.0];
    spec.color = [1.0, 0.0, 0.5, 1.0];
    dispatch(&rec, INPUT, &[spec], &mut state);
    let chunks = rendered_chunks(&rec, &storage);
    let entity = entity_for(INPUT, "v", 1);

    // rerun stores HALF sizes; ROS `scale` is the full edge length.
    let half: Vec<rerun::components::HalfSize3D> =
        component_at(&chunks, &entity, "Boxes3D:half_sizes");
    assert_eq!(half.len(), 1);
    assert_eq!(
        (half[0].x(), half[0].y(), half[0].z()),
        (0.5, 1.0, 1.5),
        "scale [1,2,3] ⇒ half sizes [0.5,1,1.5]"
    );
    // The marker's own pose rides the entity Transform3D (which is what poses the
    // geometry above — everything is logged marker-local).
    let t: Vec<rerun::components::Translation3D> =
        component_at(&chunks, &entity, "Transform3D:translation");
    assert_eq!(t.len(), 1);
    assert_eq!((t[0].x(), t[0].y(), t[0].z()), (4.0, 5.0, 6.0));
    // ColorRGBA 0..=1 floats → sRGB bytes.
    let colors: Vec<rerun::components::Color> = component_at(&chunks, &entity, "Boxes3D:colors");
    assert_eq!(colors.len(), 1);
    assert_eq!(colors[0].to_array(), [255, 0, 128, 255]);
    // A marker's pose is posed by its OWN parent_frame, and nothing
    // resolves here (no frame_id, no /tf), so it names its path parent explicitly
    // rather than omitting the component.
    let frames: Vec<rerun::components::TransformFrameId> =
        component_at(&chunks, &entity, "Transform3D:parent_frame");
    assert_eq!(
        frames.iter().map(|f| f.0.to_string()).collect::<Vec<_>>(),
        vec![format!("tf#/{}/{MARKER_CHILD}/v", topic_entity(INPUT))],
    );
}

#[test]
fn a_line_strip_records_its_vertices_and_the_radius_its_width_implies() {
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    dispatch(
        &rec,
        INPUT,
        &[MarkerSpec::new(MarkerKind::LineStrip, "v", 2)
            .scale([0.1, 0.0, 0.0])
            .points(&P)],
        &mut state,
    );
    let chunks = rendered_chunks(&rec, &storage);
    let entity = entity_for(INPUT, "v", 2);
    let radii: Vec<rerun::components::Radius> =
        component_at(&chunks, &entity, "LineStrips3D:radii");
    assert_eq!(radii.len(), 1);
    assert_eq!(radii[0].0 .0, 0.05, "ROS scale.x is the WIDTH ⇒ radius x/2");
    let strips: Vec<rerun::components::LineStrip3D> =
        component_at(&chunks, &entity, "LineStrips3D:strips");
    assert_eq!(strips.len(), 1, "ONE strip");
    let vertices: Vec<[f32; 3]> = strips[0].0.iter().map(|v| [v.x(), v.y(), v.z()]).collect();
    assert_eq!(vertices, PF.to_vec(), "verbatim marker-local points");
}

#[test]
fn a_points_marker_records_its_per_point_colors_and_radius() {
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    let mut spec = MarkerSpec::new(MarkerKind::Points, "v", 3)
        .scale([0.2, 0.2, 0.0])
        .points(&P);
    spec.colors = vec![
        [1.0, 0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0, 1.0],
        [0.0, 0.0, 1.0, 0.5],
    ];
    dispatch(&rec, INPUT, &[spec], &mut state);
    let chunks = rendered_chunks(&rec, &storage);
    let entity = entity_for(INPUT, "v", 3);
    let positions: Vec<rerun::components::Position3D> =
        component_at(&chunks, &entity, "Points3D:positions");
    assert_eq!(
        positions
            .iter()
            .map(|p| [p.x(), p.y(), p.z()])
            .collect::<Vec<_>>(),
        PF.to_vec()
    );
    let radii: Vec<rerun::components::Radius> = component_at(&chunks, &entity, "Points3D:radii");
    assert_eq!(radii[0].0 .0, 0.1);
    let colors: Vec<rerun::components::Color> = component_at(&chunks, &entity, "Points3D:colors");
    assert_eq!(
        colors.iter().map(|c| c.to_array()).collect::<Vec<_>>(),
        vec![[255, 0, 0, 255], [0, 255, 0, 255], [0, 0, 255, 128]],
        "the per-point bank really reaches rerun, one colour per point"
    );
}

#[test]
fn a_line_list_colors_each_segment_from_its_first_point() {
    // THE colour-pairing pin: a matched `colors` bank is PAIRED with the primitive rerun
    // actually renders — a LINE_LIST strip takes the colour of its first point.
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    let mut spec = MarkerSpec::new(MarkerKind::LineList, "v", 4)
        .scale([0.1, 0.0, 0.0])
        .points(&[P[0], P[1], P[2], [4.0, 0.0, 0.0]]);
    spec.colors = vec![
        [1.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0, 1.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    dispatch(&rec, INPUT, &[spec], &mut state);
    let chunks = rendered_chunks(&rec, &storage);
    let entity = entity_for(INPUT, "v", 4);
    let colors: Vec<rerun::components::Color> =
        component_at(&chunks, &entity, "LineStrips3D:colors");
    assert_eq!(
        colors.iter().map(|c| c.to_array()).collect::<Vec<_>>(),
        vec![[255, 0, 0, 255], [0, 0, 255, 255]],
        "TWO strips ⇒ two colours, each its strip's FIRST point (colors[0], colors[2])"
    );
}

#[test]
fn an_arrow_strip_colors_each_arrow_from_its_tail() {
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    let mut spec = MarkerSpec::new(MarkerKind::ArrowStrip, "v", 5)
        .scale([0.2, 0.0, 0.0])
        .points(&P);
    spec.colors = vec![
        [1.0, 0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0, 1.0],
        [0.0, 0.0, 1.0, 1.0],
    ];
    dispatch(&rec, INPUT, &[spec], &mut state);
    let chunks = rendered_chunks(&rec, &storage);
    let entity = entity_for(INPUT, "v", 5);
    let vectors: Vec<rerun::components::Vector3D> =
        component_at(&chunks, &entity, "Arrows3D:vectors");
    assert_eq!(
        vectors
            .iter()
            .map(|v| [v.x(), v.y(), v.z()])
            .collect::<Vec<_>>(),
        vec![[1.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
        "3 points ⇒ 2 arrows, each spanning a consecutive pair"
    );
    let colors: Vec<rerun::components::Color> = component_at(&chunks, &entity, "Arrows3D:colors");
    assert_eq!(
        colors.iter().map(|c| c.to_array()).collect::<Vec<_>>(),
        vec![[255, 0, 0, 255], [0, 255, 0, 255]],
        "each arrow takes its TAIL's colour; the last point starts no arrow"
    );
}

#[test]
fn a_line_strip_falls_back_to_the_flat_color_and_says_so() {
    // The other half of the colour pairing: rerun 0.34 gives a LineStrips3D ONE colour per
    // strip, so a per-VERTEX gradient is inexpressible. The bank is dropped
    // whole (never zipped), the flat colour is used, and it is REPORTED — as a
    // renderer limit, distinct from a producer-side length mismatch.
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    let mut spec = MarkerSpec::new(MarkerKind::LineStrip, "v", 6)
        .scale([0.1, 0.0, 0.0])
        .points(&P);
    spec.color = [0.0, 1.0, 0.0, 1.0];
    spec.colors = vec![
        [1.0, 0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0, 1.0],
    ];
    let plan = plan_of(std::slice::from_ref(&spec));
    assert_eq!(
        plan.reports.colors_dropped_kinds,
        vec![MarkerKind::LineStrip]
    );
    assert_eq!(
        plan.reports.color_length_mismatches, 0,
        "the bank is the right LENGTH — this is a renderer limit, not a producer bug"
    );
    dispatch(&rec, INPUT, &[spec], &mut state);
    let chunks = rendered_chunks(&rec, &storage);
    let colors: Vec<rerun::components::Color> =
        component_at(&chunks, &entity_for(INPUT, "v", 6), "LineStrips3D:colors");
    assert_eq!(
        colors.iter().map(|c| c.to_array()).collect::<Vec<_>>(),
        vec![[0, 255, 0, 255]],
        "the FLAT colour, once — never the first entry of the dropped bank"
    );
}

#[test]
fn a_triangle_list_records_scale_multiplied_vertices() {
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    dispatch(
        &rec,
        INPUT,
        &[MarkerSpec::new(MarkerKind::TriangleList, "v", 7)
            .scale([2.0, 2.0, 2.0])
            .points(&P)],
        &mut state,
    );
    let chunks = rendered_chunks(&rec, &storage);
    let vertices: Vec<rerun::components::Position3D> = component_at(
        &chunks,
        &entity_for(INPUT, "v", 7),
        "Mesh3D:vertex_positions",
    );
    assert_eq!(
        vertices
            .iter()
            .map(|p| [p.x(), p.y(), p.z()])
            .collect::<Vec<_>>(),
        vec![[2.0, 0.0, 0.0], [4.0, 0.0, 0.0], [6.0, 0.0, 0.0]],
        "ROS applies the marker scale to a triangle list's VERTICES"
    );
}

#[test]
fn a_sphere_records_half_sizes_and_a_cylinder_its_length_and_radius() {
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    dispatch(
        &rec,
        INPUT,
        &[
            MarkerSpec::new(MarkerKind::Sphere, "v", 8).scale([2.0, 4.0, 6.0]),
            MarkerSpec::new(MarkerKind::Cylinder, "v", 9).scale([2.0, 2.0, 5.0]),
        ],
        &mut state,
    );
    let chunks = rendered_chunks(&rec, &storage);
    let half: Vec<rerun::components::HalfSize3D> = component_at(
        &chunks,
        &entity_for(INPUT, "v", 8),
        "Ellipsoids3D:half_sizes",
    );
    assert_eq!(
        (half[0].x(), half[0].y(), half[0].z()),
        (1.0, 2.0, 3.0),
        "ROS scale is the DIAMETER per axis"
    );
    let cyl = entity_for(INPUT, "v", 9);
    let lengths: Vec<rerun::components::Length> =
        component_at(&chunks, &cyl, "Cylinders3D:lengths");
    let radii: Vec<rerun::components::Radius> = component_at(&chunks, &cyl, "Cylinders3D:radii");
    assert_eq!(lengths[0].0 .0, 5.0);
    assert_eq!(radii[0].0 .0, 1.0, "(scale.x + scale.y) / 4");
}

// ════════════════════════════════════════════════════════════════════════════
// 3. Loudness — every degradation named exactly once.
// ════════════════════════════════════════════════════════════════════════════

/// EVERY report arm fires on its own stimulus, exactly once, no matter how many
/// frames carry the condition.
///
/// **Every arm means every arm**: the table below drives all fourteen
/// `MarkerReports` fields plus the live-cap warn, which is the full set
/// `report_marker_plan` can emit. Each row is a (stimulus, marker substring)
/// pair; the stimulus is driven THREE times and the substring must appear
/// EXACTLY once — the once-per-`(input, discriminator)` contract, which is the
/// whole reason these are reports and not per-frame logs.
#[traced_test]
#[test]
fn every_marker_report_arm_fires_once_on_its_own_stimulus() {
    let big: Vec<[f64; 3]> = (0..(MAX_MARKER_VERTICES + 1))
        .map(|i| [i as f64, 0.0, 0.0])
        .collect();
    let mut unknown_type = MarkerSpec::new(MarkerKind::Cube, "r", 1);
    unknown_type.kind = 4242;
    let mut unknown_action = MarkerSpec::new(MarkerKind::Cube, "r", 2);
    unknown_action.action = 1;
    let mut colors_mismatch = MarkerSpec::new(MarkerKind::Points, "r", 3)
        .scale([0.2, 0.2, 0.0])
        .points(&P);
    colors_mismatch.colors = vec![[1.0, 0.0, 0.0, 1.0]];
    let mut colors_dropped = MarkerSpec::new(MarkerKind::LineStrip, "r", 4)
        .scale([0.1, 0.0, 0.0])
        .points(&P);
    colors_dropped.colors = vec![[1.0, 0.0, 0.0, 1.0]; 3];

    // (input, stimulus, the marker substring the arm's message must carry)
    let cases: Vec<(&str, Vec<MarkerSpec>, &str)> = vec![
        (
            "r_trunc_markers",
            (0..(MAX_MARKER_INSTANCES as i32 + 1))
                .map(|i| MarkerSpec::new(MarkerKind::Cube, "r", i))
                .collect(),
            "longer than max_marker_instances",
        ),
        (
            "r_trunc_vertices",
            vec![MarkerSpec::new(MarkerKind::Points, "r", 1)
                .scale([0.2, 0.2, 0.0])
                .points(&big)],
            "exceed max_marker_vertices",
        ),
        (
            "r_unknown_type",
            vec![unknown_type],
            "Marker.type this build does not",
        ),
        (
            "r_unknown_action",
            vec![unknown_action],
            "the message definition does not declare",
        ),
        (
            "r_colors_length",
            vec![colors_mismatch],
            "whose length does not",
        ),
        (
            "r_colors_dropped",
            vec![colors_dropped],
            "cannot express it",
        ),
        (
            "r_dropped_tail",
            vec![MarkerSpec::new(MarkerKind::LineList, "r", 1)
                .scale([0.1, 0.0, 0.0])
                .points(&P)],
            "does not fit its type's",
        ),
        (
            "r_elliptical",
            vec![MarkerSpec::new(MarkerKind::Cylinder, "r", 1).scale([1.0, 3.0, 2.0])],
            "elliptical cross-section",
        ),
        (
            "r_degenerate",
            vec![MarkerSpec::new(MarkerKind::Cube, "r", 1).scale([0.0, 0.0, 0.0])],
            "all-zero `scale`",
        ),
        (
            "r_lifetime",
            vec![MarkerSpec {
                lifetime: (5, 0),
                ..MarkerSpec::new(MarkerKind::Cube, "r", 1)
            }],
            "non-zero `lifetime`",
        ),
        (
            "r_frame_locked",
            vec![MarkerSpec {
                frame_locked: true,
                ..MarkerSpec::new(MarkerKind::Cube, "r", 1)
            }],
            "sets `frame_locked`",
        ),
        (
            "r_mesh",
            vec![MarkerSpec {
                mesh: "package://a/b.dae".to_string(),
                ..MarkerSpec::new(MarkerKind::MeshResource, "r", 1)
            }],
            "labelled PROXY BOX",
        ),
        (
            "r_frame_id",
            vec![MarkerSpec {
                frame_id: "map".to_string(),
                ..MarkerSpec::new(MarkerKind::Cube, "r", 1)
            }],
            "does NOT read",
        ),
        (
            // The `undecodable` arm, reached the way a REAL producer reaches it:
            // the walker degrades an invalid-UTF-8 string to `Bytes`, so a
            // TEXT_VIEW_FACING marker's `text` resolves to None and the marker
            // is dropped. Its explanatory log was previously untested.
            "r_undecodable",
            vec![MarkerSpec {
                raw_text: Some(vec![0xF0, 0x28, 0x8C, 0x28]),
                ..MarkerSpec::new(MarkerKind::TextViewFacing, "r", 1)
            }],
            "no usable body",
        ),
    ];

    let (rec, _storage) = memory();
    let mut state = SinkState::new();
    let mut markers: Vec<&str> = Vec::new();
    for (input, specs, marker) in &cases {
        for _ in 0..3 {
            dispatch(&rec, input, specs, &mut state);
        }
        markers.push(marker);
    }
    // The live-set overflow arm needs its own input driven past the ceiling.
    for chunk in (0..(MAX_LIVE_MARKERS as i32 + 2))
        .collect::<Vec<_>>()
        .chunks(2000)
    {
        let specs: Vec<MarkerSpec> = chunk
            .iter()
            .map(|i| MarkerSpec::new(MarkerKind::Cube, "r", *i))
            .collect();
        dispatch(&rec, "r_live_cap", &specs, &mut state);
    }
    markers.push("more than max_live_markers");

    let owned: Vec<String> = markers.iter().map(|m| (*m).to_string()).collect();
    logs_assert(move |lines: &[&str]| {
        for marker in &owned {
            let n = lines.iter().filter(|l| l.contains(marker.as_str())).count();
            if n != 1 {
                return Err(format!(
                    "expected exactly 1 line containing {marker:?}, got {n}"
                ));
            }
        }
        Ok(())
    });
}

#[traced_test]
#[test]
fn an_undecodable_marker_array_degrades_to_a_dump_and_names_the_field_once() {
    // The LIVE case is an rmw-published MarkerArray, whose variable element
    // bodies the walker refuses — the array degrades to
    // `NestedArrayOpaque`, which is the ONE cause of `Absent` on a valid frame.
    // The report must NAME `markers` (the actionable fact) and fire once per
    // (input, field), not per frame.
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    let frame = marker_array_frame_from_blob(&[0xABu8; 37], 42_000);
    for _ in 0..4 {
        dispatch_frame(&rec, walker(), INPUT, &frame, &mut state);
    }
    let seen = rendered(&rec, &storage);
    assert_eq!(
        seen.get(&topic_entity(INPUT))
            .map(|a| a.contains("TextDocument")),
        Some(true),
        "it renders the inspectable field dump at the topic entity"
    );
    assert!(state.took_anyvalues_fallback("visualization_msgs/MarkerArray"));
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| {
                l.contains("element bytes this build cannot decode") && l.contains("markers")
            })
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!("expected exactly 1 opaque-array line, got {n}"))
        }
    });
}

#[traced_test]
#[test]
fn an_unknown_marker_type_is_named_once_per_value_not_once_per_frame() {
    let (rec, _storage) = memory();
    let mut state = SinkState::new();
    let mut a = MarkerSpec::new(MarkerKind::Cube, "a", 1);
    a.kind = 99;
    let mut b = MarkerSpec::new(MarkerKind::Cube, "a", 2);
    b.kind = 77;
    for _ in 0..5 {
        dispatch(&rec, INPUT, &[a.clone(), b.clone()], &mut state);
    }
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("carries a Marker.type this build does not"))
            .count();
        if n == 2 {
            Ok(())
        } else {
            Err(format!("expected 2 lines (one per VALUE), got {n}"))
        }
    });
    assert!(logs_contain("marker_type=99"));
    assert!(logs_contain("marker_type=77"));
}

#[traced_test]
#[test]
fn a_mesh_resource_marker_draws_a_proxy_box_and_says_so_once_per_uri() {
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    let spec = MarkerSpec {
        mesh: "package://acme/meshes/base.dae".to_string(),
        ..MarkerSpec::new(MarkerKind::MeshResource, "a", 1).scale([1.0, 1.0, 1.0])
    };
    for _ in 0..3 {
        dispatch(&rec, INPUT, std::slice::from_ref(&spec), &mut state);
    }
    let seen = rendered(&rec, &storage);
    assert_eq!(
        seen.get(&entity_for(INPUT, "a", 1)),
        Some(&BTreeSet::from([
            "Transform3D".to_string(),
            "Boxes3D".to_string()
        ])),
        "a labelled PROXY box — never a fetched mesh"
    );
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("drawn as a labelled PROXY BOX"))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!("expected exactly 1 mesh note, got {n}"))
        }
    });
    assert!(logs_contain("package://acme/meshes/base.dae"));
}

#[traced_test]
#[test]
fn a_zero_scale_marker_still_draws_and_the_reason_is_logged_once() {
    let (rec, storage) = memory();
    let mut state = SinkState::new();
    let spec = MarkerSpec::new(MarkerKind::Cube, "a", 1).scale([0.0, 0.0, 0.0]);
    for _ in 0..4 {
        dispatch(&rec, INPUT, std::slice::from_ref(&spec), &mut state);
    }
    assert!(
        rendered(&rec, &storage).contains_key(&entity_for(INPUT, "a", 1)),
        "the marker still draws — nothing is substituted for the publisher's zero"
    );
    logs_assert(|lines: &[&str]| {
        let n = lines
            .iter()
            .filter(|l| l.contains("all-zero `scale`"))
            .count();
        if n == 1 {
            Ok(())
        } else {
            Err(format!("expected exactly 1 zero-scale note, got {n}"))
        }
    });
}

// ════════════════════════════════════════════════════════════════════════════
// The PRODUCTION render path records the live "provably rendering"
// signal the layout gates the dump companion on.
//
// The layout half is pure and pinned in `dump_companion_test.rs`; without these
// two arms it would be gated on a signal nothing ever sets — the inert-shipping
// shape. A `MarkerArray` is the ideal witness: the SAME builders here produce a
// frame its arm draws and a frame its arm refuses (an rmw-published array whose
// element bodies the walker cannot decode), so both directions are
// driven through the real `dispatch_frame` on one topic.
// ════════════════════════════════════════════════════════════════════════════

#[test]
fn a_frame_that_draws_records_a_clean_render_and_one_that_dumps_records_the_degradation() {
    const INPUT: &str = "markers";
    let (rec, _storage) = memory();
    let mut state = SinkState::new();

    // Nothing observed yet: the companion protection, and the state the layout
    // must read as "keep the companion".
    let before = state.render_proof_for(INPUT);
    assert!(
        !before.rendered_without_dumping && !before.degraded,
        "a topic no frame has reached must prove nothing, got {before:?}"
    );

    // (1) A frame its arm DRAWS.
    dispatch(
        &rec,
        INPUT,
        &[MarkerSpec::new(MarkerKind::Sphere, "ns", 1)],
        &mut state,
    );
    let drew = state.render_proof_for(INPUT);
    assert!(
        drew.rendered_without_dumping,
        "a MarkerArray the arm drew must record a clean render, got {drew:?}"
    );
    assert!(
        !drew.degraded,
        "and it must NOT record a degradation, got {drew:?}"
    );

    // (2) A frame its arm REFUSES — the undecodable element bodies of
    //     `an_undecodable_markers_array_is_absent_not_an_empty_plan`, which is
    //     what a real rmw skew produces.
    let junk = marker_array_frame_from_blob(&[0xABu8; 37], 42_000);
    dispatch_frame(&rec, walker(), INPUT, &junk, &mut state);
    let dumped = state.render_proof_for(INPUT);
    assert!(
        dumped.degraded,
        "a frame that took the field-dump fallback must record the degradation — \
         this is what makes the companion pane come BACK, got {dumped:?}"
    );
    assert!(
        dumped.rendered_without_dumping,
        "and the earlier clean render is not forgotten (both flags are sticky), \
         got {dumped:?}"
    );
}

#[test]
fn the_render_proof_is_per_input_not_per_schema() {
    // Two topics carrying the SAME schema: one bridged twin whose element bytes
    // this build cannot decode, one native producer that draws. A per-SCHEMA
    // verdict (the shape `took_anyvalues_fallback` has) would let either one
    // decide the other's layout — the bridged twin would strip the healthy
    // topic's pane, or the healthy one would hide the bridged topic's dump.
    const HEALTHY: &str = "healthy";
    const BRIDGED: &str = "bridged";
    let (rec, _storage) = memory();
    let mut state = SinkState::new();

    dispatch(
        &rec,
        HEALTHY,
        &[MarkerSpec::new(MarkerKind::Cube, "ns", 7)],
        &mut state,
    );
    let junk = marker_array_frame_from_blob(&[0xCDu8; 41], 7_000);
    dispatch_frame(&rec, walker(), BRIDGED, &junk, &mut state);

    let healthy = state.render_proof_for(HEALTHY);
    let bridged = state.render_proof_for(BRIDGED);
    assert!(
        healthy.rendered_without_dumping && !healthy.degraded,
        "the native producer is untouched by its bridged twin, got {healthy:?}"
    );
    assert!(
        bridged.degraded,
        "the bridged twin records its own degradation, got {bridged:?}"
    );
    // Anti-tautology: the two really are the same schema, so this is not two
    // unrelated topics passing trivially.
    assert!(
        state.took_anyvalues_fallback("visualization_msgs/MarkerArray"),
        "the per-SCHEMA latch DID fire — which is exactly why the per-input record \
         has to be separate from it"
    );
}

#[test]
fn a_later_clean_frame_does_not_clear_a_recorded_degradation() {
    // The RECORDER's half of stickiness. `dump_companion_test`'s sticky arm builds
    // the proof by hand, so it pins how the LAYOUT reads a latched flag and is
    // structurally blind to a recorder that un-latches it — measured: a
    // `note_render_native` that also cleared `degraded` passed the entire suite.
    //
    // A flapping topic is the real shape (an rmw producer whose element bodies this
    // build decodes on some frames and not others), and if the pane went away again
    // on the next good frame the operator could never READ the dump it appeared
    // for. So: degrade, then render cleanly, and the degradation must STAND.
    const INPUT: &str = "flapping";
    let (rec, _storage) = memory();
    let mut state = SinkState::new();

    let junk = marker_array_frame_from_blob(&[0xEFu8; 29], 1_000);
    dispatch_frame(&rec, walker(), INPUT, &junk, &mut state);
    assert!(
        state.render_proof_for(INPUT).degraded,
        "precondition: the undecodable frame must record a degradation"
    );

    for _ in 0..3 {
        dispatch(
            &rec,
            INPUT,
            &[MarkerSpec::new(MarkerKind::Sphere, "ns", 2)],
            &mut state,
        );
    }
    let after = state.render_proof_for(INPUT);
    assert!(
        after.degraded,
        "three healthy frames must NOT clear the degradation — a dump pane that \\
         vanished on the next good frame could never be read, got {after:?}"
    );
    assert!(
        after.rendered_without_dumping,
        "and the clean renders are still recorded, got {after:?}"
    );
}
