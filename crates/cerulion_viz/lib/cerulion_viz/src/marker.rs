// SPDX-License-Identifier: AGPL-3.0-only
//! `visualization_msgs/MarkerArray` — the 13 marker kinds and the
//! DELETE / DELETEALL entity-CLEAR mechanism.
//!
//! `MarkerArray` DECODES without this module: the
//! walker knows the canonical element framing, so `MarkerArray.markers` arrives as a
//! [`FrameValueKind::NestedArray`] of decoded `Marker` sub-frames. What it adds
//! is entirely viz-side, and it is three problems:
//!
//! 1. a **kind switch** over `Marker.type` — 13 values mapping to eight rerun
//!    archetypes, five of which the crate had never used (`Arrows3D`,
//!    `Ellipsoids3D`, `Cylinders3D`, `Mesh3D`, `Clear`);
//! 2. an **entity-CLEAR** mechanism, which the sink had none of;
//! 3. a **statefulness** change. Every other archetype the sink renders is
//!    stateless per frame — the newest frame REPLACES the last under rerun
//!    latest-at. A `MarkerArray` is a **stateful mutation stream**: a marker
//!    persists until it is explicitly deleted, and an incremental publisher
//!    updating one of fifty live markers sends a ONE-marker array. Auto-clearing
//!    markers merely absent from the current frame would erase the other 49, and
//!    a never-re-logged entity persists forever under latest-at — so the sink
//!    must NAME what it clears, which needs per-input state.
//!
//! # Layering
//!
//! Split exactly as the element ladder is split, so the correctness-critical
//! half tests against hand oracles with no rerun and no transport:
//!
//! - [`scan_marker_array`] — PURE `FrameValue` → [`MarkerArrayPlan`]: the ops in
//!   declaration order plus every reported degradation the frame earned.
//! - [`resolve_marker_ops`] — PURE `(ops, live set)` → [`MarkerFrameActions`]:
//!   which entities to CLEAR, which markers to DRAW, in ROS's own in-order
//!   semantics. Also where the live set is advanced.
//! - [`log_marker_draw`] / [`log_marker_clear`] — the thin rerun half.
//!
//! # The entity path
//!
//! ```text
//! <route entity>/viz-markers/<ns segment>/<id segment>
//!       ▲             ▲            ▲            ▲
//!       │             │            │            └─ id_7 / id_n3 (negative → n)
//!       │             │            └─ crate::tf::sanitize_segment(ns)
//!       │             └─ MARKER_CHILD, RESERVED (see its doc)
//!       └─ crate::sink::route_for_input(name).entity
//! ```
//!
//! One entity per LIVE marker, because per-marker `DELETE` is unimplementable
//! without it — that is most of this issue. The [`MARKER_CHILD`] segment between
//! the topic entity and the namespace is the reserved-child convention
//! (`viz-sweep` / `viz-vertices`): a robot publishing BOTH `/markers` and
//! `/markers/costmap` would otherwise land the second topic's data in the first
//! topic's `costmap` namespace subtree.
//!
//! # The same-timestamp clear-vs-add invariant, and its exact scope
//!
//! [`resolve_marker_ops`] guarantees that **no entity is both cleared and drawn
//! at one timestamp** — rerun's `Clear` docs define before/after in TIME and say
//! nothing about a tie, so the case is designed out rather than bet on.
//!
//! **That guarantee is per FRAME, and the sink does not batch across frames.** A
//! publisher that sends `DELETEALL` and its fresh set as TWO messages stamped
//! with the same wire timestamp — the two-message idiom is legal ROS — can still
//! clear and draw one entity at the same instant, and how rerun resolves that is
//! the unknown this design otherwise avoids. Nothing here detects or merges such
//! a pair; the mitigation is to send both in ONE array, which is also the
//! dominant real idiom.
//!
//! # Determinism (Principle #7)
//!
//! Everything here is a pure function of `(frame bytes, live set)`: timestamps
//! come from the wire header via [`crate::archetype::set_robot_time`], clears are
//! emitted in `BTreeSet` (sorted) order, draws in array declaration order, the
//! entity path is a pure function of `(ns, id)`, and no map iterated for output
//! is a `HashMap`. Replaying the same frames produces the same call sequence.
//!
//! # What v1 does NOT honour
//!
//! `lifetime`, `frame_locked`, `header.frame_id` (per marker), and
//! `mesh_resource` fetching. Each is NAMED once per input rather than silently
//! dropped — see [`MarkerArrayPlan`]'s report fields and
//! `crate::sink`'s marker report helper.

use std::collections::{BTreeMap, BTreeSet};

use cerulion_core::codegen::{FrameValue, FrameValueKind};
use rerun::RecordingStream;

use crate::archetype::{set_robot_time, PoseParts};
use crate::tf::sanitize_segment;

/// The SYNTHETIC child segment every marker entity hangs off
/// (`<route entity>/viz-markers/<ns>/<id>`).
///
/// **The `-` is load-bearing** — the same structural argument as
/// [`crate::archetype::PATH_VERTICES_CHILD`] and [`crate::sink::SWEEP_CHILD`]. A
/// marker's NAMESPACE is an arbitrary publisher-chosen string, and marker
/// entities are synthesized UNDER a topic's entity, so they share a namespace
/// with real topics: under the mechanical `world/<full topic>` rule a robot
/// publishing both `/markers` and `/markers/costmap` would put the second topic's
/// data on the first topic's `costmap` marker subtree. [`sanitize_segment`] emits
/// only `[A-Za-z0-9_]`, so a segment containing `-` is unreachable from any topic
/// name AND from any marker namespace — which makes this level a structural
/// separator, not a naming convention.
pub const MARKER_CHILD: &str = "viz-markers";

/// The most markers ONE `MarkerArray` frame is inspected for.
///
/// # Why this is NOT [`crate::archetype::MAX_ELEMENT_INSTANCES`]
///
/// That constant (300 000) is its MEASURED ceiling for the element-array
/// path, where N elements become ONE rerun archetype with N instances at ONE
/// entity — measured at 0.32 µs/element for the geometry scan + archetype
/// construction + log. **That measurement does not transfer here.** A marker is
/// its own ENTITY carrying its own `Transform3D` plus its own geometry
/// archetype, so N markers are N entities and ~2N rerun log calls — a per-ENTITY
/// cost, on a different axis from per-instance. Reusing the element ceiling
/// would apply a measurement to an axis it does not cover; 300 000 markers would
/// be 600 000 log calls in one frame.
///
/// 10 000 is a REASONED ceiling, not a measured one (the design says the
/// same): it is already two orders of magnitude past a normal costmap or
/// footprint overlay, and it bounds the per-frame entity fan-out. It has not been
/// measured the way `tests/element_cap_bench.rs` measured the element
/// cap.
///
/// # What is lost past the ceiling
///
/// Elements past this index are not inspected AT ALL, so a `DELETE` /
/// `DELETEALL` in the dropped tail is **not applied** and its markers can ghost.
/// That is reported once per input, in those words — the dominant real idiom
/// puts `DELETEALL` at `markers[0]`, which is always inspected.
pub const MAX_MARKER_INSTANCES: usize = 10_000;

/// The ceiling applied to EACH of one input's two tracking sets — the LIVE
/// markers, and separately the RETIRED-and-not-since-redrawn ones (see
/// [`MarkerLiveState`]). It is a per-set bound, so the worst-case tracked total
/// for one input is twice this.
///
/// **A different axis from [`MAX_MARKER_INSTANCES`], which is why it is a
/// different constant.** That one bounds ONE FRAME's inspection; these bound
/// CUMULATIVE sets that grow across frames and are only emptied by a `DELETEALL`
/// (live) or a viewer reconnect (both) — so an id-cycling publisher (a fresh
/// `(ns, id)` per frame) fills them over time even while every individual frame
/// is tiny.
///
/// Policy at either ceiling is FIRST-COME, NEVER-EVICT — the EARLIEST keys keep
/// their slots and later ones go untracked, rather than the reverse. Nothing is
/// mis-drawn either way; what is lost differs per set:
///
/// - LIVE full ⇒ the marker still DRAWS and an explicit `DELETE` still CLEARS it
///   (that is precisely why a DELETE does not gate on tracking), but a
///   `DELETEALL` cannot sweep it — it can only enumerate what it tracks;
/// - CLEARED full ⇒ the retirement is not remembered, so an identical repeated
///   `DELETE` re-emits its `Clear` every frame instead of once.
///
/// Both residuals are named by their own once-per-input overflow warns.
pub const MAX_LIVE_MARKERS: usize = 10_000;

/// The most `points` vertices ONE `MarkerArray` frame renders, summed across all
/// of its markers.
///
/// [`crate::archetype::MAX_ELEMENT_INSTANCES`] has no analog on this axis and one
/// is needed: `Marker`'s `max_slice_len` admits a single marker carrying ~170 000
/// `Point`s, so a bounded MARKER count alone still permits an unbounded per-frame
/// vertex buffer. 200 000 vertices is already denser than most rendered point
/// clouds.
///
/// Consumed in array order. When the NEXT marker's vertex count would exceed the
/// remaining budget that marker is skipped **whole** — never truncated
/// mid-geometry. A half-drawn `LINE_STRIP`, or a `TRIANGLE_LIST` missing its last
/// triangles, is exactly the "plausible but wrong" picture the walker itself
/// refuses to fabricate.
pub const MAX_MARKER_VERTICES: usize = 200_000;

/// The `MarkerArray` field carrying the markers (the vendored
/// `visualization_msgs/MarkerArray.msg` has exactly this one field).
const MARKERS_FIELD: &str = "markers";

// ────────────────────────────────────────────────────────────────────────────
// The wire vocabulary. `parse_rosmsg` DROPS constant lines, so the generated
// `visualization_msgs.rs` carries no `ARROW` / `DELETEALL` consts — these are
// declared here by hand from the vendored `.msg` and pinned by an oracle test
// that re-reads that same `.msg` text out of `native_ros2_messages::BUILTIN_MSGS`
// (so a re-vendor cannot silently drift them, which is exactly what happened
// between this issue's design and its implementation: a re-vendor of
// `Marker.msg`, adding `ARROW_STRIP=12` and four trailing fields).
// ────────────────────────────────────────────────────────────────────────────

/// One `Marker.type` value — the CLOSED set the vendored `Marker.msg` declares.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum MarkerKind {
    /// `ARROW` — one arrow, either pose-anchored (`scale.x` long along local +X)
    /// or spanning `points[0] → points[1]`.
    Arrow,
    /// `CUBE` — one box of full edge lengths `scale`.
    Cube,
    /// `SPHERE` — one ellipsoid of DIAMETERS `scale`.
    Sphere,
    /// `CYLINDER` — one +Z-aligned cylinder, height `scale.z`, diameters
    /// `scale.x`/`scale.y`.
    Cylinder,
    /// `LINE_STRIP` — one polyline through `points`, width `scale.x`.
    LineStrip,
    /// `LINE_LIST` — one segment per consecutive PAIR of `points`.
    LineList,
    /// `CUBE_LIST` — one `scale`-sized box centred at each of `points`.
    CubeList,
    /// `SPHERE_LIST` — one `scale`-diameter ellipsoid centred at each of `points`.
    SphereList,
    /// `POINTS` — `points` as dots of width `scale.x`.
    Points,
    /// `TEXT_VIEW_FACING` — the `text` string, billboarded.
    TextViewFacing,
    /// `MESH_RESOURCE` — a mesh named by the `mesh_resource` URI.
    MeshResource,
    /// `TRIANGLE_LIST` — `points` read as consecutive vertex TRIPLES.
    TriangleList,
    /// `ARROW_STRIP` — an arrow between each consecutive pair of `points`
    /// (added to the message by upstream `visualization_msgs`; vendored
    /// later, which is why the design — written against the older
    /// 12-constant copy — does not mention it).
    ArrowStrip,
}

impl MarkerKind {
    /// Every kind, in wire-value order. Oracle tables iterate this, so they
    /// inherit its totality.
    pub const ALL: [MarkerKind; 13] = [
        MarkerKind::Arrow,
        MarkerKind::Cube,
        MarkerKind::Sphere,
        MarkerKind::Cylinder,
        MarkerKind::LineStrip,
        MarkerKind::LineList,
        MarkerKind::CubeList,
        MarkerKind::SphereList,
        MarkerKind::Points,
        MarkerKind::TextViewFacing,
        MarkerKind::MeshResource,
        MarkerKind::TriangleList,
        MarkerKind::ArrowStrip,
    ];

    /// The `Marker.type` wire value.
    pub fn wire_value(self) -> i32 {
        match self {
            MarkerKind::Arrow => 0,
            MarkerKind::Cube => 1,
            MarkerKind::Sphere => 2,
            MarkerKind::Cylinder => 3,
            MarkerKind::LineStrip => 4,
            MarkerKind::LineList => 5,
            MarkerKind::CubeList => 6,
            MarkerKind::SphereList => 7,
            MarkerKind::Points => 8,
            MarkerKind::TextViewFacing => 9,
            MarkerKind::MeshResource => 10,
            MarkerKind::TriangleList => 11,
            MarkerKind::ArrowStrip => 12,
        }
    }

    /// The constant NAME as the vendored `.msg` spells it (the oracle test's
    /// cross-check key, and what the once-per-input reports quote).
    pub fn wire_name(self) -> &'static str {
        match self {
            MarkerKind::Arrow => "ARROW",
            MarkerKind::Cube => "CUBE",
            MarkerKind::Sphere => "SPHERE",
            MarkerKind::Cylinder => "CYLINDER",
            MarkerKind::LineStrip => "LINE_STRIP",
            MarkerKind::LineList => "LINE_LIST",
            MarkerKind::CubeList => "CUBE_LIST",
            MarkerKind::SphereList => "SPHERE_LIST",
            MarkerKind::Points => "POINTS",
            MarkerKind::TextViewFacing => "TEXT_VIEW_FACING",
            MarkerKind::MeshResource => "MESH_RESOURCE",
            MarkerKind::TriangleList => "TRIANGLE_LIST",
            MarkerKind::ArrowStrip => "ARROW_STRIP",
        }
    }

    /// Resolve a wire `type` value, or `None` for a value this build does not
    /// know (reported once per `(input, value)`, marker not rendered — never a
    /// guessed shape).
    pub fn from_wire(value: i32) -> Option<Self> {
        Self::ALL.into_iter().find(|k| k.wire_value() == value)
    }

    /// Whether this kind RENDERS `Marker.points` — the vertex-budget axis.
    ///
    /// The single-primitive kinds ignore `points` entirely, so charging their
    /// (stray, already length-mismatched) bank against
    /// [`MAX_MARKER_VERTICES`] would evict LATER markers that really do draw
    /// geometry. `ARROW` is here because its TWO-point form is real geometry.
    pub fn consumes_points(self) -> bool {
        match self {
            MarkerKind::Arrow
            | MarkerKind::ArrowStrip
            | MarkerKind::LineStrip
            | MarkerKind::LineList
            | MarkerKind::CubeList
            | MarkerKind::SphereList
            | MarkerKind::Points
            | MarkerKind::TriangleList => true,
            MarkerKind::Cube
            | MarkerKind::Sphere
            | MarkerKind::Cylinder
            | MarkerKind::TextViewFacing
            | MarkerKind::MeshResource => false,
        }
    }
}

/// One `Marker.action` value — the CLOSED set the vendored `Marker.msg`
/// declares. `ADD` and `MODIFY` are both `0` in ROS, so they are ONE variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum MarkerAction {
    /// `ADD` == `MODIFY` == 0.
    Add,
    /// `DELETE` == 2 — clear this ONE `(ns, id)`.
    Delete,
    /// `DELETEALL` == 3 — clear every marker this input has live.
    DeleteAll,
}

impl MarkerAction {
    /// Every action, in wire-value order.
    pub const ALL: [MarkerAction; 3] = [
        MarkerAction::Add,
        MarkerAction::Delete,
        MarkerAction::DeleteAll,
    ];

    /// The `Marker.action` wire value.
    pub fn wire_value(self) -> i32 {
        match self {
            MarkerAction::Add => 0,
            MarkerAction::Delete => 2,
            MarkerAction::DeleteAll => 3,
        }
    }

    /// Resolve a wire `action` value, or `None` for a value the `.msg` does not
    /// declare (`1` is genuinely unassigned in ROS). Reported once per
    /// `(input, value)`; the marker is NOT rendered, because "what did the
    /// publisher mean" has no defined answer.
    pub fn from_wire(value: i32) -> Option<Self> {
        Self::ALL.into_iter().find(|a| a.wire_value() == value)
    }
}

/// A marker's IDENTITY — the `(ns, id)` pair ROS keys marker lifetime on.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct MarkerKey {
    /// The raw (un-sanitized) `Marker.ns`.
    pub ns: String,
    /// The raw `Marker.id` (may be negative).
    pub id: i32,
}

impl MarkerKey {
    /// The `<ns segment>/<id segment>` entity-path TAIL for this marker — the
    /// string the live set stores and [`marker_entity`] appends.
    ///
    /// The namespace goes through [`sanitize_segment`], the ONE crate sanitizer,
    /// whose 4-hex FNV suffix keeps raw names that differ only in separators
    /// apart: without it `nav/local` and `nav.local` would collapse onto one
    /// namespace and two publishers would silently overwrite each other. The
    /// EMPTY namespace (extremely common) is handled for free — it sanitizes to
    /// `unknown_<hhhh>`, which a real namespace spelled `unknown` can never
    /// equal, since an already-valid name is returned verbatim.
    ///
    /// The id is CONSTRUCTED, not sanitized: `id` is an `i32` and may be
    /// negative, and sanitizing `-3` under the alnum rule would yield `_3` —
    /// colliding with a hypothetical namespace-mate. `id_7` / `id_n3` is total,
    /// injective, and uses only characters rerun leaves unescaped.
    pub fn entity_key(&self) -> String {
        let id = if self.id >= 0 {
            format!("id_{}", self.id)
        } else {
            format!("id_n{}", self.id.unsigned_abs())
        };
        format!("{}/{}", sanitize_segment(&self.ns), id)
    }
}

/// The full entity path a marker renders at:
/// `<route entity>/`[`MARKER_CHILD`]`/<entity key>`.
pub fn marker_entity(route_entity: &str, entity_key: &str) -> String {
    format!("{route_entity}/{MARKER_CHILD}/{entity_key}")
}

/// The renderable geometry ONE marker resolves to — expressed in MARKER-LOCAL
/// coordinates, because the marker's own `pose` is logged as a `Transform3D` at
/// the SAME entity and rerun poses an entity's own geometry by it. That is what
/// makes this module free of coordinate math (there is no `glam` feature on this
/// rerun build, so hand-rolled quaternion rotation would be the alternative), and
/// it is exactly rviz's model: `Marker.pose` transforms the primitive AND its
/// `points`.
#[derive(Debug, Clone, PartialEq)]
pub enum MarkerGeometry {
    /// `ARROW` / `ARROW_STRIP` → [`rerun::Arrows3D`]. `origins[i] + vectors[i]`
    /// is the tip; `radius` is the shaft radius.
    Arrows {
        /// Arrow tails.
        origins: Vec<[f32; 3]>,
        /// Tail→tip vectors.
        vectors: Vec<[f32; 3]>,
        /// Shaft radius.
        radius: f32,
    },
    /// `CUBE` / `CUBE_LIST` → [`rerun::Boxes3D`]. `size` is FULL edge lengths.
    Boxes {
        /// One box per centre.
        centers: Vec<[f32; 3]>,
        /// Full edge lengths, shared by every instance.
        size: [f32; 3],
    },
    /// `SPHERE` / `SPHERE_LIST` → [`rerun::Ellipsoids3D`]. ROS `scale` is the
    /// DIAMETER per axis, so this carries `scale / 2`.
    Ellipsoids {
        /// One ellipsoid per centre.
        centers: Vec<[f32; 3]>,
        /// Half sizes (ROS `scale / 2`), shared by every instance.
        half_size: [f32; 3],
    },
    /// `CYLINDER` → [`rerun::Cylinders3D`], centred at the marker origin and
    /// +Z-aligned — the same axis ROS uses, so the orientation comes for free.
    Cylinder {
        /// Height (ROS `scale.z`).
        length: f32,
        /// Radius (ROS `(scale.x + scale.y) / 4`; an ELLIPTICAL cross-section
        /// degrades to circular and is reported once per input).
        radius: f32,
    },
    /// `LINE_STRIP` (one strip) / `LINE_LIST` (one two-point strip per pair) →
    /// [`rerun::LineStrips3D`].
    LineStrips {
        /// The strips.
        strips: Vec<Vec<[f32; 3]>>,
        /// Line radius (ROS `scale.x / 2`).
        radius: f32,
    },
    /// `POINTS` → [`rerun::Points3D`].
    ///
    /// **A partial degrade, stated like `Text`'s**: ROS gives a POINTS marker a
    /// `scale.x` WIDTH and a `scale.y` HEIGHT, but a rerun point is round and
    /// takes a single radius, so `scale.y` is dropped. Non-square points are not
    /// expressible; the width is honoured.
    Points {
        /// The dots.
        positions: Vec<[f32; 3]>,
        /// Dot radius (ROS `scale.x / 2`; `scale.y` is dropped — see above).
        radius: f32,
    },
    /// `TEXT_VIEW_FACING` → a ZERO-radius labelled [`rerun::Points3D`].
    ///
    /// **A degrade, stated plainly**: rerun 0.34 has no 3D-anchored text
    /// archetype (no `Text3D`, no `Label3D`; `TextLog` is a timeline view, not
    /// in-scene), so a labelled point is the only in-scene text mechanism. ROS's
    /// `scale.z` text height has no equivalent and is dropped.
    Text {
        /// The string.
        text: String,
    },
    /// `MESH_RESOURCE` → a [`rerun::Boxes3D`] PROXY of size `scale`, labelled
    /// with the URI.
    ///
    /// **v1 does not fetch the mesh.** `mesh_resource` is almost always
    /// `package://<pkg>/meshes/foo.dae`, and resolving that needs the robot's
    /// package tree — which the brand-new laptop attaching to an unseen robot
    /// does not have. rerun's mesh entry points all need BYTES, so fetching would
    /// put network / filesystem IO on the render path. A labelled proxy at the
    /// right pose and size is strictly better than the earlier text dump and
    /// never lies about geometry.
    MeshProxy {
        /// Proxy box full edge lengths (ROS `scale`).
        size: [f32; 3],
        /// The un-fetched URI, rendered as the proxy's label.
        uri: String,
    },
    /// `TRIANGLE_LIST` → [`rerun::Mesh3D`]. rerun's default topology IS an
    /// implicit triangle list ("each triplet of positions is interpreted as a
    /// triangle"), so no `triangle_indices` are needed. Vertices are already
    /// `scale`-multiplied (ROS applies the marker scale to a triangle list's
    /// points).
    Triangles {
        /// Vertices, a multiple of three (a trailing 1–2 are dropped and
        /// reported).
        vertices: Vec<[f32; 3]>,
    },
}

impl MarkerGeometry {
    /// True when this geometry would put NOTHING on screen — an empty point
    /// list, or a text marker with no text.
    ///
    /// Used by [`resolve_marker_ops`] to turn such an `ADD` into a CLEAR: under
    /// rerun latest-at, drawing nothing leaves the PREVIOUS frame's geometry
    /// standing as the current value, so the common "shrink the array to zero =
    /// nothing detected this cycle" idiom would show stale data indefinitely.
    ///
    /// The single-primitive kinds (`CUBE`, `SPHERE`, `CYLINDER`, `MESH_RESOURCE`)
    /// always render something — possibly zero-SIZED, which is the publisher's
    /// own instruction and is reported via
    /// [`MarkerReports::degenerate_scales`], not silently converted to a delete.
    pub fn renders_nothing(&self) -> bool {
        match self {
            MarkerGeometry::Arrows { vectors, .. } => vectors.is_empty(),
            MarkerGeometry::Boxes { centers, .. } | MarkerGeometry::Ellipsoids { centers, .. } => {
                centers.is_empty()
            }
            MarkerGeometry::LineStrips { strips, .. } => strips.is_empty(),
            MarkerGeometry::Points { positions, .. } => positions.is_empty(),
            MarkerGeometry::Triangles { vertices } => vertices.is_empty(),
            MarkerGeometry::Text { text } => text.is_empty(),
            MarkerGeometry::Cylinder { .. } | MarkerGeometry::MeshProxy { .. } => false,
        }
    }
}

/// One marker to DRAW.
#[derive(Debug, Clone, PartialEq)]
pub struct MarkerDraw {
    /// Its `(ns, id)` identity.
    pub key: MarkerKey,
    /// Its pose, logged as a `Transform3D` at the marker entity (which poses the
    /// geometry below).
    pub pose: PoseParts,
    /// Its geometry, in marker-local coordinates.
    pub geometry: MarkerGeometry,
    /// The flat `Marker.color`, `[r, g, b, a]` in 0..=255.
    pub color: [u8; 4],
    /// One colour per rendered PRIMITIVE of [`MarkerDraw::geometry`] — a point
    /// for `POINTS`, a box for `CUBE_LIST`, a STRIP for `LINE_LIST`, an ARROW for
    /// `ARROW_STRIP`, a VERTEX for `TRIANGLE_LIST`.
    ///
    /// Derived from `Marker.colors` in [`scan_marker_array`], so the pairing is
    /// oracle-testable. Present only when the bank's length matched `points`
    /// AND the kind's rerun archetype can express per-primitive colour: a
    /// mismatched length is a producer bug (silently zipping would mis-colour the
    /// tail) and an inexpressible pairing (`LINE_STRIP`'s per-vertex gradient — a
    /// rerun 0.34 `LineStrips3D` takes ONE colour per strip; a single `ARROW`,
    /// which is one primitive with a bank of several) is a rerun limit. Both fall
    /// back to the flat colour, and both are reported once per input.
    pub instance_colors: Option<Vec<[u8; 4]>>,
}

/// One resolved element of a `MarkerArray` — what the publisher asked for.
#[derive(Debug, Clone, PartialEq)]
pub enum MarkerOp {
    /// `ADD` / `MODIFY`.
    Draw(Box<MarkerDraw>),
    /// `DELETE` of one `(ns, id)`.
    Delete(MarkerKey),
    /// `DELETEALL`.
    DeleteAll,
}

/// A decoded `MarkerArray` frame: the ops in DECLARATION order plus every
/// degradation the frame earned.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MarkerArrayPlan {
    /// The ops, in array declaration order.
    pub ops: Vec<MarkerOp>,
    /// Everything the frame degraded, for the sink's once-per-input reports.
    pub reports: MarkerReports,
}

impl MarkerArrayPlan {
    /// True when the frame earned no degradation report at all.
    ///
    /// Delegates to [`MarkerReports::is_clean`], which is what the sink actually
    /// calls — see that method for the divergence an `is_clean()`-only early
    /// return would miss.
    pub fn is_clean(&self) -> bool {
        self.reports.is_clean()
    }
}

impl MarkerReports {
    /// True when this frame earned no degradation report at all.
    ///
    /// Defined as `*self == Self::default()` rather than a hand-written
    /// conjunction, so it is TOTAL BY CONSTRUCTION: a new report field is covered
    /// the moment it is added, with no second place to remember. (The report
    /// fields live in their own struct precisely so this comparison is possible
    /// without cloning a plan's `ops`.)
    ///
    /// **It is NOT the sink's whole quiet condition.** The sink additionally
    /// requires both [`MarkerFrameActions`] overflow counts to be zero, and those
    /// are NOT frame-decode facts — they come from resolving the frame against
    /// per-input state, so they cannot live in this struct. An early return on
    /// `is_clean()` alone would silently swallow the two cap warns.
    pub fn is_clean(&self) -> bool {
        *self == Self::default()
    }
}

/// Every degradation ONE `MarkerArray` frame earned.
///
/// Each field feeds a once-per-`(input, discriminator)` log in the sink —
/// nothing here is dropped silently, and nothing is logged per frame. Kept as
/// its OWN struct so [`MarkerArrayPlan::is_clean`] can be a total comparison
/// against `Default` instead of a hand-maintained conjunction that a new field
/// could silently escape.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MarkerReports {
    /// Markers past [`MAX_MARKER_INSTANCES`] — NOT inspected, so an action in
    /// this tail is not applied either.
    pub truncated_markers: usize,
    /// Markers skipped WHOLE because their vertices would exceed
    /// [`MAX_MARKER_VERTICES`].
    pub truncated_vertices: usize,
    /// Unknown `Marker.type` values, sorted + deduped. Those markers are not
    /// rendered; every OTHER marker in the array still is (see the module docs on
    /// partial rendering).
    pub unknown_types: Vec<i32>,
    /// Unknown `Marker.action` values, sorted + deduped.
    pub unknown_actions: Vec<i32>,
    /// Elements that carried no usable body (not a decoded message, or missing a
    /// required field).
    pub undecodable: usize,
    /// Markers whose non-empty `colors` length did not match `points`, so the
    /// flat `color` was used.
    pub color_length_mismatches: usize,
    /// Kinds whose per-primitive colours rerun 0.34 CANNOT express, so a
    /// length-MATCHED `colors` bank was dropped in favour of the flat colour,
    /// sorted + deduped. A `LINE_STRIP` is one strip with one colour (a
    /// per-vertex gradient has no component); a single `ARROW` is one primitive.
    /// Distinct from [`MarkerReports::color_length_mismatches`], which is a
    /// producer bug — this is a renderer limit, and conflating them would send an
    /// operator to fix a bank that is already correct.
    pub colors_dropped_kinds: Vec<MarkerKind>,
    /// Kinds that DROPPED a trailing point (`LINE_LIST` with an odd count,
    /// `TRIANGLE_LIST` with a count not divisible by three, `ARROW_STRIP` /
    /// `LINE_STRIP` with a single point), sorted + deduped.
    pub dropped_tail_kinds: Vec<MarkerKind>,
    /// `CYLINDER` markers whose `scale.x != scale.y` — an elliptical
    /// cross-section rerun cannot express, drawn circular.
    pub elliptical_cylinders: usize,
    /// Markers whose kind CONSUMES `scale` but whose relevant scale components
    /// are all zero — they still render (invisibly), and the report is the only
    /// way an operator learns why nothing appeared.
    pub degenerate_scales: usize,
    /// Markers carrying a NON-ZERO `lifetime`, which v1 does not expire.
    pub lifetime_markers: usize,
    /// Markers carrying `frame_locked`, which v1 does not honour.
    pub frame_locked_markers: usize,
    /// `mesh_resource` URIs drawn as proxy boxes, sorted + deduped.
    pub mesh_uris: Vec<String>,
    /// Markers stamped with a non-empty `header.frame_id`, which v1 does NOT
    /// read — every marker renders in its topic's frame rather than its own, so
    /// a marker published in `map` and one in `base_link` land in the same place.
    ///
    /// This is the platform-wide element-frame gap, not a MarkerArray
    /// one; the report exists so the deferral is LOUD rather than merely
    /// documented.
    pub markers_with_frame_id: usize,
}

/// What [`scan_marker_array`] found — THREE outcomes, for the same reason
/// [`crate::archetype::ElementArrayScan`] has three: "an idle MarkerArray
/// carrying zero markers" and "a producer whose element framing cannot be
/// decoded" are different facts that must not share a code path.
#[derive(Debug, Clone, PartialEq)]
pub enum MarkerArrayScan {
    /// A decoded plan (possibly with degradations). BOXED because the plan is by
    /// far the largest variant (its `MarkerReports` alone is ~200 bytes) and this
    /// enum is returned by value on every frame, including the two empty arms.
    Plan(Box<MarkerArrayPlan>),
    /// The `markers` array decoded and is EMPTY this frame.
    ///
    /// Draws nothing, takes NO fallback, and — critically — does **not** clear.
    /// An empty array is not a `DELETEALL`: a publisher that stops sending
    /// markers has not asked for the live ones to be erased, and degrading to a
    /// field dump here would flip the topic between geometry and a
    /// `TextDocument` as the array empties and refills.
    Empty,
    /// No decodable `markers` array — no such field, or its element bytes were
    /// refused by the walker (`NestedArrayOpaque`; the live case is an
    /// rmw-published MarkerArray). The caller degrades to the
    /// element-enumerating field dump and names why.
    Absent,
}

// ────────────────────────────────────────────────────────────────────────────
// PURE: FrameValue → MarkerArrayPlan
// ────────────────────────────────────────────────────────────────────────────

/// Decode a walked `visualization_msgs/MarkerArray` frame into its
/// [`MarkerArrayPlan`].
///
/// Reads the `markers` field by NAME: the classification that routes a frame here
/// is itself by schema name ([`crate::sink::classify_schema`]), and the vendored
/// `MarkerArray.msg` has exactly one field. Anything else is [`MarkerArrayScan::Absent`].
///
/// **Partial rendering is deliberate**, and it is the ONE place this module
/// diverges from the all-or-nothing element rule. A `nav_msgs/Path` is ONE
/// geometric object, so half a path is a false path; a `MarkerArray` is a BAG of
/// independent objects, so rendering twelve and naming the thirteenth fabricates
/// nothing — whereas dropping all thirteen because one carried `type = 99` would
/// be the worse answer.
pub fn scan_marker_array(fv: &FrameValue) -> MarkerArrayScan {
    let Some(FrameValueKind::NestedArray { elements, .. }) = fv.field(MARKERS_FIELD) else {
        return MarkerArrayScan::Absent;
    };
    if elements.is_empty() {
        return MarkerArrayScan::Empty;
    }
    let mut plan = MarkerArrayPlan::default();
    let inspected = elements.len().min(MAX_MARKER_INSTANCES);
    plan.reports.truncated_markers = elements.len() - inspected;
    let mut vertex_budget = MAX_MARKER_VERTICES;
    let mut unknown_types = BTreeSet::new();
    let mut unknown_actions = BTreeSet::new();
    let mut dropped_tail_kinds = BTreeSet::new();
    let mut colors_dropped_kinds = BTreeSet::new();
    let mut mesh_uris = BTreeSet::new();

    for element in &elements[..inspected] {
        let FrameValueKind::Nested(marker) = element else {
            plan.reports.undecodable += 1;
            continue;
        };
        let (Some(action_value), Some(id)) = (field_i32(marker, "action"), field_i32(marker, "id"))
        else {
            plan.reports.undecodable += 1;
            continue;
        };
        let key = MarkerKey {
            ns: field_str(marker, "ns").unwrap_or_default().to_string(),
            id,
        };
        let Some(action) = MarkerAction::from_wire(action_value) else {
            unknown_actions.insert(action_value);
            continue;
        };
        match action {
            MarkerAction::DeleteAll => {
                plan.ops.push(MarkerOp::DeleteAll);
                continue;
            }
            MarkerAction::Delete => {
                plan.ops.push(MarkerOp::Delete(key));
                continue;
            }
            MarkerAction::Add => {}
        }
        let Some(type_value) = field_i32(marker, "type") else {
            plan.reports.undecodable += 1;
            continue;
        };
        let Some(kind) = MarkerKind::from_wire(type_value) else {
            unknown_types.insert(type_value);
            continue;
        };
        let Some(pose) = marker
            .field("pose")
            .and_then(nested_of)
            .and_then(crate::archetype::pose_transform_parts)
        else {
            plan.reports.undecodable += 1;
            continue;
        };
        let scale = xyz_field(marker, "scale").unwrap_or([0.0; 3]);
        let points = point_list(marker, "points");
        // The budget is charged only for vertices this kind actually RENDERS: a
        // `CUBE` / `SPHERE` / `CYLINDER` / `TEXT_VIEW_FACING` / `MESH_RESOURCE`
        // ignores `points` entirely, so charging a stray bank on one of those
        // would evict LATER markers that really do draw geometry.
        let charged = if kind.consumes_points() {
            points.len()
        } else {
            0
        };
        // The whole marker is skipped when it would blow the per-frame vertex
        // budget — NEVER half of it (a half-drawn polyline is a plausible-but-
        // wrong picture).
        if charged > vertex_budget {
            plan.reports.truncated_vertices += 1;
            continue;
        }
        vertex_budget -= charged;
        // Geometry FIRST: a marker that does not resolve is not rendered at all,
        // so it must not ALSO earn a colour report — one skipped marker, one
        // reason.
        let Some(geometry) = marker_geometry(kind, scale, &points, marker, &mut dropped_tail_kinds)
        else {
            plan.reports.undecodable += 1;
            continue;
        };
        let colors = color_list(marker, "colors");
        let instance_colors = resolve_instance_colors(
            kind,
            &points,
            colors,
            &mut plan.reports,
            &mut colors_dropped_kinds,
        );
        if kind == MarkerKind::Cylinder && scale[0] != scale[1] {
            plan.reports.elliptical_cylinders += 1;
        }
        if scale_is_degenerate(kind, scale, points.len()) {
            plan.reports.degenerate_scales += 1;
        }
        if let MarkerGeometry::MeshProxy { uri, .. } = &geometry {
            mesh_uris.insert(uri.clone());
        }
        if lifetime_is_nonzero(marker) {
            plan.reports.lifetime_markers += 1;
        }
        if field_bool(marker, "frame_locked") == Some(true) {
            plan.reports.frame_locked_markers += 1;
        }
        if marker_frame_id(marker).is_some() {
            plan.reports.markers_with_frame_id += 1;
        }
        plan.ops.push(MarkerOp::Draw(Box::new(MarkerDraw {
            key,
            pose,
            geometry,
            color: color_field(marker, "color").unwrap_or([255, 255, 255, 255]),
            instance_colors,
        })));
    }

    plan.reports.unknown_types = unknown_types.into_iter().collect();
    plan.reports.unknown_actions = unknown_actions.into_iter().collect();
    plan.reports.dropped_tail_kinds = dropped_tail_kinds.into_iter().collect();
    plan.reports.colors_dropped_kinds = colors_dropped_kinds.into_iter().collect();
    plan.reports.mesh_uris = mesh_uris.into_iter().collect();
    MarkerArrayScan::Plan(Box::new(plan))
}

/// Pair a marker's `colors` bank with the PRIMITIVES its geometry will render,
/// or report why it could not be.
///
/// Three outcomes, kept apart because they mean different things to an operator:
///
/// - `None`, silent — no bank (the overwhelmingly common case), or a kind for
///   which ROS does not define `colors` at all (a single `CUBE`/`SPHERE`/… has no
///   `points`, so a non-empty bank there is already a LENGTH mismatch);
/// - `None` + [`MarkerReports::color_length_mismatches`] — a bank whose length
///   disagrees with `points`. A PRODUCER bug; zipping it would mis-colour the tail.
/// - `None` + [`MarkerReports::colors_dropped_kinds`] — a length-MATCHED bank the
///   RENDERER cannot express (see [`MarkerDraw::instance_colors`]).
fn resolve_instance_colors(
    kind: MarkerKind,
    points: &[[f32; 3]],
    colors: Vec<[u8; 4]>,
    reports: &mut MarkerReports,
    colors_dropped_kinds: &mut BTreeSet<MarkerKind>,
) -> Option<Vec<[u8; 4]>> {
    if colors.is_empty() {
        return None;
    }
    if colors.len() != points.len() {
        reports.color_length_mismatches += 1;
        return None;
    }
    match kind {
        // One colour per point / centre / vertex — a direct 1:1 pairing.
        MarkerKind::Points | MarkerKind::CubeList | MarkerKind::SphereList => Some(colors),
        // Vertices are grouped into whole triangles, so the bank is clipped to the
        // vertices that survive the grouping (never zipped past them).
        MarkerKind::TriangleList => {
            let used = points.len() / 3 * 3;
            Some(colors[..used].to_vec())
        }
        // One colour per STRIP: strip i is points[2i..2i+2], so its colour is the
        // bank entry of its FIRST point (the trailing odd point is already dropped).
        MarkerKind::LineList => Some(
            colors
                .as_chunks::<2>()
                .0
                .iter()
                .map(|pair| pair[0])
                .collect::<Vec<_>>(),
        ),
        // One colour per ARROW: arrow i runs points[i] -> points[i+1], so it takes
        // its TAIL's colour; the last point starts no arrow.
        MarkerKind::ArrowStrip => Some(colors[..points.len().saturating_sub(1)].to_vec()),
        // Rerun limits, not producer bugs — reported separately.
        MarkerKind::LineStrip | MarkerKind::Arrow => {
            colors_dropped_kinds.insert(kind);
            None
        }
        // ROS does not define `colors` for these — each renders ONE primitive
        // sized from `scale`, never from `points`. A publisher that supplies a
        // matched `points`+`colors` pair on one of them (unusual but reachable —
        // the length check above only rejects a MIS-matched bank) has asked for
        // something the kind has no instances to carry, so the flat colour is
        // used. Not reported: the marker still draws exactly as ROS specifies it.
        MarkerKind::Cube
        | MarkerKind::Sphere
        | MarkerKind::Cylinder
        | MarkerKind::TextViewFacing
        | MarkerKind::MeshResource => None,
    }
}

/// A marker's own `header.frame_id`, when it carries a non-empty one (v1 does not
/// read it — see [`MarkerReports::markers_with_frame_id`]).
fn marker_frame_id<'f>(marker: &FrameValue<'f>) -> Option<&'f str> {
    let header = nested_of(marker.field("header")?)?;
    match header.field("frame_id")? {
        FrameValueKind::Str(s) if !s.is_empty() => Some(s),
        _ => None,
    }
}

/// Build ONE marker's geometry from its kind, scale and points.
///
/// `None` only when the kind needs a field the marker does not carry at all
/// (a `TEXT_VIEW_FACING` with no `text`, a `MESH_RESOURCE` with no
/// `mesh_resource`) — an EMPTY point list is not a failure, it draws an empty
/// archetype which the render half skips.
fn marker_geometry(
    kind: MarkerKind,
    scale: [f32; 3],
    points: &[[f32; 3]],
    marker: &FrameValue,
    dropped_tail_kinds: &mut BTreeSet<MarkerKind>,
) -> Option<MarkerGeometry> {
    Some(match kind {
        // Two forms, exactly as ROS defines them: with two points the arrow spans
        // them and `scale.x` is the SHAFT DIAMETER; otherwise the arrow is
        // `scale.x` long along the marker's local +X (the pose supplies the
        // direction) and `scale.y` is the shaft diameter.
        MarkerKind::Arrow => {
            if points.len() >= 2 {
                if points.len() > 2 {
                    dropped_tail_kinds.insert(kind);
                }
                let (a, b) = (points[0], points[1]);
                MarkerGeometry::Arrows {
                    origins: vec![a],
                    vectors: vec![[b[0] - a[0], b[1] - a[1], b[2] - a[2]]],
                    radius: scale[0] / 2.0,
                }
            } else {
                if points.len() == 1 {
                    dropped_tail_kinds.insert(kind);
                }
                MarkerGeometry::Arrows {
                    origins: vec![[0.0; 3]],
                    vectors: vec![[scale[0], 0.0, 0.0]],
                    radius: scale[1] / 2.0,
                }
            }
        }
        MarkerKind::ArrowStrip => {
            if points.len() < 2 {
                dropped_tail_kinds.insert(kind);
            }
            let mut origins = Vec::new();
            let mut vectors = Vec::new();
            for pair in points.windows(2) {
                let (a, b) = (pair[0], pair[1]);
                origins.push(a);
                vectors.push([b[0] - a[0], b[1] - a[1], b[2] - a[2]]);
            }
            MarkerGeometry::Arrows {
                origins,
                vectors,
                radius: scale[0] / 2.0,
            }
        }
        MarkerKind::Cube => MarkerGeometry::Boxes {
            centers: vec![[0.0; 3]],
            size: scale,
        },
        MarkerKind::CubeList => MarkerGeometry::Boxes {
            centers: points.to_vec(),
            size: scale,
        },
        MarkerKind::Sphere => MarkerGeometry::Ellipsoids {
            centers: vec![[0.0; 3]],
            half_size: half(scale),
        },
        MarkerKind::SphereList => MarkerGeometry::Ellipsoids {
            centers: points.to_vec(),
            half_size: half(scale),
        },
        MarkerKind::Cylinder => MarkerGeometry::Cylinder {
            length: scale[2],
            radius: (scale[0] + scale[1]) / 4.0,
        },
        MarkerKind::LineStrip => {
            if points.len() == 1 {
                dropped_tail_kinds.insert(kind);
            }
            MarkerGeometry::LineStrips {
                strips: if points.len() >= 2 {
                    vec![points.to_vec()]
                } else {
                    Vec::new()
                },
                radius: scale[0] / 2.0,
            }
        }
        MarkerKind::LineList => {
            if !points.len().is_multiple_of(2) {
                dropped_tail_kinds.insert(kind);
            }
            MarkerGeometry::LineStrips {
                // A closure, not the `<[[f32; 3]]>::to_vec` fn item: `as_chunks`
                // yields `&[[f32; 3]; 2]`, and an unsizing coercion to `&[_]` does
                // not happen for a fn item passed to `map`.
                strips: points
                    .as_chunks::<2>()
                    .0
                    .iter()
                    .map(|pair| pair.to_vec())
                    .collect(),
                radius: scale[0] / 2.0,
            }
        }
        MarkerKind::Points => MarkerGeometry::Points {
            positions: points.to_vec(),
            radius: scale[0] / 2.0,
        },
        MarkerKind::TextViewFacing => MarkerGeometry::Text {
            text: field_str(marker, "text")?.to_string(),
        },
        MarkerKind::MeshResource => MarkerGeometry::MeshProxy {
            size: scale,
            uri: field_str(marker, "mesh_resource")?.to_string(),
        },
        MarkerKind::TriangleList => {
            if !points.len().is_multiple_of(3) {
                dropped_tail_kinds.insert(kind);
            }
            // ROS applies the marker `scale` to a triangle list's vertices (it is
            // "the scale of the mesh"), unlike CUBE_LIST / SPHERE_LIST where the
            // scale sizes each INSTANCE.
            MarkerGeometry::Triangles {
                vertices: points
                    .as_chunks::<3>()
                    .0
                    .iter()
                    .flatten()
                    .map(|v| [v[0] * scale[0], v[1] * scale[1], v[2] * scale[2]])
                    .collect(),
            }
        }
    })
}

/// Halve each component (ROS `scale` is a DIAMETER; rerun ellipsoids take half
/// sizes).
fn half(v: [f32; 3]) -> [f32; 3] {
    [v[0] / 2.0, v[1] / 2.0, v[2] / 2.0]
}

/// Whether this marker renders INVISIBLY because the scale components its kind
/// actually CONSUMES are zero.
///
/// **Per-kind on the consumed components, never a blanket all-zero test.** ROS
/// gives each kind its own scale contract, and most consume only ONE axis: a
/// `LINE_STRIP` reads `scale.x` alone, so `[0, w, 0]` is a zero-width invisible
/// line that an all-zero test would call healthy — the exact silent-nothing this
/// report exists to explain. The volumetric kinds are the opposite: a `CUBE` of
/// `[1, 1, 0]` is a flat quad, which is VISIBLE and legitimate, so only all-zero
/// counts there.
///
/// `TEXT_VIEW_FACING` is never degenerate: its only scale component is `scale.z`
/// (text height), which this build drops anyway, so a zero there costs nothing.
///
/// `points_len` selects `ARROW`'s form: with two points the arrow spans them and
/// only the shaft diameter (`scale.x`) can vanish; without, `scale.x` is the
/// arrow's LENGTH and `scale.y` its diameter, so either one zeroes it.
fn scale_is_degenerate(kind: MarkerKind, scale: [f32; 3], points_len: usize) -> bool {
    match kind {
        MarkerKind::TextViewFacing => false,
        // Two-point form: scale.x is the shaft DIAMETER. Pose form: scale.x is the
        // LENGTH and scale.y the diameter — either zero makes it invisible.
        MarkerKind::Arrow => {
            if points_len >= 2 {
                scale[0] == 0.0
            } else {
                scale[0] == 0.0 || scale[1] == 0.0
            }
        }
        // Width-only kinds: the geometry comes from `points`, `scale.x` is the
        // line width / dot diameter.
        MarkerKind::ArrowStrip
        | MarkerKind::LineStrip
        | MarkerKind::LineList
        | MarkerKind::Points => scale[0] == 0.0,
        // A cylinder vanishes if it has no radius OR no height.
        MarkerKind::Cylinder => (scale[0] == 0.0 && scale[1] == 0.0) || scale[2] == 0.0,
        // Volumetric kinds: a flattened one is still visible, an all-zero one is not.
        MarkerKind::Cube
        | MarkerKind::CubeList
        | MarkerKind::Sphere
        | MarkerKind::SphereList
        | MarkerKind::MeshResource
        | MarkerKind::TriangleList => scale == [0.0; 3],
    }
}

/// True when `lifetime` is present and non-zero (v1 does not expire markers).
fn lifetime_is_nonzero(marker: &FrameValue) -> bool {
    let Some(lifetime) = marker.field("lifetime").and_then(nested_of) else {
        return false;
    };
    let sec = field_i32(lifetime, "sec").unwrap_or(0);
    let nanosec = match lifetime.field("nanosec") {
        Some(FrameValueKind::U32(v)) => *v,
        _ => 0,
    };
    sec != 0 || nanosec != 0
}

// ---- FrameValue readers (kept local so this module reads only the public
// walker API; the crate's own extractors are shaped for the spatial ladder) ----

fn nested_of<'a, 'f>(k: &'a FrameValueKind<'f>) -> Option<&'a FrameValue<'f>> {
    match k {
        FrameValueKind::Nested(inner) => Some(inner.as_ref()),
        _ => None,
    }
}

fn field_i32(fv: &FrameValue, name: &str) -> Option<i32> {
    match fv.field(name)? {
        FrameValueKind::I32(v) => Some(*v),
        _ => None,
    }
}

fn field_bool(fv: &FrameValue, name: &str) -> Option<bool> {
    match fv.field(name)? {
        FrameValueKind::Bool(v) => Some(*v),
        _ => None,
    }
}

fn field_str<'f>(fv: &FrameValue<'f>, name: &str) -> Option<&'f str> {
    match fv.field(name)? {
        FrameValueKind::Str(s) => Some(s),
        _ => None,
    }
}

/// Any numeric scalar widened to `f32` (Rerun is f32 throughout).
fn num_f32(k: &FrameValueKind) -> Option<f32> {
    Some(match k {
        FrameValueKind::F32(v) => *v,
        FrameValueKind::F64(v) => *v as f32,
        FrameValueKind::I32(v) => *v as f32,
        FrameValueKind::U32(v) => *v as f32,
        _ => return None,
    })
}

/// A nested `{x, y, z}` field as `[f32; 3]`.
fn xyz_field(fv: &FrameValue, name: &str) -> Option<[f32; 3]> {
    let inner = nested_of(fv.field(name)?)?;
    Some([
        num_f32(inner.field("x")?)?,
        num_f32(inner.field("y")?)?,
        num_f32(inner.field("z")?)?,
    ])
}

/// A `geometry_msgs/Point[]` field as marker-local vertices. An element that
/// does not decode as `{x, y, z}` is DROPPED rather than fabricated — the walker
/// already refused anything structurally wrong, so this is a defensive floor.
fn point_list(fv: &FrameValue, name: &str) -> Vec<[f32; 3]> {
    let Some(FrameValueKind::NestedArray { elements, .. }) = fv.field(name) else {
        return Vec::new();
    };
    elements
        .iter()
        .filter_map(|e| {
            let p = nested_of(e)?;
            Some([
                num_f32(p.field("x")?)?,
                num_f32(p.field("y")?)?,
                num_f32(p.field("z")?)?,
            ])
        })
        .collect()
}

/// A `std_msgs/ColorRGBA[]` field as `[r, g, b, a]` bytes.
fn color_list(fv: &FrameValue, name: &str) -> Vec<[u8; 4]> {
    let Some(FrameValueKind::NestedArray { elements, .. }) = fv.field(name) else {
        return Vec::new();
    };
    elements
        .iter()
        .filter_map(|e| rgba_of(nested_of(e)?))
        .collect()
}

/// A nested `std_msgs/ColorRGBA` field as `[r, g, b, a]` bytes.
fn color_field(fv: &FrameValue, name: &str) -> Option<[u8; 4]> {
    rgba_of(nested_of(fv.field(name)?)?)
}

/// `ColorRGBA` (f32 in 0..=1) → sRGB bytes. Clamped, because a producer sending
/// 0..=255 floats (a real and common mistake) must saturate rather than wrap.
fn rgba_of(fv: &FrameValue) -> Option<[u8; 4]> {
    let component = |name: &str| -> Option<u8> {
        let v = num_f32(fv.field(name)?)?;
        Some((v.clamp(0.0, 1.0) * 255.0).round() as u8)
    };
    Some([
        component("r")?,
        component("g")?,
        component("b")?,
        component("a")?,
    ])
}

// ────────────────────────────────────────────────────────────────────────────
// PURE: ops + live set → clears + draws
// ────────────────────────────────────────────────────────────────────────────

/// One input's per-viewer marker state: what it believes is LIVE, and what it has
/// already CLEARED and not since re-drawn.
///
/// **Why the second set exists.** Clearing a key removes it from `live`, which
/// destroys the very state that would recognise a REPEAT. Without `cleared`, the
/// common ROS idiom of re-sending a full set with its retirements — the same
/// `DELETE` for a retired `(ns, id)` every frame — emits one `rerun::Clear` chunk
/// per frame FOREVER for an otherwise idle publisher. Per occurrence that cost is
/// one chunk; per RUN it is unbounded, which is not the same thing.
///
/// The distinction the pair encodes is "not tracked because never seen this
/// session" (clear it — the rule that keeps a DELETE working past
/// the tracking ceiling) versus "not tracked because it is ALREADY cleared"
/// (nothing to clear).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct MarkerLiveState {
    /// Entity keys currently drawn and not cleared.
    live: BTreeSet<String>,
    /// Entity keys cleared and not since re-drawn — the repeat-suppression set.
    cleared: BTreeSet<String>,
}

impl MarkerLiveState {
    /// A state that believes `keys` are live and has cleared nothing — the
    /// frame-start condition (and the shape a fresh input starts from).
    pub fn with_live(keys: impl IntoIterator<Item = String>) -> Self {
        Self {
            live: keys.into_iter().collect(),
            cleared: BTreeSet::new(),
        }
    }

    /// How many markers this input believes are LIVE (observability / test seam).
    pub fn live_count(&self) -> usize {
        self.live.len()
    }

    /// The LIVE keys, sorted (observability / test seam).
    pub fn live_keys(&self) -> Vec<String> {
        self.live.iter().cloned().collect()
    }

    /// The already-CLEARED keys whose repeat is suppressed, sorted
    /// (observability / test seam).
    pub fn cleared_keys(&self) -> Vec<String> {
        self.cleared.iter().cloned().collect()
    }

    /// Whether `key` is currently believed live.
    pub fn is_live(&self, key: &str) -> bool {
        self.live.contains(key)
    }

    /// Whether a CLEAR must actually be emitted for `key` this frame.
    ///
    /// The rule is exactly "it is not already retired": a LIVE key needs
    /// clearing, and so does an UNTRACKED-and-never-retired one (the first
    /// `DELETE` of a marker past the tracking ceiling — see
    /// [`resolve_marker_ops`]). Only the REPEAT is suppressed.
    ///
    /// A `live.contains(key) ||` term would be redundant, not defensive: the two
    /// sets are DISJOINT by construction ([`MarkerLiveState::note_drawn`] removes
    /// from `cleared`, [`MarkerLiveState::note_cleared`] removes from `live`, and
    /// [`MarkerLiveState::with_live`] starts with `cleared` empty), so
    /// `live.contains` already implies `!cleared.contains`. The invariant is
    /// pinned by `the_live_and_cleared_sets_are_always_disjoint`, which is what
    /// makes this one-term form sound.
    fn needs_clear(&self, key: &str) -> bool {
        !self.cleared.contains(key)
    }

    /// Record that `key` was drawn. Returns `false` when the ceiling refused it.
    fn note_drawn(&mut self, key: &str) -> bool {
        self.cleared.remove(key);
        if self.live.contains(key) {
            return true;
        }
        if self.live.len() >= MAX_LIVE_MARKERS {
            return false;
        }
        self.live.insert(key.to_string());
        true
    }

    /// Record that `key` was cleared. Returns `false` when the ceiling refused to
    /// remember it (so a later repeat re-clears — bounded memory, one wasted chunk
    /// per frame, never a wrong picture).
    fn note_cleared(&mut self, key: &str) -> bool {
        self.live.remove(key);
        if self.cleared.contains(key) {
            return true;
        }
        if self.cleared.len() >= MAX_LIVE_MARKERS {
            return false;
        }
        self.cleared.insert(key.to_string());
        true
    }
}

/// What ONE `MarkerArray` frame does to the viewer: the entities to CLEAR and the
/// markers to DRAW, in the order they must be emitted.
#[derive(Debug, Clone, PartialEq)]
pub struct MarkerFrameActions<'a> {
    /// Entity KEYS to clear, SORTED (deterministic emission order).
    pub clears: Vec<String>,
    /// Markers to draw, in array declaration order, at most one per key
    /// (a re-`ADD` of the same key within one frame keeps its FIRST position and
    /// its LAST payload, which is what in-order `MODIFY` semantics mean).
    pub draws: Vec<&'a MarkerDraw>,
    /// DRAWN keys the LIVE half of [`MAX_LIVE_MARKERS`] refused to track. They
    /// still draw; what they lose is being swept by a future `DELETEALL`.
    ///
    /// Kept SEPARATE from [`MarkerFrameActions::clear_overflow`] because the two
    /// have different consequences AND different remediations, and a single
    /// aggregate made the warn factually wrong: the cleared half can fill with
    /// `live_count() == 0`, at which point a message saying "more than N markers
    /// live at once" describes a state that is not occurring and recommends a
    /// `DELETEALL` that would change nothing.
    pub draw_overflow: usize,
    /// CLEARED keys whose retirement the `cleared` half of [`MAX_LIVE_MARKERS`]
    /// refused to remember, so an identical repeated `DELETE` re-emits its
    /// `Clear` every frame (wasted chunks — never a wrong picture).
    pub clear_overflow: usize,
}

/// What ONE key's ops resolve to by the END of a frame — the LAST op wins.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum KeyIntent {
    /// The key's last op was an `ADD`: draw it, never clear it.
    Draw,
    /// The key's last op was a `DELETE` / `DELETEALL`, or an `ADD` whose geometry
    /// renders nothing: clear it, never draw it.
    Clear,
}

/// Resolve one frame's ops against this input's LIVE marker set, advancing that
/// set in place.
///
/// # The algorithm: LAST OP WINS, per key
///
/// Ops apply IN ORDER — ROS's own semantics, where a `DELETEALL` at `markers[0]`
/// (the dominant real idiom) is followed by the fresh set, and an `ADD` before a
/// `DELETEALL` is erased by it. Each key therefore carries exactly ONE
/// INTENT (draw or clear), overwritten by each op that touches it, and a `DELETEALL`
/// rewrites every intent so far to `Clear`.
///
/// Resolving per key — rather than moving keys between a draw set and a clear set
/// as ops stream past — is what makes the three-op sequences come out right. A
/// streaming form drops the clear on `DELETEALL, ADD k, DELETE k`: the ADD
/// cancels the pending clear, the DELETE cancels the ADD, and the DELETE's own
/// clear is then suppressed because `DELETEALL` has ALREADY emptied `live`, so
/// "was it live?" answers no. `live` is read as a frame-START snapshot and
/// mutated only at the end, so that question always has the correct answer.
///
/// # The invariant it buys
///
/// **No entity is both cleared and drawn at the same timestamp** — one intent per
/// key makes it structural. rerun's `Clear` docs define before/after in TIME and
/// say nothing about a tie, so rather than betting on an unspecified resolution
/// the case is designed out. SCOPE: this holds WITHIN one frame. Two
/// frames that share a wire timestamp (a publisher emitting `DELETEALL` and its
/// fresh set as two messages stamped alike) can still clear and draw one entity at
/// the same instant; the sink does not batch across frames.
///
/// # What is cleared
///
/// An explicit `DELETE` clears its entity **whether or not the key was tracked**.
/// Tracking is bounded ([`MAX_LIVE_MARKERS`]), so a "was it live?" gate makes
/// every DELETE past the ceiling a silent no-op and the marker a permanent ghost.
/// A `Clear` on a leaf entity is idempotent and costs one chunk; a ghost is a wrong
/// picture, and the wrong picture is worse. (The cost is real and bounded: a
/// `DELETE` for a key that never existed creates an empty entity row.) A
/// `DELETEALL` can only clear what is TRACKED — it has nothing else to enumerate —
/// which is exactly the residual the overflow warn names.
///
/// Clears are emitted at LEAF marker entities only, never on an ancestor the same
/// frame repopulates, so a recursive clear can never orphan a live sibling.
pub fn resolve_marker_ops<'a>(
    ops: &'a [MarkerOp],
    state: &mut MarkerLiveState,
) -> MarkerFrameActions<'a> {
    // Per-key final intent + the payload/slot for the ones that end up drawn.
    // `order` keeps first-appearance order so a re-ADD keeps its position and
    // takes its LAST payload (in-order `MODIFY` semantics).
    let mut intent: BTreeMap<String, KeyIntent> = BTreeMap::new();
    let mut payload: BTreeMap<String, &'a MarkerDraw> = BTreeMap::new();
    let mut order: Vec<String> = Vec::new();
    let mut deleted_all = false;

    /// Record a key's FIRST appearance so `order` is declaration order.
    fn touch(key: String, intent: &BTreeMap<String, KeyIntent>, order: &mut Vec<String>) -> String {
        if !intent.contains_key(&key) {
            order.push(key.clone());
        }
        key
    }

    for op in ops {
        match op {
            MarkerOp::Draw(draw) => {
                let key = touch(draw.key.entity_key(), &intent, &mut order);
                // An ADD whose geometry renders NOTHING is semantically "nothing
                // here" — and under latest-at, logging only its `Transform3D` and
                // returning would leave the PREVIOUS frame's geometry standing as
                // current. The "shrink to zero points = nothing detected this
                // cycle" idiom would then show stale data indefinitely, so an
                // empty ADD resolves to a CLEAR.
                let kind = if draw.geometry.renders_nothing() {
                    KeyIntent::Clear
                } else {
                    payload.insert(key.clone(), draw.as_ref());
                    KeyIntent::Draw
                };
                intent.insert(key, kind);
            }
            MarkerOp::Delete(key) => {
                let key = touch(key.entity_key(), &intent, &mut order);
                intent.insert(key, KeyIntent::Clear);
            }
            MarkerOp::DeleteAll => {
                // In-order: every ADD so far is erased. Keys already intended
                // Clear stay Clear.
                for value in intent.values_mut() {
                    *value = KeyIntent::Clear;
                }
                deleted_all = true;
            }
        }
    }

    let mut clears: BTreeSet<String> = BTreeSet::new();
    let mut draws: Vec<&'a MarkerDraw> = Vec::new();
    for key in &order {
        match intent.get(key) {
            // A key whose last op retires it is cleared UNLESS it is already
            // known-cleared and not since re-drawn — that repeat is what would
            // otherwise emit one chunk per frame forever for the common ROS
            // "resend the full set with its retirements" idiom.
            Some(KeyIntent::Clear) => {
                if state.needs_clear(key) {
                    clears.insert(key.clone());
                }
            }
            Some(KeyIntent::Draw) => {
                if let Some(draw) = payload.get(key) {
                    draws.push(*draw);
                }
            }
            None => {}
        }
    }
    // A DELETEALL additionally sweeps every TRACKED key the frame did not
    // re-ADD (a tracked key is by definition live, so `needs_clear` holds).
    // Untracked ones are unreachable here — the residual the overflow warn names.
    if deleted_all {
        for key in std::mem::take(&mut state.live) {
            if intent.get(&key) != Some(&KeyIntent::Draw) {
                clears.insert(key);
            }
        }
    }

    // CLEARS are applied BEFORE draws so a slot a DELETE frees this frame is
    // available to that SAME frame's ADDs — the full-set-with-retirements idiom
    // would otherwise report overflow while net-shrinking the live set.
    let mut clear_overflow = 0;
    for key in &clears {
        if !state.note_cleared(key) {
            clear_overflow += 1;
        }
    }
    let mut draw_overflow = 0;
    for draw in &draws {
        if !state.note_drawn(&draw.key.entity_key()) {
            draw_overflow += 1;
        }
    }
    MarkerFrameActions {
        clears: clears.into_iter().collect(),
        draws,
        draw_overflow,
        clear_overflow,
    }
}

// ────────────────────────────────────────────────────────────────────────────
// The rerun half (best-effort: a log error warns, never propagates)
// ────────────────────────────────────────────────────────────────────────────

/// CLEAR one marker entity — a [`rerun::Clear`] logged TEMPORAL at the frame's
/// wire stamp.
///
/// A `Clear` is a timestamped log EVENT, not a store deletion: a latest-at query
/// after it returns nothing, one before it still returns the marker (which is
/// correct — scrubbing backwards past a delete should re-reveal it). `log_static`
/// would shadow every timeline and is wrong here.
///
/// `recursive()` rather than `flat()`: a marker entity is a leaf today, so the two
/// are equivalent, but recursive stays correct if a kind later gains a child
/// entity — it can never orphan one. It is only ever called on a LEAF marker
/// entity, never on an ancestor the same frame repopulates (see
/// [`resolve_marker_ops`]).
///
/// **Known rerun property, not fixable here**: a query using a VISIBLE TIME RANGE
/// ignores clears entirely, so a viewer configured that way keeps showing deleted
/// markers.
pub fn log_marker_clear(rec: &RecordingStream, entity: &str, timestamp_ns: u64) {
    set_robot_time(rec, timestamp_ns);
    if let Err(e) = rec.log(entity.to_string(), &rerun::Clear::recursive()) {
        tracing::warn!(error = %e, entity, "Rerun: marker Clear log failed");
    }
}

/// DRAW one marker at its own entity: the pose as a [`rerun::Transform3D`], then
/// the kind's archetype in MARKER-LOCAL coordinates at the SAME entity.
///
/// A `Transform3D` logged at an entity poses that entity's OWN geometry, not just
/// its children — which is why nothing here does coordinate math (there is no
/// `glam` feature on this rerun build) and why `points` are used verbatim: they
/// are marker-local by ROS definition, exactly as rviz treats them.
///
/// `parent_frame` names the frame the marker's pose is expressed in (a
/// TRANSFORM payload is posed by its own `parent_frame`, never by a
/// `CoordinateFrame`, and the component must be written on EVERY row because
/// rerun 0.34 resolves a `Transform3D` atomically per row).
pub fn log_marker_draw(
    rec: &RecordingStream,
    entity: &str,
    timestamp_ns: u64,
    draw: &MarkerDraw,
    parent_frame: Option<&str>,
) {
    crate::archetype::log_transform3d_in_frame(rec, entity, timestamp_ns, &draw.pose, parent_frame);
    set_robot_time(rec, timestamp_ns);
    let color = rerun::Color::from_unmultiplied_rgba(
        draw.color[0],
        draw.color[1],
        draw.color[2],
        draw.color[3],
    );
    // ONE colour per rendered primitive: the extractor's per-instance bank when it
    // resolved one (the pairing is its job — see `resolve_instance_colors`), else
    // the flat marker colour repeated. `n` is the primitive count of THIS arm.
    let instance_colors = |n: usize| -> Vec<rerun::Color> {
        match &draw.instance_colors {
            Some(cs) => cs
                .iter()
                .map(|c| rerun::Color::from_unmultiplied_rgba(c[0], c[1], c[2], c[3]))
                .collect(),
            None => vec![color; n],
        }
    };
    // Every arm's emptiness guard is DEFENSIVE, not reachable from the sink:
    // `resolve_marker_ops` turns a `renders_nothing()` geometry into a CLEAR, so a
    // draw that arrives here always has at least one primitive.
    match &draw.geometry {
        MarkerGeometry::Arrows {
            origins,
            vectors,
            radius,
        } => {
            if vectors.is_empty() {
                return;
            }
            log_archetype(
                rec,
                entity,
                &rerun::Arrows3D::from_vectors(vectors.iter().copied())
                    .with_origins(origins.iter().copied())
                    .with_radii([*radius])
                    .with_colors(instance_colors(vectors.len())),
            )
        }
        MarkerGeometry::Boxes { centers, size } => {
            if centers.is_empty() {
                return;
            }
            log_archetype(
                rec,
                entity,
                &rerun::Boxes3D::from_centers_and_sizes(
                    centers.iter().copied(),
                    std::iter::repeat_n(*size, centers.len()),
                )
                .with_colors(instance_colors(centers.len())),
            )
        }
        MarkerGeometry::Ellipsoids { centers, half_size } => {
            if centers.is_empty() {
                return;
            }
            log_archetype(
                rec,
                entity,
                &rerun::Ellipsoids3D::from_centers_and_half_sizes(
                    centers.iter().copied(),
                    std::iter::repeat_n(*half_size, centers.len()),
                )
                .with_colors(instance_colors(centers.len())),
            )
        }
        MarkerGeometry::Cylinder { length, radius } => log_archetype(
            rec,
            entity,
            &rerun::Cylinders3D::from_lengths_and_radii([*length], [*radius])
                .with_centers([[0.0f32, 0.0, 0.0]])
                .with_colors([color]),
        ),
        MarkerGeometry::LineStrips { strips, radius } => {
            if strips.is_empty() {
                return;
            }
            log_archetype(
                rec,
                entity,
                &rerun::LineStrips3D::new(strips.iter().cloned())
                    .with_radii([*radius])
                    .with_colors(instance_colors(strips.len())),
            )
        }
        MarkerGeometry::Points { positions, radius } => {
            if positions.is_empty() {
                return;
            }
            log_archetype(
                rec,
                entity,
                &rerun::Points3D::new(positions.iter().copied())
                    .with_radii([*radius])
                    .with_colors(instance_colors(positions.len())),
            )
        }
        // The only in-scene text rerun 0.34 offers: a zero-radius labelled point
        // (see `MarkerGeometry::Text`).
        MarkerGeometry::Text { text } => log_archetype(
            rec,
            entity,
            &rerun::Points3D::new([[0.0f32, 0.0, 0.0]])
                .with_radii([0.0])
                .with_colors([color])
                .with_labels([text.clone()])
                .with_show_labels(true),
        ),
        MarkerGeometry::MeshProxy { size, uri } => log_archetype(
            rec,
            entity,
            &rerun::Boxes3D::from_centers_and_sizes([[0.0f32, 0.0, 0.0]], [*size])
                .with_colors([color])
                .with_labels([uri.clone()])
                .with_show_labels(true),
        ),
        MarkerGeometry::Triangles { vertices } => {
            if vertices.is_empty() {
                return;
            }
            // NB: never call `Mesh3D::sanity_check()` — its no-indices branch
            // tests `num_vertices % 9 == 0` (an upstream typo for 3) and would
            // reject valid meshes.
            log_archetype(
                rec,
                entity,
                &rerun::Mesh3D::new(vertices.iter().copied())
                    .with_vertex_colors(instance_colors(vertices.len())),
            )
        }
    }
}

/// Log one archetype, warning (never propagating) on failure — the house
/// best-effort contract for every rerun call in this crate.
fn log_archetype(rec: &RecordingStream, entity: &str, archetype: &impl rerun::AsComponents) {
    if let Err(e) = rec.log(entity.to_string(), archetype) {
        tracing::warn!(error = %e, entity, "Rerun: marker archetype log failed");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every `int32 NAME=VALUE` constant the VENDORED `visualization_msgs/Marker`
    /// declares, read back out of the embedded corpus.
    fn vendored_marker_constants() -> BTreeMap<String, i32> {
        let text = native_ros2_messages::BUILTIN_MSGS
            .iter()
            .find(|(pkg, name, _)| *pkg == "visualization_msgs" && *name == "Marker")
            .map(|(_, _, text)| *text)
            .expect("visualization_msgs/Marker is vendored");
        let mut out = BTreeMap::new();
        for line in text.lines() {
            let line = line.split('#').next().unwrap_or("").trim();
            let Some(rest) = line.strip_prefix("int32 ") else {
                continue;
            };
            let Some((name, value)) = rest.split_once('=') else {
                continue;
            };
            out.insert(
                name.trim().to_string(),
                value.trim().parse().expect("constant value"),
            );
        }
        out
    }

    /// THE drift oracle. `parse_rosmsg` drops constant lines, so the kind/action
    /// values live here as hand-written Rust — and a re-vendor of `Marker.msg`
    /// (one did exactly that mid-issue, adding `ARROW_STRIP`) would otherwise
    /// silently desynchronize them. Two halves: a HAND table of the pairs we
    /// intend, and a cross-check against the vendored `.msg` TEXT itself, so
    /// neither a code edit nor a corpus edit can drift alone.
    #[test]
    fn marker_kind_and_action_constants_match_the_vendored_msg() {
        let hand: [(&str, i32); 13] = [
            ("ARROW", 0),
            ("CUBE", 1),
            ("SPHERE", 2),
            ("CYLINDER", 3),
            ("LINE_STRIP", 4),
            ("LINE_LIST", 5),
            ("CUBE_LIST", 6),
            ("SPHERE_LIST", 7),
            ("POINTS", 8),
            ("TEXT_VIEW_FACING", 9),
            ("MESH_RESOURCE", 10),
            ("TRIANGLE_LIST", 11),
            ("ARROW_STRIP", 12),
        ];
        let actual: Vec<(&str, i32)> = MarkerKind::ALL
            .into_iter()
            .map(|k| (k.wire_name(), k.wire_value()))
            .collect();
        assert_eq!(
            actual,
            hand.to_vec(),
            "MarkerKind drifted from the hand table"
        );

        let vendored = vendored_marker_constants();
        for (name, value) in hand {
            assert_eq!(
                vendored.get(name).copied(),
                Some(value),
                "vendored Marker.msg disagrees about {name}"
            );
        }
        // TOTALITY the other way: every TYPE constant the corpus declares is a
        // kind we render. The action constants are named explicitly so this stays
        // an assertion about the type set.
        let actions = ["ADD", "MODIFY", "DELETE", "DELETEALL"];
        for name in vendored.keys() {
            assert!(
                actions.contains(&name.as_str()) || hand.iter().any(|(n, _)| n == name),
                "vendored Marker.msg declares {name}, which MarkerKind does not cover"
            );
        }
        assert_eq!(vendored.get("ADD").copied(), Some(0));
        assert_eq!(vendored.get("MODIFY").copied(), Some(0));
        assert_eq!(vendored.get("DELETE").copied(), Some(2));
        assert_eq!(vendored.get("DELETEALL").copied(), Some(3));
        assert_eq!(MarkerAction::from_wire(0), Some(MarkerAction::Add));
        assert_eq!(MarkerAction::from_wire(2), Some(MarkerAction::Delete));
        assert_eq!(MarkerAction::from_wire(3), Some(MarkerAction::DeleteAll));
        // `1` is genuinely unassigned in ROS — it must NOT resolve.
        assert_eq!(MarkerAction::from_wire(1), None);
        assert_eq!(MarkerKind::from_wire(13), None);
        assert_eq!(MarkerKind::from_wire(-1), None);
    }

    /// Entity keys are stable, total and COLLISION-FREE across the pairs that a
    /// naive sanitizer would merge — hand oracles on both sides.
    #[test]
    fn entity_keys_are_stable_and_collision_free() {
        let key = |ns: &str, id: i32| {
            MarkerKey {
                ns: ns.to_string(),
                id,
            }
            .entity_key()
        };
        // Unchanged namespaces are verbatim; the id half is hand-written.
        assert_eq!(key("costmap", 7), "costmap/id_7");
        assert_eq!(key("costmap", 0), "costmap/id_0");
        assert_eq!(key("costmap", -3), "costmap/id_n3");
        // A negative id can never alias a positive one.
        assert_ne!(key("costmap", -3), key("costmap", 3));
        // Separator variants stay distinct (the tf sanitizer's FNV suffix).
        assert_ne!(key("nav/local", 3), key("nav.local", 3));
        // The EMPTY namespace is distinct from every real one, including the
        // sanitizer's own fallback word.
        assert_ne!(key("", 0), key("unknown", 0));
        assert_eq!(key("unknown", 0), "unknown/id_0");
        // i32::MIN must not panic on negation.
        assert_eq!(key("ns", i32::MIN), "ns/id_n2147483648");
        // The full entity path carries the RESERVED child level.
        assert_eq!(
            marker_entity("world/markers", &key("costmap", 7)),
            "world/markers/viz-markers/costmap/id_7"
        );
    }

    /// The reserved child is outside the sanitizer's alphabet, so NEITHER a topic
    /// name NOR a marker namespace can ever produce it (the structural
    /// argument, applied to the level this module adds).
    #[test]
    fn the_marker_child_segment_is_unreachable_from_any_name() {
        assert_eq!(MARKER_CHILD, "viz-markers");
        assert!(
            MARKER_CHILD
                .chars()
                .any(|c| !(c.is_ascii_alphanumeric() || c == '_')),
            "MARKER_CHILD must contain a character sanitize_segment cannot emit"
        );
        for hostile in ["viz-markers", "viz_markers", "markers", "", "a/b"] {
            assert_ne!(sanitize_segment(hostile), MARKER_CHILD);
        }
    }

    /// A frozen `ADD` op for the resolution tests — geometry is irrelevant there,
    /// so it is the cheapest kind.
    fn draw(ns: &str, id: i32) -> MarkerOp {
        MarkerOp::Draw(Box::new(MarkerDraw {
            key: MarkerKey {
                ns: ns.to_string(),
                id,
            },
            pose: PoseParts {
                translation: [0.0; 3],
                rotation: None,
            },
            geometry: MarkerGeometry::Boxes {
                centers: vec![[0.0; 3]],
                size: [1.0; 3],
            },
            color: [255, 255, 255, 255],
            instance_colors: None,
        }))
    }

    /// A frame-START live state believing `keys` are live.
    fn live_set(keys: &[&str]) -> MarkerLiveState {
        MarkerLiveState::with_live(keys.iter().map(|k| (*k).to_string()))
    }

    /// A sorted key vector, for comparing against `live_keys()`.
    fn keys(k: &[&str]) -> Vec<String> {
        let mut v: Vec<String> = k.iter().map(|s| (*s).to_string()).collect();
        v.sort();
        v
    }

    fn drawn_keys(actions: &MarkerFrameActions) -> Vec<String> {
        actions.draws.iter().map(|d| d.key.entity_key()).collect()
    }

    #[test]
    fn a_delete_clears_exactly_its_own_entity_and_leaves_the_sibling() {
        let ops = vec![MarkerOp::Delete(MarkerKey {
            ns: "a".into(),
            id: 1,
        })];
        let mut live = live_set(&["a/id_1", "a/id_2"]);
        let actions = resolve_marker_ops(&ops, &mut live);
        assert_eq!(actions.clears, vec!["a/id_1".to_string()]);
        assert!(actions.draws.is_empty());
        assert_eq!(live.live_keys(), keys(&["a/id_2"]));
    }

    /// An explicit `DELETE` clears its entity even when the key is UNTRACKED.
    ///
    /// This is a deliberate reversal of the "no phantom clears" instinct, and the
    /// trade is stated in [`resolve_marker_ops`]: tracking is bounded, so a
    /// "was it live?" gate makes every DELETE past [`MAX_LIVE_MARKERS`] a silent
    /// no-op and its marker a permanent ghost. A `Clear` on a leaf entity is
    /// idempotent and costs one chunk (and, for a key that never existed, an
    /// empty entity row); a ghost is a WRONG PICTURE, which is worse.
    #[test]
    fn a_delete_of_an_untracked_marker_still_clears_it() {
        let ops = vec![MarkerOp::Delete(MarkerKey {
            ns: "ghost".into(),
            id: 9,
        })];
        let mut live = live_set(&["a/id_1"]);
        let actions = resolve_marker_ops(&ops, &mut live);
        assert_eq!(actions.clears, vec!["ghost/id_9".to_string()]);
        assert!(actions.draws.is_empty());
        assert_eq!(
            live.live_keys(),
            keys(&["a/id_1"]),
            "the sibling is untouched"
        );
    }

    /// **THE REPEAT-SUPPRESSION PIN.** Re-sending the full set WITH its
    /// retirements — the same `DELETE` for a retired `(ns, id)` every frame — is
    /// a common ROS idiom, and it must emit exactly ONE `Clear`, not one per
    /// frame forever.
    ///
    /// This is what the `cleared` half of [`MarkerLiveState`] exists for:
    /// clearing a key removes it from `live`, destroying the very state that
    /// would recognise the repeat.
    #[test]
    fn a_repeated_delete_of_a_retired_marker_clears_exactly_once() {
        let ops = vec![MarkerOp::Delete(MarkerKey {
            ns: "a".into(),
            id: 1,
        })];
        let mut live = live_set(&["a/id_1"]);
        // Frame 1: it was live, so it clears.
        let first = resolve_marker_ops(&ops, &mut live);
        assert_eq!(first.clears, vec!["a/id_1".to_string()]);
        // Frames 2..N: identical bytes, and NOTHING more is emitted.
        for frame in 2..=5 {
            let repeat = resolve_marker_ops(&ops, &mut live);
            assert!(
                repeat.clears.is_empty(),
                "frame {frame} re-cleared an already-retired marker"
            );
            assert!(repeat.draws.is_empty());
        }
        assert_eq!(live.cleared_keys(), vec!["a/id_1".to_string()]);
        // ...and a genuine RE-ADD re-arms it: the next DELETE clears again.
        let re_add = [draw("a", 1)];
        assert_eq!(
            drawn_keys(&resolve_marker_ops(&re_add, &mut live)),
            vec!["a/id_1"]
        );
        assert!(live.cleared_keys().is_empty(), "the re-add un-retires it");
        assert_eq!(
            resolve_marker_ops(&ops, &mut live).clears,
            vec!["a/id_1".to_string()]
        );
    }

    /// The same suppression for a repeated EMPTY ADD — the other shape that
    /// resolves to a clear (a detector re-publishing "nothing this cycle").
    #[test]
    fn a_repeated_empty_add_clears_exactly_once() {
        let empty = [MarkerOp::Draw(Box::new(MarkerDraw {
            key: MarkerKey {
                ns: "det".into(),
                id: 1,
            },
            pose: PoseParts {
                translation: [0.0; 3],
                rotation: None,
            },
            geometry: MarkerGeometry::Points {
                positions: Vec::new(),
                radius: 0.1,
            },
            color: [255, 255, 255, 255],
            instance_colors: None,
        }))];
        let mut live = live_set(&["det/id_1"]);
        assert_eq!(
            resolve_marker_ops(&empty, &mut live).clears,
            vec!["det/id_1".to_string()]
        );
        for frame in 2..=5 {
            assert!(
                resolve_marker_ops(&empty, &mut live).clears.is_empty(),
                "frame {frame} re-cleared an already-empty marker"
            );
        }
    }

    /// A DELETE frees a tracked slot for the SAME frame's ADDs — the
    /// full-set-with-retirements idiom at the ceiling must not report overflow
    /// while the live set is net-shrinking.
    #[test]
    fn a_delete_frees_its_slot_for_the_same_frames_adds() {
        let mut live = live_set(
            &(0..MAX_LIVE_MARKERS)
                .map(|i| format!("a/id_{i}"))
                .collect::<Vec<_>>()
                .iter()
                .map(String::as_str)
                .collect::<Vec<_>>(),
        );
        assert_eq!(
            live.live_count(),
            MAX_LIVE_MARKERS,
            "premise: at the ceiling"
        );
        // One retirement + one newcomer in ONE frame.
        let ops = vec![
            MarkerOp::Delete(MarkerKey {
                ns: "a".into(),
                id: 0,
            }),
            draw("a", MAX_LIVE_MARKERS as i32),
        ];
        let actions = resolve_marker_ops(&ops, &mut live);
        assert_eq!(
            (actions.draw_overflow, actions.clear_overflow),
            (0, 0),
            "the freed slot was reusable"
        );
        assert!(live.is_live(&format!("a/id_{MAX_LIVE_MARKERS}")));
    }

    /// A `DELETEALL`, by contrast, can only sweep what is TRACKED — it has
    /// nothing else to enumerate. That asymmetry is the residual the overflow
    /// warn names, and it is asserted here so a future "make DELETEALL clear
    /// everything" change has to face it.
    #[test]
    fn a_deleteall_sweeps_only_tracked_keys() {
        let ops = vec![MarkerOp::DeleteAll];
        let mut live = live_set(&["a/id_1"]);
        let actions = resolve_marker_ops(&ops, &mut live);
        assert_eq!(actions.clears, vec!["a/id_1".to_string()]);
        assert!(live.live_keys().is_empty());
        // A second DELETEALL with nothing tracked clears NOTHING (it cannot know
        // about an untracked marker).
        let actions = resolve_marker_ops(&ops, &mut live);
        assert!(actions.clears.is_empty());
    }

    /// **THE THREE-OP PIN.** `DELETEALL, ADD k, DELETE k` in ONE array must still
    /// clear `k`.
    ///
    /// A streaming form loses it: the ADD cancels the DELETEALL's
    /// pending clear, the DELETE cancels the ADD, and the DELETE's own clear is
    /// suppressed because `live.remove(&key)` answers `false` — the DELETEALL has
    /// already emptied `live`. Net: no clear, no draw, a permanent ghost.
    #[test]
    fn a_deleteall_then_add_then_delete_of_one_key_still_clears_it() {
        let ops = vec![
            MarkerOp::DeleteAll,
            draw("a", 1),
            MarkerOp::Delete(MarkerKey {
                ns: "a".into(),
                id: 1,
            }),
        ];
        let mut live = live_set(&["a/id_1"]);
        let actions = resolve_marker_ops(&ops, &mut live);
        assert_eq!(actions.clears, vec!["a/id_1".to_string()]);
        assert!(actions.draws.is_empty());
        assert!(live.live_keys().is_empty());
    }

    /// TOTALITY over every 3-op sequence on ONE key, from EVERY frame-start
    /// state: the LAST op that touches it decides, and it is never both cleared
    /// and drawn.
    ///
    /// All THREE start states are driven — Live, Fresh (untracked, never seen)
    /// and Retired (already cleared) — because they give DIFFERENT correct
    /// answers to the same op sequence; a drive of
    /// `Live` alone cannot see the Retired arm at all (the exact
    /// repeat-suppression behaviour). 3 states × 27 sequences = 81 cases.
    #[test]
    fn the_last_op_wins_across_every_three_op_permutation_from_every_start_state() {
        #[derive(Clone, Copy, PartialEq, Debug)]
        enum Op {
            Add,
            Del,
            All,
        }
        #[derive(Clone, Copy, PartialEq, Debug)]
        enum Start {
            /// Currently drawn.
            Live,
            /// Never seen this session (or evicted past the ceiling).
            Fresh,
            /// Already cleared and not since re-drawn.
            Retired,
        }
        let key = MarkerKey {
            ns: "a".into(),
            id: 1,
        };
        let entity = key.entity_key();
        let start_state = |s: Start| -> MarkerLiveState {
            match s {
                Start::Live => live_set(&[&entity]),
                Start::Fresh => MarkerLiveState::default(),
                Start::Retired => {
                    // Reach the retired state through the REAL path, so the test
                    // cannot assert against a state the code can never produce.
                    let mut st = live_set(&[&entity]);
                    let retire = [MarkerOp::Delete(key.clone())];
                    assert_eq!(
                        resolve_marker_ops(&retire, &mut st).clears,
                        vec![entity.clone()]
                    );
                    st
                }
            }
        };
        for s in [Start::Live, Start::Fresh, Start::Retired] {
            for a in [Op::Add, Op::Del, Op::All] {
                for b in [Op::Add, Op::Del, Op::All] {
                    for c in [Op::Add, Op::Del, Op::All] {
                        let seq = [a, b, c];
                        let ops: Vec<MarkerOp> = seq
                            .iter()
                            .map(|o| match o {
                                Op::Add => draw("a", 1),
                                Op::Del => MarkerOp::Delete(key.clone()),
                                Op::All => MarkerOp::DeleteAll,
                            })
                            .collect();
                        let mut live = start_state(s);
                        let actions = resolve_marker_ops(&ops, &mut live);
                        let drawn = drawn_keys(&actions).contains(&entity);
                        let cleared = actions.clears.contains(&entity);

                        // HAND ORACLE, three independent terms:
                        //  - an ADD last ⇒ drawn;
                        //  - otherwise cleared IFF the key was REACHED (named by
                        //    an ADD/DELETE, or swept because it was live) AND is
                        //    not already retired. A sequence of only DELETEALLs
                        //    never names the key, so from Fresh/Retired it does
                        //    nothing at all.
                        let want_drawn = c == Op::Add;
                        let named = seq.iter().any(|o| *o != Op::All);
                        let want_cleared =
                            !want_drawn && (named || s == Start::Live) && s != Start::Retired;

                        let case = format!("{s:?} + {seq:?}");
                        assert_eq!(drawn, want_drawn, "drawn mismatch: {case}");
                        assert_eq!(cleared, want_cleared, "cleared mismatch: {case}");
                        assert!(!(drawn && cleared), "never both at one timestamp: {case}");
                        assert_eq!(live.is_live(&entity), want_drawn, "live mismatch: {case}");
                        // A drawn key is un-retired; anything that ends cleared (now
                        // or already) stays retired.
                        assert_eq!(
                            live.cleared_keys().contains(&entity),
                            !want_drawn && (want_cleared || s == Start::Retired),
                            "retired mismatch: {case}"
                        );
                    }
                }
            }
        }
    }

    /// The two tracking sets are DISJOINT after every transition — the invariant
    /// that makes `needs_clear`'s one-term form (`!cleared.contains`) sound.
    #[test]
    fn the_live_and_cleared_sets_are_always_disjoint() {
        let key = MarkerKey {
            ns: "a".into(),
            id: 1,
        };
        let entity = key.entity_key();
        let check = |st: &MarkerLiveState, at: &str| {
            for k in st.live_keys() {
                assert!(
                    !st.cleared_keys().contains(&k),
                    "{k} is in BOTH sets after {at}"
                );
            }
            // ...and the equivalence the one-term predicate relies on.
            for k in st.live_keys() {
                assert!(st.needs_clear(&k), "a live key must always need clearing");
            }
        };
        let mut st = MarkerLiveState::default();
        check(&st, "default");
        // Drive the full cycle through the REAL entry point, checking after each.
        let add = [draw("a", 1)];
        let del = [MarkerOp::Delete(key.clone())];
        let all = [MarkerOp::DeleteAll];
        for (ops, at) in [
            (&add[..], "add"),
            (&del[..], "delete"),
            (&add[..], "re-add"),
            (&all[..], "deleteall"),
            (&del[..], "delete-of-retired"),
            (&add[..], "re-add-after-deleteall"),
        ] {
            resolve_marker_ops(ops, &mut st);
            check(&st, at);
        }
        assert!(st.is_live(&entity), "the cycle ends drawn");
        assert!(!st.cleared_keys().contains(&entity));
    }

    /// **THE RENDERS-NOTHING PIN.** An `ADD` whose geometry renders NOTHING resolves to a
    /// CLEAR, not to a bare `Transform3D` + early return.
    ///
    /// Under rerun latest-at, drawing nothing leaves the PREVIOUS frame's geometry
    /// standing as the current value — so the common "shrink the array to zero =
    /// nothing detected this cycle" idiom would show stale data indefinitely.
    #[test]
    fn an_add_whose_geometry_renders_nothing_resolves_to_a_clear() {
        let empty = MarkerOp::Draw(Box::new(MarkerDraw {
            key: MarkerKey {
                ns: "a".into(),
                id: 1,
            },
            pose: PoseParts {
                translation: [0.0; 3],
                rotation: None,
            },
            geometry: MarkerGeometry::Points {
                positions: Vec::new(),
                radius: 0.1,
            },
            color: [255, 255, 255, 255],
            instance_colors: None,
        }));
        let mut live = live_set(&["a/id_1"]);
        let actions = resolve_marker_ops(std::slice::from_ref(&empty), &mut live);
        assert_eq!(actions.clears, vec!["a/id_1".to_string()]);
        assert!(actions.draws.is_empty());
        assert!(live.live_keys().is_empty());
        // ANTI-TAUTOLOGY: the SAME key with real geometry draws and does not clear.
        let mut live = live_set(&["a/id_1"]);
        let real = [draw("a", 1)];
        let actions = resolve_marker_ops(&real, &mut live);
        assert!(actions.clears.is_empty());
        assert_eq!(drawn_keys(&actions), vec!["a/id_1"]);
    }

    /// Every geometry's emptiness verdict, against a hand table — the input to the
    /// renders-nothing rule above.
    #[test]
    fn renders_nothing_is_hand_pinned_per_geometry() {
        let cases: Vec<(MarkerGeometry, bool)> = vec![
            (
                MarkerGeometry::Arrows {
                    origins: vec![],
                    vectors: vec![],
                    radius: 1.0,
                },
                true,
            ),
            (
                MarkerGeometry::Arrows {
                    origins: vec![[0.0; 3]],
                    vectors: vec![[1.0, 0.0, 0.0]],
                    radius: 1.0,
                },
                false,
            ),
            (
                MarkerGeometry::Boxes {
                    centers: vec![],
                    size: [1.0; 3],
                },
                true,
            ),
            (
                MarkerGeometry::Ellipsoids {
                    centers: vec![],
                    half_size: [1.0; 3],
                },
                true,
            ),
            (
                MarkerGeometry::LineStrips {
                    strips: vec![],
                    radius: 1.0,
                },
                true,
            ),
            (
                MarkerGeometry::Points {
                    positions: vec![],
                    radius: 1.0,
                },
                true,
            ),
            (MarkerGeometry::Triangles { vertices: vec![] }, true),
            (
                MarkerGeometry::Text {
                    text: String::new(),
                },
                true,
            ),
            (MarkerGeometry::Text { text: "x".into() }, false),
            // A zero-SIZED single primitive is the publisher's own instruction and
            // is REPORTED, never silently converted to a delete.
            (
                MarkerGeometry::Cylinder {
                    length: 0.0,
                    radius: 0.0,
                },
                false,
            ),
            (
                MarkerGeometry::MeshProxy {
                    size: [0.0; 3],
                    uri: String::new(),
                },
                false,
            ),
        ];
        for (geometry, expected) in cases {
            assert_eq!(
                geometry.renders_nothing(),
                expected,
                "{geometry:?} emptiness verdict"
            );
        }
    }

    /// **THE PER-KIND SCALE PIN.** Scale degeneracy is judged per kind on the components
    /// that kind actually CONSUMES — one degenerate and one healthy scale each,
    /// totality-checked against [`MarkerKind::ALL`].
    ///
    /// A predicate testing `scale.x == 0 && scale.y == 0` for the
    /// width-only kinds lets a `LINE_STRIP` of `[0, w, 0]` (a zero-width invisible
    /// line) report NOTHING — the exact silent-nothing the report exists to
    /// explain.
    #[test]
    fn scale_degeneracy_is_per_kind_over_the_components_it_consumes() {
        use MarkerKind as K;
        // (kind, points_len, a DEGENERATE scale, a HEALTHY scale)
        // In `MarkerKind::ALL` order, so the totality check below is also an
        // ordering check. Width-only rows use `[0, 9, 9]` deliberately: a non-zero
        // y/z must NOT rescue a zero-width primitive (an x-and-y predicate lets it).
        let cases: Vec<(K, usize, [f32; 3], [f32; 3])> = vec![
            // Pose form: zero LENGTH (x) or zero diameter (y) is invisible.
            (K::Arrow, 0, [0.0, 1.0, 1.0], [1.0, 1.0, 1.0]),
            // Volumetric: a FLATTENED box is visible, an all-zero one is not.
            (K::Cube, 0, [0.0; 3], [1.0, 1.0, 0.0]),
            (K::Sphere, 0, [0.0; 3], [1.0, 1.0, 0.0]),
            // A cylinder vanishes with no radius OR no height.
            (K::Cylinder, 0, [1.0, 1.0, 0.0], [1.0, 1.0, 1.0]),
            // Width-only kinds: `scale.x` alone.
            (K::LineStrip, 2, [0.0, 9.0, 9.0], [1.0, 0.0, 0.0]),
            (K::LineList, 2, [0.0, 9.0, 9.0], [1.0, 0.0, 0.0]),
            (K::CubeList, 1, [0.0; 3], [1.0, 1.0, 0.0]),
            (K::SphereList, 1, [0.0; 3], [1.0, 1.0, 0.0]),
            (K::Points, 1, [0.0, 9.0, 9.0], [1.0, 0.0, 0.0]),
            // Text's only scale axis is dropped anyway, so it is NEVER degenerate —
            // the "degenerate" column is asserted NOT degenerate for this row.
            (K::TextViewFacing, 0, [0.0; 3], [1.0; 3]),
            (K::MeshResource, 0, [0.0; 3], [1.0, 1.0, 0.0]),
            (K::TriangleList, 3, [0.0; 3], [1.0, 1.0, 0.0]),
            (K::ArrowStrip, 2, [0.0, 9.0, 9.0], [1.0, 0.0, 0.0]),
        ];
        assert_eq!(
            cases.iter().map(|(k, ..)| *k).collect::<Vec<_>>(),
            MarkerKind::ALL.to_vec(),
            "a kind is missing from (or misordered in) the degeneracy table"
        );
        for (kind, points, bad, good) in cases {
            let expect_bad = kind != K::TextViewFacing;
            assert_eq!(
                scale_is_degenerate(kind, bad, points),
                expect_bad,
                "{kind:?} with scale {bad:?}"
            );
            assert!(
                !scale_is_degenerate(kind, good, points),
                "{kind:?} with scale {good:?} must be healthy"
            );
        }
        // ARROW's TWO forms differ, and the form is chosen by the point count: with
        // two points `scale.x` is the shaft diameter and `scale.y` is unused.
        assert!(!scale_is_degenerate(K::Arrow, [1.0, 0.0, 0.0], 2));
        assert!(scale_is_degenerate(K::Arrow, [1.0, 0.0, 0.0], 0));
    }

    /// Which kinds RENDER `points` — the vertex-budget axis, hand-pinned and
    /// totality-checked so a stray bank on a single-primitive kind can never evict
    /// a later marker that really draws geometry.
    #[test]
    fn consumes_points_is_hand_pinned_and_total() {
        use MarkerKind as K;
        let expected: Vec<(K, bool)> = vec![
            (K::Arrow, true),
            (K::Cube, false),
            (K::Sphere, false),
            (K::Cylinder, false),
            (K::LineStrip, true),
            (K::LineList, true),
            (K::CubeList, true),
            (K::SphereList, true),
            (K::Points, true),
            (K::TextViewFacing, false),
            (K::MeshResource, false),
            (K::TriangleList, true),
            (K::ArrowStrip, true),
        ];
        assert_eq!(
            expected.iter().map(|(k, _)| *k).collect::<Vec<_>>(),
            MarkerKind::ALL.to_vec(),
        );
        for (kind, want) in expected {
            assert_eq!(kind.consumes_points(), want, "{kind:?}");
        }
    }

    /// `is_clean` is TOTAL over the report fields.
    ///
    /// Structurally it is `reports == MarkerReports::default()`, so a NEW field is
    /// covered the moment it is added — this test pins that definition by
    /// asserting every current field individually flips it, and that `ops` does
    /// NOT (a frame with markers and no degradations is clean).
    #[test]
    fn is_clean_is_total_over_every_report_field() {
        /// One report field's name and a mutator that makes it non-default.
        type FieldMutator = (&'static str, fn(&mut MarkerReports));
        let mutators: Vec<FieldMutator> = vec![
            ("truncated_markers", |r| r.truncated_markers = 1),
            ("truncated_vertices", |r| r.truncated_vertices = 1),
            ("unknown_types", |r| r.unknown_types = vec![99]),
            ("unknown_actions", |r| r.unknown_actions = vec![1]),
            ("undecodable", |r| r.undecodable = 1),
            ("color_length_mismatches", |r| r.color_length_mismatches = 1),
            ("colors_dropped_kinds", |r| {
                r.colors_dropped_kinds = vec![MarkerKind::LineStrip]
            }),
            ("dropped_tail_kinds", |r| {
                r.dropped_tail_kinds = vec![MarkerKind::LineList]
            }),
            ("elliptical_cylinders", |r| r.elliptical_cylinders = 1),
            ("degenerate_scales", |r| r.degenerate_scales = 1),
            ("lifetime_markers", |r| r.lifetime_markers = 1),
            ("frame_locked_markers", |r| r.frame_locked_markers = 1),
            ("mesh_uris", |r| r.mesh_uris = vec!["u".into()]),
            ("markers_with_frame_id", |r| r.markers_with_frame_id = 1),
        ];
        for (name, mutate) in &mutators {
            let mut plan = MarkerArrayPlan::default();
            mutate(&mut plan.reports);
            assert!(!plan.is_clean(), "{name} must make the plan un-clean");
        }
        // `ops` is NOT a degradation: a frame full of markers with nothing wrong is
        // clean (this is what lets the sink skip the report helper entirely).
        let mut plan = MarkerArrayPlan::default();
        plan.ops.push(draw("a", 1));
        assert!(plan.is_clean());
        assert!(MarkerArrayPlan::default().is_clean());
    }

    #[test]
    fn deleteall_clears_every_live_entity_across_namespaces() {
        let ops = vec![MarkerOp::DeleteAll];
        let mut live = live_set(&["a/id_1", "b/id_2"]);
        let actions = resolve_marker_ops(&ops, &mut live);
        assert_eq!(
            actions.clears,
            vec!["a/id_1".to_string(), "b/id_2".to_string()]
        );
        assert!(live.live_keys().is_empty());
    }

    /// **THE correctness pin.** The dominant real idiom is `DELETEALL` at
    /// `markers[0]` followed by the fresh set. Only the markers that genuinely
    /// went away may be cleared: a marker the SAME frame re-adds must never be
    /// both cleared and drawn at one timestamp, because rerun does not specify
    /// how that tie resolves.
    #[test]
    fn deleteall_does_not_clear_what_the_same_frame_re_adds() {
        let ops = vec![MarkerOp::DeleteAll, draw("a", 1), draw("a", 2)];
        let mut live = live_set(&["a/id_1", "a/id_9"]);
        let actions = resolve_marker_ops(&ops, &mut live);
        assert_eq!(
            actions.clears,
            vec!["a/id_9".to_string()],
            "only the marker that really went away is cleared"
        );
        assert_eq!(drawn_keys(&actions), vec!["a/id_1", "a/id_2"]);
        let cleared: BTreeSet<&String> = actions.clears.iter().collect();
        for key in drawn_keys(&actions) {
            assert!(!cleared.contains(&key), "{key} was both cleared and drawn");
        }
        assert_eq!(live.live_keys(), keys(&["a/id_1", "a/id_2"]));
    }

    /// In-ORDER semantics: an `ADD` BEFORE a `DELETEALL` is erased by it, and the
    /// entity is cleared if it was live (never drawn-and-cleared at once).
    #[test]
    fn an_add_before_a_deleteall_is_erased_by_it() {
        let ops = vec![draw("a", 1), MarkerOp::DeleteAll, draw("a", 2)];
        let mut live = live_set(&["a/id_1"]);
        let actions = resolve_marker_ops(&ops, &mut live);
        assert_eq!(actions.clears, vec!["a/id_1".to_string()]);
        assert_eq!(drawn_keys(&actions), vec!["a/id_2"]);
        assert_eq!(live.live_keys(), keys(&["a/id_2"]));
    }

    /// A `DELETE` after an `ADD` of the same key in ONE frame erases the add.
    #[test]
    fn a_delete_after_an_add_in_one_frame_erases_the_add() {
        let ops = vec![
            draw("a", 1),
            MarkerOp::Delete(MarkerKey {
                ns: "a".into(),
                id: 1,
            }),
        ];
        let mut live = live_set(&["a/id_1"]);
        let actions = resolve_marker_ops(&ops, &mut live);
        assert_eq!(actions.clears, vec!["a/id_1".to_string()]);
        assert!(actions.draws.is_empty());
        assert!(live.live_keys().is_empty());
    }

    /// An `ADD` after a `DELETE` of the same key supersedes it — no clear, one
    /// draw (the other direction of the same-timestamp invariant).
    #[test]
    fn an_add_after_a_delete_in_one_frame_supersedes_the_clear() {
        let ops = vec![
            MarkerOp::Delete(MarkerKey {
                ns: "a".into(),
                id: 1,
            }),
            draw("a", 1),
        ];
        let mut live = live_set(&["a/id_1"]);
        let actions = resolve_marker_ops(&ops, &mut live);
        assert!(actions.clears.is_empty());
        assert_eq!(drawn_keys(&actions), vec!["a/id_1"]);
        assert_eq!(live.live_keys(), keys(&["a/id_1"]));
    }

    /// A re-`ADD` of one key within a frame keeps ONE draw (last payload wins,
    /// first position kept) — a marker entity must not be logged twice at one
    /// timestamp.
    #[test]
    fn a_re_add_within_one_frame_draws_once() {
        let mut second = draw("a", 1);
        if let MarkerOp::Draw(d) = &mut second {
            d.color = [1, 2, 3, 4];
        }
        let ops = vec![draw("a", 1), draw("b", 2), second];
        let mut live = MarkerLiveState::default();
        let actions = resolve_marker_ops(&ops, &mut live);
        assert_eq!(drawn_keys(&actions), vec!["a/id_1", "b/id_2"]);
        assert_eq!(actions.draws[0].color, [1, 2, 3, 4], "last payload wins");
    }

    #[test]
    fn the_live_set_is_capped_and_the_overflow_still_draws() {
        let ops: Vec<MarkerOp> = (0..(MAX_LIVE_MARKERS as i32 + 3))
            .map(|i| draw("a", i))
            .collect();
        let mut live = MarkerLiveState::default();
        let actions = resolve_marker_ops(&ops, &mut live);
        assert_eq!(actions.draws.len(), MAX_LIVE_MARKERS + 3, "all draw");
        assert_eq!(
            live.live_count(),
            MAX_LIVE_MARKERS,
            "the live set is capped at its OWN ceiling, not the per-frame one"
        );
        assert_eq!(
            actions.draw_overflow, 3,
            "three DRAWS past the live ceiling"
        );
        assert_eq!(actions.clear_overflow, 0, "nothing was retired here");
    }

    /// Resolving ONE op list twice ⇒ identical actions, and BOTH equal a hand
    /// oracle (so neither leg is a self-compare).
    ///
    /// Scope is exactly what the name says — one op list, not a SEQUENCE of
    /// frames. The multi-frame render-order determinism pin is
    /// `marker_array_test::the_render_sequence_is_deterministic_across_two_identical_runs`,
    /// which drives a three-frame ADD / DELETE / DELETEALL script.
    #[test]
    fn resolving_one_op_list_twice_is_identical_and_matches_the_oracle() {
        let ops = vec![
            MarkerOp::DeleteAll,
            draw("zeta", 2),
            draw("alpha", 1),
            MarkerOp::Delete(MarkerKey {
                ns: "gone".into(),
                id: 4,
            }),
        ];
        let run = || {
            let mut live = live_set(&["gone/id_4", "old/id_1", "zeta/id_2"]);
            let actions = resolve_marker_ops(&ops, &mut live);
            (
                actions.clears.clone(),
                drawn_keys(&actions),
                live.live_keys(),
                live.cleared_keys(),
            )
        };
        let a = run();
        let b = run();
        assert_eq!(a, b);
        assert_eq!(
            a,
            (
                vec!["gone/id_4".to_string(), "old/id_1".to_string()],
                vec!["zeta/id_2".to_string(), "alpha/id_1".to_string()],
                keys(&["alpha/id_1", "zeta/id_2"]),
                // The two cleared keys are REMEMBERED, so an identical next
                // frame emits nothing (the repeat-suppression half).
                keys(&["gone/id_4", "old/id_1"]),
            ),
            "hand oracle: clears sorted, draws in declaration order"
        );
    }
}
