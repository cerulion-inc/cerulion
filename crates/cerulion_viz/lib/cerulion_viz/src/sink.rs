// SPDX-License-Identifier: AGPL-3.0-only
//! Generic frame → Rerun dispatch: the "one adapter".
//!
//! This is the schema-driven core the `cerulion-vizd` daemon runs to visualize
//! ANY configured topic. It replaces the four bespoke sink crates
//! (lidar / camera / tf / tf_static) with one table:
//!
//! 1. A raw wire frame is decoded by the daemon's [`FrameWalker`] via its
//!    `WireHeader.schema_hash` ([`FrameWalker::walk_by_hash`]) — the same
//!    schema-driven decode the four sinks' typed accessors performed, but for
//!    ANY schema in the walker's set (built-ins + the bridge config's
//!    `msg_dirs` store) with zero per-message code.
//! 2. [`classify_schema`] maps the decoded schema NAME to a Rerun archetype
//!    family ([`ArchetypeKind`]); an UNMAPPED schema then falls to
//!    [`infer_archetype_from_shape`], which inspects the decoded value's FIELD
//!    SHAPE (pose / point / twist / numeric-bag / image / string) so a schema
//!    never seen before still renders as something useful — the "smart map for
//!    an unseen robot" goal.
//! 3. [`dispatch_frame`] builds the mapped-or-inferred archetype through the
//!    [`crate::archetype`] / [`crate::tf`] builders and logs it. A frame that is
//!    neither mapped nor inferable still lands as an inspectable field dump (the
//!    AnyValues-goal fallback) — nothing is un-visualizable.
//!
//! The daemon's poll loop drains each attached tap ACCUMULATE-ALL per tick. A
//! REPLACING-KIND
//! frame ([`coalesces`] — clouds / images / laser scans / the skeleton, which
//! render by OVERWRITING visual state under Rerun's latest-at semantics) is only
//! displayable as the NEWEST frame in a tick's batch, so the drain STAGES the
//! latest and renders it once ([`dispatch_or_stage`]); a per-sample kind
//! (scalars / TF / text) renders EVERY frame so plots keep full temporal
//! resolution. Nothing is clamped by a timestamp: a stream slower than the poll
//! period renders every frame.
//!
//! The render ROUTE comes from [`route_for_input`], keyed by the
//! [`route_key_for_topic`] key the daemon derives when a tap is attached over its
//! UDS control protocol. (This module also used to back a graph-embedded
//! `rerun_sink` node whose `inputs:` list was the config; that crate is deleted —
//! visualization runs desk-side, and nothing feeds this module a wired input list
//! any more.)
//!
//! **The entity path is a MECHANICAL function of the topic**:
//! `world/<sanitized topic segments>` — so distinct topics get distinct entities by
//! construction, and the frame tree lives under the reserved `world/tf-tree/` root
//! that no topic name can produce ([`crate::tf::FRAME_ROOT`]). The route key's LAST
//! SEGMENT now selects only two non-path knobs: `tf`/`tf_static` (temporal vs
//! static transform logging) and `odom`/`robot_odom`/`odometry` (the robot-root
//! election). Where a topic's geometry is POSED is a separate FRAME assignment —
//! `Transform3D:parent_frame` for a transform payload, [`rerun::CoordinateFrame`]
//! for a data one (see [`ArchetypeKind::payload_is_transform`]) — never a path
//! choice.
//!
//! Per-run state (TF unknown-frame + AnyValues once-per-schema flood latches, the
//! `/tf_static` re-broadcast dedup, and the per-input shape-inference
//! memo — see [`SinkState::archetype_for`]) lives in [`SinkState`], owned by the
//! vizd render worker ([`crate::worker`]), which is the one thread holding the
//! `RecordingStream`.
//!
//! All logging is best-effort: a walk failure or a log error is warned (once
//! per regime), never propagated out of the render pass.

use std::collections::{BTreeMap, BTreeSet};
use std::time::Instant;

use cerulion_core::codegen::unknown_hash::{
    diagnose_unknown_hash_with_walker, report_schema_hash_resolved, report_unknown_schema_hash,
    DiagnosisVantage,
};
use cerulion_core::codegen::{FrameValue, FrameValueKind, FrameWalker, WalkError};
use cerulion_core::transport::failure_regime_latch::FailureRegimeLatch;
use cerulion_core::wire::WireHeader;
use go2_tf::decode_tf_transforms;
use rerun::RecordingStream;

use crate::archetype::{
    box3d_parts, cloud_from_frame_value, declares_plottable_series_excluding,
    image_data_from_frame_value, image_dimensions, image_encoding, imu_scalars,
    infer_spatial_shape, log_boxes3d_from_frame, log_coordinate_frame,
    log_element_array_from_frame, log_encoded_image, log_field_dump, log_imu_geometry_in_frame,
    log_laserscan, log_occupancy_from_frame, log_odometry_pose_in_frame, log_points3d,
    log_pose_in_frame, log_raw_image, log_scalar, log_single_point, log_single_text,
    log_transform3d_in_frame, odometry_twist_scalars, opaque_element_arrays, pose_transform_parts,
    raw_image_plan, scalar_samples_with_skips, scalar_shape, scan_element_arrays, single_string_of,
    spatial_sibling_series, sportmode_scalars, ElementArrayParts, ElementArrayScan,
    ElementGeometry, SkippedSeries, SpatialKind, MAX_ARRAY_SERIES, MAX_ELEMENT_INSTANCES,
    MAX_STRUCT_ARRAY_SERIES, MAX_TOTAL_SERIES, PATH_VERTICES_CHILD,
};
use crate::marker::{
    log_marker_clear, log_marker_draw, marker_entity, resolve_marker_ops, scan_marker_array,
    MarkerArrayScan, MarkerFrameActions, MarkerLiveState, MarkerReports, MAX_LIVE_MARKERS,
    MAX_MARKER_INSTANCES, MAX_MARKER_VERTICES,
};
use crate::plot_rate::{DumpRefreshGate, PlotRateGate, RefusalCause, MAX_PLOT_SAMPLES_PER_SEC};
use crate::pointcloud::{FieldsLogAction, FieldsWarnLatch};
use crate::representation::{resolve_render_plan, ForcedDumpGate, RenderPlan, Representation};
use crate::skeleton::{BoundModel, BoundModelStatus, Skeleton, UrdfError, ROBOT_ROOT};
use crate::tf::{
    frame_id_of, implicit_frame_of, implicit_parent_frame_of, log_transforms, sanitize_segment,
    FrameRegistry, UnknownFrameLog, WORLD_ROOT,
};
use crate::video::VideoDemux;

/// Declare [`ArchetypeKind`] AND derive its closed-set table
/// ([`ArchetypeKind::ALL`]) from the SAME variant list.
///
/// `ALL` is generated from exactly the variants written inside the invocation,
/// so a new variant lands in the table BY CONSTRUCTION — there is no second,
/// hand-written array that could silently omit it, and "add a variant but leave
/// `ALL` stale" is not an expressible edit. (A
/// hand-written `[ArchetypeKind; 15]` literal whose doc claims "a new variant
/// cannot compile until it is placed here" enforces nothing: nothing links the
/// array to the enum, and a variant can escape it. The enforcement
/// lives in the LIBRARY build, not in a `#[cfg(test)]` match.)
///
/// Every attribute — the enum's doc comment and `#[derive(..)]`, and each
/// variant's doc comment — passes through unchanged, so rustdoc renders the enum
/// exactly as a hand-written one.
macro_rules! declare_archetype_kinds {
    // Internal: one `()` per variant, so the generated `ALL` keeps its exact
    // fixed-size-array type (a slice would change the public API and the
    // by-value `for kind in ALL` iteration every oracle table uses).
    (@unit $variant:ident) => { () };
    (
        $(#[$enum_meta:meta])*
        $vis:vis enum $name:ident {
            $( $(#[$variant_meta:meta])* $variant:ident, )+
        }
    ) => {
        $(#[$enum_meta])*
        $vis enum $name {
            $( $(#[$variant_meta])* $variant, )+
        }

        impl $name {
            /// The CLOSED set of archetype families, in declaration order.
            ///
            /// Every oracle table over archetypes iterates this, so they all
            /// inherit its totality. GENERATED from the enum's own variant list
            /// by the `declare_archetype_kinds!` macro — a variant physically
            /// cannot be missing here, so a new archetype shows up as a visible
            /// gap in every table that iterates it rather than escaping a
            /// hand-written list.
            pub const ALL: [$name; [$( declare_archetype_kinds!(@unit $variant) ),+].len()] =
                [ $( $name::$variant ),+ ];
        }
    };
}

declare_archetype_kinds! {
/// The Rerun archetype family a Cerulion schema maps to — the closed set the
/// dispatch table + structural inference produce (see [`classify_schema`] and
/// [`infer_archetype_from_shape`]). `AnyValues` is the catch-all: a schema that
/// is neither name-mapped nor shape-inferable is still logged as an inspectable
/// field dump (nothing is un-visualizable).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArchetypeKind {
    /// `sensor_msgs/PointCloud2` → [`rerun::Points3D`] (+ intensity colours).
    Points3D,
    /// `sensor_msgs/Image` / `CompressedImage` (or an inferred image-shape) →
    /// the image family: [`rerun::EncodedImage`] for a JPEG blob, a native
    /// [`rerun::Image`] for a raw `rgb8`/`bgr8`/`mono8` buffer, else a field
    /// dump for an encoding not decoded here (see [`dispatch_frame`]).
    Image,
    /// `tf2_msgs/TFMessage` → per-child [`rerun::Transform3D`] on the entity
    /// tree (temporal for `/tf`, `log_static` for `/tf_static`).
    Transforms,
    /// `geometry_msgs/Twist` / `TwistStamped` / `sensor_msgs/Joy`, OR an
    /// inferred twist-shape / top-level numeric bag → [`rerun::Scalars`] plots
    /// (one time series per component). This is the "never-seen numeric
    /// telemetry becomes live plots" win.
    ///
    /// Carries NUMBERS ONLY. A message that also declares text is
    /// [`ArchetypeKind::ScalarsWithText`] — see there for why the split exists.
    Scalars,
    /// A [`ArchetypeKind::Scalars`]-shaped message that ALSO carries something
    /// the plots cannot show → the [`rerun::Scalars`] plots **and** the
    /// structured field dump, with BOTH a `time_series` and a `text_document`
    /// view.
    ///
    /// TWO disjoint conditions elect it, and the render is identical either way
    /// (the dump shows the whole message, so it covers both):
    ///
    /// - **The message declares TEXT**
    ///   ([`crate::archetype::declares_text_fields`]). The plain `Scalars` arm
    ///   logs numeric samples and nothing else, so a vendor status / response /
    ///   diagnostic message plotted its envelope (`id`, `code`, a level byte)
    ///   while its actual text payload vanished with NO diagnostic.
    /// - **The CURATION withheld numeric fields the dump can SHOW**
    ///   ([`crate::archetype::declares_withheld_series`]). The plot curation gave a wide
    ///   message real curated plots, but the fields its caps withhold are then
    ///   rendered NOWHERE: a `/lowstate`-class bank plots `q` for twenty motors
    ///   and the remaining eleven members of each are visible only by leaving the
    ///   viewer for `cerulion topic echo`. The dump is the in-viewer window onto
    ///   them — bounded (it previews each array and enumerates the first elements
    ///   of a struct array, under a whole-document line ceiling), which is why
    ///   the election is NARROWED to the classes those bounds still cover: an
    ///   over-depth block and a flat over-cap message are deliberately NOT
    ///   elected, because the dump's own depth and line ceilings stop before the
    ///   withheld data. See that predicate for the per-class reasoning.
    ///
    /// Both are the same silent-drop class as the earlier spatial arms, whose
    /// fix ([`ArchetypeKind::Transform3DWithScalars`]) this mirrors exactly: same
    /// render plus the dropped half, and a second view so the dropped half has
    /// somewhere to land.
    ///
    /// **Decided at CLASSIFY time, not at frame time**: that is the blank-pane
    /// lesson applied up front. The layout is derived once from the archetype,
    /// so a render arm that decided per frame whether to dump would log into a
    /// view that may not exist. BOTH electing predicates are correspondingly
    /// frame-INVARIANT: a declared `string` decodes to `Str` even when empty, and
    /// the withholding half reads only curation verdicts a schema fixes (never an
    /// array length this frame happens to carry). FOUR withholding classes are
    /// knowingly excluded, and for TWO different reasons — two fail that
    /// frame-invariance rule, two fail the dump-reach rule (the dump's own depth
    /// and line ceilings stop before the withheld data). See
    /// [`crate::archetype::declares_withheld_series`] for the per-class split.
    ScalarsWithText,
    /// `geometry_msgs/Pose` / `PoseStamped` / `Transform` / `TransformStamped`,
    /// or an inferred pose- or transform-shape → a single
    /// [`rerun::Transform3D`] (translation + rotation) so the frame MOVES in
    /// the 3D view.
    Transform3D,
    /// `geometry_msgs/PointStamped` → a single-point [`rerun::Points3D`].
    Point3D,
    /// An INFERRED transform-shape that ALSO carries plottable
    /// sibling telemetry (`mavros_msgs/PositionTarget`, a custom
    /// `{pose, velocity, battery_v}`) → the [`rerun::Transform3D`] **and** the
    /// sibling [`rerun::Scalars`], exactly like [`ArchetypeKind::Odometry`].
    ///
    /// The plain [`ArchetypeKind::Transform3D`] arm renders ONLY the transform, so
    /// classifying such a message there silently dropped every non-pose number and
    /// gave it no plot view. The sibling harvest EXCLUDES the fields the transform
    /// itself consumed (see [`crate::archetype::spatial_sibling_series`]), so the
    /// pose's own components are never re-plotted.
    Transform3DWithScalars,
    /// The [`ArchetypeKind::Point3D`] twin of
    /// [`ArchetypeKind::Transform3DWithScalars`] — an inferred `{position, …}`
    /// shape whose remaining numbers are real telemetry (a detection's
    /// `{position, velocity, confidence}`) → the point **and** its sibling plots.
    Point3DWithScalars,
    /// `sensor_msgs/Imu` → orientation as a [`rerun::Transform3D`] (rotation)
    /// plus accel/gyro [`rerun::Scalars`] (six time series).
    Imu,
    /// `nav_msgs/Odometry` → the pose as a moving [`rerun::Transform3D`] plus
    /// twist [`rerun::Scalars`].
    Odometry,
    /// `sensor_msgs/LaserScan` → the ranges projected to a
    /// [`rerun::Points3D`] scan ring.
    LaserScan,
    /// `unitree_go/SportModeState` → selected body-telemetry
    /// [`rerun::Scalars`] (velocity, position, yaw speed, body height).
    SportModeState,
    /// An inferred single-string message → a [`rerun::TextLog`] line.
    TextLog,
    /// A box-shaped value (a pose-shaped `center`/`pose` plus an
    /// xyz-shaped `size`/`extents` — `vision_msgs/BoundingBox3D`,
    /// `moveit_msgs/OrientedBoundingBox`, any custom perception detection) → an
    /// oriented [`rerun::Boxes3D`].
    Boxes3D,
    /// `nav_msgs/OccupancyGrid` → the map as a grayscale
    /// [`rerun::Image`] (free → white, occupied → black, unknown → mid gray,
    /// rows flipped to rviz orientation). Previously a `data = <N bytes>` text
    /// dump on every nav2 robot's `/map`.
    OccupancyGrid,
    /// A robot's joint angles → the URDF-derived stick-figure SKELETON
    /// (per-joint [`rerun::Transform3D`]s on the URDF entity tree; see
    /// [`crate::skeleton`]).
    ///
    /// **UNREACHABLE from classification.** Neither
    /// [`classify_schema`] nor [`infer_archetype_from_shape`] can produce it: the
    /// one schema that name-mapped here was demoted BELOW the universal text
    /// floor by the mapping (see the removed row in `classify_schema`), because
    /// the last `install_skeleton` caller was deleted, leaving the archetype
    /// inert on every live run. The variant, its render arm, its
    /// [`coalesces`] / [`can_degrade_to_dump`](Self::can_degrade_to_dump)
    /// classifications and [`crate::skeleton`] itself are all kept EXACTLY as
    /// they were, reachable from tests and from [`SinkState::install_skeleton`]:
    /// `cerulion-vizd` does not call that installer, so the archetype renders only
    /// where a caller installs a skeleton, and the machinery stays intact so that
    /// wiring the installer restores it.
    Skeleton,
    /// An ORDERED array of pose-/point-shaped elements — a trajectory or
    /// a polygon ring (`nav_msgs/Path`, `geometry_msgs/Polygon[Stamped]`, any
    /// vendor type whose elements carry a per-element stamp) → a
    /// [`rerun::LineStrips3D`] polyline plus its vertices as
    /// [`rerun::Points3D`]. THE nav2 headline: a `/plan` DRAWS instead of dumping
    /// text.
    Path3D,
    /// An UNORDERED array of pose-/point-shaped elements
    /// (`geometry_msgs/PoseArray`, `nav_msgs/GridCells`) → [`rerun::Points3D`].
    /// Distinct from [`ArchetypeKind::Path3D`] because connecting unordered
    /// samples with a line would invent structure the message does not carry.
    PoseArray3D,
    /// A `uint8[]` payload whose BYTES are an H.264 Annex-B access unit
    /// → [`rerun::VideoStream`] with `VideoCodec::H264`, one child entity per
    /// rendition (see [`crate::video`]). The viewer decodes; nothing on the robot
    /// transcodes.
    ///
    /// This is the ONE archetype classified by CONTENT rather than by schema name
    /// or field shape, and deliberately so: the generality bar is a camera topic
    /// on a previously-unseen robot, whose message type is unknown.
    VideoStream,
    /// `visualization_msgs/MarkerArray` → one ENTITY per live marker
    /// (`<topic>/viz-markers/<ns>/<id>`), each carrying its pose as a
    /// [`rerun::Transform3D`] plus the archetype its `Marker.type` maps to — and
    /// [`rerun::Clear`] on `DELETE` / `DELETEALL`.
    ///
    /// The ONE STATEFUL archetype. Every other kind here is wholesale-replacing
    /// under rerun latest-at; a `MarkerArray` is a mutation STREAM (a marker
    /// persists until deleted, and an incremental publisher updating one of fifty
    /// live markers sends a ONE-marker array), so the sink keeps a per-input live
    /// set and names exactly what it clears. See [`crate::marker`].
    MarkerArray,
    /// Anything else → an inspectable field dump (the AnyValues fallback goal;
    /// nothing is un-visualizable).
    AnyValues,
}
}

impl ArchetypeKind {
    /// Whether a frame CLASSIFIED as this archetype must render its element array
    /// as an ORDERED path even when the elements' SHAPE alone would infer an
    /// unordered point bag (the classify-vs-render fix).
    ///
    /// Only [`ArchetypeKind::Path3D`] does. A `geometry_msgs/Polygon[Stamped]` is
    /// NAME-mapped to `Path3D` because a ring's order is semantic
    /// ([`classify_schema`]), but its `Point32` elements carry no per-element
    /// stamp — so the shape ladder [`crate::archetype::scan_element_arrays`] yields
    /// [`crate::archetype::ElementGeometry::Points`]. The render arms
    /// ([`crate::archetype::log_element_array_from_frame`]) thread this hint so a
    /// `Path3D`-classified array is PROMOTED to a polyline, keeping the render (a
    /// [`rerun::LineStrips3D`] + its vertices) in agreement with BOTH the
    /// classification and the blueprint components
    /// [`crate::blueprint::archetype_components`] advertises for `Path3D`
    /// (`["LineStrips3D", "Points3D"]`) — which a shape-only render would make
    /// a lie for an unstamped Polygon.
    ///
    /// [`ArchetypeKind::PoseArray3D`] deliberately does NOT force ordering: an
    /// unordered array — name-mapped (`geometry_msgs/PoseArray`,
    /// `nav_msgs/GridCells`) OR genuinely shape-inferred (an unseen vendor
    /// `acme/SampleCloud`) — stays [`rerun::Points3D`], so connecting it with a
    /// line never invents an order the message does not carry.
    ///
    /// **The promotion is asked of the NAME table, never of a memoized
    /// inference** — see [`name_mapped_forces_ordered`], the one caller. That is
    /// not a narrowing: a SHAPE-inferred `Path3D` only ever arose from elements
    /// the same frame's scan had just found STAMPED, so the promotion was already
    /// a no-op on that path (this doc's own "genuinely shape-inferred … stays
    /// Points" claim rested on exactly that). Once the inference is MEMOIZED the
    /// two can disagree — a message with several element arrays picks its winner
    /// by which are non-empty THIS frame — and a stale `Path3D` would then draw a
    /// polyline through a genuinely unordered array. Asking the schema NAME keeps
    /// the un-memoized behaviour exactly.
    pub fn forces_ordered_elements(self) -> bool {
        matches!(self, ArchetypeKind::Path3D)
    }

    /// Whether this archetype's PAYLOAD at the route entity is a rerun
    /// `Transform3D` — which decides HOW the entity is posed.
    ///
    /// rerun 0.34 has two components and they are not interchangeable:
    /// `Transform3D:parent_frame` re-parents the frame chain (the transform cache
    /// reads it; null ⇒ the entity's PATH parent), while `CoordinateFrame:frame`
    /// relocates the entity's own visualizer DATA and the transform cache never
    /// reads it. So for these kinds the frame must ride the payload transform, and
    /// a `CoordinateFrame` would be inert for composition while still moving the
    /// geometry — the worst of both.
    ///
    /// `Odometry`, `Imu` and the `Transform3D` pair log a `Transform3D` at
    /// `route.entity` as their payload. `SportModeState` and `Scalars` log only
    /// plots (nothing spatial), `Points3D`/`Path3D`/`Boxes3D`/images log DATA, and
    /// `Transforms` (a TFMessage) never renders at `route.entity` at all — those
    /// all take the `CoordinateFrame` path.
    pub fn payload_is_transform(self) -> bool {
        matches!(
            self,
            ArchetypeKind::Transform3D
                | ArchetypeKind::Transform3DWithScalars
                | ArchetypeKind::Odometry
                | ArchetypeKind::Imu
        )
    }

    /// Whether this archetype's render arm can DEGRADE to the AnyValues
    /// field dump at frame time — the SINGLE SOURCE OF TRUTH for the
    /// `text_document` view [`crate::blueprint::views_for_archetype`] gives it.
    ///
    /// **The bug this closes: nothing may render nothing.** A topic's layout is
    /// derived ONCE from its CLASSIFIED archetype, but seven render arms decide at
    /// FRAME time that this frame's content does not support the archetype and log
    /// a [`rerun::TextDocument`] instead (`anyvalues_fallback`). With
    /// spatial-only views the dump lands in an entity nothing displays, so the
    /// user sees an empty pane and NO diagnostic — the exact divergence
    /// `cerulion-vizd`'s `resolve_from_frame` doc warns about for the
    /// VideoStream case, generalized. Every kind that answers `true` here
    /// therefore ALSO gets a `text_document` view, and the const assertion in
    /// [`crate::blueprint`] makes that agreement a COMPILE-TIME fact rather than a
    /// promise in a comment.
    ///
    /// The list is EXHAUSTIVE with no wildcard arm, so a new archetype variant is
    /// a compile error until it is classified here on purpose. Each `true` names
    /// the `FallbackReason` its arm raises:
    ///
    /// - [`ArchetypeKind::Image`] — `UnsupportedImageEncoding` (a raw buffer whose
    ///   `encoding` this build does not decode);
    /// - [`ArchetypeKind::OccupancyGrid`] / [`ArchetypeKind::Boxes3D`] —
    ///   `UndecodableOccupancyOrBox` (no extractable grid / box geometry);
    /// - [`ArchetypeKind::Boxes3D`] / [`ArchetypeKind::Path3D`] /
    ///   [`ArchetypeKind::PoseArray3D`] / [`ArchetypeKind::MarkerArray`] —
    ///   `UndecodableElementArray` or `UndecodableElementBytes` (no decodable
    ///   element / marker array), via `render_element_array` /
    ///   `render_marker_array`;
    /// - [`ArchetypeKind::VideoStream`] — `VideoRescanDisagreed`;
    /// - [`ArchetypeKind::Skeleton`] — `UnmappedSchema` (the skeleton is INERT
    ///   without a URDF, which is its production state on every
    ///   robot — `install_skeleton` has no caller — so this arm is not a corner
    ///   case but the norm, and it is why a `/lowstate`-class topic showed a blank
    ///   pane);
    /// - [`ArchetypeKind::AnyValues`] — the dump IS its render, unconditionally.
    ///
    /// [`ArchetypeKind::Transforms`] is deliberately NOT here: it always has a
    /// real render on the production path (`dispatch_transforms`), so a dump pane
    /// for it would be permanently empty on every TF topic in the daemon.
    /// [`ArchetypeKind::Skeleton`] IS here — the two are NOT symmetric, for the
    /// reason its bullet above gives: an inert skeleton renders nothing and falls
    /// through to the dump, which is its shipping state.
    pub const fn can_degrade_to_dump(self) -> bool {
        match self {
            Self::Image
            | Self::OccupancyGrid
            | Self::Boxes3D
            | Self::Path3D
            | Self::PoseArray3D
            | Self::MarkerArray
            | Self::VideoStream
            | Self::Skeleton
            | Self::AnyValues => true,
            Self::Points3D
            | Self::Transforms
            | Self::Scalars
            | Self::ScalarsWithText
            | Self::Transform3D
            | Self::Point3D
            | Self::Transform3DWithScalars
            | Self::Point3DWithScalars
            | Self::Imu
            | Self::Odometry
            | Self::LaserScan
            | Self::SportModeState
            | Self::TextLog => false,
        }
    }

    /// Whether a [`rerun::TextDocument`] can appear AT THE TOPIC ENTITY for this
    /// archetype — the exact condition for the `text_document` view, and the one
    /// [`crate::blueprint`]'s const assertion checks in BOTH directions.
    ///
    /// Two disjoint ways to earn it, deliberately kept distinguishable:
    ///
    /// - a DEGRADATION ([`ArchetypeKind::can_degrade_to_dump`]) — the arm
    ///   renders its own archetype when it can and dumps when this frame's content
    ///   will not support it;
    /// - a PRIMARY render — [`ArchetypeKind::TextLog`] logs a rolling-latest
    ///   `TextDocument` mirror beside its `TextLog` line (rerun 0.34 ships
    ///   no `TextLogView`), [`ArchetypeKind::ScalarsWithText`] logs the field dump
    ///   BESIDE its plots on every frame (and likewise for a topic whose
    ///   curation withheld numeric fields — the election widened, the render did
    ///   not, so this predicate and the const assertion below are unchanged), and
    ///   [`ArchetypeKind::AnyValues`] is a dump by definition (it is counted under
    ///   degradation above, which is the same answer either way).
    ///
    /// Stating it as one predicate is what lets the view table be pinned as an
    /// IFF with no hand-maintained exception list: a kind whose views carry
    /// `text_document` without earning it here would hand every one of its topics a
    /// permanently empty pane, which is the cleanliness half of the same contract.
    pub const fn renders_text_document(self) -> bool {
        self.can_degrade_to_dump() || matches!(self, Self::TextLog | Self::ScalarsWithText)
    }
}

/// What one topic's render arm has been OBSERVED to do — the live
/// "provably rendering" signal that lets the layout refuse the always-there
/// `text_document` companion, generalized from the video-only precedent.
///
/// **Both flags are STICKY**, and each stickiness is a separate decision:
///
/// * `degraded` is sticky because that is the guarantee read literally. A
///   pane that appeared when a frame dumped and VANISHED on the next healthy frame
///   could never be read — the operator would be chasing a tile that is gone by
///   the time they look at it. Once a topic has proved it can dump, its dump has
///   somewhere to land.
/// * `rendered_without_dumping` is sticky because the alternative is a layout that is a
///   function of THIS frame, which is the frame-dependent-layout failure mode
///   (a once-resolved layout must not depend on which frame arrived first).
///
/// Together they bound the layout at **two transitions per topic** —
/// nothing-observed → proven → degraded — so the reflow this drives is the same
/// at-most-once-per-topic class as the daemon's late-resolve reflow, never a
/// per-frame flicker.
///
/// **The SCOPE is the worker's life, not one attach.** Nothing
/// removes an entry on detach — `SinkState` outlives every attach — so a re-attach
/// of the same route inherits the evidence its previous attach earned. That is
/// deliberate rather than merely unfixed: clearing on detach would make the map
/// non-monotone, and its monotonicity is what bounds the reflow the daemon drives
/// from it. The evidence also stays TRUE of the topic — and the direction it
/// carries across a re-attach is the SAFE one, since `degraded` is what keeps a
/// pane rather than removes one, while a stale `rendered_without_dumping` that has
/// stopped being true self-corrects the moment a frame degrades.
///
/// Consumers must therefore gate on whether the topic is CURRENTLY attached rather
/// than on whether an entry exists — `cerulion-vizd`'s `discover` does exactly
/// that, so a topic detached minutes ago reports the default again instead
/// of a refusal nothing is backing.
///
/// The DEFAULT (both `false`) is "nothing observed yet", which keeps the companion
/// — so a caller that cannot supply the signal gets exactly the earlier
/// always-on companion behaviour, and the un-observed topic (the never-seen robot this ladder
/// exists for) is the protected case rather than an afterthought.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RenderProof {
    /// A frame of this topic reached its render arm and the arm returned WITHOUT
    /// degrading to the field dump.
    ///
    /// The name is the exact measurement rather than the conclusion, because the
    /// two come apart on a named, bounded set of paths. Recorded by
    /// `render_classified` for every arm at once, by observing that the arm did
    /// not reach the field-dump funnel — so a newly added render arm carries the
    /// signal without being told to, which is the property that stops this from
    /// becoming the "new arm ships with no signal" bug one layer up.
    ///
    /// **Where it claims slightly more than "it drew", stated rather than
    /// implied.** Four arms can return having logged NOTHING and having dumped
    /// nothing, and they set this flag:
    ///
    /// * [`ArchetypeKind::Image`] with an empty/missing JPEG blob (warn-latched,
    ///   nothing logged) and with a `raw_image_plan` whose buffer will not extract;
    /// * an element array or marker array that decoded and was EMPTY (a `/plan`
    ///   with no poses, a marker set whose markers were all deleted) — here the
    ///   flag is exactly right, because the scan SUCCEEDED;
    /// * a video access unit the decoder produced no picture from.
    ///
    /// The over-claim costs nothing the guarantee covers: those paths log
    /// no `TextDocument` either, so there is no dump to be homeless. The pane they
    /// lose was empty in both worlds. What they do NOT do is hide a dump — the
    /// moment any of them degrades, `degraded` latches and the pane returns.
    pub rendered_without_dumping: bool,
    /// A frame of this topic DEGRADED to the AnyValues field dump.
    ///
    /// Recorded in `anyvalues_fallback`, the ONE function every degradation passes
    /// through (its own `debug_assert!` is what keeps that true).
    pub degraded: bool,
}

/// The schema-name → archetype table — `Some(kind)` for a schema we render
/// natively, `None` for an unmapped schema (the caller then tries
/// [`infer_archetype_from_shape`] before the field-dump fallback). Adding a
/// mapped schema is a one-line edit here; everything upstream is generic.
pub fn classify_schema(schema_name: &str) -> Option<ArchetypeKind> {
    Some(match schema_name {
        "sensor_msgs/PointCloud2" => ArchetypeKind::Points3D,
        "sensor_msgs/Image" | "sensor_msgs/CompressedImage" => ArchetypeKind::Image,
        "tf2_msgs/TFMessage" => ArchetypeKind::Transforms,
        "geometry_msgs/Twist" | "geometry_msgs/TwistStamped" | "sensor_msgs/Joy" => {
            ArchetypeKind::Scalars
        }
        "geometry_msgs/Pose" | "geometry_msgs/PoseStamped" => ArchetypeKind::Transform3D,
        "geometry_msgs/Transform" | "geometry_msgs/TransformStamped" => ArchetypeKind::Transform3D,
        "geometry_msgs/PointStamped" => ArchetypeKind::Point3D,
        "sensor_msgs/Imu" => ArchetypeKind::Imu,
        "nav_msgs/Odometry" => ArchetypeKind::Odometry,
        "sensor_msgs/LaserScan" => ArchetypeKind::LaserScan,
        // The occupancy VALUE semantics (`-1` unknown, `0..=100`
        // percent-occupied, origin-row-first) are not derivable from the field
        // shape, so `/map` is a named mapping — the one class here that a shape
        // rule cannot own correctly.
        "nav_msgs/OccupancyGrid" => ArchetypeKind::OccupancyGrid,
        "unitree_go/SportModeState" => ArchetypeKind::SportModeState,
        //
        // **There is deliberately NO row here for a robot's low-level
        // joint-state message.** Mapping one to
        // [`ArchetypeKind::Skeleton`] would be a DEMOTION rather than a promotion:
        // `views_for_archetype(Skeleton)` composes a 3D view, but
        // the last `install_skeleton` caller has been deleted, so the skeleton
        // stays inert on every live run and the dispatch arm falls to the field
        // dump. The topic would then render a TextDocument into a 3D-only entity
        // — invisible — while an UNMAPPED schema of the same shape rides the
        // shape ladder to `Scalars` and plots its whole joint bank. A bespoke
        // mapping that renders strictly LESS than no mapping at all is worse than
        // the universal floor it was meant to improve on, so the row is gone and
        // the ladder owns the schema.
        //
        // Re-adding it is a one-line change, and whether it is right depends on the skeleton installer:
        // wiring `install_skeleton` into `cerulion-vizd` makes the mapping
        // correct again, deleting the skeleton makes it permanently wrong. Without
        // the installer the ladder's answer is the correct one, and it is only USABLE
        // because plot curation prunes a wide joint bank (a 20-motor × 12-field
        // struct array is 240 plot series at ~500 Hz) — do not restore this row
        // without the installer, and do not remove it in some future branch
        // without the curation.
        // ── The well-known ROS element-array types ──────────────────
        //
        // These are NAMED, not shape-inferred, for a reason that is not laziness:
        // an element array's SHAPE lives in its ELEMENTS, so a frame whose array is
        // momentarily EMPTY carries no shape at all. A topic's layout is resolved
        // ONCE from its first decodable frame (`cerulion-vizd` keeps it), and an
        // idle `/plan` — a nav2 robot with no current plan, the overwhelmingly
        // likely state at attach time — publishes exactly that empty array. Inferred
        // alone, such a topic would be frozen into the view-less `AnyValues`
        // archetype and every later plan logged into a view that does not exist
        // (the frame-dependent-layout failure mode, verbatim). The named table is
        // frame-INVARIANT, so these render from the first frame whatever it carries;
        // `infer_archetype_from_shape`'s element rung then covers the unseen VENDOR
        // types (the automagic half) whose first frame does carry elements.
        //
        // `Polygon`/`PolygonStamped` vs `GridCells` additionally CANNOT be separated
        // by shape at all: a polygon ring and a cell bag are both bare xyz element
        // lists, and only the type says whether the order is semantic. Same class as
        // `nav_msgs/OccupancyGrid` above — semantics a field shape cannot carry get
        // a named mapping, never a guess.
        "nav_msgs/Path" => ArchetypeKind::Path3D,
        "geometry_msgs/Polygon" | "geometry_msgs/PolygonStamped" => ArchetypeKind::Path3D,
        "geometry_msgs/PoseArray" => ArchetypeKind::PoseArray3D,
        "nav_msgs/GridCells" => ArchetypeKind::PoseArray3D,
        "vision_msgs/Detection3DArray" => ArchetypeKind::Boxes3D,
        // A MarkerArray is NAME-mapped for the same frame-invariance
        // reason as the rows above (an idle publisher's empty array
        // carries no shape) AND for a second, sharper one: a `Marker` element IS
        // pose-shaped and stamped, so the shape rung would classify it `Path3D`
        // and draw a polyline through the origins of a dozen unrelated markers —
        // a WRONG picture, worse than the plain text dump it would otherwise
        // get. The named mapping is what routes it to the kind switch instead.
        "visualization_msgs/MarkerArray" => ArchetypeKind::MarkerArray,
        _ => return None,
    })
}

/// The WHOLE classification ladder for a decoded frame — the single seam
/// [`dispatch_frame`] resolves an archetype through.
///
/// Three rungs, in this order:
///
/// 1. **CONTENT** — [`crate::video::classify_h264_payload`]: a `uint8[]` field
///    whose bytes are an H.264 Annex-B access unit makes this a
///    [`ArchetypeKind::VideoStream`] topic.
/// 2. **NAME** — [`classify_schema`]'s table.
/// 3. **SHAPE** — [`infer_archetype_from_shape`]'s structural ladder.
///
/// **Content outranks name deliberately.** Annex-B bytes are unambiguous
/// evidence about what a payload IS, while a schema name is evidence about what
/// someone called it — so a `sensor_msgs/CompressedImage` whose `format` is
/// `h264` renders as video instead of being handed to `EncodedImage`, which
/// could never have drawn it. It cannot mis-fire on the JPEG path: a JPEG opens
/// `FF D8 FF`, which is not a start code, so a compressed IMAGE never reaches
/// rung 1.
///
/// **Stated plainly, because it is the real cost of rung 1: content outranks
/// even a CERTAIN name match, not just an ambiguous one.** The scan runs over
/// every byte field of every frame, so a payload that both belongs to a
/// name-mapped schema AND opens with a valid Annex-B NAL sequence is reclassified
/// away from an answer the name table knew for sure. `scan_annex_b` is built to
/// make that vanishingly unlikely — the start code must be at offset 0 and EVERY
/// unit in the buffer must carry a spec-valid header — and the alternative (name
/// first) forfeits the whole point: a never-seen robot's camera has no name we
/// know.
///
/// Rung 1 costs one linear scan of the payload's bytes per frame, and the render
/// arm scans again to recover the NAL structure it needs. That is the price of
/// keeping classification and rendering separate (every other archetype pays the
/// same shape twice); at camera frame rates it is far below the cost of the walk
/// that produced `fv`.
///
/// **The second scan is a KNOWN, accepted cost, not an oversight.** Threading the
/// already-scanned `H264Payload` from `classify_and_route` through to
/// `render_classified` would remove it, at the price of widening the shared
/// dispatch seam that every archetype flows through. It is not taken here for two
/// reasons: the scan is O(n) over bytes the arm ALREADY copies once
/// unconditionally (`VideoSample` needs an owned `Vec`), so removing it saves a
/// fraction of the per-frame cost rather than a multiple of it; and the seam in
/// question is the one shared surface this change was scoped to leave alone while
/// sibling PRs are in flight against it.
pub fn classify_frame(fv: &FrameValue) -> ArchetypeKind {
    classify_content_or_name(fv).unwrap_or_else(|| infer_archetype_from_shape(fv))
}

/// Rungs 1-2 of [`classify_frame`] — CONTENT then NAME — as ONE function, so the
/// memoized decision path and the standalone one cannot drift apart.
///
/// `None` means only the SHAPE rung is left, which is the half
/// [`SinkState::archetype_for`] memoizes. **Everything decided HERE is deliberately
/// outside the memo**, and for the video rung that is load-bearing, not incidental:
///
/// * A video topic whose schema is UNMAPPED (`unitree_go/Go2FrontVideoData` — the
///   live one) would otherwise reach the shape ladder, infer `Scalars` from its
///   `time_frame`/`video_height` integers, and be MEMOIZED there. The memo is
///   keyed by `(input, schema_hash)`, both of which are stable for a camera, so
///   that answer would then be served for the life of the run — video dead, no
///   error, on the exact topic this feature exists for.
/// * A video topic whose schema IS name-mapped (a `sensor_msgs/CompressedImage`
///   whose `format` is `h264`) needs content to outrank the table, and the memo is
///   consulted only AFTER a table miss — so a rung placed inside the memo could
///   never see it at all.
///
/// Cheap to keep outside: the name half is a string match, and the content half
/// bails on the first non-Annex-B byte of any topic that is not video.
fn classify_content_or_name(fv: &FrameValue) -> Option<ArchetypeKind> {
    if crate::video::classify_h264_payload(fv).is_some() {
        return Some(ArchetypeKind::VideoStream);
    }
    classify_schema(&fv.schema_name)
}

/// STRUCTURAL inference for an UNMAPPED schema (the "smart map for a schema
/// never seen before"): inspect the decoded value's FIELD SHAPE and pick an
/// archetype, so a never-before-seen message renders as something useful
/// instead of a text dump. Precedence, most specific first:
///
/// 1. image-shaped `{width, height, encoding, data}` → [`ArchetypeKind::Image`]
/// 2. box-shaped `{center|pose: pose-shape, size|extents: xyz}` →
///    [`ArchetypeKind::Boxes3D`] (BEFORE the pose rules, or a
///    `{pose, extents}` detection would degrade to a size-less transform)
/// 3. any SPATIAL shape ([`crate::archetype::infer_spatial_shape`], whose ladder
///    is: pose-shaped `{position, orientation}` / transform-shaped
///    `{translation, rotation}`, then planar `{x, y, theta}` (yaw-lifted
///    — every 2D ground robot's pose), then orientation-only (a nested
///    `orientation` / `rotation` / `quaternion` — `QuaternionStamped`, a custom
///    `{header, orientation}` attitude), then a nested `point` / `position` with
///    no orientation (a named position IS a point), then a translation with no
///    rotation) → a transform / point archetype. **Which one depends on whether
///    the message carries anything else:** a PURE spatial value →
///    [`ArchetypeKind::Transform3D`] / [`ArchetypeKind::Point3D`]; one that also
///    carries plottable sibling numbers →
///    [`ArchetypeKind::Transform3DWithScalars`] /
///    [`ArchetypeKind::Point3DWithScalars`], which render the spatial primitive
///    AND those numbers (see the sibling-telemetry note below).
/// 4. A decoded ELEMENT ARRAY whose elements yield geometry
///    ([`crate::archetype::scan_element_arrays`]) → [`ArchetypeKind::Boxes3D`] (N
///    detections), [`ArchetypeKind::Path3D`] (stamped ⇒ ordered) or
///    [`ArchetypeKind::PoseArray3D`] (unstamped ⇒ unordered). Sits BEFORE rule 5
///    deliberately: a 5 000-pose `/plan` carries plottable numbers all through its
///    elements, and classifying it `Scalars` is how it would end up in the numeric
///    harvest instead of the 3D scene.
/// 5. ANY plottable number ([`crate::archetype::harvest_series`]: top-level
///    scalars, nested-struct scalars, numeric arrays — including a bare
///    `{x, y, z}`) → [`ArchetypeKind::Scalars`]
/// 6. a single string field → [`ArchetypeKind::TextLog`]
/// 7. nothing matched → [`ArchetypeKind::AnyValues`] (structured field dump)
///
/// Rule 5 is the big win — a never-seen telemetry message becomes live plots —
/// and a later widening took it from top-level-numerics-plus-twist-shape to the full
/// numeric harvest, so a `Wrench`'s `force`/`torque`, a `MagneticField`'s
/// `magnetic_field`, and a `JointState`'s whole joint bank all plot.
///
/// **Sibling telemetry:** a rule 3 that returns the
/// bare `Transform3D` / `Point3D` for EVERY spatial match drops data, because those
/// render arms log only the spatial primitive. An `acme/DroneState { Point position; Vector3
/// velocity; float64 battery_v }` would render ONE dot, with `velocity` and
/// `battery_v` dropped silently and no plot view — a REGRESSION against
/// classifying the same message as `Scalars`, which
/// plots it. Splitting on the SIBLING harvest keeps the spatial win without the
/// drop: pure poses stay one-view, mixed messages get both views (the
/// `sensor_msgs/Imu` precedent, whose named arm already rendered rotation +
/// scalars).
///
/// Two shapes deliberately stay on rule 5 / the dump, preserving standing
/// decisions rather than guessing: a bare top-level `{x, y, z, w}` is a
/// four-series scalar bag (NOT a quaternion — only a field NAMED like an
/// orientation is treated as one), and a lone `bool` is not auto-plotted.
///
/// Pure over the decoded [`FrameValue`] (oracle-tested), so the
/// generalizes-to-unseen-robots claim is testable without Rerun.
///
/// **Cost note.** Rung 4 runs the CAP-GOVERNED
/// [`crate::archetype::scan_element_arrays`] and keeps only which variant it
/// found — the extracted geometry is dropped, and the render arm scans again.
/// At [`MAX_ELEMENT_INSTANCES`] that duplicate scan is tens of milliseconds per
/// frame, so the per-frame caller must NOT call this on every frame: it goes
/// through [`SinkState::archetype_for`], which memoizes the answer per input.
/// [`infer_archetype_with_stability`] is what makes that memoization safe.
pub fn infer_archetype_from_shape(fv: &FrameValue) -> ArchetypeKind {
    infer_archetype_with_stability(fv).0
}

/// Whether a shape inference may be REUSED for later frames of the same
/// `(input, schema)` — the cacheability half of [`infer_archetype_from_shape`]
/// for the per-input inference memo.
///
/// Almost every rung of the ladder reads DECLARATION shape (which fields exist,
/// what they are named, what type they are), and that is fixed by the schema —
/// the same evidence on every frame. Exactly one rung is not: the element rung
/// asks whether THIS frame's array carried decodable geometry, and an idle
/// `/plan` publishes an empty array. Freezing such a topic on its first frame is
/// the frame-dependent-layout failure mode verbatim (a later populated frame logged
/// into a view that does not exist), so that answer is deliberately not cached.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KindStability {
    /// Decided by evidence that cannot change frame to frame — a declaration
    /// shape, or an element array that DID yield geometry (an element's shape is
    /// fixed by its own schema, so an array that decodes to poses this frame
    /// decodes to poses every frame). Safe to memoize.
    ///
    /// **Scope, stated exactly:** the guarantee is about an element's SHAPE, not
    /// about WHICH array won. On a message declaring several element arrays
    /// (`control_msgs/MotionPrimitive`, `moveit_msgs/CollisionObject`)
    /// [`crate::archetype::scan_element_arrays`] returns the first NON-EMPTY one
    /// that yields geometry, so a later frame — with a different subset populated
    /// — can be decided by a different array, and a memoized kind can name a
    /// geometry class the current frame is not. Two things make that harmless: the
    /// render arms draw what the CURRENT scan found (the kind supplies no
    /// vertices), and the ordered-polyline promotion is asked of the schema NAME
    /// rather than of this kind (see [`ArchetypeKind::forces_ordered_elements`]),
    /// so a stale `Path3D` cannot fabricate an order — pinned by
    /// `sink_dispatch_test::a_stale_path_memo_cannot_draw_a_polyline_through_an_unordered_array`.
    ///
    /// The kind's other consumers are unaffected or converge: [`coalesces`] is
    /// true for all three element archetypes and
    /// [`ArchetypeKind::payload_is_transform`] false for all three, so staging and
    /// framing are identical whichever one is held. The render arms are NOT one
    /// arm — `Path3D`/`PoseArray3D` share one, `Boxes3D` has its own — but that
    /// own arm tries a single-box extraction and then falls through to the SAME
    /// `render_element_array`, so the geometry drawn is the CURRENT scan's on
    /// every path. What a stale kind can still change is which fallback REASON is
    /// quoted when there is nothing decodable to draw: a held `Boxes3D` names the
    /// box reason where the element-array one fits, and vice versa. A diagnostic's
    /// wording, not a drawing. (The blueprint is not a
    /// consumer at all — `cerulion-vizd` derives a topic's views and advertised
    /// components from its own attach-time resolution, never from this memo.)
    Stable,
    /// The element rung was consulted and this frame's array yielded NO geometry
    /// (empty, opaque, or elements that carry none), so the answer came from a
    /// LATER rung by default. A subsequent frame of the same topic may carry
    /// elements and classify differently — never memoize this.
    ElementsUndecided,
}

/// [`infer_archetype_from_shape`] plus whether its answer is safe to memoize
/// (see [`KindStability`]). ONE ladder — `infer_archetype_from_shape` is this
/// function's first tuple element — so the cached and uncached callers can never
/// drift apart.
pub fn infer_archetype_with_stability(fv: &FrameValue) -> (ArchetypeKind, KindStability) {
    use KindStability::{ElementsUndecided, Stable};
    if image_dimensions(fv).is_some()
        && image_encoding(fv).is_some()
        && image_data_from_frame_value(fv).is_some()
    {
        return (ArchetypeKind::Image, Stable);
    }
    if box3d_parts(fv).is_some() {
        return (ArchetypeKind::Boxes3D, Stable);
    }
    // ONE spatial ladder, shared with the render arms (`log_pose` /
    // `log_single_point` mirror it), so a value that CLASSIFIES as a transform /
    // point also RENDERS as one. Deliberately reads only NESTED `position` /
    // `point` for the point rule — a bare top-level `{x, y, z}` falls through to
    // the scalar bag below: an unmapped xyz topic is usually a velocity / force /
    // RPY, and a velocity drawn as a 3D point is a misleading dot at the origin.
    if let Some(shape) = infer_spatial_shape(fv) {
        // Anything the primitive did NOT consume is sibling telemetry. PLOTTABLE
        // siblings ⇒ the both-views archetype, so those numbers get a home
        // instead of being dropped. The split is on the PLOTTABLE half only: a
        // value whose sole other numbers are DELIBERATELY not plotted (a
        // covariance matrix, an over-cap array) would otherwise be handed a
        // time_series view with zero series — the empty-panel failure mode. Those
        // skips are still reported: the render arm calls the same harvest for the
        // plain and the WithScalars arms alike, so nothing is silent either way.
        //
        // Harvests against the shape ALREADY resolved (rather than through
        // `spatial_sibling_series`, which would re-walk the spatial ladder) — the
        // classification must stay a pure function of the frame, because the
        // staging seam re-dispatches a staged frame from its raw bytes.
        //
        // The question is about the SHAPE, not this frame's values (the
        // frame-dependent-layout lesson): every consumer of this classification resolves a topic's
        // layout ONCE (`cerulion-vizd` keeps the archetype it read from the first
        // decodable frame), so asking "does THIS frame carry plottable siblings"
        // froze a topic whose first frame's array happened to be empty into the
        // view-less twin — after which its later series were logged into a view
        // that did not exist. `declares_plottable_series_excluding` is
        // frame-invariant: a declared numeric array counts whatever its current
        // length, while an over-cap array (never expanded) still does not, so a
        // topic that genuinely never plots is still never handed an empty panel.
        let carries_telemetry = declares_plottable_series_excluding(fv, &shape.consumed);
        let kind = match (shape.kind, carries_telemetry) {
            (SpatialKind::Transform, false) => ArchetypeKind::Transform3D,
            (SpatialKind::Transform, true) => ArchetypeKind::Transform3DWithScalars,
            (SpatialKind::Point, false) => ArchetypeKind::Point3D,
            (SpatialKind::Point, true) => ArchetypeKind::Point3DWithScalars,
        };
        return (kind, Stable);
    }
    // A decoded element array whose elements yield geometry. BEFORE the
    // numeric harvest, because a `/plan`'s poses are numbers all the way down: on
    // the harvest rung a 5 000-pose plan is a plot topic (and `push_field`'s
    // `NestedArray` arm reports it as un-plotted), whereas here it is a polyline.
    //
    // Element SHAPE, not the array field's name — an unseen vendor
    // `acme/WaypointList` with the same element shape draws too, which is the
    // automagic bar. An EMPTY array cannot be classified (no elements, no shape),
    // so it falls through here; the well-known ROS types are name-mapped in
    // `classify_schema` precisely to cover that frame-invariance gap.
    if let Some(parts) = scan_element_arrays(fv).parts() {
        let kind = match parts.geometry {
            ElementGeometry::Boxes(_) => ArchetypeKind::Boxes3D,
            ElementGeometry::Path(_) => ArchetypeKind::Path3D,
            ElementGeometry::Points(_) => ArchetypeKind::PoseArray3D,
        };
        return (kind, Stable);
    }
    // Past the element rung with nothing to draw. Every answer BELOW this point
    // is therefore contingent on what this ONE frame's arrays carried, so none of
    // them may be memoized — see [`KindStability::ElementsUndecided`].
    //
    // Shape, not values, for the same reason as the spatial split above: a
    // `{name, joint_positions: float64[]}` whose FIRST frame carries an empty
    // array is still a plotting topic, and a once-resolved layout must not depend
    // on which frame happened to arrive first.
    // ONE walk for all three shape answers — the rung asks them of
    // every frame of every unmapped plot topic and each walk allocates a `String`
    // per sample, so asking them separately re-walked a `/lowstate`-class bank
    // twice per frame at 500 Hz.
    let shape = scalar_shape(fv);
    let kind = if shape.plottable {
        // The dual-view twin covers TWO disjoint ways a plots-only render would
        // drop data, both asked of the SHAPE (never this frame's values) for the
        // same reason every other split on this ladder is:
        //
        // - the message also declares TEXT, which the numeric harvest
        //   cannot plot and which is usually the payload on a vendor
        //   status / response / diagnostic message;
        // - the CURATION withheld numeric fields (a covariance matrix,
        //   a capped struct-array bank, an over-depth block, a message past the
        //   total-series backstop). Under curation those fields are rendered
        //   NOWHERE, and the standing answer was to leave the viewer and
        //   run `cerulion topic echo`. The dump is the in-viewer window onto them.
        if shape.declares_text || shape.withholds {
            ArchetypeKind::ScalarsWithText
        } else {
            ArchetypeKind::Scalars
        }
    } else if single_string_of(fv).is_some() {
        ArchetypeKind::TextLog
    } else {
        ArchetypeKind::AnyValues
    };
    (kind, ElementsUndecided)
}

/// Rotating ring size for RENDERED PointCloud2 sweeps: each rendered
/// (post-coalesce) sweep is logged to `{entity}/viz-sweep/{k}` with `k` cycling
/// `0..SWEEP_ACCUM_RING`, so the viewer shows the last 8 sweeps TOGETHER
/// (decided from live use: 24 smeared a moving robot's cloud; 8 keeps a
/// scene without the motion blur). The ring advances at most ONCE per poll tick
/// (the newest sweep of the tick's batch — see [`coalesces`]); at a representative
/// sweep rate (~14.6 Hz, slower than the 60 Hz poll) every sweep still renders,
/// so the ring holds ≈0.55 s of sweeps. Rosette / solid-state lidars (the Go2's
/// L1 included) publish sparse NON-REPETITIVE sweeps in the SENSOR frame — under
/// Rerun's latest-at semantics a single entity REPLACES each sweep with the
/// next, rendering as a sparse jumping patch instead of a scene. The ring is
/// viewer-side accumulation with zero data copies; a genuinely-changing world
/// still ages out ring-fast. Only the PointCloud2 path rotates: LaserScan stays
/// single-entity (a planar scanner's frame is a full revolution — stable under
/// replacement). TF-composited world-frame accumulation is the endgame.
pub const SWEEP_ACCUM_RING: u64 = 8;

/// The most distinct marker diagnostics one run retains
/// ([`SinkState::marker_notes`]).
///
/// **A whole-set ceiling, ACROSS every input**, not per input — the set is one
/// `BTreeSet` and the cap is its length, so a single pathological topic can
/// exhaust the budget for the others. Deliberate: the alternative (a per-input
/// sub-cap) would need per-input bookkeeping to bound the same total, and the
/// consequence here is only a suppressed DIAGNOSTIC, announced once.
///
/// Two of the discriminators are publisher-controlled — `unknown-type=<value>`
/// and `mesh=<uri>` — so the latch set is not bounded by anything in this crate.
/// Past the cap further diagnostics are suppressed (announced once); rendering is
/// untouched. Generous enough that a real robot's marker topics never approach it.
const MARKER_NOTES_CAP: usize = 512;

/// The SYNTHETIC child segment the rotating sweep ring lives under
/// (`<entity>/viz-sweep/{k}`).
///
/// **The `-` is load-bearing** — same argument as
/// [`crate::archetype::PATH_VERTICES_CHILD`]: a synthesized child shares a
/// namespace with real topics, so a robot publishing `/utlidar/cloud` AND
/// `/utlidar/cloud/sweep/0` would collide on it. `crate::tf::sanitize_segment`
/// cannot emit a `-`, so no topic name can reach this segment.
pub const SWEEP_CHILD: &str = "viz-sweep";

/// The REPLACING-KIND archetypes: those that render by OVERWRITING their visual
/// state under Rerun's latest-at semantics, so within ONE poll tick only the
/// NEWEST frame per input is displayable — every earlier frame in the same
/// tick's drained batch would be overwritten before a display frame, so the
/// drain stages the latest and renders it once (see [`dispatch_or_stage`]). A
/// stream SLOWER than the poll period is unaffected (its one-per-tick frame
/// always renders — nothing is clamped). Scalars / TF / text are NOT
/// replacing: a plot is a STREAM of samples, not whole state, so keeping only a
/// tick's newest frame would throw real data away.
///
/// **Not-replacing is not the same as unbounded.** A plot topic's cost
/// is its publisher's rate × its series count, and a 240-series bank at 500 Hz
/// is ~120 000 logs/s from one topic. Plot topics are therefore rate-GATED
/// instead of coalesced — see [`crate::plot_rate`]. The distinction is
/// deliberate and load-bearing: coalescing keys on the POLL cadence (wall-clock,
/// so a replay coalesces differently), while the gate keys on the publisher's
/// own WIRE timestamps, so the admitted sample set is a pure function of the
/// frame stream.
///
/// `Skeleton` is included: a low-level joint-state firehose (a quadruped
/// publishes one at ~500 Hz, each frame a 12-joint `Transform3D` fan-out) would
/// render at most one pose per tick — genuinely bounded by the poll cadence, not
/// by any hardcoded rate. The schema row that reached this classification has
/// since been removed, so the entry is CORRECT and
/// currently unreached; see [`ArchetypeKind::Skeleton`].
/// Two more replacing kinds followed: a `Boxes3D` detection set and an
/// `OccupancyGrid` map are each a WHOLE-STATE snapshot at one entity (the next
/// frame's boxes / map replaces the previous under latest-at), so only the
/// newest per tick is displayable.
/// The element-array pair is here for the identical reason: a `/plan`'s
/// polyline and a `PoseArray`'s point set are WHOLE geometries at one entity —
/// the next frame's plan replaces the previous one, it does not extend it.
///
/// **[`ArchetypeKind::VideoStream`] is NOT here, and that is a
/// correctness requirement, not an oversight.** A compressed video sample is the
/// opposite of whole state: a P-frame is a DELTA against the frames before it, so
/// dropping the older sample of a tick does not "replace" it — it destroys the
/// reference the survivor decodes from, and the viewer's decoder produces
/// garbage or stalls until the next keyframe. Every access unit must reach the
/// stream. This is also why an H.264 topic is the one archetype whose render cost
/// scales with the PUBLISHER's rate rather than the poll cadence.
///
/// **[`ArchetypeKind::MarkerArray`] is deliberately NOT here**, and its
/// absence is a declared choice pinned by `coalesces_exact_set_oracle` (this is
/// a `matches!`, so a missing arm would default to `false` silently). A
/// `MarkerArray` is a stateful MUTATION stream, not a whole-state snapshot:
/// coalescing it is DATA LOSS, because the frame thrown away may carry the only
/// `DELETE` for a marker, leaving a permanent ghost in the viewer. The cost is
/// that a 100 Hz marker publisher renders every frame — the same trade
/// `Transforms` already makes.
pub fn coalesces(kind: ArchetypeKind) -> bool {
    matches!(
        kind,
        ArchetypeKind::Points3D
            | ArchetypeKind::Image
            | ArchetypeKind::LaserScan
            | ArchetypeKind::Skeleton
            | ArchetypeKind::Boxes3D
            | ArchetypeKind::OccupancyGrid
            | ArchetypeKind::Path3D
            | ArchetypeKind::PoseArray3D
    )
}

/// Where a wired input's frames render, and whether they are STATIC. The
/// per-input half of the config: the input NAME picks the entity + the
/// temporal-vs-static flag (the frame's schema picks the archetype).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InputRoute {
    /// Rerun entity path for the frame (ignored for TF, which derives per-child
    /// entities from the transform tree — see [`crate::tf`]).
    pub entity: String,
    /// `true` routes TF frames through `log_static` + re-broadcast dedup (the
    /// `/tf_static` shape). Irrelevant for non-TF archetypes.
    pub is_static: bool,
    /// `true` when this input's [`ArchetypeKind::Odometry`] frames should ALSO
    /// pose the robot root ([`crate::skeleton::ROBOT_ROOT`], `world/tf-tree/robot`), so
    /// the skeleton + cloud sit at the robot's world pose instead of walking in
    /// place. Set only for odom-named inputs (see [`route_for_input`]);
    /// irrelevant for every non-Odometry archetype.
    pub drives_robot_root: bool,
    /// A CONFIGURED [`rerun::CoordinateFrame`] name that poses this
    /// input's entity, overriding whatever the message's own `frame_id` resolves
    /// to. `None` (the normal case) means "resolve the frame from the data".
    ///
    /// The one producer is [`crate::skeleton::Skeleton::reparent_cloud_route`]:
    /// with a URDF loaded, the lidar cloud is posed in the URDF `radar` link's
    /// frame so the fixed `base → radar` extrinsic superposes cloud + skeleton.
    /// Previously that was done by REWRITING `entity` to the radar link's path,
    /// which re-introduced exactly the collision this issue fixes (two cloud
    /// topics both landing on the radar entity). Posing it by FRAME keeps each
    /// topic's path unique and is the same mechanism the data-derived path uses.
    pub frame: Option<String>,
}

/// Derive the [`route_for_input`] KEY for a topic the
/// dynamic viz daemon taps by ABSOLUTE name.
///
/// **The key is the WHOLE topic, not its last segment.** It was the
/// last segment, which made the render entity a function of the topic's TAIL —
/// and on a real robot that collapses distinct topics onto one entity
/// (for example, `/lf/sportmodestate`, `/mf/sportmodestate` and
/// `/sportmodestate` all rendered to `world/odom/base/sportmodestate`; all 15
/// `/api/*/response` topics to `world/odom/base/response`). Checking two
/// topics in the sidebar drew them into the SAME place, each overwriting the
/// other — a silent wrong answer. The key is now the topic with its leading
/// and trailing `/` trimmed (`/utlidar/cloud` → `utlidar/cloud`, `/tf` →
/// `tf`), which [`route_for_input`] turns into a per-topic-unique
/// `world/utlidar/cloud`.
///
/// An explicit per-tap `entity_override` (the daemon's `attach{entity: "..."}`)
/// still WINS for the ENTITY — which is what every doc for it already claimed
/// (`protocol.rs`'s "render-entity override", Studio's `'e.g. "world/cam"'`).
/// It was fed through the last-segment key path, so `"world/cam"` landed on
/// `world/odom/base/world_cam` and the daemon's OWN reported entity fed back as
/// an override landed somewhere else again — not round-trippable through its
/// own output, i.e. broken exactly where a caller reached for it. The override
/// is normalized here: trimmed of `/`, and ONE leading `world/` segment
/// stripped (so both `world/cam` and `cam` mean the entity `world/cam`). The
/// strip is applied ONLY to an override — never to a topic, where it would
/// alias a real `/world/...` topic onto a shorter one.
///
/// **An override names an entity and NOTHING ELSE.** The key
/// used to BE the normalized override, and [`route_for_input`] derives the
/// `tf`/`tf_static`/`odom` knobs from the key's LAST SEGMENT — so
/// `attach{topic: "/anything", entity: "world/tf_static"}` silently flipped an
/// unrelated topic onto the static-TF arm, and the reported entity of a topic
/// whose knob arm answers something other than its mechanical path was not a
/// fixed point under re-feeding. The key therefore carries BOTH halves: the
/// TOPIC (which alone decides the knobs) and, after
/// [`ROUTE_KEY_OVERRIDE_SEP`], the normalized override (which alone decides the
/// entity). The separator is `U+0001`, which cannot occur in a ROS topic name
/// and is never produced by [`crate::tf::sanitize_segment`], so the split is
/// unambiguous; [`route_key_topic`] recovers the topic half for logs.
///
/// FIXED POINT: feeding the daemon's own reported entity back as an override is
/// a no-op for EVERY topic. A reported entity that equals the topic's mechanical
/// path re-derives to itself; an override that normalizes to nothing (a bare
/// `world` — what the tf arms report, since a TFMessage does not render at a
/// route entity at all) is treated as ABSENT, so the topic keeps its own answer.
///
/// Degenerate inputs are total: a bare `/`, the empty string, or a
/// trailing-slash-only path yield an empty key (which [`route_for_input`]
/// routes to the sanitizer's `unknown` fallback — never a panic).
pub fn route_key_for_topic(topic: &str, entity_override: Option<&str>) -> String {
    let topic_key = topic.trim_matches('/').to_string();
    let Some(entity) = entity_override else {
        return topic_key;
    };
    // An override names an ENTITY PATH. Strip one leading `world/` so it is a
    // fixed point under `route_for_input` (which re-roots at `world`).
    let trimmed = entity.trim_matches('/');
    let normalized = match trimmed.strip_prefix(WORLD_ROOT) {
        // `world/<rest>` → `<rest>`; a bare `world` → nothing (see below).
        Some(rest) if rest.starts_with('/') || rest.is_empty() => rest.trim_start_matches('/'),
        // Anything else (incl. a name that merely STARTS with "world", like
        // `worldly`) keeps every segment.
        _ => trimmed,
    };
    if normalized.is_empty() {
        // An override naming the bare viz root is treated as ABSENT rather than
        // routed to the `unknown` fallback: `world` is the scene root whose
        // transform composes onto EVERY entity, so no topic may claim it, and
        // it is exactly what the tf arms report — so re-feeding a reported
        // entity stays a fixed point instead of landing on `world/unknown_2325`.
        return topic_key;
    }
    format!("{topic_key}{ROUTE_KEY_OVERRIDE_SEP}{normalized}")
}

/// The separator between a route key's TOPIC half and its normalized
/// ENTITY-OVERRIDE half (see [`route_key_for_topic`]). `U+0001` cannot occur in
/// a ROS topic name and is never emitted by [`crate::tf::sanitize_segment`], so
/// the split is unambiguous for any input.
pub const ROUTE_KEY_OVERRIDE_SEP: char = '\u{1}';

/// The TOPIC half of a route key — the whole key when it carries no override.
/// Use this for anything human-facing (logs, diagnostics): the raw key may carry
/// the [`ROUTE_KEY_OVERRIDE_SEP`] control character.
pub fn route_key_topic(key: &str) -> &str {
    key.split(ROUTE_KEY_OVERRIDE_SEP).next().unwrap_or(key)
}

/// A route key's topic half restored to the
/// ABSOLUTE form an operator (and every other surface) knows the topic by.
///
/// [`route_key_for_topic`] TRIMS the leading `/`, so the render worker — which
/// only ever receives route keys — logged `topic=lowstate` while
/// `cerulion-vizd`'s resolver, holding the real topic, logged
/// `topic=/lowstate`. One condition, one `topic=` key, two spellings: an exact
/// grep on either returned half the story. Restoring the slash is the whole fix,
/// and it is exact rather than a guess — every topic vizd attaches is absolute,
/// and the trim is the only transform between the two.
///
/// An already-absolute or empty key is returned untouched, so a non-vizd
/// embedder feeding raw names is neither corrupted nor given a bare `/`.
pub fn log_topic(key: &str) -> String {
    let topic = route_key_topic(key);
    if topic.is_empty() || topic.starts_with('/') {
        topic.to_string()
    } else {
        format!("/{topic}")
    }
}

/// The normalized ENTITY-OVERRIDE half of a route key, if it carries one.
fn route_key_override(key: &str) -> Option<&str> {
    key.split_once(ROUTE_KEY_OVERRIDE_SEP)
        .map(|(_, o)| o)
        .filter(|o| !o.is_empty())
}

/// The MECHANICAL entity-path rule: `world/` + one sanitized segment
/// per non-empty `/`-delimited segment of `key`.
///
/// No topic-name special cases — the path is a pure function of the topic, so
/// distinct topics get distinct entities BY CONSTRUCTION (that is the whole
/// fix). Segments go through the ONE crate sanitizer
/// ([`crate::tf::sanitize_segment`]), whose 4-hex FNV suffix keeps raw names
/// that differ only in separators (`a/b` vs `a.b` as a single segment) apart.
/// An empty key yields `world/<sanitized "">` rather than the bare root, so a
/// degenerate topic can never claim the TF tree's root entity.
fn entity_path_for_route_key(key: &str) -> String {
    let mut out = String::from(WORLD_ROOT);
    let mut any = false;
    for seg in key.split('/').filter(|s| !s.is_empty()) {
        out.push('/');
        out.push_str(&sanitize_segment(seg));
        any = true;
    }
    if !any {
        out.push('/');
        out.push_str(&sanitize_segment(""));
    }
    out
}

/// Resolve a wired input NAME (or, on the dynamic daemon path, a
/// [`route_key_for_topic`] key) to its render route.
///
/// **The ENTITY is now mechanical**: `entity_path_for_route_key`
/// maps the whole name to `world/<sanitized segments>`, so `/utlidar/cloud` →
/// `world/utlidar/cloud` and `/api/vui/response` → `world/api/vui/response`.
/// The media table (`cloud`/`lidar`/`points`/`pointcloud` → the lidar entity,
/// `image`/`camera`/`jpeg`/`compressed` → the camera entity) is GONE as a path
/// rule: it existed to make media render POSED under the `/tf` tree, and that
/// is now a FRAME assignment on the topic's own unique entity (a
/// [`rerun::CoordinateFrame`] resolved from the message's `frame_id` — see
/// [`crate::tf`]), which composes through the same tree without folding two
/// topics onto one path.
///
/// Two non-path knobs still key on the TOPIC's LAST segment (matched
/// case-insensitively — the one-casing rule; the entity keeps the
/// ORIGINAL case, so `WristCam` → `world/WristCam`):
///
/// - `tf` / `tf_static` select temporal vs static TF logging. Their `entity` is
///   the viz root [`WORLD_ROOT`], because a TFMessage does NOT render at the
///   route entity at all — `dispatch_transforms` logs each transform at its
///   own CHILD-frame entity — so the correct answer to "where does `/tf` render"
///   is the tree rooted at `world`. That answer is only correct for a frame that
///   really IS a `tf2_msgs/TFMessage`; a tf-NAMED topic carrying anything else
///   is moved back to its mechanical entity (loudly) at dispatch — see
///   `reconcile_tf_route`, which is where the SCHEMA gets a vote.
/// - `odom` / `robot_odom` / `odometry` ALSO pose the robot root
///   (`drives_robot_root`), so an `nav_msgs/Odometry` frame moves the whole
///   robot in the world — see the [`ArchetypeKind::Odometry`] render arm. A
///   robot whose `/tf` already carries the base pose should NOT name an odom
///   input one of these (it would fight the `/tf` base transform), and two
///   odom-named topics fight EACH OTHER — the render arm warns once when a
///   second one elects.
///
/// Matching on the LAST segment (not the whole name) is what keeps these two
/// working on the daemon path, where the key is now a full topic: `/robot1/tf_static`
/// still logs static, `/utlidar/robot_odom` still poses the robot root.
///
/// **The knobs read the TOPIC half of the key only** (see
/// [`route_key_for_topic`]): an `entity` override names an entity and nothing
/// else, so it can never flip an unrelated topic onto the static-TF arm or the
/// robot-root election. An override REPLACES the mechanical entity — except on
/// the tf arms, whose entity is not a render target at all and so is always
/// [`WORLD_ROOT`].
pub fn route_for_input(name: &str) -> InputRoute {
    let trimmed = route_key_topic(name).trim_matches('/');
    let entity = match route_key_override(name) {
        Some(o) => entity_path_for_route_key(o),
        None => entity_path_for_route_key(trimmed),
    };
    let leaf = trimmed.rsplit('/').next().unwrap_or("");
    match leaf.to_ascii_lowercase().as_str() {
        "tf" => InputRoute {
            entity: WORLD_ROOT.to_string(),
            is_static: false,
            drives_robot_root: false,
            frame: None,
        },
        "tf_static" => InputRoute {
            entity: WORLD_ROOT.to_string(),
            is_static: true,
            drives_robot_root: false,
            frame: None,
        },
        "odom" | "robot_odom" | "odometry" => InputRoute {
            entity,
            is_static: false,
            drives_robot_root: true,
            frame: None,
        },
        _ => InputRoute {
            entity,
            is_static: false,
            drives_robot_root: false,
            frame: None,
        },
    }
}

/// Once-per-distinct-schema flood latch for the AnyValues fallback info log —
/// the [`UnknownFrameLog`] house pattern, keyed on schema name. The FIRST time
/// a given untabled schema is seen it logs (loudly, once); repeats stay silent
/// (so a steady stream of an untabled topic does not flood). Pure — never logs
/// itself.
#[derive(Debug, Default, Clone)]
pub struct UnknownSchemaLog {
    seen: BTreeSet<String>,
}

impl UnknownSchemaLog {
    /// A fresh latch that has seen nothing.
    pub fn new() -> Self {
        Self::default()
    }

    /// Record one sighting of an untabled `schema`. Returns `true` on the
    /// FIRST sighting (the caller logs), `false` for a repeat (silent).
    pub fn observe(&mut self, schema: &str) -> bool {
        self.seen.insert(schema.to_string())
    }

    /// True if `schema` has already taken the fallback path (observability /
    /// test seam — the structural discriminator between "logged natively" and
    /// "logged as a field dump").
    pub fn has_seen(&self, schema: &str) -> bool {
        self.seen.contains(schema)
    }
}

/// One input's memoized shape inference — the answer plus the schema
/// IDENTITY it was derived from, so a topic that starts carrying a different
/// schema (a re-point, a re-attach to another producer) re-infers instead of
/// rendering the previous producer's archetype.
///
/// The identity is the wire `schema_hash`, not the qualified name: the hash IS
/// the layout ([`cerulion_core::wire::WireHeader`]), and the walker can hold two
/// schemas that share a name and differ in layout — vizd seeds a robot's
/// network-fetched `.msg` docs alongside the built-in registry, so a robot's
/// `pkg/Type` and the vendored one coexist. Two frames with the same hash have
/// the same field layout and therefore the same shape, which is exactly the
/// question the memo answers. It is also cheaper: a `u64` compare on the
/// per-frame hit path instead of a `String` one.
#[derive(Debug, Clone, Copy)]
struct InferredArchetype {
    /// The wire `schema_hash` the cached answer was inferred from.
    schema_hash: u64,
    /// The inferred archetype (only ever a [`KindStability::Stable`] one).
    kind: ArchetypeKind,
}

/// Per-run dispatch state the [`crate::sink`] node owns across ticks: the TF
/// unknown-child-frame + AnyValues once-per-schema flood latches, the
/// undecodable-schema-hash + transforms-decode warn-once sets, the per-input
/// empty-JPEG flood latch, the per-input `/tf_static` re-broadcast dedup
/// (last logged `transforms` bytes), the per-entity rotating sweep-ring cursor
/// for rendered cloud sweeps, the running coalesced-frame total, and
/// the per-input shape-inference memo.
#[derive(Debug, Default)]
pub struct SinkState {
    /// The microseconds THIS frame's video access unit spent inside the
    /// OpenH264 decoder, handed from [`render_video_sample`] up to the one place
    /// that knows the frame's wire `sequence` ([`dispatch_or_stage`]) so the decode
    /// sub-stage can be reported on the SAME line as the render total.
    ///
    /// Written and read on the one worker thread, and CLEARED at the START of every
    /// `dispatch_or_stage` while the probe is on — so the value a line reports can
    /// only ever be its OWN call's. A frame that never reached the decoder — the
    /// probe is off, the frame is not video, or it took a [`VideoRoute::Drop`] arm
    /// (`BeforeKeyframe`/`Unattributable`) — always reports `0`. The converse does
    /// NOT hold: a unit that DID reach the decoder can also measure `0` µs (a
    /// parameter-set-only unit returning `DecodeOutcome::NoPicture` does almost no
    /// work), so `0` means "no decode time attributable to this frame", never
    /// "the decoder was not consulted".
    ///
    /// The clear is at the WRITER's door rather than the reader's because the reader
    /// only runs on SAMPLED frames: at any stride > 1 — the runbook's own
    /// recommendation — clearing on read would leave `stride - 1` of every `stride`
    /// decodes parked, and ONE `SinkState` is threaded across ALL inputs
    /// (`worker.rs::process_batch`), so the next sampled line of ANY topic would
    /// report a different frame's — often a different TOPIC's — decode time.
    probe_decode_us: u64,
    /// TF unknown child-frame warn-once (see [`UnknownFrameLog`]).
    unknown_frames: UnknownFrameLog,
    /// AnyValues-fallback once-per-schema info latch.
    unknown_schema: UnknownSchemaLog,
    /// Per-TOPIC flood latch for frames whose `schema_hash` no schema
    /// in the walker's set resolves — the render half of the unknown-hash
    /// diagnostic.
    ///
    /// Keyed by topic (not by hash, as earlier) for two reasons. The hash
    /// is the thing the operator cannot act on, while the topic is what they
    /// asked to see; and a hash-keyed `BTreeSet` warned ONCE per distinct hash
    /// FOREVER, with no counter and no re-announcement, so an operator who
    /// missed the line had no way back to it and no way to ask how bad it had
    /// got. The shared [`FailureRegimeLatch`] gives a loud head, a decade
    /// ladder, an unconditional total, and a recovery line — and recovery is
    /// REAL here, since a `SwapWalker` can seed the missing type mid-run.
    ///
    /// Entries are created only for topics that actually fail, so a healthy
    /// desk holds an empty map and pays one `is_empty()` per frame.
    unknown_hash: BTreeMap<String, FailureRegimeLatch>,
    /// Schemas whose walk/decode failed structurally — warn once each.
    decode_warn: BTreeSet<String>,
    /// Schemas whose numeric harvest deliberately SKIPPED a field (an
    /// over-long array or a covariance matrix — see
    /// [`crate::archetype::SkippedSeries`]) — info once each, so a dropped field
    /// is loud instead of silent.
    skipped_series: BTreeSet<String>,
    /// Inputs whose element array was TRUNCATED at
    /// [`MAX_ELEMENT_INSTANCES`] — warn once each (per INPUT, since two topics of
    /// one schema can differ wildly in element count).
    element_truncated: BTreeSet<String>,
    /// `input::<field NAMES>` keys for every undecodable-array report —
    /// warn once each, so a topic whose array elements this build cannot decode says
    /// WHY it is text instead of a shape. Keyed per INPUT, not per schema, because
    /// element encoding is a property of the PRODUCER: one schema's natively-produced
    /// topic and its bridged twin are different facts.
    ///
    /// The key carries field NAMES ONLY (see [`opaque_latch_key`]) — never the byte
    /// sizes the human-readable line quotes, which shift frame to frame on any
    /// variable-length array and would turn a once-per-topic warn into a per-frame
    /// flood plus an unbounded key set.
    ///
    /// Shared by BOTH reasons that name undecodable arrays
    /// ([`FallbackReason::UndecodableElementBytes`] and an unmapped schema that also
    /// carries them): they state the same producer fact about the same input, so the
    /// second is a repeat whichever one got there first.
    element_opaque: BTreeSet<String>,
    /// Per-input empty-JPEG flood latch ([`FieldsWarnLatch`] house contract:
    /// the FIRST empty frame of a regime WARNs — a camera streaming empties
    /// is operator-visible at info level, the deleted camera sink's behavior
    /// — repeats log at debug with a running count, and a non-empty frame
    /// heals the regime with one recovery info).
    empty_image: BTreeMap<String, FieldsWarnLatch>,
    /// Per-input last-logged `/tf_static` `transforms` bytes (dedup guard):
    /// an unchanged re-broadcast is NOT re-logged (storage-idempotent, since
    /// `log_static` APPENDS a chunk on every call).
    tf_static_last: BTreeMap<String, Vec<u8>>,
    /// Running total of frames COALESCED AWAY: drained this run but never
    /// rendered because a newer frame in the same tick's batch replaced them
    /// (see [`coalesces`] / [`SinkState::coalesced_frames`]). The render loop
    /// folds each batch's per-input count in via [`SinkState::record_coalesced`].
    /// Purely
    /// observability — `num_msgs` compacts same-entity Rerun logs into one chunk
    /// and cannot distinguish 1 rendered from N rendered.
    coalesced_frames: u64,
    /// Per-INPUT record of what the render arms have been observed to do
    /// — see [`SinkState::render_proofs`]. Written by the two `note_render_*`
    /// recorders, read by the worker's per-batch mirror.
    render_proofs: BTreeMap<String, RenderProof>,
    /// The run's running degradation count. Not observability — it is the
    /// DISCRIMINATOR [`render_classified`] reads before and after a render arm to
    /// learn whether that arm drew its archetype or dumped, which is what keeps the
    /// native-render record total instead of eight hand-placed calls that a new arm
    /// could silently omit.
    render_degradations: u64,
    /// Per-entity count of RENDERED (post-coalesce, actually logged)
    /// PointCloud2 sweeps — drives the rotating `sweep/{k}` sub-entity ring
    /// (slot = count % [`SWEEP_ACCUM_RING`]). Advanced ONLY on rendered
    /// sweeps, so a coalesced-away frame never burns a ring slot and replay
    /// stays deterministic (frame-sequence-driven counter — nothing wall-clock).
    sweep_counts: BTreeMap<String, u64>,
    /// The URDF stick-figure skeleton ([`SinkState::install_skeleton`]).
    ///
    /// **NO PRODUCTION INSTALLER.** Nothing in production
    /// reads `GO2_URDF_PATH`
    /// and installs a skeleton, and `cerulion-vizd` does not install one, so
    /// on a live run this stays `Default` (INERT) and the stick figure never
    /// renders. Every caller is a test. `GO2_URDF_PATH` is what an installer
    /// would read; no live path reads it.
    skeleton: Skeleton,
    /// Explicit route and recording binding, independent of schema classification.
    bound_model: Option<BoundModel>,
    // Initial SDK failure is sticky because partial rows cannot be retracted.
    bound_model_install_failed: bool,
    /// The `/tf` / `/tf_static` child frames observed this run — the
    /// evidence [`FrameRegistry::resolve`] needs to decide whether a message's
    /// `frame_id` names a frame the transform tree can actually place.
    frames: FrameRegistry,
    /// Per-entity LAST emitted `CoordinateFrame` name. The assignment is
    /// change-triggered: emit on the first frame and whenever the resolved frame
    /// DIFFERS from the last one, so a topic whose `frame_id` is constant (the
    /// normal case — `/lf/sportmodestate` reported `odom` on 594 of 594 observed
    /// frames) costs one chunk per entity per run instead of one per message.
    frame_emitted: BTreeMap<String, String>,
    /// Inputs whose message carried a `frame_id` that could NOT be
    /// resolved — warn once each. Nothing is logged for them (a fabricated mount
    /// is worse than an unposed entity), so this warn is the ONLY signal that a
    /// topic's data is not localized; it must never be silent.
    unresolved_frame_inputs: BTreeSet<String>,
    /// The TOPICS that have posed [`ROBOT_ROOT`] this run. One
    /// entity, so a second elector means two localization estimates fighting over
    /// the skeleton's pose — see [`note_robot_root_elector`], which warns once the
    /// set reaches two.
    robot_root_electors: BTreeSet<String>,
    /// The per-input H.264 demux — which rendition each access unit
    /// belongs to, and whether that sub-stream has been opened by a keyframe. See
    /// [`crate::video::VideoDemux`].
    video: VideoDemux,
    /// The per-sub-stream H.264 DECODERS the demux routes into. Kept
    /// beside the demux rather than inside it because the demux is a pure,
    /// `Clone`-able decision machine and a decoder is neither — see
    /// [`crate::video_decode`].
    video_decoders: crate::video_decode::VideoDecoders,
    /// Per-input set of marker entity KEYS (`<ns>/<id>`) currently
    /// logged and not yet cleared.
    ///
    /// THE thing that makes `DELETE`/`DELETEALL` exact. Under rerun latest-at a
    /// never-re-logged entity lives forever, so the sink cannot "stop drawing" a
    /// marker — it must NAME what it clears, which means remembering what it
    /// drew. Capped per input at [`crate::marker::MAX_LIVE_MARKERS`] — the
    /// CUMULATIVE ceiling, NOT the per-frame `MAX_MARKER_INSTANCES`; see
    /// [`crate::marker::resolve_marker_ops`].
    ///
    /// CLEARED on a viewer reconnect ([`SinkState::reset_marker_state`]) — the
    /// bounced server holds nothing, so a stale set would make a later
    /// `DELETEALL` emit clears for entities that were never re-logged.
    marker_live: BTreeMap<String, MarkerLiveState>,
    /// `<input>::<discriminator>` keys for the per-marker degradation
    /// reports — warn/info once each, never a per-frame flood.
    ///
    /// ONE latch set with composed keys rather than a dozen sets, because the
    /// reports share a shape (all are "this input's markers do X, once"). EVERY
    /// key is `<input>::<discriminator>`; only the discriminator's shape varies
    /// by class — a fixed word for the input-wide ones
    /// (`input::undecodable-elements`, `input::colors-length`),
    /// `input::unknown-type=99` for the per-VALUE ones, `input::mesh=<uri>` per
    /// URI. Keyed per INPUT, not per schema — two MarkerArray topics on one robot
    /// are different producers with different bugs.
    ///
    /// BOUNDED at [`MARKER_NOTES_CAP`]: two discriminators are publisher-controlled
    /// (`unknown-type=<value>`, `mesh=<uri>`), so a producer cycling mesh URIs
    /// would otherwise grow this set without limit for the life of the run. Also
    /// cleared on a viewer reconnect ([`SinkState::reset_marker_state`]).
    marker_notes: BTreeSet<String>,
    /// Per-INPUT memo of the SHAPE-INFERRED archetype for an unmapped
    /// schema — see [`SinkState::archetype_for`]. Keyed by input (not schema) so
    /// the per-frame lookup is one `&str` probe with NO allocation on a hit; the
    /// schema the answer came from rides the VALUE and is compared, so a topic
    /// whose schema changes re-infers.
    inferred_kind: BTreeMap<String, InferredArchetype>,
    /// How many times the shape-inference ladder actually RAN this run
    /// (Principle #3 observability, and the oracle the cache is pinned by: N
    /// frames of one unmapped element topic must advance this by exactly 1).
    inference_runs: u64,
    /// Per-INPUT memo of "this topic classified as the fully-gated
    /// plot kind [`ArchetypeKind::Scalars`], under this schema hash".
    ///
    /// The gate's cheapness claim depends on it. A SHAPE-INFERRED plot topic
    /// classifies `ElementsUndecided` ([`infer_archetype_with_stability`]), so
    /// [`SinkState::archetype_for`] EVICTS its inference memo on every frame and
    /// the ladder re-runs — and the ladder's plot rung calls
    /// [`declares_plottable_series`] and [`declares_text_fields`], each a FULL
    /// harvest walk with a `String` per sample. A 240-series bank at ~500 Hz pays
    /// that twice per frame whether or not the frame is ever rendered. That memo
    /// must keep evicting (its answer really is contingent on the frame's arrays),
    /// so this is a SEPARATE, narrower memo: it does not decide an archetype, it
    /// only says a refused frame can be dropped before the walk.
    ///
    /// **`ScalarsWithText` is remembered too, and the flag says which.**
    /// Previously this was `Scalars` ONLY, because a `ScalarsWithText` frame logged
    /// its dump unconditionally and so could never be skipped whole — which was
    /// affordable while that archetype meant "a low-rate status message". Classification now
    /// elects it for a `/lowstate`-class firehose as well, so the exemption would
    /// have handed the widest, fastest topic on the robot the most expensive path
    /// (a full walk + ladder on every one of ~500 frames per second). Its dump is
    /// metered now ([`crate::plot_rate::DumpRefreshGate`]), so such a frame CAN be
    /// dropped whole — but only when BOTH halves refuse it, which is what the
    /// `bool` records: `true` = this topic also renders a dump, so ask that gate
    /// too before dropping. Keyed per INPUT + schema hash, so a topic whose schema
    /// changes re-classifies.
    plot_kind_hint: BTreeMap<String, (u64, bool)>,
    /// Per-INPUT plot-sample rate gate — see [`crate::plot_rate`].
    ///
    /// Keyed per input, not per schema, because the rate is a property of the
    /// PRODUCER: one schema published at 500 Hz on a robot and at 1 Hz on a bench
    /// rig are different facts, and a shared gate would hold the slow one to the
    /// fast one's anchor.
    plot_rate: BTreeMap<String, PlotRateGate>,
    /// Per-INPUT field-dump refresh gate — see
    /// [`crate::plot_rate::DumpRefreshGate`]. Same per-input keying, same reason.
    dump_rate: BTreeMap<String, DumpRefreshGate>,
    /// Per-INPUT representation OVERRIDE — the operator's choice of how
    /// this topic renders, or absent for [`Representation::Auto`].
    ///
    /// Absent is the default and the overwhelming majority, so an override is
    /// stored only when it is not `Auto`: "nobody chose" then has exactly ONE
    /// encoding, and a run nobody overrode is byte-identical to one before overrides existed.
    representation: BTreeMap<String, Representation>,
    /// Per-INPUT refresh gate for a FORCED dump — see
    /// [`crate::representation::ForcedDumpGate`]. Same per-input keying, same
    /// reason as the two gates above.
    forced_dump: BTreeMap<String, ForcedDumpGate>,
    /// Frames refused by the header-only pre-walk drop — see
    /// [`SinkState::pre_walk_drops`].
    pre_walk_drops: u64,
}

impl SinkState {
    /// A fresh state that has seen nothing (and an INERT skeleton).
    pub fn new() -> Self {
        Self::default()
    }

    /// TEST SEAM: park a decode time as if a previous frame's access unit
    /// had just been decoded.
    ///
    /// A test cannot produce a RELIABLY non-zero parked value from a real decode —
    /// a fast unit can measure 0 µs, which is exactly the value the bug's absence
    /// also produces, so an end-to-end-only pin would be vacuous whenever the
    /// decoder happened to be quick. Parking a distinctive sentinel makes the
    /// regression detector deterministic (a hand oracle, not a self-compare).
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn set_probe_decode_us_for_test(&mut self, us: u64) {
        self.probe_decode_us = us;
    }

    /// TEST SEAM: read the parked decode time (Principle #3 for the test —
    /// the emitted LINE is the operator-facing oracle, this is the state behind it).
    #[cfg(any(test, feature = "test-helpers"))]
    pub fn probe_decode_us_for_test(&self) -> u64 {
        self.probe_decode_us
    }

    /// Install the URDF skeleton. An inert skeleton leaves every
    /// archetype's behavior unchanged.
    ///
    /// **Called only by tests today** — see the `skeleton` field's
    /// note. The `rerun_sink` node used to call this at `init()` with a skeleton
    /// loaded from `GO2_URDF_PATH`; that node was deleted and nothing replaced it,
    /// so a live run never reaches here. An installed skeleton is
    /// additionally not REACHED by any frame — no schema classifies to
    /// [`ArchetypeKind::Skeleton`] any more — so wiring the installer into
    /// `cerulion-vizd` means restoring that classification too.
    pub fn install_skeleton(&mut self, skeleton: Skeleton) {
        self.skeleton = skeleton;
    }

    /// Bind a strictly loaded model to one exact, already-attached route key.
    ///
    /// The caller must resolve attachment identity before calling. Initial statics
    /// are fallible SDK submissions, not evidence of GPU rendering or live motion.
    /// The URDF root must be `models/<id>`, disjoint from topic and TF paths
    /// beneath `world`. Replacement is rejected; a fresh sink owns a fresh model lifecycle.
    /// Validation failures permit retry. Once initial SDK submission fails or panics,
    /// discard this sink and its partial recording store before another installation.
    pub fn install_bound_model(
        &mut self,
        rec: &RecordingStream,
        route_key: &str,
        skeleton: Skeleton,
    ) -> Result<(), UrdfError> {
        self.install_bound_model_with(rec, route_key, skeleton, |model| model.submit_statics(rec))
    }

    pub(crate) fn install_bound_model_with(
        &mut self,
        rec: &RecordingStream,
        route_key: &str,
        skeleton: Skeleton,
        submit: impl FnOnce(&mut BoundModel) -> Result<(), UrdfError>,
    ) -> Result<(), UrdfError> {
        self.preflight_bound_model_installation(rec)?;
        let mut binding = BoundModel::prepare(rec, route_key, skeleton)?;
        // Arm before the SDK call, including unwinding. No active model survives
        // a failed initial submission, and another installation cannot replay its prefix.
        self.bound_model_install_failed = true;
        if let Err(error) = submit(&mut binding) {
            return Err(UrdfError::Submission(format!(
                "{error}; initial model submission failed; use a fresh sink and recording store"
            )));
        }
        self.bound_model = Some(binding);
        self.bound_model_install_failed = false;
        Ok(())
    }

    /// Check render-worker state before claiming an irreversible SDK submission.
    pub(crate) fn preflight_bound_model_installation(
        &self,
        rec: &RecordingStream,
    ) -> Result<(), UrdfError> {
        self.check_bound_model_install_failure()?;
        if self.bound_model.is_some() {
            return Err(UrdfError::Submission("a model is already installed".into()));
        }
        BoundModel::recording_id(rec).map(|_| ())
    }

    fn check_bound_model_install_failure(&self) -> Result<(), UrdfError> {
        if self.bound_model_install_failed {
            return Err(UrdfError::Submission(
                "initial model submission failed; use a fresh sink and recording store".into(),
            ));
        }
        Ok(())
    }

    /// Submission counters and last rejection; none is a viewer-rendering proof.
    pub fn bound_model_status(&self) -> Option<&BoundModelStatus> {
        self.bound_model.as_ref().map(BoundModel::status)
    }

    /// Re-arm only this model after reconnect; the next selected frame resubmits
    /// statics. Never call periodically: static rows append to viewer storage.
    pub fn rearm_bound_model_statics(&mut self) {
        if let Some(model) = &mut self.bound_model {
            model.rearm_statics();
        }
    }

    /// Resume model statics on the current recording, even without sensor frames.
    /// Successful rows are deduplicated until explicitly rearmed after reconnect.
    /// The worker may retry an error on its bounded reconnect probe; successful
    /// completion becomes a no-op. This proves SDK submission only.
    pub fn submit_bound_model_statics(&mut self, rec: &RecordingStream) -> Result<(), UrdfError> {
        self.check_bound_model_install_failure()?;
        match &mut self.bound_model {
            Some(model) => model.submit_statics(rec),
            None => Ok(()),
        }
    }

    fn is_bound_model_input(&self, route_key: &str) -> bool {
        self.bound_model
            .as_ref()
            .is_some_and(|m| m.matches(route_key))
    }

    fn submit_bound_model_frame(
        &mut self,
        rec: &RecordingStream,
        route_key: &str,
        timestamp_ns: u64,
        frame: &FrameValue,
    ) {
        if let Some(model) = &mut self.bound_model {
            model.submit_frame(rec, route_key, timestamp_ns, frame);
        }
    }

    /// Log the installed skeleton's STATIC link tree (test seam).
    ///
    /// The `Skeleton` archetype arm used to reach this on a low-level joint-state
    /// frame; the schema row that classified one was removed, so today this
    /// seam is how the link chain gets into a recording at all. A test that needs
    /// it to assert COMPOSITION — that a leg link stays path-parented to the robot
    /// root while the root itself is re-parented — should not have to hand-build a
    /// joint-state frame for it.
    #[doc(hidden)]
    pub fn skeleton_log_statics_for_test(&self, rec: &RecordingStream) {
        self.skeleton.log_statics_once(rec);
    }

    /// Forget the per-input `/tf_static` re-broadcast dedup so the NEXT
    /// re-broadcast re-logs. Called after a live reconnect: the bounced (empty)
    /// server has NO `/tf_static` mounts, and the dedup would otherwise suppress
    /// re-logging an unchanged re-broadcast forever — leaving the fresh viewer
    /// without the static transform tree. Complements
    /// [`crate::stream::rearm_after_reconnect`] (which re-arms the world-statics
    /// / blueprint / skeleton guards); together they restore the FULL scene
    /// setup on a reconnected server.
    pub fn clear_rebroadcast_dedup(&mut self) {
        self.tf_static_last.clear();
        self.rearm_bound_model_statics();
    }

    /// Forget every input's per-viewer MARKER state after a reconnect —
    /// the live-key sets and the degradation-report latches.
    ///
    /// **Required, not optional — but not for the reason a first reading
    /// suggests.** DRAWS are independent of the live set (`resolve_marker_ops`
    /// emits every `ADD` it is given, tracked or not), so a stale set does NOT
    /// suppress re-drawing. What it corrupts is DELETE tracking, in two ways:
    ///
    /// - a later `DELETEALL` sweeps keys the fresh (empty) server never received,
    ///   fabricating clears for entities that do not exist there; and
    /// - those dead keys keep occupying [`crate::marker::MAX_LIVE_MARKERS`], so
    ///   genuinely-new markers go UNTRACKED and their own `DELETEALL` becomes a
    ///   no-op — a real ghost, one level removed.
    ///
    /// The report latches are cleared with them because a reconnect starts a new
    /// operator-visible session: restating each degradation once is the point of
    /// once-per-input, and it bounds the latch set across a long-lived run.
    ///
    /// Called from the SAME reconnect hook as [`SinkState::clear_rebroadcast_dedup`]
    /// and [`crate::stream::rearm_after_reconnect`]; kept a separate method
    /// because the two answer different questions (one forgets what was DEDUPED,
    /// this forgets what is BELIEVED LIVE).
    pub fn reset_marker_state(&mut self) {
        self.marker_live.clear();
        self.marker_notes.clear();
    }

    /// The number of markers this input currently believes are live — the
    /// Principle #3 observable behind [`SinkState::reset_marker_state`] and the
    /// clear algorithm (test seam; production reads nothing here).
    pub fn live_marker_count(&self, input_name: &str) -> usize {
        self.marker_live
            .get(input_name)
            .map_or(0, MarkerLiveState::live_count)
    }

    /// The re-parented render route for a wired input: when the skeleton is
    /// active, the lidar cloud moves under the URDF `radar` link (superposing
    /// cloud + skeleton via the fixed extrinsic); otherwise the input's default
    /// [`route_for_input`] route is returned unchanged.
    pub fn route_for(&self, input_name: &str) -> InputRoute {
        self.skeleton
            .reparent_cloud_route(input_name, route_for_input(input_name))
    }

    /// How many frames on `topic` this sink dropped because no schema
    /// in its walker resolved their `schema_hash`.
    ///
    /// UNCONDITIONAL and never reset by recovery — the Principle #3 answer to
    /// "is this topic streaming frames I cannot decode?", answerable at
    /// `RUST_LOG=error` and hours after the loud head scrolled away. `topic` is
    /// the ABSOLUTE topic (what [`log_topic`] produces and the reporter logs),
    /// so it is the same string `cerulion-vizd`'s resolver-side line carries.
    ///
    /// # Scope: the unknown-HASH arm only
    ///
    /// It counts the frames refused at the hash gate — NOT the sibling
    /// structural-walk failures (a truncated or corrupt frame under a hash this
    /// build DOES hold), which are a different condition with a different remedy
    /// and ride their own once-per-input `decode_warn` latch. Merging them would
    /// make a single number mean two things and let one condition's volume hide
    /// the other's onset, which is exactly the separation the reporter
    /// keeps everywhere else. So a topic blanked purely by malformed frames
    /// reads `0` here, and that is a deliberate reading, not a miss.
    pub fn undecodable_frame_count(&self, topic: &str) -> u64 {
        self.unknown_hash
            .get(topic)
            .map(|l| l.total_failures())
            .unwrap_or(0)
    }

    /// True if `schema` has taken the AnyValues field-dump fallback path this
    /// run — the structural discriminator between "rendered natively" (e.g.
    /// `EncodedImage`) and "degraded to the inspectable dump" (test seam).
    pub fn took_anyvalues_fallback(&self, schema: &str) -> bool {
        self.unknown_schema.has_seen(schema)
    }

    /// What each INPUT's render arm has been observed to do — the live
    /// "provably rendering" signal the layout gates the dump companion on.
    ///
    /// Keyed by input (route key), NOT by schema like
    /// [`SinkState::took_anyvalues_fallback`]: the layout is per TOPIC, and one
    /// schema legitimately arrives on a natively-produced topic and on a bridged
    /// twin whose element bytes this build cannot decode — a per-schema verdict
    /// would let one topic's degradation refuse the other's pane, and vice versa.
    ///
    /// Mirrored onto [`crate::worker::VizWorkerCounters::render_proofs`] after each
    /// batch, exactly as [`SinkState::coalesced_frames`] and the video rendition
    /// set are: the render arms run on the worker thread and the layout is built on
    /// the daemon's control thread, so the mirror is the only path between them.
    pub fn render_proofs(&self) -> BTreeMap<String, RenderProof> {
        self.render_proofs.clone()
    }

    /// This input's render proof, or the DEFAULT (nothing observed) when no frame
    /// of it has reached a render arm yet.
    pub fn render_proof_for(&self, input_name: &str) -> RenderProof {
        self.render_proofs
            .get(input_name)
            .copied()
            .unwrap_or_default()
    }

    /// Record that this input's frame DEGRADED to the field dump. Called from the
    /// ONE funnel every degradation passes through ([`anyvalues_fallback`]), so the
    /// record is total by construction rather than by placing a call in each arm.
    fn note_render_degraded(&mut self, input_name: &str) {
        self.render_degradations = self.render_degradations.saturating_add(1);
        self.render_proofs
            .entry(input_name.to_string())
            .or_default()
            .degraded = true;
    }

    /// Record that this input's frame rendered its archetype NATIVELY.
    fn note_render_native(&mut self, input_name: &str) {
        self.render_proofs
            .entry(input_name.to_string())
            .or_default()
            .rendered_without_dumping = true;
    }

    /// The running count of degradations this run — read BEFORE and AFTER a render
    /// arm so [`render_classified`] can tell "this frame drew its archetype" from
    /// "this frame dumped" without a success call in each of the eight arms.
    fn render_degradations(&self) -> u64 {
        self.render_degradations
    }

    /// Add `n` frames coalesced away this batch (drained but replaced by a newer
    /// frame in the same batch, so never rendered) to the running total. The
    /// render loop calls this once per input per batch with the count
    /// [`dispatch_or_stage`] accumulated.
    pub fn record_coalesced(&mut self, n: u64) {
        self.coalesced_frames += n;
    }

    /// The running total of frames coalesced away this run — the
    /// compaction-independent observability signal (`num_msgs` cannot tell 1
    /// rendered from N rendered). See [`coalesces`] / [`dispatch_or_stage`].
    /// Mirrored onto the daemon's `VizWorkerCounters` (which is what `status`
    /// reports); the sink-node `shutdown()` surfacing this doc used to describe
    /// went with the deleted `rerun_sink` crate.
    pub fn coalesced_frames(&self) -> u64 {
        self.coalesced_frames
    }

    /// May this input's PLOT frame, stamped `timestamp_ns`, contribute
    /// samples? Consulted BEFORE the harvest, so a refused frame costs neither the
    /// harvest walk nor the logs.
    ///
    /// The first refusal on a topic emits the once-per-input operator report — a
    /// topic whose plots are thinned must say so, or an operator reading a
    /// single-digit-Hz trace of a 500 Hz publisher has no way to tell decimation
    /// from a sick producer. A topic that is never refused never reports, and a
    /// topic held because its publisher's clock is STOPPED gets a DIFFERENT report
    /// (see [`crate::plot_rate::RefusalCause`]) — the two conditions have different
    /// remedies and an operator must not have to guess which one they are looking
    /// at.
    fn admit_plot_frame(&mut self, input_name: &str, timestamp_ns: u64) -> bool {
        let gate = self.plot_rate.entry(input_name.to_string()).or_default();
        if gate.admit(timestamp_ns) {
            return true;
        }
        if let Some(cause) = gate.claim_report() {
            let (series, interval_ns, suppressed) =
                (gate.series(), gate.current_interval_ns(), gate.suppressed());
            let input = route_key_topic(input_name);
            match cause {
                RefusalCause::OverBudget => tracing::info!(
                    input,
                    series,
                    min_interval_ms = interval_ns as f64 / 1e6,
                    max_samples_per_sec = MAX_PLOT_SAMPLES_PER_SEC,
                    suppressed,
                    "cerulion_viz: this topic's plot samples are DECIMATED — its series count \
                     times its publish rate exceeds the per-topic budget, so at most one frame \
                     per min_interval_ms of the PUBLISHER's own wire time is plotted. Keyed on \
                     wire timestamps, never wall time, so a replay renders the same samples. \
                     Every frame is still on the wire: read them with `cerulion topic echo`"
                ),
                RefusalCause::StalledClock => tracing::warn!(
                    input,
                    series,
                    suppressed,
                    "cerulion_viz: this topic's publisher is stamping every frame with the SAME \
                     wire timestamp, so its plot holds one sample. The timeline IS the wire \
                     stamp, so further frames would all land at that one position and the plot \
                     could not separate them — this is a producer clock that is not advancing, \
                     NOT a rate limit. Check the publisher's clock source"
                ),
            }
        }
        false
    }

    /// Can this frame be dropped BEFORE the walk?
    ///
    /// `true` only when the input is a remembered fully-gated plot topic
    /// ([`SinkState::plot_kind_hint`]) carrying the same schema, AND the rate gate
    /// refuses the stamp. The refusal is fully accounted here, so the caller must
    /// not consult the gate again for this frame — see [`PlotAdmission`].
    ///
    /// Reads the header only: no `walk_by_hash`, no harvest, no logs. That is the
    /// whole point — the classification ladder a shape-inferred plot topic re-runs
    /// on every frame is what a gate placed after the walk cannot avoid.
    /// Returns the two verdicts, or `None` when the input is not a
    /// remembered gated plot topic (so nothing was consulted and nothing
    /// accounted). `Some((series, dump))` means BOTH gates ran; the caller must
    /// carry them in [`PlotAdmission::Decided`] rather than asking again.
    ///
    /// `dump` is asked only of a topic whose kind renders one — for a plain
    /// `Scalars` topic there is no second half, and consulting its floor would
    /// count refusals for a document that is never logged.
    ///
    /// The exemption is EXACTLY "a dump can ride a series-REFUSED frame",
    /// not "the topic is overridden". The hint's `renders_dump` bool
    /// IS the elected kind on this path — `plot_kind_hint` only ever records
    /// `Scalars` / `ScalarsWithText` — so the plan can be resolved from the header
    /// alone and the exemption keyed on the plan rather than on the map:
    ///
    /// - `Text` (visual suppressed) and `Both` on a plain `Scalars` topic set
    ///   `plan.dump`, so a document really does ride frames the series gate
    ///   refuses and dropping on the series verdict alone would FREEZE it — the
    ///   very freeze the anti-freeze requirement forbids. Those keep the exemption.
    /// - `Visual` resolves to `{visual: Some(Scalars), dump: false}` on both
    ///   hinted kinds, so no document rides the stream and the series verdict is
    ///   the correct criterion. A bare `contains_key` sweeps it in, which INVERTS
    ///   the cost of the override's own gesture: taking the dump away from a
    ///   ~500 Hz `/lowstate` surrenders a drop that refuses ~98 % of frames on
    ///   their header, so asking for strictly LESS rendering buys ~15-64× MORE
    ///   classification work (the ladder is non-memoizable for these kinds).
    /// - `Both` on a `ScalarsWithText` topic is plan-identical to `Auto`
    ///   (`renders_own_dump` ⇒ no overlay), so it keeps the fast path too — and
    ///   its dump verdict must still come from `admit_field_dump`, or the dump
    ///   metering would go inert.
    ///
    /// The residual cost is now confined to the two arms that provably need it.
    fn decide_plot_frame_before_walk(
        &mut self,
        input_name: &str,
        schema_hash: u64,
        timestamp_ns: u64,
    ) -> Option<(bool, bool)> {
        let renders_dump = match self.plot_kind_hint.get(input_name) {
            Some((hinted, _)) if *hinted != schema_hash => return None,
            Some((_, renders_dump)) => *renders_dump,
            None => return None,
        };
        let elected = if renders_dump {
            ArchetypeKind::ScalarsWithText
        } else {
            ArchetypeKind::Scalars
        };
        let plan = resolve_render_plan(elected, self.representation_for(input_name));
        if plan.visual.is_none() || plan.dump {
            return None;
        }
        let series = self.admit_plot_frame(input_name, timestamp_ns);
        // The elected kind's OWN dump (its metered `ScalarsWithText` half),
        // read off the resolved plan rather than the hint — `Visual` strips that
        // half, and consulting the gate for a document it never logs would count
        // refusals against nothing.
        let dump = plan.visual == Some(ArchetypeKind::ScalarsWithText)
            && self.admit_field_dump(input_name, series);
        Some((series, dump))
    }

    /// The ONE gate consult every curated-series emission passes
    /// through. `Decided` means the early pre-walk gate already decided AND
    /// accounted this frame; `Undecided` means nothing has, so decide now.
    fn admit_curated_series(
        &mut self,
        input_name: &str,
        timestamp_ns: u64,
        admission: PlotAdmission,
    ) -> bool {
        match admission {
            PlotAdmission::Decided { series, .. } => series,
            PlotAdmission::Undecided => self.admit_plot_frame(input_name, timestamp_ns),
        }
    }

    /// Whether this frame re-renders its `ScalarsWithText` field dump.
    ///
    /// Rides the SERIES verdict for the same frame — one story on the screen, no
    /// second budget — with [`DumpRefreshGate`]'s frame-count floor as the
    /// anti-freeze backstop that keeps the requirement (a stalled
    /// publisher clock must not freeze the text at frame 1 for the whole run).
    fn admit_field_dump(&mut self, input_name: &str, series_admitted: bool) -> bool {
        self.dump_rate
            .entry(input_name.to_string())
            .or_default()
            .admit(series_admitted)
    }

    /// Set (or clear) this input's representation override.
    ///
    /// [`Representation::Auto`] REMOVES the entry rather than storing a default,
    /// so "no override" has exactly one encoding and a topic returned to
    /// automagic is indistinguishable from one nobody ever touched. The forced
    /// dump's gate is dropped with it — a re-forced dump starts fresh rather than
    /// inheriting a refusal window from a choice the operator has since undone.
    pub fn set_representation(&mut self, input_name: &str, representation: Representation) {
        if representation.is_auto() {
            self.representation.remove(input_name);
            self.forced_dump.remove(input_name);
        } else {
            self.representation
                .insert(input_name.to_string(), representation);
        }
    }

    /// This input's representation — [`Representation::Auto`] unless the
    /// operator chose otherwise.
    pub fn representation_for(&self, input_name: &str) -> Representation {
        self.representation
            .get(input_name)
            .copied()
            .unwrap_or_default()
    }

    /// Whether this frame re-renders a FORCED dump.
    ///
    /// A separate gate from [`SinkState::admit_field_dump`] because a forced dump
    /// may sit on a topic with no series gate at all (a camera, a cloud, a
    /// transform tree), so there is no series verdict to ride — see
    /// [`ForcedDumpGate`].
    fn admit_forced_dump(&mut self, input_name: &str, timestamp_ns: u64) -> bool {
        self.forced_dump
            .entry(input_name.to_string())
            .or_default()
            .admit(timestamp_ns)
    }

    /// How many frames the header-only pre-walk drop refused this run —
    /// the observable that tells a `Visual` override apart from a `Text` one
    /// WITHOUT timing anything (Principle #3).
    ///
    /// The two are behaviourally identical downstream — a refused frame renders
    /// nothing either way — so the only difference an exemption makes is whether
    /// the frame was decoded and classified before being refused. A CPU
    /// measurement would answer that and fail open on a loaded runner; this counts
    /// the decision itself.
    pub fn pre_walk_drops(&self) -> u64 {
        self.pre_walk_drops
    }

    /// The running total of FORCED dump renders withheld this run,
    /// across every input (Principle #3 — observable independently of the logs).
    pub fn forced_dump_renders_withheld(&self) -> u64 {
        self.forced_dump
            .values()
            .map(ForcedDumpGate::suppressed)
            .sum()
    }

    /// The running total of field-dump renders withheld this run, across
    /// every input (Principle #3 — observable independently of the logs).
    pub fn dump_renders_withheld(&self) -> u64 {
        self.dump_rate
            .values()
            .map(DumpRefreshGate::suppressed)
            .sum()
    }

    /// Record how many series the plot frame just rendered yielded — the
    /// budget the input's NEXT [`SinkState::admit_plot_frame`] spends.
    fn note_plot_series(&mut self, input_name: &str, series: usize) {
        if let Some(gate) = self.plot_rate.get_mut(input_name) {
            gate.note_series(series);
        }
    }

    /// The running total of PLOT frames the rate gate refused this run,
    /// across every input (Principle #3 — the decimation is observable
    /// independently of whether the once-per-input log line was read or filtered).
    pub fn plot_frames_decimated(&self) -> u64 {
        self.plot_rate.values().map(PlotRateGate::suppressed).sum()
    }

    /// The rotating `{entity}/viz-sweep/{k}` sub-entity for the NEXT rendered
    /// cloud sweep at `entity`, advancing the per-entity ring cursor
    /// (`k = rendered_count % `[`SWEEP_ACCUM_RING`]). Call ONLY for a
    /// RENDERED (post-coalesce) sweep — a coalesced-away frame must not burn a
    /// ring slot, so identical frame sequences produce identical path
    /// assignments (replay determinism).
    pub fn next_sweep_entity(&mut self, entity: &str) -> String {
        let count = self.sweep_counts.entry(entity.to_string()).or_insert(0);
        let slot = *count % SWEEP_ACCUM_RING;
        *count += 1;
        format!("{entity}/{SWEEP_CHILD}/{slot}")
    }

    /// The number of rendered sweeps assigned at `entity` so far — the ring
    /// cursor is `rendered_sweeps % `[`SWEEP_ACCUM_RING`] (observability /
    /// test seam: pins that coalesced-away frames do not advance the ring).
    pub fn accepted_sweeps(&self, entity: &str) -> u64 {
        self.sweep_counts.get(entity).copied().unwrap_or(0)
    }

    /// Read-only view of the H.264 demux — which sub-streams an input
    /// has produced, how many samples each was fed, and how many access units
    /// were dropped and why (observability / test seam).
    pub fn video(&self) -> &VideoDemux {
        &self.video
    }

    /// Replace the decoder pool — the seam that lets a test drive the
    /// NO-DECODER fallback, which is otherwise unreachable in a build that
    /// compiled the decoder in (see
    /// [`crate::video_decode::VideoDecoders::unavailable_for_test`]).
    pub fn set_video_decoders(&mut self, decoders: crate::video_decode::VideoDecoders) {
        self.video_decoders = decoders;
    }

    /// Forget which sub-streams have been ANNOUNCED after a
    /// live reconnect.
    ///
    /// Load-bearing ONLY in the no-decoder FALLBACK regime — and there it is
    /// critical: that path logs `VideoStream` samples for the viewer to decode,
    /// rerun's H.264 contract REQUIRES the static codec component, and a bounced
    /// server holds none, so without this the declaration latch would never
    /// re-fire and every video topic would be dead for the rest of the run. When
    /// this desk is decoding it costs one repeated breadcrumb.
    pub fn rearm_video_after_reconnect(&mut self) {
        self.video.rearm_stream_announcements();
    }

    /// Read-only view of the per-sub-stream H.264 DECODERS — pictures
    /// produced and access units refused, per rendition (observability / test
    /// seam; both counters are log-level independent, Principle #3).
    pub fn video_decoders(&self) -> &crate::video_decode::VideoDecoders {
        &self.video_decoders
    }

    /// **THE** per-frame archetype decision: the name-mapped table
    /// ([`classify_schema`]) first, else the SHAPE inference
    /// ([`infer_archetype_from_shape`]) — MEMOIZED per input.
    ///
    /// # Why the memo exists
    ///
    /// The name-mapped half is a string match. The inference half is not: its
    /// element rung runs the cap-governed [`crate::archetype::scan_element_arrays`]
    /// and then THROWS THE GEOMETRY AWAY, after which the render arm scans the
    /// same array again. MEASURED at [`MAX_ELEMENT_INSTANCES`] (300 000) by
    /// `tests/element_cap_bench.rs`'s unmapped arm, that discarded scan is
    /// **93.6 ms per frame** (cold 552.3 ms vs warm 458.7 ms, floors) — on EXACTLY
    /// the unmapped "automagic" path the element ladder exists for, while the
    /// published 0.32 µs/element slope was measured on the cheaper name-mapped
    /// half, which never infers. So an unseen vendor's `/plan`-shaped topic paid
    /// double per frame, silently.
    ///
    /// Memoizing collapses that to ONE inference per `(input, schema layout)`: the
    /// same bench measures the warm unmapped arm at 458.7 ms against the
    /// name-mapped `nav_msgs/Path` arm's 455.1 ms — the same work, as it should
    /// be. (The FIRST frame still scans twice — one-time, amortized to nothing.)
    ///
    /// # What is and is not memoized
    ///
    /// Only a [`KindStability::Stable`] answer, i.e. one decided by declaration
    /// shape or by an element array that really yielded geometry. An inference
    /// that fell PAST the element rung because this frame's array was empty /
    /// opaque / geometry-less is re-run next frame — freezing a topic on an idle
    /// `/plan`'s empty first frame is the frame-dependent-layout failure mode, and
    /// nothing here is worth reintroducing it for.
    ///
    /// **What that costs, precisely** (not "nothing"): such a topic re-runs the
    /// ladder on every frame, exactly as it did before the memo. For an empty or
    /// array-less value that is trivial. For the third sub-case — a POPULATED
    /// array whose elements carry no geometry (a `JointTrajectoryPoint[]`, a
    /// `DiagnosticStatus[]`, a `string[]`) —
    /// [`crate::archetype::scan_element_arrays`] still walks up to
    /// [`MAX_ELEMENT_INSTANCES`] elements before concluding, per frame, forever.
    /// The memo does not improve that class and does not claim to; it is unchanged,
    /// not introduced. Making it cacheable needs evidence that the elements'
    /// geometry-lessness is a schema fact, which an emptiness-contingent scan
    /// cannot supply.
    ///
    /// # A warm memo also answers frames the COLD ladder would leave undecided
    ///
    /// The memo is consulted BEFORE stability, and that asymmetry is deliberate.
    /// [`KindStability`] governs what may be WRITTEN, not what may be SERVED: once
    /// a topic has been classified from a frame that carried elements, a later
    /// frame whose array is EMPTY is answered from the memo — it does not re-open
    /// the question. So an unmapped `/plan`-shaped topic that goes idle keeps
    /// drawing nothing under its `Path3D` archetype instead of flipping to the
    /// `Scalars` its remaining numbers would infer.
    ///
    /// That is the intended reading of "a topic's layout is resolved ONCE from its
    /// first decodable frame", and it is what the NAME-MAPPED half already does —
    /// an idle `nav_msgs/Path` stays `Path3D` forever and never plots its `header`
    /// numbers. The alternative flips a topic between a 3D view and a plot view
    /// every time its plan clears. `ElementsUndecided` exists to stop an idle FIRST
    /// frame from freezing the WRONG archetype in (idle → busy); it is not a
    /// promise to unfreeze the right one on the way out. Pinned, on rendered
    /// output rather than by assertion of intent, by
    /// `sink_dispatch_test::a_warmed_topic_that_goes_idle_keeps_its_archetype_instead_of_flipping_to_plots`.
    ///
    /// **Interaction worth knowing:** `cerulion-vizd` picks a topic's BLUEPRINT
    /// from its own one-shot attach peek (`resolve_from_frame`), which runs the
    /// UNMEMOIZED ladder over a single drained frame. If that peek lands on an idle
    /// frame it reads `Scalars` and lays out a plot view, while the sink — warmed
    /// by a populated frame — renders 3D geometry. The layout and the drawing then
    /// disagree. The disagreement predates the memo (the peek and the sink were
    /// always separate classifications of separate frames), but the memo WIDENS
    /// it: before, the sink re-classified every frame, so it agreed with an
    /// idle-peek blueprint on exactly the frames that were themselves idle; now
    /// its first populated frame sticks and it disagrees on every frame after.
    /// The peek is not aligned with the sink's memo.
    ///
    /// **Residuals, stated plainly.** The memo trusts that one input's element
    /// FRAMING does not change while its layout identity stays the same:
    ///
    /// - A producer that restarts into a binary which frames the SAME schema
    ///   differently (the paired-rollout skew) keeps the pre-restart
    ///   archetype. The hash cannot see it — the rmw convergence changes element bytes with no
    ///   hash change — but it degrades LOUDLY when it bites: the render arm's own
    ///   scan finds the array opaque and takes the named
    ///   `UndecodableElementBytes` field-dump reason, never a wrong drawing.
    /// - A topic DETACHED and re-attached (possibly to another robot's same-named
    ///   topic) keeps its entry, because nothing tears per-input state down. Any
    ///   real layout difference changes the hash and re-infers; an identical layout
    ///   means an identical shape, so the memo is right. What is left is the same
    ///   framing-skew sliver as above.
    pub fn archetype_for(
        &mut self,
        input_name: &str,
        schema_hash: u64,
        fv: &FrameValue,
    ) -> ArchetypeKind {
        // CONTENT then NAME, both BEFORE the memo — see
        // `classify_content_or_name` for why the video rung must sit here and not
        // inside the memoized half (it would be frozen as `Scalars` on an unmapped
        // camera schema, and unreachable on a name-mapped one).
        if let Some(kind) = classify_content_or_name(fv) {
            return kind;
        }
        if let Some(hit) = self.inferred_kind.get(input_name) {
            if hit.schema_hash == schema_hash {
                return hit.kind;
            }
        }
        self.inference_runs += 1;
        let (kind, stability) = infer_archetype_with_stability(fv);
        match stability {
            KindStability::Stable => {
                self.inferred_kind.insert(
                    input_name.to_string(),
                    InferredArchetype { schema_hash, kind },
                );
            }
            // Reaching here having MISSED means either nothing was memoized for
            // this input or what was memoized belongs to a schema this input no
            // longer carries. Drop it, so ON THE INFERENCE PATH the memo holds the
            // current answer or nothing. (An input that switches to a NAME-MAPPED
            // schema keeps its old entry — the early return above never touches
            // the map. That is deliberate: probing it would put a map lookup on
            // every frame of every mapped topic to tidy a value nothing reads,
            // since the memo is only ever consulted after a `classify_schema`
            // miss. `cached_archetype` documents it.)
            KindStability::ElementsUndecided => {
                self.inferred_kind.remove(input_name);
            }
        }
        kind
    }

    /// How many times the shape-inference ladder RAN this run.
    ///
    /// Principle #3 observability and the memo's oracle: N frames of one unmapped
    /// element topic must leave this at 1, while a name-mapped topic leaves it at
    /// 0 (the table never infers).
    pub fn inference_runs(&self) -> u64 {
        self.inference_runs
    }

    /// The archetype MEMOIZED for `input_name`, if any (observability / test
    /// seam) — `None` for an input that has only ever carried a name-mapped
    /// schema (the table is the answer, so nothing is memoized) and for one whose
    /// latest inference was [`KindStability::ElementsUndecided`].
    ///
    /// It reports what the memo HOLDS, which is what
    /// [`SinkState::archetype_for`]'s inference path would reuse — and that path
    /// re-checks the schema before reusing it. One case is therefore visible here
    /// but never live: an input that carried an unmapped schema and later carries
    /// a NAME-MAPPED one keeps its old entry, because the table answers first and
    /// the memo is never consulted again for it.
    pub fn cached_archetype(&self, input_name: &str) -> Option<ArchetypeKind> {
        self.inferred_kind.get(input_name).map(|c| c.kind)
    }
}

/// Decode one raw wire `frame` (WireHeader included — exactly what a
/// subscriber delivers) and RENDER its mapped Rerun archetype under the route
/// for `input_name`. Best-effort: an undecodable frame is warned once (per
/// hash / schema) and skipped, never a failure.
///
/// This always renders (no coalescing) — it is the immediate-render entry the
/// worker's per-tick drain uses to FLUSH the single staged replacing-kind frame
/// (and the direct entry the dispatch tests exercise). The drain loop routes each
/// drained frame through [`dispatch_or_stage`] instead, which stages the newest
/// replacing-kind frame and renders every other frame here.
/// Whether this frame's rate-gate decision has already been made.
///
/// The gate must be consulted EXACTLY ONCE per frame — it mutates (it moves the
/// admitted-stamp anchor and counts refusals), so a second consult on the same
/// frame would measure a zero-length interval against the anchor it just set and
/// refuse a frame that was already admitted. There are two places it can happen:
/// before the walk (cheap, but only possible once the topic is a remembered plot
/// topic) and at the curated-series choke point (always possible). This carries
/// which one ran.
/// The verdict is split in two, because a `ScalarsWithText` frame has TWO
/// metered halves and they can disagree: the series may be refused while the dump
/// is re-rendered by its anti-freeze floor
/// ([`crate::plot_rate::DUMP_REFRESH_FRAME_FLOOR`]). A frame reaching the render
/// arm is one that at least one half wanted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PlotAdmission {
    /// The pre-walk gate ran and DECIDED both halves for this frame; neither may
    /// be consulted again. A frame both halves refuse is dropped in
    /// `classify_and_route` and never reaches a render arm.
    Decided {
        /// Whether this frame's SERIES may be plotted.
        series: bool,
        /// Whether this frame re-renders its field dump (`ScalarsWithText` only;
        /// meaningless, and always false, for a topic that renders no dump).
        dump: bool,
    },
    /// Nothing has consulted the gates for this frame — the render arm decides.
    Undecided,
}

/// What a render arm wants plotted — the ONE place a curated numeric
/// harvest is turned into `log_scalar` calls.
///
/// A gate inside the `Scalars` arm alone covers 2 of the 6 arms that
/// log the SAME curated harvest; `Transform3D*`/`Point3D*` (via
/// `log_spatial_siblings`) and `Imu`/`Odometry`/`SportModeState` would be ungated, and
/// they are shape-INFERRED too — so a wide joint bank sitting BESIDE a pose
/// (`{Pose pose; MotorState[20] motors}`) is the exact composite the gate is
/// built for and the gate would be inert on it. Routing every one of them through
/// [`log_curated_series`] means a SEVENTH arm inherits the gate by construction
/// rather than by remembering.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CuratedSeries {
    /// The whole frame's numeric harvest (`Scalars` / `ScalarsWithText`).
    Whole,
    /// Everything BESIDE the spatial primitive this frame already drew.
    SpatialSiblings,
    /// An `Imu`'s accel/gyro bank.
    Imu,
    /// An `Odometry`'s twist bank.
    OdometryTwist,
    /// A `SportModeState`'s selected telemetry.
    SportMode,
}

impl CuratedSeries {
    /// Run the harvest this source names. Called ONLY after the gate admits, so a
    /// refused frame pays none of it.
    fn harvest(self, fv: &FrameValue) -> (Vec<(String, f64)>, Vec<SkippedSeries>) {
        match self {
            Self::Whole => scalar_samples_with_skips(fv),
            Self::SpatialSiblings => spatial_sibling_series(fv),
            // The three NAMED extractors read exactly the fields they declare, so
            // they skip nothing by construction (the same reasoning
            // `scalar_samples_with_skips` applies to `Twist`/`Joy`).
            Self::Imu => (imu_scalars(fv), Vec::new()),
            Self::OdometryTwist => (odometry_twist_scalars(fv), Vec::new()),
            Self::SportMode => (sportmode_scalars(fv), Vec::new()),
        }
    }
}

/// THE curated-series choke point — gate, then harvest, then log.
///
/// Every arm that turns a numeric harvest into plot series goes through here, so
/// the per-topic sample budget ([`crate::plot_rate`]) covers all of them and a new
/// arm cannot silently escape it. The gate runs BEFORE the harvest, so a refused
/// frame costs neither the walk nor the logs.
///
/// Geometry is deliberately NOT routed through this: a pose, an attitude, a cloud
/// is WHOLE STATE under rerun's latest-at semantics, so withholding one leaves the
/// robot rendered at a stale position. Only the SERIES half is metered, which is
/// also why a `Transform3DWithScalars` topic keeps drawing its transform at full
/// rate while its telemetry thins.
#[allow(clippy::too_many_arguments)]
fn log_curated_series(
    rec: &RecordingStream,
    input_name: &str,
    entity: &str,
    timestamp_ns: u64,
    fv: &FrameValue,
    state: &mut SinkState,
    admission: PlotAdmission,
    source: CuratedSeries,
    kind: ArchetypeKind,
) {
    if !state.admit_curated_series(input_name, timestamp_ns, admission) {
        return;
    }
    let (samples, skipped) = source.harvest(fv);
    state.note_plot_series(input_name, samples.len());
    for (name, value) in samples {
        log_scalar(rec, entity, &name, timestamp_ns, value);
    }
    // The report's tail depends on whether THIS topic renders a dump,
    // so it is handed the classified kind rather than guessing from the harvest.
    report_skipped_series(input_name, &fv.schema_name, &skipped, kind, state);
}

pub fn dispatch_frame(
    rec: &RecordingStream,
    walker: &FrameWalker,
    input_name: &str,
    frame: &[u8],
    state: &mut SinkState,
) {
    if let Some(c) = classify_and_route(walker, input_name, frame, state) {
        state.submit_bound_model_frame(rec, input_name, c.timestamp_ns, &c.fv);
        render_classified(
            rec,
            input_name,
            &c.route,
            c.timestamp_ns,
            &c.fv,
            c.plan,
            state,
            c.admission,
        );
    }
}

/// Drain-loop seam: walk + classify `frame` ONCE, then either STAGE it (a
/// [`coalesces`] replacing-kind frame — latest wins, the already-owned `Vec` is
/// moved with no extra copy, and a replaced earlier frame counts toward
/// `coalesced`) or RENDER it immediately (a per-sample kind — every frame must
/// draw). The node flushes the staged frame after the drain via
/// [`dispatch_frame`] (one extra walk for that single frame per input per tick).
pub fn dispatch_or_stage(
    rec: &RecordingStream,
    walker: &FrameWalker,
    input_name: &str,
    frame: Vec<u8>,
    state: &mut SinkState,
    staged: &mut Option<Vec<u8>>,
    coalesced: &mut u64,
) {
    // Scope the walk so the `FrameValue` borrow of `frame` is released before a
    // staged frame is moved. Returns whether this frame should be staged.
    // Stage S3 (vizd render): the span from "the worker picked this frame
    // up" to "rerun has it". On a NON-coalescing kind (which includes the headline
    // `VideoStream`) it ENCLOSES the walk, the classification, the OpenH264 decode
    // and the `rec.log` hand-off, so a slow render is attributable to decode vs
    // everything else without a second run. On a COALESCING kind the frame is
    // STAGED here and rendered later by `dispatch_frame`, which emits no line of its
    // own — so such a line's `render_us` covers walk + classify ONLY. The emitted
    // `staged` field is what tells the two apart.
    //
    // The header re-parse for the sequence happens ONLY when the probe is on
    // (`classify_and_route` parses its own copy regardless) — matching the other
    // three probe sites, so "no header re-parse when disabled" is true everywhere.
    let probe_on = cerulion_core::lat_probe::probe_enabled();
    // Clear FIRST, unconditionally, BEFORE the walk: the decode timer below writes
    // on EVERY video unit while the probe is on, but only SAMPLED frames read it.
    // Clearing at the writer's door is what makes the value a line reports its own
    // — see `SinkState::probe_decode_us`.
    if probe_on {
        state.probe_decode_us = 0;
    }
    let probe_seq = probe_on
        .then(|| cerulion_core::wire::WireHeader::read_from_buf(&frame).map(|h| h.sequence))
        .flatten()
        .filter(|seq| cerulion_core::lat_probe::should_sample(*seq));
    let probe_t0 = probe_seq.map(|_| Instant::now());
    let should_stage = {
        let Some(c) = classify_and_route(walker, input_name, &frame, state) else {
            return;
        };
        // Staging is keyed on what the LADDER elected, never on the operator's
        // choice: whether the latest frame REPLACES an earlier one is a property
        // of the message (a cloud replaces, a text line accumulates), and how the
        // operator chose to look at it does not change that.
        if coalesces(c.kind)
            && !(state.is_bound_model_input(input_name)
                && c.fv.schema_name == "unitree_go/LowState")
        {
            true
        } else {
            state.submit_bound_model_frame(rec, input_name, c.timestamp_ns, &c.fv);
            render_classified(
                rec,
                input_name,
                &c.route,
                c.timestamp_ns,
                &c.fv,
                c.plan,
                state,
                c.admission,
            );
            false
        }
    };
    if let (Some(seq), Some(t0)) = (probe_seq, probe_t0) {
        // A plain read: the field was cleared at the top of THIS call, so whatever is
        // here was written by THIS frame's decode or by nothing at all.
        let decode_us = state.probe_decode_us;
        tracing::info!(
            stage = "s3_vizd_render",
            input = input_name,
            seq,
            bytes = frame.len(),
            staged = should_stage,
            decode_us,
            render_us = t0.elapsed().as_micros() as u64,
            t_render_done_ns = cerulion_core::lat_probe::wall_ns() as u64,
            "latency probe stage"
        );
    }
    if should_stage {
        // Latest wins: a previously-staged frame in this tick's batch is
        // overwritten before it could ever display, so it is coalesced away.
        if staged.is_some() {
            *coalesced += 1;
        }
        *staged = Some(frame);
    }
}

/// Walk `frame` by its schema hash, resolve the render route, and classify the
/// archetype — the shared front half of [`dispatch_frame`] / [`dispatch_or_stage`].
/// `None` on an undecodable frame (warned once per hash / input and skipped).
fn classify_and_route<'a>(
    walker: &FrameWalker,
    input_name: &str,
    frame: &'a [u8],
    state: &mut SinkState,
) -> Option<Classified<'a>> {
    // When the skeleton is active this re-parents the lidar cloud onto
    // the URDF `radar` link; otherwise it is exactly `route_for_input`.
    let route = state.route_for(input_name);
    // ONE header parse for both facts it carries here: the sender's wire stamp IS
    // the timeline (held-frame-safe: it is baked into the frame the subscriber
    // delivered), and the `schema_hash` is the LAYOUT identity the
    // inference memo is keyed on. An unparseable header cannot reach the memo at
    // all — `walk_by_hash` below reads the same header and bails first.
    let header = WireHeader::read_from_buf(frame);
    let timestamp_ns = header.as_ref().map(|h| h.timestamp_ns).unwrap_or(0);
    let schema_hash = header.as_ref().map(|h| h.schema_hash).unwrap_or(0);

    // A KNOWN fully-gated plot topic whose stamp the rate gate refuses
    // is dropped HERE, on the header alone. Everything below — the walk, the
    // classification ladder (which re-runs per frame for exactly this class of
    // topic, see `plot_kind_hint`), and the render harvest — is skipped.
    //
    // BOTH halves are decided here, and the frame is dropped only when
    // BOTH refuse it. A `ScalarsWithText` topic whose series are refused may still
    // owe a dump render (the anti-freeze floor), and that frame has to be walked.
    let admission = match state.decide_plot_frame_before_walk(input_name, schema_hash, timestamp_ns)
    {
        Some((false, false))
            if !(state.is_bound_model_input(input_name)
                && walker.schema_name_for_hash(schema_hash) == Some("unitree_go/LowState")) =>
        {
            state.pre_walk_drops += 1;
            return None;
        }
        // The gates ran: they must not run twice for one frame.
        Some((series, dump)) => PlotAdmission::Decided { series, dump },
        None => PlotAdmission::Undecided,
    };

    let fv = match walker.walk_by_hash(frame) {
        Ok(fv) => {
            // A frame that DID resolve closes any open undecodable
            // regime on this topic and re-arms it. Reachable in production, not
            // theoretical: a `SwapWalker` seeds a robot's `.msg` closure mid-run,
            // so a topic that could not be decoded a moment ago starts decoding.
            // A healthy desk holds an empty map and pays one `is_empty()`.
            if !state.unknown_hash.is_empty() {
                let topic = log_topic(input_name);
                if let Some(latch) = state.unknown_hash.get_mut(&topic) {
                    report_schema_hash_resolved(latch, DiagnosisVantage::Render, &topic);
                }
            }
            fv
        }
        Err(WalkError::UnknownSchemaHash(hash)) => {
            // No schema in the walker's set for this hash — it cannot
            // be decoded, so it cannot be visualized. Reported through the ONE
            // shared unknown-hash diagnostic (loud head, decade re-announcements
            // carrying the running total, `debug!` in between) rather than the
            // old warn-once-per-hash-forever, which left an operator who
            // missed the line with nothing.
            //
            // The candidate name is `None` here, and that is a real limit of
            // this vantage rather than an omission: the wire carries a hash and
            // nothing else, and the render worker receives only frames plus a
            // `FrameWalker` (`VizMsg` has no topic→type channel), so the sink
            // genuinely cannot name the type. The NAMED half — version skew vs
            // never-compiled — is reported by `cerulion-vizd`, which holds the
            // catalog / attach-pinned type name and puts the verdict on the wire
            // for Studio.
            //
            // That split is exactly why the line
            // declares `vantage=render` and why the `Unidentified` wording claims
            // only what THIS vantage knows. Both reporters run in ONE process
            // (vizd spawns the worker in-process and pushes it the same frames),
            // so on an unmapped remote-attach type — `unitree_go/LowState`,
            // `control_msgs/PidState`, the moveit family — the resolver names the
            // type on one line while this one used to say "nothing names the
            // type" on the next: false, and false about the world rather than
            // about this observer. `log_topic` likewise restores the absolute
            // topic so both lines key one grep.
            let diagnosis = diagnose_unknown_hash_with_walker(walker, hash, None);
            let topic = log_topic(input_name);
            report_unknown_schema_hash(
                state.unknown_hash.entry(topic.clone()).or_default(),
                DiagnosisVantage::Render,
                &topic,
                &diagnosis,
            );
            return None;
        }
        Err(e) => {
            // A structural decode failure (truncated frame, etc.) — loud once
            // per input (keyed by input name, since we have no schema yet).
            if state.decode_warn.insert(format!("hash-walk::{input_name}")) {
                tracing::warn!(error = %e, input = route_key_topic(input_name), "cerulion_viz: frame walk failed — skipping");
            }
            return None;
        }
    };

    // CONTENT-gated video first, then the name table, then STRUCTURAL
    // inference — the ladder `classify_frame` documents. The sink MEMOIZES the
    // inference half per input (its element rung discards a cap-governed scan the
    // render arm then repeats, ~94 ms of pure waste per frame at the 300 000
    // ceiling), and `archetype_for` composes the two: the content and name rungs
    // answer BEFORE the memo is consulted, so a video topic can never be frozen
    // into the `Scalars` its shape would infer. See `SinkState::archetype_for`.
    let kind = state.archetype_for(input_name, schema_hash, &fv);
    // Remember a FULLY-gated plot topic so the next refused frame is
    // dropped before the walk. `ScalarsWithText` qualifies too, carrying
    // a flag so the pre-walk decision knows to ask its dump gate as well — its
    // dump is metered now, so a frame both halves refuse really is a skipped
    // frame. Recorded on every classification (cheap, and it self-corrects if the
    // topic's shape stops classifying as a gated plot kind).
    match kind {
        ArchetypeKind::Scalars => {
            state
                .plot_kind_hint
                .insert(input_name.to_string(), (schema_hash, false));
        }
        ArchetypeKind::ScalarsWithText => {
            state
                .plot_kind_hint
                .insert(input_name.to_string(), (schema_hash, true));
        }
        _ => {
            state.plot_kind_hint.remove(input_name);
        }
    }
    let route = reconcile_tf_route(input_name, route, kind, &fv.schema_name, state);
    // The operator's choice, applied to what the ladder just elected.
    // `Auto` (the default, and every topic nobody touched) resolves to exactly
    // the elected kind with no overlay, so this line is behaviour-neutral until
    // somebody sets one.
    let plan = resolve_render_plan(kind, state.representation_for(input_name));
    Some(Classified {
        route,
        timestamp_ns,
        fv,
        kind,
        plan,
        admission,
    })
}

/// One walked + classified frame, ready to render — [`classify_and_route`]'s
/// answer.
///
/// This is a struct rather than the tuple it was: the render now needs
/// BOTH the kind the ladder elected (which decides whether the frame coalesces —
/// a property of the message, not of how the operator chose to look at it) and
/// the [`RenderPlan`] that choice resolves to.
struct Classified<'a> {
    /// The resolved render route (entity, staticness, frame).
    route: InputRoute,
    /// The sender's wire stamp — the timeline.
    timestamp_ns: u64,
    /// The decoded frame.
    fv: FrameValue<'a>,
    /// What the LADDER elected, before any override.
    kind: ArchetypeKind,
    /// What actually renders, after the operator's choice.
    plan: RenderPlan,
    /// The rate gates' verdicts for this frame, if they already ran.
    admission: PlotAdmission,
}

/// Record that `input_name` posed [`ROBOT_ROOT`], and WARN —
/// once — as soon as a SECOND topic does.
///
/// [`ROBOT_ROOT`] is a single entity. Every odom-named input writes its own
/// `Transform3D` there, so two of them (the Go2 ships FOUR — `/uslam/frontend/odom`,
/// `/uslam/localization/odom`, `/lio_sam_ros2/mapping/odometry`,
/// `/utlidar/robot_odom`) make the URDF skeleton — and, with a URDF loaded, the
/// radar-framed cloud posed off it — snap between two localization estimates every
/// frame. That is the same silent wrong SPATIAL answer per-topic entities fixed one
/// level down, and arbitrating it automatically would mean guessing which estimate
/// the operator trusts. So it stays a fight, but a LOUD one: the fix is to attach
/// only one, or rename the others.
fn note_robot_root_elector(input_name: &str, state: &mut SinkState) {
    let topic = route_key_topic(input_name).to_string();
    if !state.robot_root_electors.insert(topic) || state.robot_root_electors.len() < 2 {
        return;
    }
    let electors: Vec<&str> = state
        .robot_root_electors
        .iter()
        .map(String::as_str)
        .collect();
    tracing::warn!(
        electors = %electors.join(", "),
        entity = ROBOT_ROOT,
        "cerulion_viz: more than one odom-named topic is posing the robot root, so the URDF \
         skeleton (and any cloud posed off it) snaps between their estimates every frame — \
         each overwrites the other on one entity; attach only one of them, or rename the \
         others so their last segment is not odom/robot_odom/odometry"
    );
}

/// Where a tf-NAMED topic renders once its frames turn out NOT to be a
/// `tf2_msgs/TFMessage` — the ordinary mechanical/override entity.
///
/// Shared with the daemon ([`reported_entity_for`]) so the entity a caller is TOLD
/// about is the entity the sink logs at.
fn reconciled_tf_entity(input_name: &str) -> String {
    match route_key_override(input_name) {
        Some(o) => entity_path_for_route_key(o),
        None => entity_path_for_route_key(route_key_topic(input_name).trim_matches('/')),
    }
}

/// The entity to REPORT for a route key, given what (if anything) the topic has
/// been observed to render as.
///
/// [`route_for_input`] answers [`WORLD_ROOT`] for a `tf`/`tf_static`-named topic
/// because a TFMessage renders at no route entity at all. Once a frame proves the
/// topic is NOT a TFMessage, `reconcile_tf_route` moves it to its own entity —
/// and the daemon must report that same value, or `discover`/`list`/`attach` name
/// a location nothing is ever logged at (and an `entity` override fed back from
/// such a report would not round-trip).
///
/// `None` = nothing resolved yet, so the name-derived answer stands.
pub fn reported_entity_for(route_key: &str, kind: Option<ArchetypeKind>) -> String {
    match kind {
        Some(k) if !matches!(k, ArchetypeKind::Transforms) => non_tf_entity_for(route_key),
        _ => route_for_input(route_key).entity,
    }
}

/// The entity a route key resolves to for a payload that is NOT a
/// `tf2_msgs/TFMessage` — i.e. [`reported_entity_for`] with the schema question
/// already answered.
///
/// An IDENTITY for every topic except a `tf`/`tf_static`-named one, whose route
/// arm answers the bare viz root only because a TFMessage renders at no route
/// entity at all. Used where the payload's kind is not known but the tf arm's
/// root answer would be actively misleading — notably the daemon's
/// entity-override conflict check, where collapsing every override to `world`
/// made two DIFFERENT overrides compare equal and a conflicting re-point was
/// accepted silently.
pub fn non_tf_entity_for(route_key: &str) -> String {
    let route = route_for_input(route_key);
    if route.entity == WORLD_ROOT {
        reconciled_tf_entity(route_key)
    } else {
        route.entity
    }
}

/// Give the SCHEMA a vote on the `tf`/`tf_static` NAME arm.
///
/// [`route_for_input`] answers [`WORLD_ROOT`] for a topic whose last segment is
/// `tf`/`tf_static`, and that answer is only correct for a `tf2_msgs/TFMessage`:
/// [`dispatch_transforms`] ignores `route.entity` entirely and logs each
/// transform at its own child-frame entity. EVERY other archetype arm renders AT
/// `route.entity` — and `assign_coordinate_frame` runs before all of them — so a
/// tf-NAMED topic carrying some other schema would write its geometry, and a
/// `CoordinateFrame`, at the bare viz ROOT, whose transform composes onto the
/// whole scene. (A vendor or multi-robot stack publishing `/robot1/tf` with a
/// non-TFMessage type is unusual but not impossible, and `entity_path_for_route_key`
/// already guarantees "a degenerate topic can never claim the TF tree's root
/// entity" — the tf arm was the one remaining path that could.)
///
/// So: a tf-named topic whose frame is NOT classified [`ArchetypeKind::Transforms`]
/// falls back to the ordinary mechanical per-topic entity, LOUDLY once per input.
/// `is_static` is left alone — only [`dispatch_transforms`] reads it, and that arm
/// is unreachable here by construction.
fn reconcile_tf_route(
    input_name: &str,
    route: InputRoute,
    kind: ArchetypeKind,
    schema_name: &str,
    state: &mut SinkState,
) -> InputRoute {
    if route.entity != WORLD_ROOT || matches!(kind, ArchetypeKind::Transforms) {
        return route;
    }
    let topic = route_key_topic(input_name);
    // The override still names the ENTITY (the tf arm ignores it only because a
    // TFMessage has no route entity at all — this frame is not one). Reported by
    // the daemon through the SAME helper, so `discover`/`list`/`attach` cannot
    // disagree with where the sink actually renders.
    let entity = reconciled_tf_entity(input_name);
    if state.decode_warn.insert(format!("tf-schema::{input_name}")) {
        tracing::warn!(
            input = topic,
            schema = schema_name,
            entity = %entity,
            "cerulion_viz: this topic is NAMED tf/tf_static but does not carry \
             tf2_msgs/TFMessage, so it is not a transform tree — rendering it at its own \
             entity instead of at the viz root `world` (logging it at the root would \
             re-pose every other topic in the scene)"
        );
    }
    InputRoute { entity, ..route }
}

/// RESOLVE the coordinate frame that poses this input's entity — the
/// name only. Assignment is the caller's, because rerun 0.34 has TWO mechanisms
/// and which one is correct depends on what the archetype logs at that entity:
///
/// - a DATA payload (`Points3D`, `LineStrips3D`, images, …) is posed by a
///   [`rerun::CoordinateFrame`] ([`emit_frame_at`]);
/// - a TRANSFORM payload (`Odometry`, `Pose`, `Imu`, and the robot root) is posed
///   by the `Transform3D`'s own `parent_frame` component
///   ([`crate::archetype::log_transform3d_in_frame`]) — a `CoordinateFrame` there
///   is inert for the chain and actively wrong for the entity's own geometry.
///
/// [`ArchetypeKind::payload_is_transform`] is the switch, applied in
/// [`render_classified`].
///
/// Resolution order:
/// 1. an explicit [`InputRoute::frame`] (today only the URDF skeleton's radar
///    re-parent) — configuration beats data;
/// 2. the message's own `frame_id`, in either wire shape
///    ([`crate::tf::frame_id_of`]), resolved through the run's
///    [`FrameRegistry`];
/// 3. nothing.
///
/// A `frame_id` that is PRESENT but unresolvable, or ABSENT entirely (58 of the
/// Go2's 75 topics), resolves to NOTHING — see [`FrameRegistry::resolve`] for why
/// withholding beats naming the fallback entity. The entity then renders under its
/// own implicit `tf#/world/<topic>` frame, which is identity-connected to
/// `tf#/world`, i.e. at the world origin. That is a deliberate semantic change —
/// such a topic no longer inherits the robot base pose the old
/// `world/odom/base/...` nesting gave it for free — and none of the Go2's
/// frame-less topics renders spatially today.
///
/// An unresolvable frame warns ONCE per input: nothing poses the topic, so that
/// warn is the only signal its data is not localized.
fn resolve_route_frame(
    input_name: &str,
    route: &InputRoute,
    fv: &FrameValue,
    state: &mut SinkState,
) -> Option<String> {
    if let Some(configured) = &route.frame {
        return Some(configured.clone());
    }
    let frame_id = frame_id_of(fv)?;
    let resolved = state.frames.resolve(frame_id);
    if resolved.is_none() && state.unresolved_frame_inputs.insert(input_name.to_string()) {
        tracing::warn!(
            input = route_key_topic(input_name),
            frame_id = frame_id,
            entity = %route.entity,
            "cerulion_viz: this topic's messages are stamped with a coordinate frame \
             that the transform tree cannot place (it is not a known frame alias and \
             no /tf transform for it has been seen this run), so its data is rendered \
             UNPOSED at the world origin rather than at a fabricated pose (if it was \
             posed by an earlier message it is un-posed now, not left at the stale \
             mount); publish /tf (or /tf_static) for this frame to place it"
        );
    }
    resolved
}

/// The PARENT-FRAME argument for a transform-payload arm: the resolved frame, or —
/// when nothing resolves — the entity's own path-parent frame, named EXPLICITLY.
///
/// Explicit rather than `None`, because a rerun archetype does not serialize its
/// `None` fields: a later `Transform3D` logged without a parent frame inherits the
/// previous row's under latest-at, so omission would leave exactly the stale pose
/// the withhold rule exists to prevent.
fn transform_parent_frame(entity: &str, resolved: &Option<String>) -> Option<String> {
    match resolved {
        Some(f) => Some(f.clone()),
        None => implicit_parent_frame_of(entity),
    }
}

/// Emit a [`rerun::CoordinateFrame`] at `entity` when the resolved frame CHANGED
/// (or is new).
///
/// Change-triggered per ENTITY, so a topic with a constant `frame_id` pays one
/// chunk for the whole run while a topic whose frame genuinely varies per message
/// still re-points correctly. Keyed per entity rather than per input because the
/// cloud's rotating `sweep/{k}` sub-entities each need their own assignment.
///
/// `None` means "this message poses nothing". It is a NO-OP on an entity that was
/// never posed (no phantom frame declaration on the 58 Go2 topics that carry no
/// `frame_id`), and a CLEAR on one that was: the entity is re-pointed at its OWN
/// implicit frame — the identity-to-`world` frame it would have had with no
/// assignment at all — and the record dropped, so a later resolvable frame re-emits.
/// Without the clear, the previous (temporal, latest-at) assignment would keep
/// posing every subsequent message at a mount the current message never claimed.
fn emit_frame_at(
    rec: &RecordingStream,
    entity: &str,
    timestamp_ns: u64,
    frame: &Option<String>,
    state: &mut SinkState,
) {
    let Some(frame) = frame else {
        // CLEAR — but only if this entity actually holds an assignment.
        if state.frame_emitted.remove(entity).is_some() {
            log_coordinate_frame(rec, entity, timestamp_ns, &implicit_frame_of(entity));
        }
        return;
    };
    if state.frame_emitted.get(entity).map(String::as_str) == Some(frame.as_str()) {
        return;
    }
    state
        .frame_emitted
        .insert(entity.to_string(), frame.clone());
    log_coordinate_frame(rec, entity, timestamp_ns, frame);
}

/// Render an already-walked + classified frame to its Rerun archetype. Split
/// out of [`dispatch_frame`] so the drain loop can stage a replacing-kind frame
/// (rendering only the newest) without re-walking a non-staged frame.
#[allow(clippy::too_many_arguments)]
fn render_classified(
    rec: &RecordingStream,
    input_name: &str,
    route: &InputRoute,
    timestamp_ns: u64,
    fv: &FrameValue,
    plan: RenderPlan,
    state: &mut SinkState,
    admission: PlotAdmission,
) {
    // The FORCED dump, first — it is the whole rendering when the
    // operator suppressed the visual half, and it needs no pose (a document is
    // not geometry). `plan.dump` is false for every un-overridden topic and for
    // the kinds whose own arm already dumps, so this is inert until somebody
    // asks for it.
    if plan.dump && state.admit_forced_dump(input_name, timestamp_ns) {
        log_field_dump(rec, &route.entity, timestamp_ns, fv);
    }
    // A suppressed visual half logs no geometry, so it also resolves no frame:
    // returning here leaves `frame_emitted` untouched rather than posing an
    // entity nothing will draw at.
    let Some(kind) = plan.visual else {
        return;
    };
    // Pose this topic's entity, resolved from the message's own
    // `frame_id` (or a configured route frame). Done BEFORE the geometry so the
    // frame is in the store by the time the data lands.
    //
    // WHICH MECHANISM depends on what this archetype logs here: a
    // TRANSFORM payload is posed by its own `parent_frame` component and must NOT
    // get a `CoordinateFrame` (which would relocate its data while leaving the
    // chain path-parented — inert at best, scene-tearing at worst); a DATA payload
    // is posed by a `CoordinateFrame`. See `resolve_route_frame`.
    let resolved_frame = resolve_route_frame(input_name, route, fv, state);
    let payload_parent_frame = if kind.payload_is_transform() {
        transform_parent_frame(&route.entity, &resolved_frame)
    } else {
        emit_frame_at(rec, &route.entity, timestamp_ns, &resolved_frame, state);
        None
    };
    let payload_parent_frame = payload_parent_frame.as_deref();
    // Read the degradation counter BEFORE the arm runs, so the pair of
    // reads around it says whether this frame drew its archetype or dumped. One
    // site for all eight degradable arms: the alternative — a `note_render_native`
    // call at each success branch — is exactly the shape that ships a new arm with
    // no signal and therefore a permanently empty pane, which is the bug
    // this feature is allowed to trade against and must not re-create.
    let degradations_before = state.render_degradations();
    match kind {
        ArchetypeKind::Points3D => {
            let cloud = cloud_from_frame_value(fv);
            if cloud.skipped > 0 {
                tracing::warn!(
                    skipped = cloud.skipped,
                    kept = cloud.positions.len(),
                    input = route_key_topic(input_name),
                    "cerulion_viz: skipped malformed/out-of-bounds cloud points"
                );
            }
            // Rendered sweeps rotate across `{entity}/viz-sweep/{k}` sub-entities:
            // a rosette lidar's sparse SENSOR-frame sweeps would otherwise
            // REPLACE each other under latest-at semantics (a sparse jumping
            // patch); the ring keeps the last SWEEP_ACCUM_RING sweeps visible
            // together — viewer-side accumulation, zero data copies (see the
            // const; world-frame accumulation is not implemented).
            let sweep_entity = state.next_sweep_entity(&route.entity);
            // A sub-entity's frame does NOT inherit the parent's assignment —
            // rerun derives a child's implicit frame from the PATH string, so it
            // chains to `tf#<parent path>`, not to the frame the parent was
            // re-pointed at. The cloud's geometry lives HERE, so the assignment
            // has to be repeated here or the points render unposed.
            emit_frame_at(rec, &sweep_entity, timestamp_ns, &resolved_frame, state);
            log_points3d(rec, &sweep_entity, timestamp_ns, &cloud);
        }
        ArchetypeKind::Image => {
            // A COMPRESSED image has a JPEG blob Rerun renders natively (zero
            // decode). A raw image (or an inferred image-shape) carries
            // un-encoded pixels: a common 8-bit encoding decodes to a native
            // `rerun::Image`; an encoding not decoded here degrades to the
            // field dump rather than mis-rendering raw bytes.
            match fv.schema_name.as_str() {
                "sensor_msgs/CompressedImage" => match image_data_from_frame_value(fv) {
                    Some(data) if !data.is_empty() => {
                        // A non-empty frame heals any empty-frame regime (one
                        // recovery info carrying the suppressed count — the
                        // FieldsWarnLatch contract; None = never in a regime).
                        if let Some(latch) = state.empty_image.get_mut(input_name) {
                            if let Some(suppressed) = latch.on_decoded() {
                                tracing::info!(
                                    input = route_key_topic(input_name),
                                    suppressed_count = suppressed,
                                    "cerulion_viz: camera frames non-empty again — \
                                     empty-frame regime healed"
                                );
                            }
                        }
                        log_encoded_image(rec, &route.entity, timestamp_ns, data);
                    }
                    _ => {
                        // Empty (or missing) JPEG blob. LATCHED warn — the
                        // deleted camera sink warned per frame; a camera
                        // streaming empties must stay operator-visible at
                        // info level WITHOUT flooding: first-of-regime warn!,
                        // repeats debug! with a running count. Coalescing keys
                        // on bytes BEFORE this content inspection, so if the
                        // NEWEST frame of a drained batch happens to be an empty
                        // JPEG it is the one staged + warned here even when an
                        // earlier batch frame was good; the next tick's good
                        // frame heals the regime.
                        match state
                            .empty_image
                            .entry(input_name.to_string())
                            .or_default()
                            .on_inferred()
                        {
                            FieldsLogAction::WarnFirst => tracing::warn!(
                                input = route_key_topic(input_name),
                                "cerulion_viz: empty JPEG frame — skipping (repeats \
                                 log at debug until a non-empty frame arrives)"
                            ),
                            FieldsLogAction::DebugSuppressed { suppressed } => tracing::debug!(
                                input = route_key_topic(input_name),
                                suppressed,
                                "cerulion_viz: empty JPEG frame sustained (warn suppressed)"
                            ),
                        }
                    }
                },
                // Raw / inferred image-shape: decode rgb8/bgr8/mono8 natively,
                // else degrade to a dump (the remediation is a raw-pixel
                // encoding we support, not a classify_schema entry).
                _ => match raw_image_plan(fv) {
                    Some((w, h, enc)) => {
                        if let Some(data) = image_data_from_frame_value(fv) {
                            log_raw_image(rec, &route.entity, timestamp_ns, w, h, enc, data);
                        }
                    }
                    None => anyvalues_fallback(
                        rec,
                        input_name,
                        &route.entity,
                        timestamp_ns,
                        fv,
                        state,
                        kind,
                        FallbackReason::UnsupportedImageEncoding,
                    ),
                },
            }
        }
        ArchetypeKind::Transforms => {
            dispatch_transforms(rec, input_name, route, timestamp_ns, fv, state);
        }
        ArchetypeKind::Scalars | ArchetypeKind::ScalarsWithText => {
            // The twin ALSO logs the structured dump at the topic
            // entity, so what the numeric harvest cannot plot — declared text, or
            // the fields the curation withheld — is visible in the
            // `text_document` view its classification earned. NOT a degradation:
            // this is the second half of one render, which is why the kind is
            // decided at classify time rather than per frame.
            //
            // **It is METERED, and rides the SERIES verdict.** An exemption that
            // logs it before the gate, unconditionally, avoids metering it
            // freezing the document at the first admitted frame on a stopped-clock
            // publisher. That reasoning holds while `ScalarsWithText` means a
            // low-rate status message; classification also elects the same archetype for a
            // `/lowstate`-class firehose, where unconditional means a MEASURED
            // 2 150-byte document at ~500 Hz — ~1 MB/s of markdown into the
            // viewer, on the very topic the plot curation exists to rescue. So
            // the dump refreshes when the plot refreshes (one story on screen),
            // and the anti-freeze requirement is kept by `DumpRefreshGate`'s frame-count
            // floor instead of by an exemption. A topic whose series clear their
            // budget every frame still dumps every frame.
            //
            // ONE gate consult for the frame, shared by both halves: the gates
            // MUTATE, so `log_curated_series` below is handed the decision rather
            // than being allowed to re-ask it.
            let series_admitted = state.admit_curated_series(input_name, timestamp_ns, admission);
            if matches!(kind, ArchetypeKind::ScalarsWithText) {
                let dump = match admission {
                    PlotAdmission::Decided { dump, .. } => dump,
                    PlotAdmission::Undecided => state.admit_field_dump(input_name, series_admitted),
                };
                if dump {
                    log_field_dump(rec, &route.entity, timestamp_ns, fv);
                }
            }
            if !series_admitted {
                return;
            }
            let admission = PlotAdmission::Decided {
                series: true,
                dump: false,
            };
            // A plot topic is still NOT coalesced ([`coalesces`]) — a plot is a
            // stream of samples, not whole state, so keeping only a tick's newest
            // frame would throw real data away. What IS bounded is the SAMPLE
            // RATE, through the ONE choke point every curated harvest passes.
            log_curated_series(
                rec,
                input_name,
                &route.entity,
                timestamp_ns,
                fv,
                state,
                admission,
                CuratedSeries::Whole,
                kind,
            );
        }
        // The spatial primitive AND every number BESIDE it. The
        // `WithScalars` twins render IDENTICALLY — they differ only in the layout
        // mapping ([`crate::blueprint::views_for_archetype`] gives them a plot
        // view), so a message that carries telemetry gets a home for it while a
        // pure pose is never given an empty plot panel.
        ArchetypeKind::Transform3D | ArchetypeKind::Transform3DWithScalars => {
            log_pose_in_frame(rec, &route.entity, timestamp_ns, fv, payload_parent_frame);
            log_curated_series(
                rec,
                input_name,
                &route.entity,
                timestamp_ns,
                fv,
                state,
                admission,
                CuratedSeries::SpatialSiblings,
                kind,
            );
        }
        ArchetypeKind::Point3D | ArchetypeKind::Point3DWithScalars => {
            log_single_point(rec, &route.entity, timestamp_ns, fv);
            log_curated_series(
                rec,
                input_name,
                &route.entity,
                timestamp_ns,
                fv,
                state,
                admission,
                CuratedSeries::SpatialSiblings,
                kind,
            );
        }
        // The attitude is whole state and renders every frame; the
        // accel/gyro bank is metered like every other plot series.
        ArchetypeKind::Imu => {
            log_imu_geometry_in_frame(rec, &route.entity, timestamp_ns, fv, payload_parent_frame);
            log_curated_series(
                rec,
                input_name,
                &route.entity,
                timestamp_ns,
                fv,
                state,
                admission,
                CuratedSeries::Imu,
                kind,
            );
        }
        ArchetypeKind::Odometry => {
            // The leaf pose (a moving Transform3D) + twist scalar plots, as for
            // every odometry input.
            log_odometry_pose_in_frame(rec, &route.entity, timestamp_ns, fv, payload_parent_frame);
            log_curated_series(
                rec,
                input_name,
                &route.entity,
                timestamp_ns,
                fv,
                state,
                admission,
                CuratedSeries::OdometryTwist,
                kind,
            );
            // An odom-elected input ALSO poses the robot root so the
            // skeleton + cloud sit at the robot's WORLD pose (fixing the
            // walking-in-place offset). Deliberately NOT `world/odom/base`
            // (`crate::tf::BASE_ENTITY`): that entity is `/tf`'s documented
            // contract (see [`crate::tf`]), so driving it here would fight a
            // real `/tf` base transform. `world/tf-tree/robot` is the skeleton's own
            // root.
            if route.drives_robot_root {
                if let Some(parts) = pose_transform_parts(fv) {
                    // The robot root gets the SAME frame
                    // discipline as the topic's own entity — through the
                    // transform's own `parent_frame`, which is the ONLY component
                    // that re-parents a frame chain. The pose in an `Odometry`
                    // payload is expressed in the message's `frame_id`
                    // (canonically `odom`), and `world/tf-tree/robot`'s implicit
                    // parent is its PATH parent, so without this an odom-frame
                    // pose is applied as if it were a world-frame pose — off by
                    // the whole `world → odom` correction on any robot whose
                    // `/tf` publishes one.
                    //
                    // A `CoordinateFrame` here would be WORSE than nothing: it
                    // relocates only the root's own marker + bones while every
                    // skeleton link keeps chaining through the path-parented
                    // `Transform3D`, which TEARS the stick figure in two.
                    //
                    // Unresolvable ⇒ the path-parent frame NAMED EXPLICITLY, so
                    // the root is un-posed rather than left at a stale mount.
                    let root_frame = transform_parent_frame(ROBOT_ROOT, &resolved_frame);
                    log_transform3d_in_frame(
                        rec,
                        ROBOT_ROOT,
                        timestamp_ns,
                        &parts,
                        root_frame.as_deref(),
                    );
                    note_robot_root_elector(input_name, state);
                }
            }
        }
        ArchetypeKind::LaserScan => log_laserscan(rec, &route.entity, timestamp_ns, fv),
        ArchetypeKind::SportModeState => log_curated_series(
            rec,
            input_name,
            &route.entity,
            timestamp_ns,
            fv,
            state,
            admission,
            CuratedSeries::SportMode,
            kind,
        ),
        ArchetypeKind::TextLog => log_single_text(rec, &route.entity, timestamp_ns, fv),
        // An oriented detection box. A value that will not extract one
        // (a classify-time shape that this frame does not actually carry) degrades
        // to an inspectable dump rather than an empty view.
        // The SAME archetype now also carries an N-instance detection SET
        // (`vision_msgs/Detection3DArray`), tried in the classifier's own
        // precedence — the single box is its rung 2, the element array a later rung
        // — so classification and rendering can never disagree about which one this
        // frame is.
        ArchetypeKind::Boxes3D => {
            if !log_boxes3d_from_frame(rec, &route.entity, timestamp_ns, fv) {
                // The Detection3DArray element-array path first; if there is no
                // decodable element array either, the actual failure is a
                // single box whose geometry did not extract, so the fallback
                // names the BOX reason — not the element-array framing reason.
                render_element_array(
                    rec,
                    input_name,
                    route,
                    timestamp_ns,
                    fv,
                    state,
                    kind,
                    FallbackReason::UndecodableOccupancyOrBox,
                    &resolved_frame,
                );
            }
        }
        // An ordered path / unordered pose-or-point array. A name-mapped
        // Polygon (Path3D, unstamped elements) renders a polyline instead of
        // falling back to shape-only points — decided from the schema NAME inside
        // `render_element_array`, not from `kind`.
        ArchetypeKind::Path3D | ArchetypeKind::PoseArray3D => {
            render_element_array(
                rec,
                input_name,
                route,
                timestamp_ns,
                fv,
                state,
                kind,
                FallbackReason::UndecodableElementArray,
                &resolved_frame,
            );
        }
        // `/map` as a grayscale image. A grid whose `info` dimensions are
        // absent or whose cell buffer is short degrades to a dump — never a
        // mis-rendered map.
        ArchetypeKind::OccupancyGrid => {
            if !log_occupancy_from_frame(rec, &route.entity, timestamp_ns, fv) {
                anyvalues_fallback(
                    rec,
                    input_name,
                    &route.entity,
                    timestamp_ns,
                    fv,
                    state,
                    kind,
                    FallbackReason::UndecodableOccupancyOrBox,
                );
            }
        }
        ArchetypeKind::Skeleton => {
            // The URDF stick figure. Inert (GO2_URDF_PATH unset) ⇒ both
            // calls are no-ops and the frame falls through to a field dump so it
            // stays inspectable, and the dump companion gives the archetype a
            // `text_document` view, so that dump is now actually VISIBLE. This is
            // the SHIPPING state, not a corner: the last
            // `install_skeleton` caller is gone, so a skeleton-classified topic dumps on
            // every frame and, before the companion, rendered a blank pane.
            // The skeleton COALESCES (see [`coalesces`]), so a
            // ~500 Hz `/lowstate` firehose renders at most one 12-joint pose per
            // poll tick — bounded by the poll cadence, not a hardcoded rate. The
            // skeleton logs its OWN per-joint entity paths under `world/tf-tree/robot/...`,
            // so `route.entity` is unused here.
            if state.skeleton.is_active() {
                state.skeleton.log_statics_once(rec);
                state.skeleton.log_joint_angles(rec, timestamp_ns, fv);
            } else {
                anyvalues_fallback(
                    rec,
                    input_name,
                    &route.entity,
                    timestamp_ns,
                    fv,
                    state,
                    kind,
                    FallbackReason::UnmappedSchema,
                );
            }
        }
        // The desk-side H.264 path. The classifier already proved this
        // frame carries an Annex-B access unit, so a re-scan here cannot fail —
        // but if a future refactor ever let a non-video frame reach this arm, it
        // degrades to the inspectable dump rather than rendering nothing.
        ArchetypeKind::VideoStream => {
            let Some(payload) = crate::video::classify_h264_payload(fv) else {
                anyvalues_fallback(
                    rec,
                    input_name,
                    &route.entity,
                    timestamp_ns,
                    fv,
                    state,
                    kind,
                    FallbackReason::VideoRescanDisagreed,
                );
                return;
            };
            render_video_sample(
                rec,
                input_name,
                &route.entity,
                timestamp_ns,
                fv,
                &payload,
                state,
            );
        }
        // One entity per LIVE marker, plus the entity CLEARs a `DELETE`
        // / `DELETEALL` resolves to. The only stateful arm — see `render_marker_array`.
        ArchetypeKind::MarkerArray => {
            render_marker_array(
                rec,
                input_name,
                route,
                timestamp_ns,
                fv,
                state,
                &resolved_frame,
            );
        }
        ArchetypeKind::AnyValues => anyvalues_fallback(
            rec,
            input_name,
            &route.entity,
            timestamp_ns,
            fv,
            state,
            kind,
            FallbackReason::UnmappedSchema,
        ),
    }
    // The arm ran without reaching the dump funnel, so this frame drew
    // its archetype — the positive half of the signal, recorded once for every arm.
    //
    // `ArchetypeKind::AnyValues` cannot reach this: its arm IS the funnel, so it
    // always bumps the counter. That is the correct answer for it — the dump is its
    // render, so it has no native render to prove, and its companion is refused a
    // second time (structurally) by the layout's visual-half conjunct.
    if state.render_degradations() == degradations_before {
        state.note_render_native(input_name);
    }
}

/// Route one classified H.264 access unit to its rendition's
/// decoder and log the picture it produces, or account for the drop LOUDLY-ONCE.
///
/// **The decoder changed WHAT is logged, not how routing works.** The original video path handed the
/// access unit to the viewer as a [`rerun::VideoStream`] sample; measured, rerun
/// 0.34's native backend then decoded it in a spawned `ffmpeg` whose default
/// frame threading cost **567 ms** of lag on a 16-core desk (and MORE on a bigger
/// one). The desk now decodes in-process — 1 access unit in, 1 picture out,
/// 0.5 ms — and logs a plain [`rerun::Image`], which the viewer REPLACES on
/// arrival rather than scheduling onto a media timeline. That "latest frame now"
/// semantics is what a live robot camera wants and what the JPEG route always
/// had. See [`crate::video_decode`] for the full measurement table and for what
/// scrubbing this gives up.
///
/// Both drop reasons are real and neither is silent: an access unit that arrives
/// before its sub-stream's first keyframe cannot be decoded from (debug — this is
/// the NORMAL start-up transient, one short burst per attach, and warning about
/// it would fire on every healthy camera), while one that cannot be attributed to
/// a rendition at all means the topic is interleaving streams this build cannot
/// separate (warn once — that IS an operator-actionable degradation, and the
/// counters stay exact whatever the log level).
fn render_video_sample(
    rec: &RecordingStream,
    input_name: &str,
    entity: &str,
    timestamp_ns: u64,
    fv: &FrameValue,
    payload: &crate::video::H264Payload,
    state: &mut SinkState,
) {
    match state.video.route(input_name, fv, payload) {
        crate::video::VideoRoute::Feed {
            key,
            is_keyframe,
            first_sample,
        } => {
            let stream_entity = crate::video::video_entity(entity, key);
            if first_sample {
                tracing::info!(
                    input = route_key_topic(input_name),
                    entity = %stream_entity,
                    width = key.width,
                    height = key.height,
                    keyframe = is_keyframe,
                    field = %payload.field,
                    // States the STREAM fact only. Which side decodes it is not
                    // known at this point and is not this line's to claim: the
                    // decoder logs "decoder opened for this rendition" when the
                    // desk takes it, and the no-decoder warn when the viewer does.
                    // Saying "decoded on this desk" here was wrong in exactly the
                    // regime an operator most needs to identify.
                    "cerulion_viz: H.264 stream opened"
                );
            }
            // Feed EVERY routed access unit — P-frames chain off references, so
            // skipping one to "catch up" would break the frames after it. The
            // catch-up comes for free at the OTHER end: each decoded picture is
            // logged as an `Image`, which REPLACES its predecessor at the same
            // entity, so a burst of five costs five ~0.5 ms decodes and the viewer
            // shows the newest.
            // The decode sub-stage. Clock reads only when the
            // probe is on; the elapsed time is parked on `state` for the caller
            // that holds this frame's wire sequence.
            let probe_decode_t0 = cerulion_core::lat_probe::probe_enabled().then(Instant::now);
            let decoded = state
                .video_decoders
                .decode(input_name, key, payload.bytes, timestamp_ns);
            if let Some(t0) = probe_decode_t0 {
                state.probe_decode_us = t0.elapsed().as_micros() as u64;
            }
            match decoded {
                crate::video_decode::DecodeOutcome::Frame(frame) => {
                    // The picture's OWN stamp, not this message's.
                    // openh264 holds a picture for one call on a stream it cannot
                    // release inline, so the frame that comes out here belongs to
                    // an earlier access unit; `timestamp_ns` would label it one
                    // frame-period newer than it is.
                    crate::archetype::log_raw_image(
                        rec,
                        &stream_entity,
                        frame.timestamp_ns,
                        frame.width,
                        frame.height,
                        crate::archetype::RawEncoding::Rgb8,
                        &frame.rgb,
                    );
                }
                // No picture (a parameter-set-only unit) or a refused one (a
                // missing reference across a stall). Both are accounted for
                // inside the decoder — counters plus a flood-latched log — and
                // neither may render a fabricated frame here.
                crate::video_decode::DecodeOutcome::NoPicture
                | crate::video_decode::DecodeOutcome::Failed => {}
                // NO DECODER on this desk (Cisco's binary is not cached yet —
                // see the fetch module). Hand the access unit to the VIEWER exactly as the original path
                // did: that path is slow (the 567 ms of ffmpeg frame threading
                // this issue exists to remove) but it RENDERS, which beats a black
                // pane. The decoder logs the reason once; here we only re-declare
                // the codec, which rerun REQUIRES to decode H.264 at all.
                crate::video_decode::DecodeOutcome::DecoderUnavailable => {
                    if first_sample {
                        crate::video::log_video_codec(rec, &stream_entity);
                    }
                    crate::video::log_video_sample(
                        rec,
                        &stream_entity,
                        timestamp_ns,
                        payload.bytes,
                        is_keyframe,
                    );
                }
            }
        }
        crate::video::VideoRoute::Drop(crate::video::VideoReject::BeforeKeyframe) => {
            tracing::debug!(
                input = route_key_topic(input_name),
                dropped = state.video.dropped_before_keyframe(input_name),
                "cerulion_viz: H.264 access unit dropped — waiting for the stream's first \
                 keyframe (SPS), which a decoder needs to start"
            );
            // ESCALATION: a short wait is the normal mid-GOP attach transient, but
            // a topic that has waited this long with NOTHING ever opened will not
            // render, and an empty pane with only debug logs is exactly the silent
            // failure the loud-not-silent rule forbids.
            if state.video.take_never_keyframed_warn(input_name) {
                tracing::warn!(
                    input = route_key_topic(input_name),
                    dropped = state.video.dropped_before_keyframe(input_name),
                    sps_parse_failures = state.video.sps_parse_failures(input_name),
                    "cerulion_viz: this topic is carrying H.264 but has never delivered a \
                     parameter set, so no decoder can be started and NOTHING has rendered. A \
                     producer that emits SPS/PPS only at long intervals (or never) is the usual \
                     cause; a nonzero sps_parse_failures instead means the parameter sets ARE \
                     arriving and this build cannot read them."
                );
            }
        }
        crate::video::VideoRoute::Drop(crate::video::VideoReject::Unattributable) => {
            if state.video.take_unattributable_warn(input_name) {
                tracing::warn!(
                    input = route_key_topic(input_name),
                    streams = state.video.streams(input_name).len(),
                    "cerulion_viz: this topic has shown two or more H.264 resolutions and an \
                     access unit carried neither its own parameter set nor a value this build \
                     has learned to map to one of them, so it cannot be attributed; such frames \
                     are DROPPED rather than fed to the wrong decoder (which would corrupt that \
                     stream's picture). Two causes look alike here: a topic genuinely \
                     INTERLEAVING renditions (publish each on its own topic), or one that CHANGED \
                     resolution without a field this build could learn as a rendition tag (it \
                     recovers on its own once the new resolution's next keyframe arrives)."
                );
            }
        }
    }
}

/// Draw a `MarkerArray` frame's markers at their own entities and emit
/// the CLEARs its `DELETE` / `DELETEALL` actions resolve to.
///
/// The three [`MarkerArrayScan`] outcomes are deliberately NOT collapsed, exactly
/// as [`render_element_array`] keeps its three apart:
///
/// - `Plan` — clears first (sorted), then draws (declaration order), then the
///   once-per-input degradation reports;
/// - `Empty` — an idle publisher's zero-marker array. Draws nothing, takes NO
///   fallback, and **does not clear**: an empty array is not a `DELETEALL`, and
///   degrading here would flip the topic between geometry and a `TextDocument`
///   as the array empties and refills;
/// - `Absent` — no decodable `markers` array. Degrades to the element-enumerating
///   field dump, resolving the reason from the FRAME exactly as
///   [`render_element_array`] does: a top-level array whose ELEMENT BYTES the
///   walker refused is both the most specific cause and the only actionable one,
///   so it wins and NAMES the field. On a valid `MarkerArray` frame that is the
///   ONLY cause — the walker always decodes `markers` as one of the two array
///   kinds, and a truncated payload fails before dispatch — which is why there is
///   no marker-specific `FallbackReason`: it would be a variant nothing could
///   produce. The archetype-level `UndecodableElementArray` remains as the total
///   fallback (its text names the marker-array case).
///
/// **Posing.** `route.entity` itself renders nothing here — every
/// marker lives on a child — but it still takes the ordinary
/// `CoordinateFrame` assignment in [`render_classified`] (as `Scalars` does,
/// which likewise draws nothing spatial there). A child does NOT inherit that
/// assignment (rerun derives a child's implicit frame from the PATH), so each
/// marker's OWN `Transform3D` carries the resolved frame as its `parent_frame` —
/// the transform-payload mechanism, since a `CoordinateFrame` on an entity whose
/// payload is a transform relocates its data while leaving the chain
/// path-parented.
fn render_marker_array(
    rec: &RecordingStream,
    input_name: &str,
    route: &InputRoute,
    timestamp_ns: u64,
    fv: &FrameValue,
    state: &mut SinkState,
    resolved_frame: &Option<String>,
) {
    let plan = match scan_marker_array(fv) {
        MarkerArrayScan::Plan(plan) => plan,
        MarkerArrayScan::Empty => return,
        MarkerArrayScan::Absent => {
            let opaque = opaque_element_arrays(fv);
            let reason = if opaque.is_empty() {
                FallbackReason::UndecodableElementArray
            } else {
                FallbackReason::UndecodableElementBytes(opaque)
            };
            anyvalues_fallback(
                rec,
                input_name,
                &route.entity,
                timestamp_ns,
                fv,
                state,
                ArchetypeKind::MarkerArray,
                reason,
            );
            return;
        }
    };
    let live = state.marker_live.entry(input_name.to_string()).or_default();
    let actions = resolve_marker_ops(&plan.ops, live);
    for key in &actions.clears {
        log_marker_clear(rec, &marker_entity(&route.entity, key), timestamp_ns);
    }
    for draw in &actions.draws {
        let entity = marker_entity(&route.entity, &draw.key.entity_key());
        let parent = transform_parent_frame(&entity, resolved_frame);
        log_marker_draw(rec, &entity, timestamp_ns, draw, parent.as_deref());
    }
    report_marker_plan(input_name, &plan.reports, &actions, state);
}

/// Name every degradation a `MarkerArray` frame earned — ONCE per
/// `(input, discriminator)`, never per frame.
///
/// Each arm is a NAMED omission rather than a silent one. A marker that did not
/// render, a colour bank that was ignored, a trailing point that was dropped, a
/// `lifetime` that is not enforced: an operator seeing fewer markers than the
/// robot published must be able to find out why from the log, and a per-frame
/// flood would bury it.
fn report_marker_plan(
    input_name: &str,
    plan: &MarkerReports,
    actions: &MarkerFrameActions,
    state: &mut SinkState,
) {
    // The two overflow counts are deliberately SECOND terms: they are not
    // frame-decode facts (they come from resolving the frame against this
    // input's live state), so they cannot live in `MarkerReports` — and
    // returning on `is_clean()` alone would swallow both cap warns. See
    // `MarkerReports::is_clean`.
    if plan.is_clean() && actions.draw_overflow == 0 && actions.clear_overflow == 0 {
        return;
    }
    let topic = route_key_topic(input_name);
    // `true` on the FIRST sighting of this (input, discriminator) — the house
    // once-per-X latch, with the key shape documented on `SinkState::marker_notes`.
    // The set is CAPPED because two discriminators are publisher-controlled; the
    // cap announces itself once so the resulting silence is never mysterious.
    let mut first = |discriminator: &str| -> bool {
        if state.marker_notes.len() >= MARKER_NOTES_CAP {
            return false;
        }
        let fresh = state
            .marker_notes
            .insert(format!("{input_name}::{discriminator}"));
        if fresh && state.marker_notes.len() == MARKER_NOTES_CAP {
            tracing::warn!(
                input = topic,
                max_marker_notes = MARKER_NOTES_CAP,
                "cerulion_viz: this run has hit max_marker_notes distinct marker diagnostics, \
                 so further ones are SUPPRESSED for the rest of the run (a publisher cycling \
                 mesh URIs or marker type values reaches this). Rendering is unaffected"
            );
        }
        fresh
    };

    if plan.truncated_markers > 0 && first("truncated-markers") {
        tracing::warn!(
            input = topic,
            not_rendered = plan.truncated_markers,
            max_marker_instances = MAX_MARKER_INSTANCES,
            "cerulion_viz: MarkerArray longer than max_marker_instances — inspecting the first \
             max_marker_instances markers and DROPPING the rest for this topic. The dropped tail's \
             ACTIONS are not applied either, so a DELETE or DELETEALL in it will not take effect \
             and its markers can linger. The full array is still on the wire: read it with \
             `cerulion topic echo`"
        );
    }
    if plan.truncated_vertices > 0 && first("truncated-vertices") {
        tracing::warn!(
            input = topic,
            markers_skipped = plan.truncated_vertices,
            max_marker_vertices = MAX_MARKER_VERTICES,
            "cerulion_viz: this frame's markers exceed max_marker_vertices in total, so whole \
             markers were skipped (never half a polyline or a mesh missing its last triangles). \
             The full array is still on the wire: read it with `cerulion topic echo`"
        );
    }
    for value in &plan.unknown_types {
        if first(&format!("unknown-type={value}")) {
            tracing::warn!(
                input = topic,
                marker_type = value,
                "cerulion_viz: MarkerArray element carries a Marker.type this build does not \
                 render, so THAT marker is skipped — every other marker in the array still \
                 draws. Nothing is guessed from an unknown type"
            );
        }
    }
    for value in &plan.unknown_actions {
        if first(&format!("unknown-action={value}")) {
            tracing::warn!(
                input = topic,
                marker_action = value,
                "cerulion_viz: MarkerArray element carries a Marker.action the message \
                 definition does not declare (ADD/MODIFY=0, DELETE=2, DELETEALL=3), so THAT \
                 marker is skipped — an undeclared action has no defined meaning"
            );
        }
    }
    if plan.undecodable > 0 && first("undecodable-elements") {
        tracing::warn!(
            input = topic,
            markers_skipped = plan.undecodable,
            "cerulion_viz: MarkerArray elements carried no usable body (not a decoded message, \
             or missing `id`/`action`/`type`/`pose`), so those markers are skipped. The \
             remediation is the PRODUCER's payload"
        );
    }
    if plan.color_length_mismatches > 0 && first("colors-length") {
        tracing::info!(
            input = topic,
            markers = plan.color_length_mismatches,
            "cerulion_viz: MarkerArray marker has a non-empty `colors` whose length does not \
             match `points`, so the flat `color` is used for every vertex — zipping a short \
             colour bank would mis-colour the tail"
        );
    }
    for kind in &plan.colors_dropped_kinds {
        if first(&format!("colors-dropped={}", kind.wire_name())) {
            tracing::info!(
                input = topic,
                marker_type = kind.wire_name(),
                "cerulion_viz: MarkerArray marker supplies a correctly-sized per-point `colors` \
                 bank for a type whose rerun archetype cannot express it (a LINE_STRIP is ONE \
                 strip with one colour, a single ARROW one primitive), so the flat `color` is \
                 used. This is a RENDERER limit, not a producer bug — the bank is fine; split \
                 the marker into per-segment LINE_LIST entries to colour it per segment"
            );
        }
    }
    if plan.markers_with_frame_id > 0 && first("frame-id") {
        tracing::info!(
            input = topic,
            markers = plan.markers_with_frame_id,
            "cerulion_viz: MarkerArray marker carries its own `header.frame_id`, which this \
             build does NOT read — every marker on this topic renders in the TOPIC's frame, so \
             markers stamped in different frames land in the same place. This is the \
             platform-wide per-element frame gap (every element archetype has it), not a \
             MarkerArray one; publish one topic per frame to place them correctly"
        );
    }
    for kind in &plan.dropped_tail_kinds {
        if first(&format!("dropped-tail={}", kind.wire_name())) {
            tracing::info!(
                input = topic,
                marker_type = kind.wire_name(),
                "cerulion_viz: MarkerArray marker's `points` count does not fit its type's \
                 grouping (LINE_LIST needs pairs, TRIANGLE_LIST triples, an arrow or strip at \
                 least two), so the trailing point(s) are dropped rather than drawn as a \
                 partial primitive"
            );
        }
    }
    if plan.elliptical_cylinders > 0 && first("elliptical-cylinder") {
        tracing::info!(
            input = topic,
            markers = plan.elliptical_cylinders,
            "cerulion_viz: CYLINDER marker has scale.x != scale.y (an elliptical cross-section) \
             — rerun cylinders take ONE radius, so it is drawn circular at the mean radius"
        );
    }
    if plan.degenerate_scales > 0 && first("degenerate-scale") {
        tracing::info!(
            input = topic,
            markers = plan.degenerate_scales,
            "cerulion_viz: MarkerArray marker has an all-zero `scale` for a type that needs it, \
             so it renders with zero size and is effectively invisible. This is what the \
             publisher asked for — nothing is substituted — and this note exists so an empty \
             view is explainable"
        );
    }
    if plan.lifetime_markers > 0 && first("lifetime") {
        tracing::info!(
            input = topic,
            markers = plan.lifetime_markers,
            "cerulion_viz: MarkerArray marker declares a non-zero `lifetime`, which this build \
             does NOT expire — the marker persists until an explicit DELETE/DELETEALL or a \
             viewer reconnect"
        );
    }
    if plan.frame_locked_markers > 0 && first("frame-locked") {
        tracing::info!(
            input = topic,
            markers = plan.frame_locked_markers,
            "cerulion_viz: MarkerArray marker sets `frame_locked`, which this build does NOT \
             honour — the marker is posed once from its own `pose` rather than re-transformed \
             every frame in its `header.frame_id`"
        );
    }
    for uri in &plan.mesh_uris {
        if first(&format!("mesh={uri}")) {
            tracing::info!(
                input = topic,
                mesh_resource = %uri,
                "cerulion_viz: MESH_RESOURCE marker is drawn as a labelled PROXY BOX at the \
                 marker's pose and scale — the mesh is not fetched. Resolving a `package://` \
                 URI needs the robot's package tree, which a desk attaching to an unseen robot \
                 does not have, and fetching it would put IO on the render path"
            );
        }
    }
    // TWO ceilings, TWO messages. They were one aggregate, which made the warn
    // factually wrong in the second regime: the cleared half fills with
    // `live_count() == 0`, where "more than N markers live at once" describes a
    // state that is not happening and the DELETEALL remedy changes nothing.
    if actions.draw_overflow > 0 && first("live-cap") {
        tracing::warn!(
            input = topic,
            untracked = actions.draw_overflow,
            max_live_markers = MAX_LIVE_MARKERS,
            "cerulion_viz: this input has more than max_live_markers markers LIVE at once, so \
             the excess are NOT TRACKED. They still DRAW, and an explicit DELETE of one still \
             clears it — what is lost is DELETEALL, which can only sweep what is tracked, so \
             those markers linger until they are individually deleted or the viewer \
             reconnects. A publisher that cycles marker ids (a fresh ns/id every frame) fills \
             this ceiling over time even with small frames; reuse ids, or send a DELETEALL \
             before each fresh set"
        );
    }
    if actions.clear_overflow > 0 && first("cleared-cap") {
        tracing::warn!(
            input = topic,
            untracked = actions.clear_overflow,
            max_live_markers = MAX_LIVE_MARKERS,
            "cerulion_viz: this input has RETIRED more than max_live_markers distinct markers, \
             so their retirement is no longer remembered. Nothing is mis-drawn and no marker \
             lingers — the cost is that an identical repeated DELETE for one of them re-emits \
             its clear on EVERY frame instead of once. A publisher that cycles marker ids and \
             re-sends each retirement reaches this; stop re-sending DELETEs for markers already \
             retired, or send ONE DELETEALL instead of per-marker DELETEs"
        );
    }
}

/// Extract the `transforms` array bytes from a walked `TFMessage` — exactly
/// the blob [`decode_tf_transforms`] consumes and the `/tf_static` dedup keys
/// on.
///
/// Every array-shaped variant yields the SAME bytes: `NestedArrayOpaque`
/// carries the field slice verbatim, and the walker's `NestedArray` (bytes the
/// walker decoded under the canonical element framing) carries that identical
/// slice in `raw`. So a canonically-framed `/tf` — which the `ros2 attach` CDR
/// path really does produce — reaches this module's bespoke decoder exactly as
/// it did before element framing and gets the same TRUTHFUL "undecodable TF
/// transforms blob" diagnosis, instead of the false "frame has no `transforms`
/// array". An EMPTY array's `raw` is the field's own empty slice, which
/// [`decode_tf_transforms`] reads as zero transforms. (Consuming the walker's
/// decoded `elements` instead of re-parsing `raw` is
/// deliberately not done here.)
fn transforms_bytes_of<'a>(fv: &FrameValue<'a>) -> Option<&'a [u8]> {
    match fv.field("transforms") {
        Some(FrameValueKind::NestedArrayOpaque(b)) => Some(*b),
        Some(FrameValueKind::Bytes(b)) => Some(*b),
        Some(FrameValueKind::NestedArray { raw, .. }) => Some(*raw),
        _ => None,
    }
}

/// Decode + log a `TFMessage` frame's transforms on the entity tree. For a
/// static route, dedups against the last logged `transforms` bytes (a
/// re-broadcast is not re-logged) and logs via `log_static`.
fn dispatch_transforms(
    rec: &RecordingStream,
    input_name: &str,
    route: &InputRoute,
    timestamp_ns: u64,
    fv: &FrameValue,
    state: &mut SinkState,
) {
    let Some(tf_bytes) = transforms_bytes_of(fv) else {
        if state.decode_warn.insert(format!("tf-field::{input_name}")) {
            tracing::warn!(
                input = route_key_topic(input_name),
                "cerulion_viz: TFMessage frame has no `transforms` array — skipping"
            );
        }
        return;
    };

    if route.is_static {
        // Dedup: a re-broadcast of the SAME mount table is not re-logged
        // (log_static APPENDS a chunk on every call, so a ~10 Hz re-broadcast
        // would grow viewer memory without bound).
        let changed = state.tf_static_last.get(input_name).map(Vec::as_slice) != Some(tf_bytes);
        if !changed {
            return;
        }
        state
            .tf_static_last
            .insert(input_name.to_string(), tf_bytes.to_vec());
    }

    match decode_tf_transforms(tf_bytes) {
        Ok(transforms) => {
            // Record every CHILD frame this tree places, so a topic
            // stamped with one of them can be POSED (`assign_coordinate_frame`).
            // This is the only evidence that a non-alias frame is real — without
            // it, a robot publishing `/tf` for a frame the Go2 alias table has
            // never heard of would leave its sensor topics unposed.
            for t in &transforms {
                state.frames.observe_child(&t.child_frame_id);
            }
            // Static transforms have no timeline (`log_static` ignores the
            // stamp), so pass 0 for them; temporal ride the wire stamp.
            let ts = if route.is_static { 0 } else { timestamp_ns };
            log_transforms(
                rec,
                ts,
                &transforms,
                route.is_static,
                &mut state.unknown_frames,
            );
        }
        Err(e) => {
            if state.decode_warn.insert(format!("tf-decode::{input_name}")) {
                tracing::warn!(error = %e, input = route_key_topic(input_name), "cerulion_viz: undecodable TF transforms blob — skipping");
            }
        }
    }
}

/// Why a frame took the field-dump fallback — selects the TRUTHFUL
/// remediation in [`anyvalues_fallback`]'s once-per-schema info. An
/// image-family frame with an encoding not decoded here degrades inside the
/// Image arm, so the generic "add a mapping / shape" advice would send an
/// operator on a wasted debug cycle.
///
/// **This enum is the ONE place a degrade explains itself.**
/// [`anyvalues_fallback`] emits exactly one latched line per fallback, so every
/// distinct remediation must be a variant here rather than a second diagnostic
/// alongside it: two lines for one condition read as two problems, and the
/// less-specific of the two is the one an operator acts on first.
#[derive(Debug, Clone)]
enum FallbackReason {
    /// The schema is neither in [`classify_schema`]'s table nor inferable by
    /// [`infer_archetype_from_shape`] — a native mapping or a recognizable
    /// field shape is the correct remediation.
    ///
    /// The ONE reason that can CO-OCCUR with undecodable array bytes without being
    /// caused by them (an unmapped schema dumps either way), so
    /// [`anyvalues_fallback`] splits it: with undecodable arrays it names them and
    /// carries BOTH remediations on ONE per-INPUT warn; without, it stays the
    /// per-schema info. Never both — one condition, one line.
    UnmappedSchema,
    /// The frame is an image (mapped or inferred) but its `encoding` is one we
    /// do not decode to a native [`rerun::Image`] — the missing piece is
    /// raw-pixel decode support for that encoding, not a table entry.
    UnsupportedImageEncoding,
    /// The frame CLASSIFIED as a box / occupancy grid but this frame's
    /// geometry did not extract (a box with no reachable centre, a grid with no
    /// `info` dimensions or a cell buffer shorter than `width * height`) — the
    /// remediation is the producer's payload, not a mapping.
    UndecodableOccupancyOrBox,
    /// The frame CLASSIFIED as an element-array archetype (a path, a pose
    /// array, a detection set) but carries no DECODABLE element array, and no
    /// TOP-LEVEL array field is the culprit either (the reachable shape is an array
    /// one hop down — a `geometry_msgs/PolygonStamped` whose `polygon.points` the
    /// walker refused — or elements that decoded but carry no geometry). Nothing
    /// specific can be named, so the message stays at the archetype level; when a
    /// top-level array IS the culprit, [`FallbackReason::UndecodableElementBytes`]
    /// supersedes this and names it.
    UndecodableElementArray,
    /// The frame carries a top-level array field whose ELEMENT
    /// BYTES the walker refused (`NestedArrayOpaque`), so there is nothing to
    /// enumerate and nothing to draw — the remediation is the PRODUCER's element
    /// encoding, and the actionable fact is WHICH field. Carries the affected
    /// `(field, byte length)` pairs, so the reason cannot be raised without them.
    ///
    /// Resolved from the FRAME in [`render_element_array`], not chosen by the
    /// calling arm, and it OVERRIDES the arm's category reason. That override is
    /// the point: a `vision_msgs/Detection3DArray` whose `detections` bytes are
    /// undecodable is not a box that "needs a reachable `center`/`pose`"
    /// ([`FallbackReason::UndecodableOccupancyOrBox`]) — sending an operator to
    /// inspect the box payload is exactly the wasted debug cycle this enum exists
    /// to prevent.
    ///
    /// Deliberately says nothing about WHICH producer wrote the bytes: the encoding
    /// gap is the durable fact, while any given producer's divergence is a moving
    /// target (the ROS 2 rmw bridge has since converged onto the canonical element
    /// body, after which those topics decode and never reach here — a message
    /// naming that bridge would then be advice for a case that no longer exists).
    UndecodableElementBytes(Vec<(String, usize)>),
    /// The frame CLASSIFIED as [`ArchetypeKind::VideoStream`] — so the
    /// classifier DID find an Annex-B access unit in it — yet the render arm's
    /// re-scan found none.
    ///
    /// A distinct reason because [`FallbackReason::UnmappedSchema`] would be a
    /// provable LIE here: the schema is not unmapped, it is CONTENT-mapped, and
    /// sending an operator to add a table entry would waste the debug cycle this
    /// enum exists to prevent. Classify and render read the same bytes with the
    /// same function, so this is unreachable today; it exists so a future refactor
    /// that breaks the agreement reports the truth rather than a plausible fiction.
    VideoRescanDisagreed,
}

/// Log an untabled (or mapped-but-builder-less) frame as an inspectable
/// field dump — the AnyValues fallback. Emits EXACTLY ONE latched log the first
/// time each distinct schema takes this path (loud, but never a per-frame flood),
/// with the remediation matched to `reason`.
///
/// **One condition, one explanation.** Every reason routes
/// through the single `match` below; nothing else here logs, and each reason emits
/// at most one line. Reasons that name undecodable arrays latch per INPUT + field
/// NAME set rather than per schema, because element encoding is a property of the
/// PRODUCER: one schema can arrive on a natively-produced topic and on a bridged
/// twin, and those are different facts. `unknown_schema` is still `observe`d for
/// EVERY fallback — it is the structural "this schema took the dump path" seam
/// ([`SinkState::took_anyvalues_fallback`]), independent of which latch gates the
/// human-readable line.
///
/// The `match` is over `(reason, first_for_schema)` with no wildcard arm ON THE
/// REASON, so a new [`FallbackReason`] variant is a COMPILE ERROR here rather than
/// a degrade that silently explains nothing — the invariant this enum's own doc
/// states, kept by the type system instead of by inspection.
///
/// **`kind` is the DEGRADING archetype, and it is checked here.** The
/// layout gives a `text_document` view to exactly the kinds
/// [`ArchetypeKind::can_degrade_to_dump`] names, so an arm that reaches this
/// function for a kind the predicate calls un-degradable logs its dump into an
/// entity whose views cannot display it — the user sees nothing, with no
/// diagnostic. The `debug_assert!` makes that a LOUD test failure at the first
/// frame that takes the path, which is the link between the predicate and the
/// render arms: the const assertion in [`crate::blueprint`] proves the predicate
/// and the view table agree, and this proves the predicate and the CODE agree.
#[allow(clippy::too_many_arguments)] // one arg per independent fact; see `kind` above
fn anyvalues_fallback(
    rec: &RecordingStream,
    input_name: &str,
    entity: &str,
    timestamp_ns: u64,
    fv: &FrameValue,
    state: &mut SinkState,
    kind: ArchetypeKind,
    reason: FallbackReason,
) {
    debug_assert!(
        kind.can_degrade_to_dump(),
        "{kind:?} reached the field-dump fallback but \
         ArchetypeKind::can_degrade_to_dump() says it cannot, so the layout gives it no \
         text_document view and this dump renders NOWHERE. Add {kind:?} to \
         can_degrade_to_dump() (and to views_for_archetype's text_document set)"
    );
    // This frame DEGRADED, so the topic's dump companion is earned — for
    // the rest of the attach. Recorded here rather than in the arms because this is
    // the one function every degradation reaches (the `debug_assert!` above is what
    // keeps that true), so the record cannot go stale when an arm is added.
    //
    // Unconditional, and deliberately not gated on `first_for_schema` or on any
    // latch: the LATCHES bound how often a human-readable line is printed, while
    // this is the state a layout is derived from and must be set the first time
    // it becomes true, whichever reason arm the report takes.
    state.note_render_degraded(input_name);
    let first_for_schema = state.unknown_schema.observe(&fv.schema_name);
    match (&reason, first_for_schema) {
        // WARN where the others are INFO: the other four describe a frame whose
        // CONTENT did not support the archetype (a genuine data condition, and
        // the dump still shows every field). This one describes bytes this build
        // could not decode AT ALL, so the elements are missing from the dump too.
        // Per INPUT, so `first_for_schema` is not this arm's gate.
        (FallbackReason::UndecodableElementBytes(fields), _) => {
            if state
                .element_opaque
                .insert(opaque_latch_key(input_name, fields))
            {
                tracing::warn!(
                    schema = %fv.schema_name,
                    input = route_key_topic(input_name),
                    entity,
                    fields = %describe_opaque_fields(fields),
                    "cerulion_viz: this topic's array field carries element bytes this build \
                     cannot decode, so its elements are not enumerated and the topic renders \
                     as a field dump instead of a shape. The frame itself is intact on the \
                     wire — read it with `cerulion topic echo`. The remediation is the \
                     PRODUCER's array element encoding, not a classify_schema mapping"
                );
            }
        }
        // The one reason that can CO-OCCUR with undecodable array bytes without
        // being caused by them: an unmapped schema dumps whether or not its arrays
        // decode. So it splits, and the two halves are mutually exclusive — still
        // exactly one line per fallback:
        //
        // - arrays all decoded ⇒ the per-SCHEMA "add a mapping" info, unchanged;
        // - some array did NOT ⇒ ONE warn carrying BOTH remediations, latched per
        //   INPUT like every other undecodable-array report. Per-input and WARN are
        //   both load-bearing: element encoding is a producer fact, so a second
        //   input on the same unmapped schema is a NEW one and must not be
        //   swallowed by the schema latch, and an operator running at
        //   `RUST_LOG=warn` would never see it at info.
        (FallbackReason::UnmappedSchema, first_for_schema) => {
            let opaque = opaque_element_arrays(fv);
            if opaque.is_empty() {
                if first_for_schema {
                    tracing::info!(
                        schema = %fv.schema_name,
                        entity,
                        "cerulion_viz: no archetype mapping AND no inferable field shape for \
                         this schema — logging a field dump (AnyValues fallback; nothing is \
                         un-visualizable). Add a mapping in cerulion_viz::sink::classify_schema \
                         (or a recognizable shape) to render it natively"
                    );
                }
            } else if state
                .element_opaque
                .insert(opaque_latch_key(input_name, &opaque))
            {
                tracing::warn!(
                    schema = %fv.schema_name,
                    input = route_key_topic(input_name),
                    entity,
                    fields = %describe_opaque_fields(&opaque),
                    "cerulion_viz: no archetype mapping AND no inferable field shape for this \
                     schema, AND its array elements are in an encoding this build cannot \
                     decode — so the field dump does not enumerate them either. The frame \
                     itself is intact on the wire — read it with `cerulion topic echo`. Two \
                     remediations: add a mapping in cerulion_viz::sink::classify_schema (or a \
                     recognizable shape) to render the schema natively, and the elements need \
                     the PRODUCER's array element encoding"
                );
            }
        }
        (FallbackReason::UnsupportedImageEncoding, true) => tracing::info!(
            schema = %fv.schema_name,
            entity,
            "cerulion_viz: image frame with an encoding we do not decode — showing a \
             field dump (rgb8/bgr8/mono8 raw + JPEG CompressedImage render natively; \
             the missing piece is raw-pixel decode support for this encoding, NOT a \
             classify_schema mapping)"
        ),
        (FallbackReason::UndecodableOccupancyOrBox, true) => tracing::info!(
            schema = %fv.schema_name,
            entity,
            "cerulion_viz: frame maps to a box / occupancy-grid archetype but this frame \
             carries no extractable geometry (a box needs a reachable `center`/`pose`; a \
             grid needs `info.width`/`info.height` and at least width*height cells) — \
             showing a field dump instead of a mis-rendered shape"
        ),
        (FallbackReason::UndecodableElementArray, true) => tracing::info!(
            schema = %fv.schema_name,
            entity,
            "cerulion_viz: frame maps to a path / pose-array / detection-set / marker-array \
             archetype but carries no DECODABLE element array — showing an element-enumerating \
             field dump instead of a mis-rendered shape"
        ),
        // Classify said video, the render re-scan said no. Never expected
        // (one function, same bytes), so it is a WARN not an info — an inconsistency
        // between the two halves is a bug in this crate, not a producer's payload.
        (FallbackReason::VideoRescanDisagreed, true) => tracing::warn!(
            schema = %fv.schema_name,
            entity,
            "cerulion_viz: frame classified as an H.264 video stream but the render path found \
             no access unit in it — showing a field dump. Classification and rendering read the \
             same bytes through the same scanner, so this disagreement is a Cerulion bug, not a \
             problem with the robot's stream"
        ),
        // A repeat sighting of a schema already explained: silent by design. Spelled
        // out per variant rather than as a wildcard, so a NEW reason cannot inherit
        // silence by default (see this fn's doc).
        (FallbackReason::UnsupportedImageEncoding, false)
        | (FallbackReason::UndecodableOccupancyOrBox, false)
        | (FallbackReason::UndecodableElementArray, false)
        | (FallbackReason::VideoRescanDisagreed, false) => {}
    }
    log_field_dump(rec, entity, timestamp_ns, fv);
}

/// `field (N bytes)` per undecodable array, comma-joined — the operator-facing
/// naming of the affected fields, quoted in the log MESSAGE so a reader can size
/// the blob they are about to `cerulion topic echo`.
///
/// Deliberately NOT the flood-latch key: the byte count of a variable-length array
/// (a `nav_msgs/Path`'s `4 + Σ(4 + body)`, a `MarkerArray` that shifts with any
/// marker text) moves on almost every frame, so keying on it would re-warn per
/// frame and retain one `String` per distinct size forever. [`opaque_latch_key`] is
/// the key.
fn describe_opaque_fields(fields: &[(String, usize)]) -> String {
    fields
        .iter()
        .map(|(name, bytes)| format!("{name} ({bytes} bytes)"))
        .collect::<Vec<_>>()
        .join(", ")
}

/// The [`SinkState::element_opaque`] flood-latch key for an undecodable-array
/// report: `input::<field names>`, in the frame's declaration order.
///
/// NAMES ONLY — the identity of the CONDITION ("these fields of this input do not
/// decode"), which is stable for as long as the condition holds. A topic whose
/// undecodable SET later changes (a producer that starts framing one of two arrays
/// canonically) is a genuinely new fact and reports again; one whose array merely
/// changes LENGTH is the same fact and stays silent.
fn opaque_latch_key(input_name: &str, fields: &[(String, usize)]) -> String {
    let names = fields
        .iter()
        .map(|(name, _)| name.as_str())
        .collect::<Vec<_>>()
        .join(", ");
    format!("{input_name}::{names}")
}

/// Whether an element array must be PROMOTED to an ordered polyline — asked of
/// the schema NAME, never of a classified (possibly memoized) archetype.
///
/// The promotion exists for exactly one situation: a type whose element ORDER is
/// semantic while its elements carry no per-element stamp
/// (`geometry_msgs/Polygon[Stamped]`), which only [`classify_schema`] can know.
/// See [`ArchetypeKind::forces_ordered_elements`] for why routing it through the
/// name table — rather than through the classified kind, as before the memo —
/// preserves the pre-memo behaviour exactly.
///
/// The element-render path asks it by NAME rather than reading the classified
/// kind, because that kind can be MEMOIZED per topic: one predicate, one
/// meaning, and no way for a stale memo to decide the promotion.
///
/// `pub` because it is half of the classify↔render contract and is oracle-tested
/// directly: it must DISAGREE with `kind.forces_ordered_elements()` exactly where
/// a memoized kind could be stale (an unmapped schema inferred to `Path3D`), which
/// is the whole point and is not observable from the render output alone.
pub fn name_mapped_forces_ordered(schema_name: &str) -> bool {
    mapped_kind_forces_ordered(classify_schema(schema_name))
}

/// [`name_mapped_forces_ordered`] over an ALREADY-RESOLVED table lookup — the one
/// implementation of the rule, so the two spellings can never diverge.
///
/// For a caller that has ALREADY asked [`classify_schema`], asking again by name
/// would walk the table twice per frame for the same answer. `None` — an
/// UNMAPPED schema — is `false`: the promotion is a property of the NAME table
/// alone, never of a shape-inferred or memoized kind.
pub fn mapped_kind_forces_ordered(mapped: Option<ArchetypeKind>) -> bool {
    mapped.is_some_and(ArchetypeKind::forces_ordered_elements)
}

/// Draw an element-array frame's geometry, degrading to the
/// element-enumerating field dump ONLY when the frame carries no decodable element
/// array at all.
///
/// The three [`ElementArrayScan`] outcomes are deliberately NOT collapsed:
///
/// - `Geometry` — drawn, plus the once-per-input truncation report when the array
///   exceeded [`MAX_ELEMENT_INSTANCES`];
/// - `Empty` — an idle array (a nav2 robot with no current plan). NOTHING is drawn
///   and NO fallback is taken: degrading here would flip the topic between a
///   polyline and a `TextDocument` every time the plan clears, and would falsely
///   mark the schema as having taken the AnyValues path;
/// - `Absent` — the real failure, and the ONE place the dump's reason is chosen.
///
/// `Absent` has more than one cause, so the reason is resolved from the FRAME and
/// only falls back to the caller's `absent_reason`:
///
/// - a top-level array field whose ELEMENT BYTES the walker refused is both the
///   most specific cause and the only one that names something an operator can act
///   on, so it wins ⇒ [`FallbackReason::UndecodableElementBytes`];
/// - otherwise the caller's `absent_reason` stands, and it is the caller's because
///   the SAME element-array scan backs two classes of archetype: a `Boxes3D` frame
///   reached this scan only because its single-box geometry did not extract, so its
///   actual failure is [`FallbackReason::UndecodableOccupancyOrBox`] (the
///   producer's box payload), whereas a `Path3D`/`PoseArray3D` frame's is
///   [`FallbackReason::UndecodableElementArray`].
///
/// The override REPLACES rather than joins: emitting both left an operator with two
/// explanations for one condition, and for a `Boxes3D` frame the category line
/// ("a box needs a reachable `center`/`pose`") points at the wrong payload
/// entirely.
#[allow(clippy::too_many_arguments)]
fn render_element_array(
    rec: &RecordingStream,
    input_name: &str,
    route: &InputRoute,
    timestamp_ns: u64,
    fv: &FrameValue,
    state: &mut SinkState,
    kind: ArchetypeKind,
    absent_reason: FallbackReason,
    resolved_frame: &Option<String>,
) {
    // A name-mapped `geometry_msgs/Polygon` classifies as `Path3D` (a ring) but
    // its `Point32` elements are unstamped, so the shape ladder would yield an
    // unordered point bag — the hint promotes it back to a polyline. Asked of the
    // NAME, so a memoized inference can never fabricate an order (see
    // [`name_mapped_forces_ordered`]).
    let force_ordered = name_mapped_forces_ordered(&fv.schema_name);
    match log_element_array_from_frame(rec, &route.entity, timestamp_ns, fv, force_ordered) {
        ElementArrayScan::Geometry(parts) => {
            // A PATH also draws its waypoints at the
            // `<entity>/viz-vertices` child, whose implicit frame chains to the
            // parent's PATH — not to the frame the parent was re-pointed at — so
            // the assignment is repeated there or the waypoints float unposed
            // beside their own polyline. Emitted only for a Path (a Points/Boxes
            // scan creates no vertices child, and a frame declaration on an
            // entity with no geometry would add a phantom tree row).
            if matches!(parts.geometry, ElementGeometry::Path(_)) {
                let vertex_entity = format!("{}/{PATH_VERTICES_CHILD}", route.entity);
                emit_frame_at(rec, &vertex_entity, timestamp_ns, resolved_frame, state);
            }
            report_element_truncation(input_name, &fv.schema_name, &parts, state)
        }
        ElementArrayScan::Empty { .. } => {}
        ElementArrayScan::Absent => {
            let opaque = opaque_element_arrays(fv);
            let reason = if opaque.is_empty() {
                absent_reason
            } else {
                FallbackReason::UndecodableElementBytes(opaque)
            };
            anyvalues_fallback(
                rec,
                input_name,
                &route.entity,
                timestamp_ns,
                fv,
                state,
                kind,
                reason,
            )
        }
    }
}

/// Report, ONCE per input, that an element array was TRUNCATED at
/// [`MAX_ELEMENT_INSTANCES`]. Rendering the ceiling's worth of a longer array is
/// the right behaviour (an unbounded expansion is a per-frame render-buffer
/// hazard on an untrusted producer), but a silently short polyline reads as a
/// producer bug, so the ceiling names itself — and names its own value, so this
/// prose does not have to restate a number that has moved before (it was
/// raised from 10 000 to 300 000) and would rot again.
///
/// Latched per INPUT rather than per schema: two topics of the same schema can
/// differ wildly in element count, and the operator needs to know WHICH one is
/// clipped.
fn report_element_truncation(
    input_name: &str,
    schema_name: &str,
    parts: &ElementArrayParts,
    state: &mut SinkState,
) {
    if parts.truncated == 0 || !state.element_truncated.insert(input_name.to_string()) {
        return;
    }
    tracing::warn!(
        schema = %schema_name,
        input = route_key_topic(input_name),
        field = %parts.field,
        not_rendered = parts.truncated,
        max_element_instances = MAX_ELEMENT_INSTANCES,
        "cerulion_viz: element array longer than max_element_instances — rendering the first \
         max_element_instances elements and DROPPING the rest for this topic. The full array is \
         still on the wire: read it with `cerulion topic echo`"
    );
}

/// Report, ONCE per schema, any numeric field the series harvest
/// deliberately did not expand — an array longer than
/// [`MAX_ARRAY_SERIES`](crate::archetype::MAX_ARRAY_SERIES), a covariance MATRIX,
/// or a nested message below the harvest's hop limit. A silently missing plot
/// would look like a bug, so every skip is named exactly once (never a per-frame
/// flood).
///
/// **The message claims exactly the view the TOPIC has** — it must never promise
/// a rendering that does not exist, and must never deny one that does.
/// The earlier tail ("Every value stays visible in the topic's field dump")
/// promised a panel a plots-only topic does not have and sent an operator
/// hunting for it; it was replaced with a flat "NOT rendered anywhere else",
/// which the withheld-series election then made false for the topics it elects to
/// [`ArchetypeKind::ScalarsWithText`] — those DO render a dump, and telling their
/// operator to leave the viewer would send them away from the panel that now
/// holds the answer.
///
/// So the tail is keyed on [`ArchetypeKind::renders_text_document`] — the SAME
/// predicate the layout is built from, so the copy cannot drift from the views.
/// Either way it points at `cerulion topic echo` for the COMPLETE values: the
/// dump is bounded (a preview of each array, the first elements of a struct
/// array, a line ceiling), so it is a window onto the withheld fields, not a
/// replacement for reading them.
fn report_skipped_series(
    input_name: &str,
    schema_name: &str,
    skipped: &[SkippedSeries],
    kind: ArchetypeKind,
    state: &mut SinkState,
) {
    if skipped.is_empty() || !state.skipped_series.insert(schema_name.to_string()) {
        return;
    }
    let fields: Vec<String> = skipped.iter().map(|s| s.to_string()).collect();
    let where_to_look = if kind.renders_text_document() {
        "This topic's field dump (the text pane beside the plot) shows the message's own \
         values, which is where the fields above become readable — but it is BOUNDED and \
         names its own limits: it previews the first elements of each array, does not expand \
         a nested message past 4 hops, and stops after 200 lines. For the complete values: \
         `cerulion topic echo`"
    } else {
        "They are NOT rendered anywhere else for this topic. The values are still on the \
         wire: read them with `cerulion topic echo`"
    };
    tracing::info!(
        schema = %schema_name,
        input = route_key_topic(input_name),
        archetype = %crate::blueprint::archetype_wire_name(kind),
        not_plotted = %fields.join("; "),
        max_array_series = MAX_ARRAY_SERIES,
        max_struct_array_series = MAX_STRUCT_ARRAY_SERIES,
        max_total_series = MAX_TOTAL_SERIES,
        "cerulion_viz: these numeric fields are NOT plotted — an array longer than \
         max_array_series would emit one plot series per element, a struct array over \
         max_struct_array_series keeps its earliest fields for EVERY element and drops the \
         rest, a message over max_total_series keeps its earliest-declared series, a \
         covariance MATRIX is not a bank of time series, and a nested message below the \
         harvest's hop limit is not expanded. {where_to_look}"
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    use cerulion_core::codegen::NamedValue;

    use crate::archetype::inferred_scalars;

    #[test]
    fn classify_schema_maps_each_family_to_its_archetype() {
        // Hand oracles for the whole table (the authoritative mapping assertion).
        assert_eq!(
            classify_schema("sensor_msgs/PointCloud2"),
            Some(ArchetypeKind::Points3D)
        );
        assert_eq!(
            classify_schema("sensor_msgs/Image"),
            Some(ArchetypeKind::Image)
        );
        assert_eq!(
            classify_schema("sensor_msgs/CompressedImage"),
            Some(ArchetypeKind::Image)
        );
        assert_eq!(
            classify_schema("tf2_msgs/TFMessage"),
            Some(ArchetypeKind::Transforms)
        );
        assert_eq!(
            classify_schema("geometry_msgs/Twist"),
            Some(ArchetypeKind::Scalars)
        );
        assert_eq!(
            classify_schema("sensor_msgs/Joy"),
            Some(ArchetypeKind::Scalars)
        );
        // Later additions.
        assert_eq!(
            classify_schema("geometry_msgs/Pose"),
            Some(ArchetypeKind::Transform3D)
        );
        assert_eq!(
            classify_schema("geometry_msgs/PoseStamped"),
            Some(ArchetypeKind::Transform3D)
        );
        assert_eq!(
            classify_schema("geometry_msgs/Transform"),
            Some(ArchetypeKind::Transform3D)
        );
        assert_eq!(
            classify_schema("geometry_msgs/TransformStamped"),
            Some(ArchetypeKind::Transform3D)
        );
        assert_eq!(
            classify_schema("geometry_msgs/PointStamped"),
            Some(ArchetypeKind::Point3D)
        );
        assert_eq!(classify_schema("sensor_msgs/Imu"), Some(ArchetypeKind::Imu));
        assert_eq!(
            classify_schema("nav_msgs/Odometry"),
            Some(ArchetypeKind::Odometry)
        );
        assert_eq!(
            classify_schema("sensor_msgs/LaserScan"),
            Some(ArchetypeKind::LaserScan)
        );
        assert_eq!(
            classify_schema("unitree_go/SportModeState"),
            Some(ArchetypeKind::SportModeState)
        );
        // The low-level joint state is deliberately NOT name-mapped.
        // The old row sent it to the (inert) skeleton, whose
        // 3D-only view made its degraded field dump invisible — strictly less
        // than an unmapped schema of the same shape renders. It now rides the
        // shape ladder; `a_joint_state_bank_rides_the_shape_ladder_to_plots`
        // below is the behavioural half of this pin.
        assert_eq!(classify_schema("unitree_go/LowState"), None);
        // The well-known ROS element-array types. NAMED, not inferred,
        // because an EMPTY array carries no element shape and a topic's layout is
        // resolved once from its first decodable frame — an idle nav2 `/plan`
        // (overwhelmingly likely at attach time) would otherwise be frozen into the
        // view-less archetype forever.
        assert_eq!(
            classify_schema("nav_msgs/Path"),
            Some(ArchetypeKind::Path3D)
        );
        assert_eq!(
            classify_schema("geometry_msgs/Polygon"),
            Some(ArchetypeKind::Path3D)
        );
        assert_eq!(
            classify_schema("geometry_msgs/PolygonStamped"),
            Some(ArchetypeKind::Path3D)
        );
        assert_eq!(
            classify_schema("geometry_msgs/PoseArray"),
            Some(ArchetypeKind::PoseArray3D)
        );
        assert_eq!(
            classify_schema("nav_msgs/GridCells"),
            Some(ArchetypeKind::PoseArray3D)
        );
        assert_eq!(
            classify_schema("vision_msgs/Detection3DArray"),
            Some(ArchetypeKind::Boxes3D)
        );
        // MarkerArray resolves by NAME to its own archetype (see the test
        // below for why the mapping — not shape inference — must own it).
        assert_eq!(
            classify_schema("visualization_msgs/MarkerArray"),
            Some(ArchetypeKind::MarkerArray)
        );
        // Unmapped → None (the caller then tries shape inference).
        assert_eq!(classify_schema("std_msgs/String"), None);
        assert_eq!(classify_schema("acme/Widget"), None);
        assert_eq!(classify_schema(""), None);
    }

    // ---- shape-inference oracle (the generalizes-to-unseen-robots claim) ----

    fn f64f(name: &str, v: f64) -> NamedValue<'static> {
        NamedValue {
            name: name.to_string(),
            value: FrameValueKind::F64(v),
        }
    }

    fn nested_f(name: &str, inner: FrameValue<'static>) -> NamedValue<'static> {
        NamedValue {
            name: name.to_string(),
            value: FrameValueKind::Nested(Box::new(inner)),
        }
    }

    fn xyz(x: f64, y: f64, z: f64) -> FrameValue<'static> {
        FrameValue {
            schema_name: "x/Vec3".to_string(),
            fields: vec![f64f("x", x), f64f("y", y), f64f("z", z)],
        }
    }

    fn fv(schema: &str, fields: Vec<NamedValue<'static>>) -> FrameValue<'static> {
        FrameValue {
            schema_name: schema.to_string(),
            fields,
        }
    }

    #[test]
    fn infer_pose_shape_to_transform3d() {
        let pose = fv(
            "unknown/Waypoint",
            vec![
                nested_f("position", xyz(1.0, 2.0, 3.0)),
                nested_f(
                    "orientation",
                    fv(
                        "x/Quat",
                        vec![
                            f64f("x", 0.0),
                            f64f("y", 0.0),
                            f64f("z", 0.0),
                            f64f("w", 1.0),
                        ],
                    ),
                ),
            ],
        );
        assert_eq!(
            infer_archetype_from_shape(&pose),
            ArchetypeKind::Transform3D
        );
    }

    #[test]
    fn infer_bare_xyz_and_quaternion_both_to_scalars() {
        // Bare {x,y,z} → three scalar plots, NOT a 3D point: an unmapped xyz
        // topic is usually a velocity / force / RPY (real positions arrive via
        // the name-mapped Point / Pose types), and a velocity drawn as a point
        // is a misleading dot at the origin.
        let v = xyz(1.0, 2.0, 3.0);
        assert_eq!(infer_archetype_from_shape(&v), ArchetypeKind::Scalars);
        assert_eq!(
            inferred_scalars(&v),
            vec![
                ("x".to_string(), 1.0),
                ("y".to_string(), 2.0),
                ("z".to_string(), 3.0),
            ]
        );
        // {x,y,z,w} has FOUR numerics → also a scalar bag (never a misleading
        // point).
        let quat = fv(
            "x/Quat",
            vec![
                f64f("x", 0.0),
                f64f("y", 0.0),
                f64f("z", 0.0),
                f64f("w", 1.0),
            ],
        );
        assert_eq!(infer_archetype_from_shape(&quat), ArchetypeKind::Scalars);
    }

    #[test]
    fn infer_transform_shape_to_transform3d() {
        // {translation, rotation} (a bare geometry_msgs/Transform) infers to a
        // moving Transform3D, exactly like a {position, orientation} pose.
        let tf = fv(
            "unknown/Frame",
            vec![
                nested_f("translation", xyz(1.0, 2.0, 3.0)),
                nested_f(
                    "rotation",
                    fv(
                        "x/Quat",
                        vec![
                            f64f("x", 0.0),
                            f64f("y", 0.0),
                            f64f("z", 0.0),
                            f64f("w", 1.0),
                        ],
                    ),
                ),
            ],
        );
        assert_eq!(infer_archetype_from_shape(&tf), ArchetypeKind::Transform3D);
    }

    #[test]
    fn infer_twist_shape_and_numeric_bag_to_scalars() {
        // Nested {linear, angular} → Scalars even with no top-level numerics.
        let twist = fv(
            "unknown/Cmd",
            vec![
                nested_f("linear", xyz(1.0, 0.0, 0.0)),
                nested_f("angular", xyz(0.0, 0.0, 0.5)),
            ],
        );
        assert_eq!(infer_archetype_from_shape(&twist), ArchetypeKind::Scalars);
        // Any top-level numeric telemetry → Scalars (the big win).
        let telemetry = fv(
            "unknown/Battery",
            vec![
                f64f("voltage", 12.4),
                f64f("current", 3.1),
                f64f("temp", 40.0),
            ],
        );
        assert_eq!(
            infer_archetype_from_shape(&telemetry),
            ArchetypeKind::Scalars
        );
    }

    #[test]
    fn infer_image_shape_to_image() {
        let img = fv(
            "unknown/Frame",
            vec![
                NamedValue {
                    name: "height".to_string(),
                    value: FrameValueKind::U32(2),
                },
                NamedValue {
                    name: "width".to_string(),
                    value: FrameValueKind::U32(2),
                },
                NamedValue {
                    name: "encoding".to_string(),
                    value: FrameValueKind::Str("rgb8"),
                },
                NamedValue {
                    name: "data".to_string(),
                    value: FrameValueKind::Bytes(&[0u8; 12]),
                },
            ],
        );
        // Image-shape wins over the numeric-bag it would otherwise be
        // (height/width are numerics).
        assert_eq!(infer_archetype_from_shape(&img), ArchetypeKind::Image);
    }

    #[test]
    fn infer_single_string_to_textlog() {
        let s = fv(
            "std_msgs/String",
            vec![NamedValue {
                name: "data".to_string(),
                value: FrameValueKind::Str("hello"),
            }],
        );
        assert_eq!(infer_archetype_from_shape(&s), ArchetypeKind::TextLog);
    }

    // ---- The gap classes that used to fall to the text dump --------

    fn quat_f(x: f64, y: f64, z: f64, w: f64) -> FrameValue<'static> {
        fv(
            "geometry_msgs/Quaternion",
            vec![f64f("x", x), f64f("y", y), f64f("z", z), f64f("w", w)],
        )
    }

    fn pose_f(pos: [f64; 3], rot: [f64; 4]) -> FrameValue<'static> {
        fv(
            "geometry_msgs/Pose",
            vec![
                nested_f("position", xyz(pos[0], pos[1], pos[2])),
                nested_f("orientation", quat_f(rot[0], rot[1], rot[2], rot[3])),
            ],
        )
    }

    #[test]
    fn infer_nested_numeric_payloads_to_scalars() {
        // Every one of these SHAPES fell to the AnyValues text dump before
        // (the harvest only knew top-level numerics + a `linear`/`angular`
        // twist). Each is a real ROS common-interfaces message the walker decodes.
        // geometry_msgs/Wrench — {force, torque} (F/T sensors, manipulation).
        let wrench = fv(
            "geometry_msgs/Wrench",
            vec![
                nested_f("force", xyz(1.0, 0.0, 0.0)),
                nested_f("torque", xyz(0.0, 0.0, 0.5)),
            ],
        );
        assert_eq!(infer_archetype_from_shape(&wrench), ArchetypeKind::Scalars);
        // sensor_msgs/MagneticField — {header, magnetic_field}.
        let mag = fv(
            "sensor_msgs/MagneticField",
            vec![nested_f("magnetic_field", xyz(1.0, 2.0, 3.0))],
        );
        assert_eq!(infer_archetype_from_shape(&mag), ArchetypeKind::Scalars);
        // geometry_msgs/Vector3Stamped — a payload one hop under `vector`.
        let vs = fv(
            "geometry_msgs/Vector3Stamped",
            vec![nested_f("vector", xyz(0.1, 0.2, 0.3))],
        );
        assert_eq!(infer_archetype_from_shape(&vs), ArchetypeKind::Scalars);
        // sensor_msgs/JointState — the joint bank lives in numeric ARRAYS.
        const POSITIONS: [u8; 16] = [0, 0, 0, 0, 0, 0, 224, 63, 0, 0, 0, 0, 0, 0, 208, 191];
        let js = fv(
            "sensor_msgs/JointState",
            vec![NamedValue {
                name: "position".to_string(),
                value: FrameValueKind::PrimArray(cerulion_core::codegen::PrimArray {
                    elem: cerulion_core::codegen::PrimType::F64,
                    bytes: &POSITIONS,
                    count: 2,
                }),
            }],
        );
        assert_eq!(infer_archetype_from_shape(&js), ArchetypeKind::Scalars);
    }

    #[test]
    fn infer_box_shape_to_boxes3d_before_the_pose_rules() {
        // vision_msgs/BoundingBox3D — {center: Pose, size: Vector3}.
        let bb = fv(
            "vision_msgs/BoundingBox3D",
            vec![
                nested_f("center", pose_f([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])),
                nested_f("size", xyz(0.4, 0.4, 1.8)),
            ],
        );
        assert_eq!(infer_archetype_from_shape(&bb), ArchetypeKind::Boxes3D);
        // moveit_msgs/OrientedBoundingBox — {pose, extents}. The box rule MUST
        // win over the pose rule (which would reach the same pose through the
        // one-hop `pose` wrapper and lose the size entirely).
        let obb = fv(
            "moveit_msgs/OrientedBoundingBox",
            vec![
                nested_f("pose", pose_f([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])),
                nested_f("extents", xyz(1.0, 2.0, 3.0)),
            ],
        );
        assert_eq!(infer_archetype_from_shape(&obb), ArchetypeKind::Boxes3D);
        // A pose with NO size stays a Transform3D (anti-tautology: the box rule
        // is not swallowing every pose).
        assert_eq!(
            infer_archetype_from_shape(&pose_f([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])),
            ArchetypeKind::Transform3D
        );
    }

    #[test]
    fn infer_planar_pose_and_orientation_only_to_transform3d() {
        // geometry_msgs/Pose2D — {x, y, theta}: a ground robot's pose MOVES its
        // frame instead of plotting as three disconnected numbers.
        let p2 = fv(
            "geometry_msgs/Pose2D",
            vec![f64f("x", 1.0), f64f("y", 2.0), f64f("theta", 0.5)],
        );
        assert_eq!(infer_archetype_from_shape(&p2), ArchetypeKind::Transform3D);
        // geometry_msgs/QuaternionStamped — orientation only.
        let qs = fv(
            "geometry_msgs/QuaternionStamped",
            vec![nested_f("quaternion", quat_f(0.0, 0.0, 0.0, 1.0))],
        );
        assert_eq!(infer_archetype_from_shape(&qs), ArchetypeKind::Transform3D);
    }

    #[test]
    fn infer_named_position_to_point3d_and_translation_only_to_transform3d() {
        // A NAMED `position` with no orientation is a POINT (not a frame, not a
        // scalar bag) — earlier this was a text dump.
        let waypoint = fv(
            "acme/Waypoint",
            vec![nested_f("position", xyz(1.0, 2.0, 3.0))],
        );
        assert_eq!(
            infer_archetype_from_shape(&waypoint),
            ArchetypeKind::Point3D
        );
        // A `translation` with no rotation is still a rigid motion.
        let shift = fv(
            "acme/Shift",
            vec![nested_f("translation", xyz(1.0, 0.0, 0.0))],
        );
        assert_eq!(
            infer_archetype_from_shape(&shift),
            ArchetypeKind::Transform3D
        );
    }

    // ---- Spatial + SIBLING telemetry ----------------------------------------
    //
    // The regression these pin: an inferred spatial shape used to classify to the
    // bare Transform3D / Point3D, whose render arms log ONLY the spatial
    // primitive — so every sibling number was dropped with no plot view. The old
    // fixtures carried ONLY the spatial field, so the drop was unpinned in BOTH
    // directions (pure AND mixed).

    #[test]
    fn position_only_shape_with_sibling_telemetry_keeps_its_numbers() {
        // THE headline case (acme/DroneState, and the real mavros_msgs/
        // PositionTarget): a named `position` plus real telemetry. A bare spatial
        // classification (Point3D) renders ONE dot — `velocity/*` and `battery_v`
        // vanish, while the SAME message classified Scalars plots
        // them, so the drop is a regression, not a missing feature.
        let drone = fv(
            "acme/DroneState",
            vec![
                nested_f("position", xyz(1.0, 2.0, 3.0)),
                nested_f("velocity", xyz(0.5, 0.0, -0.25)),
                f64f("battery_v", 12.4),
            ],
        );
        assert_eq!(
            infer_archetype_from_shape(&drone),
            ArchetypeKind::Point3DWithScalars,
            "a position-shaped message carrying telemetry must keep BOTH"
        );
        // Hand oracle: the four sibling numbers, in declaration order, with the
        // consumed `position` EXCLUDED (its components are the rendered point,
        // never duplicate plot series).
        let (samples, skipped) = spatial_sibling_series(&drone);
        assert_eq!(
            samples,
            vec![
                ("velocity/x".to_string(), 0.5),
                ("velocity/y".to_string(), 0.0),
                ("velocity/z".to_string(), -0.25),
                ("battery_v".to_string(), 12.4),
            ]
        );
        assert!(skipped.is_empty(), "nothing was skipped: {skipped:?}");
        // The PURE twin stays one-view: a bare `{position}` must NOT be given a
        // plot panel with nothing in it.
        let pure = fv(
            "acme/Waypoint",
            vec![nested_f("position", xyz(1.0, 2.0, 3.0))],
        );
        assert_eq!(infer_archetype_from_shape(&pure), ArchetypeKind::Point3D);
        assert!(spatial_sibling_series(&pure).0.is_empty());
    }

    #[test]
    fn orientation_only_shape_with_sibling_telemetry_keeps_its_numbers() {
        // The orientation-only path's twin gap: an attitude message that also
        // reports signal quality / temperature. Without sibling telemetry: one
        // rotation, both numbers gone.
        let attitude = fv(
            "acme/AttitudeReport",
            vec![
                nested_f("orientation", quat_f(0.0, 0.0, 0.0, 1.0)),
                f64f("yaw_rate", 0.75),
                f64f("temperature_c", 41.5),
            ],
        );
        assert_eq!(
            infer_archetype_from_shape(&attitude),
            ArchetypeKind::Transform3DWithScalars
        );
        let (samples, _) = spatial_sibling_series(&attitude);
        assert_eq!(
            samples,
            vec![
                ("yaw_rate".to_string(), 0.75),
                ("temperature_c".to_string(), 41.5),
            ],
            "the quaternion's own x/y/z/w are the ROTATION, never four extra series"
        );
        // The PURE twin (QuaternionStamped) keeps the one-view Transform3D.
        let qs = fv(
            "geometry_msgs/QuaternionStamped",
            vec![nested_f("quaternion", quat_f(0.0, 0.0, 0.0, 1.0))],
        );
        assert_eq!(infer_archetype_from_shape(&qs), ArchetypeKind::Transform3D);
        assert!(spatial_sibling_series(&qs).0.is_empty());
    }

    #[test]
    fn full_pose_and_planar_shapes_with_siblings_keep_their_numbers() {
        // A full `{position, orientation}` pose plus telemetry (the shape a
        // custom `{pose, velocity}` odometry-like message takes).
        let tracked = fv(
            "acme/TrackedTarget",
            vec![
                nested_f("position", xyz(1.0, 2.0, 3.0)),
                nested_f("orientation", quat_f(0.0, 0.0, 0.0, 1.0)),
                f64f("confidence", 0.9),
            ],
        );
        assert_eq!(
            infer_archetype_from_shape(&tracked),
            ArchetypeKind::Transform3DWithScalars
        );
        assert_eq!(
            spatial_sibling_series(&tracked).0,
            vec![("confidence".to_string(), 0.9)]
        );
        // A PLANAR `{x, y, theta}` pose plus a speed reading: the three fields the
        // yaw-lift consumed are excluded, `speed` survives.
        let ground = fv(
            "acme/GroundPose",
            vec![
                f64f("x", 1.0),
                f64f("y", 2.0),
                f64f("theta", 0.5),
                f64f("speed", 0.3),
            ],
        );
        assert_eq!(
            infer_archetype_from_shape(&ground),
            ArchetypeKind::Transform3DWithScalars
        );
        assert_eq!(
            spatial_sibling_series(&ground).0,
            vec![("speed".to_string(), 0.3)]
        );
        // The bare `geometry_msgs/Pose2D` stays PURE (one view).
        let p2 = fv(
            "geometry_msgs/Pose2D",
            vec![f64f("x", 1.0), f64f("y", 2.0), f64f("theta", 0.5)],
        );
        assert_eq!(infer_archetype_from_shape(&p2), ArchetypeKind::Transform3D);
        assert!(spatial_sibling_series(&p2).0.is_empty());
    }

    #[test]
    fn a_sibling_inside_the_pose_wrapper_is_not_swallowed() {
        // The one-level-down variant of the same bug: a `{pose: {...}}` wrapper
        // whose inner value carries telemetry beside the pose. Excluding the
        // whole `pose` wrapper (rather than the two fields the transform read)
        // would drop `pose/confidence` just as silently.
        let stamped = fv(
            "acme/PoseWithConfidence",
            vec![nested_f(
                "pose",
                fv(
                    "acme/InnerPose",
                    vec![
                        nested_f("position", xyz(1.0, 2.0, 3.0)),
                        nested_f("orientation", quat_f(0.0, 0.0, 0.0, 1.0)),
                        f64f("confidence", 0.42),
                    ],
                ),
            )],
        );
        assert_eq!(
            infer_archetype_from_shape(&stamped),
            ArchetypeKind::Transform3DWithScalars
        );
        assert_eq!(
            spatial_sibling_series(&stamped).0,
            vec![("pose/confidence".to_string(), 0.42)]
        );
    }

    #[test]
    fn a_header_is_not_sibling_telemetry() {
        // The anti-tautology guard for the split: a `{header, pose}` message is
        // PURELY spatial. The header's sec/nanosec are the frame's own clock
        // (already the plot X axis), so they must not fabricate "telemetry" and
        // hand every stamped pose an empty plot panel.
        let stamped = fv(
            "geometry_msgs/PoseStamped",
            vec![
                nested_f(
                    "header",
                    fv(
                        "std_msgs/Header",
                        vec![f64f("sec", 12.0), f64f("nanosec", 34.0)],
                    ),
                ),
                nested_f(
                    "pose",
                    fv(
                        "geometry_msgs/Pose",
                        vec![
                            nested_f("position", xyz(1.0, 2.0, 3.0)),
                            nested_f("orientation", quat_f(0.0, 0.0, 0.0, 1.0)),
                        ],
                    ),
                ),
            ],
        );
        assert_eq!(
            infer_archetype_from_shape(&stamped),
            ArchetypeKind::Transform3D,
            "a stamped pose is PURE spatial — the header is not telemetry"
        );
        let (samples, skipped) = spatial_sibling_series(&stamped);
        assert!(samples.is_empty(), "got {samples:?}");
        assert!(skipped.is_empty(), "got {skipped:?}");
    }

    #[test]
    fn a_covariance_beside_a_pose_stays_one_view_but_is_reported() {
        // A `{pose, covariance[36]}` carries numbers we deliberately never plot.
        // Electing the both-views archetype for it would create a time_series
        // view with ZERO series (the empty-panel failure mode), so it stays a
        // plain Transform3D — but the covariance is still REPORTED (the render
        // arm calls the same helper), never dropped in silence.
        let cov_bytes: Vec<u8> = (0..36u32).flat_map(|i| (i as f64).to_le_bytes()).collect();
        let leaked: &'static [u8] = Box::leak(cov_bytes.into_boxed_slice());
        let with_cov = fv(
            "acme/PoseWithCovariance",
            vec![
                nested_f(
                    "pose",
                    fv(
                        "geometry_msgs/Pose",
                        vec![
                            nested_f("position", xyz(1.0, 2.0, 3.0)),
                            nested_f("orientation", quat_f(0.0, 0.0, 0.0, 1.0)),
                        ],
                    ),
                ),
                NamedValue {
                    name: "covariance".to_string(),
                    value: FrameValueKind::PrimArray(cerulion_core::codegen::PrimArray {
                        elem: cerulion_core::codegen::PrimType::F64,
                        bytes: leaked,
                        count: 36,
                    }),
                },
            ],
        );
        assert_eq!(
            infer_archetype_from_shape(&with_cov),
            ArchetypeKind::Transform3D,
            "no plottable sibling ⇒ no plot view (never an empty panel)"
        );
        let (samples, skipped) = spatial_sibling_series(&with_cov);
        assert!(
            samples.is_empty(),
            "a covariance is not a plot: {samples:?}"
        );
        assert_eq!(
            skipped,
            vec![crate::archetype::SkippedSeries::Covariance {
                field: "covariance".to_string(),
                count: 36,
            }],
            "the skip is REPORTED, not silent"
        );
    }

    /// THE freeze pin: the archetype must be a function of
    /// the SHAPE, not of whichever frame happened to arrive first.
    ///
    /// Every consumer resolves a topic's layout ONCE — `cerulion-vizd` reads the
    /// archetype from the first decodable frame and keeps it — so a
    /// value-dependent split silently froze a topic whose first frame's array was
    /// empty into the VIEW-LESS twin. Its later frames still logged
    /// `joint_positions/*` scalars, into a `time_series` view that did not exist:
    /// invisible telemetry, permanently, on exactly the never-seen-robot path
    /// this rule exists for. A value-dependent split classifies these two frames
    /// DIFFERENTLY (`Point3D` vs `Point3DWithScalars`).
    #[test]
    fn an_empty_array_in_the_first_frame_does_not_freeze_out_the_plot_view() {
        let bytes: Vec<u8> = [0.1f64, 0.2].iter().flat_map(|v| v.to_le_bytes()).collect();
        let leaked: &'static [u8] = Box::leak(bytes.into_boxed_slice());
        let arm = |data: &'static [u8], count: usize| {
            fv(
                "acme/ArmState",
                vec![
                    nested_f("position", xyz(1.0, 2.0, 3.0)),
                    NamedValue {
                        name: "joint_positions".to_string(),
                        value: FrameValueKind::PrimArray(cerulion_core::codegen::PrimArray {
                            elem: cerulion_core::codegen::PrimType::F64,
                            bytes: data,
                            count,
                        }),
                    },
                ],
            )
        };
        let first_frame_empty = arm(&[], 0);
        let later_frame_full = arm(leaked, 2);
        assert_eq!(
            infer_archetype_from_shape(&first_frame_empty),
            ArchetypeKind::Point3DWithScalars,
            "an EMPTY declared array is still a plotting field — the topic needs its view"
        );
        assert_eq!(
            infer_archetype_from_shape(&later_frame_full),
            infer_archetype_from_shape(&first_frame_empty),
            "the SAME schema must classify identically whatever the array's length"
        );
        // The view the classification earns is the one the later frames' series
        // land in (the whole point of not freezing the view-less twin).
        assert!(
            crate::blueprint::views_for_archetype(ArchetypeKind::Point3DWithScalars)
                .contains(&crate::blueprint::ViewKind::TimeSeries),
            "the WithScalars twin is what carries the plot view"
        );
        assert!(
            !crate::blueprint::views_for_archetype(ArchetypeKind::Point3D)
                .contains(&crate::blueprint::ViewKind::TimeSeries),
            "…and the plain twin does not — so the freeze really did lose the plots"
        );
        // The empty first frame plots nothing YET (no fabricated samples), and the
        // later frame's series land in the view the first frame already earned.
        assert!(spatial_sibling_series(&first_frame_empty).0.is_empty());
        assert_eq!(
            spatial_sibling_series(&later_frame_full).0,
            vec![
                ("joint_positions/0".to_string(), 0.1),
                ("joint_positions/1".to_string(), 0.2),
            ]
        );
    }

    /// The same frame-invariance one rule further down: a `Scalars` topic whose
    /// first frame's array is empty must not freeze into the field-dump / text
    /// fallback, which renders its later joint bank as a text document forever.
    /// The ANTI-TAUTOLOGY halves are in the same body: a shape with NO declared
    /// numeric field at all still falls through, and an array permanently OVER
    /// the plot cap still does NOT earn a (permanently empty) plot view.
    #[test]
    fn a_scalars_topic_is_classified_by_shape_not_by_the_first_frames_values() {
        let empty_bank = fv(
            "acme/JointBank",
            vec![
                NamedValue {
                    name: "name".to_string(),
                    value: FrameValueKind::Str("arm"),
                },
                NamedValue {
                    name: "position".to_string(),
                    value: FrameValueKind::PrimArray(cerulion_core::codegen::PrimArray {
                        elem: cerulion_core::codegen::PrimType::F64,
                        bytes: &[],
                        count: 0,
                    }),
                },
            ],
        );
        assert_eq!(
            infer_archetype_from_shape(&empty_bank),
            // This fixture carries a SCALAR `Str` beside the numeric
            // bank — a scalar-string telemetry bank — so it elects the TEXT TWIN.
            // It is NOT `sensor_msgs/JointState`'s shape: real JointState declares
            // `string[] name`, which this predicate deliberately does NOT count
            // (see `declares_text_fields`), so the most-published ROS topic there
            // is stays plots-only. That case is pinned over a REAL JointState frame
            // by `sink_dispatch_test::a_real_joint_state_frame_stays_plots_only`.
            //
            // The frame-invariance claim under test is unchanged and is what the
            // assertion still proves: an EMPTY declared array is a plot topic
            // either way (the earlier failure was falling through to
            // `TextLog`/`AnyValues`, which neither of these is).
            ArchetypeKind::ScalarsWithText,
            "a declared joint bank is a plot topic even on the frame where it is empty"
        );
        // The SAME invariance on the text-free path, so the claim is not carried
        // by the twin alone: drop the `name` string and the empty bank still
        // classifies as the plots-only `Scalars`.
        let empty_bank_no_text = fv(
            "acme/JointBankNoName",
            vec![NamedValue {
                name: "position".to_string(),
                value: FrameValueKind::PrimArray(cerulion_core::codegen::PrimArray {
                    elem: cerulion_core::codegen::PrimType::F64,
                    bytes: &[],
                    count: 0,
                }),
            }],
        );
        assert_eq!(
            infer_archetype_from_shape(&empty_bank_no_text),
            ArchetypeKind::Scalars,
            "a text-free declared bank is a plot topic even on the frame where it is empty"
        );
        // Anti-tautology 1: nothing numeric declared ⇒ still not a plot topic.
        let just_text = fv(
            "acme/Status",
            vec![NamedValue {
                name: "message".to_string(),
                value: FrameValueKind::Str("ok"),
            }],
        );
        assert_eq!(
            infer_archetype_from_shape(&just_text),
            ArchetypeKind::TextLog
        );
        // Anti-tautology 2: an array OVER the plot cap is never expanded, so it
        // must NOT earn a view it can only render empty (it is reported instead).
        let over_cap: Vec<u8> = (0..(MAX_ARRAY_SERIES as u32 + 1))
            .flat_map(|i| (i as f64).to_le_bytes())
            .collect();
        let leaked: &'static [u8] = Box::leak(over_cap.into_boxed_slice());
        let huge = fv(
            "acme/Scan",
            vec![NamedValue {
                name: "bins".to_string(),
                value: FrameValueKind::PrimArray(cerulion_core::codegen::PrimArray {
                    elem: cerulion_core::codegen::PrimType::F64,
                    bytes: leaked,
                    count: MAX_ARRAY_SERIES + 1,
                }),
            }],
        );
        assert_eq!(
            infer_archetype_from_shape(&huge),
            ArchetypeKind::AnyValues,
            "an over-cap array is never plotted, so it never earns an empty plot panel"
        );
    }

    #[test]
    fn standing_rulings_survive_the_widened_inference() {
        // The bare {x,y,z} scalar-bag decision (a velocity / force / RPY drawn as a
        // point would be a misleading dot at the origin).
        assert_eq!(
            infer_archetype_from_shape(&xyz(1.0, 2.0, 3.0)),
            ArchetypeKind::Scalars
        );
        // The bare {x,y,z,w} four-series decision (never inferred as a rotation —
        // only a field NAMED like an orientation is treated as one).
        assert_eq!(
            infer_archetype_from_shape(&quat_f(0.0, 0.0, 0.0, 1.0)),
            ArchetypeKind::Scalars
        );
        // A lone bool is still NOT auto-plotted (it stays an inspectable dump).
        let b = fv(
            "std_msgs/Bool",
            vec![NamedValue {
                name: "data".to_string(),
                value: FrameValueKind::Bool(true),
            }],
        );
        assert_eq!(infer_archetype_from_shape(&b), ArchetypeKind::AnyValues);
    }

    #[test]
    fn occupancy_grid_is_a_named_mapping_not_a_dump() {
        assert_eq!(
            classify_schema("nav_msgs/OccupancyGrid"),
            Some(ArchetypeKind::OccupancyGrid)
        );
        // The occupancy grid's VALUE semantics are not shape-derivable, so an
        // unnamed look-alike deliberately does NOT infer to a grid (it takes the
        // plain generic route — here the numeric bag its `info` dimensions form).
        let lookalike = fv(
            "acme/GridLike",
            vec![
                nested_f(
                    "info",
                    fv(
                        "acme/Meta",
                        vec![
                            NamedValue {
                                name: "width".to_string(),
                                value: FrameValueKind::U32(2),
                            },
                            NamedValue {
                                name: "height".to_string(),
                                value: FrameValueKind::U32(2),
                            },
                        ],
                    ),
                ),
                NamedValue {
                    name: "data".to_string(),
                    value: FrameValueKind::Bytes(&[0u8; 4]),
                },
            ],
        );
        assert_ne!(
            infer_archetype_from_shape(&lookalike),
            ArchetypeKind::OccupancyGrid
        );
    }

    #[test]
    fn infer_no_match_falls_back_to_anyvalues() {
        // A lone bool is neither numeric nor a string → the field-dump fallback
        // (bools/enums are not auto-plotted).
        let b = fv(
            "std_msgs/Bool",
            vec![NamedValue {
                name: "data".to_string(),
                value: FrameValueKind::Bool(true),
            }],
        );
        assert_eq!(infer_archetype_from_shape(&b), ArchetypeKind::AnyValues);
        // An empty message dumps too.
        assert_eq!(
            infer_archetype_from_shape(&fv("empty/Msg", vec![])),
            ArchetypeKind::AnyValues
        );
    }

    // ---- the skipped-series diagnostic -------------------

    /// The fields a plot-kind topic did not expand, as the harvest reports them.
    fn skips() -> Vec<SkippedSeries> {
        vec![
            SkippedSeries::Oversized {
                field: "ranges".to_string(),
                count: 1081,
            },
            SkippedSeries::Covariance {
                field: "pose/covariance".to_string(),
                count: 36,
            },
            SkippedSeries::TooDeep {
                field: "a/b/c/deep".to_string(),
            },
        ]
    }

    #[tracing_test::traced_test]
    #[test]
    fn report_skipped_series_names_every_field_and_claims_no_view() {
        // The copy must not end "Every value
        // stays visible in the topic's field dump" — a promise a Scalars topic
        // cannot keep: a plot-kind topic renders time series only, so the skipped
        // data would be invisible in the viewer and the operator sent hunting for a
        // panel that does not exist.
        //
        // The withheld-series election keeps that contract for the kinds it still describes; a
        // `Transform3DWithScalars` renders a pose and plots, never a dump — which
        // is why this arm now drives a kind whose views carry NO text document.
        let mut state = SinkState::new();
        report_skipped_series(
            "scan",
            "acme/Telemetry",
            &skips(),
            ArchetypeKind::Transform3DWithScalars,
            &mut state,
        );

        // (1) Every skipped field is NAMED, with its reason.
        assert!(logs_contain("ranges (1081 elements"), "names the array");
        assert!(
            logs_contain("pose/covariance (36 elements, a covariance matrix)"),
            "names the covariance"
        );
        assert!(
            logs_contain("a/b/c/deep (nested deeper than the 3-hop harvest limit)"),
            "names the over-depth nested"
        );
        // (2) It claims NO rendering that does not exist — the accuracy contract.
        assert!(
            !logs_contain("field dump"),
            "a plot-kind topic renders no field dump; the copy must not promise one"
        );
        assert!(
            logs_contain("NOT rendered anywhere else for this topic"),
            "the copy must say plainly that these values are not shown"
        );
        // (3) It points somewhere REAL for the values.
        assert!(
            logs_contain("cerulion topic echo"),
            "the remediation must be an actual way to read the values"
        );
    }

    /// The OTHER half of the same accuracy contract. A topic elected to
    /// `ScalarsWithText` DOES render a dump, so the copy — "NOT rendered
    /// anywhere else … read them with `cerulion topic echo`" — would be actively
    /// misleading on exactly the topics this election exists for: it would send an
    /// operator out of the viewer, away from the pane that now holds the answer.
    ///
    /// Both directions are asserted (the line must NOT keep the plots-only tail),
    /// and the `echo` pointer must SURVIVE, because the dump is bounded and is a
    /// window onto the withheld fields rather than a replacement for them.
    #[tracing_test::traced_test]
    #[test]
    fn report_skipped_series_points_at_the_dump_when_the_topic_renders_one() {
        let mut state = SinkState::new();
        report_skipped_series(
            "lowstate",
            "unitree_go/LowState",
            &skips(),
            ArchetypeKind::ScalarsWithText,
            &mut state,
        );

        assert!(
            logs_contain("field dump (the text pane beside the plot) shows"),
            "an elected topic's copy must point at the pane that shows them"
        );
        assert!(
            !logs_contain("NOT rendered anywhere else for this topic"),
            "the plots-only tail is FALSE for a topic that renders a dump"
        );
        // The copy must name the bounds that actually BIND, not just
        // the two array previews. The dump's DEPTH and LINE ceilings are why the
        // election is narrowed, and an operator who does not find their field
        // needs to know a ceiling exists rather than concluding the pane is broken.
        assert!(
            logs_contain("BOUNDED"),
            "the copy must not oversell the dump"
        );
        for bound in ["first elements of each array", "4 hops", "200 lines"] {
            assert!(
                logs_contain(bound),
                "the copy must name the bound {bound:?} — it is one of the three \
                 that can hide a withheld field"
            );
        }
        assert!(
            logs_contain("cerulion topic echo"),
            "the COMPLETE values still need the wire"
        );
        // The archetype rides the line, so an operator can tell WHICH contract
        // this topic got without re-deriving the classification.
        assert!(
            logs_contain("ScalarsWithText"),
            "the line names the archetype it is describing"
        );
    }

    #[tracing_test::traced_test]
    #[test]
    fn report_skipped_series_is_silent_when_nothing_was_skipped() {
        // The anti-flood / anti-noise arm: a topic whose numbers ALL plot says
        // nothing at all.
        let mut state = SinkState::new();
        report_skipped_series(
            "scan",
            "acme/Clean",
            &[],
            ArchetypeKind::Scalars,
            &mut state,
        );
        assert!(
            !logs_contain("acme/Clean"),
            "no skips ⇒ no diagnostic at all"
        );
    }

    #[tracing_test::traced_test]
    #[test]
    fn report_skipped_series_is_once_per_schema_not_per_frame() {
        // The flood guard: the sink calls this on EVERY frame of a plot topic, so
        // the diagnostic must latch per schema — while a DIFFERENT schema still
        // gets its own line (the latch is not a global "reported once ever").
        let mut state = SinkState::new();
        for _ in 0..5 {
            report_skipped_series(
                "scan",
                "acme/Telemetry",
                &skips(),
                ArchetypeKind::Scalars,
                &mut state,
            );
        }
        logs_assert(|lines: &[&str]| {
            let n = lines
                .iter()
                .filter(|l| l.contains("acme/Telemetry"))
                .count();
            if n == 1 {
                Ok(())
            } else {
                Err(format!("expected exactly 1 line for the schema, got {n}"))
            }
        });
        report_skipped_series(
            "other",
            "acme/Second",
            &skips(),
            ArchetypeKind::Scalars,
            &mut state,
        );
        assert!(
            logs_contain("acme/Second"),
            "a second schema reports independently"
        );
    }

    // ---- The element-array rung of the shape ladder ----------------

    /// The `raw` half of a decoded element array. The ladder
    /// reads only `elements`, so an arbitrary non-empty `raw` is exactly right.
    const RAW_SENTINEL: &[u8] = &[0xAB, 0xCD];

    fn arr(name: &str, elements: Vec<FrameValueKind<'static>>) -> NamedValue<'static> {
        NamedValue {
            name: name.to_string(),
            value: FrameValueKind::NestedArray {
                elements,
                raw: RAW_SENTINEL,
            },
        }
    }

    fn el(inner: FrameValue<'static>) -> FrameValueKind<'static> {
        FrameValueKind::Nested(Box::new(inner))
    }

    fn stamp_hdr() -> NamedValue<'static> {
        nested_f("header", fv("std_msgs/Header", vec![f64f("__unused", 0.0)]))
    }

    fn el_pose(pos: [f64; 3]) -> FrameValue<'static> {
        fv(
            "geometry_msgs/Pose",
            vec![
                nested_f("position", xyz(pos[0], pos[1], pos[2])),
                nested_f(
                    "orientation",
                    fv(
                        "geometry_msgs/Quaternion",
                        vec![
                            f64f("x", 0.0),
                            f64f("y", 0.0),
                            f64f("z", 0.0),
                            f64f("w", 1.0),
                        ],
                    ),
                ),
            ],
        )
    }

    fn pose_stamped_f(pos: [f64; 3]) -> FrameValue<'static> {
        fv(
            "geometry_msgs/PoseStamped",
            vec![stamp_hdr(), nested_f("pose", el_pose(pos))],
        )
    }

    #[test]
    fn infer_stamped_element_array_to_path3d_and_unstamped_to_posearray3d() {
        // The AUTOMAGIC half: unseen VENDOR schemas (nothing name-maps these), so
        // the ELEMENT shape is what decides. A per-element stamp means the elements
        // are samples of one thing over time ⇒ ORDERED ⇒ a polyline.
        let plan = fv(
            "acme/WaypointList",
            vec![arr(
                "waypoints",
                vec![
                    el(pose_stamped_f([0.0, 0.0, 0.0])),
                    el(pose_stamped_f([1.0, 0.0, 0.0])),
                ],
            )],
        );
        assert_eq!(infer_archetype_from_shape(&plan), ArchetypeKind::Path3D);
        // The SHARPEST flip: the same array with the per-element header removed is
        // UNORDERED. One field on the element, two different archetypes.
        let samples = fv(
            "acme/WaypointList",
            vec![arr(
                "waypoints",
                vec![el(el_pose([0.0, 0.0, 0.0])), el(el_pose([1.0, 0.0, 0.0]))],
            )],
        );
        assert_eq!(
            infer_archetype_from_shape(&samples),
            ArchetypeKind::PoseArray3D
        );
    }

    #[test]
    fn infer_box_element_array_to_boxes3d() {
        // An unseen vendor detection list: each element wraps its box in `bbox`.
        let det = |c: [f64; 3]| {
            el(fv(
                "acme/Detection",
                vec![
                    stamp_hdr(),
                    nested_f(
                        "bbox",
                        fv(
                            "acme/Box",
                            vec![
                                nested_f("center", el_pose(c)),
                                nested_f("size", xyz(1.0, 1.0, 1.0)),
                            ],
                        ),
                    ),
                ],
            ))
        };
        let detections = fv(
            "acme/DetectionList",
            vec![arr(
                "detections",
                vec![det([1.0, 2.0, 3.0]), det([4.0, 5.0, 6.0])],
            )],
        );
        assert_eq!(
            infer_archetype_from_shape(&detections),
            ArchetypeKind::Boxes3D
        );
    }

    #[test]
    fn a_big_element_array_never_lands_in_the_numeric_harvest() {
        // The high risk this rung's placement exists to close: a `/plan`'s poses
        // are numbers all the way down, so on the harvest rung a 5 000-pose plan
        // would classify `Scalars` and be handed a time_series view. The element rung
        // sits BEFORE it.
        let big = fv(
            "acme/WaypointList",
            vec![arr(
                "waypoints",
                (0..5_000)
                    .map(|i| el(pose_stamped_f([i as f64, 0.0, 0.0])))
                    .collect(),
            )],
        );
        assert_eq!(infer_archetype_from_shape(&big), ArchetypeKind::Path3D);
        // ANTI-TAUTOLOGY: the rung is not swallowing every array — the SAME shape
        // with nothing spatial in its elements still reaches the dump, and a plain
        // numeric bag still plots.
        let readings = fv(
            "acme/Readings",
            vec![arr(
                "readings",
                vec![el(fv("acme/Reading", vec![f64f("volts", 12.4)]))],
            )],
        );
        assert_eq!(
            infer_archetype_from_shape(&readings),
            ArchetypeKind::AnyValues
        );
        assert_eq!(
            infer_archetype_from_shape(&fv("acme/Battery", vec![f64f("volts", 12.4)])),
            ArchetypeKind::Scalars
        );
    }

    #[test]
    fn an_empty_element_array_does_not_classify_which_is_why_the_named_table_exists() {
        // An empty array carries NO element shape, so inference cannot tell a path
        // from a point bag from a detection set and must not guess — it falls
        // through. This is the exact gap `classify_schema`'s element-array rows close: an
        // idle nav2 `/plan` resolves by NAME and renders from its very first frame.
        let idle = fv(
            "acme/WaypointList",
            vec![NamedValue {
                name: "waypoints".to_string(),
                value: FrameValueKind::NestedArray {
                    elements: vec![],
                    raw: &[],
                },
            }],
        );
        assert_eq!(infer_archetype_from_shape(&idle), ArchetypeKind::AnyValues);
        assert_eq!(
            classify_schema("nav_msgs/Path"),
            Some(ArchetypeKind::Path3D)
        );
    }

    #[test]
    fn the_markerarray_name_mapping_is_load_bearing_not_decoration() {
        // A `Marker` element IS pose-shaped and stamped, so the element rung
        // would classify a MarkerArray `Path3D` and draw a polyline through the
        // origins of a dozen unrelated markers — a WRONG picture. This asserts BOTH
        // halves: inference really would draw it, and the NAME mapping really stops
        // it by routing the frame to the marker kind switch instead. (Before the mapping
        // the same two halves held with `AnyValues` as the destination — a plain
        // text dump; the mapping is what upgraded the destination, and the reason
        // it must stay named is unchanged.)
        let marker = |c: [f64; 3]| {
            el(fv(
                "visualization_msgs/Marker",
                vec![
                    stamp_hdr(),
                    nested_f("pose", el_pose(c)),
                    nested_f("scale", xyz(1.0, 1.0, 1.0)),
                ],
            ))
        };
        let markers = fv(
            "visualization_msgs/MarkerArray",
            vec![arr(
                "markers",
                vec![marker([0.0, 0.0, 0.0]), marker([5.0, 5.0, 0.0])],
            )],
        );
        assert_eq!(
            infer_archetype_from_shape(&markers),
            ArchetypeKind::Path3D,
            "inference WOULD draw a spurious polyline — this is why the hold exists"
        );
        assert_eq!(
            classify_schema(&markers.schema_name),
            Some(ArchetypeKind::MarkerArray),
            "the named mapping must win over inference"
        );
    }

    // ---- coalescing (replacing-kind) oracle ---------------------------------

    #[test]
    fn coalesces_exact_set_oracle() {
        // EVERY ArchetypeKind variant asserted against its hand class: exactly
        // the replacing (overwrite-under-latest-at) kinds coalesce; every
        // per-sample kind does not. The `expected` match below is EXHAUSTIVE with
        // no wildcard arm, so a NEW archetype variant fails to compile until it is
        // classified here on purpose (the earlier two-array form silently
        // ignored new variants).
        use ArchetypeKind as A;
        for kind in A::ALL {
            let expected = match kind {
                // Whole-visual-state snapshots: the next frame REPLACES them.
                A::Points3D
                | A::Image
                | A::LaserScan
                | A::Skeleton
                | A::Boxes3D
                | A::OccupancyGrid
                // A `/plan`'s polyline and a `PoseArray`'s point set are
                // WHOLE geometries at one entity — the next frame REPLACES them.
                | A::Path3D
                | A::PoseArray3D => true,
                // Per-sample kinds: every frame must draw (plots need full
                // temporal resolution; TF/text accumulate).
                //
                // `VideoStream` is the STRONGEST member of this class. A
                // P-frame is a DELTA against its predecessors, so dropping the
                // older sample of a tick does not replace it — it destroys the
                // reference the survivor decodes from. Coalescing video is not a
                // display trade-off, it is corruption.
                //
                // `MarkerArray` is here for a STRONGER reason than the
                // others: coalescing it is DATA LOSS, not merely reduced
                // resolution — the frame thrown away may carry the only DELETE
                // for a marker, leaving a permanent ghost. This oracle is the
                // whole guard, since `coalesces` is a `matches!` whose missing
                // arm defaults to `false` silently; the behavioural twin lives in
                // `marker_array_test::a_marker_array_is_not_coalesced_so_a_delete_in_a_batch_survives`.
                A::MarkerArray
                | A::Transforms
                | A::Scalars
                // The text twin plots per sample exactly as `Scalars`
                // does, and its dump is a rolling latest — coalescing it would
                // drop plot resolution for no gain.
                | A::ScalarsWithText
                | A::Transform3D
                | A::Point3D
                | A::Transform3DWithScalars
                | A::Point3DWithScalars
                | A::Imu
                | A::Odometry
                | A::SportModeState
                | A::TextLog
                | A::VideoStream
                | A::AnyValues => false,
            };
            assert_eq!(
                coalesces(kind),
                expected,
                "{kind:?} coalescing class regressed"
            );
        }
    }

    #[test]
    fn all_is_the_complete_closed_set() {
        use ArchetypeKind as A;
        // COMPLETENESS is no longer a test's job: `ALL` is GENERATED from the
        // enum's own variant list by `declare_archetype_kinds!`, so a variant
        // physically cannot be missing from it (a hand-written
        // `[ArchetypeKind; 15]` literal claims a compile-time guarantee nothing
        // enforces — a variant can escape it).
        //
        // What still needs pinning is DECLARATION ORDER — `ALL` is the iteration
        // order every oracle table and the layout compiler walk, so a reshuffle
        // is a behaviour change. The exhaustive match (no wildcard arm) makes a
        // new variant a compile error here until it is given a position on
        // purpose.
        for kind in A::ALL {
            let pos = match kind {
                A::Points3D => 0,
                A::Image => 1,
                A::Transforms => 2,
                A::Scalars => 3,
                // Declared immediately after its base, like every other
                // twin, so the enum reads as base-then-variant.
                A::ScalarsWithText => 4,
                A::Transform3D => 5,
                A::Point3D => 6,
                A::Transform3DWithScalars => 7,
                A::Point3DWithScalars => 8,
                A::Imu => 9,
                A::Odometry => 10,
                A::LaserScan => 11,
                A::SportModeState => 12,
                A::TextLog => 13,
                A::Boxes3D => 14,
                A::OccupancyGrid => 15,
                A::Skeleton => 16,
                A::Path3D => 17,
                A::PoseArray3D => 18,
                A::VideoStream => 19,
                A::MarkerArray => 20,
                A::AnyValues => 21,
            };
            assert_eq!(A::ALL[pos], kind, "{kind:?} is at the wrong ALL position");
        }
        // The positions above are a bijection onto `ALL`'s entries: 22 distinct
        // positions, 22 distinct wire names, 22 entries (the two element-array
        // variants, the video one and the MarkerArray one were added later, all
        // immediately before the AnyValues catch-all, which
        // stays LAST; `ScalarsWithText` was added beside its `Scalars` base,
        // which is why every later position shifted by one).
        assert_eq!(A::ALL.len(), 22, "a variant was added or removed");
        let names: BTreeSet<&str> = A::ALL
            .iter()
            .map(|k| crate::blueprint::archetype_wire_name(*k))
            .collect();
        assert_eq!(names.len(), A::ALL.len(), "duplicate variant in ALL");
    }

    #[test]
    fn record_coalesced_accumulates_into_total() {
        let mut state = SinkState::new();
        assert_eq!(
            state.coalesced_frames(),
            0,
            "fresh state has coalesced none"
        );
        state.record_coalesced(4);
        state.record_coalesced(3);
        assert_eq!(state.coalesced_frames(), 7, "the total is the running sum");
    }

    #[test]
    fn sweep_ring_assignment_oracle() {
        let mut state = SinkState::new();
        let entity = "world/odom/base/lidar";
        // Slots fill 0, 1, .., 7 in acceptance order (hand-pinned endpoints,
        // loop for the middle of the ring).
        assert_eq!(
            state.next_sweep_entity(entity),
            "world/odom/base/lidar/viz-sweep/0"
        );
        assert_eq!(
            state.next_sweep_entity(entity),
            "world/odom/base/lidar/viz-sweep/1"
        );
        for k in 2..SWEEP_ACCUM_RING {
            assert_eq!(
                state.next_sweep_entity(entity),
                format!("{entity}/{SWEEP_CHILD}/{k}")
            );
        }
        // The 9th accepted sweep WRAPS to slot 0 (the ring recycles the
        // oldest sub-entity — the viewer ages the scene out ring-fast).
        assert_eq!(
            state.next_sweep_entity(entity),
            "world/odom/base/lidar/viz-sweep/0"
        );
        assert_eq!(state.accepted_sweeps(entity), SWEEP_ACCUM_RING + 1);
        // A different entity owns an INDEPENDENT ring (no cross-talk).
        assert_eq!(
            state.next_sweep_entity("world/odom/base/other"),
            "world/odom/base/other/viz-sweep/0"
        );
        assert_eq!(state.accepted_sweeps(entity), SWEEP_ACCUM_RING + 1);
        assert_eq!(state.accepted_sweeps("world/odom/base/other"), 1);
        // An entity with no accepted sweeps reads 0 (no fabricated cursor).
        assert_eq!(state.accepted_sweeps("world/odom/base/untouched"), 0);
    }

    /// `log_topic` is the exact inverse of the trim
    /// `route_key_for_topic` applies, so the render worker's `topic=` key is the
    /// SAME string the resolver logs for the same topic.
    ///
    /// Hand oracles, driven THROUGH `route_key_for_topic` rather than over
    /// hand-written keys — a self-consistent pair of constants would pass even if
    /// the two functions drifted apart, which is precisely the defect (two
    /// spellings of one key) this closes.
    #[test]
    fn log_topic_round_trips_the_route_key_trim() {
        for topic in [
            "/lowstate",
            "/utlidar/cloud",
            "/go2/camera/compressed",
            "/tf_static",
        ] {
            let key = route_key_for_topic(topic, None);
            assert_ne!(key, topic, "the trim must actually bite: {topic}");
            assert_eq!(log_topic(&key), topic, "round trip failed for {topic}");
            // An ENTITY OVERRIDE rides the key after a control char; the logged
            // topic must be the topic half only, still absolute.
            let with_override = route_key_for_topic(topic, Some("world/custom"));
            assert_eq!(log_topic(&with_override), topic);
        }
        // Degenerate inputs are returned untouched — never corrupted, and never
        // turned into a bare `/`.
        assert_eq!(log_topic(""), "");
        assert_eq!(log_topic("/already/absolute"), "/already/absolute");
    }

    #[test]
    fn route_key_for_topic_is_the_whole_topic() {
        // The route key is the topic with its leading/trailing `/`
        // trimmed — NOT its last segment (which is what collapsed 38 of a real
        // robot's 75 topics onto 6 entities). Hand oracles.
        assert_eq!(route_key_for_topic("/utlidar/cloud", None), "utlidar/cloud");
        assert_eq!(route_key_for_topic("/tf", None), "tf");
        assert_eq!(route_key_for_topic("/tf_static", None), "tf_static");
        assert_eq!(route_key_for_topic("/robot/odom", None), "robot/odom");
        assert_eq!(
            route_key_for_topic("/api/vui/response", None),
            "api/vui/response"
        );
        // A bare (already-short) name is its own key.
        assert_eq!(route_key_for_topic("image", None), "image");
    }

    #[test]
    fn route_key_for_topic_composes_into_a_topic_unique_entity() {
        // The derived key, fed to `route_for_input`, yields `world/<topic>`.
        assert_eq!(
            route_for_input(&route_key_for_topic("/utlidar/cloud", None)).entity,
            "world/utlidar/cloud"
        );
        // /tf renders as the transform TREE, so its reported entity is the viz
        // root — nothing is logged at a per-topic entity for a TFMessage.
        let tf = route_for_input(&route_key_for_topic("/tf", None));
        assert_eq!(tf.entity, WORLD_ROOT);
        assert!(!tf.is_static, "/tf is temporal");
        assert!(
            route_for_input(&route_key_for_topic("/robot/odom", None)).drives_robot_root,
            "an odom topic's last segment elects the robot root"
        );
    }

    #[test]
    fn route_key_for_topic_entity_override_is_a_round_trippable_entity_path() {
        // The override is a genuine ENTITY-PATH override — which every
        // doc for it already claims. Fed through the
        // last-segment key path instead, `world/cam` would land on
        // `world/odom/base/world_cam`: not round-trippable through its own
        // output, i.e. broken exactly where a caller reaches for it.
        assert_eq!(
            route_for_input(&route_key_for_topic("/some/blah", Some("world/cam"))).entity,
            "world/cam",
            "a path-shaped override IS the entity path"
        );
        // A `world/`-less override means the same entity (one leading `world/`
        // is stripped, so both spellings agree).
        assert_eq!(
            route_for_input(&route_key_for_topic("/some/blah", Some("cam"))).entity,
            "world/cam"
        );
        // THE ROUND TRIP: feeding the daemon's OWN reported entity back as an
        // override is a fixed point (the property the docs promise).
        for topic in ["/utlidar/cloud", "/api/vui/response", "/lf/sportmodestate"] {
            let reported = route_for_input(&route_key_for_topic(topic, None)).entity;
            let refed = route_for_input(&route_key_for_topic(topic, Some(&reported))).entity;
            assert_eq!(refed, reported, "{topic}: reported entity must round-trip");
        }
        // A deep override keeps every segment (sanitized), and the strip applies
        // to at most ONE leading `world`.
        assert_eq!(
            route_for_input(&route_key_for_topic(
                "/x/y",
                Some("/world/odom/base/wrist/")
            ))
            .entity,
            "world/odom/base/wrist"
        );
        assert_eq!(
            route_for_input(&route_key_for_topic("/x/y", Some("world/world/w"))).entity,
            "world/world/w"
        );
        // The strip is override-ONLY: a real `/world/...` TOPIC keeps its own
        // `world` segment, so it can never alias onto a shorter topic.
        assert_eq!(
            route_for_input(&route_key_for_topic("/world/model/pose", None)).entity,
            "world/world/model/pose"
        );
        assert_ne!(
            route_for_input(&route_key_for_topic("/world/model/pose", None)).entity,
            route_for_input(&route_key_for_topic("/model/pose", None)).entity,
        );
    }

    #[test]
    fn route_key_for_topic_degenerate_inputs_are_total() {
        // Degenerate inputs never panic: they collapse to an empty key.
        assert_eq!(route_key_for_topic("/", None), "");
        assert_eq!(route_key_for_topic("", None), "");
        assert_eq!(route_key_for_topic("//", None), "");
        // A trailing slash is trimmed.
        assert_eq!(route_key_for_topic("/robot/odom/", None), "robot/odom");
        // An empty key still routes — and NEVER claims the TF tree's root
        // entity (it takes the sanitizer's `unknown` fallback one level down).
        let degenerate = route_for_input(&route_key_for_topic("/", None)).entity;
        assert_eq!(degenerate, "world/unknown_2325");
        assert_ne!(degenerate, WORLD_ROOT);
        // An override that normalizes to NOTHING (a bare `world` — the viz root,
        // which no topic may claim, and exactly what the tf arms report) is
        // treated as ABSENT: the topic keeps its own answer, so re-feeding a
        // reported entity is a fixed point instead of a nonsense entity.
        assert_eq!(route_key_for_topic("/x", Some("world")), "x");
        assert_eq!(route_key_for_topic("/x", Some("/")), "x");
        assert_eq!(
            route_for_input(&route_key_for_topic("/x", Some("world"))).entity,
            "world/x"
        );
        // Degenerate topic AND a nothing-override: still total, still not the root.
        assert_eq!(
            route_for_input(&route_key_for_topic("/", Some("world"))).entity,
            "world/unknown_2325"
        );
    }

    #[test]
    fn an_entity_override_names_an_entity_and_never_flips_a_knob() {
        // The route key carries the TOPIC (which alone decides
        // the tf/odom knobs) and the normalized override (which alone decides the
        // entity). If the key WERE the override, an override whose last
        // segment happened to be a knob name would silently re-route an unrelated
        // topic — `entity: "world/tf_static"` would put a camera on the static-TF arm,
        // and `entity: "world/tf"` would move its geometry to the scene ROOT.
        let camera = route_for_input(&route_key_for_topic(
            "/front/camera",
            Some("world/tf_static"),
        ));
        assert_eq!(
            camera.entity, "world/tf_static",
            "the override IS the entity"
        );
        assert!(
            !camera.is_static,
            "an override must not flip the static knob"
        );
        assert!(!camera.drives_robot_root);
        let plain = route_for_input(&route_key_for_topic("/front/camera", Some("world/tf")));
        assert_eq!(plain.entity, "world/tf");
        assert_ne!(
            plain.entity, WORLD_ROOT,
            "an override can never claim the viz root"
        );
        assert!(!plain.drives_robot_root);
        assert!(
            !route_for_input(&route_key_for_topic("/front/camera", Some("world/odom")))
                .drives_robot_root,
            "an override must not elect the robot root"
        );
        // …and conversely the knobs SURVIVE an override: a real /tf_static keeps
        // logging static no matter what entity the caller names.
        let tfs = route_for_input(&route_key_for_topic("/tf_static", Some("world/anywhere")));
        assert!(tfs.is_static, "the topic decides the static knob");
        let od = route_for_input(&route_key_for_topic("/utlidar/robot_odom", Some("world/o")));
        assert!(
            od.drives_robot_root,
            "the topic decides the robot-root election"
        );
        assert_eq!(od.entity, "world/o");
        // The separator is an implementation detail of the key; `route_key_topic`
        // recovers the human-facing half.
        let key = route_key_for_topic("/front/camera", Some("world/cam"));
        assert_eq!(route_key_topic(&key), "front/camera");
        assert_eq!(route_key_topic("front/camera"), "front/camera");
    }

    #[test]
    fn the_reported_entity_is_a_fixed_point_for_every_topic_including_tf() {
        // THE round trip, over a corpus that INCLUDES `/tf` and `/tf_static` —
        // the only two topics whose
        // reported entity is not their mechanical path. Feeding a reported
        // entity back must be a no-op on the ENTITY and on both KNOBS.
        for topic in [
            "/utlidar/cloud",
            "/api/vui/response",
            "/lf/sportmodestate",
            "/tf",
            "/tf_static",
            "/uslam/frontend/odom",
            "/",
            "/wrist cam!",
        ] {
            let first = route_for_input(&route_key_for_topic(topic, None));
            let refed = route_for_input(&route_key_for_topic(topic, Some(&first.entity)));
            assert_eq!(
                refed, first,
                "{topic}: re-feeding the reported entity must be a no-op"
            );
        }
    }

    #[test]
    fn route_for_input_builds_the_mechanical_entity_and_keeps_two_knobs() {
        // The entity is `world/` + sanitized segments. The media table
        // is GONE — `cloud` is an ordinary name now.
        assert_eq!(
            route_for_input("cloud"),
            InputRoute {
                entity: "world/cloud".to_string(),
                is_static: false,
                drives_robot_root: false,
                // No CONFIGURED frame — the frame is resolved from the
                // message's own `frame_id` at render time.
                frame: None,
            }
        );
        // Leading '/' stripped before building (absolute wiring names).
        assert_eq!(route_for_input("/cloud").entity, "world/cloud");
        assert_eq!(route_for_input("image").entity, "world/image");
        assert_eq!(route_for_input("jpeg").entity, "world/jpeg");
        // A multi-segment name becomes a multi-segment entity.
        assert_eq!(
            route_for_input("utlidar/cloud_deskewed").entity,
            "world/utlidar/cloud_deskewed"
        );
        // tf vs tf_static: BOTH report the viz root, differing only in static.
        assert!(!route_for_input("tf").is_static);
        assert!(route_for_input("tf_static").is_static);
        assert_eq!(route_for_input("tf").entity, WORLD_ROOT);
        assert_eq!(route_for_input("tf_static").entity, WORLD_ROOT);
        // A hostile name goes through the ONE crate sanitizer (so it carries the
        // aliasing suffix — `world/wrist_cam_` alone could collide).
        let other = route_for_input("wrist cam!");
        assert_eq!(other.entity, "world/wrist_cam__38ca");
        assert!(!other.is_static);
        // No media / tf input poses the robot root.
        for name in ["cloud", "lidar", "image", "camera", "tf", "tf_static"] {
            assert!(
                !route_for_input(name).drives_robot_root,
                "{name} must not drive the robot root"
            );
        }
    }

    #[test]
    fn route_for_input_knobs_key_on_the_last_segment_case_insensitively() {
        // The one-casing rule — now applied to the name's LAST SEGMENT, which
        // is what keeps the knobs working on the daemon path where the key is a
        // whole topic.
        let tfs = route_for_input("TF_STATIC");
        assert_eq!(tfs.entity, WORLD_ROOT);
        assert!(tfs.is_static, "TF_STATIC selects the static TF arm");
        assert!(!route_for_input("TF").is_static);
        // A NAMESPACED tf_static topic still logs static (the earlier code
        // matched the whole name, so a full-topic key would have lost this).
        assert!(route_for_input("robot1/tf_static").is_static);
        assert!(!route_for_input("robot1/tf").is_static);
        assert_eq!(route_for_input("robot1/tf").entity, WORLD_ROOT);
        // The odom election is case-insensitive and last-segment keyed.
        assert!(route_for_input("ODOM").drives_robot_root);
        assert!(route_for_input("utlidar/robot_odom").drives_robot_root);
        // Entity segments preserve the ORIGINAL case (matching is uniform).
        let mixed = route_for_input("WristCam");
        assert_eq!(mixed.entity, "world/WristCam");
        assert!(!mixed.drives_robot_root);
    }

    #[test]
    fn odom_named_inputs_drive_robot_root_keeping_their_own_entity() {
        // `odom` / `robot_odom` / `odometry` (case-insensitive, last
        // segment) elect to pose `world/robot`, and keep EXACTLY their own
        // mechanical entity with the original case preserved.
        for name in ["odom", "Odometry", "ROBOT_ODOM", "/odom", "robot_odom"] {
            let r = route_for_input(name);
            assert!(r.drives_robot_root, "{name} must drive the robot root");
            assert!(!r.is_static, "{name} is temporal");
            let leaf = name.trim_start_matches('/');
            assert_eq!(
                r.entity,
                format!("world/{leaf}"),
                "{name} keeps its own entity with original case"
            );
        }
        // The two `/uslam/*/odom` topics that COLLIDED earlier (both onto
        // `world/odom/base/odom`, each overwriting the other's Transform3D — a
        // genuinely wrong spatial answer) now get DISTINCT entities. That is the
        // property this arm pins: the election knob survives the whole-topic key
        // WITHOUT the two topics sharing a path.
        //
        // They do both still ELECT, and `world/tf-tree/robot` is one entity, so
        // attaching both still makes the skeleton snap between two localization
        // estimates. That residual is NOT declared correct here — it is pre-existing
        // (the earlier last-segment key elected both too), arbitrating it
        // would mean guessing which estimate the operator trusts, and it is now
        // LOUD: the render arm warns once, naming every elector, pinned by
        // `coordinate_frame_test::a_second_odom_topic_electing_the_robot_root_warns_exactly_once`.
        let front = route_for_input(&route_key_for_topic("/uslam/frontend/odom", None));
        let local = route_for_input(&route_key_for_topic("/uslam/localization/odom", None));
        assert_eq!(front.entity, "world/uslam/frontend/odom");
        assert_eq!(local.entity, "world/uslam/localization/odom");
        assert_ne!(front.entity, local.entity);
        assert!(front.drives_robot_root && local.drives_robot_root);
        // Non-odom names do NOT elect (exact-word match, not a substring).
        for name in [
            "cloud",
            "tf",
            "pose_est",
            "odometry_raw",
            "base_odom",
            "wheel",
        ] {
            assert!(
                !route_for_input(name).drives_robot_root,
                "{name} must not drive the robot root"
            );
        }
    }

    #[test]
    fn unknown_schema_log_fires_once_per_distinct_schema() {
        let mut log = UnknownSchemaLog::new();
        assert!(log.observe("nav_msgs/Odometry"), "first sighting logs");
        assert!(!log.observe("nav_msgs/Odometry"), "repeat is silent");
        assert!(log.observe("std_msgs/String"), "a different schema logs");
        assert!(!log.observe("std_msgs/String"));
    }

    #[test]
    fn sanitize_segment_is_the_one_crate_sanitizer() {
        // This file no longer carries a private copy; this is
        // `crate::tf::sanitize_segment`, whose full contract (incl. the aliasing
        // suffix) is oracle-pinned there. Spot-check that the import is real:
        // the suffix is the discriminator a local copy would NOT produce.
        assert_eq!(sanitize_segment("imu_link"), "imu_link");
        assert_eq!(sanitize_segment("a/b.c"), "a_b_c_78ba");
        assert_eq!(sanitize_segment("///"), "____3e64");
    }
}
